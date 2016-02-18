/*

  "Frame" class
    x --> Has features associated with it
    x --> Some kind of graph of features that exist in the next frame
      --> (Perhaps only *detect* new features in new keyframes)
    --> Distinguish between normal frames and keyframes
    --> local sba requires all frames
    --> global SBA requires all frames
    --> global SBA refinement requires only keyframes

  --> Bootstrap
    x --> LK
    x --> Triangulate
    --> SBA

  --> Tracking
    --> Initialize
      x --> Optical Flow
      x --> Detect new points
      x --> Track them & correspondence
      --> Path-reversal testing
      --> Depth filter them
      --> Covariance weighted epipolar search

  --> Estimate Motion
    x --> Estimate fundamental matrix
        x --> Estimate inconsistent motion from essential matrix
    x --> Triangulate points
    --> Improve with SBA
      --> SBA tools
    --> Metric Motion
      x --> PnP solving
      --> Huber distance Lie-Alg optimization?
      --> Sparse model-based image alignment
      --> GN solver
        --> Inverse Compositional solver

    --> Bayes Filter
      --> Discrete/histogram
      --> Parametric mixture-model
      --> OpenCL histogram
      --> Particle filter feasible?
      --> Unscented transform feasible?

    --> Reinitialize when needed
      --> Not restricting ourselves to pure optical flow may...
        - Improve speed (only search on epipolar lines + covariance)
        - Increase robustness to sudden motion
      --> Kalman filter pose estimate (This may not be necessary)

  --> Loop closure
    --> Place recognition
    --> g2o (A little more complicated, but more globally useful than sba.h)
    --> Sparse bundle-adjustment
      x --> Get SBA to build
      x --> Make some crappy SBA classes

  --> Visualization
    x --> Visualize triangulated points
    x --> Consider using RVIZ to visualize
        x --> Make visualizer for our own triangulation
        x --> Make Rviz interface
    --> Visualize depth certainty

  --> SBA
    x --> Get Google Ceres to build
    x --> Create a usable install script
    --> Create Ceres demo
    --> Create Ceres cost functors for SBA
      --> Snavely
    --> Create Ceres SBA helpers (And dispose of the old sba stuff)

  --> Testing
    x --> Make an analysis set
    x --> Record data
    --> Make a two-frame-sandbox executable for testing various things

  --> Bugs
    --> Fix pnp frame (Issue originates with triangulation)
    --> SBA doesn't behave properly

  --> Improvements
    --> SSE on color
    --> Determine when bootstrapping has failed
      (Scale invariant minimum motion requirement)
      (Minimum sum translation?)
      --> Struggle depth certainty is probably an indicator of a need for reinitializing

  --> Experiments
    x --> See if 32F improves speed...it will not affect accuracy

  --> Try OpenCL for...
    --> Optical Flow
    --> Explicit posterior computation
    --> SSE for larger patches (invariance promises weaken past 4x4)
      --> pose to SSE

  x --> Put this in Sub Git
*/

#include <sub8_slam/slam.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ros/ros.h>


// Probably not super smooth to initialize using normal arrays
float _intrinsics[] = {999.56099338, 0.0, 250.45094176, 0.0, 998.88505653,
                       491.09728365, 0.0, 0.0, 1.0};
float _distortion[] = {-1.02803222e-01, 1.87112802e+00, 6.68849551e-03, -1.82291800e-03,
                       -6.61732743e+00};

// // slam::Pose currentPose;
// // slam::Post currentMotion;
// // vector<Point3> points;

cv::Mat intrinsics(3, 3, CV_32F, _intrinsics);
cv::Mat distortion(5, 1, CV_32F, _distortion);



int main(int argc, char **argv) {
  ros::init(argc, argv, "monocular_slam");
  slam::RvizVisualizer rviz;
  if (argc != 2) {
    std::cout << "Give me a video" << std::endl;
    return -1;
  }

  const std::string video_source = argv[1];
  cv::VideoCapture cap(video_source);

  if (!cap.isOpened()) {
    std::cout << "Could not open target video " << video_source << std::endl;
    return -1;
  }
  //int frame_num = 0;
  //char key_press = '\n';
  //bool map_initialized = false;

  //cv::Size refS = cv::Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  cv::namedWindow("input", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("input", 400, 0);

  slam::IdVector prev_feature_ids, new_feature_ids;
  slam::PointVector prev_feature_locations;
  slam::PointVector new_feature_locations;
  std::vector<slam::Frame> frames;
  slam::Point3Vector map;

  // YO VISUALIZE POINTS
  // For debugging
  slam::Point3Vector camera_locations;

  cv::Mat input_frame, new_frame, render_frame;
  cv::Mat prev_frame;
  slam::Pose prev_pose;
  prev_pose = Eigen::Affine3f::Identity();
  double average_reproj= 0.0;
  double tess_av = 0.0;
  double pnp_av = 0.0;
  int count = 0;
  slam::PNPSolver pnp = slam::PNPSolver();
  for (;;) {
  //for(int i = 0; i != 30; ++i){
    ////// Bookkeeping
    cap >> input_frame;
    if (input_frame.empty()) {
      std::cout << "Done";
      break;
    }
    ////// Preprocessing
    slam::preprocess(input_frame, new_frame, intrinsics, distortion);

    // Make a nice new frame, on which to draw colored things
    cv::cvtColor(new_frame, render_frame, CV_GRAY2BGR, 3);

    ////// Initialization
    if (prev_feature_locations.size() == 0) {
      std::cout << "Initializing Frame\n";
      slam::initialize(new_frame, prev_feature_locations, prev_feature_ids);
      // Initialize previous feature ids to a standard range
      prev_frame = new_frame.clone();
      continue;
    }

    ////// Track known points and dismiss outliers
    slam::StatusVector status;
    slam::optical_flow(prev_frame, new_frame, prev_feature_locations, new_feature_locations,
                       status);
    // Determine the IDs of the preserved points
    // TODO: Filter and which_pts in the same function
    new_feature_ids = slam::which_points(status, prev_feature_ids);
    new_feature_locations = slam::filter(status, new_feature_locations);
    prev_feature_locations = slam::filter(status, prev_feature_locations);

    ////// Compute the fundametnal matrix and dismiss outliers
    slam::StatusVector inliers;
    cv::Mat F;
    F = slam::estimate_fundamental_matrix(prev_feature_locations, new_feature_locations, inliers);
    new_feature_ids = slam::which_points(inliers, new_feature_ids);

    // Filter by inlier mask
    new_feature_locations = slam::filter(inliers, new_feature_locations);
    prev_feature_locations = slam::filter(inliers, prev_feature_locations);

    ////// Estimate motion from the fundamental matrix
    slam::Pose pose_F;
    pose_F = slam::estimate_motion_fundamental_matrix(prev_feature_locations, new_feature_locations,
                                                      F, intrinsics);

    ////// Triangulate points and dismiss the guess based on magnitude of reprojection error
    slam::Point3Vector triangulation_F;
    slam::triangulate(prev_pose, pose_F, intrinsics, prev_feature_locations, new_feature_locations,
                      triangulation_F);

    double error_amt = slam::average_reprojection_error(triangulation_F, new_feature_locations,
                                                        pose_F, intrinsics);

    ++count;

    std::cout<<"POSEF ERROR "<<error_amt<<std::endl;

    if (map.size() == 0) {
        map = triangulation_F;
    }

    slam::Point3Vector map_visible;
      // Determine which points in the map are currently visible
    map_visible = slam::get_points(new_feature_ids, map);
    slam::Pose pose_pnp;
    pose_pnp = slam::estimate_motion_pnp(map_visible, new_feature_locations, intrinsics, pose_F);

    //std::cout<<pose_pnp.rotation<<std::endl;
    //std::cout<<pose_pnp.matrix()<<std::endl;

    double pnp_error_amt = slam::average_reprojection_error(map_visible, new_feature_locations,
                                                              pose_pnp, intrinsics);

    std::cout << "PNP Error: " << pnp_error_amt << std::endl;
    //slam::draw_reprojection(render_frame, map_visible, pose_pnp, intrinsics);
    slam::Pose pose_tess;
    pose_tess = pnp.solvePNP(new_feature_locations, map_visible, pose_pnp, intrinsics, 1);
    //std::cout<<pose_tess.matrix()<<std::endl;

    double tess_error_amt = slam::average_reprojection_error(map_visible, new_feature_locations,
                                                              pose_tess, intrinsics);

    average_reproj += error_amt;
    pnp_av += pnp_error_amt;
    tess_av += tess_error_amt;
    std::cout << "MY Error: " << tess_error_amt << std::endl;
    //std::cout<< "my error "<< myer <<std::endl;

  }

  std::cout<<"\nerror ";
  std::cout<<average_reproj/count<<std::endl;
  std::cout<<"\ntesserror ";
  std::cout<<tess_av/count<<std::endl;
  std::cout<<"\npnperror ";
  std::cout<<pnp_av/count<<std::endl;
  return 0;
}