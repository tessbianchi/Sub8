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
// float _intrinsics[] = {999.56099338, 0.0, 250.45094176, 0.0, 998.88505653,
//                        491.09728365, 0.0, 0.0, 1.0};
// float _distortion[] = {-1.02803222e-01, 1.87112802e+00, 6.68849551e-03, -1.82291800e-03,
//                        -6.61732743e+00};

// // slam::Pose currentPose;
// // slam::Post currentMotion;
// // vector<Point3> points;

// cv::Mat intrinsics(3, 3, CV_32F, _intrinsics);
// cv::Mat distortion(5, 1, CV_32F, _distortion);



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
  int frame_num = 0;
  char key_press = '\n';
  bool map_initialized = false;
  slam::PtamMap map = slam::PtamMap();
  slam::PoseTracker poseTracker = slam::PoseTracker(map, rviz);
  poseTracker.bootstrap(cap);



  return 0;
}