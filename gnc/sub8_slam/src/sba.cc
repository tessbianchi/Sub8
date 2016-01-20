/* Sparse Bundle Adjustment for Sub8

--> SBA
  x --> Setup up camera parameters
  x --> Work for at least 5 frames
  --> local, global (Be able to choose keyframes)
  --> Don't repeat lots of work on each bundle adjustment
  x --> Track which features appear in which frames
  x --> Associate a transform with each frame
  x --> Camera pose struct
  x--> Fix first node
  --> Fix "collocation nodes" in global sba
  x --> TODO: clone our own sba source and debug with that
    --> Or just just use libg2o

*/
#include <sba/sba.h>
#include <sub8_slam/slam.h>

#define VISUALIZE
namespace slam {

frame_common::CamParams decompose_cv_intrinsics(cv::Mat& K) {
  // For OpenCV projection matrix
  frame_common::CamParams cam_params;
  cam_params.fx = (double)(K.at<float>(0, 0));
  cam_params.fy = (double)(K.at<float>(1, 1));
  cam_params.cx = (double)(K.at<float>(0, 2));
  cam_params.cy = (double)(K.at<float>(1, 2));
  cam_params.tx = 0;  // No baseline
  return cam_params;
}

void run_sba(cv::Mat& intrinsics, FrameVector& frames, Point3Vector& map) {
#ifdef VISUALIZE
  slam::RvizVisualizer rviz;
#endif
  sba::SysSBA sys;  // TODO: Don't repeat this
  frame_common::CamParams cam_params;
  cam_params = decompose_cv_intrinsics(intrinsics);

  for (size_t frame_num = 0; frame_num < frames.size(); frame_num++) {
    /////// Set up a frame
    Frame frame(frames[frame_num]);
    Pose pose;

    pose = frame.camera_pose;
    Vector4d trans_d;
    trans_d << pose.translation().matrix().cast<double>(), 1.0;

    Matrix3d orientation_d;
    orientation_d = pose.linear().matrix().cast<double>();
    Quaterniond q(orientation_d);

    sys.addNode(trans_d, q, cam_params, frame_num == 0);
    sys.nodes[frame_num].setProjection();
    sys.nodes[frame_num].normRot();
  }
  for (size_t map_pt_num = 0; map_pt_num < map.size(); map_pt_num++) {
    Vector4d point((double)map[map_pt_num].x, (double)map[map_pt_num].y, (double)map[map_pt_num].z,
                   1.0);
    sys.addPoint(point);
  }

  /////// Set up 2d-3d correspondences
  for (size_t frame_num = 0; frame_num < frames.size(); frame_num++) {
    Frame frame(frames[frame_num]);
    for (size_t frame_ftr_num = 0; frame_ftr_num < frame.feature_ids.size(); frame_ftr_num++) {
      Eigen::Vector2d feature_position((double)frame.feature_locations[frame_ftr_num].x,
                                       (double)frame.feature_locations[frame_ftr_num].y);

      int fid = frame.feature_ids[frame_ftr_num];
      sys.addMonoProj(frame_num, fid, feature_position);
    }
  }

#ifdef VISUALIZE
  rviz.draw_sba(sys, 1, 0);
  ros::Duration(0.5).sleep();
#endif

  // SPARSE CHOLESKY, BABY!!
  // TODO: Ceres; Schur-Jacobi mmm
  sys.useCholmod(true);
  sys.doSBA(25, 1e-2, SBA_SPARSE_CHOLESKY);

#ifdef VISUALIZE
  rviz.draw_sba(sys, 1, 1);
  ros::Duration(1.0).sleep();
#endif

  /////// Update frames
  for (size_t frame_num = 0; frame_num < frames.size(); frame_num++) {
    Frame frame(frames[frame_num]);
    Eigen::Affine3f tf;
    tf.matrix().block<3, 3>(0, 0) = sys.nodes[frame_num].qrot.toRotationMatrix().cast<float>();
    tf.matrix().col(3) = sys.nodes[frame_num].trans.cast<float>();
    frame.set_pose(tf);
    for (size_t ftr_id_num = 0; ftr_id_num < frame.feature_ids.size(); ftr_id_num++) {
      Eigen::Vector2d proj;
      sys.nodes[frame_num].project2im(proj, sys.tracks[frame.feature_ids[ftr_id_num]].point);
      frame.reprojection_locations.push_back(Point2((float)proj.x(), (float)proj.y()));
    }
  }
  for (size_t point_num = 0; point_num < map.size(); point_num++) {
    Vector4d pt = sys.tracks[point_num].point;
    Point3 point(pt.x(), pt.y(), pt.z());
    map[point_num] = point;
  }
}
}