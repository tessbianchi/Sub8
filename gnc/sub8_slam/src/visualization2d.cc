#include <sub8_slam/slam.h>

namespace slam {

void draw_points(cv::Mat& frame, const PointVector& points, int radius, int thickness) {
  // TODO: Let user pass a color
  for (unsigned int k = 0; k < points.size(); k++) {
    cv::circle(frame, points[k], radius, cv::Scalar(240, 0, 0), thickness);
    // TODO: red rectangle outlines would look cooler
  }
}

void draw_point_ids(cv::Mat& frame, const PointVector& points, const IdVector& point_ids) {
  for (unsigned int k = 0; k < points.size(); k++) {
    cv::putText(frame, std::to_string(point_ids[k]), points[k], cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 250, 0));
  }
}

void draw_reprojection(cv::Mat& frame, const Point3Vector& points3d, const Pose& pose,
                       const cv::Mat& K) {
  cv::Mat points2d_est;
  cv::Mat rotation_vector;

  CvPose q_pose(pose);
  // Eigen::Matrix3f pose_rot;
  // pose_rot = pose.linear()
  // cv::eigen2cv(pose_rot.matrix(), rotation_matrix);
  // cv::Rodrigues(rotation_matrix, rotation_vector);
  cv::Rodrigues(q_pose.rotation, rotation_vector);
  cv::projectPoints(points3d, rotation_vector, q_pose.translation, K, cv::Mat(), points2d_est);

  PointVector points_est = points2d_est;  // casting tho
  for (unsigned int k = 0; k < points_est.size(); k++) {
    cv::circle(frame, points_est[k], 2, cv::Scalar(10, 10, 240), -1);
  }
}
}