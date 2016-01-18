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
  PointVector points_est;
  cv::Mat rotation_vector;

  CvPose cv_pose(pose);
  cv::Rodrigues(cv_pose.rotation, rotation_vector);
  cv::projectPoints(points3d, rotation_vector, cv_pose.translation, K, cv::Mat(), points_est);

  for (unsigned int k = 0; k < points_est.size(); k++) {
    cv::circle(frame, points_est[k], 2, cv::Scalar(10, 10, 240), -1);
  }
}

void draw_frame(cv::Mat& image, const Frame& frame, const cv::Mat& K,
                const Point3Vector& points_3d) {
  // Draw slam:Frame w/ data
  draw_points(image, frame.feature_locations);
  draw_point_ids(image, frame.feature_locations, frame.feature_ids);

  draw_reprojection(image, points_3d, frame.camera_pose, K);
}
}