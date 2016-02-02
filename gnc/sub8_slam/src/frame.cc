#include <sub8_slam/slam.h>

namespace slam {

Frame::Frame(cv::Mat& image, Pose& pose, IdVector& feature_ids, PointVector& feature_locations) {
  // set_image(image);  // Hmm...
  max_x = image.cols;
  max_y = image.rows;
  set_features(feature_ids, feature_locations);
  set_pose(pose);
  set_image(image);
}

Frame::Frame(Pose& pose, IdVector& feature_ids, PointVector& feature_locations) {
  set_features(feature_ids, feature_locations);
  set_pose(pose);
}

void Frame::set_pose(Pose& pose) { camera_pose = pose; }

void Frame::set_features(IdVector& feature_ids, PointVector& feature_locations) {
  this->feature_ids = feature_ids;
  this->feature_locations = feature_locations;
}

void Frame::set_image(cv::Mat& image) { this->image = image.clone(); }
}