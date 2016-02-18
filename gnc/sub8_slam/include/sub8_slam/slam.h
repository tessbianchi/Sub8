#pragma once  // header guard
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <sba/sba.h>
#include <ros/ros.h>
#include <ros/assert.h>

// #define VISUALIZE

namespace slam {

typedef float SlamPrecision;
typedef cv::Point2f Point2;
typedef cv::Point3f Point3;

typedef std::vector<Point2> PointVector;
typedef std::vector<Point3> Point3Vector;
typedef std::vector<cv::KeyPoint> KeyPointVector;
typedef std::vector<cv::Mat> Point4Vector;
typedef std::vector<uchar> StatusVector;
// TODO: int->size_t
typedef std::vector<int> IdVector;

typedef Eigen::Affine3f Pose;

struct CvPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  // TODO: Deprecate this
  cv::Mat rotation;
  cv::Mat translation;

  CvPose(const Pose& pose) {
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    R = pose.linear();
    t = pose.translation();
    cv::eigen2cv(R, rotation);
    cv::eigen2cv(t, translation);
  }

  CvPose() {
    rotation = cv::Mat();
    translation = cv::Mat();
  }

  Pose pose() {
    Eigen::Matrix4f pose_matrix;
    Eigen::Matrix3f eigen_rotation;
    Eigen::Vector3f eigen_translation;

    cv::cv2eigen(rotation, eigen_rotation);
    cv::cv2eigen(translation, eigen_translation);
    pose_matrix << eigen_rotation, eigen_translation, 0.0, 0.0, 0.0, 1.0;
    Eigen::Affine3f eigen_pose;
    eigen_pose = pose_matrix;
    return eigen_pose;
  }
};
// Undistort the image, downsample by half
void preprocess(cv::Mat& input_image, cv::Mat& output_image, const cv::Mat& intrinsics,
                const cv::Mat& distortion);

// Discover the first set of features (Should rename this to discover_features)
void initialize(const cv::Mat& frame, PointVector& corners, IdVector& feature_ids);

PointVector filter(const std::vector<uchar>& status, const PointVector& points);

void optical_flow(const cv::Mat& prev_frame, const cv::Mat& cur_frame, PointVector& prev_pts,
                  PointVector& next_pts, std::vector<uchar>& status);

Point3Vector get_points(const IdVector& keep_ids, const Point3Vector& points);
PointVector get_points(const IdVector& keep_ids, const PointVector& points);
IdVector which_points(const StatusVector& status, const IdVector& previous);

// ******* Motion *******
cv::Mat estimate_fundamental_matrix(const PointVector& pts_1, const PointVector& pts_2,
                                    std::vector<uchar>& inliers);

// Estimate a pose transform given two sets of corresponding features in two images, using a
// fundamental matrix decomposition
Pose estimate_motion_fundamental_matrix(const PointVector& pts_1, const PointVector& pts_2,
                                        const cv::Mat& F, const cv::Mat& K);

// Estimate a pose transform given some manner of 3d "map", with each point in that map
// corresponding to a 2d point (in a new image). PNP stands for "Perspective n-point"
Pose estimate_motion_pnp(const Point3Vector& pts_3d, const PointVector& pts_2d, const cv::Mat& K,
                         Pose& guess_pose);

void triangulate(const Pose& pose_1, const Pose& pose_2, const cv::Mat K, const PointVector& pts_1,
                 const PointVector& pts_2, Point3Vector& triangulated);

double average_reprojection_error(const Point3Vector& points3d, const PointVector& points2d,
                                  const Pose& pose, const cv::Mat& K);

void reproject_points(const Point3Vector& points3d, PointVector& points2d,
                                  const Pose& pose, const cv::Mat& K);

// ******* SBA *******
class Frame {
  // TODO: Keyframe class that carries the actual image
  // Map feature ids
  // Actual image + pyramid
  // 2d feature locations when frame was taken
  // Estimated pose of frame
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  cv::Mat image;
  double max_x = 0;  // Must be set (Even though this is an integer in principle)
  double max_y = 0;  // Must be set (Even though this is an integer in principle)
  IdVector feature_ids;
  PointVector feature_locations;
  Pose camera_pose;
  PointVector reprojection_locations;
  void set_pose(Pose& pose);
  void set_features(IdVector& feature_ids, PointVector& feature_locations);
  void set_image(cv::Mat& image);
  Frame(cv::Mat& image, Pose& pose, IdVector& feature_ids, PointVector& feature_locations);
  Frame(Pose& pose, IdVector& feature_ids, PointVector& feature_locations);
};
typedef std::vector<Frame, Eigen::aligned_allocator<Frame> > FrameVector;

void run_sba(cv::Mat& intrinsics, FrameVector& frames, Point3Vector& map, int iterations=2);

// ******* 2d visualization *******
void draw_points(cv::Mat& frame, const PointVector& points, int radius = 5, int thickness = 1);
void draw_point_ids(cv::Mat& frame, const PointVector& points, const std::vector<int>& point_ids);
void draw_reprojection(cv::Mat& frame, const Point3Vector& points3d, const Pose& pose,
                       const cv::Mat& K);
void draw_frame(cv::Mat& image, const Frame& frame, const cv::Mat& K);

// ******* 3D Visualization *******
class RvizVisualizer {
 public:
  ros::Publisher camera_pub;
  ros::Publisher point_pub;
  ros::NodeHandle nh;
  RvizVisualizer();
  void create_marker(visualization_msgs::Marker& camera_marker,
                     visualization_msgs::Marker& point_marker);
  void draw_points(Point3Vector& points, bool flag);
  void draw_sba(const sba::SysSBA& sba, int decimation, int bicolor);
  void draw_cameras(const FrameVector& frames, int skip_frames = 1);
};


class PtamMap{
  public:
    PtamMap();
    void addKeyFrame(Frame frame);
};

class PtamFrame{
  public:
  cv::Mat image;
  // double max_x = 0;  // Must be set (Even though this is an integer in principle)
  // double max_y = 0;  // Must be set (Even though this is an integer in principle)
  // IdVector feature_ids;
  // PointVector feature_locations;
  Pose camera_pose;
  // void set_pose(Pose& pose);
  // void set_features(IdVector& feature_ids, PointVector& feature_locations);
  //void set_image(cv::Mat& image, Pose& pose);
  PtamFrame(cv::Mat& image, Pose& pose);
  PtamFrame();
  //Frame(cv::Mat& image, Pose& pose, IdVector& feature_ids, PointVector& feature_locations);
  //Frame(Pose& pose, IdVector& feature_ids, PointVector& feature_locations);
};

class PoseTracker{
  public:
    PtamMap map;
    CvPose currentPose;
    CvPose currentMotion;
    RvizVisualizer rviz;
    PoseTracker(PtamMap map, RvizVisualizer rviz);
    void bootstrap(cv::VideoCapture cap);
};

class PtamPoint{
  public:
    PtamFrame source_frame;
    int source_level;
    Point3 world_coords;
    Point2 image_coords;
    PtamPoint(const PtamFrame& frame, int source_level, const Point3& world_coords, const Point2& image_coords);
    cv::Mat getPatch();
};

class PNPSolver{
  public:
    slam::Pose solvePNP(slam::PointVector points2d, Point3Vector points3d, Pose pose_guess, cv::Mat K, int method);
};
}