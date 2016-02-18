#include <sub8_slam/slam.h>
#include <vector>
#include <sophus/se3.hpp>


float _intrinsics[] = {999.56099338, 0.0, 250.45094176, 0.0, 998.88505653,
                       491.09728365, 0.0, 0.0, 1.0};
float _distortion[] = {-1.02803222e-01, 1.87112802e+00, 6.68849551e-03, -1.82291800e-03,
                       -6.61732743e+00};


typedef Sophus::SE3Group<float> Soph;


Soph::Transformation currentMotion;
Soph::Transformation currentPose;
Soph::Tangent currentVelocityVector;



// slam::IdVector prev_feature_ids, new_feature_ids;
// slam::PointVector prev_feature_locations;
// slam::PointVector new_feature_locations;
// std::vector<slam::Frame> frames;
// slam::Point3Vector points;

// // YO VISUALIZE POINTS
// // For debugging
// slam::Point3Vector camera_locations;

// cv::Mat input_frame, new_frame, render_frame;
// cv::Mat prev_frame;
// slam::Pose prev_pose = Eigen::Affine3f::Identity();
// slam::Pose current_pose;

// vector<Point3> points;

vector<slam::PtamPoint> ptam_points;
vector<slam::PtamFrame> ptam_frames;
//slam::Point3Vector points_3d;

cv::Mat intrinsics(3, 3, CV_32F, _intrinsics);
cv::Mat distortion(5, 1, CV_32F, _distortion);

// void printStuff(){
//     for(int i=0; i<points.size(); ++i)
//         std::cout << points[i] << ' ';
//     std::cout<<endl;
//     for(int i=0; i<prev_feature_locations.size(); ++i)
//         std::cout << prev_feature_locations[i] << ' ';

// }




namespace slam {




    PoseTracker::PoseTracker(PtamMap map, RvizVisualizer rviz) { this->map = map; this->rviz = rviz; }

    void PoseTracker::bootstrap(cv::VideoCapture cap) {
        cv::Size refS =
        cv::Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
        cv::namedWindow("input", CV_WINDOW_AUTOSIZE);
        cvMoveWindow("input", 400, 0);
        // prev_pose.rotation = cv::Mat::eye(3, 3, CV_32F);
        // prev_pose.translation = (cv::Mat_<float>(3, 1) << 0.0, 0.0, 0.0);
        bool map_drawn = false;  // Draw the map only once

        int frame_num = 0;
        char key_press = '\n';
        cv::Mat tframe1,tframe2;
        cv::Mat frame1,frame2;
        cap >> tframe1;
        cap >> tframe2;

        slam::IdVector featureIDs1, featureIDs2;
        slam::PointVector featureLocation1;
        slam::PointVector featureLocation2;
        slam::Pose pose1 = Eigen::Affine3f::Identity();
        slam::Pose pose2;

        slam::preprocess(tframe1, frame1, intrinsics, distortion);
        slam::preprocess(tframe2, frame2, intrinsics, distortion);
        ////// Initialization

        slam::initialize(frame1, featureLocation1, featureIDs1);

        slam::StatusVector status;
        slam::optical_flow(frame1, frame2, featureLocation1, featureLocation2,
                               status);

        featureIDs2 = slam::which_points(status, featureIDs1);
        featureLocation2 = slam::filter(status, featureLocation2);
        featureLocation1 = slam::filter(status, featureLocation1);


        slam::StatusVector inliers;
        cv::Mat F;
        F = slam::estimate_fundamental_matrix(featureLocation1, featureLocation2, inliers);
        featureIDs2 = slam::which_points(inliers, featureIDs2);
        // Filter by inlier mask
        featureLocation2 = slam::filter(inliers, featureLocation2);
        featureLocation1 = slam::filter(inliers, featureLocation1);

        ////// Estimate motin from the fundamental matrix
        slam::Pose pose_F;
        pose_F = slam::estimate_motion_fundamental_matrix(featureLocation1, featureLocation2,
                                                          F, intrinsics);

        ////// Triangulate points and dismiss the guess based on magnitude of reprojection error
        slam::Point3Vector points;
        slam::triangulate(pose1, pose_F, intrinsics, featureLocation1, featureLocation2,
                          points);

        //pose2 = slam::estimate_motion_pnp(points, featureLocation2, intrinsics);
        slam::PtamFrame k1 = slam::PtamFrame(frame1, pose1);
        slam::PtamFrame k2 = slam::PtamFrame(frame2, pose_F);

        currentPose = pose_F;

        ptam_frames.push_back(k1);
        ptam_frames.push_back(k2);

        for(int i = 0; i != featureLocation2.size(); ++i){
            slam::PtamPoint p1 = slam::PtamPoint(k1, 0, points[i], featureLocation1[i]);
            slam::PtamPoint p2 = slam::PtamPoint(k2, 0, points[i], featureLocation2[i]);
            ptam_points.push_back(p1);
            ptam_points.push_back(p2);
        }
        //this->points = points;

    }

    void smallPoseEstimate(cv::VideoCapture cap){
        //Todo pick the closest points
        vector<slam::PtamPoint> mypoints;
        int i = ptam_points.size()-1;
        while(i != -1){
            mypoints.push_back(ptam_points[i]);
            i--;
            if(i == ptam_points.size()-51){
                break;
            }
        }

        cv::Mat points2d_est;

        //Check order of rot and trans

        Soph::Tangent t = Soph::vee(currentMotion * currentPose);
        std::cout<<t[0,0]<<std::endl;
        float tran[] = {t[0,0],t[1,0], t[2,0]};
        float ro[] = {t[3,0],t[4,0], t[5,0]};

        cv::Mat trans(3,1, CV_32F, tran);
        cv::Mat rot(3,1, CV_32F, ro);
        // vector<float> rot(ro);


        cv::projectPoints(mypoints, rot, trans, intrinsics, cv::Mat(), points2d_est);

        //Do the affine transform from the points original frame

        // for each ptampoint, call get optical flow point, this will go to the point, get the





        //use optical flow in a wide area to see if they have the right matches
        //make pnp solver to minimize the distance between them
        //

        // cap >> input_frame;
        // slam::PointVector keypoints;
        // cv::FAST(cap, keypoints, 10, true);
        // //reproject the old points onto the image
        // slam::PointVector points_est;
        // slam::reproject_points(points, points_est, current_pose, intrinsics);
        //match get the first 50 from each and match them all

    }

    void processImage(cv::Mat img){
        cv::Mat new_frame0;
        cv::Mat new_frame1;
        cv::Mat new_frame2;
        cv::Mat new_frame3;
        cv::cvtColor(img, new_frame0, CV_RGB2GRAY);
        cv::pyrDown( new_frame0, new_frame1, cv::Size( new_frame0.cols/2, new_frame0.rows/2 ) );
        cv::pyrDown( new_frame1, new_frame2, cv::Size( new_frame1.cols/2, new_frame1.rows/2 ) );
        cv::pyrDown( new_frame2, new_frame3, cv::Size( new_frame2.cols/2, new_frame2.rows/2 ) );

        slam::KeyPointVector keypoints0;
        slam::KeyPointVector keypoints1;
        slam::KeyPointVector keypoints2;
        slam::KeyPointVector keypoints3;
        cv::FAST(new_frame0, keypoints0, 10, true);
        cv::FAST(new_frame1, keypoints1, 10, true);
        cv::FAST(new_frame2, keypoints2, 10, true);
        cv::FAST(new_frame3, keypoints3, 10, true);

        //slam::KeyPointVector points_est;
        //slam::reproject_points(points, points_est, current_pose, intrinsics);

        //each points corresponds to an 8x8 patch

    }


    void updateMotionModel(double deltaT){
        Eigen::Matrix4f prev_prev_pose = ptam_frames[ptam_frames.size() - 2].camera_pose.matrix();
        Eigen::Matrix4f prev_pose = ptam_frames[ptam_frames.size() - 1].camera_pose.matrix();
        Eigen::Matrix4f curr = prev_prev_pose.inverse() * prev_pose;
        Soph::Tangent motionVector = Soph::vee(curr);
        currentVelocityVector = .9 * (currentVelocityVector + motionVector);
        currentMotion = Soph::exp(currentVelocityVector * deltaT).matrix();
    }
}


