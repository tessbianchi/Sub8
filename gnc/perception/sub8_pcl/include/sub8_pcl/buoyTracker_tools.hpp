#include "opencv2/opencv.hpp"
#include <opencv2/video/background_segm.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>

namespace sub{
class BuoyTracker{
    private:
        struct BuoyBucket{
            double lowerBound;
            double upperBound;
            int count;
            int count_since_last;
            cv::Scalar color;
            int frame_number;
            cv::Vec3f last_circle;
            cv::Mat last_image;
        };

        struct less_than_key{
            inline bool operator() (const BuoyBucket& struct1, const BuoyBucket& struct2){
                return (struct1.count < struct2.count);
            }
        };

        int calibration_count;
        int calibration_amount;
        bool _isCalibrating;
        cv::Ptr<cv::BackgroundSubtractor> pMOG;
        std::vector<BuoyBucket> my_buckets;
        bool hasBuoy;
        cv::Mat sourceImage;
        std::vector<cv::Vec2f> center_of_circle;
        std::vector<cv::Vec2f> new_center_of_circle;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::Size win_size;
        int max_level;
        cv::TermCriteria criteria;


        cv::Scalar lowerColor;
        cv::Scalar upperColor;


        void remove_background(const cv::Mat input, cv::Mat& output);
        void get_colored_object(const cv::Mat input, cv::Mat& output);
        void get_circles(const cv::Mat input, std::vector<cv::Vec3f>& circles);
        void draw_circle(const cv::Vec3f circle, const cv::Mat input, cv::Mat output, cv::Scalar color);


    public:
        static const int YELLOW = 1;
        static const int RED = 2;

        BuoyTracker(int color);
        BuoyTracker();
        void calibrateTracker(cv::Mat frame);
        void trackBuoy(cv::Mat frame, cv::Point& center);
        bool isCalibrating() {return _isCalibrating;}

};
} //sub