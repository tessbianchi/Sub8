#include <sub8_slam/slam.h>


namespace slam{
    vector<slam::PtamPoint> ptam_points;
    vector<slam::PtamFrame> ptam_frames;

    PtamMap::PtamMap(){
    }
    void PtamMap::addKeyFrame(Frame frame){

    }










    PtamPoint::PtamPoint(const PtamFrame& frame, int source_level, const Point3& world_coords, const Point2& image_coords ){
        this->source_frame = frame;
        this->source_level = source_level;
        this->world_coords = world_coords;
        this->image_coords = image_coords;
    }

    cv::Mat PtamPoint::getPatch(){
        double x = this->image_coords.x;
        double y = this->image_coords.y;
        int upperHeight = 4;//closer to 0
        int lowerHeight = 4;
        int leftWidth = 4;
        int rightWidth = 4;
        if(y - 4 < 0){
            upperHeight = y;

        }else if(y + 4 > source_frame.image.rows){
            lowerHeight = source_frame.image.rows - y;

        }
        if(x - 4 < 0){
            leftWidth = x;

        }else if(y + 4 > source_frame.image.cols){
            rightWidth = source_frame.image.cols - x;
        }

        double cornerX = x - leftWidth;
        double cornerY = y - upperHeight;

        double lengthX = leftWidth + rightWidth;
        double lengthY = upperHeight + lowerHeight;

        cv::Rect patch = cv::Rect(cornerX, cornerY, lengthX, lengthY);
        return cv::Mat(source_frame.image,patch);
    }

    void getOpticalFlowPoint(const cv::Mat& img, const Point2& warpedCoords){

    }

    PtamFrame::PtamFrame(cv::Mat& image, Pose& pose){
        this->image = image;
        this->camera_pose = pose;

    }

    PtamFrame::PtamFrame(){}

}