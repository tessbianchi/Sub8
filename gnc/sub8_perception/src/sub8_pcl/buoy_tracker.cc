/*
TODO
--> [X] Make it so a circle is drawn around the bouy the whole time, and nothing else.
--> [X] Threshold to a certain color
--> [x] Remove background image
--> [x] Organize code
--> [X] Get visual data from Ralph
--> [X] Have certain fraction of frame mean that the sub has ided the buoy
    --> [X] Optical Flow
    --> [X] Fix the threshold/calibration problem
    --> [] Make sure the last one isn't an oulier
    --> [] Make sure it still works when the buoy is out of view.
--> [] Check out the rest of the tasks with the buoy
--> [] Put it in the rest of the code and somehow publish it to ros
*/



#include <sub8_pcl/buoy_tracker_tools.hpp>

namespace sub {

void BuoyTracker::remove_background(const cv::Mat input, cv::Mat& output){
    pMOG->operator()(input, output);
}

void BuoyTracker::get_colored_object(const cv::Mat input, cv::Mat& output){
    //output = input;
    cv::cvtColor(input, output, CV_RGB2HSV);
    //cv::inRange(output,cv::Scalar(23,41,133),cv::Scalar(40,150,255),output);
    cv::inRange(output, lowerColor, upperColor, output);
    cv::bitwise_and(input, input, output, output);

}

void BuoyTracker::get_circles(const cv::Mat input, std::vector<cv::Vec3f>& circles){
    cv::Mat temp;
    cv::cvtColor(input, temp, CV_BGR2GRAY );
    cv::GaussianBlur(temp, temp, cv::Size(9, 9), 2, 2 );
    cv::HoughCircles( temp, circles, CV_HOUGH_GRADIENT, 1, temp.rows/8, 200, 25, 0, 0 );
}

void BuoyTracker::draw_circle(const cv::Vec3f circle, const cv::Mat input, cv::Mat output, cv::Scalar color){
    output = input;
    cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
    int radius = cvRound(circle[2]);
    cv::circle( output, center, 3, color, -1, 8, 0 );
    cv::circle( output, center, radius, color, 3, 8, 0 );

}

BuoyTracker::BuoyTracker(){

}

BuoyTracker::BuoyTracker(int color){
        if(color == 1){
            lowerColor = cv::Scalar(15, 100, 100);
            upperColor = cv::Scalar(30, 255, 255);

        }else if(color == 2){
            lowerColor = cv::Scalar(105, 135, 135);
            upperColor = cv::Scalar(120, 255, 255);

        }

        calibration_count = 0;
        calibration_amount = 20;
        _isCalibrating = true;
        pMOG = new cv::BackgroundSubtractorMOG();
        hasBuoy = false;

        win_size = cv::Size(7, 7);
        max_level = 2;
        criteria = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 20, 0.03);

}

void BuoyTracker::calibrateTracker(const cv::Mat frame){

    if(frame.rows == 0){
        std::cout<< "You passed an invalid image"<<std::endl;
        return;
    }
    cv::Mat output;

    std::vector<cv::Vec3f> curr_circles;
    get_colored_object(frame, output);

   if(_isCalibrating){
        get_circles(output, curr_circles);
        for(int i = 0; i != curr_circles.size(); ++i){
            cv::Vec3f c = curr_circles[i];
            bool added = false;
            for(int j = 0; j!= my_buckets.size(); ++j){
                BuoyBucket b = my_buckets[j];
                if(c[2] < b.upperBound && c[2] > b.lowerBound){
                    // cv::Point circ(c[0], c[1]);
                    // cv::Point currbuck(cvRound(my_buckets[j].last_circle[0]), cvRound(my_buckets[j].last_circle[1]));
                    // cv::Point diff = circ - currbuck;
                    // double n = cv::norm(diff);
                    // int la = fn - my_buckets[j].frame_number;
                    added = true;
                    my_buckets[j].last_circle = c;
                    my_buckets[j].last_image = frame.clone();
                    my_buckets[j].count++;
                    my_buckets[j].frame_number = calibration_count;
                    my_buckets[j].count_since_last = 0;
                    break;
                }
            }
            if(my_buckets.empty() || !added){
                BuoyBucket buck = BuoyBucket();
                buck.last_circle = c;
                buck.frame_number = calibration_count;
                buck.upperBound = c[2] + 10;
                buck.lowerBound = c[2] - 10;
                buck.count_since_last = 0;
                buck.last_image = frame.clone();
                buck.count = 1;
                buck.color = cv::Scalar(rand()%255,rand()%255, rand()%255);
                my_buckets.push_back(buck);

            }

            // for(int i = 0; i != my_buckets.size(); ++i){
            //     draw_circle(my_buckets[i].last_circle, output, output, my_buckets[i].color);
            // }

            // for(int i = 0; i != curr_circles.size(); ++i){
            //     draw_circle(curr_circles[i], output, output, cv::Scalar(255,0,0));
            // }
        }
        if(calibration_count == calibration_amount - 1){
            _isCalibrating = false;
        }

        cv::imshow("input", output);
        cv::waitKey(50);
    }else{
        std::cout<<"Done calibrating"<<std::endl;
        return;
    }
    ++calibration_count;
}

void BuoyTracker::trackBuoy(cv::Mat frame, cv::Point& center){
    if(frame.rows == 0){
        std::cout<< "You passed an invalid image"<<std::endl;
        return;
    }

    if(!hasBuoy){
        std::sort(my_buckets.begin(), my_buckets.end(), less_than_key());
        cv::Vec3f c = my_buckets[my_buckets.size()-1].last_circle;
        sourceImage = my_buckets[my_buckets.size()-1].last_image.clone();
        std::vector<cv::Vec2f> myPoints;
        cv::Vec2f _v;
        _v[0] = (c[0]);
        _v[1] = (c[1]);
        myPoints.push_back(_v);

        // std::cout<<center_of_circle.size()<<std::endl;
        // cv::imshow("input", frame);
        // cv::waitKey(30)

        //draw_circle(v, output, output, cv::Scalar(0,255,0));
        cv::calcOpticalFlowPyrLK(sourceImage, frame,
                                myPoints,
                                center_of_circle,
                                status,
                                err,
                                win_size,
                                max_level,
                                criteria);

        cv::Point mycenter(center_of_circle[0][0], center_of_circle[0][1]);
        center = mycenter;
        sourceImage = frame.clone();
        hasBuoy = true;


    }else{
         cv::calcOpticalFlowPyrLK(sourceImage, frame,
                                center_of_circle,
                                new_center_of_circle,
                                status,
                                err,
                                win_size,
                                max_level,
                                criteria);
            sourceImage = frame.clone();
            std::cout<<center_of_circle[0]<<std::endl;
            std::cout<<new_center_of_circle[0]<<std::endl;
            center_of_circle = new_center_of_circle;
            cv::Point mycenter(new_center_of_circle[0][0], new_center_of_circle[0][1]);
            center = mycenter;


    }

}

}

