#include <sub8_slam/slam.h>
namespace slam{
    void find_patch_in_image(const cv::Mat src, const cv::Mat patch, cv::Mat& dest){

        cv::Mat result;
        matchTemplate( src, patch, result, cv::TM_SQDIFF_NORMED );
        normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

        /// Localizing the best match with minMaxLoc
        double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
        cv::Point matchLoc;

        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

        matchLoc = minLoc;

        cv::Rect rect = cv::Rect(matchLoc.x, matchLoc.y, patch.cols, patch.rows);
        dest = cv::Mat(src, rect);


    }

    /*
        Takes the difference of squares between two pictures and gets the result
    */
    float compare_patches(cv::Mat patch1, cv::Mat patch2){
        if(patch1.rows != patch2.rows || patch2.cols != patch2.cols){
            //TODO make this change the size of the ap
            return float(INT_MAX);
        }
        float sum;
        for (int iy = 0; iy <= patch1.rows; iy++){
            uchar * row1 = patch1.ptr<uchar>(iy);
            uchar * row2 = patch2.ptr<uchar>(iy);
            for(int ix = 0; ix != patch1.cols; ++ix){
                float v = row1[ix] - row2[ix];
                sum += v * v;
            }
        }
        return sum;
    }

    void get_most_matching_patch(cv::Mat src, vector<cv::Mat> patches, cv::Mat& dest){
        float min = INT_MAX;
        for(int i = 0; i != patches.size(); ++i){
            float res = compare_patches(src, patches[i]);
            if(res < min){
                dest = src;
                min = res;
            }
        }
    }
}