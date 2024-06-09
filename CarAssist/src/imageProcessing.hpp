//
//  imagePreprocess.hpp
//  CarAssist
//
//  Created by Jean B on 04.06.24.
//

#ifndef imageProcessing_hpp
#define imageProcessing_hpp
    
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> //for cvtColor etc.
#include <opencv2/video/tracking.hpp>

namespace ImageProcessing{

    cv::Point findIntersectionWithLowerBorder(const cv::Point& pt1, const cv::Point& pt2, int imgHeight) ;
    cv::KalmanFilter initKalmanFilter();
    cv::Mat loadImage(std::string_view path);
    void disp( const cv::Mat& _img, const std::string& titel="Image", int wait=0);
    void filter(const cv::Mat& src, cv::Mat& dst, cv::KalmanFilter& , cv::KalmanFilter&, cv::KalmanFilter&, cv::KalmanFilter&, bool DISP=true ); // Static method for filtering
    void crop(const cv::Mat& src, cv::Mat& dst, const int cropPercentage);
    void updateKalmanFilter(cv::KalmanFilter& kalmanFilter, cv::Point& avgPt, const cv::Point& sumPt, int count, bool& detected);
    void applyMask(const cv::Mat& binary, cv::Mat& masked, bool DISP);
    void accumulateLinePoints(const std::vector<cv::Vec2f>& lines, cv::Mat& cdst, cv::Point& leftSumPt1, cv::Point& leftSumPt2, int& leftCount, cv::Point& rightSumPt1, cv::Point& rightSumPt2, int& rightCount, bool drawAllLines) ;
};
#endif /* imageProcessing_hpp */
