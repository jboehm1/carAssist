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
    
};
#endif /* imageProcessing_hpp */
