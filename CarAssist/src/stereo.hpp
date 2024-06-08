//
//  3dAnalysis.hpp
//  CarAssist
//
//  Created by Jean B on 04.06.24.
//

#ifndef stereo_hpp
#define stereo_hpp

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <stdio.h>
#include <iostream>

namespace Stereo{
    extern double fx, fy, cx, cy, baseline;
void setCalibrationParameters(double fx_, double fy_, double cx_, double cy_, double baseline_);
    void computeDisparity(const cv::Mat& a, const cv::Mat& b, cv::Mat& disparity);
    void loadImage(std::string_view& path, std::vector<cv::String>&, std::vector<cv::String>&);
    void preprocessDisp(const cv::Mat& leftImage, const cv::Mat& rightImage, const cv::Mat& disparity, cv::Mat& finalDisplay);
void depthMap(const cv::Mat& );
double depthPoint(const cv::Mat& disparity, const cv::Point& point);
bool depthRegion(const cv::Mat& disparity, cv::Rect& roi, double);

};
#endif /* _dAnalysis_hpp */
