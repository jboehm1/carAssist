//
//  3dAnalysis.cpp
//  CarAssist
//
//  Created by Jean B on 04.06.24.
//

#include "stereo.hpp"
#include <iostream>

// Define the calibration parameters
double Stereo::fx = 0.0;
double Stereo::fy = 0.0;
double Stereo::cx = 0.0;
double Stereo::cy = 0.0;
double Stereo::baseline = 0.0;

void Stereo::setCalibrationParameters(double fx_, double fy_, double cx_, double cy_, double baseline_) {
    fx = fx_;
    fy = fy_;
    cx = cx_;
    cy = cy_;
    baseline = baseline_;
}

void Stereo::loadImage(std::string_view& path,std::vector<cv::String>& leftImages,std::vector<cv::String>& rightImages){
    cv::glob(std::string(path.data())+"/image_0/*.png", leftImages, false);
    cv::glob(std::string(path.data())+"/image_1/*.png", rightImages, false);
    if (leftImages.size() != rightImages.size()) {
        std::cerr << "Error: Number of left and right images do not match." << std::endl;
        //return -1;
    }
}
void computeDepthMap(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity, int numDisparities){
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(numDisparities, 9); //16 9
    stereo->compute(leftImage, rightImage, disparity);
}
void computeDepthMapAdvanced(const cv::Mat& left, const cv::Mat& right, cv::Mat& disparity, int numDisparities) {
    // Preprocessing steps
    cv::Mat left_gray, right_gray;
    if (left.channels() == 3) {
        cv::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY);
    } else {
        left_gray = left.clone();
    }
    if (right.channels() == 3) {
        cv::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY);
    } else {
        right_gray = right.clone();
    }

    // Apply GaussianBlur to reduce noise
    cv::GaussianBlur(left_gray, left_gray, cv::Size(5, 5), 0);
    cv::GaussianBlur(right_gray, right_gray, cv::Size(5, 5), 0);

    // StereoSGBM parameters
    int blockSize = 9;
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(0, numDisparities, blockSize);

    stereo->setPreFilterCap(63);
    stereo->setBlockSize(blockSize);
    stereo->setP1(8 * left_gray.channels() * blockSize * blockSize);
    stereo->setP2(32 * left_gray.channels() * blockSize * blockSize);
    stereo->setMinDisparity(0);
    stereo->setNumDisparities(numDisparities);
    stereo->setUniquenessRatio(10);
    stereo->setSpeckleWindowSize(100);
    stereo->setSpeckleRange(32);
    stereo->setDisp12MaxDiff(1);
    stereo->setMode(cv::StereoSGBM::MODE_SGBM);

    // Compute the disparity map
    //cv::Mat disparity16S;
    stereo->compute(left_gray, right_gray, disparity);

    
}
void Stereo::computeDisparity(const cv::Mat& leftImage, const cv::Mat& rightImage, cv::Mat& disparity){
    
    cv::Mat left_gray, right_gray;
    if (leftImage.channels() == 3) {
        cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2GRAY);
    }
    if (rightImage.channels() == 3) {
        cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2GRAY);
    }
//   cv::GaussianBlur(leftImage, leftImage, cv::Size(5, 5), 0);
//    cv::GaussianBlur(rightImage, rightImage, cv::Size(5, 5), 0);
    int numDisparities(64);
    computeDepthMapAdvanced(leftImage, rightImage, disparity, numDisparities);
    
    cv::Mat disp8;
    disparity.convertTo(disp8, CV_8U, 255/(numDisparities*16.0));
    disparity=disp8;
    // Apply a color map to the disparity map
    cv::applyColorMap(disparity, disparity, cv::COLORMAP_JET);
    // Post-processing: Apply a median filter to the disparity map
    cv::medianBlur(disparity, disparity, 3);
    
    //cv::imshow("Left Image", leftImage);
    //cv::imshow("Right Image", rightImage);
    //cv::imshow("Disparity Map", disp8);
    //cv::waitKey(1);
}
void Stereo::preprocessDisp(const cv::Mat& leftImage, const cv::Mat& rightImage, const cv::Mat& disparity, cv::Mat& finalDisplay){
    
    cv::Mat left_resized = leftImage;
    cv::Mat right_resized = rightImage;
    // Resize left and right images to fit the same width as disparity
    cv::Size size(disparity.cols / 2, disparity.rows/2);
    cv::resize(leftImage, left_resized, size);
    cv::resize(rightImage, right_resized, size);

    // Concatenate left and right images horizontally
    cv::Mat combinedLR;
    cv::hconcat(right_resized, right_resized, combinedLR);
    cv::resize(combinedLR, combinedLR, cv::Size(disparity.cols, disparity.rows/2));
    // Concatenate combined left-right image with disparity map vertically
    cv::vconcat(combinedLR, disparity, finalDisplay);
    
}

void Stereo::depthMap(const cv::Mat& disparity){
    
    // Convert disparity to depth
    cv::Mat depthMap(disparity.size(), CV_64F);
    for (int y = 0; y < disparity.rows; y++) {
        for (int x = 0; x < disparity.cols; x++) {
            double disparityValue = disparity.at<uchar>(y, x);
            if (disparityValue > 0) {
                depthMap.at<double>(y, x) = (fx * baseline) / disparityValue;
            } else {
                depthMap.at<double>(y, x) = 0;
            }
        }
    }
}
bool Stereo::depthRegion(const cv::Mat& disparity, cv::Rect& roi, double meanValue){
    
    // Convert a region to a depth map and compute a mean value among non zero values
    cv::Mat depthMap(roi.size(), CV_64F);
    double sum=0.0;
    int count=0;
    double mem=0;
    for (int y = roi.y; y < roi.y + roi.height; y++) {
        for (int x = roi.x ; x < roi.x + roi.width; x++) {
            double disparityValue = disparity.at<uchar>(y, x);
            if (disparityValue > 0) {
                double depthValue = (fx * baseline) / disparityValue;
                sum+=depthValue;
                std::cout << depthValue << std::endl;

                if (depthValue>mem)
                    mem = depthValue;
                count++;
                depthMap.at<double>(y, x) = depthValue;
            } else {
                depthMap.at<double>(y, x) = 0;
            }
        }
    }
    
    meanValue=sum/(roi.width*roi.height);//count;
    std::cout << mem << std::endl;
    return count>roi.width*roi.height/2;
}
double Stereo::depthPoint(const cv::Mat& disparity, const cv::Point& point){
    double result3dPoint;
    double disparityValue = disparity.at<uchar>(point.y, point.x);
    if (disparityValue > 0) {
        result3dPoint = (fx * baseline) / disparityValue;
    } else {
        result3dPoint = 0;
    }
    return result3dPoint;
}
