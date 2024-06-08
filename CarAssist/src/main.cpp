//
//  main.cpp
//  CarAssist
//
//  Created by Jean B on 04.06.24.
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include "imageProcessing.hpp"
#include "stereo.hpp"
#include <vector>

int main(int argc, const char * argv[]) {
    
    bool DISP=false;
    bool use3D=true;
    
    // Initialize Kalman filters for left and right lanes
    cv::KalmanFilter avgLeft1 = ImageProcessing::initKalmanFilter();
    cv::KalmanFilter avgLeft2 = ImageProcessing::initKalmanFilter();
    cv::KalmanFilter avgRight1 = ImageProcessing::initKalmanFilter();
    cv::KalmanFilter avgRight2 = ImageProcessing::initKalmanFilter();
    
    if (!use3D){
        std::string captureSource("sequence");// images, webcam, video, sequences
        std::cout << "Starting!\n";
        std::vector<cv::String> fn;
        if (captureSource=="images")
            cv::glob("/Users/jeanb/Documents/weiterbildung/cpp/CarAssist/CarAssist/img/*.jpg", fn, false);
        else if (captureSource=="sequence")
            cv::glob("/Users/jeanb/Documents/weiterbildung/cpp/CarAssist/CarAssist/img/20/image_0/*.png", fn, false);
        std::vector<cv::Mat> images;
        size_t count = fn.size();
        std::cout << "Found "<< count<< " images." << std::endl;
        
        for (size_t i=0; i<count; i++)
        {
            auto start = std::chrono::high_resolution_clock::now();
            cv::Mat img_orig = ImageProcessing::loadImage(fn[i]);
//            ImageProcessing::disp(img_orig,"",2);
            cv::Mat img_filtered;
            ImageProcessing::crop(img_orig, img_orig,
                                  55);
            ImageProcessing::filter(img_orig, img_filtered, avgLeft1, avgLeft2, avgRight1, avgRight2, DISP);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            // Calculate FPS
                int fps = 1.0 / elapsed.count();
            cv::putText(img_filtered, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
            std::cout << "FPS:"<<fps<<"/";
            ImageProcessing::disp(img_filtered, "lines", 1);
            char c = (char)cv::waitKey(1);
            if (c == 27)
                break;
        }
    }
    else{
        // Set stereo calibration parameters
        Stereo::setCalibrationParameters(7.188560000000e+02, 7.188560000000e+02, 6.071928000000e+02, 1.852157000000e+02, 3.861448000000e+02 / 7.188560000000e+02);

        std::vector<cv::String> leftImages, rightImages;
        std::string_view path="/Users/jeanb/Documents/weiterbildung/cpp/CarAssist/CarAssist/img/20";
        Stereo::loadImage(path, leftImages, rightImages  );
        for ( int i = 0; i<leftImages.size(); i++){
            auto start = std::chrono::high_resolution_clock::now();

            cv::Mat leftImage = cv::imread(leftImages[i], cv::IMREAD_GRAYSCALE);
            cv::Mat rightImage = cv::imread(rightImages[i], 0);
            cv::Mat disparity,finalDisplay;
            
            Stereo::computeDisparity(leftImage, rightImage, disparity);
            
            // Obstacle detection
            int border=50;
            double meanRegionDepth=0.0;
            cv::Rect roi(cv::Point(disparity.cols/2-border, 2*disparity.rows/3), cv::Point(disparity.cols/2+border, 2*disparity.rows/3+border));
            if ( Stereo::depthRegion(disparity, roi, meanRegionDepth) ){
                // Disp a red rect if obstacle detected close
                cv::Scalar rectColor = (meanRegionDepth > 1.4) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
                cv::rectangle(disparity, roi, rectColor, 2);
            }
            else{
                cv::rectangle(disparity, roi, cv::Scalar(0,0,255), 2);

            }
            
            //cv::Mat test_3d;
            //cv::threshold(disparity, test_3d, 254, 255, 0);
            cv::cvtColor(leftImage, leftImage, cv::COLOR_GRAY2BGR);
            cv::cvtColor(rightImage, rightImage, cv::COLOR_GRAY2BGR);
            //cv::cvtColor(disparity, disparity, cv::COLOR_GRAY2BGR);
            
            
            
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            // Calculate FPS
            int fps = 1.0 / elapsed.count();
            cv::putText(disparity, "FPS: " + std::to_string(fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            
            Stereo::preprocessDisp(leftImage,rightImage,disparity,finalDisplay);
            
            cv::imshow("Stereo Images and Disparity", finalDisplay);
            cv::waitKey(1);
            
//            ImageProcessing::crop(leftImage, leftImage,
//                                  55);
//            ImageProcessing::filter(leftImage, leftImage, avgLeft1, avgLeft2, avgRight1, avgRight2);
//            ImageProcessing::disp(leftImage, "lines", 1);
            
        }
        return 0;
    }
}
