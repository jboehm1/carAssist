//
//  imagePreprocess.cpp
//  CarAssist
//
//  Created by Jean B on 04.06.24.
//

#include "imageProcessing.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

// Function to initialize the Kalman Filter
cv::KalmanFilter ImageProcessing::initKalmanFilter() {
    cv::KalmanFilter kf(4, 2, 0);
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                                                     0, 1, 0, 1,
                                                     0, 0, 1, 0,
                                                     0, 0, 0, 1);
    kf.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
                                                     0, 1, 0, 0);
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));
    return kf;
}

// Function to find the intersection of a line with the lower border of the image
cv::Point ImageProcessing::findIntersectionWithLowerBorder(const cv::Point& pt1, const cv::Point& pt2, int imgHeight) {
    double slope = (double)(pt2.y - pt1.y) / (pt2.x - pt1.x);
    double intercept = pt1.y - slope * pt1.x;
    int y = imgHeight - 1;
    int x = (y - intercept) / slope;
    return cv::Point(x, y);
}

cv::Mat ImageProcessing::loadImage(std::string_view path) {
    cv::Mat img = cv::imread(path.data(), cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        exit(EXIT_FAILURE);
    }
    return img;
}

void ImageProcessing::disp(const cv::Mat& _img, const std::string& titel, int waitMs) {
    if (_img.empty()) {
        std::cerr << "Input image is empty!" << std::endl;
        return;
    }
    cv::imshow(titel, _img);
    cv::waitKey(waitMs); // Wait indefinitely until a key is pressed
}

void ImageProcessing::crop(const cv::Mat& src, cv::Mat& dst, int cropPercentage) {
    if (src.empty()) {
        std::cerr << "Input image is empty!" << std::endl;
        return;
    }
    if (cropPercentage <= 0 || cropPercentage > 100) {
        std::cerr << "Invalid crop percentage!" << std::endl;
        return;
    }
    int newHeight = src.rows * cropPercentage / 100;
    cv::Rect myROI(0, src.rows - newHeight, src.cols, newHeight);
    dst = src(myROI).clone();
}
void ImageProcessing::accumulateLinePoints(const std::vector<cv::Vec2f>& lines, cv::Mat& cdst, cv::Point& leftSumPt1, cv::Point& leftSumPt2, int& leftCount, cv::Point& rightSumPt1, cv::Point& rightSumPt2, int& rightCount, bool drawAllLines) {
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];

        if (rho < -310 || rho > 310) {
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));

            if (rho < -310) {
                leftSumPt1 += pt1;
                leftSumPt2 += pt2;
                leftCount++;
                if (drawAllLines) cv::line(cdst, pt1, pt2, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
            } else {
                rightSumPt1 += pt1;
                rightSumPt2 += pt2;
                rightCount++;
                if (drawAllLines) cv::line(cdst, pt1, pt2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }
    }
}
void ImageProcessing::applyMask(const cv::Mat& binary, cv::Mat& masked, bool DISP) {
    cv::Mat mask = cv::Mat::zeros(binary.size(), binary.type());
    cv::Point points[1][3] = { {cv::Point(0, binary.rows), cv::Point(binary.cols, binary.rows), cv::Point(binary.cols / 2, 0)} };
    const cv::Point* ppt[1] = { points[0] };
    int npt[] = { 3 };
    cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255, 255, 255), 8);
    binary.copyTo(masked, mask);
    if (DISP) disp(masked, "masked", 1);
}
void ImageProcessing::updateKalmanFilter(cv::KalmanFilter& kalmanFilter, cv::Point& avgPt, const cv::Point& sumPt, int count, bool& detected) {
    if (count > 0) {
        avgPt = cv::Point(sumPt.x / count, sumPt.y / count);

        cv::Mat measurement = (cv::Mat_<float>(2, 1) << avgPt.x, avgPt.y);
        kalmanFilter.correct(measurement);
        cv::Mat prediction = kalmanFilter.predict();
        avgPt = cv::Point(prediction.at<float>(0), prediction.at<float>(1));

        detected = true;
    } else {
        cv::Mat prediction = kalmanFilter.predict();
        avgPt = cv::Point(prediction.at<float>(0), prediction.at<float>(1));
    }
}
void ImageProcessing::filter(const cv::Mat& src, cv::Mat& dst, cv::KalmanFilter& leftKalmanFilter1, cv::KalmanFilter& leftKalmanFilter2, cv::KalmanFilter& rightKalmanFilter1, cv::KalmanFilter& rightKalmanFilter2, bool DISP) {
    if (src.empty()) {
        std::cerr << "Input image is empty!" << std::endl;
        return;
    }
    
   
    
    // Convert to grey
    cv::Mat grey, hsv;
    cv::Mat cdst;
    bool SOEBEL = false;
    bool DISP_HOUGHLINEP = false;
    if(src.channels()==1){
        grey=src.clone();
        cv::cvtColor(src, cdst, cv::COLOR_GRAY2BGR);
    }
    else{
        cv::cvtColor(src, grey, cv::COLOR_BGR2GRAY);
        cdst = src.clone();
    }
    
//    //HSV fitlering
//    cv::equalizeHist(grey, grey);
//    disp(grey, "nromalized", 1);
//    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
//    cv::inRange(hsv, cv::Scalar(0, 0, 100), cv::Scalar(255, 255, 255), grey);
//    disp(grey, "hsv filter ",1);
    
    
    // Gaussian Filtering
    cv::GaussianBlur(grey, grey, cv::Size(3, 3), 0);
    
    
    //otsu
    cv::Mat binary;
    // cv::threshold(grey, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
//    cv::adaptiveThreshold(grey, binary, 125, cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY_INV, 13, 12);
    //disp(binary, "", 1);

    
    //Canny filtering
    cv::Mat bin;
    cv::Canny(grey, binary, 50, 200, 3);
    if (DISP) disp(binary, "Canny", 1);
    
    
//     //Soebel filter
//    cv::Mat grad_x;
//    cv::Sobel(grey, grad_x, CV_16S, 1, 0, 3);//, 3, 1, 1, cv::BORDER_DEFAULT);
//    // converting back to CV_8U
//    convertScaleAbs(grad_x, binary);
//    disp(binary, " and soebel", 1);
//    cv::threshold(binary, binary, 20, 255, 0);
//    disp(binary, "threshed", 1);
    
    
    // Morph filtering
    int morph_elem = 1;
    int morph_size=1;
    cv::Mat element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    //morphologyEx( binary, binary, cv::MORPH_OPEN, element );
    morphologyEx( binary, binary, cv::MORPH_CLOSE, element );
    if(DISP) disp(binary, "morph", 1);
    
    // Create a mask with the same size as the image, initially set to black
    cv::Mat masked;
    applyMask(binary, masked, DISP);
    
    //cv::cvtColor(grey, cdst, cv::COLOR_GRAY2BGR);
    //cdst = src;
    cv::Mat cdst1=cdst.clone();
    
    //Compute hough lines
    std::vector<cv::Vec2f> lines;
    HoughLines(masked, lines, 1, CV_PI / 180, 55, 0, 0);//, -CV_PI/4, CV_PI/4);

    // Variables to accumulate points
    cv::Point leftSumPt1(0, 0), leftSumPt2(0, 0);
    cv::Point rightSumPt1(0, 0), rightSumPt2(0, 0);
    int leftCount = 0, rightCount = 0;
    bool drawAllLines=false;
    
    accumulateLinePoints(lines, cdst, leftSumPt1, leftSumPt2, leftCount, rightSumPt1, rightSumPt2, rightCount, drawAllLines);

    
    
    cv::Point avgLeftPt1, avgLeftPt2, avgRightPt1, avgRightPt2;
    bool leftDetected = false;
    bool rightDetected = false;

    updateKalmanFilter(leftKalmanFilter1, avgLeftPt1, leftSumPt1, leftCount, leftDetected);
    updateKalmanFilter(leftKalmanFilter2, avgLeftPt2, leftSumPt2, leftCount, leftDetected);
    updateKalmanFilter(rightKalmanFilter1, avgRightPt1, rightSumPt1, rightCount, rightDetected);
    updateKalmanFilter(rightKalmanFilter2, avgRightPt2, rightSumPt2, rightCount, rightDetected);

        // Draw lines
        if (leftDetected) {
            cv::line(cdst, avgLeftPt1, avgLeftPt2, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
        } else {
            cv::line(cdst, avgLeftPt1, avgLeftPt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA); // Red to indicate prediction
            cv::putText(cdst, "One lane detected", cv::Point(src.cols/2-100,src.rows/2), 1, 2, cv::Scalar(0, 0, 255), 2);
        }

        if (rightDetected) {
            cv::line(cdst, avgRightPt1, avgRightPt2, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        } else {
            cv::line(cdst, avgRightPt1, avgRightPt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA); // Red to indicate prediction
        }

    // Find the intersection points with the lower border
    cv::Point leftIntersection, rightIntersection;
    if (leftCount > 0) {
        leftIntersection = findIntersectionWithLowerBorder(avgLeftPt1, avgLeftPt2, src.rows);
    }
    if (rightCount > 0) {
        rightIntersection = findIntersectionWithLowerBorder(avgRightPt1, avgRightPt2, src.rows);
    }

    // Calculate the midpoint
    cv::Point midpoint;
    if (leftCount > 0 && rightCount > 0) {
        midpoint = cv::Point((leftIntersection.x + rightIntersection.x) / 2, src.rows - 1);
        cv::Scalar markerColor;
        if (midpoint.x>src.cols/2 - 75 && midpoint.x < src.cols/2+75)
            markerColor = cv::Scalar(0,255,0);
        
        else{
            markerColor=cv::Scalar(0,0,255);
            cv::putText(cdst, "Changing lane", cv::Point(src.cols/2-100,src.rows/2), 1, 2, markerColor, 2);
        }
        cv::drawMarker(cdst, midpoint, markerColor, cv::MARKER_CROSS,40, 3 );
    }
    
    if (DISP_HOUGHLINEP){
        // Probabilistic Line Transform
        std::vector<cv::Vec4i> linesP; // will hold the results of the detection
        HoughLinesP(masked, linesP, 2, CV_PI/180, 50,50,150);//20, 20, 3
        // Draw the lines
        for( size_t i = 0; i < linesP.size(); i++ )
        {
            cv::Vec4i l = linesP[i];
            line( cdst1, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 2, cv::LINE_AA);
        }
        disp(cdst1,"HoughLineP",1);
    }
    
    dst = cdst.clone(); // Update the destination with the final result
}
