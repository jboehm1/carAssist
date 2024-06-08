#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
//#include "imageProcessing.hpp"

#include "../src/imageProcessing.hpp" // Adjust path if needed
#include "../src/laneDetection.hpp"    // Adjust path if needed
#include "../src/stereo.hpp"           // Adjust path if needed
#include "../src/videoAnalysis.hpp"    // Adjust path if needed
// Test case for initKalmanFilter function
TEST(ImageProcessingTest, InitKalmanFilter) {
    ImageProcessing ip;
    cv::KalmanFilter kf = ip.initKalmanFilter();

    EXPECT_EQ(kf.transitionMatrix.rows, 4);
    EXPECT_EQ(kf.transitionMatrix.cols, 4);
    EXPECT_EQ(kf.measurementMatrix.rows, 2);
    EXPECT_EQ(kf.measurementMatrix.cols, 4);
    EXPECT_EQ(kf.statePost.rows, 4);
    EXPECT_EQ(kf.statePost.cols, 1);
}

// Test case for findIntersectionWithLowerBorder function
TEST(ImageProcessingTest, FindIntersectionWithLowerBorder) {
    cv::Point pt1(0, 0);
    cv::Point pt2(10, 10);
    int imgHeight = 20;

    cv::Point intersection = ImageProcessing::findIntersectionWithLowerBorder(pt1, pt2, imgHeight);

    EXPECT_EQ(intersection.y, imgHeight - 1);
    EXPECT_NEAR(intersection.x, 19, 1);
}

// Test case for loadImage function
TEST(ImageProcessingTest, LoadImage) {
    ImageProcessing ip;
    std::string imagePath = "/Users/jeanb/Documents/weiterbildung/cpp/CarAssist/CarAssist/img/Route-7.jpg";
    
    // Ensure the image path is valid and the image exists
    cv::Mat img = ip.loadImage(imagePath);
    
    EXPECT_FALSE(img.empty());
}

// Test case for disp function
TEST(ImageProcessingTest, Disp) {
    ImageProcessing ip;
    cv::Mat img = cv::Mat::zeros(100, 100, CV_8UC3);
    std::string title = "Test Image";
    int waitMs = 1;
    
    // This test will display an image window, ensure it does not crash
    ip.disp(img, title, waitMs);
}

// Test case for crop function
TEST(ImageProcessingTest, Crop) {
    ImageProcessing ip;
    cv::Mat img = cv::Mat::ones(100, 100, CV_8UC3);
    cv::Mat croppedImg;
    int cropPercentage = 50;
    
    ip.crop(img, croppedImg, cropPercentage);
    
    EXPECT_EQ(croppedImg.rows, img.rows * cropPercentage / 100);
    EXPECT_EQ(croppedImg.cols, img.cols);
    EXPECT_FALSE(croppedImg.empty());
}

// Test case for filter function
TEST(ImageProcessingTest, Filter) {
    ImageProcessing ip;
    cv::Mat img = cv::imread("/Users/jeanb/Documents/weiterbildung/cpp/CarAssist/CarAssist/img/Route-7.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat filteredImg;
    
    cv::KalmanFilter leftKalmanFilter1 = ip.initKalmanFilter();
    cv::KalmanFilter leftKalmanFilter2 = ip.initKalmanFilter();
    cv::KalmanFilter rightKalmanFilter1 = ip.initKalmanFilter();
    cv::KalmanFilter rightKalmanFilter2 = ip.initKalmanFilter();
    
    ip.filter(img, filteredImg, leftKalmanFilter1, leftKalmanFilter2, rightKalmanFilter1, rightKalmanFilter2);
    
    EXPECT_FALSE(filteredImg.empty());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
