#include <opencv2/opencv.hpp>
#include <iostream>

#include "segmentation.h"
namespace Segmentation{

void segmentDepthImage(const cv::Mat& image, 
  std::vector<std::pair<int, int>>& obstacles){

    cv::Mat grayscaleMat;

    //TODO: Very inefficient
    grayscaleMat = image.clone();
    
    grayscaleMat.convertTo(grayscaleMat, CV_8U);

    cv::Mat binaryMat(grayscaleMat.size(), grayscaleMat.type());

    cv::threshold(grayscaleMat, binaryMat, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);    

    // Dilate a bit the dist image
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
    dilate(binaryMat, binaryMat, kernel1);

    // Find total markers
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(binaryMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    cv::imshow("binary", binaryMat);

    cv::waitKey(0);
}

}; // namespace Segmentation