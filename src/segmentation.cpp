#include <opencv2/opencv.hpp>
#include <iostream>

#include "segmentation.h"

using namespace cv;

namespace segmentation{

void segmentDepthImage(const Mat& src, 
  std::vector<std::pair<int, int>>& obstacles){

    // Bag file testing

    // Mat new_src = imread("/home/bhairav/Documents/Projects/PCL_Kinect/data/clear.png");
    // Mat obs = imread("/home/bhairav/Documents/Projects/PCL_Kinect/data/obstacle.png");
    // absdiff(new_src, obs, new_src);
    // cvtColor(new_src, new_src, CV_RGB2GRAY);

    // End Bag file testing

    Mat new_src = src.clone();
  
    imshow("original2", new_src);

    // Create binary image from source image
    Mat bw = new_src.clone();
    Mat bw2 = new_src.clone();

    inRange(bw, Scalar(10), Scalar(160), bw);

    imshow("BeforeW", bw2);

    for  (int i = 0; i < bw.rows; ++i){
        for (int j = 0; j < bw.cols; ++j){
            if (bw.at<uint8_t>(i, j) == 0){
                bw2.at<uint8_t>(i,j) = 0;
            }
        }
    }

    imshow("Binary Image", bw2);

    // Create a kernel that we will use for accuting/sharpening our image
    Mat kernel = (Mat_<float>(3,3) <<
            1,  1, 1,
            1, -8, 1,
            1,  1, 1); 

    Mat imgLaplacian;
    Mat sharp = bw2; // copy source image to another temporary one
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    bw2.convertTo(sharp, CV_32F);
    Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    // imshow( "Laplace Filtered Image", imgLaplacian );
    imshow( "New Sharped Image", imgResult );

    threshold(imgResult, imgResult, 15, 255, CV_THRESH_BINARY);


    // Plain Contours

    // Dilate a bit the dist image
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
    dilate(imgResult, imgResult, kernel1);

    // Find total markers
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(imgResult, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<Point> > contours_poly( contours.size() );
    std::vector<Rect> boundRect( contours.size() );
    std::vector<Point2f>center( contours.size() );
    std::vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
     { 
       approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
     }


    /// Draw polygonal contour + bonding rects + circles
    Mat drawing = Mat::zeros( imgResult.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
     {
       if (boundRect[i].area() < 400 || boundRect[i].area() > 200*200) continue;
       Scalar color = Scalar( 255, 0, 0 );
       drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<Vec4i>(), 0, Point() );
       rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
     }

     imshow( "Contours", drawing );

    // Blob Detection

    // cv::SimpleBlobDetector::Params params; 

    // params.filterByArea = true;         // filter my blobs by area of blob
    // params.minArea = 100.0;              // min 10 pixels each side
    // Ptr<SimpleBlobDetector> blob_detect = SimpleBlobDetector::create(params);
    // std::vector<KeyPoint> myBlobs;
    // blob_detect->detect(imgResult, myBlobs);


    // std::cout << myBlobs.size() << std::endl;

    // drawKeypoints(imgResult, myBlobs, imgResult);
    // imshow("Blobs", imgResult);

     // End Blob Detection


     // TODO: Bounding boxes into Obstacle objects
     // Add back bg to get actual depth


    waitKey(0);
}

}; // namespace Segmentation

