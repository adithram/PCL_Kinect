#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <opencv2/opencv.hpp>
#include "Obstacle.h"

namespace segmentation{

void segmentDepthImage(const cv::Mat& image, const cv::Mat& bg_img_,
  std::vector<Obstacle>& obstacles);


}; // namespace Segmentation


#endif // SEGMENTATION_H