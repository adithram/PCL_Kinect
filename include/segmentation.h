#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <opencv2/opencv.hpp>

namespace segmentation{

void segmentDepthImage(const cv::Mat& image, 
  std::vector<std::pair<int, int>>& obstacles);


}; // namespace Segmentation


#endif // SEGMENTATION_H