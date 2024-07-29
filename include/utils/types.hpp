//
// Created by arghadeep on 29.07.24.
//

#ifndef IMAGE_STITCHING_TYPES_HPP
#define IMAGE_STITCHING_TYPES_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace image_stitching::is {
typedef struct ImageFeature {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};
} // namespace image_stitching::is

#endif // IMAGE_STITCHING_TYPES_HPP
