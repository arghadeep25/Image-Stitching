/**
 *
 */

#ifndef IMAGE_STITCHING_TYPES_HPP
#define IMAGE_STITCHING_TYPES_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace is::types {

using Image = cv::Mat;

using MatchedFeatures = std::vector<cv::DMatch>;

using Point = cv::Point2f;

using Points = std::vector<Point>;

using Keypoint = cv::KeyPoint;
using Keypoints = std::vector<Keypoint>;

/**
 * To define the image feature.
 */
typedef struct ImageFeature {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

/**
 * To define the image features.
 */
using ImageFeatures = std::vector<ImageFeature>;

/**
 * To define the batch of images.
 */
using ImageBatch = std::vector<cv::Mat>;

using MatchFeature = std::vector<cv::BFMatcher>;

using MatchFeatures = std::vector<MatchFeature>;

} // namespace image_stitching::is

#endif // IMAGE_STITCHING_TYPES_HPP
