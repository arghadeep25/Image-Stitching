//
// Created by arghadeep on 30.07.24.
//

#ifndef IMAGE_STITCHING_MATCH_FEATURE_HPP_
#define IMAGE_STITCHING_MATCH_FEATURE_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/types.hpp>
#include <vector>

namespace is::feature_match {
/**
 * Feature matching using the Euclidean distance. Lowe's ratio
 * test is used to filter out the false matches.
 */
class MatchFeature {
 public:
  MatchFeature() = default;

 public:
  ~MatchFeature() = default;

 public:
  /**
   * Match the features of the images.
   * @param images_features extracted from keypoint descriptors
   * @param ratio default value is 0.75
   */
  static std::vector<cv::DMatch> match(const types::ImageFeatures &images_features, const float &ratio = 0.75f) {
    if (images_features.size() < 2)
      throw std::invalid_argument("At least two images are required to match features");

    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    std::vector<cv::DMatch> good_matches;

    for (size_t idx = 0; idx < images_features.size() - 1; idx++)
      matcher.knnMatch(images_features[idx].descriptors, images_features[idx + 1].descriptors, knn_matches, 2);

    for (auto &knn_match : knn_matches)
      if (knn_match[0].distance < ratio * knn_match[1].distance)
        good_matches.push_back(knn_match[0]);
    return good_matches;
  }
};
}// namespace is::feature_match
#endif//IMAGE_STITCHING_MATCH_FEATURE_HPP_
