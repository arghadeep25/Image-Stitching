//
// Created by arghadeep on 30.07.24.
//

#ifndef IMAGE_STITCHING_MATCH_IMAGE_HPP_
#define IMAGE_STITCHING_MATCH_IMAGE_HPP_

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/types.hpp>

namespace is::match_image {
/**
 * Class to match images by estimating homography using RANSAC.
 */
class MatchImage {
 public:
  MatchImage() = default;
 public:
  ~MatchImage() = default;

 public:
  static std::vector<cv::DMatch> match(const std::vector<cv::KeyPoint> &kp1,
                    const std::vector<cv::KeyPoint> &kp2,
                    const std::vector<cv::DMatch> &matches,
                    const double &rejection_threshold = 3.0,
                    const int &maxIters = 3000,
                    const double &confidence = 0.995) {
    std::vector<cv::Point2f> points1, points2;
    for (auto matche : matches) {
      points1.push_back(kp1[matche.queryIdx].pt);
      points2.push_back(kp2[matche.trainIdx].pt);
    }
    std::vector<unsigned char> inliers_mask(points1.size());
    cv::Mat
        H = cv::findHomography(points1, points2, cv::RANSAC, rejection_threshold, inliers_mask, maxIters, confidence);
    std::vector<cv::DMatch> inliers;
    for (size_t i = 0; i < inliers_mask.size(); i++) {
      if (inliers_mask[i])
        inliers.push_back(matches[i]);
    }
    return inliers;
  };
};
}
#endif //IMAGE_STITCHING_MATCH_IMAGE_HPP_
