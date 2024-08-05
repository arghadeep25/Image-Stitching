//
// Created by arghadeep on 04.08.24.
//

#ifndef IMAGE_STITCHING__FEATURES_HPP_
#define IMAGE_STITCHING__FEATURES_HPP_

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <utils/types.hpp>

namespace is::features {
class Features {
  struct FeaturesParameters {
    bool visualization = false;
    std::string matcher_type = "FlannBasedMatcher";
    double rejection_threshold = 3.0;
    int max_iterations = 3000;
    double confidence = 0.995;
  };

 public:
  Features() = default;

 public:
  ~Features() = default;

 public:
  cv::Mat compute_homography(const cv::Mat &src_img, const cv::Mat &dst_img) {
    if (src_img.empty() || dst_img.empty())
      throw std::runtime_error("Image is empty");

    types::ImageFeature src_features = extract_features(src_img);
    types::ImageFeature dst_features = extract_features(dst_img);

    std::vector<cv::DMatch> matches = match_features(src_features.descriptors,
                                                     dst_features.descriptors);

    std::vector<cv::Point2f> points1, points2;
    for (auto match : matches) {
      points1.push_back(src_features.keypoints[match.queryIdx].pt);
      points2.push_back(dst_features.keypoints[match.trainIdx].pt);
    }

    if (params.visualization)
      this->draw_matches(src_img, src_features, dst_img, dst_features, matches);

    std::vector<uchar> inliers_mask;

    cv::Mat homography = cv::findHomography(points1,
                                            points2,
                                            cv::RANSAC,
                                            params.rejection_threshold,
                                            inliers_mask,
                                            params.max_iterations,
                                            params.confidence);
    std::cout << "Homography matrix: \n" << homography << std::endl;
    return homography;

  }

 private:
  types::ImageFeature extract_features(const cv::Mat &img) {
    cv::Mat img_clone = img.clone();

    if (img.empty())
      throw std::runtime_error("Image is empty");

    if (img.channels() != 1)
      cv::cvtColor(img, img_clone, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    cv::Ptr<cv::Feature2D> sift = cv::SIFT::create();
    sift->detectAndCompute(img_clone, cv::noArray(), keypoints, descriptors);
    return {keypoints, descriptors};
  }

 private:
  std::vector<cv::DMatch> match_features(const cv::Mat &src_features,
                                         const cv::Mat &dst_features) {
    if (src_features.empty() || dst_features.empty())
      throw std::runtime_error("Features are empty");

    std::vector<std::vector<cv::DMatch>> matches;

    if (params.matcher_type == "BFMatcher") {
      cv::BFMatcher matcher(cv::NORM_L2, true);
      matcher.knnMatch(src_features, dst_features, matches, 2);
    } else if (params.matcher_type == "FlannBasedMatcher") {
      cv::FlannBasedMatcher matcher;
      matcher.knnMatch(src_features, dst_features, matches, 2);
    } else {
      throw std::runtime_error("Invalid matcher type");
    }

    std::vector<cv::DMatch> good_matches;
    for (auto &match : matches)
      if (match[0].distance < 0.75 * match[1].distance)
        good_matches.push_back(match[0]);

    return good_matches;
  }

 private:
  void draw_matches(const cv::Mat &src_img,
                    const types::ImageFeature &src_features,
                    const cv::Mat &dst_img,
                    const types::ImageFeature &dst_features,
                    const std::vector<cv::DMatch> &matches) {
    cv::Mat img_matches;
    cv::drawMatches(src_img, src_features.keypoints,
                    dst_img, dst_features.keypoints,
                    matches, img_matches);
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);
  }

 public:
  FeaturesParameters params;
};
} // namespace is::features


#endif //IMAGE_STITCHING__FEATURES_HPP_
