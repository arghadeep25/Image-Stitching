//
// Created by arghadeep on 29.07.24.
//

#ifndef IMAGE_STITCHING_EXTRACT_FEATURE_HPP
#define IMAGE_STITCHING_EXTRACT_FEATURE_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <utils/types.hpp>
#include <vector>

namespace image_stitching::feature_extraction {
class FeatureExtraction {
public:
  FeatureExtraction() = default;

public:
  ~FeatureExtraction() {
    this->m_images_.clear();
    this->features_.clear();
  }

public:
  void load_images(const std::vector<cv::Mat> &images) {
    for (const auto &image : images)
      this->m_images_.push_back(image);
  }

public:
  std::vector<is::ImageFeature> get_features() {
    for (const auto &image : this->m_images_)
      this->features_.push_back(this->extract_feature(image));
    return this->features_;
  }

private:
  is::ImageFeature extract_feature(const cv::Mat &image) {
    cv::Mat image_copy = image.clone();

    if (image_copy.empty())
      throw std::runtime_error("Image is empty");

    if (image_copy.channels() == 3)
      cv::cvtColor(image_copy, image_copy, cv::COLOR_BGR2GRAY);

    is::ImageFeature feature;
    feature.descriptors = cv::Mat();
    feature.keypoints = std::vector<cv::KeyPoint>();

    cv::Ptr<cv::Feature2D> sift = cv::SIFT::create();
    sift->detectAndCompute(image_copy, cv::noArray(), feature.keypoints,
                           feature.descriptors);
    return feature;
  }

private:
  std::vector<cv::Mat> m_images_;
  std::vector<is::ImageFeature> features_;
};
} // namespace image_stitching::feature_extraction

#endif // IMAGE_STITCHING_EXTRACT_FEATURE_HPP
