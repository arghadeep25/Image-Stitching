//
// Created by arghadeep on 29.07.24.
//

#ifndef IMAGE_STITCHING_EXTRACT_FEATURE_HPP
#define IMAGE_STITCHING_EXTRACT_FEATURE_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <mutex>
#include <thread>
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/xfeatures2d.hpp"
#endif

#include <utils/types.hpp>
#include <vector>

namespace is::extract_feature {
class FeatureExtraction {
 public:
  FeatureExtraction() = default;

 public:
  ~FeatureExtraction() {
    this->m_images_.clear();
    this->features_.clear();
  }

 public:
  /**
   * Load images in the form of a vector to extract features.
   * @param images The images to extract features from.
   * @return void
   */
  void load_images(const std::vector<cv::Mat> &images) {
    for (const auto &image : images)
      this->m_images_.push_back(image);
  }

 public:
  /**
   * Get the extracted features.
   * @return The extracted features.
   */
  inline std::vector<types::ImageFeature> get_features() {
    std::vector<std::thread> threads;
    std::vector<types::ImageFeature> local_features;
    std::mutex local_mutex;

    for (const auto &image : this->m_images_) {
      threads.emplace_back([&]() {
        auto feature = this->extract_feature(image);
        std::lock_guard<std::mutex> lock(local_mutex);
        local_features.push_back(feature);
      });
    }

    for (auto &thread : threads)
      if (thread.joinable())
        thread.join();

    {
      std::lock_guard<std::mutex> lock(this->m_mutex_);
      this->features_ = std::move(local_features);
    }

    return this->features_;
  }

 private:
  types::ImageFeature extract_feature(const cv::Mat &image) {
    cv::Mat image_copy = image.clone();

    if (image_copy.empty())
      throw std::runtime_error("Image is empty");

    if (image_copy.channels() == 3)
      cv::cvtColor(image_copy, image_copy, cv::COLOR_BGR2GRAY);

    types::ImageFeature feature;
    feature.descriptors = cv::Mat();
    feature.keypoints = std::vector<cv::KeyPoint>();

    cv::Ptr<cv::Feature2D> sift = cv::SIFT::create();
    sift->detectAndCompute(image_copy, cv::noArray(), feature.keypoints,
                           feature.descriptors);
    return feature;
  }

 private:
  std::vector<cv::Mat> m_images_;
  std::vector<types::ImageFeature> features_;
  std::mutex m_mutex_;
};

} // namespace image_stitching::feature_extraction

#endif // IMAGE_STITCHING_EXTRACT_FEATURE_HPP
