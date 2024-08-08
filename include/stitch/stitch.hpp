/**
 * @file blend.hpp
 * @details Image blending using multi-band blending.
 * @author Arghadeep Mazumder
 * @version 0.1.0
 * @copyright -
 */

#ifndef IMAGE_STITCHING_STITCH_HPP
#define IMAGE_STITCHING_STITCH_HPP

#include <blending/blend.hpp>
#include <filesystem>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>
#include <utils/types.hpp>
#include <vector>

namespace is::stitch {
class Stitch {
public:
  Stitch() = default;

public:
  ~Stitch() {
    if (!this->m_images.empty()) {
      std::cout << "Flushing the system memory" << std::endl;
      this->m_images.clear();
    }
  };

public:
  void load_images(const std::string &path, bool resize) {
    this->m_path = path;
    this->m_resize = resize;
    this->load_images();
  };

public:
  cv::Mat stitch() {
    this->stitch_images();
    return this->m_stitched_image;
  }

public:
  void load_images() {

    // Check if the directory exists
    if (!std::filesystem::exists(this->m_path) ||
        !std::filesystem::is_directory(this->m_path)) {
      throw std::runtime_error(
          "Directory does not exist or is not a directory.");
    }

    std::vector<std::string> image_paths;

    // Collect all image file paths
    for (const auto &entry :
         std::filesystem::directory_iterator(this->m_path)) {
      if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
        image_paths.push_back(entry.path().string());
      }
    }

    // Sort paths
    std::sort(image_paths.begin(), image_paths.end());

    // Load and optionally resize images
    for (const auto &image_path : image_paths) {
      cv::Mat image = cv::imread(image_path);
      if (image.empty()) {
        throw std::runtime_error("Failed to load image: " + image_path);
      }
      if (this->m_resize) {
        cv::resize(image, image, cv::Size(image.cols / 4, image.rows / 4));
      }
      this->m_images.push_back(image);
    }
  }

public:
  void stitch_images() {
    size_t n = (this->m_images.size() + 1) / 2;
    std::vector<cv::Mat> left(this->m_images.begin(),
                              this->m_images.begin() + n);
    std::vector<cv::Mat> right(this->m_images.begin() + n - 1,
                               this->m_images.end());
    std::reverse(right.begin(), right.end());

    is::blend::ImageBlending blending;

    // Stitch left images
    while (left.size() > 1) {
      cv::Mat dst_img = left.back();
      left.pop_back();
      cv::Mat src_img = left.back();
      left.pop_back();
      cv::Mat left_pano = blending.warp_images(src_img, dst_img);
      left_pano.convertTo(left_pano, CV_8U);
      left.push_back(left_pano);
    }

    // Stitch right images
    while (right.size() > 1) {
      cv::Mat dst_img = right.back();
      right.pop_back();
      cv::Mat src_img = right.back();
      right.pop_back();
      cv::Mat right_pano = blending.warp_images(src_img, dst_img);
      right_pano.convertTo(right_pano, CV_8U);
      right.push_back(right_pano);
    }

    // Final stitch of left and right panoramas
    cv::Mat final_pano;
    if (right.back().cols >= left.back().cols) {
      final_pano = blending.warp_images(left.back(), right.back());
    } else {
      final_pano = blending.warp_images(right.back(), left.back());
    }
    this->m_stitched_image = final_pano;
    //    return final_pano;
  }

private:
  bool m_resize;
  std::string m_path;
  std::vector<cv::Mat> m_images;
  types::Image m_stitched_image;
};
} // namespace is::stitch

#endif // IMAGE_STITCHING_STITCH_HPP
