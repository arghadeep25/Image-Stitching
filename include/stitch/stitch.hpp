/**
 * @file stitch.hpp
 * @details Stitch multiple images together to create a panorama. The images are
 * loaded from the given path. The images are stitched using image blending.
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
#include <utils/image_reader.hpp>
#include <utils/types.hpp>
#include <vector>

namespace is::stitch {
/**
 * @brief Class to stitch multiple images together to create a panorama.
 * The images are loaded from the given path. The images are stitched
 * using image blending.
 * @details First the images are loaded from the given path. The images are
 * resized if the resize flag is set to true. The stitching process is done
 * in two steps. First, the left images are stitched together. Then, the right
 * images are stitched together. Finally, the left and right panoramas are
 * stitched together to create the final panorama.
 */
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
  /**
   * @brief Load images from the given path and set to the image batch.
   * @todo if images are greater than certain size, automatic resize
   * @param path The path to the directory containing images.
   * @param resize Flag to resize the images.
   */
  void load_images(const std::string &path, bool resize) {
    this->m_path = path;
    this->m_resize = resize;
    this->load_images();
  };

 public:
  /**
   * @brief Stitch the image batch.
   * @return The stitched image.
   */
  types::Image stitch() {
    this->stitch_images();
    return this->m_stitched_image;
  }

 private:
  /**
   * @brief Load images from the given path and stored in a batch of images.
   */
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

    if (image_paths.empty())
      throw std::runtime_error("No images found in the directory.");

    // Sort paths
    std::sort(image_paths.begin(), image_paths.end());

    // Load and optionally resize images
    for (const auto &image_path : image_paths) {
      types::Image image = utils::read_image(image_path);
      if (this->m_resize) {
        cv::resize(image, image, cv::Size(image.cols / 4, image.rows / 4));
      }
      this->m_images.push_back(image);
    }

    if (this->m_images.empty())
      throw std::runtime_error("No images loaded.");
  }

 private:
  /**
   * @brief Stitch the images.
   * @details
   */
  void stich_image_batch(types::ImageBatch &images) {
    if (images.empty())
      throw std::runtime_error("No images to stitch.");

    while (images.size() > 1) {
      types::Image dst_img = images.back();
      images.pop_back();
      types::Image src_img = images.back();
      images.pop_back();
      types::Image left_pano = this->m_blending.warp_images(src_img, dst_img);
      left_pano.convertTo(left_pano, CV_8U);
      images.push_back(left_pano);
    }
  }

 private:
  /**
   * @brief Stitch the images.
   * @details The images are split into two halves. The left images are
   * stitched together. The right images are stitched together. Finally, the
   * left and right panoramas are stitched together to create the final
   * panorama.
   */
  void stitch_images() {
    if (this->m_images.empty())
      throw std::runtime_error("No images to stitch.");

    size_t image_idx_mid = (this->m_images.size() + 1) / 2;
    types::ImageBatch left(this->m_images.begin(),
                           this->m_images.begin() + image_idx_mid);
    types::ImageBatch right(this->m_images.begin() + image_idx_mid - 1,
                            this->m_images.end());
    std::reverse(right.begin(), right.end());

    is::blend::ImageBlending blending;

    // Stitch left images
    this->stich_image_batch(left);
    // Stitch right images
    this->stich_image_batch(right);

    if (left.empty() || right.empty())
      throw std::runtime_error("No images to stitch.");

    // Final stitch of left and right panoramas
    types::Image final_pano;
    if (right.back().cols >= left.back().cols) {
      final_pano = blending.warp_images(left.back(), right.back());
    } else {
      final_pano = blending.warp_images(right.back(), left.back());
    }
    this->m_stitched_image = final_pano;
  }

 private:
  bool m_resize{};
  std::string m_path;
  types::ImageBatch m_images;
  types::Image m_stitched_image;
  is::blend::ImageBlending m_blending;
};
} // namespace is::stitch

#endif // IMAGE_STITCHING_STITCH_HPP
