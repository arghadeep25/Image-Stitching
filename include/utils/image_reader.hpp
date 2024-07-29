
#ifndef IMAGE_STITCHING_IMAGE_READER_HPP
#define IMAGE_STITCHING_IMAGE_READER_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

/**
 * Read an image from the given path.
 * @param image_path The path to the image.
 * @return The image read from the given path.
 */
namespace image_stitching ::utils {
cv::Mat read_image(const std::string &image_path) {
  if (image_path.empty())
    throw std::invalid_argument("Image path is empty");
  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  if (image.empty())
    throw std::runtime_error("Could not read the image: " + image_path);
  std::cout << "Successfully read the image: " << image_path << std::endl;
  return image;
}
} // namespace image_stitching::utils

#endif // IMAGE_STITCHING_IMAGE_READER_HPP
