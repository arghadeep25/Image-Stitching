
#ifndef IMAGE_STITCHING_IMAGE_READER_HPP
#define IMAGE_STITCHING_IMAGE_READER_HPP

#include <iostream>
#include <opencv2/opencv.hpp>

/**
 * Read an image from the given path.
 * @param image_path The path to the image.
 * @return The image read from the given path.
 */
namespace is ::utils {
types::Image read_image(const std::string &image_path) {
  if (image_path.empty())
    throw std::invalid_argument("Image path is empty");
  types::Image image = cv::imread(image_path, cv::IMREAD_COLOR);
  if (image.empty())
    throw std::runtime_error("Could not read the image: " + image_path);
  std::cout << "Successfully read the image: " << image_path << std::endl;
  return image;
}

/**
 * Read images from the given paths and store in a vector.
 * @param image_paths
 * @return
 */
types::ImageBatch read_images(const std::vector<std::string> &image_paths) {
  if (image_paths.empty())
    throw std::invalid_argument("Image paths are empty");
  types::ImageBatch images;
  for (const auto &image_path : image_paths)
    images.push_back(read_image(image_path));
  return images;
}

}// namespace is::utils
#endif// IMAGE_STITCHING_IMAGE_READER_HPP
