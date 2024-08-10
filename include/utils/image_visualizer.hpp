/**
 * @file image_visualizer.hpp
 * @details Header to visualize the image using OpenCV function.
 * @author Arghadeep Mazumder
 * @version 0.1.0
 * @copyright -
 */

#ifndef IMAGE_STITCHING_IMAGE_VISUALIZER_HPP_
#define IMAGE_STITCHING_IMAGE_VISUALIZER_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/types.hpp>

namespace is::vis {
/**
 * Display the image.
 * @param image The image to display.
 * @param window_name The window name.
 * @return void
 */
inline void display(const types::Image &image,
                    const std::string &window_name = "Image") {
  cv::imshow(window_name, image);
  cv::waitKey(0);
}
} // namespace is::vis

#endif // IMAGE_STITCHING_IMAGE_VISUALIZER_HPP_
