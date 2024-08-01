/**
 * @file blend.hpp
 * @details Image blending using multi-band blending.
 * @author Arghadeep Mazumder
 * @version 0.1.0
 * @copyright -
 */

#ifndef IMAGE_STITCHING_BLEND_HPP
#define IMAGE_STITCHING_BLEND_HPP

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <utils/types.hpp>

namespace is::blend {
/**
 * Image blending class
 */
class ImageBlending {
 public:
  ImageBlending() = default;

 public:
  ~ImageBlending() = default;

 public:
  /**
   * Blend two images using the given homography matrix.
   * @param image1 The first image.
   * @param image2 The second image.
   * @param homography The homography matrix.
   * @todo Implement the advanced exposure compensation method
   * @todo Implement handling multiple images
   * @return The blended image.
   */
  cv::Mat blend(const cv::Mat &image1, const cv::Mat &image2, const cv::Mat &homography) {
    return warp_images(image1, image2, homography);
//    cv::Size size = {image1.cols + image2.cols, image1.rows};
//    // 1. Warp image
//    auto wrapped_image1 = warp_image(image1, homography, size);
//    auto wrapped_image2 = warp_image(image2, cv::Mat::eye(3, 3, CV_64F), size);
//
//    // 2. Exposure Compensation
//    cv::Mat mask = cv::Mat::zeros(size, CV_8U);
//    cv::rectangle(mask, cv::Rect(0, 0, size.width, size.height), cv::Scalar(255), -1);
//    auto exposure_compensated_image = exposure_compensation(wrapped_image1, wrapped_image2, mask);
//
//    // 3. Find Optimal Seam
//    auto optimal_seam = find_optimal_seam(wrapped_image1, wrapped_image2);
//
//    // 4. Multi-band blending
//    auto blended_image = multi_band_blending(wrapped_image1, wrapped_image2, mask);
//    return blended_image;
  };

 private:
  /**
   * @brief Warp images using the given homography matrix
   * @param image1
   * @param image2
   * @param H
   * @return
   */
  cv::Mat warp_images(const cv::Mat &image1, const cv::Mat &image2, const cv::Mat &H) {
    // Get image dimensions
    int h1 = image1.rows;
    int w1 = image1.cols;
    int h2 = image2.rows;
    int w2 = image2.cols;

    // Points for the transformed image boundaries
    std::vector<cv::Point2f> pts1 = {cv::Point2f(0, 0), cv::Point2f(0, h1),
                                     cv::Point2f(w1, h1), cv::Point2f(w1, 0)};
    std::vector<cv::Point2f> pts2 = {cv::Point2f(0, 0), cv::Point2f(0, h2),
                                     cv::Point2f(w2, h2), cv::Point2f(w2, 0)};

    // Transform points to the new coordinates using the homography matrix
    std::vector<cv::Point2f> pts2_transformed;
    cv::perspectiveTransform(pts2, pts2_transformed, H);

    // Combine the points from both images
    std::vector<cv::Point2f> all_points = pts1;
    all_points.insert(all_points.end(), pts2_transformed.begin(), pts2_transformed.end());

    // Get the bounding box of the combined image
    cv::Rect bbox = cv::boundingRect(all_points);

    // Calculate the translation matrix
    cv::Mat translation_matrix = (cv::Mat_<double>(3, 3) << 1, 0, -bbox.x, 0, 1, -bbox.y, 0, 0, 1);

    // Warp the first image
    cv::Mat warped_image1;
    cv::warpPerspective(image1, warped_image1, translation_matrix * H, bbox.size());

    // Warp the second image
    cv::Mat warped_image2;
    cv::warpPerspective(image2, warped_image2, translation_matrix, bbox.size());

    // Create a mask for the second image
    cv::Mat mask2 = (warped_image2 > 0);
    cv::cvtColor(mask2, mask2, cv::COLOR_BGR2GRAY);

    // Blend the two images together
    cv::Mat result = warped_image1.clone();
    warped_image2.copyTo(result, mask2);

    return result;
  }

 private:
  /**
   * @brief Warp image using the given homography matrix
   * @param image
   * @param homography
   * @return
   */
  cv::Mat warp_image(const cv::Mat &image, const cv::Mat &homography,
                     const cv::Size &size) {
    cv::Mat warped_image;
    cv::warpPerspective(image, warped_image, homography, size);
    return warped_image;
  };

 private:
  void warpMultiImages() {};

 private:
  /**
   * Exposure compensation basic implementation
   * @todo replace with advanced method of exposure compensation
   * @param image1
   * @param image2
   * @param mask
   * @return Exposure compensated image
   */
  cv::Mat exposure_compensation(const cv::Mat &image1, const cv::Mat &image2,
                                const cv::Mat &mask, const std::string &type = "mean") {
    cv::Mat result = image1.clone();
    cv::Scalar mean1 = cv::mean(image1, mask);
    cv::Scalar mean2 = cv::mean(image2, mask);

    if (type == "mean") {
      cv::Scalar meanDiff = mean2 - mean1;
      result.convertTo(result, -1, 1, -meanDiff[0]);
    } else if (type == "gain") {
      cv::Scalar gain = mean2 / mean1;
      result.convertTo(result, -1, gain[0]);
    } else {
      std::cerr << "Error: Invalid exposure compensation type." << std::endl;
    }
    return result;
  };

 private:
  /**
   * Find optimal seam between two images using absolute difference between the images
   * @param image1
   * @param image2
   * @return Gray Scale Optimal Seam
   */
  cv::Mat find_optimal_seam(const cv::Mat &image1, const cv::Mat &image2) {
    cv::Mat diff, gray_diff;
    cv::absdiff(image1, image2, diff);
    cv::cvtColor(diff, gray_diff, cv::COLOR_BGR2GRAY);
    return gray_diff;
  }

 private:
  /**
   * Laplacian pyramid implementation
   * @param image
   * @param levels
   * @return Laplacian pyramid
   */
  std::vector<cv::Mat> laplacian_pyramid(const cv::Mat &image,
                                         const int &levels) {
    std::vector<cv::Mat> pyramid;
    cv::Mat current = image.clone();
    for (int i = 0; i < levels; i++) {
      cv::Mat down, up;
      cv::pyrDown(current, down);
      cv::pyrUp(down, up, current.size());
      cv::Mat laplacian = current - up;
      pyramid.push_back(laplacian);
      current = down;
    }
    pyramid.push_back(current);
    return pyramid;
  }

 private:
  /**
   * Multi-band blending implementation
   * @param image1
   * @param image2
   * @param mask
   * @param levels
   * @return
   */
  cv::Mat multi_band_blending(const cv::Mat &image1,
                              const cv::Mat &image2,
                              const cv::Mat &mask,
                              const int &levels = 5) {
    cv::Mat output;
    // Building Laplacian pyramid for the images
    auto laplacian_pyramid1 = laplacian_pyramid(image1, levels);
    auto laplacian_pyramid2 = laplacian_pyramid(image2, levels);

    // Laplacian pyramid for the mask
    std::vector<cv::Mat> mask_pyramid;

    // Building Gaussian pyramid for the mask
    cv::Mat curr_mask = mask.clone();
    for (size_t i = 0; i < levels; i++) {
      cv::Mat down;
      cv::pyrDown(curr_mask, down);
      mask_pyramid.push_back(curr_mask);
      curr_mask = down;
    }
    mask_pyramid.push_back(curr_mask);

    // Blending Pyramids
    std::vector<cv::Mat> blended_pyramid;
    for (size_t i = 0; i < levels; i++) {
      cv::Mat blended;
      // Ensure that the mask has the same number of channels as the Laplacian pyramids
      cv::Mat mask_ = mask_pyramid[i];  // Create a non-const copy
      if (mask_.channels() == 1 && laplacian_pyramid1[i].channels() == 3) {
        std::vector<cv::Mat> mask_channels(3);
        for (int c = 0; c < 3; ++c) {
          mask_channels[c] = mask_;  // Copy single-channel mask to each channel
        }
        cv::merge(mask_channels, mask_);
      }
      // Multiply the mask with the first Laplacian pyramid image
      cv::multiply(mask_, laplacian_pyramid1[i], blended);
      // Multiply the inverse mask with the second Laplacian pyramid image
      cv::Mat temp;
      cv::Mat inverted_mask = cv::Scalar::all(1.0) - mask_;
      cv::multiply(inverted_mask, laplacian_pyramid2[i], temp);
      // Add the two results together
      blended += temp;
      blended_pyramid.push_back(blended);
    }
    // Reconstructing the image from the blended pyramid
    cv::Mat current = blended_pyramid.back();

    for (int i = levels - 1; i >= 0; i--) {
      cv::Mat up;
      // Up-sample the image; doubling the size
      if (!current.empty()) {
        cv::pyrUp(current, up);
      } else {
        std::cerr << "Error: Current matrix is empty before pyrUp." << std::endl;
        return cv::Mat(); // Return empty matrix on failure
      }
      // Expected size after up-sampling
      cv::Size expected_size = blended_pyramid[i].size();
      // If the up-sampled size does not match the expected size, adjust manually
      if (up.size() != expected_size) {
        if (!up.empty() && expected_size.width > 0 && expected_size.height > 0) {
          cv::resize(up, up, expected_size);
        } else {
          std::cerr << "Error: Upsampled matrix is empty or expected size is invalid." << std::endl;
          return cv::Mat(); // Return empty matrix on failure
        }
      }
      // Add the upsampled image to the corresponding level of the blended pyramid
      current = up + blended_pyramid[i];
    }
    output = current;
    return output;
  }
};
}// namespace is::blend

#endif// IMAGE_STITCHING_BLEND_HPP
