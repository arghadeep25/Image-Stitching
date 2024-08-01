//
// Created by arghadeep on 31.07.24.
//

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
  void blend() {
    // 1. Warp image
    
    // 2. Exposure compensation
    // 3. Find optimal seam
    // 4. Multi-band blending
  };

private:
  /**
   *
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
                                const cv::Mat &mask) {
    cv::Mat result = image1.clone();
    cv::Scalar mean1 = cv::mean(image1, mask);
    cv::Scalar mean2 = cv::mean(image2, mask);
    // using mean difference to compensate the exposure
    cv::Scalar meanDiff = mean2 - mean1;
    // using gain
    //    cv::Scalar gain = mean2 / mean1;
    //    result.convertTo(result, -1, gain[0]);
    result.convertTo(result, -1, 1, -meanDiff[0]);
    return result;
  };

private:
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
  std::vector<cv::Mat> multi_band_blending(const cv::Mat &image1,
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
    for (size_t i = 0; i <= levels; i++) {
      cv::Mat down;
      cv::pyrDown(curr_mask, down);
      mask_pyramid.push_back(curr_mask);
      curr_mask = down;
    }
    mask_pyramid.push_back(curr_mask);

    // Blending Pyramids
    std::vector<cv::Mat> blended_pyramid;
    for (size_t i = 0; i <= levels; i++) {
      cv::Mat blended;
      cv::multiply(mask_pyramid[i], laplacian_pyramid1[i], blended);
      cv::Mat temp;
      cv::multiply(cv::Scalar::all(1.0) - mask_pyramid[i],
                   laplacian_pyramid2[i], temp);
      blended += temp;
      blended_pyramid.push_back(blended);
    }

    // Reconstructing the image from the blended pyramid
    cv::Mat current = blended_pyramid.back();
    for (int i = levels - 1; i >= 0; i--) {
      cv::Mat up;
      cv::pyrUp(current, up, blended_pyramid[i].size());
      current = up + blended_pyramid[i];
    }
    output = current;
    return output;
  }
};
} // namespace is::blend

#endif // IMAGE_STITCHING_BLEND_HPP
