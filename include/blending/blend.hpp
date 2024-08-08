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

#include <features/features.hpp>
#include <opencv2/opencv.hpp>
#include <utils/image_visualizer.hpp>
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
  void warp_images(const cv::Mat &src_img, const cv::Mat &dst_img) {
    if (src_img.empty() || dst_img.empty()) {
      std::cerr << "Images are empty." << std::endl;
      return;
    }
    // Size of the source and destination image
    cv::Size src_img_size = src_img.size();
    cv::Size dst_img_size = dst_img.size();

    // Compute the homography matrix
    is::features::Features features;
    cv::Mat homography = features.compute_homography(src_img, dst_img);
    std::cout << "Homography Matrix: \n" << homography << std::endl;

    // Extract the corners of the source image
    std::vector<cv::Point2f> src_corners = {
        cv::Point2f(0, 0),
        cv::Point2f(0, static_cast<float>(src_img_size.height)),
        cv::Point2f(static_cast<float>(src_img_size.width),
                    static_cast<float>(src_img_size.height)),
        cv::Point2f(static_cast<float>(src_img_size.width), 0)};

    // Extract the corners of the destination image
    std::vector<cv::Point2f> dst_corners = {
        cv::Point2f(0, 0),
        cv::Point2f(0, static_cast<float>(dst_img_size.height)),
        cv::Point2f(static_cast<float>(dst_img_size.width),
                    static_cast<float>(dst_img_size.height)),
        cv::Point2f(static_cast<float>(dst_img_size.width), 0)};

    // Apply perspective transformation to the source image based on the
    // homography matrix
    std::vector<cv::Point2f> src_warped_corners;
    cv::perspectiveTransform(src_corners, src_warped_corners, homography);

    // Combine the source and destination corners
    std::vector<cv::Point2f> dst_corners_warped = src_warped_corners;
    dst_corners_warped.insert(dst_corners_warped.end(), dst_corners.begin(),
                              dst_corners.end());

    // Extracting Min and Max points from the merged warped corners
    cv::Point2f min_point, max_point;
    auto min_max_coords = this->min_max_points(dst_corners_warped);
    min_point = min_max_coords.first;
    max_point = min_max_coords.second;

    // Translation Coordinates
    cv::Point2f translation{-min_point.x, -min_point.y};
    std::string side;

    // Estimate the size of the panorama image
    int width_pano, height_pano = static_cast<int>(max_point.y - min_point.y);

    if (dst_corners_warped[0].x < 0) {
      width_pano = dst_img_size.width + static_cast<int>(translation.x);
      side = "left";
    } else {
      width_pano = static_cast<int>(src_warped_corners[3].x);
      side = "right";
    }

    // Translation Matrix
    cv::Mat Ht = cv::Mat::eye(3, 3, homography.type());
    Ht.at<double>(0, 2) = translation.x;
    Ht.at<double>(1, 2) = translation.y;

    // Warp the source image
    cv::Mat src_img_warped;
    cv::warpPerspective(src_img, src_img_warped, Ht * homography,
                        cv::Size(width_pano, height_pano));

    // Resize the destination image
    cv::Mat dst_img_rz =
        cv::Mat::zeros(cv::Size(width_pano, height_pano), dst_img.type());

    auto roi = cv::Rect(0, static_cast<int>(translation.y), dst_img_size.width,
                        src_img_size.height);
    side == "left" ? roi.x = static_cast<int>(translation.x) : roi.x = 0;

    dst_img.copyTo(dst_img_rz(roi));

    // Panorama Blending
    auto [pano, nonblend, leftside, rightside] = this->panoramaBlending(
        dst_img_rz, src_img_warped, dst_img_size.width, side, false);

    pano = crop(pano, dst_img_size.height, dst_corners_warped);
    is::vis::display(pano, "Pano Image");
    //    test(dst_img_rz, src_img_warped, dst_width, side, true);
    //    is::vis::display(src_img_warped,                     "Warped Image");
  }

private:
  /**
   * @brief Extract the min and max points from the given points.
   * @param points The points to extract min and max points from.
   * @return The min and max points.
   */
  std::pair<cv::Point2f, cv::Point2f>
  min_max_points(const std::vector<cv::Point2f> &points) {
    cv::Point2f min_point, max_point;
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto &pt : points) {
      min_x = std::min(min_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_x = std::max(max_x, pt.x);
      max_y = std::max(max_y, pt.y);
    }

    min_point = {min_x, min_y};
    max_point = {max_x, max_y};

    return {min_point, max_point};
  }

private:
  /**
   * @brief Blending mask for the given image.
   * @param height The height of the image.
   * @param width The width of the image.
   * @param barrier The barrier for the mask.
   * @param smoothing_window The smoothing window for the mask.
   * @param left_biased The bias for the mask.
   * @return The blended mask.
   */
  cv::Mat blend_mask(const int &height, const int &width, const int &barrier,
                     const int &smoothing_window, bool left_biased) {
    assert(barrier < width);
    cv::Mat mask = cv::Mat::zeros(height, width, CV_32F);
    int offset = smoothing_window / 2;
    try {
      if (left_biased) {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j <= barrier + offset; ++j) {
            mask.at<float>(i, j) =
                1 - (float)(j - (barrier - offset)) / (2 * offset + 1);
          }
        }
        mask.colRange(0, barrier - offset).setTo(1);
      } else {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j <= barrier + offset; ++j) {
            mask.at<float>(i, j) =
                (float)(j - (barrier - offset)) / (2 * offset + 1);
          }
        }
        mask.colRange(barrier + offset, width).setTo(1);
      }
    } catch (const std::exception &) {
      if (left_biased) {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j < barrier + offset; ++j) {
            mask.at<float>(i, j) =
                1 - (float)(j - (barrier - offset)) / (2 * offset);
          }
        }
        mask.colRange(0, barrier - offset).setTo(1);
      } else {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j < barrier + offset; ++j) {
            mask.at<float>(i, j) =
                (float)(j - (barrier - offset)) / (2 * offset);
          }
        }
        mask.colRange(barrier + offset, width).setTo(1);
      }
    }
    cv::normalize(mask, mask, 0, 1, cv::NORM_MINMAX);

    cv::Mat mask_3channel;
    std::vector<cv::Mat> channels = {mask, mask, mask};
    merge(channels, mask_3channel);

    return mask_3channel;
  }

private:
  /**
   * @brief Panorama blending for the given images.
   * @param dst_img_rz The destination image.
   * @param src_img_warped The source image.
   * @param width_dst The width of the destination image.
   * @param side The side of the image.
   * @param showstep The flag to show the steps.
   * @return The blended images.
   */
  std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat>
  panoramaBlending(const cv::Mat &dst_img_rz, const cv::Mat &src_img_warped,
                   const int &width_dst, const std::string &side,
                   bool showstep = false) {
    cv::Size dst_img_size = dst_img_rz.size();
    int smoothing_window = width_dst / 8;
    int barrier = width_dst - smoothing_window / 2;

    // Generate masks with CV_32F type
    cv::Mat mask1 = blend_mask(dst_img_size.height, dst_img_size.width, barrier,
                               smoothing_window, true);
    cv::Mat mask2 = blend_mask(dst_img_size.height, dst_img_size.width, barrier,
                               smoothing_window, false);

    // Copy the images
    cv::Mat dst_img_rz_copy = dst_img_rz.clone();
    cv::Mat src_img_warped_copy = src_img_warped.clone();
    // Convert images to CV_32F for consistent operations
    dst_img_rz_copy.convertTo(dst_img_rz_copy, CV_32F);
    src_img_warped_copy.convertTo(src_img_warped_copy, CV_32F);

    cv::Mat nonblend, leftside, rightside;

    if (showstep) {
      nonblend = dst_img_rz_copy + src_img_warped_copy;
    }

    // Blending process
    if (side == "left") {
      cv::flip(dst_img_rz_copy, dst_img_rz_copy, 1);
      cv::flip(src_img_warped_copy, src_img_warped_copy, 1);

      dst_img_rz_copy = dst_img_rz_copy.mul(mask1);
      dst_img_rz_copy.convertTo(dst_img_rz_copy, CV_8U);

      src_img_warped_copy = src_img_warped_copy.mul(mask2);
      src_img_warped_copy.convertTo(src_img_warped_copy, CV_8U);

      cv::Mat pano = src_img_warped_copy + dst_img_rz_copy;
      cv::flip(pano, pano, 1);
      if (showstep) {
        leftside = src_img_warped_copy.clone();
        rightside = dst_img_rz_copy.clone();
        cv::flip(leftside, leftside, 1);
        cv::flip(rightside, rightside, 1);
      }
      return {pano, nonblend, leftside, rightside};
    } else {
      dst_img_rz_copy = dst_img_rz_copy.mul(mask1);
      dst_img_rz_copy.convertTo(dst_img_rz_copy, CV_8U);

      src_img_warped_copy = src_img_warped_copy.mul(mask2);
      src_img_warped_copy.convertTo(src_img_warped_copy, CV_8U);

      cv::Mat pano = src_img_warped_copy + dst_img_rz_copy;
      if (showstep) {
        leftside = dst_img_rz_copy.clone();
        rightside = src_img_warped_copy.clone();
      }
      // Normalize and convert to CV_8U
      cv::normalize(pano, pano, 0, 255, cv::NORM_MINMAX);
      pano.convertTo(pano, CV_8U);

      return {pano, nonblend, leftside, rightside};
    }
  }

private:
  /**
   * @brief Crop the panorama image.
   * @param panorama The panorama image.
   * @param height The height of the panorama.
   * @param corners The corners of the panorama.
   * @return The cropped panorama.
   */
  cv::Mat crop(const cv::Mat &panorama, const int &height,
               const std::vector<cv::Point2f> &corners) {
    cv::Point2f min_point, max_point;
    auto min_max_points = this->min_max_points(corners);
    min_point = min_max_points.first;
    max_point = min_max_points.second;

    cv::Point2f translation(-min_point.x, -min_point.y);
    int n = std::abs(-corners[1].x + corners[0].x);
    cv::Mat cropped_panorama;
    if (corners[0].x < 0) {
      cv::Rect roi(n, static_cast<int>(translation.y),
                   static_cast<int>(panorama.cols - n), height);
      cropped_panorama = panorama(roi);
    } else {
      cv::Rect roi(0, static_cast<int>(translation.y),
                   (int)std::min(corners[2].x, corners[3].x), height);
      cropped_panorama = panorama(roi);
    }
    return cropped_panorama;
  }
};
} // namespace is::blend

#endif // IMAGE_STITCHING_BLEND_HPP
