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
#include <featues/features.hpp>
#include <utils/image_visualizer.hpp>

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
    std::cout << "src_img_typeL " << src_img.type() << std::endl;
    std::cout << "dst_img_typeL " << dst_img.type() << std::endl;

    is::features::Features features;
    cv::Mat homography = features.compute_homography(src_img, dst_img);

    // Height and Width of the source image
    int src_height = src_img.rows;
    int src_width = src_img.cols;

    // Height and Width of the destination image
    int dst_height = dst_img.rows;
    int dst_width = dst_img.cols;

    // Extract the corners of the source image
    std::vector<cv::Point2f> src_corners = {cv::Point2f(0, 0),
                                            cv::Point2f(0, static_cast<float>(src_height)),
                                            cv::Point2f(static_cast<float>(src_width), static_cast<float>(src_height)),
                                            cv::Point2f(static_cast<float>(src_width), 0)};

    // Extract the corners of the destination image
    std::vector<cv::Point2f> dst_corners = {cv::Point2f(0, 0),
                                            cv::Point2f(0, static_cast<float>(dst_height)),
                                            cv::Point2f(static_cast<float>(dst_width), static_cast<float>(dst_height)),
                                            cv::Point2f(static_cast<float>(dst_width), 0)};

    // Apply perspective transformation to the source image
    std::vector<cv::Point2f> src_warped_corners;
    cv::perspectiveTransform(src_corners, src_warped_corners, homography);

    std::vector<cv::Point2f> dst_corners_warped = src_warped_corners;
    dst_corners_warped.insert(dst_corners_warped.end(), dst_corners.begin(), dst_corners.end());

    // Extracting Min and Max points
    cv::Point2f min_point, max_point;
    auto min_max_coords = this->min_max_points(dst_corners_warped);
    min_point = min_max_coords.first;
    max_point = min_max_coords.second;

    cv::Point2f translation(-min_point.x, -min_point.y);
    std::string side;
    int width_pano, height_pano = static_cast<int>(max_point.y - min_point.x);

    if (dst_corners_warped[0].x < 0) {
      width_pano = dst_width + static_cast<int>(translation.x);
      side = "left";
    } else {
      width_pano = static_cast<int>(src_warped_corners[3].x);
      side = "right";
    }

    // Translation Matrix
    cv::Mat Ht = cv::Mat::eye(3, 3, homography.type());
    Ht.at<double>(0, 2) = translation.x;
    Ht.at<double>(1, 2) = translation.y;

    cv::Mat src_img_warped;
    cv::warpPerspective(src_img, src_img_warped, Ht * homography, cv::Size(width_pano, height_pano));

    // Resize the destination image
    cv::Mat dst_img_rz = cv::Mat::zeros(cv::Size(width_pano, height_pano), dst_img.type());
    if (side == "left") {
      dst_img_rz(cv::Rect(static_cast<int>(translation.x),
                          static_cast<int>(translation.y),
                          dst_width, src_height)) = dst_img;
    } else {
      dst_img_rz(cv::Rect(0, static_cast<int>(translation.y),
                          dst_width, src_height)) = dst_img;
    }

    // Panorama Blending
    auto [pano, nonblend, leftside, rightside] =
        this->panoramaBlending(dst_img_rz, src_img_warped, dst_width, side, true);

    pano = crop(pano, dst_height, dst_corners_warped);
    is::vis::display(pano, "Pano Image");
//    test(dst_img_rz, src_img_warped, dst_width, side, true);
    is::vis::display(src_img_warped, "Warped Image");

  }

 private:
  std::pair<cv::Point2f, cv::Point2f> min_max_points(const std::vector<cv::Point2f> &points) {
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
  cv::Mat blend_mask(const int &height,
                     const int &width,
                     const int &barrier,
                     const int &smoothing_window,
                     bool left_biased) {
    assert(barrier < width);
    cv::Mat mask = cv::Mat::zeros(height, width, CV_32F);
    std::cout << "mask type: " << mask.type() << std::endl;
    int offset = smoothing_window / 2;
    try {
      if (left_biased) {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j <= barrier + offset; ++j) {
            mask.at<float>(i, j) = 1 - (float) (j - (barrier - offset)) / (2 * offset + 1);
          }
        }
        mask.colRange(0, barrier - offset).setTo(1);
      } else {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j <= barrier + offset; ++j) {
            mask.at<float>(i, j) = (float) (j - (barrier - offset)) / (2 * offset + 1);
          }
        }
        mask.colRange(barrier + offset, width).setTo(1);
      }
    } catch (const std::exception &) {
      if (left_biased) {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j < barrier + offset; ++j) {
            mask.at<float>(i, j) = 1 - (float) (j - (barrier - offset)) / (2 * offset);
          }
        }
        mask.colRange(0, barrier - offset).setTo(1);
      } else {
        for (int i = 0; i < height; ++i) {
          for (int j = barrier - offset; j < barrier + offset; ++j) {
            mask.at<float>(i, j) = (float) (j - (barrier - offset)) / (2 * offset);
          }
        }
        mask.colRange(barrier + offset, width).setTo(1);
      }
    }

    cv::Mat mask_3channel;
    std::vector<cv::Mat> channels = {mask, mask, mask};
    merge(channels, mask_3channel);
    mask_3channel.convertTo(mask_3channel, CV_8U);
    cv::normalize(mask_3channel, mask_3channel, 0, 1, cv::NORM_MINMAX);
    return mask_3channel;
  }

 private:
  std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> panoramaBlending(const cv::Mat &dst_img_rz,
                                                                  const cv::Mat &src_img_warped,
                                                                  int width_dst,
                                                                  const std::string &side,
                                                                  bool showstep = false) {
    int h = dst_img_rz.rows;
    int w = dst_img_rz.cols;
    int smoothing_window = width_dst / 8;
    int barrier = width_dst - smoothing_window / 2;

    cv::Mat mask1 = blend_mask(h, w, barrier, smoothing_window, true);
    cv::Mat mask2 = blend_mask(h, w, barrier, smoothing_window, false);

    std::cout << "mask1 type: " << mask1.type() << std::endl;
    std::cout << "mask2 type: " << mask2.type() << std::endl;

    cv::Mat nonblend, leftside, rightside;
    cv::Mat dst_img_rz_copy = dst_img_rz.clone();
    cv::Mat src_img_warped_copy = src_img_warped.clone();

    std::cout << "dst_img_rz_copy type: " << dst_img_rz_copy.type() << std::endl;
    std::cout << "src_img_warped_copy type: " << src_img_warped_copy.type() << std::endl;


    // Asserting both the images and masks of same type
    mask1.convertTo(mask1, dst_img_rz_copy.type());
    mask2.convertTo(mask2, src_img_warped_copy.type());

    std::cout << "mask1 type: " << mask1.type() << std::endl;
    std::cout << "mask2 type: " << mask2.type() << std::endl;

    if (side == "left") {
      cv::flip(dst_img_rz_copy, dst_img_rz_copy, 1);
      cv::flip(src_img_warped_copy, src_img_warped_copy, 1);

      dst_img_rz_copy = dst_img_rz_copy.mul(mask1);
      src_img_warped_copy = src_img_warped_copy.mul(mask2);

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
      src_img_warped_copy = src_img_warped_copy.mul(mask2);
      cv::Mat pano = src_img_warped_copy + dst_img_rz_copy;
      if (showstep) {
        leftside = dst_img_rz_copy.clone();
        rightside = src_img_warped_copy.clone();
      }
      return {pano, nonblend, leftside, rightside};
    }
  }

 private:
  cv::Mat crop(const cv::Mat &panorama, const int &height, const std::vector<cv::Point2f> &corners) {
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
                   (int) std::min(corners[2].x, corners[3].x), height);
      cropped_panorama = panorama(roi);
    }
    return cropped_panorama;
  }
};
}// namespace is::blend

#endif// IMAGE_STITCHING_BLEND_HPP
