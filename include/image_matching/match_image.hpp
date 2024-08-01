//
// Created by arghadeep on 30.07.24.
//

#ifndef IMAGE_STITCHING_MATCH_IMAGE_HPP_
#define IMAGE_STITCHING_MATCH_IMAGE_HPP_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/types.hpp>
#include <vector>

namespace is::match_image {
/**
 * Class to match images by estimating homography using RANSAC.
 */
class MatchImage {
public:
  MatchImage() = default;

public:
  ~MatchImage() = default;

public:
  /**
   * Match the features of the images.
   * @param kp1 keypoints of the first image
   * @param kp2 keypoints of the second image
   * @param matches matches between the keypoints
   * @todo handling batch of images
   */
  std::vector<cv::DMatch> match(const std::vector<cv::KeyPoint> &kp1,
                                const std::vector<cv::KeyPoint> &kp2,
                                const std::vector<cv::DMatch> &matches) {
    std::vector<cv::DMatch> verified_matches;
    auto homography = compute_homography(kp1, kp2, matches);
    verified_matches = verify_match(kp1, kp2, matches, homography, 3.0);
    return verified_matches;
  }

public:
  /**
   * Compute the homography matrix using RANSAC.
   * @param kp1 keypoints of the first image
   * @param kp2 keypoints of the second image
   * @param matches matches between the keypoints
   * @param rejection_threshold default value is 3.0
   * @param maxIters default value is 3000
   * @param confidence default value is 0.995
   */
  static cv::Mat compute_homography(const std::vector<cv::KeyPoint> &kp1,
                                    const std::vector<cv::KeyPoint> &kp2,
                                    const std::vector<cv::DMatch> &matches,
                                    const double &rejection_threshold = 3.0,
                                    const int &maxIters = 3000,
                                    const double &confidence = 0.995) {
    std::vector<cv::Point2f> points1, points2;
    for (auto match : matches) {
      points1.push_back(kp1[match.queryIdx].pt);
      points2.push_back(kp2[match.trainIdx].pt);
    }
    std::vector<unsigned char> inliers_mask(points1.size());
    cv::Mat H =
        cv::findHomography(points1, points2, cv::RANSAC, rejection_threshold,
                           inliers_mask, maxIters, confidence);
    return H;
  }

private:
  /**
   * Verify the matches using the homography matrix.
   * @param kp1 keypoints of the first image
   * @param kp2 keypoints of the second image
   * @param matches matches between the keypoints
   * @param H homography matrix
   * @param maxReprojectionError default value is 3.0
   */
  std::vector<cv::DMatch> verify_match(const std::vector<cv::KeyPoint> &kp1,
                                       const std::vector<cv::KeyPoint> &kp2,
                                       const std::vector<cv::DMatch> &matches,
                                       const cv::Mat &H,
                                       const double &maxReprojectionError) {
    std::vector<cv::DMatch> inliers;

    for (const auto &match : matches) {
      cv::Point2f pt1 = kp1[match.queryIdx].pt;
      cv::Point2f pt2 = kp2[match.trainIdx].pt;

      // Converting points to homogeneous coordinates
      cv::Mat pt1Mat = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1.f);
      cv::Mat pt2Mat = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1.f);

      cv::Mat pt1_transformed = H * pt1Mat;
      pt1_transformed /= pt1_transformed.at<double>(2); // Normalizing

      // Compute re-projection error
      double error = cv::norm(pt2 - cv::Point2f(pt1_transformed.at<double>(0),
                                                pt1_transformed.at<double>(1)));

      if (error <= maxReprojectionError)
        inliers.push_back(match);
    }
    return inliers;
  }
};
} // namespace is::match_image
#endif // IMAGE_STITCHING_MATCH_IMAGE_HPP_
