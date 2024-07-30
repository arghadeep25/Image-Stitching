#include <feature_extraction/extract_feature.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/image_reader.hpp>
#include <utils/types.hpp>
#include <utils/scoped_timer.hpp>
#include <feature_match/match_feature.hpp>
#include <utils/image_visualizer.hpp>
#include <image_matching/match_image.hpp>

int main() {
  std::cout << "OpenCV Version: " << CV_VERSION << std::endl;

  std::vector<std::string> image_paths;
  image_paths.emplace_back("../data/001.jpg");
  image_paths.emplace_back("../data/002.jpg");
//  image_paths.emplace_back("../data/003.jpg");

  is::types::ImageBatch images = is::utils::read_images(image_paths);

  is::types::ImageFeature feature;
  is::extract_feature::FeatureExtraction feature_extraction;
  feature_extraction.load_images(images);

//  {
//    ScopedTimer timer;
  is::types::ImageFeatures features = feature_extraction.get_features();
  std::cout << "Feature Size: " << features.size() << std::endl;
//  for (size_t idx = 0; idx < features.size(); idx++) {
//    is::types::Image disp_;
//    cv::drawKeypoints(images[idx], features[idx].keypoints, disp_);
//    is::vis::display(disp_, "Image " + std::to_string(idx));
//  }
  auto matches = is::feature_match::MatchFeature::match(features);
//  }
//  is::types::Image disp_img;
//  cv::drawMatches(images[0], features[0].keypoints, images[1], features[1].keypoints, matches, disp_img);
//  is::vis::display(disp_img);

  auto homography = is::match_image::MatchImage::match(features[0].keypoints, features[1].keypoints, matches);
  is::types::Image stitched_image;
  cv::drawMatches(images[0], features[0].keypoints, images[1], features[1].keypoints, matches, stitched_image);
}