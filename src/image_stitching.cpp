#include <feature_extraction/extract_feature.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <utils/image_reader.hpp>
#include <utils/types.hpp>
#include <utils/scoped_timer.hpp>

int main() {
  std::cout << "Hello, OpenCV!" << std::endl;
  std::string image_path = "../data/001.jpg";
  cv::Mat image = image_stitching::utils::read_image(image_path);
  image_stitching::is::ImageFeature feature;
  image_stitching::feature_extraction::FeatureExtraction feature_extraction;
  feature_extraction.load_images({image});
  {
    ScopedTimer timer;
    std::vector<image_stitching::is::ImageFeature> features = feature_extraction.get_features();
  }
}