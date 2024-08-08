#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>
#include <utils/image_reader.hpp>
#include <utils/types.hpp>

TEST(OpenCVTest, CanLoadImage) {
  is::types::Image image = is::utils::read_image("../../data/berlin/001.jpg");
  if (image.empty()) std::cout << "Image is empty" << std::endl;
  ASSERT_FALSE(image.empty());
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}