#include <gtest/gtest.h>
#include <features/features.hpp>
#include <utils/types.hpp>
#include <utils/image_reader.hpp>

class FeatureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    this->src_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "/data/berlin/001.jpg");
    this->dst_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "/data/berlin/002.jpg");

    if (this->src_img.empty() || this->dst_img.empty()) {
      GTEST_SKIP() << "Could not load images";
    }
  }
  is::types::Image src_img;
  is::types::Image dst_img;
  is::features::Features m_features;
};

TEST_F(FeatureTest, ComputeHomographyTest_ValidImages_ReturnsHomography) {
  ASSERT_NO_THROW({
    cv::Mat homography = this->m_features.compute_homography(src_img, dst_img);
    EXPECT_EQ(homography.rows, 3);
    EXPECT_EQ(homography.cols, 3);
  });
}

TEST_F(FeatureTest, ComputeHomographyTest_EmptyImages_ThrowsException) {
  ASSERT_THROW({
     is::types::Image empty_img;
     this->m_features.compute_homography(empty_img, empty_img);
   }, std::runtime_error);
}

TEST_F(FeatureTest, ComputeHomographyTest_EmptyDstImage_ThrowsException) {
  ASSERT_THROW({
     this->src_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "/data/berlin/001.jpg");
     is::types::Image empty_img;
     this->m_features.compute_homography(this->src_img, empty_img);
   }, std::runtime_error);
}

TEST_F(FeatureTest, ComputeHomographyTest_EmptySrcImage_ThrowsException) {
  ASSERT_THROW({
     this->dst_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "/data/berlin/001.jpg");
     is::types::Image empty_img;
     this->m_features.compute_homography(empty_img, this->dst_img);
   }, std::runtime_error);
}
