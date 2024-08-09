#include <gtest/gtest.h>
#include <utils/types.hpp>
#include <utils/image_reader.hpp>
#include <blending/blend.hpp>

class BlendTest : public ::testing::Test {
 protected:
  std::string m_test_data_path = std::string(PROJECT_SOURCE_DIR) + "/data/berlin";
  is::types::Image src_img;
  is::types::Image dst_img;
  is::blend::ImageBlending m_blend;
};

TEST_F(BlendTest, BlendImagesTest_ValidImages_ReturnsBlendedImage) {
  ASSERT_NO_THROW({
    this->src_img = is::utils::read_image(this->m_test_data_path + "/001.jpg");
    this->dst_img = is::utils::read_image(this->m_test_data_path + "/002.jpg");
    cv::Mat blended_img = this->m_blend.warp_images(src_img, dst_img);
    EXPECT_FALSE(blended_img.empty());
  });
}

TEST_F(BlendTest, BlendImagesTest_EmptySrcImage_ThrowException) {
  ASSERT_THROW({
    this->src_img = is::types::Image::zeros(500, 500, CV_8U);
    this->dst_img = is::utils::read_image(this->m_test_data_path + "/002.jpg");
    cv::Mat blended_img = this->m_blend.warp_images(src_img, dst_img);
  }, std::runtime_error);
}

TEST_F(BlendTest, BlendImagesTest_EmptyDstImage_ThrowException) {
  ASSERT_THROW({
    this->src_img = is::utils::read_image(this->m_test_data_path + "/001.jpg");
    this->dst_img = is::types::Image::zeros(500, 500, CV_8U);
    cv::Mat blended_img = this->m_blend.warp_images(src_img, dst_img);
  }, std::runtime_error);
}