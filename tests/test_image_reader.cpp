#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>
#include <utils/image_reader.hpp>
#include <utils/types.hpp>

class ImageReadTest : public ::testing::Test {
 protected:
  is::types::Image src_img;
  is::types::Image dst_img;

};

TEST_F(ImageReadTest, ReadImageTest_ReturnImage) {
  ASSERT_NO_THROW({
    this->src_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "/data/berlin/001.jpg");
    this->dst_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "/data/berlin/002.jpg");
    EXPECT_FALSE(this->src_img.empty());
    EXPECT_FALSE(this->dst_img.empty());
  });
}

TEST_F(ImageReadTest, HandleImageFailureTest_ThrowException) {
  ASSERT_THROW({
     this->src_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "something.jpg");
     this->dst_img = is::utils::read_image(std::string(PROJECT_SOURCE_DIR) + "anything.jpg");
   }, std::runtime_error);
}
