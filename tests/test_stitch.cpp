#include <gtest/gtest.h>
#include <utils/types.hpp>
#include <utils/image_reader.hpp>
#include <stitch/stitch.hpp>

class StitchTest : public ::testing::Test {
 protected:
  std::string m_test_data_path = std::string(PROJECT_SOURCE_DIR) + "/data/berlin";
  is::types::Image src_img;
  is::types::Image dst_img;
  is::stitch::Stitch m_stitch;
};

TEST_F(StitchTest, StitchImage_ValidPath_ReturnsStitchedImage) {
  ASSERT_NO_THROW({
    this->m_stitch.load_images(this->m_test_data_path, true);
    is::types::Image stitched_img = this->m_stitch.stitch();
    EXPECT_FALSE(stitched_img.empty());
  });
}

TEST_F(StitchTest, StitchImage_InvalidPath_ThrowException) {
  ASSERT_THROW({
    this->m_stitch.load_images("", true);
    is::types::Image stitched_img = this->m_stitch.stitch();
  }, std::runtime_error);
}
