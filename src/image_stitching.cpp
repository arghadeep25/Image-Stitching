
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stitch/stitch.hpp>
#include <utils/image_visualizer.hpp>
#include <utils/scoped_timer.hpp>
#include <utils/argument_parser.hpp>

int main(int argc, char *argv[]) {
  std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
  std::string input_path, output_path;
  is::utils::parse(argc, argv, input_path, output_path);
  cv::Mat pano;
  {
    ScopedTimer timer;
    is::stitch::Stitch stitch;
    stitch.load_images(input_path, false);
    pano = stitch.stitch();
  }
  is::vis::display(pano, "Panorama");
  cv::imwrite(output_path, pano);
}