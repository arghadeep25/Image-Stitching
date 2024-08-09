
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stitch/stitch.hpp>
#include <utils/image_visualizer.hpp>
#include <utils/scoped_timer.hpp>

int main(int argc, char *argv[]) {
  std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
  cv::Mat pano;
  std::string input_path = argc > 1 ? argv[1] : "../data/berlin";
  std::string output_path = argc > 2 ? argv[2] : "../results/berlin.jpg";
  {
    ScopedTimer timer;
    is::stitch::Stitch stitch;
    stitch.load_images(input_path, false);
    pano = stitch.stitch();
  }
  is::vis::display(pano, "Panorama");
  cv::imwrite(output_path, pano);
}