
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stitch/stitch.hpp>
#include <utils/image_visualizer.hpp>
#include <utils/scoped_timer.hpp>

int main() {
  std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
  cv::Mat pano;
  {
    ScopedTimer timer;
    is::stitch::Stitch stitch;
    stitch.load_images("../data/berlin", false);
    pano = stitch.stitch();
  }
  is::vis::display(pano, "Panorama");
  cv::imwrite("../results/berlin.jpg", pano);
}