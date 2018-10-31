#include "triangulation.h"
#include "camera_matrices.h"
#include "common.h"
#include "optical_flow.h"
#include "rich_feature.h"
#include "reconstruction.h"
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "USAGE: " << argv[0] << " <path_to_images>" << std::endl;
    return 0;
  }
  std::vector<std::string> names = open_dir(std::string(argv[1]));
  std::sort(names.begin(), names.end());
  std::vector<cv::Mat> images = open_images(std::string(argv[1]), names);

  // Hardcode the calibration values for now
  cv::Matx34d proj_mat;
  proj_mat << 7.070493e+02, 0.000000e+00, 6.040814e+02, 0.000000e+00, 0.000000e+00, 7.070493e+02,
      1.805066e+02, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;
  cv::Mat K;
  K = (cv::Mat_<double>(3, 3) << 9.812178e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00,
       9.758994e+02, 2.471364e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
  cv::Mat distortion_coeff;
  distortion_coeff = (cv::Mat_<double>(5, 1) << -3.791375e-01, 2.148119e-01, 1.227094e-03,
                      2.343833e-03, -7.910379e-02);

  Reconstruction rcr(images, K, distortion_coeff);
  rcr.process();
  return 1;
}
