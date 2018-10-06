#include <iostream>
#include "common.h"
#include "optical_flow.h"

int main(int argc, char**argv) {
  if (argc < 2) {
    std::cerr<< "USAGE: " << argv[0] << " <path_to_images>" << std::endl;
    return 0;
  }
  std::vector<std::string> names = open_dir(std::string(argv[1]));

  std::vector<cv::Mat> images = open_images(std::string(argv[1]), names);

  std::vector<cv::DMatch> matches;
  MatchOpticalFlowFeatures(images[0], images[1], matches);

  return 1;
}
