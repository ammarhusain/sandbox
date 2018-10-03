#include <iostream>
#include "common.h"

int main(int argc, char**argv) {
  if (argc < 2) {
    std::cerr<< "USAGE: " << argv[0] << " <path_to_images>" << std::endl;
    return 0;
  }
  std::vector<std::string> names = open_dir(std::string(argv[1]));
  std::cout << names.size() << std::endl;
  for (auto name : names) {
    std::cout << name << std::endl;
  }
  std::vector<cv::Mat> images = open_images(std::string(argv[1]), names);

  return 1;
}
