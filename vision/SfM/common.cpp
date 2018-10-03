#include "common.h"

bool has_ending(std::string const& full_string, std::string const& ending) {
  if (full_string.length() >= ending.length()) {
    return (0 ==
            full_string.compare(full_string.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

bool has_ending_lower(std::string full_string, const std::string& ending) {
  std::transform(full_string.begin(), full_string.end(), full_string.begin(),
                 [](unsigned char c) { return std::tolower(c); });   // to lower
  return has_ending(full_string, ending);
}

std::vector<std::string> open_dir(const std::string& dir_name) {
  std::vector<std::string> names;
  DIR* dp;
  struct dirent* ep;
  dp = opendir(dir_name.c_str());

  if (dp != NULL) {
    while ((ep = readdir(dp))) {
      if (ep->d_name[0] != '.')
        names.push_back(ep->d_name);
    }

    (void) closedir(dp);
  } else {
    std::cerr << "Couldn't open the directory" << std::endl;
  }
  return names;
}

std::vector<cv::Mat> open_images(const std::string& dir_name,
                               const std::vector<std::string>& images_names,
                               double downscale_factor) {
  std::vector<cv::Mat> images;
  for (unsigned int i = 0; i < images_names.size(); i++) {
    if (images_names[i][0] == '.' ||
        !(has_ending_lower(images_names[i], "jpg") || has_ending_lower(images_names[i], "png"))) {
      continue;
    }
    cv::Mat m = cv::imread(std::string(dir_name).append("/").append(images_names[i]));
    if (downscale_factor != 1.0)
      cv::resize(m, m, cv::Size(), downscale_factor, downscale_factor);
    images.push_back(m);
  }
  return images;
}
