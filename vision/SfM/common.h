# pragma once

#include <dirent.h>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::vector<std::string> open_dir(const std::string& dir_name);

std::vector<cv::Mat> open_images(const std::string& dir_name, const std::vector<std::string>& images_names, double downscale_factor = 1.0);

bool has_ending_lower (std::string full_string, const std::string& ending);

bool has_ending (const std::string& full_string, const std::string& ending);
