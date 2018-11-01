#pragma once
#include <vector>
#include <map>
#include "triangulation.h"
#include "camera_matrices.h"
#include "common.h"
#include "rich_feature.h"
#include "optical_flow.h"

class Reconstruction {
public:
  Reconstruction() = delete;
  explicit Reconstruction(std::vector<cv::Mat> images, cv::Mat K, cv::Mat distortion_coeff);

  void process();

private:
  std::vector<cv::Mat> images_;
  std::map<int,cv::Matx34d> P_mats_;
  cv::Mat K_;
  cv::Mat distortion_coeff_;
  std::map<std::pair<int, int>, std::tuple<std::vector<cv::DMatch>, cv::Mat>> correspondence_matrix_;
  std::unordered_map<int, std::vector<cv::Point2f>> img_pts_;
};
