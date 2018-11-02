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
  cv::Mat K_;
  cv::Mat distortion_coeff_;
  // Storing the raw points in the correspondence datastructure. This is not terribly efficient, should be storing indices instead. Will refactor if it becomes a concern. Code simplicity for now.
  std::map<std::pair<int, int>, std::tuple<std::vector<cv::DMatch>, std::vector<cv::Point2f>, std::vector<cv::Point2f>, cv::Mat>> correspondence_matrix_;
  std::unordered_map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> img_keypts_;
  std::unordered_map<int,cv::Matx34d> img_P_mats_;
};
