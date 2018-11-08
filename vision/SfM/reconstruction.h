#pragma once
#include "camera_matrices.h"
#include "common.h"
#include "optical_flow.h"
#include "rich_feature.h"
#include "triangulation.h"
#include <map>
#include <vector>


class Reconstruction {
 public:
  Reconstruction() = delete;
  explicit Reconstruction(std::vector<cv::Mat> images, cv::Mat K, cv::Mat distortion_coeff);

  void process();

 private:
  std::vector<cv::Mat> images_;
  cv::Mat K_;
  cv::Mat distortion_coeff_;
  // Storing the raw points in the correspondence datastructure. This is not terribly efficient,
  // should be storing indices instead. Will refactor if it becomes a concern. Code simplicity for
  // now.

  struct ScrollingCmp {
    bool operator()(const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) const {
      if (rhs.second > lhs.second) {
        return true;
      } else if (rhs.second == lhs.second) {
        return rhs.first < lhs.first;
      } else
        return false;
    }
  };

  std::map<std::pair<int, int>, std::tuple<std::vector<cv::DMatch>, std::vector<cv::Point2f>,
                                           std::vector<cv::Point2f>, cv::Mat>,
           ScrollingCmp>
      correspondence_matrix_;
  int scroll_window_ = 10;
  std::unordered_map<int, std::pair<std::vector<cv::KeyPoint>, cv::Mat>> img_keypts_;
  std::unordered_map<int, cv::Matx34d> img_P_mats_;
};
