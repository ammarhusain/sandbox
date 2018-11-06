#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <vector>
//#include <opencv2/gpu/gpu.hpp>
#include "common.h"
#include <opencv2/features2d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/highgui/highgui.hpp>

void MatchOpticalFlowFeatures(
    const cv::Mat& left_img, const std::pair<std::vector<cv::KeyPoint>, cv::Mat>& l_kps_descriptors,
    const cv::Mat& right_img,
    std::pair<std::vector<cv::KeyPoint>, cv::Mat>& r_kps_descriptors,
    std::vector<cv::DMatch>& matches);
