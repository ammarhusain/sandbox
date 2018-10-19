#pragma once

#include <vector>
#include <opencv2/video/video.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/gpu/gpu.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "common.h"

void MatchRichFeatures(const cv::Mat& left_img, const cv::Mat& right_img, std::vector<cv::DMatch>& matches, std::vector<cv::KeyPoint>& l_kps, std::vector<cv::KeyPoint>& r_kps);
