#pragma once

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "common.h"
#include <numeric>

// Macros
#define EPSILON 0.0001


double ComputeReprojectionError(const cv::Matx34d& P1, const std::vector<cv::Point2f>& pts1,
                                const int& idx1, const cv::Matx34d& P2,
                                const std::vector<cv::Point2f>& pts2, const int& idx2,
                                const cv::Mat& K, const cv::Mat& distortion_coeff,
                                std::vector<CloudPoint>& triangulated_pts);

cv::Mat_<double> LinearLSTriangulation(cv::Point3d kp1, cv::Matx34d P1, cv::Point3d kp2,
                                       cv::Matx34d P2, double w1 = 1.0, double w2 = 1.0);

//
/*!
 * @brief
 * Implementation based on: https://users.cecs.anu.edu.au/~hartley/Papers/triangulation/triangulation.pdf
 *
 * @param kp1
 * @param P1
 * @param kp2
 * @param P2
 *
 * @return
 */
cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d kp1, cv::Matx34d P1, cv::Point3d kp2,
                                                cv::Matx34d P2);

bool TestTriangulation(const std::vector<CloudPoint>& triangulated_cpts, const cv::Matx34d& P,
                       std::vector<uchar>& status);
