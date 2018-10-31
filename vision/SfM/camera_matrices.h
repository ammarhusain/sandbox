#pragma once

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

void AlignPointsForMatch(const std::vector<cv::KeyPoint>& kps1,
                         const std::vector<cv::KeyPoint>& kps2,
                         const std::vector<cv::DMatch>& matches,
                         std::vector<cv::KeyPoint>& aligned_kps1,
                         std::vector<cv::KeyPoint>& aligned_kps2);

cv::Mat GetFundamentalMat(const std::vector<cv::KeyPoint>& imgpts1,
                          const std::vector<cv::KeyPoint>& imgpts2,
                          const std::vector<cv::DMatch>& matches,
                          std::vector<cv::KeyPoint>& aligned_kps1,
                          std::vector<cv::KeyPoint>& aligned_kps2, std::vector<uchar>& status);

cv::Mat EpipolarFeatureRefinement(const std::vector<cv::KeyPoint>& kps1,
                                  const std::vector<cv::KeyPoint>& kps2,
                                  const std::vector<cv::DMatch>& matches,
                                  std::vector<cv::KeyPoint>& good_kps1,
                                  std::vector<cv::KeyPoint>& good_kps2);


double ComputeReprojectionError(const cv::Matx34d& P1, const std::vector<cv::Point2f>& pts1, const cv::Matx34d& P2,
                                const std::vector<cv::Point2f>& pts2, const cv::Mat& K, const cv::Mat& distortion_coeff, std::vector<cv::Point3d>& triangulated_pts);

cv::Mat_<double> LinearLSTriangulation_AH(cv::Point3d kp1, cv::Matx34d P1, cv::Point3d kp2,
                                       cv::Matx34d P2, double w1 = 1.0, double w2 = 1.0);

// Implementation based on: https://users.cecs.anu.edu.au/~hartley/Papers/triangulation/triangulation.pdf
//!!  Verify that this implementation can work ... Need to step through code.
cv::Mat_<double> IterativeLinearLSTriangulation_AH(cv::Point3d kp1, cv::Matx34d P1, cv::Point3d kp2,
                                                cv::Matx34d P2);


bool DecomposeEtoRT(cv::Mat E, cv::Mat_<double>& R1, cv::Mat_<double>&  R2, cv::Mat_<double>& t1, cv::Mat_<double>& t2);

#define EPSILON 0.0001
#define MY_TRIANGULATE 0
cv::Mat_<double> LinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1,
                                       cv::Matx34d P1);

cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u, cv::Matx34d P, cv::Point3d u1,
                                                cv::Matx34d P1);

bool TestTriangulation(const std::vector<cv::Point3d>& triangulated_pts, const cv::Matx34d& P,
                       std::vector<uchar>& status);
