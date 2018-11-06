/**
 * Copyright (c) 2018 <Ammar Husain>
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file   common.h
 * @brief  Common utility functions for image processing.
 *
 *
 */
#pragma once

#include <algorithm>
#include <dirent.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

std::vector<std::string> open_dir(const std::string& dir_name);

std::vector<cv::Mat> open_images(const std::string& dir_name,
                                 const std::vector<std::string>& images_names,
                                 double downscale_factor = 1.0);

bool has_ending_lower(std::string full_string, const std::string& ending);

bool has_ending(const std::string& full_string, const std::string& ending);

void draw_arrows(cv::Mat& frame, const std::vector<cv::Point2f>& prev_pts,
                 const std::vector<cv::Point2f>& next_pts, const std::vector<uchar>& status,
                 const std::vector<float>& verror,
                 const cv::Scalar& line_color_i = cv::Scalar(0, 0, 255));

#define CV_PROFILE(msg, code)                                                                      \
  {                                                                                                \
    std::cout << msg << " ";                                                                       \
    double __time_in_ticks = (double) cv::getTickCount();                                          \
    { code }                                                                                       \
    std::cout << "DONE "                                                                           \
              << ((double) cv::getTickCount() - __time_in_ticks) / cv::getTickFrequency() << "s"   \
              << std::endl;                                                                        \
  }

void populate_pcl_pointcloud(const std::vector<cv::Point3d> ocv_pts, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud);
