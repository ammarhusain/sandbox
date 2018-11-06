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
 * @file   common.cpp
 * @brief
 *
 *
 */
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

void draw_arrows(cv::Mat& frame, const std::vector<cv::Point2f>& prev_pts,
                 const std::vector<cv::Point2f>& next_pts, const std::vector<uchar>& status,
                 const std::vector<float>& verror, const cv::Scalar& line_color_i) {
  double min_val, max_val;
  cv::minMaxIdx(verror, &min_val, &max_val, 0, 0, status);
  int line_thickness = 1;

  for (size_t i = 0; i < prev_pts.size(); ++i) {
    if (status[i]) {
      auto intrpmnmx = [](double val, double min, double max) {
        return (max == min ? 0.0 : ((val) -min) / (max - min));
      };

      double alpha = intrpmnmx(verror[i], min_val, max_val);
      alpha        = 1.0 - alpha;
      cv::Scalar line_color(alpha * line_color_i[0], alpha * line_color_i[1], alpha * line_color_i[2]);

      cv::Point p = prev_pts[i];
      cv::Point q     = next_pts[i];

      double angle = atan2((double) p.y - q.y, (double) p.x - q.x);

      double hypotenuse =
          sqrt((double) (p.y - q.y) * (p.y - q.y) + (double) (p.x - q.x) * (p.x - q.x));

      if (hypotenuse < 1.0)
        continue;

      // Here we lengthen the arrow by a factor of three.
      q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
      q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

      // Now we draw the main line of the arrow.
      cv::line(frame, p, q, line_color, line_thickness);

      // Now draw the tips of the arrow. I do some scaling so that the
      // tips look proportional to the main line of the arrow.

      p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
      p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
      cv::line(frame, p, q, line_color, line_thickness);

      p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
      p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
      cv::line(frame, p, q, line_color, line_thickness);
    }
  }
}

void populate_pcl_pointcloud(const std::vector<cv::Point3d>& ocv_pts, pcl::PointCloud<pcl::PointXYZRGB>& pcl_cloud) {
  pcl_cloud.clear();
  for (auto ocvp : ocv_pts) {
    pcl::PointXYZRGB pclp;
    pclp.x = ocvp.x;
    pclp.y = ocvp.y;
    pclp.z = ocvp.z;
    // pack r/g/b into rgb
    uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    pclp.rgb = *reinterpret_cast<float*>(&rgb);
    pcl_cloud.push_back(pclp);
  }
  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl::PCDWriter pw;
  pw.write("current_pointcloud.pcd",pcl_cloud);
}
