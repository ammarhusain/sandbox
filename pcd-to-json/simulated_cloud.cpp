// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include "tinyxml2/tinyxml2.h"
#include "json/json.h"
#include <cmath>
#include <iostream> // std::cout
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <random>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>

int pcd_to_json(std::string file, double x, double y, double z, double qx,
                double qy, double qz, double qw) {
  Json::Value root;
  root["device_position"]["x"] = x;
  root["device_position"]["y"] = y;
  root["device_position"]["z"] = z;
  root["device_heading"]["x"] = qx;
  root["device_heading"]["y"] = qy;
  root["device_heading"]["z"] = qz;
  root["device_heading"]["w"] = qw;

  for (size_t i = 0; i < 1000; ++i) {
    Json::Value point;
    point["x"] = x + (i / 100.0);
    point["y"] = y + (i / 100.0);
    point["z"] = z + (i / 100.0);
    point["i"] = 0;
    root["points"].append(point);
  }

  std::ofstream json_file;
  json_file.open(file + ".json");
  json_file << root;
  json_file << std::endl;
  return 1;
}

int main(int argc, char **argv) {
  for (size_t i = 0; i < 100; i++) {
    std::ostringstream ss;
    ss << "cloud_" <<std::setw(5) << std::setfill('0') << i;
    pcd_to_json(ss.str(), i, i, 0, 0, 0, 0, 1);
  }
}
