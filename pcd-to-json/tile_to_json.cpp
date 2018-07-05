// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include "tinyxml2/tinyxml2.h"
#include "json/json.h"
#include <cmath>
#include <iostream> // std::cout
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <random>

int pcd_to_json(std::string pcd_file) {
  size_t lastindex = pcd_file.find_last_of(".");
  std::string rawname = pcd_file.substr(0, lastindex);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud) ==
      -1) //* load the file
  {
    std::cout << "Couldn't load PCD file: " << pcd_file << std::endl;
    return (-1);
  }
  std::cout << "Loaded " << cloud->width * cloud->height << " data points from "
            << pcd_file << " with the following fields: " << std::endl;

  Json::Value root;
  /* TMP manipulation
  cloud->sensor_origin_.head<3>()(0) = 1;
  cloud->sensor_origin_.head<3>()(1) = 0;
  cloud->sensor_origin_.head<3>()(2) = 0;
  cloud->sensor_orientation_.x() = 0;
  cloud->sensor_orientation_.y() = 0;
  cloud->sensor_orientation_.z() = 0;
  cloud->sensor_orientation_.w() = 1;
  */

  root["device_position"]["x"] = cloud->sensor_origin_.head<3>()(0);
  root["device_position"]["y"] = cloud->sensor_origin_.head<3>()(1);
  root["device_position"]["z"] = cloud->sensor_origin_.head<3>()(2);
  root["device_heading"]["x"] = cloud->sensor_orientation_.x();
  root["device_heading"]["y"] = cloud->sensor_orientation_.y();
  root["device_heading"]["z"] = cloud->sensor_orientation_.z();
  root["device_heading"]["w"] = cloud->sensor_orientation_.w();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::transformPointCloud (*cloud,
                            *transformed_cloud,
                            cloud->sensor_origin_.head<3>(),
                            cloud->sensor_orientation_);


  for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
    Json::Value point;
    point["x"] = transformed_cloud->points[i].x;
    point["y"] = transformed_cloud->points[i].y;
    point["z"] = transformed_cloud->points[i].z;
    point["i"] = transformed_cloud->points[i].intensity;

    if ((transformed_cloud->points[i].x != transformed_cloud->points[i].x) ||
        (transformed_cloud->points[i].y != transformed_cloud->points[i].y) ||
        (transformed_cloud->points[i].z != transformed_cloud->points[i].z) ||
        (transformed_cloud->points[i].intensity != transformed_cloud->points[i].intensity))
      // if (isnan() || isnan() || isnan())
      continue;
    root["points"].append(point);
  }

  std::ofstream json_file;
  json_file.open(rawname + ".json");

  json_file << root;
  json_file << std::endl;
  return 1;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Enter a PCD file to parse: ./pcd-to-json <file.pcd>"
              << std::endl;
    return 0;
  }

  std::string pcd_file(argv[1]);
  size_t dir_found = pcd_file.find_last_of("/\\");
  std::string directory = pcd_file.substr(0, dir_found);

  pcd_to_json(pcd_file);

}
