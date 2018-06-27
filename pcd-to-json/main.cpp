// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include "tinyxml2/tinyxml2.h"
#include "json/json.h"
#include <cmath>
#include <iostream> // std::cout
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <random>

int pcd_to_json(std::string pcd_file, double x, double y, double z, double qx, double qy, double qz, double qw) {
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
  root["device_position"]["x"] = x;
  root["device_position"]["y"] = y;
  root["device_position"]["z"] = z;
  root["device_heading"]["x"] = qx;
  root["device_heading"]["y"] = qy;
  root["device_heading"]["z"] = qz;
  root["device_heading"]["w"] = qw;

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    Json::Value point;
    point["x"] = cloud->points[i].x;
    point["y"] = cloud->points[i].y;
    point["z"] = cloud->points[i].z;
    point["i"] = cloud->points[i].intensity;

    if ((cloud->points[i].x != cloud->points[i].x) ||
        (cloud->points[i].y != cloud->points[i].y) ||
        (cloud->points[i].z != cloud->points[i].z) ||
        (cloud->points[i].intensity != cloud->points[i].intensity))
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
    std::cerr << "Enter a slam file to parse: ./pcd-to-json <file.pcd>"
              << std::endl;
    return 0;
  }

  std::string slam_file(argv[1]);
  size_t dir_found = slam_file.find_last_of("/\\");
  std::string directory = slam_file.substr(0, dir_found);

  tinyxml2::XMLDocument slam_xml;
  if (slam_xml.LoadFile(slam_file.c_str()) != 0) {
    std::cerr << "Failed to load " << argv[1] << std::endl;
    return 0;
  }

  // tinyxml2::XMLNode* slam_xml.FirstChild()

  tinyxml2::XMLElement *node_root =
      slam_xml.FirstChildElement("data"); //->FirstChildElement("node");
  for (tinyxml2::XMLElement *n = node_root->FirstChildElement("node");
       n != NULL; n = n->NextSiblingElement("node")) {
    double n_x = n->FirstChildElement("slam")->DoubleAttribute("x");
    double n_y = n->FirstChildElement("slam")->DoubleAttribute("y");
    double n_z = n->FirstChildElement("slam")->DoubleAttribute("z");
    double n_qx = n->FirstChildElement("slam")->DoubleAttribute("qx");
    double n_qy = n->FirstChildElement("slam")->DoubleAttribute("qy");
    double n_qz = n->FirstChildElement("slam")->DoubleAttribute("qz");
    double n_qw = n->FirstChildElement("slam")->DoubleAttribute("qw");
    std::string pcd_file(n->Attribute("cloud_filename"));
    pcd_to_json(directory + "/" + pcd_file, n_x, n_y, n_z, n_qx, n_qy, n_qz, n_qw);
  }
}
