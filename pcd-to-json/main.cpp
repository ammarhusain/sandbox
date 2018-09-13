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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int pcd_to_json(std::string pcd_file, double x, double y, double z, double qx, double qy, double qz, double qw, std::string img_name) {
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

   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
   transform_2.translation() << x, y, z;

  // The same rotation matrix as before; theta radians around Z axis
   transform_2.rotate (Eigen::Quaternionf (qw, qx, qy, qz));
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  //pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud = cloud;

  // // pcl::transformPointCloud(*cloud, *transformed_cloud,
  // //                          cloud->sensor_origin_.head<3>(),
  // //                          cloud->sensor_orientation_);
  pcl::transformPointCloud(*cloud, *transformed_cloud,
                           transform_2);

  for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
    Json::Value point;
    point["x"] = transformed_cloud->points[i].x;
    point["y"] = transformed_cloud->points[i].y;
    point["z"] = transformed_cloud->points[i].z;
    point["i"] = transformed_cloud->points[i].intensity;

    if ((transformed_cloud->points[i].x != transformed_cloud->points[i].x) ||
        (transformed_cloud->points[i].y != transformed_cloud->points[i].y) ||
        (transformed_cloud->points[i].z != transformed_cloud->points[i].z) ||
        (transformed_cloud->points[i].intensity !=
         transformed_cloud->points[i].intensity))
      // if (isnan() || isnan() || isnan())
      continue;
    root["points"].append(point);
  }

  // Write out the image files.
  Json::Value img_info;
  img_info["position"]["x"] = x;
  img_info["position"]["y"] = y;
  img_info["position"]["z"] = z;
  img_info["heading"]["x"] = qx;
  img_info["heading"]["y"] = qy;
  img_info["heading"]["z"] = qz;
  img_info["heading"]["w"] = qw;
  img_info["image_url"] = "https://s3-us-west-2.amazonaws.com/marble-image-labeling/scale_drivable_05_22_1/" + img_name;
  img_info["fx"] = 1;
  img_info["fy"] = 1;
  img_info["cx"] = 1;
  img_info["cy"] = 1;
  img_info["skew"] = 1;
  img_info["k1"] = 1;
  img_info["k2"] = 1;
  img_info["k3"] = 1;
  img_info["scale_factor"] = 1;
  root["images"].append(img_info);

  std::ofstream json_file;
  json_file.open(rawname + ".json");

  json_file << root;
  json_file << std::endl;
  return 1;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Enter a slam file to parse: ./pcd-to-json <file.slam> <rosbag.bag>"
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

  // Now open the bag file
  rosbag::Bag bag;
  bag.open(argv[2], rosbag::bagmode::Read);

  std::vector<std::string> topics;
  // topics.push_back(
  //     std::string("/perception_cam_forward_left/image_rect/compressed"));
  // topics.push_back(
  //     std::string("/perception_cam_forward_right/image_rect/compressed"));
  topics.push_back(std::string("/rs_front_forward/color/image_raw/compressed"));

  rosbag::View rs_view(bag, rosbag::TopicQuery(topics));

  rosbag::View::iterator rs_it = rs_view.begin();
  // for (; rs_it != rs_view.end(); rs_it++) {
  //     sensor_msgs::CompressedImage::Ptr rs_img=
  //     (*rs_it).instantiate<sensor_msgs::CompressedImage>();
  //     ros::Time tm = rs_img->header.stamp;
  //     std::cout << "rs: " << tm << std::endl;
  //     cv::Mat matrix = cv::imdecode(cv::Mat(rs_img->data),1);
  //     cv::imwrite("foo.jpeg", matrix);
  // }
  // rosbag::MessageInstance const m;
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
    std::string img_name;

    // Now find the relevant image to dump
    double n_t = n->DoubleAttribute("cloud_stamp");
    while (rs_it != rs_view.end()) {
      sensor_msgs::CompressedImage::Ptr rs_img=
      (*rs_it).instantiate<sensor_msgs::CompressedImage>();
      double tm = rs_img->header.stamp.toSec();
      if (tm - n_t > 0.1) { // Node is in the past
        // Figure out how to handle this
        rs_it = rs_view.begin();
        break;
        // rs_it--;
        // continue;
      } else if (n_t - tm > 0.1) { // Node is in the future
          rs_it++;
          continue;
        } else { // extract this img
      cv::Mat matrix = cv::imdecode(cv::Mat(rs_img->data),1);
       size_t lastindex = pcd_file.find_last_of(".");
       img_name = pcd_file.substr(0, lastindex) + "_rs.jpeg";

      cv::imwrite(directory + "/" + img_name, matrix);
      break;
      }
    }
    if (rs_it == rs_view.end()) {
      std::cout << "Unable to find img for " << pcd_file << std::endl;
      // Reset for next node to search again.
      rs_it = rs_view.begin();
    }

    pcd_to_json(directory + "/" + pcd_file, n_x, n_y, n_z, n_qx, n_qy, n_qz,
                n_qw, img_name);
  }
}
