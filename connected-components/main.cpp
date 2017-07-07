// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include <iostream> // std::cout
#include <Eigen/Dense>

void compute_connected_components(
    const Eigen::MatrixXf &inp_mat, Eigen::MatrixXi &label_mat,
    float threshold) {

  std::vector<size_t> labels;

  auto union_label = [&labels](size_t a, size_t b) { std::cout << "un: " << a << " .. " << b << std::endl;return 0; };

  auto get_neighbors = [&inp_mat](std::pair<int, int> c) {
    std::vector<std::pair<int, int>> neighbors;
    if (c.first - 1 > 0) {
      neighbors.push_back(std::make_pair(c.first - 1, c.second));
      if (c.second - 1 > 0) {
        neighbors.push_back(std::make_pair(c.first - 1, c.second - 1));
      }
      if (c.second + 1 < inp_mat.cols()) {
        neighbors.push_back(std::make_pair(c.first - 1, c.second + 1));
      }
    }
    if (c.second - 1 > 0) {
      neighbors.push_back(std::make_pair(c.first, c.second - 1));
    }
    return neighbors;
  };

  for (int32_t rI = 0; rI < inp_mat.rows(); ++rI) {
    for (int32_t cI = 0; cI < inp_mat.cols(); ++cI) {
      // initialize label
      label_mat(rI, cI) = -1;
      if (inp_mat(rI, cI) > threshold) {
        std::vector<std::pair<int, int>> neighbors =
            get_neighbors(std::make_pair(rI, cI));
        for (auto n : neighbors) {
          if (label_mat(n.first, n.second) >= 0) {
                      std::cout << "me: " << rI << "," << cI << std::endl;
                                std::cout << "ng: " << n.first << "," << n.second << std::endl;
            // Either add as label or make equivalence relation.
            if (label_mat(rI, cI) == -1) {
              label_mat(rI, cI) = label_mat(n.first, n.second);
            } else {
              union_label(label_mat(rI, cI), label_mat(n.first, n.second));
            }
          }
        }
        // Add label if still not been assigned.
        if (label_mat(rI, cI) == -1) {
          std::cout << "nl: " << rI << "," << cI << std::endl;

          size_t id = labels.size();
          label_mat(rI, cI) = id;
          labels.push_back(id);
        }
      }
    }
  }

  std::cout << "inp: \n" << inp_mat << std::endl;
  std::cout << "lab: \n" << label_mat << std::endl;
}
int main(int argc, char **argv) {

  std::cout << "Hello world!" << std::endl;

  Eigen::MatrixXf inp(4,4);

  inp << 0, 0, 1, 0,
    1, 1, 0, 0,
    0, 0, 0, 0,
    1,1,1,1;

Eigen::MatrixXi lab(4,4);

 compute_connected_components(inp, lab, 0.5);
  return 0;
}
