// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include <iostream> // std::cout
#include <Eigen/Dense>
#include <random>

void compute_connected_components(
    const Eigen::MatrixXf &inp_mat, Eigen::MatrixXi &label_mat,
    float threshold) {

  label_mat.resize(inp_mat.rows(), inp_mat.cols());

  std::vector<size_t> labels;

  auto root = [&labels](size_t i) {
    while (i != labels[i]) {
      labels[i] = labels[labels[i]];
      i = labels[i];
    }
    return i;
  };
  auto union_label = [&labels, &root](size_t a, size_t b) {
    if ((a >= labels.size()) || (b >= labels.size())) {
      return false;
    }
    int ra = root(a);
    int rb = root(b);
    // Doesnt check for set sizes atm ... unions the second into first.
    labels[ra] = labels[rb];
    return true;
  };

  int block_sz = 3;
  auto get_neighbors = [&inp_mat, &block_sz](std::pair<int, int> c) {
    // Add the neighbors clockwise: l, tl, t, tr.
    std::vector<std::pair<int, int>> neighbors;
    for (int i = -1; i >= -block_sz; --i) {
      for (int j = -block_sz; j < block_sz; ++j) {
        if ((c.first + i >= 0) && (c.second + j >= 0) && (c.second + j < inp_mat.cols())){
            neighbors.push_back(std::make_pair(c.first + i, c.second + j));
          }
      }
    }

  return neighbors;

    if (c.second - 1 >= 0) {
      neighbors.push_back(std::make_pair(c.first, c.second - 1));
    }
    if (c.first - 1 >= 0) {
      if (c.second - 1 >= 0) {
        neighbors.push_back(std::make_pair(c.first - 1, c.second - 1));
      }

      neighbors.push_back(std::make_pair(c.first - 1, c.second));

      if (c.second + 1 < inp_mat.cols()) {
        neighbors.push_back(std::make_pair(c.first - 1, c.second + 1));
      }
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
          size_t id = labels.size();
          label_mat(rI, cI) = id;
          labels.push_back(id);
        }
      }
    }
  }

  // Second pass to fuse labels.
  for (int32_t rI = 0; rI < label_mat.rows(); ++rI) {
    for (int32_t cI = 0; cI < label_mat.cols(); ++cI) {
      if (label_mat(rI, cI) >= 0) {
        label_mat(rI, cI) = root(label_mat(rI, cI));
      }
    }
  }

  std::cout << "inp: \n" << inp_mat << std::endl;
  std::cout << "lab: \n" << label_mat << std::endl;
}

int main(int argc, char **argv) {

    std::cout << "Hello world!" << std::endl;
    {
      Eigen::MatrixXf inp(4, 4);

      inp << 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1;

      Eigen::MatrixXi lab;

      compute_connected_components(inp, lab, 0.5);
    }

    // create a large random matrix.
    // {
    //   std::default_random_engine generator;
    //   std::uniform_real_distribution<float> distribution(0.0, 1.0);
    //   Eigen::MatrixXf inp
    // }

    return 0;
}
