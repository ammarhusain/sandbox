// Copyright (C) 2018 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include <iostream>
#include <Eigen/Dense>
#include <random>

class State {

  Eigen::Matrix4d control_update;
  Eigen::Vector4d control_input;
  // control commands: heading & acceleration
  double accel, heading;
  Eigen::Matrix4d state_update;
  // x, y, v_x, v_y
  Eigen::Vector4d state;
};

class Filter {

};

int main(int argc, char **argv) {

    // create a large random matrix.
    // {
    //   std::default_random_engine generator;
    //   std::uniform_real_distribution<float> distribution(0.0, 1.0);
    //   Eigen::MatrixXf inp
    // }

    return 0;
}
