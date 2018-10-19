// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include <Eigen/Dense>
#include <iostream> // std::cout
#include <math.h>
#include <random>

/// Shifts a matrix/vector row-wise.
/// A negative \a down value is taken to mean shifting up.
/// When passed zero for \a down, the input matrix is returned unchanged.
/// The type \a M can be either a fixed- or dynamically-sized matrix.
template <typename M> M shiftedByRows(const M &in, int down) {
  if (!down)
    return in;
  M out(in.rows(), in.cols());
  if (down > 0)
    down = down % in.rows();
  else
    down = in.rows() - (-down % in.rows());
  // We avoid the implementation-defined sign of modulus with negative arg.
  int rest = in.rows() - down;
  out.topRows(down) = in.bottomRows(down);
  out.bottomRows(rest) = in.topRows(rest);
  return out;
}

/// Shifts a matrix/vector row-wise.
/// A negative \a down value is taken to mean shifting up.
/// When passed zero for \a down, the input matrix is returned unchanged.
/// The type \a M can be either a fixed- or dynamically-sized matrix.
template <typename M> M shiftedByCols(const M &in, int right) {
  if (!right)
    return in;
  M out(in.rows(), in.cols());
  if (right > 0)
    right = right % in.cols();
  else
    right = in.cols() - (-right % in.cols());
  // We avoid the implementation-defined sign of modulus with negative arg.
  int rest = in.cols() - right;
  out.leftCols(right) = in.rightCols(right);
  out.rightCols(rest) = in.leftCols(rest);
  return out;
}

Eigen::MatrixXf block_shift_eigen_matrix(const Eigen::MatrixXf &inp_mat,
                                         Eigen::Vector2i idx) {
  // return shiftedByRows(inp_mat, idx(0));
  return shiftedByCols(shiftedByRows(inp_mat, idx(0)), idx(1));
}

int main(int argc, char **argv) {
  std::cout << "Hello world!" << std::endl;
  Eigen::MatrixXf inp(4, 4);

  inp << 0, 0, 2, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 5, 1, 1;

  Eigen::Vector2i lab(-1,-16);

  Eigen::MatrixXf out = block_shift_eigen_matrix(inp, lab);

  std::cout << "inp:\n" << inp << std::endl;
  std::cout << "out:\n" << out << std::endl;

  return 0;
}
