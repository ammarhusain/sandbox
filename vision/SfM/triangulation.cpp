#include "triangulation.h"

cv::Mat_<double> LinearLSTriangulation(cv::Point3d h_pt1, cv::Matx34d P1, cv::Point3d h_pt2,
                                       cv::Matx34d P2, double w1, double w2) {

  // build matrix A for homogenous equation system Ax = 0
  // assume X = (x,y,z,1), for Linear-LS method
  // which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
  cv::Mat eqs(4, 4, CV_64FC1, 0.0);
  // set the 4 equations.
  cv::Mat((1 / w1) * (h_pt1.x * P1.row(2).t() - P1.row(0).t()).t()).copyTo(eqs.row(0));
  cv::Mat((1 / w1) * (h_pt1.y * P1.row(2).t() - P1.row(1).t()).t()).copyTo(eqs.row(1));
  cv::Mat((1 / w2) * (h_pt2.x * P2.row(2).t() - P2.row(0).t()).t()).copyTo(eqs.row(2));
  cv::Mat((1 / w2) * (h_pt2.y * P2.row(2).t() - P2.row(1).t()).t()).copyTo(eqs.row(3));

  cv::Mat A(4,3, CV_64FC1);
  // A = eqs.colRange(0, 2);
  cv::Mat(eqs.col(0)).copyTo(A.col(0));
  cv::Mat(eqs.col(1)).copyTo(A.col(1));
  cv::Mat(eqs.col(2)).copyTo(A.col(2));

  cv::Mat B(4,1, CV_64FC1);
  cv::Mat(eqs.col(3)).copyTo(B.col(0));
  // Since the equation is Ax = B, the sign of B should be flipped to move it over to the other side.
  B = -1.0*B;
  cv::Mat_<double> X;
  cv::solve(A, B, X, cv::DECOMP_SVD);

  // cv::Mat Bz(4,1, CV_64FC1, 0.0);
  // cv::Mat_<double> Xz;
  // cv::solve(eqs, Bz, Xz, cv::DECOMP_LU);

  // std::cout << "XAhZ\n";
  // std::cout << Xz << std::endl;

  return X;
}

cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d h_pt1, cv::Matx34d P1, cv::Point3d h_pt2,
                                                cv::Matx34d P2) {
  double w1 = 1, w2 = 1;
  cv::Mat_<double> X(3, 1), HX(4,1);
      for (int i = 0; i < 10; ++i) {
        //for (int i = 0; i < 1; ++i) {
    X = LinearLSTriangulation(h_pt1, P1, h_pt2, P2, w1, w2);

    // Convert to homogeneous coordinates;
    HX << X(0), X(1), X(2), 1.0;

    double P1_3TX = P1.row(2).t().dot(HX);
    double P2_3TX = P2.row(2).t().dot(HX);

    // breaking point
    if (fabsf(w1 - P1_3TX) <= EPSILON && fabsf(w2 - P2_3TX) <= EPSILON)
      break;

    w1 = P1_3TX;
    w2 = P2_3TX;
  }
  return HX;
}

bool TestTriangulation(const std::vector<cv::Point3d>& triangulated_pts, const cv::Matx34d& P,
                       std::vector<uchar>& status) {
  std::vector<cv::Point3d> pcloud_pt3d_projected(triangulated_pts.size());

  cv::Matx44d P4x4 = cv::Matx44d::eye();
  for (int i    = 0; i < 12; i++)
    P4x4.val[i] = P.val[i];

  perspectiveTransform(triangulated_pts, pcloud_pt3d_projected, P4x4);

  status.resize(triangulated_pts.size(), 0);
  for (int i = 0; i < triangulated_pts.size(); i++) {
    status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
  }
  int count = cv::countNonZero(status);

  double percentage = ((double) count / (double) triangulated_pts.size());
  std::cout << count << "/" << triangulated_pts.size() << " = " << percentage * 100.0
            << "% are in front of camera" << std::endl;
  if (percentage < 0.75)
    return false;   // less than 75% of the points are in front of the camera
  return true;
}

double ComputeReprojectionError(const cv::Matx34d& P1, const std::vector<cv::Point2f>& pts1,
                                const cv::Matx34d& P2, const std::vector<cv::Point2f>& pts2,
                                const cv::Mat& K, const cv::Mat& distortion_coeff,
                                std::vector<cv::Point3d>& triangulated_pts) {
  // Clear the output constainer.
  triangulated_pts.clear();

  std::vector<double> reproj_error;
  cv::Mat_<double> KP2 = K * cv::Mat(P2);
  cv::Mat_<double> KP1 = K * cv::Mat(P1);
  cv::Mat Kinv = K.inv();
   for (int i = 0; i < pts1.size(); i++) {
  //for (int i = 0; i < 10; i++) {
    cv::Point2f pt1 = pts1[i];
    cv::Point3d pt1_h(pt1.x, pt1.y, 1.0);
    cv::Mat_<double> um = Kinv * cv::Mat_<double>(pt1_h);
    pt1_h.x             = um(0);
    pt1_h.y             = um(1);
    pt1_h.z             = um(2);

    cv::Point2f pt2 = pts2[i];
    cv::Point3d pt2_h(pt2.x, pt2.y, 1.0);
    um = Kinv * cv::Mat_<double>(pt2_h);
    pt2_h.x             = um(0);
    pt2_h.y             = um(1);
    pt2_h.z             = um(2);

    cv::Mat_<double> X = IterativeLinearLSTriangulation(pt1_h, P1, pt2_h, P2);

    cv::Mat_<double> x_pt2_h = KP2 * X;   // reproject
    cv::Point2f x_pt2(x_pt2_h(0) / x_pt2_h(2), x_pt2_h(1) / x_pt2_h(2));
    reproj_error.push_back(norm(x_pt2 - pt2));

    cv::Mat_<double> x_pt1_h = KP1 * X;   // reproject
    cv::Point2f x_pt1(x_pt1_h(0) / x_pt1_h(2), x_pt1_h(1) / x_pt1_h(2));

    // Try computing reprojection error with first image as well?

    triangulated_pts.push_back(cv::Point3d(X(0)/X(3), X(1)/X(3), X(2)/X(3)));
  }


  return std::accumulate(reproj_error.begin(), reproj_error.end(), 0.0) / reproj_error.size();
}
