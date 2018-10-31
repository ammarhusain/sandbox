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

    std::cout << i << "  "
              << "P1_3TX " << P1_3TX << "  P2_3TX  " << P2_3TX << std::endl;

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

#ifndef MY_TRIANGULATE
  std::vector<cv::Point2f> pts1_u, pts2_u;
  cv::undistortPoints(pts1, pts1_u, K, distortion_coeff, cv::noArray(), K);
  cv::undistortPoints(pts2, pts2_u, K, distortion_coeff, cv::noArray(), K);

  cv::Mat pt_3d_h(4, pts1.size(), CV_32F);
  cv::triangulatePoints(P1, P2, pts1_u, pts2_u, pt_3d_h);

  // std::vector<cv::Vec4f> pt_3d_hv(pts1.size());
  // cv::triangulatePoints(P1, P2, pts1_u, pts2_u, pt_3d_hv);

  // calculate reprojection
  // Conversion lambda to go from homogeneous to euclidean.
  auto convert_homogeneous2euclidean = [](const cv::Mat& pt_3d_h) {
    assert(pt_3d_h.rows == 4 && pt_3d_h.cols >= 1);
    std::vector<cv::Point3f> pt_3dv;
    pt_3dv.reserve(pt_3d_h.cols);
    for (size_t c = 0; c < pt_3d_h.cols; ++c) {
      if (fabs(pt_3d_h.at<float>(3, c)) > 1e-2)
        pt_3dv.push_back(cv::Point3f(pt_3d_h.at<float>(0, c) / pt_3d_h.at<float>(3, c),
                                     pt_3d_h.at<float>(1, c) / pt_3d_h.at<float>(3, c),
                                     pt_3d_h.at<float>(2, c) / pt_3d_h.at<float>(3, c)));
      else
        pt_3dv.push_back(cv::Point3f(0, 0, 0));
    }
    return pt_3dv;
  };

  std::vector<cv::Point3f> pt_3dv = convert_homogeneous2euclidean(pt_3d_h);

  // shrink pts for debugging
  pt_3dv.resize(6);
  for (size_t i = 0; i < pt_3dv.size(); ++i)
    std::cout << "pts1 " << pts1[i] << " pts1_u " << pts1_u[i] << " pts2 " << pts2[i] << " pts2_u "
              << pts2_u[i] << " hp " << pt_3d_h.col(i).t() << " ep " << pt_3dv[i] << std::endl;

  cv::Mat_<double> R = (cv::Mat_<double>(3, 3) << P2(0, 0), P2(0, 1), P2(0, 2), P2(1, 0), P2(1, 1),
                        P2(1, 2), P2(2, 0), P2(2, 1), P2(2, 2));
  cv::Vec3d rvec;
  // std::cout << "R\n" << R << std::endl;
  cv::Rodrigues(R, rvec);
  cv::Vec3d tvec(P2(0, 3), P2(1, 3), P2(2, 3));
  // std::cout << "rvec " << rvec << "  tvec  " << tvec << std::endl;
  std::vector<cv::Point2f> reprojected_pt_set1;
  // cv::projectPoints(pt_3dv, rvec, tvec, K, distortion_coeff, reprojected_pt_set1);
  cv::Mat d_zero = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
  cv::projectPoints(pt_3dv, rvec, tvec, K, d_zero, reprojected_pt_set1);
  std::vector<double> reproj_error;
  for (unsigned int i = 0; i < pt_3dv.size(); i++) {
    // cv::CloudPoint cp;
    // cp.pt = pt_3d[i];
    // pointcloud.push_back(cp);
    std::cout << "3d_pt " << pt_3dv[i] << " pt " << pts1[i] << "  rpt " << reprojected_pt_set1[i]
              << std::endl;
    reproj_error.push_back(norm(pts2[i] - reprojected_pt_set1[i]));
  }
  return std::accumulate(reproj_error.begin(), reproj_error.end(), 0.0) / reproj_error.size();
#else
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
#endif
}
