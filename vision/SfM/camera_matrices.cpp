#include "camera_matrices.h"

void AlignPointsForMatch(const std::vector<cv::KeyPoint>& kps1,
                         const std::vector<cv::KeyPoint>& kps2,
                         const std::vector<cv::DMatch>& matches,
                         std::vector<cv::KeyPoint>& aligned_kps1,
                         std::vector<cv::KeyPoint>& aligned_kps2) {
  for (auto match : matches) {
    assert(match.queryIdx < kps1.size());
    aligned_kps1.push_back(kps1[match.queryIdx]);
    assert(match.trainIdx < kps2.size());
    aligned_kps2.push_back(kps2[match.trainIdx]);
  }
}

cv::Mat GetFundamentalMat(const std::vector<cv::KeyPoint>& kps1,
                          const std::vector<cv::KeyPoint>& kps2,
                          const std::vector<cv::DMatch>& matches,
                          std::vector<cv::KeyPoint>& aligned_kps1,
                          std::vector<cv::KeyPoint>& aligned_kps2, std::vector<uchar>& status) {

  aligned_kps1.clear();
  aligned_kps2.clear();
  AlignPointsForMatch(kps1, kps2, matches, aligned_kps1, aligned_kps2);

  std::vector<cv::Point2f> pts1, pts2;
  cv::KeyPoint::convert(aligned_kps1, pts1);
  cv::KeyPoint::convert(aligned_kps2, pts2);

  double minVal, maxVal;
  status.resize(kps1.size());
  cv::minMaxIdx(pts1, &minVal, &maxVal);
  return cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.006 * maxVal, 0.99,
                                status);   // threshold from [Snavely07 4.1]
}

cv::Mat EpipolarFeatureRefinement(const std::vector<cv::KeyPoint>& kps1,
                                  const std::vector<cv::KeyPoint>& kps2,
                                  const std::vector<cv::DMatch>& matches,
                                  std::vector<cv::KeyPoint>& good_kps1,
                                  std::vector<cv::KeyPoint>& good_kps2) {
  std::vector<uchar> status;
  std::vector<cv::KeyPoint> aligned_kps1, aligned_kps2;
  cv::Mat F = GetFundamentalMat(kps1, kps2, matches, aligned_kps1, aligned_kps2, status);

  good_kps1.clear();
  good_kps2.clear();
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      good_kps1.push_back(aligned_kps1[i]);
      good_kps2.push_back(aligned_kps2[i]);
    }
  }
  std::cout << good_kps1.size() << " good keypoints out of " << kps1.size() << " total keypoints." << std::endl;
  return F;
}

double ComputeReprojectionError(const cv::Matx34d& P1, const std::vector<cv::Point2f>& pts1, const cv::Matx34d& P2,
                                const std::vector<cv::Point2f>& pts2, const cv::Mat& K, const cv::Mat& distortion_coeff) {

  cv::Mat pt_3d_h(1, pts1.size(),CV_32FC4);
    cv::triangulatePoints(P1, P2, pts1, pts2, pt_3d_h);

    // calculate reprojection
    std::vector<cv::Point3f> pt_3d;
    convertPointsHomogeneous(pt_3d_h.reshape(4, 1), pt_3d);
    // cv::Mat_<double> R = (cv::Mat_<double>(3, 3) << P1(0, 0), P1(0, 1), P1(0, 2), P1(1, 0),
    //                       P1(1, 1), P1(1, 2), P1(2, 0), P1(2, 1), P1(2, 2));
    // cv::Vec3d rvec;
    // cv::Rodrigues(R, rvec);
    // cv::Vec3d tvec(P1(0, 3), P1(1, 3), P1(2, 3));
    // std::cout << "rvec " << rvec << "  tvec  " << tvec << std::endl;
    // std::vector<cv::Point2f> reprojected_pt_set1;
    // projectPoints(pt_3d, rvec, tvec, K, distortion_coeff, reprojected_pt_set1);
    // std::vector<double> reproj_error;
    // for (unsigned int i = 0; i < pts1.size(); i++) {
    //   // cv::CloudPoint cp;
    //   // cp.pt = pt_3d[i];
    //   // pointcloud.push_back(cp);
    //   //std::cout << "pt " << pts1[i] << "  rpt " << reprojected_pt_set1[i] << std::endl;
    //   reproj_error.push_back(norm(pts1[i] - reprojected_pt_set1[i]));
    // }

    cv::Mat_<double> R = (cv::Mat_<double>(3, 3) << P2(0, 0), P2(0, 1), P2(0, 2), P2(1, 0),
                          P2(1, 1), P2(1, 2), P2(2, 0), P2(2, 1), P2(2, 2));
    cv::Vec3d rvec;
    cv::Rodrigues(R, rvec);
    cv::Vec3d tvec(P2(0, 3), P2(1, 3), P2(2, 3));
    std::cout << "rvec " << rvec << "  tvec  " << tvec << std::endl;
    std::vector<cv::Point2f> reprojected_pt_set1;
    projectPoints(pt_3d, rvec, tvec, K, distortion_coeff, reprojected_pt_set1);
    std::vector<double> reproj_error;
    for (unsigned int i = 0; i < pts2.size(); i++) {
      // cv::CloudPoint cp;
      // cp.pt = pt_3d[i];
      // pointcloud.push_back(cp);
      //std::cout << "pt " << pts1[i] << "  rpt " << reprojected_pt_set1[i] << std::endl;
      reproj_error.push_back(norm(pts2[i] - reprojected_pt_set1[i]));
    }

    //std::cout << "rps " << reproj_error.size() << std::endl;
    return std::accumulate(reproj_error.begin(), reproj_error.end(), 0.0) / reproj_error.size();
}

/**
 */
/*!
 * @brief  From "Linear Triangulation Methods (12.2)", Hartley & Zisserman
 *
 * @param kp1 homogenous image point (kp1,v,1)
 * @param P1 camera 1 matrix
 * @param kp2 homogenous image point in 2nd camera
 * @param P2 camera 2 matrix
 *
 * @return
 */
cv::Mat_<double> LinearLSTriangulation(cv::Point3d kp1, cv::Matx34d P1, cv::Point3d kp2,
                                       cv::Matx34d P2, double w1, double w2) {

  // build matrix A for homogenous equation system Ax = 0
  // assume X = (x,y,z,1), for Linear-LS method
  // which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1

  cv::Mat tmp(((kp1.x * P1.row(2).t() - P1.row(0).t()).t()).t());
  cv::Mat A(4, 4, CV_64FC1, 0.0);
  // set the 4 equations.
  // A.row(0) << 1,2,3,4;
  // A.row(1) =  cv::Mat_<double>(1, 4) << ((kp1.x*P1.row(2).t() - P1.row(0).t()).t()).t();

  cv::Mat((1 / w1) * (kp1.x * P1.row(2).t() - P1.row(0).t()).t()).copyTo(A.row(0));
  cv::Mat((1 / w1) * (kp1.y * P1.row(2).t() - P1.row(1).t()).t()).copyTo(A.row(1));
  cv::Mat((1 / w2) * (kp2.x * P1.row(2).t() - P1.row(0).t()).t()).copyTo(A.row(2));
  cv::Mat((1 / w2) * (kp2.y * P1.row(2).t() - P1.row(1).t()).t()).copyTo(A.row(3));

  cv::Matx41d B(0.0);
  cv::Mat_<double> X;
  cv::solve(A, B, X, cv::DECOMP_SVD);

  std::cout << "A\n";
  std::cout << A << std::endl;
  std::cout << "B\n";
  std::cout << B << std::endl;
  std::cout << "X\n";
  std::cout << X << std::endl;


  // cv::Matx43d Ab(kp1.x * P1(2, 0) - P1(0, 0), kp1.x * P1(2, 1) - P1(0, 1), kp1.x * P1(2, 2) - P1(0, 2),
  //           kp1.y * P1(2, 0) - P1(1, 0), kp1.y * P1(2, 1) - P1(1, 1), kp1.y * P1(2, 2) - P1(1, 2),
  //           kp2.x * P2(2, 0) - P2(0, 0), kp2.x * P2(2, 1) - P2(0, 1), kp2.x * P2(2, 2) - P2(0, 2),
  //           kp2.y * P2(2, 0) - P2(1, 0), kp2.y * P2(2, 1) - P2(1, 1), kp2.y * P2(2, 2) - P2(1, 2));
  // cv::Matx41d Bb(-(kp1.x * P1(2, 3) - P1(0, 3)), -(kp1.y * P1(2, 3) - P1(1, 3)),
  //           -(kp2.x * P2(2, 3) - P2(0, 3)), -(kp2.y * P2(2, 3) - P2(1, 3)));

  // cv::Mat_<double> Xb;
  // cv::solve(Ab, Bb, Xb, cv::DECOMP_SVD);

  // std::cout << "Ab\n";
  // std::cout << Ab << std::endl;
  // std::cout << "Bb\n";
  // std::cout << Bb << std::endl;
  // std::cout << "Xb\n";
  // std::cout << Xb << std::endl;

  return X;
}

cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d kp1, cv::Matx34d P1, cv::Point3d kp2,
                                       cv::Matx34d P2) {
  double w1 = 1, w2 = 1;
  cv::Mat_<double> X(4,1);
  for (int i = 0; i < 10; ++i) {
    X = LinearLSTriangulation(kp1, P1, kp2, P2, w1, w2);
    // Renormalize X?
    X(3) = 1.0;

    std::cout << i << "  " << "P1_3TX " << P1.row(2) << "  P2_3TX  " << X << std::endl;

    double P1_3TX = P1.row(2).t().dot(X);
    double P2_3TX = P2.row(2).t().dot(X);

    std::cout << i << "  " << "P1_3TX " << P1_3TX << "  P2_3TX  " << P2_3TX << std::endl;

    // if (fabsf(w1 - P1_3TX) <= 0.0001 && fabsf(w2 - P2_3TX) <= 0.0001) break;

    w1 = P1_3TX;
    w2 = P2_3TX;
  }
  return X;
}

bool DecomposeEtoRT(cv::Mat E, cv::Mat_<double>& R1, cv::Mat_<double>&  R2, cv::Mat_<double>& t1, cv::Mat_<double>& t2) {
    // Decompose E to R & T
    // Using OpenCV's SVD
    cv::SVD svd(E, cv::SVD::MODIFY_A);

    // check if first and second singular values are the same (as they should be)
    double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1));
    if (singular_values_ratio > 1.0)
      singular_values_ratio = 1.0 / singular_values_ratio;   // flip ratio to keep it [0,1]
    if (singular_values_ratio < 0.7) {
      std::cout << "singular values are too far apart\n";
      return false;
    }

    cv::Matx33d W(0, -1, 0,   // HZ 9.13
                  1, 0, 0, 0, 0, 1);
    cv::Matx33d Wt(0, 1, 0, -1, 0, 0, 0, 0, 1);
    R1 = svd.u * cv::Mat(W) * svd.vt;    // HZ 9.19
    R2 = svd.u * cv::Mat(Wt) * svd.vt;   // HZ 9.19
    t1 = svd.u.col(2);               // u3
    t2 = -svd.u.col(2);              // u3

    std::cout << "R1\n" << R1 << std::endl;
    std::cout << "R2\n" << R2 << std::endl;
    std::cout << "t1\n" << t1 << std::endl;
    std::cout << "t2\n" << t2 << std::endl;

    auto check_coherent_rotation = [](const cv::Mat_<double>& R) {
      if (fabsf(determinant(R)) - 1.0 > 1e-07) {
        std::cerr << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
        return false;
      }
      return true;
    };


    return check_coherent_rotation(R1) && check_coherent_rotation(R2);
}
