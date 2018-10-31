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
  status.clear();
  AlignPointsForMatch(kps1, kps2, matches, aligned_kps1, aligned_kps2);

  std::vector<cv::Point2f> pts1, pts2;
  cv::KeyPoint::convert(aligned_kps1, pts1);
  cv::KeyPoint::convert(aligned_kps2, pts2);

  return cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 1.0, 0.99,
                                status);   // threshold from [Snavely07 4.1]
  // double minVal, maxVal;
  // status.resize(kps1.size());
  // cv::minMaxIdx(pts1, &minVal, &maxVal);
  // return cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.006 * maxVal, 0.99,
  //                               status);   // threshold from [Snavely07 4.1]
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
  std::cout << "EF Refinement:: matches : " << matches.size() << " kps: " << kps1.size() << " to "
            << good_kps1.size() << " & " << kps2.size() << " to " << good_kps2.size() << std::endl;
  return F;
}


bool DecomposeEtoRT(cv::Mat E, cv::Mat_<double>& R1, cv::Mat_<double>& R2, cv::Mat_<double>& t1,
                    cv::Mat_<double>& t2) {
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
  t1 = svd.u.col(2);                   // u3
  t2 = -svd.u.col(2);                  // u3

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
