#include "triangulation.h"
#include "camera_matrices.h"
#include "common.h"
#include "optical_flow.h"
#include "rich_feature.h"
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "USAGE: " << argv[0] << " <path_to_images>" << std::endl;
    return 0;
  }
  std::vector<std::string> names = open_dir(std::string(argv[1]));
  std::sort(names.begin(), names.end());
  std::vector<cv::Mat> images = open_images(std::string(argv[1]), names);

  // Hardcode the calibration values for now
  cv::Matx34d proj_mat;
  proj_mat << 7.070493e+02, 0.000000e+00, 6.040814e+02, 0.000000e+00, 0.000000e+00, 7.070493e+02,
      1.805066e+02, 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00, 0.000000e+00;
  cv::Mat K;
  K = (cv::Mat_<double>(3, 3) << 9.812178e+02, 0.000000e+00, 6.900000e+02, 0.000000e+00,
       9.758994e+02, 2.471364e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
  cv::Mat distortion_coeff;
  distortion_coeff = (cv::Mat_<double>(5, 1) << -3.791375e-01, 2.148119e-01, 1.227094e-03,
                      2.343833e-03, -7.910379e-02);

  /** ------------------------------------------------------ */
  // Go through images
  for (int i = 0; i < images.size() - 1; ++i) {
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> l_kps, r_kps;
    MatchRichFeatures(images[i], images[i + 1], matches, l_kps, r_kps);
    // MatchOpticalFlowFeatures(images[i], images[i + 1], matches, l_kps, r_kps);

    cv::Mat viz_img;
    std::vector<cv::Point2f> i_pts, j_pts;

    {
      // draw original features in red
      std::vector<cv::KeyPoint> a_l_kps, a_r_kps;
      AlignPointsForMatch(l_kps, r_kps, matches, a_l_kps, a_r_kps);
      cv::KeyPoint::convert(a_l_kps, i_pts);
      cv::KeyPoint::convert(a_r_kps, j_pts);

      std::vector<uchar> vstatus(a_l_kps.size(), 1);
      std::vector<float> verror(a_l_kps.size(), 1.0);
      images[i].copyTo(viz_img);
      // draw_arrows(viz_img, i_pts, j_pts, vstatus, verror, cv::Scalar(0, 0, 255));
    }

    std::vector<cv::KeyPoint> l_good_kps, r_good_kps;
    cv::Mat F = EpipolarFeatureRefinement(l_kps, r_kps, matches, l_good_kps, r_good_kps);

    // superimpose filtered features in green
    // cv::KeyPoint::convert(l_good_kps, i_pts);
    // cv::KeyPoint::convert(r_good_kps, j_pts);

    i_pts.clear();
    j_pts.clear();
    std::vector<cv::DMatch> refined_matches;
    // Filter based on displacement
    for (size_t i = 0; i < l_good_kps.size(); ++i) {
      double disp = std::sqrt(cv::norm(l_good_kps[i].pt - r_good_kps[i].pt));
      // if (disp > 2 && disp < 15) {
      i_pts.push_back(l_good_kps[i].pt);
      j_pts.push_back(r_good_kps[i].pt);
      refined_matches.push_back(cv::DMatch(i_pts.size() - 1, j_pts.size() - 1, 0.0));
      //}
    }

    //

    {
      std::vector<uchar> vstatus(l_kps.size(), 1);
      std::vector<float> verror(l_kps.size(), 1.0);
      draw_arrows(viz_img, i_pts, j_pts, vstatus, verror, cv::Scalar(0, 255, 0));
      std::stringstream ss;
      ss << "filtered_matches_" << i_pts.size() << ".png";
      cv::imshow(ss.str(), viz_img);
      int c = cv::waitKey(0);
      if (c == 's') {
        cv::imwrite(ss.str(), viz_img);
      }
      cv::destroyWindow(ss.str());
    }

    {
      //-- Draw only "good" matches: shuffle & draw a random 100
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(refined_matches.begin(), refined_matches.end(), g);
      refined_matches.resize(10);
      cv::drawMatches(images[i], l_good_kps, images[i + 1], r_good_kps, refined_matches, viz_img,
                      cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      // std::shuffle(matches.begin(), matches.end(), g);
      // matches.resize(10);
      // cv::drawMatches(images[i], l_kps, images[i + 1], r_kps, matches, viz_img,
      //                 cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
      //                 cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      //-- Show detected matches
      std::stringstream ss;
      ss << "feature_matches.png";
      cv::imshow(ss.str(), viz_img);
      int c = cv::waitKey(0);
      if (c == 's') {
        cv::imwrite(ss.str(), viz_img);
      }
      cv::destroyWindow(ss.str());
    }

    // Triangulation
    // Split F to compute E, then R & T
    // Essential matrix: compute then extract cameras [R|t]
    cv::Mat_<double> E = K.t() * F * K;   // according to HZ (9.12

    cv::Mat_<double> R1, R2, t1, t2;

    DecomposeEtoRT(E, R1, R2, t1, t2);

    // shrink pts for debugging
    // i_pts.resize(6);
    // j_pts.resize(6);

    // Use opencv triangulation
    cv::Matx34d P1(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    cv::Matx34d P2(R1(0, 0), R1(0, 1), R1(0, 2), t1(0), R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
                   R1(2, 0), R1(2, 1), R1(2, 2), t1(2));
    double cr1, cr2;
    std::vector<cv::Point3d> triangulated_pts1, triangulated_pts2;
    std::vector<uchar> triangulation_status;
    cr1 = ComputeReprojectionError(P1, i_pts, P2, j_pts, K, distortion_coeff, triangulated_pts1);
    cr2 = ComputeReprojectionError(P2, j_pts, P1, i_pts, K, distortion_coeff, triangulated_pts2);
    std::cout << "error: " << cr1 << "   " << cr2 << std::endl;

    if (!TestTriangulation(triangulated_pts1, P2, triangulation_status) ||
        !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
        cr2 > 100.0) {
      P2 = cv::Matx34d(R1(0, 0), R1(0, 1), R1(0, 2), t2(0), R1(1, 0), R1(1, 1), R1(1, 2), t2(1),
                       R1(2, 0), R1(2, 1), R1(2, 2), t2(2));
      cr1 = ComputeReprojectionError(P1, i_pts, P2, j_pts, K, distortion_coeff, triangulated_pts1);
      cr2 = ComputeReprojectionError(P2, j_pts, P1, i_pts, K, distortion_coeff, triangulated_pts2);
      std::cout << "error: " << cr1 << "   " << cr2 << std::endl;
      if (!TestTriangulation(triangulated_pts1, P2, triangulation_status) ||
          !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
          cr2 > 100.0) {
        P2 = cv::Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t1(0), R2(1, 0), R2(1, 1), R2(1, 2), t1(1),
                         R2(2, 0), R2(2, 1), R2(2, 2), t1(2));
        cr1 =
            ComputeReprojectionError(P1, i_pts, P2, j_pts, K, distortion_coeff, triangulated_pts1);
        cr2 =
            ComputeReprojectionError(P2, j_pts, P1, i_pts, K, distortion_coeff, triangulated_pts2);
        std::cout << "error: " << cr1 << "   " << cr2 << std::endl;
        if (!TestTriangulation(triangulated_pts1, P2, triangulation_status) ||
            !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
            cr2 > 100.0) {
          P2 = cv::Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t2(0), R2(1, 0), R2(1, 1), R2(1, 2), t2(1),
                           R2(2, 0), R2(2, 1), R2(2, 2), t2(2));
          cr1 = ComputeReprojectionError(P1, i_pts, P2, j_pts, K, distortion_coeff,
                                         triangulated_pts1);
          cr2 = ComputeReprojectionError(P2, j_pts, P1, i_pts, K, distortion_coeff,
                                         triangulated_pts2);
          std::cout << "error: " << cr1 << "   " << cr2 << std::endl;
          if (!TestTriangulation(triangulated_pts1, P2, triangulation_status) ||
              !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
              cr2 > 100.0) {
            std::cout << "All 4 disambiguations failed to triangulate!!\n";
          }
        }
      }
    }
  }
  return 1;
}
