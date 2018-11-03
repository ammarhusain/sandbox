#include "reconstruction.h"

Reconstruction::Reconstruction(std::vector<cv::Mat> images, cv::Mat K, cv::Mat distortion_coeff)
    : images_(std::move(images))
    , K_(K)
    , distortion_coeff_(distortion_coeff) {}

void Reconstruction::process() {
  std::cout << "K\n"
            << K_ << "\ndistortion_coeff\n"
            << distortion_coeff_ << "\nProcessing # images " << images_.size() << std::endl;

  int start_idx = 0;
  int end_idx   =  images_.size() - 1;
  /** ------------------------------------------------------ */
  // Go through images
  for (int img_idx = start_idx; img_idx < end_idx; ++img_idx) {
    std::vector<cv::DMatch> matches;

    // Check if keypoints have already been computed. If so extract & reuse.
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> l_kps_descriptors;
    auto kps = img_keypts_.find(img_idx);
    if (kps != img_keypts_.end()) {
      std::cout << "found  l kps  " << img_idx << std::endl;
      l_kps_descriptors = kps->second;
    } else {
      GetFeatures(images_[img_idx], l_kps_descriptors);
      img_keypts_.insert(std::make_pair(img_idx, l_kps_descriptors));
    }
    std::pair<std::vector<cv::KeyPoint>, cv::Mat> r_kps_descriptors;
    kps = img_keypts_.find(img_idx + 1);
    if (kps != img_keypts_.end()) {
      std::cout << "found  r kps  " << img_idx + 1 << std::endl;
      r_kps_descriptors = kps->second;
    } else {
      GetFeatures(images_[img_idx + 1], r_kps_descriptors);
      img_keypts_.insert(std::make_pair(img_idx+1, r_kps_descriptors));
    }

    //MatchRichFeatures(l_kps_descriptors, r_kps_descriptors, matches);
    MatchOpticalFlowFeatures(images_[img_idx], l_kps_descriptors, images_[img_idx + 1], r_kps_descriptors, matches);


    const std::vector<cv::KeyPoint> l_kps = l_kps_descriptors.first, r_kps = r_kps_descriptors.first;

    cv::Mat viz_img;
    std::vector<cv::Point2f> i_pts, j_pts;
    std::vector<cv::KeyPoint> l_good_a_kps, r_good_a_kps;
    cv::Mat F = EpipolarFeatureRefinement(l_kps, r_kps, matches, l_good_a_kps, r_good_a_kps);

    i_pts.clear();
    j_pts.clear();

    // cv::KeyPoint::convert(l_good_a_kps, i_pts);
    // cv::KeyPoint::convert(r_good_a_kps, j_pts);

    std::vector<cv::DMatch> refined_matches;
    // Filter based on displacement
    for (size_t i = 0; i < l_good_a_kps.size(); ++i) {
      double disp = std::sqrt(cv::norm(l_good_a_kps[i].pt - r_good_a_kps[i].pt));
      // if (disp > 2 && disp < 15) {
      i_pts.push_back(l_good_a_kps[i].pt);
      j_pts.push_back(r_good_a_kps[i].pt);
      refined_matches.push_back(cv::DMatch(i_pts.size() - 1, j_pts.size() - 1, 0.0));
      //}
    }

    correspondence_matrix_.insert(std::make_pair(
        std::make_pair(img_idx, img_idx + 1), std::make_tuple(refined_matches, i_pts, j_pts, F)));

    // Visualization code
    {
      images_[img_idx].copyTo(viz_img);
      std::vector<uchar> vstatus(l_kps.size(), 1);
      std::vector<float> verror(l_kps.size(), 1.0);
      draw_arrows(viz_img, i_pts, j_pts, vstatus, verror, cv::Scalar(0, 255, 0));
      std::stringstream ss;
      ss << "filtered_matches_" << i_pts.size() << ".png";
      cv::imshow(ss.str(), viz_img);
      // int c = cv::waitKey(0);
      // if (c == 's') {
      //   cv::imwrite(ss.str(), viz_img);
      // }
      // cv::destroyWindow(ss.str());
    }

    {
      //-- Draw only "good" matches: shuffle & draw a random 100
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(refined_matches.begin(), refined_matches.end(), g);
      refined_matches.resize(10);
      cv::drawMatches(images_[img_idx], l_good_a_kps, images_[img_idx + 1], r_good_a_kps,
                      refined_matches, viz_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                      std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
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
  }

  std::cout << "Computing Projection Matrices\n";

  // Add the identity matrix for the very first image to set the origin coordinates.
  img_P_mats_.insert(std::make_pair(start_idx, cv::Matx34d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0)));

  // Iterate over the correspondences datastructure to start processing
  for (auto crsp_itr = correspondence_matrix_.begin(); crsp_itr != correspondence_matrix_.end();
       ++crsp_itr) {
    // Extract data from the matches containers.
    cv::Mat F                      = std::get<3>(crsp_itr->second);
    int i_idx                      = std::get<0>(crsp_itr->first);
    int j_idx                      = std::get<1>(crsp_itr->first);
    std::vector<cv::Point2f> i_pts = std::get<1>(crsp_itr->second);
    std::vector<cv::Point2f> j_pts = std::get<2>(crsp_itr->second);
    cv::Matx34d P1                 = img_P_mats_[i_idx];

    // Split F to compute E, then R & T
    // Essential matrix: compute then extract cameras [R|t]
    cv::Mat_<double> E = K_.t() * F * K_;   // according to HZ (9.12

    cv::Mat_<double> R1, R2, t1, t2;

    DecomposeEtoRT(E, R1, R2, t1, t2);

    // Triangulation
    cv::Matx34d P2(R1(0, 0), R1(0, 1), R1(0, 2), t1(0), R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
                   R1(2, 0), R1(2, 1), R1(2, 2), t1(2));
    // debug printing
    std::cout << "P1\n" << P1 << std::endl;
    std::cout << "P2\n" << P2 << std::endl;

    double cr1, cr2;
    std::vector<cv::Point3d> triangulated_pts1, triangulated_pts2;
    std::vector<uchar> triangulation_status;
    cr1 = ComputeReprojectionError(P1, i_pts, P2, j_pts, K_, distortion_coeff_, triangulated_pts1);
    cr2 = ComputeReprojectionError(P2, j_pts, P1, i_pts, K_, distortion_coeff_, triangulated_pts2);

    std::vector<double> i_j_err;
    double s_i_j = 0;
    for (int i = 0; i < i_pts.size(); i++) {
      s_i_j += cv::norm(i_pts[i] - j_pts[i]);
      i_j_err.push_back(cv::norm(i_pts[i] - j_pts[i]));
    }
    std::cout << "i_j_err  " << std::accumulate(i_j_err.begin(), i_j_err.end(), 0.0) << "  s_i_j "
              << s_i_j << "  sz " << i_j_err.size() << std::endl;

    std::vector<double> tp1_tp2_err;
    for (size_t i = 0; i < triangulated_pts1.size(); ++i) {
      tp1_tp2_err.push_back(cv::norm(triangulated_pts1[i] - triangulated_pts2[i]));
    }
    std::cout << "error: " << cr1 << "   " << cr2 << "  "
              << std::accumulate(tp1_tp2_err.begin(), tp1_tp2_err.end(), 0) / tp1_tp2_err.size()
              << std::endl;

    if (!TestTriangulation(triangulated_pts2, P1, triangulation_status) ||
        !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
        cr2 > 100.0) {
      P2 = cv::Matx34d(R1(0, 0), R1(0, 1), R1(0, 2), t2(0), R1(1, 0), R1(1, 1), R1(1, 2), t2(1),
                       R1(2, 0), R1(2, 1), R1(2, 2), t2(2));
      cr1 =
          ComputeReprojectionError(P1, i_pts, P2, j_pts, K_, distortion_coeff_, triangulated_pts1);
      cr2 =
          ComputeReprojectionError(P2, j_pts, P1, i_pts, K_, distortion_coeff_, triangulated_pts2);
      for (size_t i = 0; i < triangulated_pts1.size(); ++i) {
        tp1_tp2_err.push_back(cv::norm(triangulated_pts1[i] - triangulated_pts2[i]));
      }
      std::cout << "error: " << cr1 << "   " << cr2 << "  "
                << std::accumulate(tp1_tp2_err.begin(), tp1_tp2_err.end(), 0) / tp1_tp2_err.size()
                << std::endl;
      if (!TestTriangulation(triangulated_pts2, P1, triangulation_status) ||
          !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
          cr2 > 100.0) {
        P2 = cv::Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t1(0), R2(1, 0), R2(1, 1), R2(1, 2), t1(1),
                         R2(2, 0), R2(2, 1), R2(2, 2), t1(2));
        cr1 = ComputeReprojectionError(P1, i_pts, P2, j_pts, K_, distortion_coeff_,
                                       triangulated_pts1);
        cr2 = ComputeReprojectionError(P2, j_pts, P1, i_pts, K_, distortion_coeff_,
                                       triangulated_pts2);
        for (size_t i = 0; i < triangulated_pts1.size(); ++i) {
          tp1_tp2_err.push_back(cv::norm(triangulated_pts1[i] - triangulated_pts2[i]));
        }
        std::cout << "error: " << cr1 << "   " << cr2 << "  "
                  << std::accumulate(tp1_tp2_err.begin(), tp1_tp2_err.end(), 0) / tp1_tp2_err.size()
                  << std::endl;
        if (!TestTriangulation(triangulated_pts2, P1, triangulation_status) ||
            !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
            cr2 > 100.0) {
          P2 = cv::Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t2(0), R2(1, 0), R2(1, 1), R2(1, 2), t2(1),
                           R2(2, 0), R2(2, 1), R2(2, 2), t2(2));
          cr1 = ComputeReprojectionError(P1, i_pts, P2, j_pts, K_, distortion_coeff_,
                                         triangulated_pts1);
          cr2 = ComputeReprojectionError(P2, j_pts, P1, i_pts, K_, distortion_coeff_,
                                         triangulated_pts2);
          for (size_t i = 0; i < triangulated_pts1.size(); ++i) {
            tp1_tp2_err.push_back(cv::norm(triangulated_pts1[i] - triangulated_pts2[i]));
          }
          std::cout << "error: " << cr1 << "   " << cr2 << "  "
                    << std::accumulate(tp1_tp2_err.begin(), tp1_tp2_err.end(), 0) /
                           tp1_tp2_err.size()
                    << std::endl;
          if (!TestTriangulation(triangulated_pts2, P1, triangulation_status) ||
              !TestTriangulation(triangulated_pts1, P2, triangulation_status) || cr1 > 100.0 ||
              cr2 > 100.0) {
            std::cout << "All 4 disambiguations failed to triangulate!!\n";
            // continue;
          }
        }
      }
    }
    // Store the computed second projection matrix
    img_P_mats_.insert(std::make_pair(j_idx, P2));

    // Quit for debugging
    std::string name;
    getline(std::cin, name);
  }
}
