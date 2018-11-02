#include "reconstruction.h"

Reconstruction::Reconstruction(std::vector<cv::Mat> images, cv::Mat K, cv::Mat distortion_coeff)
    : images_(std::move(images))
    , K_(K)
    , distortion_coeff_(distortion_coeff) {}

void Reconstruction::process() {
  std::cout << "K\n" << K_ << "\ndistortion_coeff\n" << distortion_coeff_ << "\nProcessing # images " << images_.size() << std::endl;

  int start_idx = 4;
  int end_idx = 10; // images_.size() - 1
    /** ------------------------------------------------------ */
  // Go through images
  for (int img_idx = start_idx; img_idx < end_idx; ++img_idx) {
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> l_kps, r_kps;
    MatchRichFeatures(images_[img_idx], images_[img_idx + 1], matches, l_kps, r_kps);
    // MatchOpticalFlowFeatures(images_[i], images_[i + 1], matches, l_kps, r_kps);

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

    correspondence_matrix_.insert(std::make_pair(std::make_pair(img_idx, img_idx+1), std::make_tuple(refined_matches, F)));
    img_pts_.insert(std::make_pair(img_idx, i_pts));
    img_pts_.insert(std::make_pair(img_idx+1, j_pts));


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
      cv::drawMatches(images_[img_idx], l_good_a_kps, images_[img_idx + 1], r_good_a_kps, refined_matches, viz_img,
                      cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
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
  std::vector<cv::Point2f> prev_j_pts;
  // Iterate over the correspondences datastructure to start processing
  for (auto crsp_itr = correspondence_matrix_.begin(); crsp_itr != correspondence_matrix_.end(); ++crsp_itr) {
    // Extract data from the matches containers.
    cv::Mat F = std::get<1>(crsp_itr->second);
    int i_idx = std::get<0>(crsp_itr->first);
    int j_idx = std::get<1>(crsp_itr->first);
    std::vector<cv::Point2f> i_pts = img_pts_[i_idx];
    std::vector<cv::Point2f> j_pts = img_pts_[j_idx];
    cv::Matx34d P1                 = img_P_mats_[i_idx];
    std::cout << "  i_idx " << i_idx << "  j_idx  " << j_idx << std::endl;
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
      std::cout << i_pts[i] << " " << j_pts[i] << " " <<cv::norm(i_pts[i] - j_pts[i]) << "  ";
    }
    std::cout <<std::endl;
    std::cout << "i_j_err  " << std::accumulate(i_j_err.begin(), i_j_err.end(), 0.0) << "  s_i_j " << s_i_j << "  sz " << i_j_err.size()
              << std::endl;

    // check if prev_j has crap... should be same as i_pts now
    for (int i = 0; i < prev_j_pts.size(); ++i) {
      if(prev_j_pts[i] != i_pts[i] )
        std::cout << "yell" << std::endl;
    }
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
          }
        }
      }
    }
    // store prev
    prev_j_pts = j_pts;
    // Store the computed second projection matrix
    img_P_mats_.insert(std::make_pair(j_idx, P2));

    // Quit for debugging
    std::string name;
    getline(std::cin, name);
  }
}
