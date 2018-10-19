#include "rich_feature.h"
#include "camera_matrices.h"

void MatchRichFeatures(const cv::Mat& left_img, const cv::Mat& right_img, std::vector<cv::DMatch>& matches, std::vector<cv::KeyPoint>& l_kps, std::vector<cv::KeyPoint>& r_kps) {
  // clear output variable
  matches.clear(); l_kps.clear(); r_kps.clear();

  cv::Mat left_desc, right_desc;
  // cv::Ptr<cv::FeatureDetector> detector  = cv::FastFeatureDetector::create();
  // cv::Ptr<cv::FeatureDetector> extractor = cv::ORB::create();

  cv::Ptr<cv::FeatureDetector> detector  = cv::GFTTDetector::create();
  cv::Ptr<cv::FeatureDetector> extractor = cv::ORB::create();


  CV_PROFILE("FeatureDetection", detector->detect(left_img, l_kps);
             extractor->compute(left_img, l_kps, left_desc);
             detector->detect(right_img, r_kps);
             extractor->compute(right_img, r_kps, right_desc););
  std::cout << "l_kps has " << l_kps.size() << " points (descriptors " << left_desc.rows
            << ")" << std::endl;
  std::cout << "r_kps has " << r_kps.size() << " points (descriptors " << right_desc.rows
            << ")" << std::endl;

  if (left_desc.empty()) {
    CV_Error(0, "left_desc is empty");
  }
  if (right_desc.empty()) {
    CV_Error(0, "right_desc is empty");
  }

  // matching descriptor vectors using Brute Force matcher
  cv::BFMatcher matcher(
      cv::NORM_HAMMING,
      true);   // allow cross-check. use Hamming distance for binary descriptor (ORB)

  // NOTE: Get this radial matching working
  // NOTE: Make sure your train & query indices are not flipped in ther matcher
  // matcher.radiusMatch(InputArray queryDescriptors, InputArray trainDescriptors, std::vector<std::vector<DMatch> > &matches, float maxDistance)
  // matcher.radiusMatch(left_desc, right_desc, matches, 20);
  matcher.match(left_desc, right_desc, matches);
  assert(matches.size() > 0);

  std::vector<cv::DMatch> filt_matches;

  // Eliminate any re-matching of training points (multiple queries to one training)
  std::set<int> existing_trainIdx;
  for (unsigned int i = 0; i < matches.size(); i++) {
    //"normalize" matching: somtimes imgIdx is the one holding the trainIdx
    if (matches[i].trainIdx <= 0) {
      matches[i].trainIdx = matches[i].imgIdx;
    }

    if (existing_trainIdx.find(matches[i].trainIdx) == existing_trainIdx.end() &&
        matches[i].trainIdx >= 0 && matches[i].trainIdx < (int) (r_kps.size()) &&
           matches[i].distance > 0.0 && matches[i].distance < 20.0) {
      filt_matches.push_back(matches[i]);
      existing_trainIdx.insert(matches[i].trainIdx);
    }
  }

  std::cout << "matches has " << matches.size() << " filtered has " << filt_matches.size() << std::endl;

  matches = filt_matches;

  // Code for drawing matches side by side.
  // {
  //   //-- Draw only "good" matches
  //   cv::drawMatches(left_img, l_kps, right_img, r_kps, filt_matches, viz_img,
  //                   cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
  //                   cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  //   //-- Show detected matches
  //   std::stringstream ss;
  //   ss << "Feature Matches ";
  //   cv::imshow(ss.str(), viz_img);
  //   cv::waitKey(0);
  //   cv::destroyWindow(ss.str());
  // }


}
