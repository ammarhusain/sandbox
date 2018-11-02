#include "rich_feature.h"
#include "camera_matrices.h"

void GetFeatures(const cv::Mat& img, std::pair<std::vector<cv::KeyPoint>, cv::Mat>& kps_descriptors) {
  cv::Ptr<cv::FeatureDetector> detector  = cv::GFTTDetector::create();
  cv::Ptr<cv::FeatureDetector> extractor = cv::ORB::create();
  // cv::Ptr<cv::FeatureDetector> detector  = cv::FastFeatureDetector::create();
  // cv::Ptr<cv::FeatureDetector> extractor = cv::ORB::create();

  kps_descriptors.first.clear();
  CV_PROFILE("FeatureDetection", detector->detect(img, kps_descriptors.first);
             extractor->compute(img, kps_descriptors.first, kps_descriptors.second););

  assert(kps_descriptors.first.size() == kps_descriptors.second.rows);

  std::cout << "kps has " << kps_descriptors.first.size() << " points (descriptors " << kps_descriptors.second.rows
            << ")" << std::endl;
}


void MatchRichFeatures(const std::pair<std::vector<cv::KeyPoint>, cv::Mat>& l_kps_descriptors,
                       const std::pair<std::vector<cv::KeyPoint>, cv::Mat>& r_kps_descriptors,
                       std::vector<cv::DMatch>& matches) {
  // clear output variable
  matches.clear();

  const std::vector<cv::KeyPoint>& l_kps = l_kps_descriptors.first;
  const cv::Mat& left_descriptors = l_kps_descriptors.second;
  const std::vector<cv::KeyPoint>& r_kps = r_kps_descriptors.first;
  const cv::Mat& right_descriptors = r_kps_descriptors.second;

  // matching descriptor vectors using Brute Force matcher
  cv::BFMatcher matcher(
      cv::NORM_HAMMING,
      true);   // allow cross-check. use Hamming distance for binary descriptor (ORB)

  // NOTE: Get this radial matching working
  // NOTE: Make sure your train & query indices are not flipped in ther matcher
  // matcher.radiusMatch(InputArray queryDescriptors, InputArray trainDescriptors, std::vector<std::vector<DMatch> > &matches, float maxDistance)
  // matcher.radiusMatch(left_descriptors, right_descriptors, matches, 20);
  matcher.match(left_descriptors, right_descriptors, matches);
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
