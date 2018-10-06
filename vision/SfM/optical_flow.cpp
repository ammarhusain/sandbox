#include "optical_flow.h"

void MatchOpticalFlowFeatures(cv::Mat left_img, cv::Mat right_img, std::vector<cv::DMatch>& matches) {
  std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
   cv::FAST(left_img, left_keypoints, 10);
   cv::FAST(right_img, right_keypoints, 10);

   std::vector<cv::Point2f> l_pts;
   cv::KeyPoint::convert(left_keypoints,l_pts);

   std::vector<cv::Point2f> r_pts(l_pts.size());

   // making sure images are grayscale
   cv::Mat prevgray,gray;
   if (left_img.channels() == 3) {
     cvtColor(left_img,prevgray,CV_RGB2GRAY);
     cvtColor(right_img,gray,CV_RGB2GRAY);
   } else {
     prevgray = left_img;
     gray = right_img;
   }

   std::vector<uchar> vstatus(l_pts.size());
   std::vector<float> verror(l_pts.size());

   // Run optical flow.
   CV_PROFILE("OpticalFlow",calcOpticalFlowPyrLK(prevgray, gray, l_pts, r_pts, vstatus, verror);)

   std::vector<cv::Point2f> to_find;
   std::vector<int> to_find_back_idx;
   for (unsigned int i = 0; i < vstatus.size(); i++) {
     if (vstatus[i] && verror[i] < 12.0) {
       to_find_back_idx.push_back(i);
       to_find.push_back(r_pts[i]);
     } else {
       vstatus[i] = 0;
     }
   }

   std::set<int> found_in_imgpts_r;
   cv::Mat to_find_flat = cv::Mat(to_find).reshape(1, to_find.size());

   std::vector<cv::Point2f> r_pts_to_find;
   cv::KeyPoint::convert(right_keypoints, r_pts_to_find);
   cv::Mat r_pts_flat = cv::Mat(r_pts_to_find).reshape(1, r_pts_to_find.size());

   std::vector<std::vector<cv::DMatch> > knn_matches;
   // FlannBasedMatcher matcher;
   cv::BFMatcher matcher(CV_L2);
   CV_PROFILE("RadiusMatch", matcher.radiusMatch(to_find_flat, r_pts_flat, knn_matches, 2.0f);)
   CV_PROFILE("Prune", for (int i = 0; i < knn_matches.size(); i++) {
     cv::DMatch mtch;
     if (knn_matches[i].size() == 1) {
       mtch = knn_matches[i][0];
     } else if (knn_matches[i].size() > 1) {
       if (knn_matches[i][0].distance / knn_matches[i][1].distance < 0.7) {
         mtch = knn_matches[i][0];
       } else {
         continue;   // did not pass ratio test
       }
     } else {
       continue;   // no match
     }
     if (found_in_imgpts_r.find(mtch.trainIdx) == found_in_imgpts_r.end()) {   // prevent duplicates
       mtch.queryIdx =
           to_find_back_idx[mtch.queryIdx];   // back to original indexing of points for <i_idx>
       matches.push_back(mtch);
       found_in_imgpts_r.insert(mtch.trainIdx);
     }
   })

   // draw flow field
   cv::Mat img_matches;
   cv::cvtColor(left_img, img_matches, CV_GRAY2BGR);
   l_pts.clear();
   r_pts.clear();
   for (int i = 0; i < matches.size(); i++) {
     // if (i%2 != 0) {
     //				continue;
     //			}
     cv::Point l_pt = left_keypoints[(matches)[i].queryIdx].pt;
     cv::Point r_pt = right_keypoints[(matches)[i].trainIdx].pt;
     l_pts.push_back(l_pt);
     r_pts.push_back(r_pt);
     vstatus[i] = 1;
   }
   // drawArrows(img_matches, l_pts, r_pts, vstatus, verror, Scalar(0, 255));
   std::stringstream ss;
   ss << matches.size() << " matches";
   ss.clear();
   ss << "flow_field_" <<  ".png";
   imshow(ss.str(), img_matches);
   int c = cv::waitKey(0);
   if (c == 's') {
     cv::imwrite(ss.str(), img_matches);
   }
   cv::destroyWindow(ss.str());
}
