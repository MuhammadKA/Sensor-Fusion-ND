
#include "matching2D.hpp"
#include <numeric>

using namespace std;

// Find best matches for keypoints in two camera images based on several
// matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches,
                      std::string descriptorType, std::string matcherType,
                      std::string selectorType) {
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcherType.compare("MAT_BF") == 0) {
    int normType = cv::NORM_HAMMING;
    matcher = cv::BFMatcher::create(normType, crossCheck);
  } else if (matcherType.compare("MAT_FLANN") == 0) {
    if (descSource.type() != CV_32F)
      descSource.convertTo(descSource, CV_32F);
    if (descRef.type() != CV_32F)
      descRef.convertTo(descRef, CV_32F);

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::cout << "FLANN matching done" << endl;
  }

  // perform matching task
  std::vector<std::vector<cv::DMatch>> knnMatches;
  if (selectorType.compare("SEL_NN") == 0) { // nearest neighbor (best match)

    matcher->match(
        descSource, descRef,
        matches); // Finds the best match for each descriptor in desc1
  } else if (selectorType.compare("SEL_KNN") ==
             0) { // k nearest neighbors (k=2)

    matcher->knnMatch(descSource, descRef, knnMatches, 2);
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify
// keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                   cv::Mat &descriptors, string descriptorType) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptorType.compare("BRISK") == 0) {

    int threshold = 30;        // FAST/AGAST detection threshold score.
    int octaves = 3;           // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f; // apply this scale to the pattern used for
                               // sampling the neighbourhood of a keypoint.

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  } else if (descriptorType.compare("ORB") == 0) {
    int nfeatures = 500;
    extractor = cv::ORB::create(nfeatures);
  } else if (descriptorType.compare("FREAK") == 0) {
    extractor = cv::xfeatures2d::FREAK::create();
  } else if (descriptorType.compare("BRIEF") == 0) {
    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
  } else if (descriptorType.compare("AKAZE") == 0) {
    extractor = cv::AKAZE::create();
  } else // SIFT
  {
    extractor = cv::xfeatures2d::SIFT::create();
  }

  // perform feature description
  double t = (double)cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0
       << " ms" << endl;

  if (false) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Error?";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                           bool bVis) {
  // compute detector parameters based on image size
  int blockSize = 4; //  size of an average block for computing a derivative
                     //  covariation matrix over each pixel neighborhood
  double maxOverlap = 0.0; // max. permissible overlap between two features in %
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners =
      img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

  double qualityLevel = 0.01; // minimal accepted quality of image corners
  double k = 0.04;

  // Apply corner detection
  double t = (double)cv::getTickCount();
  vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance,
                          cv::Mat(), blockSize, false, k);

  // add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it) {

    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in "
       << 1000 * t / 1.0 << " ms" << endl;

  // visualize results
  if (bVis) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Shi-Tomasi Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                        std::string detectorType, bool bVis) {
  if (detectorType.compare("FAST") == 0) {
    int threshold = 30;
    bool bNMS = true;
    cv::FastFeatureDetector::DetectorType type =
        cv::FastFeatureDetector::TYPE_9_16;
    cv::Ptr<cv::FeatureDetector> detector =
        cv::FastFeatureDetector::create(threshold, bNMS, type);

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "FAST with n= " << keypoints.size()
              << "   time (ms) = " << 1000 * t / 1.0 << endl;
    if (bVis) {
      cv::Mat visImage = img.clone();
      cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      string windowName = "FAST Detector Results";
      cv::namedWindow(windowName, 6);
      imshow(windowName, visImage);
      cv::waitKey(0);
    }

  }

  else if (detectorType.compare("BRISK") == 0) {
    int threshold = 30;
    int octaves = 3;
    float patternScale = 1.0f;
    cv::Ptr<cv::FeatureDetector> detector =
        cv::BRISK::create(threshold, octaves, patternScale);

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "BRISK with n= " << keypoints.size()
              << "   time (ms) = " << 1000 * t / 1.0 << endl;

    if (bVis) {
      cv::Mat visImage = img.clone();
      cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      string windowName = "BRISK Detector Results";
      cv::namedWindow(windowName, 6);
      imshow(windowName, visImage);
      cv::waitKey(0);
    }
  }

  else if (detectorType.compare("ORB") == 0) {
    int nfeatures = 1500;
    float scaleFactor = 1.2f;
    int nlevels = 8;
    int edgeThreshold = 31;
    int fristLevel = 0;
    int WTA_k = 2;
    int scoreType = 0; //(int)cv::ORB::HARRIS_SCORE;
    int patchSize = 31;
    int fastThreshold = 20;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(
        nfeatures =
            1500); // nfeatures, scaleFactor, nlevels, edgeThreshold,
                   // fristLevel, WTA_k, scoreType, patchSize,fastThreshold);

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "ORB with n= " << keypoints.size()
              << "   time (ms) = " << 1000 * t / 1.0 << endl;

    if (bVis) {
      cv::Mat visImage = img.clone();
      cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      string windowName = "ORB Detector Results";
      cv::namedWindow(windowName, 6);
      imshow(windowName, visImage);
      cv::waitKey(0);
    }
  } else if (detectorType.compare("AKAZE") == 0) {
    cv::Ptr<cv::FeatureDetector> detector =
        cv::AKAZE::create(); // nfeatures, scaleFactor, nlevels, edgeThreshold,
                             // fristLevel, WTA_k, scoreType,
                             // patchSize,fastThreshold);

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "AKAZE with n= " << keypoints.size()
              << "   time (ms) = " << 1000 * t / 1.0 << endl;

    if (bVis) {
      cv::Mat visImage = img.clone();
      cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      string windowName = "AKAZE Detector Results";
      cv::namedWindow(windowName, 6);
      imshow(windowName, visImage);
      cv::waitKey(0);
    }
  } else // SIFT
  {
    cv::Ptr<cv::FeatureDetector> detector =
        cv::xfeatures2d::SIFT::create(); // nfeatures, scaleFactor, nlevels,
                                         // edgeThreshold, fristLevel, WTA_k,
                                         // scoreType, patchSize,fastThreshold);

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "SIFT with n= " << keypoints.size()
              << "   time (ms) = " << 1000 * t / 1.0 << endl;

    if (bVis) {
      cv::Mat visImage = img.clone();
      cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      string windowName = "SIFT Detector Results";
      cv::namedWindow(windowName, 6);
      imshow(windowName, visImage);
      cv::waitKey(0);
    }
  }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                        bool bVis) {
  int blockSize = 2;
  int apertureSize = 3;
  int minResponse = 100;
  double k = 0.04;
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);

  double t = (double)cv::getTickCount();
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  std::cout << "Here1" << endl;

  double maxOverlap = 0.0;
  for (size_t j = 0; j < dst_norm_scaled.rows; j++) {
    std::cout << "Here2" << endl;
    for (size_t i = 0; i < dst_norm_scaled.cols; i++) {
      int response = (int)dst_norm_scaled.at<float>(j, i);
      if (response > minResponse) {
        std::cout << "Here3" << endl;
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f(j, i);
        newKeyPoint.size = 2 * apertureSize;
        newKeyPoint.response = response;
        bool bOverlap = false;
        for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
          std::cout << "Here4" << endl;
          double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
          if (kptOverlap > maxOverlap) {
            bOverlap = true;
            if (newKeyPoint.response > (*it).response) {
              *it = newKeyPoint;
              break;
            }
          }
        }
        std::cout << "Here5" << endl;
        if (!bOverlap) {
          keypoints.push_back(newKeyPoint);
        }
      }
    }
  }
  std::cout << "Here33" << endl;
  t = (double)(cv::getTickCount() - t) / cv::getTickFrequency();
  // visualize results
  if (bVis) {
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Harris Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}