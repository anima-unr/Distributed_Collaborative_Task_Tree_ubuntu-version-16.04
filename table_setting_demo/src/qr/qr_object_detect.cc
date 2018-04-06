#include "qr/qr_object_detect.h"
#include <limits>
namespace qr {
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Tracker ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Tracker::Tracker(
    cv::Ptr<cv::Feature2D> detector_,
    cv::Ptr<cv::DescriptorMatcher> matcher_) : 
      detector(detector_),
      matcher(matcher_) {
  // Constructor for Tracker Class
}

void Tracker::InitializeTracker(
    const cv::Mat &frame,
    std::vector<cv::Point> bbox,
    std::string object_id) {

  cv::Ptr<cv::Tracker> tracker;
  bounding.x = bbox[0].x;
  bounding.y = bbox[0].y;
  bounding.width  = bbox[1].x - bbox[0].x;
  bounding.height = bbox[1].y - bbox[0].y;

  // Initialize filter
  filter.InitializeFilter(bounding);

  // crop image for orb detector frame
  cv::Mat roi = frame(
    cv::Range(bbox[0].y, bbox[1].y),
    cv::Range(bbox[0].x, bbox[1].x));



  roi_center.x = roi.cols/2.0;
  roi_center.y = roi.rows/2.0;

  detector->detect(roi, key_points);
  detector->compute(roi, key_points, descriptor);
  cv::Mat img = roi.clone();
  cv::drawKeypoints(roi, key_points, img);
  imshow("Original", img);
}

std::vector<cv::Point2f> KeyToPoints(std::vector<cv::KeyPoint> points) {
  std::vector<cv::Point2f> output(points.size());
  for (int i = 0; i < points.size(); ++i) {
    output[i] = points[i].pt;
  }
  return output;
}

void Tracker::ProcessFrame(const cv::Mat &image) {
  const double nn_match_ratio = 0.8f;
  std::vector<cv::KeyPoint> img_keypoints;
  cv::Mat img_descriptor;
  double ransac_thresh = 2.5f;
  cv::Mat image_roi;
  bool roi_set = false;
  cv::Rect prediction;
  filter.GetStatePrediction(&prediction);
  try {
    image_roi = image(prediction);
    if (image_roi.rows > 128 && image_roi.cols > 128) {
      imshow("Prediction", image_roi);
      roi_set = true;
    }
  } catch (cv::Exception e) {
    LOG_INFO("ROI: Not within frame!");
  }

  if (roi_set) {
    detector->detect(image_roi, img_keypoints);
    detector->compute(image_roi, img_keypoints, img_descriptor);
  } else {
    detector->detect(image, img_keypoints);
    detector->compute(image, img_keypoints, img_descriptor);
  }

  std::vector< std::vector<cv::DMatch> > matches;
  std::vector<cv::KeyPoint> matched_sample, matched_image;

  matcher->knnMatch(descriptor, img_descriptor, matches, 2);

  for (int i = 0; i <matches.size(); ++i) {
    if (matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
      matched_image.push_back(key_points[matches[i][0].queryIdx]);
      matched_sample.push_back(img_keypoints[matches[i][0].trainIdx]);
    }
  }


  // Solve Homography
  cv::Mat inlier_mask, homography;
  std::vector<cv::KeyPoint> inliers1, inliers2;
  std::vector<cv::DMatch> inlier_matches;
  cv::Rect estimate;
  if (roi_set) {
    for (int i = 0; i < matched_sample.size(); ++i) {
      matched_sample[i].pt.x += prediction.x;
      matched_sample[i].pt.y += prediction.y;
    }
  }

  if (matched_image.size() >= 4) {
    homography = cv::findHomography(
      KeyToPoints(matched_image),
      KeyToPoints(matched_sample),
      cv::RANSAC,
      ransac_thresh,
      inlier_mask);
  }

  if (!homography.empty()) {
    std::vector<cv::Point2f> positions, transformed_positions;
    cv::Point2f point;

    point.x = roi_center.x;
    point.y = roi_center.y;
    positions.push_back(point);
    point.x = roi_center.x + 10;
    point.y = roi_center.y + 10;
    positions.push_back(point);
    point.x = roi_center.x + 10;
    point.y = roi_center.y - 10;
    positions.push_back(point);
    point.x = roi_center.x - 10;
    point.y = roi_center.y + 10;
    positions.push_back(point);
    point.x = roi_center.x - 10;
    point.y = roi_center.y - 10;
    positions.push_back(point);
    float old_width = 20;
    float old_height = 20;
    cv::perspectiveTransform(positions, transformed_positions, homography);

    // update kalman filter with measurement
    filter.GetStateEstimate(&estimate);
    cv::Rect rec;
    rec.x = transformed_positions[0].x - estimate.width/2;
    rec.y = transformed_positions[0].y - estimate.height/2;

    // average width change and height
    float maxx = -std::numeric_limits<float>::infinity();
    float minx =  std::numeric_limits<float>::infinity();
    float maxy = -std::numeric_limits<float>::infinity();
    float miny =  std::numeric_limits<float>::infinity();
    for (int i = 1; i < 5; ++i) {
      if (maxx < transformed_positions[i].x)
        maxx = transformed_positions[i].x;
      if (minx > transformed_positions[i].x)
        minx = transformed_positions[i].x;
      if (maxy < transformed_positions[i].y)
        maxy = transformed_positions[i].y;
      if (miny > transformed_positions[i].y)
        miny = transformed_positions[i].y;
    }
    float new_width  = maxx - minx;
    float new_height = maxy - miny;

    float width_change_ratio  = new_width / old_width;
    float height_change_ratio = new_height / old_height;
    rec.width  = bounding.width * width_change_ratio;
    rec.height = bounding.height * height_change_ratio;

    filter.MeasurementUpdate(rec);
    filter.GetStateEstimate(&estimate);

    cv::Mat img2 = image.clone();
    cv::rectangle(img2, 
      cv::Point(estimate.x, estimate.y),
      cv::Point(estimate.x + estimate.width, estimate.y + estimate.height),
      cv::Scalar(0,255,80),
      3);
    cv::circle(img2, transformed_positions[0], 3, cv::Scalar(50,255,50), 4);
    cv::imshow("Tracked", img2);

  } else {
    LOG_INFO("Homography not found!");
    // object not found in this frame update Kalman filter  and estimate state
    filter.Update();

    filter.GetStateEstimate(&estimate);
    cv::Mat img2 = image.clone();
    cv::rectangle(img2,
      cv::Point(estimate.x, estimate.y),
      cv::Point(estimate.x + estimate.width, estimate.y + estimate.height),
      cv::Scalar(0, 255, 80),
      3);
    cv::imshow("Tracked", img2);
  }
}
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Kalman2DTracker ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Kalman2DTracker::Kalman2DTracker() {}
Kalman2DTracker::~Kalman2DTracker() {}
void Kalman2DTracker::InitializeFilter(const cv::Rect &measurement) {
  position_filter.init(4, 2, 0);
  bounding_filter.init(4, 2, 0);

  // Set Transition Matrices
  position_filter.transitionMatrix = (cv::Mat_<float>(4, 4) <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1);
  bounding_filter.transitionMatrix = (cv::Mat_<float>(4, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1);

  // set measurement Matrices
  cv::Size size;
  size = position_filter.measurementMatrix.size();
  position_filter.measurementMatrix = cv::Mat::eye(size, CV_32F) * 1;

  size = bounding_filter.measurementMatrix.size();
  bounding_filter.measurementMatrix = cv::Mat::eye(size, CV_32F) * 1;

  // set Process Noise Cov
  size = position_filter.processNoiseCov.size();
  position_filter.processNoiseCov = cv::Mat::eye(size, CV_32F) * 0.001;

  size = bounding_filter.processNoiseCov.size();
  bounding_filter.processNoiseCov = cv::Mat::eye(size, CV_32F) * 0.00001;

  // set Measurement Noise Covariance
  size = position_filter.measurementNoiseCov.size();
  position_filter.measurementNoiseCov = cv::Mat::eye(size, CV_32F) * 0.5;

  size = bounding_filter.measurementNoiseCov.size();
  bounding_filter.measurementNoiseCov = cv::Mat::eye(size, CV_32F) * 0.001;

  // set Error Covariance Post
  size = position_filter.errorCovPost.size();
  position_filter.errorCovPost = cv::Mat::eye(size, CV_32F) * .1;

  size = bounding_filter.errorCovPost.size();
  bounding_filter.errorCovPost = cv::Mat::eye(size, CV_32F) * .1;

  // Initialize state
  position_filter.statePre.at<float>(0) = measurement.x + measurement.width/2;
  position_filter.statePre.at<float>(1) = measurement.y + measurement.height/2;
  position_filter.statePre.at<float>(2) = 0;
  position_filter.statePre.at<float>(3) = 0;

  bounding_filter.statePre.at<float>(0) = measurement.width;
  bounding_filter.statePre.at<float>(1) = measurement.height;
  bounding_filter.statePre.at<float>(2) = 0;
  bounding_filter.statePre.at<float>(3) = 0;

  // Initial predict
  state_position_prediction = position_filter.predict();
  state_bounding_prediction = bounding_filter.predict();
}

void Kalman2DTracker::MeasurementUpdate(const cv::Rect &measurement) {
  cv::Mat_<float> position(2, 1);
  position.at<float>(0) = measurement.x + measurement.width  / 2;
  position.at<float>(1) = measurement.y + measurement.height / 2;

  cv::Mat_<float> bounding(2, 1);
  bounding.at<float>(0) = measurement.width;
  bounding.at<float>(1) = measurement.height;

  state_position_estimate = position_filter.correct(position);
  state_bounding_estimate = bounding_filter.correct(bounding);

  state_position_prediction = position_filter.predict();
  state_bounding_prediction = bounding_filter.predict();
}

void Kalman2DTracker::Update() {
  state_position_estimate = state_position_prediction;
  state_bounding_estimate = state_bounding_prediction;

  state_position_prediction = position_filter.predict();
  state_bounding_prediction = bounding_filter.predict();
}

void Kalman2DTracker::GetStatePrediction(cv::Rect *measurement) {
  measurement->x = state_position_prediction.at<float>(0) -
                   state_bounding_prediction.at<float>(0)/2;
  measurement->y = state_position_prediction.at<float>(1) -
                   state_bounding_prediction.at<float>(1)/2;

  measurement->width  = state_bounding_prediction.at<float>(0);
  measurement->height = state_bounding_prediction.at<float>(1);
}

void Kalman2DTracker::GetStateEstimate(cv::Rect *measurement) {
  measurement->x = state_position_estimate.at<float>(0) -
                   state_bounding_estimate.at<float>(0)/2;
  measurement->y = state_position_estimate.at<float>(1) -
                   state_bounding_estimate.at<float>(1)/2;

  measurement->width  = state_bounding_estimate.at<float>(0);
  measurement->height = state_bounding_estimate.at<float>(1);
}
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// QrFeatureDetector //////////////////////////////
////////////////////////////////////////////////////////////////////////////////
QrFeatureDetector::QrFeatureDetector() {}
QrFeatureDetector::~QrFeatureDetector() {}
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// QrTracker //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
QrTracker::QrTracker() {}
QrTracker::~QrTracker() {}
////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// QrObjectsTrack ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
QrObjectsTrack::QrObjectsTrack() {}
QrObjectsTrack::~QrObjectsTrack() {}

uint32_t QrObjectsTrack::Init(
    const char *camera_topic,
    std::vector<std::string> object_list,
    cv::Mat image) {
  // cv::TrackerKCF::Params params;
  // params.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
  // params.desc_npca = 0;
  // params.compress_feature = true;
  // params.compressed_size = 2;
  // params.resize = true;
  object_list_ = object_list;
  const char *window = "TestSelection";
  // tracker = new cv::MultiTracker("TLD");
  tracker = new cv::MultiTrackerTLD();
  cv::namedWindow(window);

  std::vector<cv::Rect2d> rois(object_list.size());
  for (int i = 0; i < object_list.size(); ++i) {
    cv::Mat text_image = image.clone();
    cv::putText(
      text_image,
      object_list[i],
      cv::Point(10, 50),
      cv::FONT_HERSHEY_PLAIN,
      3,
      cv::Scalar(10, 200, 50),
      3);
    rois[i] = cv::selectROI(window, text_image);
  }
  cv::destroyWindow(window);

  // Initialize the trackers to the ROIs
  // tracker->add(image, rois);
  for (int i = 0; i < rois.size(); ++i) {
    tracker->addTarget(image, rois[i], "TLD");
  }
}
bool QrObjectsTrack::UpdateFrame(const cv::Mat &image) {
  tracker->update_opt(image);
}

std::vector<cv::Rect2d> QrObjectsTrack::GetTrackedROIs() {
  return tracker->boundingBoxes;
}
void QrObjectsTrack::InitializeTrack(cv::Rect2d roi) {}
bool QrObjectsTrack::GetObject(std::string object, std::string &object_id) {
  for (int i = 0; i < object_list_.size(); ++i) {
    if (object.compare(object_list_[i]) == 0) {
      object_id = object;
      return true;
    }
  }
  return false;
}
bool QrObjectsTrack::ObjectInView(std::string object) {
  for (int i = 0; i < object_list_.size(); ++i) {
    if (object.compare(object_list_[i]) == 0)
      return true;
  }
  return false;
}
}  // namespace qr
