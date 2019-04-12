#ifndef QR_OBJECT_DETECT_H_
#define QR_OBJECT_DETECT_H_
#include "log.h"
#include "qr_detect.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace qr {
typedef enum TrackingState {
  TRACK = 0,
  PREDICT,
} TrackingState_e;

typedef struct TrackerState {
  uint32_t tracking_state;
  bool valid;
} TrackerState_t;

class Kalman2DTracker {
 public:
  Kalman2DTracker();
  virtual ~Kalman2DTracker();

  void InitializeFilter(const cv::Rect &measurement);
  void MeasurementUpdate(const cv::Rect &measurement);
  void Update();
  void GetStatePrediction(cv::Rect *measurement);
  void GetStateEstimate(cv::Rect *measurement);
 protected:
  cv::KalmanFilter position_filter;
  cv::KalmanFilter bounding_filter;
  cv::Mat state_position_prediction, state_position_estimate;
  cv::Mat state_bounding_prediction, state_bounding_estimate;
};

class Tracker {
 public:
  Tracker(
    cv::Ptr<cv::Feature2D> detector_,
    cv::Ptr<cv::DescriptorMatcher> matcher_);
  virtual void InitializeTracker(
    const cv::Mat &frame,
    std::vector<cv::Point> bbox,
    std::string object_id);
  virtual void ProcessFrame(const cv::Mat &image);
 protected:
  cv::Ptr<cv::Feature2D> detector, detector_alt;
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Mat descriptor;
  std::vector<cv::KeyPoint> key_points, key_points_alt;
  cv::Rect bounding;
  cv::Point2f roi_center;

  Kalman2DTracker filter;
};

class QrFeatureDetector {
 public:
  QrFeatureDetector();
  virtual ~QrFeatureDetector();
};

class QrTracker : public Kalman2DTracker {
 public:
  QrTracker();
  virtual ~QrTracker();
};

class QrObjectsTrack {
 public:
  QrObjectsTrack();
  virtual ~QrObjectsTrack();

  uint32_t Init(const char *camera_topic, std::vector<std::string> object_list,
    cv::Mat image);
  bool UpdateFrame(const cv::Mat &image);
  std::vector<cv::Rect2d> GetTrackedROIs();
  void InitializeTrack(cv::Rect2d roi);
  bool GetObject(std::string object, std::string &object_id);
  bool ObjectInView(std::string object);
 private:
  cv::Ptr<cv::MultiTrackerTLD> tracker;
  std::vector<cv::Rect2d> tracking_regions;
  std::vector<std::string> object_list_;
};
}  // namespace qr
#endif  // QR_OBJECT_DETECT_H_
