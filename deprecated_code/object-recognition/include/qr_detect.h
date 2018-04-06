#ifndef INCLUDE_QR_DETECT_H_
#define INCLUDE_QR_DETECT_H_
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace qr {
// Type Definitions //
typedef std::vector< std::vector<cv::Point> > Contour_t;

bool QRDetectIdentifiers(cv::Mat image, cv::Rect *roi, cv::Mat &qr_image);
}
#endif  // INCLUDE_QR_DETECT_H_
