#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "log.h"
#include "qr_detect.h"

#define FREE_CHECK_NULL(x) do {                                                \
  if (!x) {                                                                    \
    free(x);                                                                   \
    x = NULL;                                                                  \
  }                                                                            \
} while(0)

int main(int argc, char *argv[]) {

  cv::VideoCapture cap(0);
  if (!cap.isOpened())
    return -1;
  cv::Mat frame;
  cv::namedWindow("Detection", 1);
  while(true) {
    cap >> frame;
    // Detect QR Corners Identifiers
    qr::Contour_t corners;
    qr::QRDetectIdentifiers(frame, &corners);
    cv::drawContours(frame, corners, -1, cv::Scalar(1,0,0));
    corners.clear();
    cv::imshow("Detection", frame);
    if (cv::waitKey(30) >= 0) break;
  }
  return 0;
}