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

  if (argc < 2) {
    printf("Usage: No arguments present\n");
    return -1;
  }
  printf("Testing QR Code Detection...\n");

  char *input_img, *output_img;
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "-i")) {
      LOG_INFO("Input Image: %s", argv[i+1]);
      input_img = strdup(argv[i+1]);
    }
    if (!strcmp(argv[i], "-f")) {
      LOG_INFO("Output Image: %s", argv[i+1]);
      output_img = strdup(argv[i+1]);
    }
  }

  printf("Input: %s, Output: %s\n", input_img, output_img);
  cv::Mat in_img, out_img;
  in_img = cv::imread(input_img);

  // Detect QR Corners Identifiers
  qr::Contour_t corners;
  qr::QRDetectIdentifiers(in_img, &corners);

  // cv::imshow("QR Test", in_img);
  cv::waitKey(0);
  FREE_CHECK_NULL(input_img);
  FREE_CHECK_NULL(output_img);
  return 0;
}