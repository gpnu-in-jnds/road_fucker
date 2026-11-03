#pragma once

#include "road_finder/road_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace road_finder::test {

inline void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

inline Config config() {
  Config value;
  value.input_width = 640;
  value.input_height = 360;
  value.process_width = 640;
  value.roi = {0, 180, 640, 180};
  value.road_end_roi = {280, 240, 80, 50};
  return value;
}

inline cv::Mat road(int top = 0, int center = 320, int width = 81) {
  cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(mask, cv::Rect(center - width / 2, top, width, mask.rows - top), cv::Scalar(255), -1);
  return mask;
}

inline cv::Mat image(const cv::Mat& mask, cv::Scalar floor = {160, 160, 160},
                     cv::Scalar color = {150, 100, 80}) {
  cv::Mat frame(mask.size(), CV_8UC3, floor);
  frame.setTo(color, mask);
  return frame;
}

inline RoadResult detect(const cv::Mat& mask) {
  return RoadDetector(config()).processMask(mask, 42);
}

}  // namespace road_finder::test
