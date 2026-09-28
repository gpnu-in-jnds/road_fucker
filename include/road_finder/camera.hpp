#pragma once

#include "road_finder/road_detector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <string>

namespace road_finder {

// True for a camera index ("0") or a V4L2 device path ("/dev/videoN").
bool isCameraSource(const std::string& source);

// True for an image file, matched by extension. Video files and GStreamer
// pipelines are not accepted sources.
bool isImageSource(const std::string& source);

struct CameraOptions {
  std::string fourcc;  // Empty: a /dev/videoN source defaults to MJPG, an index sets none.
  double fps = 30.0;
};

// Owns the V4L2 capture: opens the device, requests fourcc/size/fps from the
// configuration and normalizes every frame to the configured input size as
// 8-bit BGR.
class Camera {
 public:
  Camera(const std::string& source, const CameraOptions& options, const Config& config);
  // Throws with a v4l2-ctl hint when the camera stops delivering frames.
  void read(cv::Mat& bgr);

 private:
  cv::VideoCapture capture_;
  cv::Size expected_;
  std::string source_;
  bool resize_reported_ = false;
};

}  // namespace road_finder
