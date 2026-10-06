#include "road_finder/camera.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cctype>
#include <iostream>
#include <stdexcept>

namespace road_finder {
namespace {

bool isDigits(const std::string& text) {
  return !text.empty() && text.find_first_not_of("0123456789") == std::string::npos;
}

std::string lowerExtension(const std::string& source) {
  const auto dot = source.find_last_of('.');
  if (dot == std::string::npos) return {};
  std::string extension = source.substr(dot);
  for (char& ch : extension) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  return extension;
}

int cameraIndex(const std::string& source) {
  constexpr const char* prefix = "/dev/video";
  return std::stoi(source.rfind(prefix, 0) == 0 ? source.substr(10) : source);
}

// Scales other 16:9 camera modes to the configured size; anything else is an error.
cv::Mat normalizeFrame(const cv::Mat& frame, const cv::Size& expected, bool& resize_reported) {
  if (frame.empty()) throw std::runtime_error("camera returned an empty frame");
  if (frame.depth() != CV_8U) throw std::runtime_error("camera must deliver 8-bit frames");
  cv::Mat bgr;
  if (frame.channels() == 3) bgr = frame;
  else if (frame.channels() == 1) cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
  else if (frame.channels() == 4) cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
  else
    throw std::runtime_error("unsupported camera frame: " +
                             std::to_string(frame.channels()) + " channels");

  if (bgr.size() == expected) return bgr;
  const std::string actual = std::to_string(bgr.cols) + "x" + std::to_string(bgr.rows);
  const std::string wanted = std::to_string(expected.width) + "x" +
                             std::to_string(expected.height);
  const double aspect_error = std::abs(static_cast<double>(bgr.cols) / bgr.rows -
                                       static_cast<double>(expected.width) / expected.height);
  if (aspect_error > 0.01)
    throw std::runtime_error("camera delivered " + actual + ", expected " + wanted +
                             "; select a 16:9 camera mode with v4l2-ctl or change the input "
                             "size in config");
  if (!resize_reported) {
    std::cerr << "camera delivered " << actual << "; resizing to " << wanted
              << " for detection. Calibrate against the actual camera resolution.\n";
    resize_reported = true;
  }
  cv::Mat resized;
  cv::resize(bgr, resized, expected, 0, 0, cv::INTER_LINEAR);
  return resized;
}

}  // namespace

bool isCameraSource(const std::string& source) {
  constexpr const char* prefix = "/dev/video";
  return isDigits(source) ||
         (source.rfind(prefix, 0) == 0 && isDigits(source.substr(10)));
}

bool isImageSource(const std::string& source) {
  const std::string extension = lowerExtension(source);
  return extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
         extension == ".bmp" || extension == ".webp";
}

Camera::Camera(const std::string& source, const CameraOptions& options, const Config& config)
    : expected_(config.input_width, config.input_height), source_(source) {
  const bool device_path = source.rfind("/dev/video", 0) == 0;
  if (!isCameraSource(source)) throw std::runtime_error("not a camera source: " + source);

  capture_.open(cameraIndex(source), device_path ? cv::CAP_V4L2 : cv::CAP_ANY);
  if (!capture_.isOpened())
    throw std::runtime_error("cannot open source: " + source + (device_path ? " with V4L2" : ""));

  // UVC cameras often provide 720p at full frame rate only in MJPG mode.
  const std::string code = options.fourcc.empty() && device_path ? "MJPG" : options.fourcc;
  if (!code.empty() &&
      !capture_.set(cv::CAP_PROP_FOURCC,
                    cv::VideoWriter::fourcc(code[0], code[1], code[2], code[3])))
    std::cerr << "camera did not accept FOURCC " << code << '\n';

  capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.input_width);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.input_height);
  if (!capture_.set(cv::CAP_PROP_FPS, options.fps))
    std::cerr << "camera did not accept requested " << options.fps << " FPS\n";
  capture_.set(cv::CAP_PROP_CONVERT_RGB, 1);
  // One buffer keeps the frame that is read as fresh as possible; not every
  // backend accepts it, and the frame rate does not depend on it.
  capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  const double negotiated_fps = capture_.get(cv::CAP_PROP_FPS);
  std::cerr << "camera requested " << config.input_width << 'x' << config.input_height
            << ", negotiated " << static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH))
            << 'x' << static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT))
            << " at " << negotiated_fps << " FPS via " << capture_.getBackendName() << '\n';
  if (negotiated_fps > 0.0 && negotiated_fps < options.fps * 0.5)
    std::cerr << "camera negotiated " << negotiated_fps << " FPS (requested " << options.fps
              << "); the preview cannot run faster than the camera mode - check "
                 "v4l2-ctl -d " << source_ << " --list-formats-ext and prefer MJPG at "
              << config.input_width << 'x' << config.input_height << '\n';
}

void Camera::read(cv::Mat& bgr) {
  cv::Mat frame;
  if (!capture_.read(frame))
    throw std::runtime_error("camera did not deliver a frame from " + source_ +
                             "; test the device with v4l2-ctl -d " + source_ +
                             " --stream-mmap --stream-count=10");
  bgr = normalizeFrame(frame, expected_, resize_reported_);
}

}  // namespace road_finder
