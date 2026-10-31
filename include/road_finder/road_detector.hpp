#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace road_finder {

enum class RoadType {
  UNKNOWN,
  STRAIGHT,
  LEFT_TURN,
  RIGHT_TURN,
  LEFT_BRANCH,
  RIGHT_BRANCH,
  T_JUNCTION,
};

enum Direction : unsigned { LEFT = 1, STRAIGHT = 2, RIGHT = 4 };

// OpenCV's 8-bit Lab representation: L, a and b are all in [0, 255].
struct LabRange {
  int l_min = 133, l_max = 171;
  int a_min = 124, a_max = 148;
  int b_min = 123, b_max = 138;
};

struct Config {
  int input_width = 1280;
  int input_height = 720;
  int process_width = 640;
  // Main road search window in original input-image pixels.
  cv::Rect roi{26, 86, 1229, 612};
  std::array<cv::Point2f, 4> perspective_src{};
  std::array<cv::Point2f, 4> perspective_dst{};
  bool use_perspective = false;

  LabRange road_lab{};
  // "lab" preserves calibrated color segmentation; "adaptive_dark" learns
  // road/floor chroma when distinct, with row/frame brightness as fallback.
  std::string segmentation_mode = "lab";
  int road_contrast_min = 18;
  float min_road_width_fraction = 0.06f;
  float max_road_width_fraction = 0.85f;
  int classification_confirm_frames = 1;
  int classification_hold_frames = 0;
  float geometry_smoothing_alpha = 1.0f;
  // The sample's six scan strips and event regions are calibrated in 128x90
  // coordinates, then scaled independently to the processing image.
  std::array<cv::Rect, 6> scan_bands{{
      {0, 42, 128, 7}, {0, 49, 128, 7}, {0, 56, 128, 7},
      {0, 63, 128, 7}, {0, 70, 128, 7}, {0, 77, 128, 7}}};
  cv::Rect indicator_roi{50, 34, 28, 30};
  cv::Rect left_marker_roi{10, 50, 10, 10};
  cv::Rect right_marker_roi{108, 50, 10, 10};
  cv::Rect stop_roi{50, 50, 28, 7};
  LabRange stop_lab{51, 135, 149, 193, 83, 119};
  bool stop_detection_enabled = false;
  int min_blob_area_reference = 10;
  int indicator_min_area_reference = 53;
  int stop_min_area_reference = 53;
  int morphology_pixels = 3;

  int processHeight() const {
    return static_cast<int>(std::lround(static_cast<double>(input_height) * process_width /
                                        input_width));
  }
};

Config loadConfig(const std::string& path);
void validateConfig(const Config& config);

struct RoadResult {
  std::int64_t timestamp_ms = 0;
  bool valid = false;  // Measured steering geometry; independent of junction type.
  RoadType type = RoadType::UNKNOWN;
  RoadType raw_type = RoadType::UNKNOWN;
  unsigned available_directions = 0;
  float confidence = 0.0f;
  float classification_confidence = 0.0f;
  bool stop_detected = false;
  float lateral_error = 0.0f;  // Positive: target center is to the vehicle's right.
  float heading_error_rad = 0.0f;  // Positive: path turns to the right.
  std::vector<cv::Point2f> left_boundary;   // Original input-image coordinates.
  std::vector<cv::Point2f> right_boundary;
  std::vector<cv::Point2f> centerline;
  cv::Mat road_mask;  // Optional debug image, at process resolution.
  std::vector<cv::Point2f> roi_polygon;
};

const char* toString(RoadType type);

class RoadDetector {
 public:
  explicit RoadDetector(Config config);
  // Temporal filtering is opt-in for video; still images remain independent.
  RoadResult process(const cv::Mat& bgr, std::int64_t timestamp_ms, bool temporal = false);
  // Public for deterministic tests and camera-specific segmenters. Mask must
  // be CV_8UC1 at the configured processing size, with road pixels nonzero.
  RoadResult processMask(const cv::Mat& road_mask, std::int64_t timestamp_ms,
                         bool temporal = false);
  const cv::Rect& processRoi() const { return roi_processed_; }
  const std::vector<cv::Point2f>& roiPolygon() const { return roi_polygon_; }

 private:
  Config config_;
  cv::Mat warp_;
  cv::Mat inverse_warp_;
  cv::Mat valid_view_mask_;
  cv::Rect roi_processed_;
  std::vector<cv::Point2f> roi_polygon_;
  std::array<cv::Rect, 6> scan_bands_processed_{};
  cv::Rect indicator_roi_processed_;
  cv::Rect left_marker_roi_processed_;
  cv::Rect right_marker_roi_processed_;
  cv::Rect stop_roi_processed_;
  RoadType stable_type_ = RoadType::UNKNOWN;
  RoadType pending_type_ = RoadType::UNKNOWN;
  int pending_frames_ = 0;
  int unsupported_frames_ = 0;
  float stable_classification_confidence_ = 0.0f;
  bool has_previous_result_ = false;
  std::int64_t previous_timestamp_ms_ = 0;
  RoadResult previous_geometry_;

  void prepareView(const cv::Mat& bgr, cv::Mat& processed, cv::Mat& view) const;
  cv::Mat segment(const cv::Mat& view) const;
  RoadResult analyzeMask(const cv::Mat& mask, std::int64_t timestamp_ms,
                         const cv::Mat* processed_bgr);
  void stabilize(RoadResult& result);
  cv::Point2f sourcePoint(const cv::Point2f& processed) const;
};

}  // namespace road_finder
