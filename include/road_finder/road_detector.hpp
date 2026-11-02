#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
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

// OpenCV 8-bit HSV: H in [0,179], S/V in [0,255]; H_min > H_max wraps at zero.
struct HsvRange {
  int h_min = 80, h_max = 135;
  int s_min = 10, s_max = 150;
  int v_min = 40, v_max = 230;
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

  HsvRange road_hsv{};
  std::optional<HsvRange> side_road_hsv;  // Unset: inherit road_hsv on both sides.
  int side_left_end_reference = 36;
  int side_right_begin_reference = 92;
  float marker_min_coverage = 0.35f;
  int white_s_max = 45;
  int white_v_min = 170;
  float occlusion_max_gap_fraction = 0.25f;
  float geometry_min_span_fraction = 0.15f;
  // "hsv" preserves calibrated color segmentation; "adaptive_dark" learns
  // road/floor chroma when distinct, with row/frame brightness as fallback.
  std::string segmentation_mode = "hsv";
  int road_contrast_min = 18;
  float min_road_width_fraction = 0.06f;
  float max_road_width_fraction = 0.85f;
  int classification_confirm_frames = 1;
  int classification_hold_frames = 0;
  float geometry_smoothing_alpha = 1.0f;
  int threads = 1;  // Worker count; the ROI has threads * 6 horizontal strips.
  // Event regions use independently scaled 128x90 reference coordinates.
  cv::Rect indicator_roi{50, 34, 28, 30};
  std::array<cv::Rect, 3> left_marker_rois{{{10, 38, 10, 10}, {24, 28, 10, 10},
                                           {24, 48, 10, 10}}};
  std::array<cv::Rect, 3> right_marker_rois{{{108, 38, 10, 10}, {94, 28, 10, 10},
                                            {94, 48, 10, 10}}};
  cv::Rect stop_roi{50, 50, 28, 7};
  HsvRange stop_hsv{170, 10, 100, 255, 80, 255};
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

struct BlockObservation {
  cv::Rect processed_region;
  std::vector<cv::Point2f> polygon;  // Original input-image coordinates.
  bool found = false;
  float coverage = 0.0f;  // Fraction of measured road pixels in a junction box.
  cv::Point2f midpoint{};  // Measured, before line fitting and temporal smoothing.
};

struct RoadResult {
  std::int64_t timestamp_ms = 0;
  bool valid = false;  // Measured steering geometry; independent of junction type.
  RoadType type = RoadType::UNKNOWN;
  RoadType raw_type = RoadType::UNKNOWN;
  unsigned available_directions = 0;
  float confidence = 0.0f;
  float classification_confidence = 0.0f;
  bool stop_detected = false;
  int left_junction_hits = 0;
  int right_junction_hits = 0;
  bool occlusion_detected = false;
  float lateral_error = 0.0f;  // Positive: target center is to the vehicle's right.
  float heading_error_rad = 0.0f;  // Positive: path turns to the right.
  std::vector<cv::Point2f> left_boundary;   // Original input-image coordinates.
  std::vector<cv::Point2f> right_boundary;
  std::vector<cv::Point2f> centerline;
  std::vector<BlockObservation> scan_blocks;
  std::array<BlockObservation, 6> junction_blocks{};  // L1..L3, R1..R3.
  cv::Mat white_mask;  // Color candidates; not all white pixels are occlusions.
  cv::Mat occlusion_mask;  // White inside a geometrically supported road corridor.
  cv::Mat bridge_mask;  // Association only; never measured or counted in votes.
  std::array<int, 2> hsv_region_edges{};  // Processing x coordinates.
  cv::Mat road_mask;  // Optional debug image, at process resolution.
  std::vector<cv::Point2f> roi_polygon;
};

const char* toString(RoadType type);

class WorkerPool;

class RoadDetector {
 public:
  explicit RoadDetector(Config config);
  ~RoadDetector();
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
  std::vector<cv::Rect> scan_bands_processed_;
  std::unique_ptr<WorkerPool> workers_;
  cv::Rect indicator_roi_processed_;
  std::array<cv::Rect, 6> marker_rois_processed_{};
  cv::Rect stop_roi_processed_;
  RoadType stable_type_ = RoadType::UNKNOWN;
  RoadType pending_type_ = RoadType::UNKNOWN;
  int pending_frames_ = 0;
  int unsupported_frames_ = 0;
  float stable_classification_confidence_ = 0.0f;
  bool has_previous_result_ = false;
  std::int64_t previous_timestamp_ms_ = 0;
  RoadResult previous_geometry_;

  std::vector<cv::Point2f> sourcePolygon(const cv::Rect& rect) const;
  void prepareView(const cv::Mat& bgr, cv::Mat& processed, cv::Mat& view) const;
  cv::Mat segment(const cv::Mat& view) const;
  RoadResult analyzeMask(const cv::Mat& mask, std::int64_t timestamp_ms,
                         const cv::Mat* processed_bgr);
  void stabilize(RoadResult& result);
  cv::Point2f sourcePoint(const cv::Point2f& processed) const;
};

}  // namespace road_finder
