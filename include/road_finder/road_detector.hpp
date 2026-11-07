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

// Historical enum (implicit values, NOT the old UART type bytes):
// enum class RoadType {
//   UNKNOWN,       // 0; old UART 0x04
//   STRAIGHT,      // 1; old UART 0x00
//   LEFT_TURN,     // 2; old UART 0x01
//   RIGHT_TURN,    // 3; old UART 0x02
//   LEFT_BRANCH,   // 4; old UART 0x04
//   RIGHT_BRANCH,  // 5; old UART 0x04
//   T_JUNCTION,    // 6; old UART 0x03
// };
enum class RoadType { Straight = 0, RoadEnd = 1, Unknown = 2 };

// OpenCV 8-bit HSV: H in [0,179], S/V in [0,255]. Bounds are inclusive.
// For each channel, min > max means [min,channel_max] OR [0,max];
// min == max matches only that value. All three channels must match.
struct HsvRange {
  int h_min = 70, h_max = 10;  // H in [70,179] OR [0,10].
  int s_min = 10, s_max = 150;
  int v_min = 40, v_max = 230;
};

struct CenterlineFilterConfig {
  bool enabled = true;
  double max_residual_fraction = 0.025;  // Horizontal residual / processing image width.
};

struct Config {
  int input_width = 1280;
  int input_height = 720;
  int process_width = 640;
  // Main road search window in original input-image pixels.
  cv::Rect roi{0, 360, 1280, 360};
  std::array<cv::Point2f, 4> perspective_src{};
  std::array<cv::Point2f, 4> perspective_dst{};
  bool use_perspective = false;

  HsvRange road_hsv{};
  std::optional<HsvRange> road_end_hsv;  // Extra road pixels for fitting and RoadEnd coverage.
  int white_s_max = 45;
  int white_v_min = 170;
  float occlusion_max_gap_fraction = 0.25f;
  float min_road_width_fraction = 0.06f;
  float max_road_width_fraction = 0.85f;
  // Near-vehicle trigger window in ORIGINAL input-image pixels.
  cv::Rect road_end_roi{400, 540, 480, 120};
  int threads = 1;  // Worker count; the ROI has threads * 6 horizontal strips.
  int morphology_pixels = 3;
  CenterlineFilterConfig centerline_filter;

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
  bool fit_used = false;  // Measured scan point retained for the final centerline.
  float coverage = 0.0f;  // Strip mask fill; road_end_region uses the non-mask area fraction.
  cv::Point2f midpoint{};  // Geometric centroid of all nonzero mask pixels in this strip.
};

struct RoadResult {
  std::int64_t timestamp_ms = 0;
  bool valid = false;  // Measured steering geometry; independent of RoadEnd.
  RoadType type = RoadType::Unknown;
  RoadType raw_type = RoadType::Unknown;
  float confidence = 0.0f;
  int raw_point_count = 0;
  int fit_point_count = 0;
  std::string centerline_filter_status = "no_points";
  double fit_residual_px = 0.0;  // RMS horizontal residual of retained points in processing pixels.
  float classification_confidence = 0.0f;
  bool road_end_detected = false;  // Two retained centroids and >=80% non-mask in the end box.
  BlockObservation road_end_region;
  std::vector<cv::Point2f> road_end_boundary;  // Retained for compatibility; area detection leaves empty.
  bool occlusion_detected = false;
  float lateral_error = 0.0f;  // Positive: target center is to the vehicle's right.
  float heading_error_rad = 0.0f;  // Positive: path turns to the right.
  std::vector<cv::Point2f> left_boundary;   // Original input-image coordinates.
  std::vector<cv::Point2f> right_boundary;
  std::vector<cv::Point2f> centerline;
  std::vector<BlockObservation> scan_blocks;
  cv::Mat white_mask;  // Color candidates; not all white pixels are occlusions.
  cv::Mat occlusion_mask;  // White inside a geometrically supported road corridor.
  cv::Mat bridge_mask;  // Association only; never used for measured centers.
  cv::Mat road_mask;  // Combined road/end-color mask for fitting and RoadEnd coverage.
  cv::Mat road_end_color_mask;  // HSV-matched end pixels; empty when not configured/no BGR.
  // Homography from road_mask to original input pixels. Empty: resize only.
  cv::Mat road_mask_to_source;
  std::vector<cv::Point2f> roi_polygon;
};

const char* toString(RoadType type);

class WorkerPool;

class RoadDetector {
 public:
  explicit RoadDetector(Config config);
  ~RoadDetector();
  // temporal is retained for compatibility; all results use the current frame only.
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
  cv::Mat mask_to_source_;
  cv::Mat valid_view_mask_;
  cv::Rect roi_processed_;
  std::vector<cv::Point2f> roi_polygon_;
  std::vector<cv::Rect> scan_bands_processed_;
  std::unique_ptr<WorkerPool> workers_;
  cv::Mat road_end_view_mask_;  // Source trigger rectangle transformed as a polygon.
  cv::Rect road_end_region_processed_;
  std::vector<cv::Point2f> road_end_polygon_;
  int road_end_valid_pixels_ = 0;

  std::vector<cv::Point2f> sourcePolygon(const cv::Rect& rect) const;
  void prepareView(const cv::Mat& bgr, cv::Mat& processed, cv::Mat& view) const;
  cv::Mat segment(const cv::Mat& view) const;
  RoadResult analyzeMask(const cv::Mat& mask, std::int64_t timestamp_ms,
                         const cv::Mat* processed_bgr);
  void detectRoadEnd(bool can_fit_line, RoadResult& result) const;
  cv::Point2f sourcePoint(const cv::Point2f& processed) const;
};

}  // namespace road_finder
