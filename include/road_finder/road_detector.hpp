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

struct HsvRange {
  int h_min = 0, h_max = 179;
  int s_min = 0, s_max = 70;
  int v_min = 35, v_max = 220;
};

struct Config {
  int input_width = 1280;
  int input_height = 720;
  // Processing resolution: only the width is configured, the height follows the
  // input aspect ratio so the two can never disagree. Pixel-unit thresholds
  // (edge_line_residual_px, morphology_pixels, ...) use this processed image.
  int process_width = 640;
  // Detection window in original input-image pixels [x, y, width, height]; the
  // same coordinate system as the preview, --overlay exports and the
  // perspective points. Converted to the processed (and warped) image internally.
  cv::Rect roi{26, 86, 1229, 612};
  std::array<cv::Point2f, 4> perspective_src{};
  std::array<cv::Point2f, 4> perspective_dst{};
  bool use_perspective = false;
  bool equalize_value = false;
  HsvRange road_hsv{};
  float color_distance_max = 85.0f;
  float color_background_ratio = 1.15f;
  float chroma_row_ratio = 0.20f;
  float chroma_min_contrast = 3.0f;
  float edge_min_score = 3.5f;
  float edge_line_residual_px = 8.0f;
  float edge_min_inlier_ratio = 0.48f;
  float edge_min_span_fraction = 0.28f;
  int morphology_pixels = 3;
  float max_white_gap_width_ratio = 0.22f;
  float min_road_width_fraction = 0.08f;
  float max_road_width_fraction = 0.92f;
  float junction_min_arm_widths = 0.65f;
  int junction_vote_frames = 3;
  int junction_vote_window = 5;
  float lookahead_fraction = 0.55f;
  float minimum_confidence = 0.38f;

  int processHeight() const {
    return static_cast<int>(std::lround(static_cast<double>(input_height) * process_width /
                                        input_width));
  }
};

Config loadConfig(const std::string& path);
void validateConfig(const Config& config);

struct RoadResult {
  std::int64_t timestamp_ms = 0;
  bool valid = false;
  RoadType type = RoadType::UNKNOWN;
  unsigned available_directions = 0;
  float confidence = 0.0f;
  float lateral_error = 0.0f;  // Positive: target center is to the vehicle's right.
  float heading_error_rad = 0.0f;  // Positive: path turns to the right.
  std::vector<cv::Point2f> left_boundary;   // 1280x720 source coordinates.
  std::vector<cv::Point2f> right_boundary;
  std::vector<cv::Point2f> centerline;
  cv::Mat road_mask;  // Optional debug image, at process resolution.
  // Detection window actually used for this frame, as four corners in input-image
  // pixels. Drawn by the preview and by --overlay exports. Not part of the JSON.
  std::vector<cv::Point2f> roi_polygon;
};

const char* toString(RoadType type);

class RoadDetector {
 public:
  explicit RoadDetector(Config config);
  RoadResult process(const cv::Mat& bgr, std::int64_t timestamp_ms);
  // Public for repeatable tests and camera-specific segmenters. Mask must be
  // CV_8UC1 at the configured processing size, with road pixels nonzero.
  RoadResult processMask(const cv::Mat& road_mask, std::int64_t timestamp_ms);
  // Detection window in processed (and, with perspective, warped) pixels.
  const cv::Rect& processRoi() const { return roi_processed_; }
  // Detection window corners in input-image pixels, for debug drawing.
  const std::vector<cv::Point2f>& roiPolygon() const { return roi_polygon_; }

 private:
  Config config_;
  cv::Mat warp_;
  cv::Mat inverse_warp_;
  cv::Rect roi_processed_;
  std::vector<cv::Point2f> roi_polygon_;
  std::vector<RoadType> recent_types_;
  float previous_center_x_ = -1.0f;
  float vehicle_x_ = -1.0f;
  cv::Mat segment(const cv::Mat& bgr) const;
  RoadResult analyzeMask(const cv::Mat& mask, std::int64_t timestamp_ms);
  bool refineWithEdges(const cv::Mat& processed_bgr, RoadResult& result);
  RoadType stabilize(RoadType current);
  cv::Point2f sourcePoint(const cv::Point2f& processed) const;
};

}  // namespace road_finder
