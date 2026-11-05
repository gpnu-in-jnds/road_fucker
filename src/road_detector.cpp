#include "road_finder/road_detector.hpp"
#include "centerline_filter.hpp"
#include "worker_pool.hpp"
#include "occlusion.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

#if CV_VERSION_MAJOR >= 5
// OpenCV 5 moved contour and perspective geometry out of imgproc.
#include <opencv2/geometry.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace road_finder {
namespace {

constexpr int kReferenceWidth = 128;

template <typename T>
void readValue(const cv::FileNode& node, const char* key, T& value) {
  if (!node[key].empty()) node[key] >> value;
}

HsvRange readHsvRange(const cv::FileNode& node, const HsvRange& fallback, const char* key) {
  if (node.empty()) return fallback;
  if (node.size() != 6)
    throw std::runtime_error(std::string(key) +
                             " needs [H_min,H_max,S_min,S_max,V_min,V_max]");
  return {static_cast<int>(node[0]), static_cast<int>(node[1]),
          static_cast<int>(node[2]), static_cast<int>(node[3]),
          static_cast<int>(node[4]), static_cast<int>(node[5])};
}

cv::Rect readRect(const cv::FileNode& node, const char* key, const cv::Rect& fallback) {
  if (node.empty()) return fallback;
  if (node.size() != 4)
    throw std::runtime_error(std::string(key) + " needs [x,y,width,height]");
  return {static_cast<int>(node[0]), static_cast<int>(node[1]),
          static_cast<int>(node[2]), static_cast<int>(node[3])};
}

cv::Rect roiFromFraction(const cv::FileNode& node, int input_width, int input_height) {
  if (node.size() != 4) throw std::runtime_error("roi_fraction needs [x,y,width,height]");
  const auto pixels = [](double fraction, int size) {
    return static_cast<int>(std::lround(fraction * size));
  };
  return {pixels(static_cast<double>(node[0]), input_width),
          pixels(static_cast<double>(node[1]), input_height),
          pixels(static_cast<double>(node[2]), input_width),
          pixels(static_cast<double>(node[3]), input_height)};
}

std::array<cv::Point2f, 4> readPoints(const cv::FileNode& node) {
  if (node.size() != 4) throw std::runtime_error("perspective points need four [x,y] pairs");
  std::array<cv::Point2f, 4> points{};
  for (int i = 0; i < 4; ++i) {
    if (node[i].size() != 2) throw std::runtime_error("invalid perspective point");
    points[i].x = static_cast<float>(static_cast<double>(node[i][0]));
    points[i].y = static_cast<float>(static_cast<double>(node[i][1]));
  }
  return points;
}

bool validHsvRange(const HsvRange& range) {
  const auto inByteRange = [](int value) { return value >= 0 && value <= 255; };
  return (range.h_min >= 0 && range.h_min <= 179) &&
         (range.h_max >= 0 && range.h_max <= 179) &&
         inByteRange(range.s_min) && inByteRange(range.s_max) &&
         inByteRange(range.v_min) && inByteRange(range.v_max);
}

struct Blob {
  bool found = false;
  int area = 0;
  cv::Rect bounds;
  cv::Point2f center;
};

cv::Mat hsvMask(const cv::Mat& bgr, const HsvRange& range) {
  cv::Mat hsv, mask;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  const std::array<int, 3> first{{range.h_min, range.s_min, range.v_min}};
  const std::array<int, 3> last{{range.h_max, range.s_max, range.v_max}};
  const std::array<int, 3> maximum{{179, 255, 255}};
  std::array<cv::Mat, 3> channels;
  cv::split(hsv, channels.data());
  for (int i = 0; i < 3; ++i) {
    cv::Mat channel_mask;
    if (first[i] <= last[i]) {
      cv::inRange(channels[i], cv::Scalar(first[i]), cv::Scalar(last[i]), channel_mask);
    } else {
      cv::Mat upper;
      cv::inRange(channels[i], cv::Scalar(0), cv::Scalar(last[i]), channel_mask);
      cv::inRange(channels[i], cv::Scalar(first[i]), cv::Scalar(maximum[i]), upper);
      cv::bitwise_or(channel_mask, upper, channel_mask);
    }
    // Each channel may contain two intervals; all three channels must match.
    if (i == 0) mask = channel_mask;
    else cv::bitwise_and(mask, channel_mask, mask);
  }
  return mask;
}

void rejectDeprecatedKeys(const cv::FileNode& root, const Config& config) {
  // Obsolete geometry and endpoint tuning keys are intentionally ignored.
  // Endpoints now use a fixed 80% non-mask threshold without temporal filtering.
  if (!root["process_height"].empty())
    throw std::runtime_error(
        "process_height is no longer used: the processing height follows the input aspect "
        "ratio, remove the key (process_width " + std::to_string(config.process_width) +
        " -> " + std::to_string(config.processHeight()) + ")");
  if (!root["roi_fraction"].empty()) {
    const cv::Rect roi = roiFromFraction(root["roi_fraction"], config.input_width,
                                         config.input_height);
    throw std::runtime_error(
        "roi_fraction was replaced by roi in input-image pixels; use roi: [" +
        std::to_string(roi.x) + ", " + std::to_string(roi.y) + ", " +
        std::to_string(roi.width) + ", " + std::to_string(roi.height) + "]");
  }
}

}  // namespace

Config loadConfig(const std::string& path) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) throw std::runtime_error("cannot open config: " + path);

  Config config;
  const cv::FileNode root = fs.root();
  for (const char* key : {"road_lab", "stop_lab", "scan_bands", "left_marker_roi",
                          "right_marker_roi", "segmentation_mode", "road_contrast_min",
                          "side_road_hsv", "side_left_end_reference", "side_right_begin_reference",
                          "marker_min_coverage", "classification_confirm_frames",
                          "classification_hold_frames", "indicator_roi",
                          "indicator_min_area_reference", "left_marker_rois", "right_marker_rois",
                          "stop_roi", "stop_hsv", "stop_detection_enabled", "stop_min_area_reference"}) {
    if (!root[key].empty())
      throw std::runtime_error(std::string(key) +
          " is no longer supported: use one fixed road_hsv and road_end_roi; "
          "strips follow ROI and --threads");
  }
  readValue(root, "input_width", config.input_width);
  readValue(root, "input_height", config.input_height);
  readValue(root, "process_width", config.process_width);
  readValue(root, "min_road_width_fraction", config.min_road_width_fraction);
  readValue(root, "max_road_width_fraction", config.max_road_width_fraction);
  readValue(root, "morphology_pixels", config.morphology_pixels);
  config.road_end_roi = readRect(root["road_end_roi"], "road_end_roi", config.road_end_roi);
  config.road_hsv = readHsvRange(root["road_hsv"], config.road_hsv, "road_hsv");
  if (!root["road_end_hsv"].empty())
    config.road_end_hsv = readHsvRange(root["road_end_hsv"], config.road_hsv, "road_end_hsv");
  readValue(root, "white_s_max", config.white_s_max);
  readValue(root, "white_v_min", config.white_v_min);
  readValue(root, "occlusion_max_gap_fraction", config.occlusion_max_gap_fraction);
  config.roi = readRect(root["roi"], "roi", config.roi);
  const cv::FileNode filter = root["centerline_filter"];
  if (!filter.empty()) {
    if (!filter.isMap()) throw std::runtime_error("centerline_filter must be a mapping");
    if (!filter["enabled"].empty()) {
      const auto enabled = filter["enabled"];
      if (!enabled.isInt() || (static_cast<int>(enabled) != 0 && static_cast<int>(enabled) != 1))
        throw std::runtime_error("centerline_filter.enabled must be 0 or 1");
      config.centerline_filter.enabled = static_cast<int>(enabled) != 0;
    }
    readValue(filter, "max_residual_fraction", config.centerline_filter.max_residual_fraction);
  }

  const cv::FileNode src = root["perspective_src"];
  const cv::FileNode dst = root["perspective_dst"];
  if (!src.empty() || !dst.empty()) {
    if (src.empty() || dst.empty())
      throw std::runtime_error("both perspective point sets are required");
    config.perspective_src = readPoints(src);
    config.perspective_dst = readPoints(dst);
    config.use_perspective = true;
  }

  validateConfig(config);
  rejectDeprecatedKeys(root, config);
  return config;
}

void validateConfig(const Config& config) {
  if (config.input_width <= 0 || config.input_height <= 0)
    throw std::runtime_error("input_width and input_height must be positive");
  if (config.process_width < 80)
    throw std::runtime_error("process_width must be at least 80");
  if (config.processHeight() < 60)
    throw std::runtime_error("processing height is too small");
  if (config.roi.width <= 0 || config.roi.height <= 0 || config.roi.x < 0 ||
      config.roi.y < 0 || config.roi.x + config.roi.width > config.input_width ||
      config.roi.y + config.roi.height > config.input_height)
    throw std::runtime_error("roi must stay inside the input image");
  if (config.roi.y + config.roi.height < static_cast<int>(0.8 * config.input_height))
    throw std::runtime_error("roi must reach at least 80% of the input height");
  if (!validHsvRange(config.road_hsv))
    throw std::runtime_error("invalid HSV range: H endpoints within [0,179], S/V endpoints within [0,255]");
  if (config.road_end_hsv && !validHsvRange(*config.road_end_hsv))
    throw std::runtime_error("invalid road_end_hsv: H endpoints within [0,179], S/V endpoints within [0,255]");
  const cv::Rect image_bounds(0, 0, config.input_width, config.input_height);
  if (config.road_end_roi.empty() || (config.road_end_roi & image_bounds) != config.road_end_roi)
    throw std::runtime_error("road_end_roi must be nonempty and inside the input image");
  const auto fraction = [](float value) {
    return std::isfinite(value) && value > 0.0f && value <= 1.0f;
  };
  if (!fraction(config.occlusion_max_gap_fraction))
    throw std::runtime_error("gap fractions must be within (0,1]");
  if (!std::isfinite(config.centerline_filter.max_residual_fraction) ||
      config.centerline_filter.max_residual_fraction <= 0.0 ||
      config.centerline_filter.max_residual_fraction > 1.0)
    throw std::runtime_error("centerline_filter.max_residual_fraction must be within (0,1]");
  if (config.white_s_max < 0 || config.white_s_max > 255 ||
      config.white_v_min < 0 || config.white_v_min > 255)
    throw std::runtime_error("white thresholds must be within [0,255]");
  if (!std::isfinite(config.min_road_width_fraction) ||
      !std::isfinite(config.max_road_width_fraction) ||
      config.min_road_width_fraction <= 0.0f ||
      config.max_road_width_fraction >= 1.0f ||
      config.min_road_width_fraction >= config.max_road_width_fraction)
    throw std::runtime_error("road width fractions must satisfy 0 < min < max < 1");
  if (config.morphology_pixels < 1 || config.morphology_pixels > 31)
    throw std::runtime_error("morphology_pixels must be within [1,31]");

  if (config.threads < 1 || config.threads > std::numeric_limits<int>::max() / 6)
    throw std::runtime_error("threads must be positive and threads * 6 must fit in an integer");
  if (config.use_perspective) {
    std::vector<cv::Point2f> src(config.perspective_src.begin(), config.perspective_src.end());
    std::vector<cv::Point2f> dst(config.perspective_dst.begin(), config.perspective_dst.end());
    for (const auto& points : {src, dst}) {
      for (const auto& point : points)
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
          throw std::runtime_error("perspective coordinates must be finite");
      if (!cv::isContourConvex(points) || std::abs(cv::contourArea(points)) < 100)
        throw std::runtime_error("perspective quadrilateral must be convex with non-tiny area");
    }
  }
}

const char* toString(RoadType type) {
  switch (type) {
    case RoadType::Straight: return "STRAIGHT";
    case RoadType::RoadEnd: return "ROAD_END";
    case RoadType::Unknown: return "UNKNOWN";
  }
  return "UNKNOWN";
}

RoadDetector::RoadDetector(Config config) : config_(std::move(config)) {
  validateConfig(config_);
  const int width = config_.process_width;
  const int height = config_.processHeight();
  const float sx = static_cast<float>(width) / config_.input_width;
  const float sy = static_cast<float>(height) / config_.input_height;

  if (config_.use_perspective) {
    std::vector<cv::Point2f> src(config_.perspective_src.begin(), config_.perspective_src.end());
    std::vector<cv::Point2f> dst(config_.perspective_dst.begin(), config_.perspective_dst.end());
    for (auto& point : src) {
      point.x *= sx;
      point.y *= sy;
    }
    for (auto& point : dst) {
      point.x *= sx;
      point.y *= sy;
    }
    warp_ = cv::getPerspectiveTransform(src, dst);
    inverse_warp_ = warp_.inv();
    cv::Mat source_scale = cv::Mat::eye(3, 3, CV_64F);
    source_scale.at<double>(0, 0) = 1.0 / sx;
    source_scale.at<double>(1, 1) = 1.0 / sy;
    mask_to_source_ = source_scale * inverse_warp_;
  }
  valid_view_mask_ = cv::Mat(height, width, CV_8UC1, cv::Scalar(255));
  if (config_.use_perspective) {
    cv::Mat support;
    cv::warpPerspective(valid_view_mask_, support, warp_, valid_view_mask_.size(),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    valid_view_mask_ = support;
  }

  std::vector<cv::Point2f> corners{
      {config_.roi.x * sx, config_.roi.y * sy},
      {(config_.roi.x + config_.roi.width) * sx, config_.roi.y * sy},
      {(config_.roi.x + config_.roi.width) * sx,
       (config_.roi.y + config_.roi.height) * sy},
      {config_.roi.x * sx, (config_.roi.y + config_.roi.height) * sy}};
  cv::Rect2f processed(config_.roi.x * sx, config_.roi.y * sy,
                       config_.roi.width * sx, config_.roi.height * sy);
  if (config_.use_perspective) {
    cv::perspectiveTransform(corners, corners, warp_);
    float min_x = corners.front().x;
    float max_x = corners.front().x;
    float min_y = corners.front().y;
    float max_y = corners.front().y;
    for (const cv::Point2f& point : corners) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    processed = cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
  }
  const cv::Rect2f image(0, 0, width, height);
  if ((processed & image).area() <= 0)
    throw std::runtime_error("roi lies outside the processed image");
  processed &= image;
  roi_processed_ = {cvRound(processed.x), cvRound(processed.y),
                    cvRound(processed.width), cvRound(processed.height)};
  roi_processed_ &= cv::Rect(0, 0, width, height);

  std::vector<cv::Point2f> used{
      {static_cast<float>(roi_processed_.x), static_cast<float>(roi_processed_.y)},
      {static_cast<float>(roi_processed_.x + roi_processed_.width),
       static_cast<float>(roi_processed_.y)},
      {static_cast<float>(roi_processed_.x + roi_processed_.width),
       static_cast<float>(roi_processed_.y + roi_processed_.height)},
      {static_cast<float>(roi_processed_.x),
       static_cast<float>(roi_processed_.y + roi_processed_.height)}};
  if (config_.use_perspective) cv::perspectiveTransform(used, used, inverse_warp_);
  for (const cv::Point2f& point : used)
    roi_polygon_.emplace_back(point.x / sx, point.y / sy);

  const int count = config_.threads * 6;
  if (count > roi_processed_.height)
    throw std::runtime_error("threads * 6 exceeds the processed ROI height");
  for (int i = 0; i < count; ++i) {
    const int top = roi_processed_.y + static_cast<int>(static_cast<std::int64_t>(i) * roi_processed_.height / count);
    const int bottom = roi_processed_.y + static_cast<int>(static_cast<std::int64_t>(i + 1) * roi_processed_.height / count);
    scan_bands_processed_.emplace_back(roi_processed_.x, top, roi_processed_.width, bottom - top);
  }
  const cv::Rect& end = config_.road_end_roi;
  road_end_polygon_ = {{static_cast<float>(end.x), static_cast<float>(end.y)},
      {static_cast<float>(end.x + end.width), static_cast<float>(end.y)},
      {static_cast<float>(end.x + end.width), static_cast<float>(end.y + end.height)},
      {static_cast<float>(end.x), static_cast<float>(end.y + end.height)}};
  // Rasterize the source rectangle with half-open pixel bounds, then apply
  // the same resize/warp as the road mask. Warp padding never supplies evidence.
  cv::Mat source_end_mask = cv::Mat::zeros(config_.input_height, config_.input_width, CV_8UC1);
  source_end_mask(config_.road_end_roi).setTo(255);
  cv::resize(source_end_mask, road_end_view_mask_, {width, height}, 0, 0, cv::INTER_NEAREST);
  if (config_.use_perspective) {
    cv::Mat transformed;
    cv::warpPerspective(road_end_view_mask_, transformed, warp_, {width, height},
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    road_end_view_mask_ = transformed;
  }
  cv::bitwise_and(road_end_view_mask_, valid_view_mask_, road_end_view_mask_);
  road_end_valid_pixels_ = cv::countNonZero(road_end_view_mask_);
  road_end_region_processed_ = cv::boundingRect(road_end_view_mask_);
  workers_ = std::make_unique<WorkerPool>(config_.threads);
}

RoadDetector::~RoadDetector() = default;

std::vector<cv::Point2f> RoadDetector::sourcePolygon(const cv::Rect& rect) const {
  return {sourcePoint({static_cast<float>(rect.x), static_cast<float>(rect.y)}),
          sourcePoint({static_cast<float>(rect.x + rect.width), static_cast<float>(rect.y)}),
          sourcePoint({static_cast<float>(rect.x + rect.width), static_cast<float>(rect.y + rect.height)}),
          sourcePoint({static_cast<float>(rect.x), static_cast<float>(rect.y + rect.height)})};
}

void RoadDetector::prepareView(const cv::Mat& bgr, cv::Mat& processed,
                              cv::Mat& view) const {
  cv::resize(bgr, processed, cv::Size(config_.process_width, config_.processHeight()),
             0, 0, cv::INTER_LINEAR);
  if (config_.use_perspective)
    cv::warpPerspective(processed, view, warp_, processed.size(), cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  else
    view = processed;
}

cv::Mat RoadDetector::segment(const cv::Mat& view) const {
  cv::Mat blurred;
  cv::GaussianBlur(view, blurred, cv::Size(3, 3), 0);
  cv::Mat mask = hsvMask(blurred, config_.road_hsv);
  const int kernel_size = config_.morphology_pixels | 1;
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::bitwise_and(mask, valid_view_mask_, mask);
  return mask;
}

RoadResult RoadDetector::process(const cv::Mat& bgr, std::int64_t timestamp_ms, bool /*temporal*/) {
  if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols != config_.input_width ||
      bgr.rows != config_.input_height)
    throw std::invalid_argument("BGR frame size/type does not match configured input size");

  cv::Mat processed, view;
  prepareView(bgr, processed, view);
  RoadResult result = analyzeMask(segment(view), timestamp_ms, &view);
  result.roi_polygon = roi_polygon_;
  result.raw_type = result.type;
  return result;
}

RoadResult RoadDetector::processMask(const cv::Mat& road_mask,
                                     std::int64_t timestamp_ms, bool /*temporal*/) {
  if (road_mask.empty() || road_mask.type() != CV_8UC1 ||
      road_mask.cols != config_.process_width ||
      road_mask.rows != config_.processHeight())
    throw std::invalid_argument("mask size/type does not match configured processing size");
  cv::Mat measured;
  cv::bitwise_and(road_mask, valid_view_mask_, measured);
  RoadResult result = analyzeMask(measured, timestamp_ms, nullptr);
  result.roi_polygon = roi_polygon_;
  result.raw_type = result.type;
  return result;
}

RoadResult RoadDetector::analyzeMask(const cv::Mat& mask, std::int64_t timestamp_ms,
                                     const cv::Mat* processed_bgr) {
  RoadResult result;
  result.timestamp_ms = timestamp_ms;
  result.road_mask = mask.clone();
  if (processed_bgr != nullptr && config_.road_end_hsv) {
    // Like the white candidate threshold, use actual pixels in the processed
    // view. These are measured fitting pixels, not synthetic bridge pixels.
    result.road_end_color_mask = hsvMask(*processed_bgr, *config_.road_end_hsv);
    cv::bitwise_and(result.road_end_color_mask, valid_view_mask_, result.road_end_color_mask);
    cv::bitwise_or(result.road_mask, result.road_end_color_mask, result.road_mask);
  }
  result.road_mask_to_source = mask_to_source_.clone();
  result.road_end_region.polygon = road_end_polygon_;
  result.road_end_region.processed_region = road_end_region_processed_;
  result.white_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
  if (processed_bgr != nullptr) {
    result.white_mask = hsvMask(*processed_bgr,
        {0, 179, 0, config_.white_s_max, config_.white_v_min, 255});
    cv::bitwise_and(result.white_mask, valid_view_mask_, result.white_mask);
    result.white_mask.setTo(0, result.road_mask);  // Measured fitting pixels are never unknown.
  }
  const auto occlusion = findOcclusions(result.road_mask, result.white_mask, roi_processed_, config_);
  result.occlusion_mask = occlusion.unknown;
  result.bridge_mask = occlusion.bridges;
  result.occlusion_detected = cv::countNonZero(result.occlusion_mask) > 0;
  const int block_count = static_cast<int>(scan_bands_processed_.size());
  std::vector<Blob> samples(block_count);
  result.scan_blocks.resize(block_count);
  for (int i = 0; i < block_count; ++i) {
    result.scan_blocks[i].processed_region = scan_bands_processed_[i];
    result.scan_blocks[i].polygon = sourcePolygon(scan_bands_processed_[i]);
  }

  // A strip contributes its centroid only above 60% fill of its mask bounds.
  // All nonzero pixels contribute, including disconnected and edge-touching regions.
  workers_->run(block_count, [&](int i) {
    const cv::Rect& band = scan_bands_processed_[i];
    const cv::Mat block_mask = result.road_mask(band);
    const cv::Rect bounds = cv::boundingRect(block_mask);
    if (bounds.empty()) return;
    const cv::Moments moments = cv::moments(block_mask, true);
    const int area = cvRound(moments.m00);
    result.scan_blocks[i].coverage = static_cast<float>(area) / bounds.area();
    // Integer products keep exactly 60% on the rejected side of the threshold.
    if (static_cast<std::int64_t>(area) * 5 <=
        static_cast<std::int64_t>(bounds.area()) * 3) return;
    samples[i] = {true, area,
                  bounds + band.tl(),
                  {static_cast<float>(band.x + moments.m10 / moments.m00),
                   static_cast<float>(band.y + moments.m01 / moments.m00)}};
  });
  std::vector<cv::Point2f> points;
  std::vector<int> sample_indices;
  points.reserve(samples.size());
  sample_indices.reserve(samples.size());
  const double sx = static_cast<double>(mask.cols) / kReferenceWidth;
  for (int i = 0; i < block_count; ++i) {
    const Blob& sample = samples[i];
    if (!sample.found) continue;
    result.scan_blocks[i].found = true;
    result.scan_blocks[i].midpoint = sourcePoint(sample.center);
    points.push_back(sample.center);
    sample_indices.push_back(i);
  }

  const auto fit = fitCenterline(points, mask.cols, config_.centerline_filter);
  result.raw_point_count = static_cast<int>(points.size());
  result.fit_point_count = static_cast<int>(fit.used.size());
  result.centerline_filter_status = fit.status;
  result.fit_residual_px = fit.rms;
  if (fit.used.empty()) {
    detectRoadEnd(mask, false, result);
    return result;
  }

  std::vector<int> widths_reference;
  double fill_sum = 0.0;
  for (const auto i : fit.used) {
    const int index = sample_indices[i];
    result.scan_blocks[index].fit_used = true;
    const Blob& sample = samples[index];
    widths_reference.push_back(std::max(1, cvRound(sample.bounds.width / sx)));
    fill_sum += static_cast<double>(sample.area) / sample.bounds.area();
  }

  // With filtering enabled, steering follows the refitted line, including when
  // the bottommost raw observation was rejected. Disabled keeps the legacy offset.
  const double bottom_x = config_.centerline_filter.enabled
      ? fit.xAt(points[fit.used.back()].y) : points.back().x;
  const float bottom_x_reference = bottom_x / sx;
  const float offset_reference = bottom_x_reference - kReferenceWidth / 2.0f;
  std::sort(widths_reference.begin(), widths_reference.end());
  const float median_width = static_cast<float>(widths_reference[widths_reference.size() / 2]);
  result.lateral_error = offset_reference / std::max(1.0f, median_width / 2.0f);
  const double fit_quality = 1.0 / (1.0 + fit.rms / std::max(3.0, median_width * sx * 0.15));
  result.confidence = static_cast<float>(
      fit.used.size() / static_cast<double>(samples.size()) * fit_quality * fill_sum / fit.used.size());
  result.heading_error_rad = static_cast<float>(std::atan(-fit.dx_dy));
  result.valid = true;

  // One centroid cannot determine slope. In that case expose the vertical
  // line through it; with multiple strips, report the fitted line at each
  // retained row. All points are returned in source-image coordinates.
  if (fit.used.size() == 1) {
    const float x = points[fit.used.front()].x;
    result.centerline.push_back(sourcePoint(
        {x, static_cast<float>(roi_processed_.y)}));
    result.centerline.push_back(sourcePoint(
        {x, static_cast<float>(roi_processed_.y + roi_processed_.height - 1)}));
  }
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!result.scan_blocks[i].fit_used) continue;
    const float y = samples[i].center.y;
    const float fitted_x = static_cast<float>(fit.xAt(y));
    if (fit.used.size() > 1) result.centerline.push_back(sourcePoint({fitted_x, y}));
    result.left_boundary.push_back(
        sourcePoint({static_cast<float>(samples[i].bounds.x), y}));
    result.right_boundary.push_back(
        sourcePoint({static_cast<float>(samples[i].bounds.x + samples[i].bounds.width - 1), y}));
  }

  result.type = RoadType::Straight;
  result.classification_confidence = result.confidence;
  // Count retained measurements, not rejected points or single-point fallback endpoints.
  // End colors support geometry but must not fill the non-road area used by
  // the endpoint trigger. The original road-only mask remains the evidence.
  detectRoadEnd(mask, fit.used.size() >= 2 && fit.y_variance_sum > 1e-6, result);
  return result;
}

void RoadDetector::detectRoadEnd(const cv::Mat& mask, bool can_fit_line,
                                 RoadResult& result) const {
  if (road_end_valid_pixels_ == 0) return;
  cv::Mat road_pixels;
  cv::compare(mask(road_end_region_processed_), cv::Scalar(0), road_pixels, cv::CMP_NE);
  cv::bitwise_and(road_pixels, road_end_view_mask_(road_end_region_processed_), road_pixels);
  const int non_mask_pixels = road_end_valid_pixels_ - cv::countNonZero(road_pixels);
  const float coverage = static_cast<float>(non_mask_pixels) / road_end_valid_pixels_;
  result.road_end_region.coverage = coverage;
  // Integer arithmetic includes exactly 80%, even for masks with values other than 255.
  if (!can_fit_line || static_cast<std::int64_t>(non_mask_pixels) * 5 <
                       static_cast<std::int64_t>(road_end_valid_pixels_) * 4) return;
  result.road_end_detected = result.road_end_region.found = true;
  result.type = RoadType::RoadEnd;
  result.classification_confidence = coverage;
}

cv::Point2f RoadDetector::sourcePoint(const cv::Point2f& processed) const {
  cv::Point2f point = processed;
  if (config_.use_perspective) {
    std::vector<cv::Point2f> in{point}, out;
    cv::perspectiveTransform(in, out, inverse_warp_);
    point = out.front();
  }
  point.x *= static_cast<float>(config_.input_width) / config_.process_width;
  point.y *= static_cast<float>(config_.input_height) / config_.processHeight();
  return point;
}

}  // namespace road_finder
