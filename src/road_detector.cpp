#include "road_finder/road_detector.hpp"

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>

#if CV_VERSION_MAJOR >= 5
// OpenCV 5 moved contour and perspective geometry out of imgproc.
#include <opencv2/geometry.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace road_finder {
namespace {

constexpr int kReferenceWidth = 128;
constexpr int kReferenceHeight = 90;

template <typename T>
void readValue(const cv::FileNode& node, const char* key, T& value) {
  if (!node[key].empty()) node[key] >> value;
}

LabRange readLabRange(const cv::FileNode& node, const LabRange& fallback, const char* key) {
  if (node.empty()) return fallback;
  if (node.size() != 6)
    throw std::runtime_error(std::string(key) +
                             " needs [L_min,L_max,a_min,a_max,b_min,b_max]");
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

bool validLabRange(const LabRange& range) {
  const auto inByteRange = [](int value) { return value >= 0 && value <= 255; };
  return inByteRange(range.l_min) && inByteRange(range.l_max) &&
         inByteRange(range.a_min) && inByteRange(range.a_max) &&
         inByteRange(range.b_min) && inByteRange(range.b_max) &&
         range.l_min <= range.l_max && range.a_min <= range.a_max &&
         range.b_min <= range.b_max;
}

cv::Rect scaleReferenceRect(const cv::Rect& reference, cv::Size size) {
  const double sx = static_cast<double>(size.width) / kReferenceWidth;
  const double sy = static_cast<double>(size.height) / kReferenceHeight;
  const int left = static_cast<int>(std::floor(reference.x * sx));
  const int top = static_cast<int>(std::floor(reference.y * sy));
  const int right = static_cast<int>(std::ceil((reference.x + reference.width) * sx));
  const int bottom = static_cast<int>(std::ceil((reference.y + reference.height) * sy));
  return {left, top, right - left, bottom - top};
}

int scaledArea(int reference_area, cv::Size size) {
  const double scale = static_cast<double>(size.width) * size.height /
                       (kReferenceWidth * kReferenceHeight);
  return std::max(1, cvRound(reference_area * scale));
}

struct Blob {
  bool found = false;
  int area = 0;
  cv::Rect bounds;
  cv::Point2f center;
};

Blob largestBlob(const cv::Mat& binary, cv::Rect roi, bool foreground,
                 int min_area) {
  roi &= cv::Rect(0, 0, binary.cols, binary.rows);
  if (roi.empty()) return {};

  cv::Mat selected;
  const cv::Mat view = binary(roi);
  if (foreground)
    cv::compare(view, cv::Scalar(0), selected, cv::CMP_GT);
  else
    cv::compare(view, cv::Scalar(0), selected, cv::CMP_EQ);

  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(
      selected, labels, stats, centroids, 8, CV_32S);
  int best = -1;
  int best_area = min_area - 1;
  for (int label = 1; label < count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area > best_area) {
      best = label;
      best_area = area;
    }
  }
  if (best < 0) return {};

  Blob blob;
  blob.found = true;
  blob.area = best_area;
  blob.bounds = cv::Rect(stats.at<int>(best, cv::CC_STAT_LEFT) + roi.x,
                         stats.at<int>(best, cv::CC_STAT_TOP) + roi.y,
                         stats.at<int>(best, cv::CC_STAT_WIDTH),
                         stats.at<int>(best, cv::CC_STAT_HEIGHT));
  blob.center = {static_cast<float>(centroids.at<double>(best, 0) + roi.x),
                 static_cast<float>(centroids.at<double>(best, 1) + roi.y)};
  return blob;
}

// Select the road from the vehicle outward. A blob touching a search boundary
// has only one measured edge and cannot provide a reliable road center.
Blob roadBlob(const cv::Mat& mask, const cv::Mat& support, const cv::Rect& band, int min_area,
              const Config& config, float expected_x, float max_step) {
  if (band.empty()) return {};
  cv::Mat selected, labels, stats, centroids;
  cv::compare(mask(band), cv::Scalar(0), selected, cv::CMP_GT);
  const int count = cv::connectedComponentsWithStats(
      selected, labels, stats, centroids, 8, CV_32S);
  Blob best;
  float best_score = std::numeric_limits<float>::max();
  for (int label = 1; label < count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
    const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
    const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
    if (area < min_area || width < config.min_road_width_fraction * mask.cols ||
        width > config.max_road_width_fraction * mask.cols ||
        x <= 1 || x + width >= band.width - 1)
      continue;
    const float center_x = static_cast<float>(centroids.at<double>(label, 0) + band.x);
    const int center_y = cvRound(centroids.at<double>(label, 1) + band.y);
    if (!support.at<unsigned char>(center_y, x + band.x - 1) ||
        !support.at<unsigned char>(center_y, x + band.x + width))
      continue;
    const float distance = std::abs(center_x - expected_x);
    if (distance > max_step) continue;
    const float fill = static_cast<float>(area) / (width * height);
    const float score = distance / mask.cols + 0.1f * (1.0f - fill);
    if (score >= best_score) continue;
    best_score = score;
    best = {true, area,
            {x + band.x, stats.at<int>(label, cv::CC_STAT_TOP) + band.y, width, height},
            {center_x, static_cast<float>(centroids.at<double>(label, 1) + band.y)}};
  }
  return best;
}

struct RoadChroma {
  bool valid = false;
  cv::Vec2f midpoint{};
  cv::Vec2f direction{};

  bool contains(const cv::Vec3b& pixel) const {
    // Opponent channels cancel a shared brightness offset. Learn their
    // direction from the current scene instead of assuming a road hue.
    const cv::Vec2f chroma(static_cast<float>(pixel[0]) - pixel[1],
                          static_cast<float>(pixel[2]) - pixel[1]);
    return (chroma - midpoint).dot(direction) > 0.0f;
  }
};

struct ColorHistogram {
  std::array<int, 511> blue_green{};
  std::array<int, 511> red_green{};
  std::array<int, 256> brightness{};
  int count = 0;

  void add(const cv::Vec3b& pixel, unsigned char value) {
    ++blue_green[static_cast<int>(pixel[0]) - pixel[1] + 255];
    ++red_green[static_cast<int>(pixel[2]) - pixel[1] + 255];
    ++brightness[value];
    ++count;
  }

  template <std::size_t Size>
  int median(const std::array<int, Size>& histogram) const {
    int cumulative = 0;
    for (std::size_t i = 0; i < Size; ++i) {
      cumulative += histogram[i];
      if (cumulative > count / 2) return static_cast<int>(i);
    }
    return 0;
  }

  cv::Vec2f chroma() const {
    return {static_cast<float>(median(blue_green) - 255),
            static_cast<float>(median(red_green) - 255)};
  }
};

RoadChroma estimateRoadChroma(const cv::Mat& bgr, const cv::Mat& gray,
                               const cv::Mat& valid, const cv::Rect& roi,
                               int minimum_contrast) {
  ColorHistogram road, floor;
  const cv::Rect seed = cv::Rect(roi.x + roi.width * 3 / 8, bgr.rows * 4 / 5,
                                 std::max(1, roi.width / 4),
                                 std::max(1, bgr.rows * 17 / 100)) & roi;
  const int minimum_samples = std::max(16, roi.width * bgr.rows / 1000);
  for (int y = seed.y; y < seed.y + seed.height; ++y) {
    const auto* colors = bgr.ptr<cv::Vec3b>(y);
    const auto* values = gray.ptr<unsigned char>(y);
    const auto* support = valid.ptr<unsigned char>(y);
    for (int x = seed.x; x < seed.x + seed.width; ++x)
      if (support[x]) road.add(colors[x], values[x]);
  }
  if (road.count < minimum_samples) return {};

  // The top border and lower outer corners are potential floor samples.
  // Require brighter evidence so a dark side patch cannot define the floor.
  const int floor_minimum = road.median(road.brightness) + minimum_contrast;
  const int top_end = std::max(1, bgr.rows / 20);
  const int corner_start = bgr.rows * 4 / 5;
  for (int y = 0; y < bgr.rows; ++y) {
    if (y >= top_end && y < corner_start) continue;
    const auto* colors = bgr.ptr<cv::Vec3b>(y);
    const auto* values = gray.ptr<unsigned char>(y);
    const auto* support = valid.ptr<unsigned char>(y);
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      if (y >= corner_start && x >= roi.x + roi.width / 10 &&
          x < roi.x + roi.width * 9 / 10)
        continue;
      if (support[x] && values[x] >= floor_minimum) floor.add(colors[x], values[x]);
    }
  }
  if (floor.count < minimum_samples) return {};
  const cv::Vec2f road_color = road.chroma();
  const cv::Vec2f floor_color = floor.chroma();
  const cv::Vec2f direction = road_color - floor_color;
  const float minimum_chroma = static_cast<float>(std::max(8, minimum_contrast / 2));
  if (direction.dot(direction) < minimum_chroma * minimum_chroma) return {};
  return {true, (road_color + floor_color) * 0.5f, direction};
}

cv::Mat adaptiveDarkMask(const cv::Mat& bgr, const cv::Mat& valid,
                         const cv::Rect& roi, int minimum_contrast) {
  // Grayscale avoids the costly Lab conversion on the board. A frame-level
  // reference keeps a full-width crossbar visible when a row has no floor.
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  const RoadChroma chroma = estimateRoadChroma(bgr, gray, valid, roi, minimum_contrast);
  cv::Mat mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
  if (chroma.valid) {
    for (int y = 0; y < bgr.rows; ++y) {
      const auto* colors = bgr.ptr<cv::Vec3b>(y);
      const auto* support = valid.ptr<unsigned char>(y);
      auto* output = mask.ptr<unsigned char>(y);
      for (int x = roi.x; x < roi.x + roi.width; ++x)
        if (support[x] && chroma.contains(colors[x])) output[x] = 255;
    }
    return mask;
  }
  const auto percentile = [](const std::array<int, 256>& histogram, int rank) {
    int cumulative = 0;
    for (int value = 0; value < 256; ++value) {
      cumulative += histogram[value];
      if (cumulative > rank) return value;
    }
    return 255;
  };
  std::vector<std::array<int, 256>> rows(gray.rows);
  std::vector<int> counts(gray.rows, 0);
  std::array<int, 256> frame_histogram{};
  int total = 0;
  for (int y = 0; y < gray.rows; ++y) {
    const auto* row = gray.ptr<unsigned char>(y);
    const auto* support = valid.ptr<unsigned char>(y);
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      if (!support[x]) continue;
      ++rows[y][row[x]];
      ++counts[y];
    }
    for (int value = 0; value < 256; ++value)
      frame_histogram[value] += rows[y][value];
    total += counts[y];
  }
  if (total == 0) return mask;
  const int frame_dark = percentile(frame_histogram, total / 10);
  const int frame_light = percentile(frame_histogram, total * 95 / 100);
  const bool frame_contrast = frame_light - frame_dark >= minimum_contrast;
  const int frame_threshold = (frame_dark + frame_light) / 2;
  for (int y = 0; y < gray.rows; ++y) {
    const auto* row = gray.ptr<unsigned char>(y);
    const auto* support = valid.ptr<unsigned char>(y);
    const int count = counts[y];
    if (count == 0) continue;
    const int dark = percentile(rows[y], count / 10);
    const int light = percentile(rows[y], count * 9 / 10);
    const bool row_contrast = light - dark >= minimum_contrast;
    if (!row_contrast && !frame_contrast) continue;
    const int threshold = row_contrast ? (dark + light) / 2 : frame_threshold;
    auto* output = mask.ptr<unsigned char>(y);
    for (int x = roi.x; x < roi.x + roi.width; ++x)
      if (support[x] && row[x] < threshold) output[x] = 255;
  }
  return mask;
}

cv::Mat connectedRoad(const cv::Mat& mask, const Blob& bottom) {
  cv::Mat labels, selected;
  cv::connectedComponents(mask, labels, 8, CV_32S);
  cv::Point seed;
  float nearest = std::numeric_limits<float>::max();
  for (int y = bottom.bounds.y; y < bottom.bounds.y + bottom.bounds.height; ++y) {
    for (int x = bottom.bounds.x; x < bottom.bounds.x + bottom.bounds.width; ++x) {
      if (!mask.at<unsigned char>(y, x)) continue;
      const float dx = x - bottom.center.x;
      const float dy = y - bottom.center.y;
      const float distance = dx * dx + dy * dy;
      if (distance >= nearest) continue;
      nearest = distance;
      seed = {x, y};
    }
  }
  cv::compare(labels, labels.at<int>(seed), selected, cv::CMP_EQ);
  return selected;
}

unsigned directionsFor(RoadType type) {
  switch (type) {
    case RoadType::STRAIGHT: return STRAIGHT;
    case RoadType::LEFT_TURN: return LEFT;
    case RoadType::RIGHT_TURN: return RIGHT;
    case RoadType::T_JUNCTION: return LEFT | RIGHT;
    case RoadType::LEFT_BRANCH: return LEFT | STRAIGHT;
    case RoadType::RIGHT_BRANCH: return RIGHT | STRAIGHT;
    case RoadType::UNKNOWN: return 0;
  }
  return 0;
}

struct Junction {
  RoadType type = RoadType::UNKNOWN;
  float confidence = 0.0f;
};

// A junction needs an incoming stem, wider connected lateral exits, and a
// blocked forward path. Image-edge contact is allowed for the crossbar only.
Junction classifyJunction(const cv::Mat& mask, const cv::Rect& roi) {
  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
  const cv::Rect anchor = cv::Rect(mask.cols * 3 / 8, mask.rows * 4 / 5,
                                  mask.cols / 4, std::max(1, mask.rows / 6)) & roi;
  if (anchor.empty()) return {};
  std::vector<int> hits(count, 0);
  for (int y = anchor.y; y < anchor.y + anchor.height; ++y)
    for (int x = anchor.x; x < anchor.x + anchor.width; ++x)
      ++hits[labels.at<int>(y, x)];
  int best = 0;
  int best_hits = 0;
  for (int label = 1; label < count; ++label)
    if (hits[label] > best_hits) { best = label; best_hits = hits[label]; }
  if (best == 0 || hits[best] < anchor.area() / 5) return {};

  struct Span { int left = 0; int right = -1; int width() const { return right - left + 1; } };
  std::vector<Span> profile(mask.rows);
  const int center = cvRound(centroids.at<double>(best, 0));
  for (int y = 0; y < mask.rows; ++y) {
    const int* row = labels.ptr<int>(y);
    Span closest;
    int nearest = mask.cols;
    for (int x = roi.x; x < roi.x + roi.width;) {
      if (row[x] != best) { ++x; continue; }
      const int left = x;
      while (x < roi.x + roi.width && row[x] == best) ++x;
      const int distance = center < left ? left - center : std::max(0, center - x + 1);
      if (distance < nearest) {
        nearest = distance;
        closest = {left, x - 1};
      }
    }
    if (nearest < mask.cols / 8) profile[y] = closest;
  }

  std::vector<int> lefts, rights;
  for (int y = mask.rows * 85 / 100; y < mask.rows * 97 / 100; ++y) {
    const Span& span = profile[y];
    if (span.width() < mask.cols * 0.06 || span.left <= roi.x + 1 ||
        span.right >= roi.x + roi.width - 2)
      continue;
    lefts.push_back(span.left);
    rights.push_back(span.right);
  }
  if (lefts.size() < static_cast<std::size_t>(std::max(3, mask.rows / 30))) return {};
  std::sort(lefts.begin(), lefts.end());
  std::sort(rights.begin(), rights.end());
  const int neck_left = lefts[lefts.size() / 2];
  const int neck_right = rights[rights.size() / 2];
  const int neck_width = neck_right - neck_left + 1;
  const int opening = std::max(4, cvRound(mask.cols * 0.035));
  // Near a junction, one incoming edge may be only a few pixels from the
  // image boundary. Do not demand expansion beyond pixels the camera can see.
  const int left_opening = std::min(opening, std::max(2, (neck_left - roi.x + 1) / 2));
  const int right_margin = roi.x + roi.width - 1 - neck_right;
  const int right_opening = std::min(opening, std::max(2, (right_margin + 1) / 2));
  const int minimum_bar_height = std::max(3, mask.rows / 20);
  const int probe = std::max(3, mask.rows / 20);
  const int neck_center = (neck_left + neck_right) / 2;
  int left_run = 0, right_run = 0, both_run = 0;
  int max_left = 0, max_right = 0, max_both = 0;
  int since_branch = 0, closed_rows = 0;
  for (int y = mask.rows * 85 / 100; y >= 0; --y) {
    const Span& span = profile[y];
    const bool left = span.width() > 0 && span.left <= neck_left - left_opening;
    const bool right = span.width() > 0 && span.right >= neck_right + right_opening;
    const bool forward = span.width() > 0 && span.left <= neck_center + neck_width / 6 &&
                         span.right >= neck_center - neck_width / 6;
    left_run = forward && left ? left_run + 1 : 0;
    right_run = forward && right ? right_run + 1 : 0;
    both_run = forward && left && right ? both_run + 1 : 0;
    max_left = std::max(max_left, left_run);
    max_right = std::max(max_right, right_run);
    max_both = std::max(max_both, both_run);
    since_branch = forward && (left || right) ? 0 : since_branch + 1;
    closed_rows = forward ? 0 : closed_rows + 1;
    if (closed_rows >= probe && since_branch <= probe * 2) {
      // Remember both exits across a slanted top edge, rather than allowing
      // the last remaining side of that edge to overwrite the T classification.
      RoadType type = RoadType::UNKNOWN;
      int evidence = 0;
      if (max_both >= minimum_bar_height) {
        type = RoadType::T_JUNCTION;
        evidence = max_both;
      } else if (max_left >= minimum_bar_height && max_right < minimum_bar_height) {
        type = RoadType::LEFT_TURN;
        evidence = max_left;
      } else if (max_right >= minimum_bar_height && max_left < minimum_bar_height) {
        type = RoadType::RIGHT_TURN;
        evidence = max_right;
      }
      if (type != RoadType::UNKNOWN)
        return {type, std::min(0.95f, 0.75f + 0.2f * evidence / (2.0f * minimum_bar_height))};
    }
    if (since_branch > probe * 2) max_left = max_right = max_both = 0;
  }
  return {};
}

cv::Mat labMask(const cv::Mat& bgr, const LabRange& range) {
  cv::Mat lab, mask;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  cv::inRange(lab,
              cv::Scalar(range.l_min, range.a_min, range.b_min),
              cv::Scalar(range.l_max, range.a_max, range.b_max), mask);
  return mask;
}

void rejectDeprecatedKeys(const cv::FileNode& root, const Config& config) {
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
  readValue(root, "input_width", config.input_width);
  readValue(root, "input_height", config.input_height);
  readValue(root, "process_width", config.process_width);
  readValue(root, "segmentation_mode", config.segmentation_mode);
  readValue(root, "road_contrast_min", config.road_contrast_min);
  readValue(root, "min_road_width_fraction", config.min_road_width_fraction);
  readValue(root, "max_road_width_fraction", config.max_road_width_fraction);
  readValue(root, "classification_confirm_frames", config.classification_confirm_frames);
  readValue(root, "classification_hold_frames", config.classification_hold_frames);
  readValue(root, "geometry_smoothing_alpha", config.geometry_smoothing_alpha);
  readValue(root, "morphology_pixels", config.morphology_pixels);
  readValue(root, "min_blob_area_reference", config.min_blob_area_reference);
  readValue(root, "indicator_min_area_reference", config.indicator_min_area_reference);
  readValue(root, "stop_min_area_reference", config.stop_min_area_reference);
  int stop_enabled = config.stop_detection_enabled ? 1 : 0;
  readValue(root, "stop_detection_enabled", stop_enabled);
  config.stop_detection_enabled = stop_enabled != 0;
  config.road_lab = readLabRange(root["road_lab"], config.road_lab, "road_lab");
  config.stop_lab = readLabRange(root["stop_lab"], config.stop_lab, "stop_lab");
  config.indicator_roi = readRect(root["indicator_roi"], "indicator_roi", config.indicator_roi);
  config.left_marker_roi = readRect(root["left_marker_roi"], "left_marker_roi",
                                    config.left_marker_roi);
  config.right_marker_roi = readRect(root["right_marker_roi"], "right_marker_roi",
                                     config.right_marker_roi);
  config.stop_roi = readRect(root["stop_roi"], "stop_roi", config.stop_roi);
  config.roi = readRect(root["roi"], "roi", config.roi);

  const cv::FileNode bands = root["scan_bands"];
  if (!bands.empty()) {
    if (bands.size() != config.scan_bands.size())
      throw std::runtime_error("scan_bands needs six [x,y,width,height] entries");
    for (std::size_t i = 0; i < config.scan_bands.size(); ++i)
      config.scan_bands[i] = readRect(bands[static_cast<int>(i)], "scan_bands entry",
                                      config.scan_bands[i]);
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
  if (!validLabRange(config.road_lab) || !validLabRange(config.stop_lab))
    throw std::runtime_error("invalid Lab range: values must be ordered and within [0,255]");
  if (config.segmentation_mode != "lab" && config.segmentation_mode != "adaptive_dark")
    throw std::runtime_error("segmentation_mode must be lab or adaptive_dark");
  if (config.road_contrast_min < 1 || config.road_contrast_min > 255)
    throw std::runtime_error("road_contrast_min must be within [1,255]");
  if (!std::isfinite(config.min_road_width_fraction) ||
      !std::isfinite(config.max_road_width_fraction) ||
      config.min_road_width_fraction <= 0.0f ||
      config.max_road_width_fraction >= 1.0f ||
      config.min_road_width_fraction >= config.max_road_width_fraction)
    throw std::runtime_error("road width fractions must satisfy 0 < min < max < 1");
  if (config.classification_confirm_frames < 1 || config.classification_confirm_frames > 30 ||
      config.classification_hold_frames < 0 || config.classification_hold_frames > 10)
    throw std::runtime_error("classification frame counts must be confirm [1,30], hold [0,10]");
  if (!std::isfinite(config.geometry_smoothing_alpha) ||
      config.geometry_smoothing_alpha <= 0.0f || config.geometry_smoothing_alpha > 1.0f)
    throw std::runtime_error("geometry_smoothing_alpha must be within (0,1]");
  if (config.morphology_pixels < 1 || config.morphology_pixels > 31 ||
      config.min_blob_area_reference < 1 || config.indicator_min_area_reference < 1 ||
      config.stop_min_area_reference < 1)
    throw std::runtime_error("blob areas and morphology_pixels must be positive");

  const auto validReferenceRect = [](const cv::Rect& roi) {
    return roi.x >= 0 && roi.y >= 0 && roi.width > 0 && roi.height > 0 &&
           roi.x + roi.width <= kReferenceWidth &&
           roi.y + roi.height <= kReferenceHeight;
  };
  for (const cv::Rect& band : config.scan_bands)
    if (!validReferenceRect(band))
      throw std::runtime_error("each scan_bands ROI must fit inside the 128x90 reference image");
  if (!validReferenceRect(config.indicator_roi) ||
      !validReferenceRect(config.left_marker_roi) ||
      !validReferenceRect(config.right_marker_roi) ||
      !validReferenceRect(config.stop_roi))
    throw std::runtime_error("marker and stop ROIs must fit inside the 128x90 reference image");

  if (config.use_perspective) {
    std::vector<cv::Point2f> src(config.perspective_src.begin(), config.perspective_src.end());
    std::vector<cv::Point2f> dst(config.perspective_dst.begin(), config.perspective_dst.end());
    if (std::abs(cv::contourArea(src)) < 100 || std::abs(cv::contourArea(dst)) < 100)
      throw std::runtime_error("perspective quadrilateral has zero or tiny area");
  }
}

const char* toString(RoadType type) {
  switch (type) {
    case RoadType::UNKNOWN: return "UNKNOWN";
    case RoadType::STRAIGHT: return "STRAIGHT";
    case RoadType::LEFT_TURN: return "LEFT_TURN";
    case RoadType::RIGHT_TURN: return "RIGHT_TURN";
    case RoadType::LEFT_BRANCH: return "LEFT_BRANCH";
    case RoadType::RIGHT_BRANCH: return "RIGHT_BRANCH";
    case RoadType::T_JUNCTION: return "T_JUNCTION";
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

  const cv::Size process_size(width, height);
  for (std::size_t i = 0; i < scan_bands_processed_.size(); ++i)
    scan_bands_processed_[i] = scaleReferenceRect(config_.scan_bands[i], process_size);
  indicator_roi_processed_ = scaleReferenceRect(config_.indicator_roi, process_size);
  left_marker_roi_processed_ = scaleReferenceRect(config_.left_marker_roi, process_size);
  right_marker_roi_processed_ = scaleReferenceRect(config_.right_marker_roi, process_size);
  stop_roi_processed_ = scaleReferenceRect(config_.stop_roi, process_size);
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
  const int blur_size = config_.segmentation_mode == "adaptive_dark" ? 5 : 3;
  cv::GaussianBlur(view, blurred, cv::Size(blur_size, blur_size), 0);
  cv::Mat mask = config_.segmentation_mode == "adaptive_dark"
      ? adaptiveDarkMask(blurred, valid_view_mask_, roi_processed_, config_.road_contrast_min)
      : labMask(blurred, config_.road_lab);
  const int kernel_size = config_.morphology_pixels | 1;
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  if (config_.segmentation_mode == "adaptive_dark")
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   kernel);
  cv::bitwise_and(mask, valid_view_mask_, mask);
  return mask;
}

RoadResult RoadDetector::process(const cv::Mat& bgr, std::int64_t timestamp_ms, bool temporal) {
  if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols != config_.input_width ||
      bgr.rows != config_.input_height)
    throw std::invalid_argument("BGR frame size/type does not match configured input size");

  cv::Mat processed, view;
  prepareView(bgr, processed, view);
  RoadResult result = analyzeMask(segment(view), timestamp_ms, &view);
  result.roi_polygon = roi_polygon_;
  result.raw_type = result.type;
  if (temporal) stabilize(result);
  return result;
}

RoadResult RoadDetector::processMask(const cv::Mat& road_mask,
                                     std::int64_t timestamp_ms, bool temporal) {
  if (road_mask.empty() || road_mask.type() != CV_8UC1 ||
      road_mask.cols != config_.process_width ||
      road_mask.rows != config_.processHeight())
    throw std::invalid_argument("mask size/type does not match configured processing size");
  RoadResult result = analyzeMask(road_mask, timestamp_ms, nullptr);
  result.roi_polygon = roi_polygon_;
  result.raw_type = result.type;
  if (temporal) stabilize(result);
  return result;
}

RoadResult RoadDetector::analyzeMask(const cv::Mat& mask, std::int64_t timestamp_ms,
                                     const cv::Mat* processed_bgr) {
  RoadResult result;
  result.timestamp_ms = timestamp_ms;
  result.road_mask = mask.clone();
  const cv::Rect image_bounds(0, 0, mask.cols, mask.rows);
  const cv::Size process_size(mask.cols, mask.rows);
  const int minimum_area = scaledArea(config_.min_blob_area_reference, process_size);
  const Junction junction = classifyJunction(mask, roi_processed_);
  result.type = junction.type;
  result.available_directions = directionsFor(junction.type);
  result.classification_confidence = junction.confidence;
  if (processed_bgr != nullptr && config_.stop_detection_enabled) {
    cv::Mat stop_blurred;
    cv::GaussianBlur(*processed_bgr, stop_blurred, cv::Size(3, 3), 0);
    const cv::Mat stop_mask = labMask(stop_blurred, config_.stop_lab);
    const int stop_area = scaledArea(config_.stop_min_area_reference, process_size);
    result.stop_detected = largestBlob(stop_mask, stop_roi_processed_, true, stop_area).found;
    if (result.stop_detected) {
      result.type = RoadType::UNKNOWN;
      result.available_directions = 0;
      result.classification_confidence = 0.0f;
    }
  }

  std::array<Blob, 6> samples{};
  int valid_samples = 0;
  std::vector<int> widths_reference;
  widths_reference.reserve(samples.size());
  float expected_x = mask.cols / 2.0f;
  int previous_band = static_cast<int>(samples.size());
  cv::Mat tracking_mask = mask;
  for (int i = static_cast<int>(samples.size()) - 1; i >= 0; --i) {
    cv::Rect band = (scan_bands_processed_[i] & roi_processed_) & image_bounds;
    // Permit one missing strip, but never jump across several unobserved bands.
    if (previous_band - i > 2) break;
    const float max_step = valid_samples == 0 ? mask.cols * 0.5f
        : mask.cols * 0.12f * (previous_band - i);
    samples[i] = roadBlob(tracking_mask, valid_view_mask_, band, minimum_area,
                           config_, expected_x, max_step);
    if (i == static_cast<int>(samples.size()) - 1 && !samples[i].found) return result;
    if (!samples[i].found) continue;
    if (valid_samples == 0) tracking_mask = connectedRoad(mask, samples[i]);
    expected_x = samples[i].center.x;
    previous_band = i;
    ++valid_samples;
    const double sx = static_cast<double>(mask.cols) / kReferenceWidth;
    widths_reference.push_back(std::max(1, cvRound(samples[i].bounds.width / sx)));
  }

  // Require support across at least half the strips, including the bottom one.
  if (valid_samples < 3 || !samples.back().found) return result;

  std::vector<cv::Point2f> points;
  points.reserve(samples.size());
  for (const Blob& sample : samples)
    if (sample.found) points.push_back(sample.center);
  if (points.size() < 2) return result;

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const cv::Point2f& point : points) {
    mean_x += point.x;
    mean_y += point.y;
  }
  mean_x /= points.size();
  mean_y /= points.size();
  double numerator = 0.0;
  double denominator = 0.0;
  for (const cv::Point2f& point : points) {
    numerator += (point.y - mean_y) * (point.x - mean_x);
    denominator += (point.y - mean_y) * (point.y - mean_y);
  }
  if (denominator <= 1e-6) return result;
  const double dx_dy = numerator / denominator;
  result.heading_error_rad = static_cast<float>(std::atan(-dx_dy));

  const double sx = static_cast<double>(mask.cols) / kReferenceWidth;
  const float bottom_x_reference =
      static_cast<float>(samples.back().center.x / sx);
  const float offset_reference = bottom_x_reference - kReferenceWidth / 2.0f;
  std::sort(widths_reference.begin(), widths_reference.end());
  const float median_width = static_cast<float>(widths_reference[widths_reference.size() / 2]);
  result.lateral_error = offset_reference / std::max(1.0f, median_width / 2.0f);
  double residual_squared = 0.0;
  double fill_sum = 0.0;
  for (const Blob& sample : samples) {
    if (!sample.found) continue;
    const double residual = sample.center.x - mean_x - dx_dy * (sample.center.y - mean_y);
    residual_squared += residual * residual;
    fill_sum += static_cast<double>(sample.area) / sample.bounds.area();
  }
  const double rms = std::sqrt(residual_squared / valid_samples);
  const double fit_quality = 1.0 / (1.0 + rms / std::max(3.0, median_width * sx * 0.15));
  result.confidence = static_cast<float>(
      valid_samples / static_cast<double>(samples.size()) * fit_quality * fill_sum / valid_samples);
  result.valid = true;

  // Expose the fitted centerline at the measured rows and the observed strip
  // edges, all converted back to source-image coordinates for preview/JSON.
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!samples[i].found) continue;
    const float y = samples[i].center.y;
    const float fitted_x = static_cast<float>(mean_x + dx_dy * (y - mean_y));
    result.centerline.push_back(sourcePoint({fitted_x, y}));
    result.left_boundary.push_back(
        sourcePoint({static_cast<float>(samples[i].bounds.x), y}));
    result.right_boundary.push_back(
        sourcePoint({static_cast<float>(samples[i].bounds.x + samples[i].bounds.width - 1), y}));
  }

  bool left = false;
  bool right = false;
  // Legacy color-marker layouts remain available in calibrated Lab mode.
  if (config_.segmentation_mode == "lab") {
    const int indicator_area = scaledArea(config_.indicator_min_area_reference, process_size);
    const Blob indicator = largestBlob(tracking_mask, indicator_roi_processed_, false, indicator_area);
    if (indicator.found) {
      left = largestBlob(tracking_mask, left_marker_roi_processed_, true, minimum_area).found;
      right = largestBlob(tracking_mask, right_marker_roi_processed_, true, minimum_area).found;
    }
  }

  if (junction.type == RoadType::UNKNOWN && !result.stop_detected) {
    result.type = left && right ? RoadType::T_JUNCTION
        : (left ? RoadType::LEFT_TURN : (right ? RoadType::RIGHT_TURN : RoadType::STRAIGHT));
    result.available_directions = directionsFor(result.type);
    result.classification_confidence = result.confidence;
  }
  return result;
}

void RoadDetector::stabilize(RoadResult& result) {
  // A discontinuous stream must not inherit a junction or geometry from an old scene.
  if (!has_previous_result_ || result.timestamp_ms <= previous_timestamp_ms_ ||
      result.timestamp_ms - previous_timestamp_ms_ > 500) {
    stable_type_ = pending_type_ = RoadType::UNKNOWN;
    pending_frames_ = unsupported_frames_ = 0;
    stable_classification_confidence_ = 0.0f;
    previous_geometry_ = RoadResult{};
  }
  has_previous_result_ = true;
  previous_timestamp_ms_ = result.timestamp_ms;

  const RoadType observed = result.type;
  // An enabled stop gate clears a known type immediately; do not debounce it.
  const bool stop = result.stop_detected;
  if (stop) {
    stable_type_ = pending_type_ = RoadType::UNKNOWN;
    pending_frames_ = unsupported_frames_ = 0;
    stable_classification_confidence_ = 0.0f;
  } else if (observed == stable_type_) {
    pending_type_ = RoadType::UNKNOWN;
    pending_frames_ = unsupported_frames_ = 0;
    stable_classification_confidence_ = result.classification_confidence;
  } else {
    ++unsupported_frames_;
    if (observed != RoadType::UNKNOWN) {
      if (observed == pending_type_) ++pending_frames_;
      else { pending_type_ = observed; pending_frames_ = 1; }
      if (pending_frames_ >= config_.classification_confirm_frames) {
        stable_type_ = observed;
        stable_classification_confidence_ = result.classification_confidence;
        pending_type_ = RoadType::UNKNOWN;
        pending_frames_ = unsupported_frames_ = 0;
      }
    } else {
      pending_type_ = RoadType::UNKNOWN;
      pending_frames_ = 0;
    }
    if (unsupported_frames_ > config_.classification_hold_frames) {
      stable_type_ = RoadType::UNKNOWN;
      stable_classification_confidence_ = 0.0f;
    } else {
      stable_classification_confidence_ *= 0.8f;
    }
  }
  result.type = stable_type_;
  result.available_directions = directionsFor(stable_type_);
  result.classification_confidence = stable_classification_confidence_;

  const float alpha = config_.geometry_smoothing_alpha;
  bool compatible = result.valid && previous_geometry_.valid &&
      result.centerline.size() == previous_geometry_.centerline.size() &&
      result.left_boundary.size() == previous_geometry_.left_boundary.size() &&
      result.right_boundary.size() == previous_geometry_.right_boundary.size();
  if (compatible) {
    for (std::size_t i = 0; i < result.centerline.size(); ++i)
      if (std::abs(result.centerline[i].y - previous_geometry_.centerline[i].y) >
              config_.input_height * 0.04f ||
          std::abs(result.centerline[i].x - previous_geometry_.centerline[i].x) >
              config_.input_width * 0.15f)
        compatible = false;
  }
  if (compatible && alpha < 1.0f) {
    result.lateral_error = alpha * result.lateral_error +
                           (1.0f - alpha) * previous_geometry_.lateral_error;
    result.heading_error_rad = alpha * result.heading_error_rad +
                               (1.0f - alpha) * previous_geometry_.heading_error_rad;
    for (std::size_t i = 0; i < result.centerline.size(); ++i) {
      result.centerline[i].x = alpha * result.centerline[i].x +
                               (1.0f - alpha) * previous_geometry_.centerline[i].x;
      result.left_boundary[i].x = alpha * result.left_boundary[i].x +
                                  (1.0f - alpha) * previous_geometry_.left_boundary[i].x;
      result.right_boundary[i].x = alpha * result.right_boundary[i].x +
                                   (1.0f - alpha) * previous_geometry_.right_boundary[i].x;
    }
  }
  // No previous image/mask buffers, and no invented steering on an invalid frame.
  previous_geometry_ = RoadResult{};
  previous_geometry_.valid = result.valid;
  if (result.valid) {
    previous_geometry_.lateral_error = result.lateral_error;
    previous_geometry_.heading_error_rad = result.heading_error_rad;
    previous_geometry_.centerline = result.centerline;
    previous_geometry_.left_boundary = result.left_boundary;
    previous_geometry_.right_boundary = result.right_boundary;
  }
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
