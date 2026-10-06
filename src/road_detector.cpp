#include "road_finder/road_detector.hpp"

#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/2d.hpp>
#include <opencv2/geometry/3d.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>

namespace road_finder {
namespace {

template <typename T>
void readValue(const cv::FileNode& node, const char* key, T& value) {
  if (!node[key].empty()) node[key] >> value;
}

std::array<cv::Point2f, 4> readPoints(const cv::FileNode& node) {
  if (node.size() != 4) throw std::runtime_error("perspective points need four [x,y] pairs");
  std::array<cv::Point2f, 4> points{};
  for (int i = 0; i < 4; ++i) {
    if (node[i].size() != 2) throw std::runtime_error("invalid perspective point");
    points[i].x = static_cast<float>((double)node[i][0]);
    points[i].y = static_cast<float>((double)node[i][1]);
  }
  return points;
}

// Legacy roi_fraction values are fractions of the processed image; the aspect
// ratio matches the input image, so the fractions carry over to input pixels.
cv::Rect roiFromFraction(const cv::FileNode& node, int input_width, int input_height) {
  if (node.size() != 4) throw std::runtime_error("roi_fraction needs [x,y,w,h]");
  const auto pixels = [](double fraction, int size) {
    return static_cast<int>(std::lround(fraction * size));
  };
  return {pixels(static_cast<double>(node[0]), input_width),
          pixels(static_cast<double>(node[1]), input_height),
          pixels(static_cast<double>(node[2]), input_width),
          pixels(static_cast<double>(node[3]), input_height)};
}

// Deprecated keys are rejected with the replacement value in the message, so
// migrating a tuned configuration is a copy-paste.
void rejectDeprecatedKeys(const cv::FileNode& root, const Config& c) {
  if (!root["process_height"].empty())
    throw std::runtime_error(
        "process_height is no longer used: the processing height follows the input aspect "
        "ratio, remove the key (process_width " + std::to_string(c.process_width) + " -> " +
        std::to_string(c.processHeight()) + ")");
  if (!root["roi_fraction"].empty()) {
    const cv::Rect roi = roiFromFraction(root["roi_fraction"], c.input_width, c.input_height);
    throw std::runtime_error(
        "roi_fraction was replaced by roi in input-image pixels; use roi: [" +
        std::to_string(roi.x) + ", " + std::to_string(roi.y) + ", " +
        std::to_string(roi.width) + ", " + std::to_string(roi.height) + "]");
  }
}

// Channels are caller-owned scratch: the same three buffers are reused for
// every frame, so only the first frame pays for the allocations.
cv::Scalar medianHsv(const cv::Mat& hsv, cv::Rect rect, std::array<std::vector<int>, 3>& channels) {
  rect &= cv::Rect(0, 0, hsv.cols, hsv.rows);
  for (auto& values : channels) values.clear();
  if (rect.empty()) return cv::Scalar(0, 0, 0);
  const std::size_t wanted = static_cast<std::size_t>(rect.width) * rect.height;
  for (auto& values : channels) values.reserve(wanted);
  for (int y = rect.y; y < rect.y + rect.height; ++y) {
    const cv::Vec3b* row = hsv.ptr<cv::Vec3b>(y);
    for (int x = rect.x; x < rect.x + rect.width; ++x) {
      channels[0].push_back(row[x][0]);
      channels[1].push_back(row[x][1]);
      channels[2].push_back(row[x][2]);
    }
  }
  cv::Scalar result;
  for (int c = 0; c < 3; ++c) {
    auto& values = channels[c];
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    result[c] = values[values.size() / 2];
  }
  return result;
}

float hsvDistanceSquared(const cv::Vec3b& pixel, const cv::Scalar& model) {
  const float s = static_cast<float>(pixel[1]);
  const float model_s = static_cast<float>(model[1]);
  float hue = std::abs(static_cast<float>(pixel[0] - model[0]));
  hue = std::min(hue, 180.0f - hue);
  if (s < 15.0f || model_s < 15.0f) hue = 0.0f;
  const float ds = s - model_s;
  const float dv = static_cast<float>(pixel[2] - model[2]);
  return 0.6f * hue * hue + 0.7f * ds * ds + dv * dv;
}

float hsvDistance(const cv::Vec3b& pixel, const cv::Scalar& model) {
  return std::sqrt(hsvDistanceSquared(pixel, model));
}



int neighborCount(const cv::Mat& binary, int y, int x) {
  int count = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if ((dx || dy) && binary.at<uchar>(y + dy, x + dx)) ++count;
    }
  }
  return count;
}

// Rows and columns that hold no foreground pixel cannot produce a change, and
// thinning eats the shape from the outside, so the work area shrinks as it goes.
void trimThinBounds(const cv::Mat& binary, int& y_top, int& y_bottom, int& x_left, int& x_right) {
  while (y_top <= y_bottom && cv::countNonZero(binary.row(y_top)) == 0) ++y_top;
  while (y_bottom >= y_top && cv::countNonZero(binary.row(y_bottom)) == 0) --y_bottom;
  while (x_left <= x_right && cv::countNonZero(binary.col(x_left)) == 0) ++x_left;
  while (x_right >= x_left && cv::countNonZero(binary.col(x_right)) == 0) --x_right;
}

cv::Mat thin(const cv::Mat& mask) {
  cv::Mat binary;
  cv::threshold(mask, binary, 0, 1, cv::THRESH_BINARY);
  binary.row(0).setTo(0);
  binary.row(binary.rows - 1).setTo(0);
  binary.col(0).setTo(0);
  binary.col(binary.cols - 1).setTo(0);
  cv::Mat marker = cv::Mat::zeros(binary.size(), CV_8UC1);
  int y_top = 1, y_bottom = binary.rows - 2, x_left = 1, x_right = binary.cols - 2;
  for (int iteration = 0; iteration < 120; ++iteration) {
    trimThinBounds(binary, y_top, y_bottom, x_left, x_right);
    if (y_top > y_bottom || x_left > x_right) break;
    const cv::Rect work(x_left, y_top, x_right - x_left + 1, y_bottom - y_top + 1);
    cv::Mat work_binary = binary(work);
    int changed = 0;
    for (int pass = 0; pass < 2; ++pass) {
      cv::Mat work_marker = marker(work);
      work_marker.setTo(0);
      // Within one pass every pixel only reads the frozen binary image, so the
      // rows can be handed to the thread pool without changing the result.
      cv::parallel_for_(cv::Range(y_top, y_bottom + 1), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; ++y) {
          const uchar* above = binary.ptr<uchar>(y - 1);
          const uchar* current = binary.ptr<uchar>(y);
          const uchar* below = binary.ptr<uchar>(y + 1);
          uchar* out = marker.ptr<uchar>(y);
          for (int x = x_left; x <= x_right; ++x) {
            if (!current[x]) continue;
            const int p2 = above[x], p3 = above[x + 1], p4 = current[x + 1], p5 = below[x + 1];
            const int p6 = below[x], p7 = below[x - 1], p8 = current[x - 1], p9 = above[x - 1];
            const int n = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            if (n < 2 || n > 6) continue;
            const int transitions = (!p2 && p3) + (!p3 && p4) + (!p4 && p5) +
                                    (!p5 && p6) + (!p6 && p7) + (!p7 && p8) +
                                    (!p8 && p9) + (!p9 && p2);
            if (transitions != 1) continue;
            const bool protect_a = pass == 0 ? p2 * p4 * p6 : p2 * p4 * p8;
            const bool protect_b = pass == 0 ? p4 * p6 * p8 : p2 * p6 * p8;
            if (!protect_a && !protect_b) out[x] = 1;
          }
        }
      });
      changed += cv::countNonZero(work_marker);
      work_binary.setTo(0, work_marker);
    }
    if (!changed) break;
  }
  return binary;
}

struct Topology {
  RoadType type = RoadType::STRAIGHT;
  unsigned directions = STRAIGHT;
};

Topology topologyFromMask(const cv::Mat& mask, float road_width,
                          const Config& config) {
  const cv::Size small_size(std::max(40, mask.cols / 4), std::max(30, mask.rows / 4));
  cv::Mat small, skeleton;
  cv::resize(mask, small, small_size, 0, 0, cv::INTER_NEAREST);
  skeleton = thin(small);
  const float scale = static_cast<float>(small.cols) / mask.cols;
  const float min_length = std::max(5.0f, road_width * scale * config.junction_min_arm_widths);
  std::vector<cv::Point> endpoints;
  cv::Mat branches = cv::Mat::zeros(skeleton.size(), CV_8UC1);
  for (int y = 1; y < skeleton.rows - 1; ++y) {
    for (int x = 1; x < skeleton.cols - 1; ++x) {
      if (!skeleton.at<uchar>(y, x)) continue;
      const int degree = neighborCount(skeleton, y, x);
      if (degree == 1) endpoints.emplace_back(x, y);
      if (degree >= 3) branches.at<uchar>(y, x) = 255;
    }
  }

  // Adjacent branch pixels form one junction. Removing the complete cluster
  // separates its arms so that small skeleton artifacts cannot count as roads.
  cv::Mat branch_labels, branch_stats, branch_centroids;
  const int branch_count = cv::connectedComponentsWithStats(
      branches, branch_labels, branch_stats, branch_centroids, 8);
  int junction = -1;
  for (int label = 1; label < branch_count; ++label) {
    const int y = branch_stats.at<int>(label, cv::CC_STAT_TOP);
    if (y < small.rows / 8 || y > small.rows * 4 / 5) continue;
    if (junction < 0 || y > branch_stats.at<int>(junction, cv::CC_STAT_TOP))
      junction = label;
  }
  if (junction > 0) {
    cv::Mat cluster = branch_labels == junction;
    cv::dilate(cluster, cluster, cv::Mat::ones(3, 3, CV_8UC1));
    cv::Mat arms = skeleton.clone();
    arms.setTo(0, cluster);
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(arms, labels, stats, centroids, 8);
    const cv::Point2f center(static_cast<float>(branch_centroids.at<double>(junction, 0)),
                             static_cast<float>(branch_centroids.at<double>(junction, 1)));
    bool left = false, straight = false, right = false;
    bool incoming = false;
    for (int label = 1; label < count; ++label) {
      float far_distance = 0.0f;
      cv::Point far;
      bool touches_cluster = false;
      for (int y = 1; y < arms.rows - 1; ++y) {
        for (int x = 1; x < arms.cols - 1; ++x) {
          if (labels.at<int>(y, x) != label) continue;
          const float distance = cv::norm(cv::Point2f(x, y) - center);
          if (distance > far_distance) {
            far_distance = distance;
            far = {x, y};
          }
          if (cluster.at<uchar>(y, x)) touches_cluster = true;
          for (int dy = -1; dy <= 1 && !touches_cluster; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
              if (cluster.at<uchar>(y + dy, x + dx)) touches_cluster = true;
        }
      }
      if (!touches_cluster || far_distance < min_length) continue;
      const float dx = far.x - center.x;
      const float dy = far.y - center.y;
      if (dy > min_length * 0.65f && std::abs(dx) < dy * 1.3f) incoming = true;
      else if (dx < -min_length * 0.55f && std::abs(dx) > std::abs(dy) * 0.65f) left = true;
      else if (dx > min_length * 0.55f && std::abs(dx) > std::abs(dy) * 0.65f) right = true;
      else if (dy < -min_length * 0.55f) straight = true;
    }
    // An upper arm can be disconnected from the branch cluster by thinning
    // around a broad side junction. Confirm it in the original occupancy mask.
    const int probe = std::max(static_cast<int>(std::ceil(min_length * 1.2f)),
                               static_cast<int>(std::ceil(road_width * scale * 0.65f)));
    const int px = static_cast<int>(std::round(center.x));
    const int py = static_cast<int>(std::round(center.y)) - probe;
    if (!straight && py >= 2 && py < small.rows - 2 && px >= 2 && px < small.cols - 2) {
      const cv::Rect sample(px - 2, py - 2, 5, 5);
      if (cv::countNonZero(small(sample)) >= 15) straight = true;
    }
    if (incoming) {
      if (left && right && !straight) return {RoadType::T_JUNCTION, LEFT | RIGHT};
      if (left && straight && !right) return {RoadType::LEFT_BRANCH, LEFT | STRAIGHT};
      if (right && straight && !left) return {RoadType::RIGHT_BRANCH, RIGHT | STRAIGHT};
    }
  }

  // A bend has no branch node. Use the outgoing arm's local tangent rather
  // than its total displacement, which is perspective-dependent.
  if (endpoints.size() == 2) {
    const cv::Point incoming = endpoints[0].y > endpoints[1].y ? endpoints[0] : endpoints[1];
    const cv::Point outgoing = endpoints[0].y > endpoints[1].y ? endpoints[1] : endpoints[0];
    if (incoming.y > small.rows * 2 / 3) {
      cv::Point current = outgoing, previous(-1, -1);
      const int steps = static_cast<int>(std::max(5.0f, min_length));
      for (int i = 0; i < steps; ++i) {
        cv::Point next(-1, -1);
        for (int dy = -1; dy <= 1 && next.x < 0; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const cv::Point p = current + cv::Point(dx, dy);
            if ((!dx && !dy) || p == previous || p.x <= 0 || p.y <= 0 ||
                p.x >= skeleton.cols - 1 || p.y >= skeleton.rows - 1) continue;
            if (skeleton.at<uchar>(p)) { next = p; break; }
          }
        }
        if (next.x < 0) break;
        previous = current;
        current = next;
      }
      const float dx = outgoing.x - current.x;
      const float dy = outgoing.y - current.y;
      if (std::abs(dx) > std::max(4.0f, std::abs(dy) * 1.4f)) {
        if (dx < 0) return {RoadType::LEFT_TURN, LEFT};
        return {RoadType::RIGHT_TURN, RIGHT};
      }
    }
  }
  return {};
}

bool fitQuadratic(const std::vector<cv::Point2f>& points, cv::Vec3d& coeff) {
  if (points.size() < 4) return false;
  cv::Mat a(static_cast<int>(points.size()), 3, CV_64F);
  cv::Mat b(static_cast<int>(points.size()), 1, CV_64F);
  for (int i = 0; i < a.rows; ++i) {
    const double y = points[i].y;
    a.at<double>(i, 0) = 1;
    a.at<double>(i, 1) = y;
    a.at<double>(i, 2) = y * y;
    b.at<double>(i) = points[i].x;
  }
  cv::Mat solution;
  if (!cv::solve(a, b, solution, cv::DECOMP_SVD)) return false;
  coeff = {solution.at<double>(0), solution.at<double>(1), solution.at<double>(2)};
  return true;
}

float eval(const cv::Vec3d& c, float y) {
  return static_cast<float>(c[0] + c[1] * y + c[2] * y * y);
}

struct LinearModel {
  double intercept = 0;
  double slope = 0;
  int first_y = 0;
  int last_y = 0;
  float inlier_ratio = 0;
  float rmse = 0;
  float at(float y) const { return static_cast<float>(intercept + slope * y); }
};

bool leastSquaresLine(const std::vector<cv::Point2f>& points, LinearModel& line) {
  if (points.size() < 2) return false;
  double sum_y = 0, sum_x = 0, sum_yy = 0, sum_yx = 0;
  for (const auto& p : points) {
    sum_y += p.y;
    sum_x += p.x;
    sum_yy += static_cast<double>(p.y) * p.y;
    sum_yx += static_cast<double>(p.y) * p.x;
  }
  const double n = static_cast<double>(points.size());
  const double denominator = n * sum_yy - sum_y * sum_y;
  if (std::abs(denominator) < 1e-6) return false;
  line.slope = (n * sum_yx - sum_y * sum_x) / denominator;
  line.intercept = (sum_x - line.slope * sum_y) / n;
  return true;
}

bool robustLine(const std::vector<cv::Point2f>& points, float residual_limit,
                float minimum_ratio, float minimum_span, LinearModel& line) {
  if (!leastSquaresLine(points, line)) return false;
  std::vector<cv::Point2f> inliers;
  for (int iteration = 0; iteration < 3; ++iteration) {
    inliers.clear();
    for (const auto& p : points)
      if (std::abs(p.x - line.at(p.y)) <= residual_limit) inliers.push_back(p);
    if (inliers.size() < 2 || !leastSquaresLine(inliers, line)) return false;
  }
  line.first_y = static_cast<int>(inliers.front().y);
  line.last_y = static_cast<int>(inliers.back().y);
  line.inlier_ratio = static_cast<float>(inliers.size()) / points.size();
  double squared = 0;
  for (const auto& p : inliers) {
    const double residual = p.x - line.at(p.y);
    squared += residual * residual;
  }
  line.rmse = static_cast<float>(std::sqrt(squared / inliers.size()));
  return line.inlier_ratio >= minimum_ratio &&
         line.last_y - line.first_y >= minimum_span &&
         line.rmse <= residual_limit * 0.75f;
}

struct EdgeCandidate {
  float x = 0;
  float score = 0;
};

struct CandidateRow {
  int y = 0;
  std::vector<EdgeCandidate> edges;
};

struct LineSupport {
  std::vector<cv::Point2f> points;
  int eligible_rows = 0;
  float score = 0;
  float squared_error = 0;
};

LineSupport supportForLine(const LinearModel& line,
                           const std::vector<CandidateRow>& rows,
                           const LinearModel& left, float minimum_width,
                           float residual_limit, int image_width) {
  LineSupport support;
  for (const auto& row : rows) {
    const float expected = line.at(static_cast<float>(row.y));
    if (expected <= left.at(static_cast<float>(row.y)) + minimum_width ||
        expected >= image_width - 13) continue;
    ++support.eligible_rows;
    const EdgeCandidate* best = nullptr;
    float best_value = -std::numeric_limits<float>::infinity();
    for (const auto& candidate : row.edges) {
      const float distance = std::abs(candidate.x - expected);
      if (distance > residual_limit) continue;
      const float value = candidate.score - distance * 0.3f;
      if (value > best_value) { best = &candidate; best_value = value; }
    }
    if (best) {
      support.points.emplace_back(best->x, static_cast<float>(row.y));
      support.score += best->score;
      const float residual = best->x - expected;
      support.squared_error += residual * residual;
    }
  }
  return support;
}

bool supportIsEnough(const LineSupport& support, float minimum_ratio,
                     float minimum_span, float residual_limit,
                     float lookahead, LinearModel& line) {
  if (support.points.size() < 5 || support.eligible_rows < 5) return false;
  line.first_y = static_cast<int>(support.points.front().y);
  line.last_y = static_cast<int>(support.points.back().y);
  line.inlier_ratio = static_cast<float>(support.points.size()) / support.eligible_rows;
  line.rmse = std::sqrt(support.squared_error / support.points.size());
  const bool target_supported = std::any_of(support.points.begin(), support.points.end(),
      [lookahead](const cv::Point2f& p) { return std::abs(p.y - lookahead) <= 18; });
  return line.inlier_ratio >= minimum_ratio &&
         line.last_y - line.first_y >= minimum_span && target_supported &&
         line.rmse <= residual_limit * 0.8f;
}

}  // namespace

Config loadConfig(const std::string& path) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) throw std::runtime_error("cannot open config: " + path);
  Config c;
  readValue(fs.root(), "input_width", c.input_width);
  readValue(fs.root(), "input_height", c.input_height);
  readValue(fs.root(), "process_width", c.process_width);
  readValue(fs.root(), "equalize_value", c.equalize_value);
  readValue(fs.root(), "color_distance_max", c.color_distance_max);
  readValue(fs.root(), "color_background_ratio", c.color_background_ratio);
  readValue(fs.root(), "chroma_row_ratio", c.chroma_row_ratio);
  readValue(fs.root(), "chroma_min_contrast", c.chroma_min_contrast);
  readValue(fs.root(), "edge_min_score", c.edge_min_score);
  readValue(fs.root(), "edge_line_residual_px", c.edge_line_residual_px);
  readValue(fs.root(), "edge_min_inlier_ratio", c.edge_min_inlier_ratio);
  readValue(fs.root(), "edge_min_span_fraction", c.edge_min_span_fraction);
  readValue(fs.root(), "morphology_pixels", c.morphology_pixels);
  readValue(fs.root(), "max_white_gap_width_ratio", c.max_white_gap_width_ratio);
  readValue(fs.root(), "min_road_width_fraction", c.min_road_width_fraction);
  readValue(fs.root(), "max_road_width_fraction", c.max_road_width_fraction);
  readValue(fs.root(), "junction_min_arm_widths", c.junction_min_arm_widths);
  readValue(fs.root(), "junction_vote_frames", c.junction_vote_frames);
  readValue(fs.root(), "junction_vote_window", c.junction_vote_window);
  readValue(fs.root(), "lookahead_fraction", c.lookahead_fraction);
  readValue(fs.root(), "minimum_confidence", c.minimum_confidence);
  const auto roi = fs["roi"];
  if (!roi.empty()) {
    if (roi.size() != 4)
      throw std::runtime_error("roi needs [x, y, width, height] in input-image pixels");
    c.roi = cv::Rect(static_cast<int>(roi[0]), static_cast<int>(roi[1]),
                     static_cast<int>(roi[2]), static_cast<int>(roi[3]));
  }
  const auto hsv = fs["road_hsv"];
  if (!hsv.empty()) {
    if (hsv.size() != 6) throw std::runtime_error("road_hsv needs [h_min,h_max,s_min,s_max,v_min,v_max]");
    c.road_hsv = {(int)hsv[0], (int)hsv[1], (int)hsv[2],
                  (int)hsv[3], (int)hsv[4], (int)hsv[5]};
  }
  const auto src = fs["perspective_src"];
  const auto dst = fs["perspective_dst"];
  if (!src.empty() || !dst.empty()) {
    if (src.empty() || dst.empty()) throw std::runtime_error("both perspective point sets are required");
    c.perspective_src = readPoints(src);
    c.perspective_dst = readPoints(dst);
    c.use_perspective = true;
  }
  validateConfig(c);
  rejectDeprecatedKeys(fs.root(), c);
  return c;
}

void validateConfig(const Config& c) {
  if (c.input_width <= 0 || c.input_height <= 0)
    throw std::runtime_error("input_width and input_height must be positive");
  if (c.process_width < 80)
    throw std::runtime_error("process_width must be at least 80, got " +
                             std::to_string(c.process_width));
  if (c.processHeight() < 60)
    throw std::runtime_error("process_width " + std::to_string(c.process_width) + " over " +
                             std::to_string(c.input_width) + "x" + std::to_string(c.input_height) +
                             " gives a processing height of " + std::to_string(c.processHeight()) +
                             ", which is too small");
  if (c.roi.width <= 0 || c.roi.height <= 0 || c.roi.x < 0 || c.roi.y < 0 ||
      c.roi.x + c.roi.width > c.input_width || c.roi.y + c.roi.height > c.input_height)
    throw std::runtime_error(
        "roi [" + std::to_string(c.roi.x) + ", " + std::to_string(c.roi.y) + ", " +
        std::to_string(c.roi.width) + ", " + std::to_string(c.roi.height) +
        "] must stay inside the input image " + std::to_string(c.input_width) + "x" +
        std::to_string(c.input_height));
  if (c.road_hsv.h_min < 0 || c.road_hsv.h_min > 179 ||
      c.road_hsv.h_max < 0 || c.road_hsv.h_max > 179 || c.road_hsv.s_min < 0 ||
      c.road_hsv.s_max > 255 || c.road_hsv.v_min < 0 || c.road_hsv.v_max > 255 ||
      c.road_hsv.s_min > c.road_hsv.s_max || c.road_hsv.v_min > c.road_hsv.v_max)
    throw std::runtime_error("invalid HSV range");
  if (c.junction_vote_frames < 1 || c.junction_vote_window < c.junction_vote_frames ||
      c.morphology_pixels < 1 || c.morphology_pixels > 31 ||
      c.color_distance_max <= 0 || c.color_background_ratio <= 0 ||
      c.chroma_row_ratio < 0 || c.chroma_row_ratio > 1 ||
      c.chroma_min_contrast < 0 || c.edge_min_score <= 0 ||
      c.edge_line_residual_px <= 0 || c.edge_min_inlier_ratio <= 0 ||
      c.edge_min_inlier_ratio > 1 || c.edge_min_span_fraction <= 0 ||
      c.edge_min_span_fraction > 1 ||
      c.max_white_gap_width_ratio <= 0 || c.junction_min_arm_widths <= 0 ||
      c.min_road_width_fraction <= 0 || c.max_road_width_fraction > 1 ||
      c.min_road_width_fraction >= c.max_road_width_fraction ||
      c.lookahead_fraction <= 0 || c.lookahead_fraction >= 1 ||
      c.minimum_confidence < 0 || c.minimum_confidence > 1)
    throw std::runtime_error("invalid detection parameter");
  // The road profile is grown from the bottom of the window, so the window has
  // to reach down to the bottom of the image.
  if (c.roi.y + c.roi.height < static_cast<int>(0.8f * c.input_height))
    throw std::runtime_error(
        "roi must reach at least 80% of the input height (" +
        std::to_string(static_cast<int>(0.8f * c.input_height)) + " px); got " +
        std::to_string(c.roi.y + c.roi.height));
  if (c.use_perspective) {
    std::vector<cv::Point2f> src(c.perspective_src.begin(), c.perspective_src.end());
    std::vector<cv::Point2f> dst(c.perspective_dst.begin(), c.perspective_dst.end());
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
  const int process_width = config_.process_width;
  const int process_height = config_.processHeight();
  const float sx = static_cast<float>(process_width) / config_.input_width;
  const float sy = static_cast<float>(process_height) / config_.input_height;
  vehicle_x_ = process_width / 2.0f;
  if (config_.use_perspective) {
    std::vector<cv::Point2f> src(config_.perspective_src.begin(), config_.perspective_src.end());
    std::vector<cv::Point2f> dst(config_.perspective_dst.begin(), config_.perspective_dst.end());
    for (auto& p : src) { p.x *= sx; p.y *= sy; }
    for (auto& p : dst) { p.x *= sx; p.y *= sy; }
    warp_ = cv::getPerspectiveTransform(src, dst);
    inverse_warp_ = warp_.inv();
    std::vector<cv::Point2f> vehicle{{process_width / 2.0f, process_height * 0.95f}}, projected;
    cv::perspectiveTransform(vehicle, projected, warp_);
    vehicle_x_ = projected.front().x;
  }

  // The configured window lives in input-image pixels; everything below works on
  // the processed image, and with perspective a rectangle becomes a quad, so the
  // bounding box is what the detector can use as a search window.
  const cv::Rect2f& roi = config_.roi;
  std::vector<cv::Point2f> corners{
      {roi.x * sx, roi.y * sy},
      {(roi.x + roi.width) * sx, roi.y * sy},
      {(roi.x + roi.width) * sx, (roi.y + roi.height) * sy},
      {roi.x * sx, (roi.y + roi.height) * sy}};
  cv::Rect2f processed(roi.x * sx, roi.y * sy, roi.width * sx, roi.height * sy);
  if (config_.use_perspective) {
    cv::perspectiveTransform(corners, corners, warp_);
    float min_x = corners.front().x, max_x = corners.front().x;
    float min_y = corners.front().y, max_y = corners.front().y;
    for (const cv::Point2f& corner : corners) {
      min_x = std::min(min_x, corner.x);
      max_x = std::max(max_x, corner.x);
      min_y = std::min(min_y, corner.y);
      max_y = std::max(max_y, corner.y);
    }
    processed = cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
  }
  const cv::Rect2f image(0, 0, process_width, process_height);
  if ((processed & image).area() <= 0)
    throw std::runtime_error(
        "roi lies outside the processed image; check roi and the perspective points");
  processed &= image;
  roi_processed_ = cv::Rect(cvRound(processed.x), cvRound(processed.y),
                            cvRound(processed.width), cvRound(processed.height));

  // Report the region the detector really uses: the corners of that window,
  // mapped back to input-image pixels for the preview and the --overlay export.
  const float left = static_cast<float>(roi_processed_.x);
  const float top = static_cast<float>(roi_processed_.y);
  const float right = static_cast<float>(roi_processed_.x + roi_processed_.width);
  const float bottom = static_cast<float>(roi_processed_.y + roi_processed_.height);
  std::vector<cv::Point2f> used{{left, top}, {right, top}, {right, bottom}, {left, bottom}};
  if (config_.use_perspective) cv::perspectiveTransform(used, used, inverse_warp_);
  roi_polygon_.reserve(used.size());
  for (const cv::Point2f& point : used)
    roi_polygon_.emplace_back(point.x / sx, point.y / sy);

  const HsvRange& hsv = config_.road_hsv;
  for (int h = 0; h < 180; ++h)
    hsv_lut_.hue[h] = hsv.h_min <= hsv.h_max ? (h >= hsv.h_min && h <= hsv.h_max)
                                             : (h >= hsv.h_min || h <= hsv.h_max);
  for (int s = 0; s < 256; ++s)
    hsv_lut_.saturation[s] = s >= hsv.s_min && s <= hsv.s_max;
  for (int v = 0; v < 256; ++v)
    hsv_lut_.value[v] = v >= hsv.v_min && v <= hsv.v_max;
}

cv::Mat RoadDetector::keepBottomConnected(const cv::Mat& mask) const {
  const int count = cv::connectedComponentsWithStats(mask, labels_scratch_, stats_scratch_,
                                                     centroids_scratch_, 8);
  int best = -1;
  seed_counts_scratch_.assign(static_cast<std::size_t>(count), 0);
  const int y_start = mask.rows * 4 / 5;
  const int x0 = mask.cols / 3;
  const int x1 = mask.cols * 2 / 3;
  for (int y = y_start; y < mask.rows; ++y) {
    const int* labels_row = labels_scratch_.ptr<int>(y);
    for (int x = x0; x < x1; ++x) ++seed_counts_scratch_[labels_row[x]];
  }
  int best_seeds = 0;
  for (int label = 1; label < count; ++label) {
    const int area = stats_scratch_.at<int>(label, cv::CC_STAT_AREA);
    if (seed_counts_scratch_[label] > best_seeds &&
        area > static_cast<int>(mask.total() / 500)) {
      best = label;
      best_seeds = seed_counts_scratch_[label];
    }
  }
  cv::Mat& selected = connected_scratch_;
  selected.create(mask.size(), CV_8UC1);
  if (best > 0) cv::inRange(labels_scratch_, cv::Scalar(best), cv::Scalar(best), selected);
  else selected.setTo(0);
  return selected;
}

void RoadDetector::prepareView(const cv::Mat& bgr, cv::Mat& processed, cv::Mat& view) const {
  cv::resize(bgr, processed, cv::Size(config_.process_width, config_.processHeight()));
  if (config_.use_perspective)
    cv::warpPerspective(processed, view, warp_, processed.size(), cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  else view = processed;
}

cv::Mat RoadDetector::segment(const cv::Mat& view) const {
  cv::Mat hsv;
  cv::cvtColor(view, hsv, cv::COLOR_BGR2HSV);
  if (config_.equalize_value) {
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(channels[2], channels[2]);
    cv::merge(channels, hsv);
  }
  const int w = hsv.cols, h = hsv.rows;
  // In a forward view the road can fill the bottom corners. Sample color from
  // mid-range, where both road and outer surface are visible.
  const cv::Rect road_seed(w * 44 / 100, h * 36 / 100, w * 12 / 100, h * 14 / 100);
  const cv::Rect left_seed(w * 2 / 100, h * 36 / 100, w * 8 / 100, h * 14 / 100);
  const cv::Rect right_seed(w * 90 / 100, h * 36 / 100, w * 8 / 100, h * 14 / 100);
  const cv::Scalar road = medianHsv(hsv, road_seed, median_scratch_.channels);
  const cv::Scalar left = medianHsv(hsv, left_seed, median_scratch_.channels);
  const cv::Scalar right = medianHsv(hsv, right_seed, median_scratch_.channels);
  cv::Mat smoothed;
  cv::GaussianBlur(view, smoothed, cv::Size(15, 7), 0);
  const cv::Rect& roi = roi_processed_;
  cv::Mat& candidates = candidates_scratch_;
  candidates.create(hsv.size(), CV_8UC1);
  candidates.setTo(0);
  // The candidate test only compares distances, so it works on squared values
  // and skips the square roots; the HSV range test is a table lookup.
  const float max_distance_squared = config_.color_distance_max * config_.color_distance_max;
  const float background_ratio_squared =
      config_.color_background_ratio * config_.color_background_ratio;
  const float chroma_row_ratio = config_.chroma_row_ratio;
  const float chroma_min_contrast = config_.chroma_min_contrast;
  const HsvLut& lut = hsv_lut_;
  const int x_begin = std::max(0, roi.x);
  const int x_end = std::min(w, roi.x + roi.width);
  const int y_end = std::min(h, roi.y + roi.height);
  // Rows are independent (each row only writes its own output row), so the scan
  // can use the thread pool without changing the result.
  cv::parallel_for_(cv::Range(std::max(0, roi.y), y_end), [&](const cv::Range& range) {
    for (int y = range.start; y < range.end; ++y) {
      const auto* pixels = hsv.ptr<cv::Vec3b>(y);
      const auto* colors = smoothed.ptr<cv::Vec3b>(y);
      auto* row = candidates.ptr<uchar>(y);
      const auto cool = [&](int x) { return static_cast<float>(colors[x][0]) - colors[x][2]; };
      const float outer = std::min(cool(std::clamp(w / 16, 0, w - 1)),
                                   cool(std::clamp(w * 15 / 16, 0, w - 1)));
      float road_cool = -255.0f;
      for (int x = w * 45 / 100; x < w * 55 / 100; x += 3)
        road_cool = std::max(road_cool, cool(x));
      const float cool_threshold = outer + chroma_row_ratio * (road_cool - outer);
      const bool chroma_row = road_cool > outer + chroma_min_contrast;
      for (int x = x_begin; x < x_end; ++x) {
        const cv::Vec3b p = pixels[x];
        if (!lut.within(p)) continue;
        const float dr = hsvDistanceSquared(p, road);
        const float db = std::min(hsvDistanceSquared(p, left), hsvDistanceSquared(p, right));
        const bool hsv_match = dr <= max_distance_squared && dr <= db * background_ratio_squared;
        const bool chroma_match = chroma_row && cool(x) >= cool_threshold;
        if (hsv_match || chroma_match)
          row[x] = 255;
      }
    }
  });
  const int k = config_.morphology_pixels | 1;
  cv::morphologyEx(candidates, candidates, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));
  // The vertical close bridges only short horizontal white bars.
  int estimated_width = std::max(20, w / 3);
  const int sample_y = h * 85 / 100;
  if (candidates.at<uchar>(sample_y, w / 2)) {
    int left_x = w / 2, right_x = w / 2;
    while (left_x > 0 && candidates.at<uchar>(sample_y, left_x - 1)) --left_x;
    while (right_x < w - 1 && candidates.at<uchar>(sample_y, right_x + 1)) ++right_x;
    const int observed_width = right_x - left_x + 1;
    if (observed_width >= config_.min_road_width_fraction * w &&
        observed_width <= config_.max_road_width_fraction * w)
      estimated_width = observed_width;
  }
  int gap = static_cast<int>(estimated_width * config_.max_white_gap_width_ratio);
  gap = std::max(3, std::min(gap | 1, h / 5));
  cv::morphologyEx(candidates, candidates, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, gap)));
  return keepBottomConnected(candidates);
}

RoadResult RoadDetector::process(const cv::Mat& bgr, std::int64_t timestamp_ms) {
  if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols != config_.input_width ||
      bgr.rows != config_.input_height)
    throw std::invalid_argument("expected 1280x720 CV_8UC3 BGR frame (or configured input size)");
  const float previous_center = previous_center_x_;
  cv::Mat processed, view;
  prepareView(bgr, processed, view);
  RoadResult result = analyzeMask(segment(view), timestamp_ms);
  result.roi_polygon = roi_polygon_;
  if (result.type == RoadType::STRAIGHT || result.type == RoadType::UNKNOWN) {
    if (!refineWithEdges(view, result)) {
      previous_center_x_ = previous_center;
      result.valid = false;
      result.type = RoadType::UNKNOWN;
      result.available_directions = 0;
      result.confidence = 0;
      result.lateral_error = 0;
      result.heading_error_rad = 0;
      result.left_boundary.clear();
      result.right_boundary.clear();
      result.centerline.clear();
    }
  }
  return result;
}

RoadResult RoadDetector::processMask(const cv::Mat& mask, std::int64_t timestamp_ms) {
  if (mask.empty() || mask.type() != CV_8UC1 || mask.cols != config_.process_width ||
      mask.rows != config_.processHeight())
    throw std::invalid_argument("mask size/type does not match configured processing size");
  RoadResult result = analyzeMask(keepBottomConnected(mask), timestamp_ms);
  result.roi_polygon = roi_polygon_;
  return result;
}

RoadType RoadDetector::stabilize(RoadType current) {
  recent_types_.push_back(current);
  if (static_cast<int>(recent_types_.size()) > config_.junction_vote_window)
    recent_types_.erase(recent_types_.begin());
  if (current == RoadType::UNKNOWN) return RoadType::UNKNOWN;
  if (current == RoadType::STRAIGHT) return RoadType::STRAIGHT;
  const int count = static_cast<int>(std::count(recent_types_.begin(), recent_types_.end(), current));
  return count >= config_.junction_vote_frames ? current : RoadType::UNKNOWN;
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

bool RoadDetector::refineWithEdges(const cv::Mat& image, RoadResult& result) {
  cv::Mat smooth;
  cv::GaussianBlur(image, smooth, cv::Size(21, 7), 0);
  cv::Mat hsv;
  cv::cvtColor(smooth, hsv, cv::COLOR_BGR2HSV);
  const int w = image.cols, h = image.rows;
  const cv::Scalar road_model = medianHsv(hsv, cv::Rect(w * 44 / 100, h * 36 / 100,
                                                       w * 12 / 100, h * 14 / 100),
                                         median_scratch_.channels);
  const cv::Rect& roi = roi_processed_;
  const int first_y = std::max(static_cast<int>(h * 0.17f), roi.y + 5);
  const int last_y = std::min(h - 15, roi.y + roi.height - 5);
  const auto edgeScore = [&](int y, int x, int side) -> float {
    const cv::Vec3b a = smooth.at<cv::Vec3b>(y, x - 12);
    const cv::Vec3b b = smooth.at<cv::Vec3b>(y, x + 12);
    const float cool_a = static_cast<float>(a[0]) - a[2];
    const float cool_b = static_cast<float>(b[0]) - b[2];
    const float light_a = (static_cast<float>(a[0]) + a[1] + a[2]) / 3.0f;
    const float light_b = (static_cast<float>(b[0]) + b[1] + b[2]) / 3.0f;
    const float chroma_transition = side == 0 ? cool_b - cool_a : cool_a - cool_b;
    float score = chroma_transition + 0.1f * std::abs(light_b - light_a);
    if (side == 1) {
      const cv::Vec3b inner = hsv.at<cv::Vec3b>(y, x - 12);
      const cv::Vec3b outer = hsv.at<cv::Vec3b>(y, x + 12);
      if (!hsv_lut_.within(inner)) return -1000.0f;
      score += std::clamp(0.12f * (hsvDistance(outer, road_model) -
                                    hsvDistance(inner, road_model)), -2.0f, 2.0f);
    }
    return score;
  };
  std::vector<cv::Point2f> left_points;
  float position = -1, velocity = 0;
  int misses = 0;
  for (int y = first_y; y <= last_y; y += 6) {
    int x0, x1;
    if (position < 0) {
      x0 = static_cast<int>(w * 0.10f);
      x1 = static_cast<int>(w * 0.45f);
    } else {
      const int predicted = static_cast<int>(std::round(position + velocity));
      const int radius = 17 + std::min(misses, 2) * 8;
      x0 = predicted - radius;
      x1 = predicted + radius;
    }
    x0 = std::clamp(x0, 13, w - 14);
    x1 = std::clamp(x1, 13, w - 14);
    float best = -std::numeric_limits<float>::infinity();
    int best_x = -1;
    for (int x = x0; x <= x1; ++x) {
      const float displacement = position < 0 ? 0 : x - (position + velocity);
      const float score = edgeScore(y, x, 0) - 0.018f * displacement * displacement;
      if (score > best) { best = score; best_x = x; }
    }
    if (best_x < 0 || edgeScore(y, best_x, 0) < config_.edge_min_score) {
      ++misses;
      continue;
    }
    if (position >= 0)
      velocity = std::clamp(0.6f * velocity + 0.4f * (best_x - position), -12.0f, 12.0f);
    position = static_cast<float>(best_x);
    misses = 0;
    left_points.emplace_back(position, static_cast<float>(y));
  }
  const float minimum_span = config_.edge_min_span_fraction * h;
  LinearModel left;
  if (!robustLine(left_points, config_.edge_line_residual_px,
                  config_.edge_min_inlier_ratio, minimum_span, left)) return false;

  std::vector<CandidateRow> rows;
  for (int y = first_y; y <= last_y; y += 6) {
    CandidateRow row;
    row.y = y;
    const int x0 = std::max({13, static_cast<int>(w * 0.45f),
                             static_cast<int>(left.at(y) +
                                              config_.min_road_width_fraction * w)});
    for (int x = x0 + 3; x < w - 17; ++x) {
      const float score = edgeScore(y, x, 1);
      if (score < config_.edge_min_score || score < edgeScore(y, x - 3, 1) ||
          score < edgeScore(y, x + 3, 1)) continue;
      row.edges.push_back({static_cast<float>(x), score});
    }
    std::sort(row.edges.begin(), row.edges.end(),
              [](const EdgeCandidate& a, const EdgeCandidate& b) { return a.score > b.score; });
    std::vector<EdgeCandidate> distinct;
    for (const auto& candidate : row.edges) {
      const bool separated = std::all_of(distinct.begin(), distinct.end(),
          [&candidate](const EdgeCandidate& accepted) {
            return std::abs(candidate.x - accepted.x) >= 8;
          });
      if (separated) distinct.push_back(candidate);
      if (distinct.size() == 5) break;
    }
    row.edges = std::move(distinct);
    rows.push_back(std::move(row));
  }

  const float target_y = config_.lookahead_fraction * h;
  LinearModel best_right;
  float best_value = -std::numeric_limits<float>::infinity();
  const float min_width = config_.min_road_width_fraction * w;
  for (std::size_t i = 0; i < rows.size(); i += 2) {
    for (std::size_t j = i + 1; j < rows.size(); j += 2) {
      const int dy = rows[j].y - rows[i].y;
      if (dy < minimum_span) continue;
      for (const auto& first : rows[i].edges) {
        for (const auto& last : rows[j].edges) {
          LinearModel hypothesis;
          hypothesis.slope = (last.x - first.x) / dy;
          hypothesis.intercept = first.x - hypothesis.slope * rows[i].y;
          if (!config_.use_perspective && hypothesis.slope - left.slope < -0.08)
            continue;
          const float width = hypothesis.at(target_y) - left.at(target_y);
          if (width < min_width || width > config_.max_road_width_fraction * w)
            continue;
          const auto support = supportForLine(hypothesis, rows, left, min_width,
                                              config_.edge_line_residual_px, w);
          if (!supportIsEnough(support, config_.edge_min_inlier_ratio, minimum_span,
                               config_.edge_line_residual_px, target_y, hypothesis)) continue;
          const float value = support.points.size() * 100.0f + support.score * 0.2f -
                              support.squared_error * 0.15f;
          if (value > best_value) { best_value = value; best_right = hypothesis; }
        }
      }
    }
  }
  if (!std::isfinite(best_value)) return false;
  LinearModel right = best_right;
  for (int iteration = 0; iteration < 3; ++iteration) {
    const auto support = supportForLine(right, rows, left, min_width,
                                        config_.edge_line_residual_px, w);
    if (!supportIsEnough(support, config_.edge_min_inlier_ratio, minimum_span,
                         config_.edge_line_residual_px, target_y, right) ||
        !leastSquaresLine(support.points, right)) return false;
  }
  const auto final_support = supportForLine(right, rows, left, min_width,
                                             config_.edge_line_residual_px, w);
  if (!supportIsEnough(final_support, config_.edge_min_inlier_ratio, minimum_span,
                       config_.edge_line_residual_px, target_y, right)) return false;
  if (!config_.use_perspective && right.slope - left.slope < -0.08) return false;
  const float width = right.at(target_y) - left.at(target_y);
  if (width < min_width || width > config_.max_road_width_fraction * w) return false;
  const int first_observed = std::max(left.first_y, right.first_y);
  const int last_observed = std::min(left.last_y, right.last_y);
  if (last_observed - first_observed < minimum_span || target_y < first_observed ||
      target_y > last_observed) return false;
  const float center = (left.at(target_y) + right.at(target_y)) / 2.0f;
  const float near = (left.at(target_y + 20) + right.at(target_y + 20)) / 2.0f;
  const float far = (left.at(target_y - 20) + right.at(target_y - 20)) / 2.0f;
  result.left_boundary.clear();
  result.right_boundary.clear();
  result.centerline.clear();
  for (int y = first_observed; y <= last_observed; y += 6) {
    const float l = left.at(static_cast<float>(y));
    const float r = right.at(static_cast<float>(y));
    if (l < 0 || r >= w || l >= r) continue;
    result.left_boundary.push_back(sourcePoint({l, static_cast<float>(y)}));
    result.right_boundary.push_back(sourcePoint({r, static_cast<float>(y)}));
    result.centerline.push_back(sourcePoint({(l + r) / 2, static_cast<float>(y)}));
  }
  if (result.centerline.empty()) return false;
  result.lateral_error = (center - vehicle_x_) / (width / 2.0f);
  result.heading_error_rad = std::atan2(far - near, 40.0f);
  const float line_quality = 1.0f - std::clamp((left.rmse + right.rmse) /
      (2.0f * config_.edge_line_residual_px), 0.0f, 1.0f);
  result.confidence = std::clamp(0.35f * left.inlier_ratio +
      0.40f * right.inlier_ratio + 0.15f *
      ((last_observed - first_observed) / static_cast<float>(h)) +
      0.10f * line_quality, 0.0f, 1.0f);
  if (result.confidence < config_.minimum_confidence) return false;
  result.valid = true;
  previous_center_x_ = center;
  return true;
}

RoadResult RoadDetector::analyzeMask(const cv::Mat& mask, std::int64_t timestamp_ms) {
  RoadResult result;
  result.timestamp_ms = timestamp_ms;
  result.road_mask = mask.clone();
  const int w = mask.cols, h = mask.rows;
  std::vector<cv::Point2f> left, right;
  std::vector<float> widths;
  float predicted = previous_center_x_ < 0 ? vehicle_x_ : previous_center_x_;
  const cv::Rect& roi = roi_processed_;
  const int y_top = std::max(1, roi.y);
  const int y_bottom = std::min(h - 2, roi.y + roi.height - 1);
  int examined = 0;
  for (int y = y_bottom; y >= y_top; y -= 6) {
    ++examined;
    const auto* pixels = mask.ptr<uchar>(y);
    int seed = std::clamp(static_cast<int>(predicted), 0, w - 1);
    if (!pixels[seed]) {
      int nearest = -1;
      for (int distance = 1; distance < w / 3; ++distance) {
        if (seed - distance >= 0 && pixels[seed - distance]) { nearest = seed - distance; break; }
        if (seed + distance < w && pixels[seed + distance]) { nearest = seed + distance; break; }
      }
      if (nearest < 0) continue;
      seed = nearest;
    }
    int x0 = seed, x1 = seed;
    while (x0 > 0 && pixels[x0 - 1]) --x0;
    while (x1 < w - 1 && pixels[x1 + 1]) ++x1;
    const float width = static_cast<float>(x1 - x0 + 1);
    predicted = (x0 + x1) / 2.0f;
    if (width < config_.min_road_width_fraction * w ||
        width > config_.max_road_width_fraction * w) continue;
    left.emplace_back(static_cast<float>(x0), static_cast<float>(y));
    right.emplace_back(static_cast<float>(x1), static_cast<float>(y));
    widths.push_back(width);
  }
  if (widths.size() < 5) {
    stabilize(RoadType::UNKNOWN);
    return result;
  }
  std::vector<float> sorted_widths = widths;
  std::nth_element(sorted_widths.begin(), sorted_widths.begin() + sorted_widths.size() / 2,
                   sorted_widths.end());
  const float median_width = sorted_widths[sorted_widths.size() / 2];
  std::vector<cv::Point2f> left_fit, right_fit;
  for (std::size_t i = 0; i < widths.size(); ++i) {
    if (widths[i] >= median_width * 0.55f && widths[i] <= median_width * 1.45f) {
      left_fit.push_back(left[i]);
      right_fit.push_back(right[i]);
    }
  }
  cv::Vec3d lc, rc;
  if (!fitQuadratic(left_fit, lc) || !fitQuadratic(right_fit, rc)) {
    stabilize(RoadType::UNKNOWN);
    return result;
  }
  const float look_y = h * config_.lookahead_fraction;
  const float center_x = (eval(lc, look_y) + eval(rc, look_y)) / 2.0f;
  const float near_x = (eval(lc, look_y + 20) + eval(rc, look_y + 20)) / 2.0f;
  const float far_x = (eval(lc, look_y - 20) + eval(rc, look_y - 20)) / 2.0f;
  const float fitted_width = eval(rc, look_y) - eval(lc, look_y);
  if (!std::isfinite(center_x) || fitted_width < config_.min_road_width_fraction * w ||
      fitted_width > config_.max_road_width_fraction * w) {
    stabilize(RoadType::UNKNOWN);
    return result;
  }
  int consistent = 0;
  for (float width : widths)
    if (std::abs(width - median_width) < median_width * 0.35f) ++consistent;
  const float coverage = static_cast<float>(widths.size()) / std::max(1, examined);
  const float consistency = static_cast<float>(consistent) / widths.size();
  result.confidence = std::clamp(0.65f * coverage + 0.35f * consistency, 0.0f, 1.0f);
  if (result.confidence < config_.minimum_confidence) {
    stabilize(RoadType::UNKNOWN);
    return result;
  }
  result.valid = true;
  result.lateral_error = (center_x - vehicle_x_) / std::max(1.0f, fitted_width / 2.0f);
  result.heading_error_rad = std::atan2(far_x - near_x, 40.0f);
  previous_center_x_ = center_x;
  for (int y = y_bottom; y >= y_top; y -= 6) {
    const float l = eval(lc, static_cast<float>(y));
    const float r = eval(rc, static_cast<float>(y));
    if (l < 0 || r >= w || l >= r) continue;
    result.left_boundary.push_back(sourcePoint({l, static_cast<float>(y)}));
    result.right_boundary.push_back(sourcePoint({r, static_cast<float>(y)}));
    result.centerline.push_back(sourcePoint({(l + r) / 2, static_cast<float>(y)}));
  }
  const Topology topology = topologyFromMask(mask, median_width, config_);
  result.type = stabilize(topology.type);
  if (result.type == topology.type) result.available_directions = topology.directions;
  return result;
}

}  // namespace road_finder
