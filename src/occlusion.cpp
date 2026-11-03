#include "occlusion.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace road_finder {
namespace {
struct Span {
  int left = 0;
  int right = 0;  // Exclusive.
  int width() const { return right - left; }
  float center() const { return (left + right) * 0.5f; }
};

bool compatible(const Span& a, const Span& b) {
  return a.width() > 0 && b.width() > 0 &&
      std::min(a.width(), b.width()) >= 0.7f * std::max(a.width(), b.width()) &&
      std::abs(a.center() - b.center()) <= 0.35f * std::min(a.width(), b.width());
}
}  // namespace

OcclusionEvidence findOcclusions(const cv::Mat& road, const cv::Mat& white,
                                 cv::Rect roi, const Config& config) {
  OcclusionEvidence result{cv::Mat::zeros(road.size(), CV_8UC1),
                           cv::Mat::zeros(road.size(), CV_8UC1)};
  if (white.empty() || cv::countNonZero(white) == 0) return result;
  // Search above the steering ROI too, so distant measured road can explain
  // a white gap. Keep the gap budget relative to the steering ROI.
  const cv::Rect search(roi.x, 0, roi.width, roi.y + roi.height);
  std::vector<Span> profile(road.rows);
  for (int y = search.y; y < roi.y + roi.height; ++y) {
    const auto* row = road.ptr<unsigned char>(y);
    float best_score = 1e9f;
    for (int x = roi.x; x < roi.x + roi.width;) {
      if (!row[x]) { ++x; continue; }
      const int left = x;
      while (x < roi.x + roi.width && row[x]) ++x;
      const Span span{left, x};
      if (span.width() < road.cols * config.min_road_width_fraction ||
          span.width() > road.cols * config.max_road_width_fraction ||
          left <= roi.x + 1 || x >= roi.x + roi.width - 1)
        continue;
      const float score = std::abs(span.center() - road.cols * 0.5f);
      if (score < best_score) { profile[y] = span; best_score = score; }
    }
  }
  const auto supported = [&](int y, int direction) {
    for (int step = 1; step <= 2; ++step) {
      const int neighbor = y + direction * step;
      if (neighbor < search.y || neighbor >= roi.y + roi.height ||
          !compatible(profile[y], profile[neighbor])) return false;
    }
    return true;
  };
  const auto slope = [&](int y, int direction) {
    int other = y;
    const int extent = std::max(3, road.rows / 60);
    for (int step = 1; step <= extent; ++step) {
      const int neighbor = y + direction * step;
      if (neighbor < search.y || neighbor >= search.y + search.height ||
          !compatible(profile[y], profile[neighbor])) break;
      other = neighbor;
    }
    return other == y ? 0.0f : (profile[other].center() - profile[y].center()) / (other - y);
  };
  const auto corridor = [&](int top, int bottom, const Span& upper, const Span& lower,
                             bool bridge) {
    if (bottom - top < 3) return;
    std::vector<cv::Rect> rows;
    for (int y = top; y < bottom; ++y) {
      const float t = static_cast<float>(y - top + 1) / (bottom - top + 1);
      const int left = cvRound(upper.left + t * (lower.left - upper.left));
      const int right = cvRound(upper.right + t * (lower.right - upper.right));
      const cv::Rect region(left, y, right - left, 1);
      // Every row must be mostly white; shadows/background gaps are not bridged.
      if (region.width <= 0 || cv::countNonZero(white(region)) < region.width * 0.8f)
        return;
      rows.push_back(region);
    }
    for (const auto& region : rows) {
      result.unknown(region).setTo(255);
      if (bridge) white(region).copyTo(result.bridges(region));
    }
  };
  int last = -1;
  for (int y = search.y; y < roi.y + roi.height; ++y) {
    if (!profile[y].width()) continue;
    if (last >= 0 && y - last > 1 && supported(last, -1) && supported(y, 1) &&
        compatible(profile[last], profile[y])) {
      const int gap = y - last - 1;
      const float upper_slope = slope(last, -1);
      const float lower_slope = slope(y, 1);
      const bool direction_matches =
          std::abs(std::atan(upper_slope) - std::atan(lower_slope)) <= 0.45f &&
          std::abs(profile[y].center() - profile[last].center() -
              (upper_slope + lower_slope) * 0.5f * (gap + 1)) <=
              0.35f * std::min(profile[last].width(), profile[y].width());
      corridor(last + 1, y, profile[last], profile[y],
               direction_matches && gap <= config.occlusion_max_gap_fraction * roi.height);
    }
    last = y;
  }
  // A white region reaching the bottom has no second measured edge. Mark it
  // unknown, without fabricating a bridge to the vehicle or old frame geometry.
  if (last >= 0 && last < roi.y + roi.height - 1 && supported(last, -1))
    corridor(last + 1, roi.y + roi.height, profile[last], profile[last], false);
  return result;
}
}  // namespace road_finder
