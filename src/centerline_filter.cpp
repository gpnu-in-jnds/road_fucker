#include "centerline_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace road_finder {
namespace {

constexpr std::size_t kMaxCandidates = 512;
constexpr double kEpsilon = 1e-6;

void leastSquares(const std::vector<cv::Point2f>& points, CenterlineFit& fit) {
  fit.mean_x = 0.0;
  fit.mean_y = 0.0;
  for (const auto i : fit.used) {
    fit.mean_x += points[i].x;
    fit.mean_y += points[i].y;
  }
  fit.mean_x /= fit.used.size();
  fit.mean_y /= fit.used.size();
  double covariance = 0.0;
  fit.y_variance_sum = 0.0;
  for (const auto i : fit.used) {
    const double dy = points[i].y - fit.mean_y;
    covariance += dy * (points[i].x - fit.mean_x);
    fit.y_variance_sum += dy * dy;
  }
  fit.dx_dy = fit.y_variance_sum <= kEpsilon ? 0.0 : covariance / fit.y_variance_sum;
  double squared_error = 0.0;
  for (const auto i : fit.used) {
    const double residual = points[i].x - fit.xAt(points[i].y);
    squared_error += residual * residual;
  }
  fit.rms = std::sqrt(squared_error / fit.used.size());
}

}  // namespace

CenterlineFit fitCenterline(const std::vector<cv::Point2f>& points, int image_width,
                           const CenterlineFilterConfig& config) {
  CenterlineFit fit;
  if (points.empty()) return fit;
  if (!config.enabled || points.size() < 4) {
    fit.used.resize(points.size());
    std::iota(fit.used.begin(), fit.used.end(), 0);
    leastSquares(points, fit);
    fit.status = config.enabled ? "insufficient_points" : "disabled";
    return fit;
  }

  const double tolerance = config.max_residual_fraction * image_width;
  const std::size_t required = std::max<std::size_t>(3, points.size() / 2 + 1);
  const std::size_t pair_count = points.size() * (points.size() - 1) / 2;
  const std::size_t candidate_count = std::min(pair_count, kMaxCandidates);
  double best_error = std::numeric_limits<double>::infinity();
  double best_span = -1.0;
  std::vector<std::size_t> inliers;
  inliers.reserve(points.size());
  // Enumerate small sets; uniformly sample pair indices for large sets. No RNG,
  // temporal state, or thread scheduling can change the candidate sequence.
  std::size_t first = 0;
  std::size_t first_pair = 0;
  for (std::size_t candidate = 0; candidate < candidate_count; ++candidate) {
    const std::size_t pair = candidate * (pair_count - 1) / (candidate_count - 1);
    while (pair >= first_pair + points.size() - first - 1) {
      first_pair += points.size() - first - 1;
      ++first;
    }
    const std::size_t second = first + 1 + pair - first_pair;
    const double dy = static_cast<double>(points[second].y) - points[first].y;
    if (std::abs(dy) <= kEpsilon) continue;
    const double slope = (static_cast<double>(points[second].x) - points[first].x) / dy;
    inliers.clear();
    double squared_error = 0.0;
    double low_y = std::numeric_limits<double>::infinity();
    double high_y = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double predicted = points[first].x + slope * (points[i].y - points[first].y);
      const double residual = points[i].x - predicted;
      if (std::abs(residual) > tolerance + kEpsilon) continue;
      inliers.push_back(i);
      squared_error += residual * residual;
      low_y = std::min(low_y, static_cast<double>(points[i].y));
      high_y = std::max(high_y, static_cast<double>(points[i].y));
    }
    if (inliers.size() < required) continue;
    const double error = squared_error / inliers.size();
    const double span = high_y - low_y;
    if (inliers.size() > fit.used.size() ||
        (inliers.size() == fit.used.size() &&
         (error < best_error - kEpsilon ||
          (std::abs(error - best_error) <= kEpsilon && span > best_span + kEpsilon)))) {
      fit.used = inliers;
      best_error = error;
      best_span = span;
    }
  }

  // Refitting can move a marginal point outside the tolerance. Remove it and
  // repeat; the set only shrinks, so this terminates in at most points.size() fits.
  while (fit.used.size() >= required) {
    leastSquares(points, fit);
    const auto end = std::remove_if(fit.used.begin(), fit.used.end(), [&](std::size_t i) {
      return std::abs(points[i].x - fit.xAt(points[i].y)) > tolerance + kEpsilon;
    });
    if (end == fit.used.end()) {
      fit.status = fit.used.size() == points.size() ? "accepted" : "filtered";
      return fit;
    }
    fit.used.erase(end, fit.used.end());
  }
  fit = CenterlineFit{};
  fit.status = "no_consensus";
  return fit;
}

}  // namespace road_finder
