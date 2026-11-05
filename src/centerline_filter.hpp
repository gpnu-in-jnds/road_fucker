#pragma once

#include "road_finder/road_detector.hpp"

#include <cstddef>

namespace road_finder {

// Internal fitting result. Indices address the original, unmodified observations.
struct CenterlineFit {
  std::vector<std::size_t> used;
  double mean_x = 0.0;
  double mean_y = 0.0;
  double dx_dy = 0.0;
  double y_variance_sum = 0.0;
  double rms = 0.0;
  const char* status = "no_points";

  double xAt(double y) const { return mean_x + dx_dy * (y - mean_y); }
};

CenterlineFit fitCenterline(const std::vector<cv::Point2f>& points, int image_width,
                           const CenterlineFilterConfig& config);

}  // namespace road_finder
