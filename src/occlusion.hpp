#pragma once

#include "road_finder/road_detector.hpp"

namespace road_finder {
struct OcclusionEvidence {
  cv::Mat unknown;
  cv::Mat bridges;
};

// White regions are associated only with measured road edges. Bridges carry
// component connectivity, never geometric measurements.
OcclusionEvidence findOcclusions(const cv::Mat& road, const cv::Mat& white,
                                 cv::Rect roi, const Config& config);
}  // namespace road_finder
