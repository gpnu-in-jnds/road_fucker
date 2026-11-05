#include "../src/centerline_filter.hpp"
#include "road_finder/preview.hpp"
#include "road_finder/uart_sender.hpp"
#include "test_support.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <iostream>
#include <numeric>

namespace {
using namespace road_finder;
using namespace road_finder::test;

cv::Mat stripMask(const std::vector<int>& centers) {
  cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
  const int height = 180 / static_cast<int>(centers.size());
  for (std::size_t i = 0; i < centers.size(); ++i)
    if (centers[i] >= 0)
      mask(cv::Rect(centers[i] - 20, 180 + static_cast<int>(i) * height, 41, height)).setTo(255);
  return mask;
}

void isolatedOutliers() {
  for (const auto& bad : {std::vector<int>{0}, {2}, {5}, {0, 5}, {1, 4}}) {
    std::vector<int> centers(6, 320);
    for (const int i : bad) centers[i] = i % 2 ? 540 : 100;
    const auto mask = stripMask(centers);
    RoadDetector detector(config());
    const auto result = detector.processMask(mask, 1);
    require(result.valid && result.raw_point_count == 6 &&
            result.fit_point_count == 6 - static_cast<int>(bad.size()) &&
            result.centerline_filter_status == "filtered", "isolated outlier count/status");
    require(std::abs(result.lateral_error) < 1e-6 &&
            std::abs(result.heading_error_rad) < 1e-6 && result.fit_residual_px < 1e-6,
            "outliers must not change fitted steering");
    require(std::abs(result.confidence - result.fit_point_count / 6.0) < 1e-6,
            "removed points must still reduce confidence relative to all strips");
    require(static_cast<int>(result.centerline.size()) == result.fit_point_count &&
            result.left_boundary.size() == result.centerline.size(),
            "geometry and width diagnostics must only contain retained rows");
    for (int i = 0; i < 6; ++i) {
      const auto& block = result.scan_blocks[i];
      require(block.found && std::abs(block.midpoint.x - centers[i]) < 1e-6 &&
              block.fit_used == (std::find(bad.begin(), bad.end(), i) == bad.end()),
              "raw observation must survive filtering with an independent fit flag");
    }
    const auto packet = encodeRoadPacket(result, 640);
    require(packet.size == 7 && packet.bytes[3] == 0 && packet.bytes[4] == 0 &&
            packet.bytes[5] == 0 && packet.bytes[6] == 0,
            "UART must use the refitted centerline, including a rejected bottom point");
    const auto recovered = detector.processMask(road(), 2, true);
    require(recovered.fit_point_count == 6 && recovered.centerline_filter_status == "accepted",
            "point rejection must not persist into later frames");
    if (bad == std::vector<int>{0, 5}) {
      cv::imwrite("centerline_filtered_overlay.png", annotateResult(image(mask), result));
      cv::imwrite("centerline_filtered_process.png", annotateResult(image(mask), result, true));
    }
  }
}

void validGeometryAndCompatibility() {
  for (const auto& centers : {std::vector<int>{220, 220, 220, 220, 220, 220},
                             {420, 420, 420, 420, 420, 420},
                             {260, 284, 308, 332, 356, 380},
                             {380, 356, 332, 308, 284, 260}}) {
    const auto result = detect(stripMask(centers));
    const double slope = (centers.back() - centers.front()) / 150.0;
    require(result.valid && result.fit_point_count == 6 &&
            result.centerline_filter_status == "accepted" &&
            std::abs(result.heading_error_rad - std::atan(-slope)) < 1e-6,
            "offset or slanted roads must not be treated as outliers");
  }
  const auto empty = detect(cv::Mat::zeros(360, 640, CV_8UC1));
  require(!empty.valid && empty.centerline_filter_status == "no_points" &&
          empty.raw_point_count == 0 && empty.fit_point_count == 0, "empty frame diagnostics");
  for (int n : {1, 2, 3}) {
    std::vector<int> centers(6, -1);
    for (int i = 0; i < n; ++i) centers[5 - i] = 280 + i * 20;
    const auto result = detect(stripMask(centers));
    require(result.valid && result.raw_point_count == n && result.fit_point_count == n &&
            result.centerline_filter_status == "insufficient_points",
            "one to three observations retain legacy fitting and report insufficient evidence");
    if (n == 1) require(!result.road_end_detected, "fallback endpoints are not two measurements");
  }
  auto disabled = config();
  disabled.centerline_filter.enabled = false;
  const auto result = RoadDetector(disabled).processMask(stripMask({320,320,320,320,320,540}), 1);
  require(result.valid && result.fit_point_count == 6 && result.centerline_filter_status == "disabled" &&
          result.lateral_error > 5 && std::abs(result.heading_error_rad) > 0.5,
          "disabled filtering must retain old all-point fit and raw-bottom offset");
}

void noConsensus() {
  const auto mask = stripMask({100, 540, 100, 540, 100, 540});
  auto value = config();
  value.road_end_roi = {280, 240, 80, 50};  // Completely empty, but no usable fit.
  const auto result = RoadDetector(value).processMask(mask, 1);
  require(!result.valid && result.type == RoadType::Unknown && !result.road_end_detected &&
          result.road_end_region.coverage == 1.0f && result.raw_point_count == 6 &&
          result.fit_point_count == 0 && result.centerline.empty() &&
          result.centerline_filter_status == "no_consensus", "equal groups cannot supply majority geometry");
  for (const auto& block : result.scan_blocks)
    require(block.found && !block.fit_used, "no-consensus observations must remain available as black points");
  const auto packet = encodeRoadPacket(result, 640);
  require(packet.size == 3 && packet.bytes[2] == 2, "no consensus must publish Unknown to UART");
}

void thresholdAndRefit() {
  CenterlineFilterConfig filter;
  // Symmetric offsets at the same height make the final residual exactly d.
  // Duplicate heights cannot generate a candidate, but still count as observations.
  for (float d : {15.0f, 16.0f, 17.0f}) {
    std::vector<cv::Point2f> points{{320,0}, {320,10}, {320,20}, {320,30}, {320,40}, {320,50},
                                     {320-d,25}, {320+d,25}};
    const auto fit = fitCenterline(points, 640, filter);
    require(d <= 16 ? fit.used.size() == 8 : fit.used.size() < 8,
            "inclusive threshold retains +/-16 but cannot retain both +/-17 at the same height");
    if (d <= 16)
      require(std::abs(fit.dx_dy) < 1e-6 && std::abs(fit.xAt(25) - 320) < 1e-6,
              "symmetric inliers should preserve the analytic line");
  }
  std::vector<cv::Point2f> points;
  for (int i = 0; i < 120; ++i) {
    const float x = i % 7 == 0 ? (i % 2 ? 550.0f : 80.0f) : 250.0f + i * 0.5f;
    points.emplace_back(x, static_cast<float>(i * 2));
  }
  const auto fit = fitCenterline(points, 640, filter);
  require(fit.used.size() == 102 && std::abs(fit.dx_dy - 0.25) < 1e-6,
          "bounded candidate sampling must reject the known outliers in a large point set");
  for (int repeat = 0; repeat < 5; ++repeat) {
    const auto repeated = fitCenterline(points, 640, filter);
    require(repeated.used == fit.used && repeated.dx_dy == fit.dx_dy,
            "candidate selection must be deterministic");
  }
  // Bounded noise plus isolated errors: every retained point must satisfy the
  // threshold of the FINAL least-squares line, not only the initial candidate.
  for (int variant = 0; variant < 30; ++variant) {
    for (int i = 0; i < 24; ++i)
      points[i] = {static_cast<float>(320 + ((i * 17 + variant * 13) % 39) - 19),
                   static_cast<float>(i * 7)};
    points.resize(24);
    const auto refit = fitCenterline(points, 640, filter);
    require(refit.used.empty() || refit.used.size() > points.size() / 2,
            "refit cannot weaken majority requirement");
    for (const auto i : refit.used)
      require(std::abs(points[i].x - refit.xAt(points[i].y)) <= 16 + 1e-6,
              "retained point exceeds final residual threshold");
  }
}

void scaleThreadsAndPerspective() {
  const auto original = stripMask({320, 320, 100, 320, 320, 540});
  for (int width : {320, 640, 1280}) {
    for (int threads : {1, 2, 4}) {
      auto value = config();
      value.process_width = width;
      value.threads = threads;
      cv::Mat mask;
      cv::resize(original, mask, {width, value.processHeight()}, 0, 0, cv::INTER_NEAREST);
      const auto result = RoadDetector(value).processMask(mask, 1);
      require(result.valid && result.fit_point_count == 4 * threads &&
              result.raw_point_count == 6 * threads && std::abs(result.heading_error_rad) < 1e-6,
              "resolution/threads must preserve the supported line and reject the same road regions");
      require(std::abs(result.centerline.back().x - 320) <= 0.5,
              "source-coordinate line must survive resize quantization");
    }
  }
  auto value = config();
  value.use_perspective = true;
  value.perspective_src = {{{0,0}, {639,0}, {639,359}, {0,359}}};
  value.perspective_dst = {{{20,0}, {659,0}, {659,359}, {20,359}}};
  // Keep each synthetic patch away from strip edges so the Gaussian prefilter
  // cannot leak a distant patch into a neighbor's mask bounding rectangle.
  cv::Mat separated = original.clone();
  for (int y = 180; y < 360; y += 30) {
    separated.rowRange(y, y + 2).setTo(0);
    separated.rowRange(y + 28, y + 30).setTo(0);
  }
  const auto result = RoadDetector(value).process(image(separated), 1);
  require(result.valid && result.fit_point_count == 4 &&
          std::abs(result.centerline.back().x - 320) < 1,
          "filter must run in the warped view and return the fitted line in source coordinates");
  require(!result.scan_blocks[5].fit_used && std::abs(result.scan_blocks[5].midpoint.x - 540) < 1,
          "rejected point must keep its inverse-transformed source position");
}

}  // namespace

int main() {
  try {
    isolatedOutliers();
    validGeometryAndCompatibility();
    noConsensus();
    thresholdAndRefit();
    scaleThreadsAndPerspective();
    std::cout << "centerline filter tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "centerline filter tests failed: " << error.what() << '\n';
    return 1;
  }
}
