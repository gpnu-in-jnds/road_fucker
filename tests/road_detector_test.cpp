#include "test_support.hpp"

#include <algorithm>
#include <iostream>

namespace {
using namespace road_finder;
using namespace road_finder::test;

void hsvThresholds() {
  // Flat swatches sweep every H/S/V value, followed by mixed-channel colors.
  // Compare the public BGR processing path against scalar interval membership.
  cv::Mat colors(360, 640, CV_8UC3);
  for (int i = 0; i < 32 * 30; ++i) {
    cv::Scalar color;
    if (i < 180) color = {static_cast<double>(i), 255, 255};
    else if (i < 436) color = {0, static_cast<double>(i - 180), 255};
    else if (i < 692) color = {0, 255, static_cast<double>(i - 436)};
    else color = {static_cast<double>((i * 37) % 180),
                  static_cast<double>((i * 53) % 256),
                  static_cast<double>((i * 71) % 256)};
    colors(cv::Rect((i % 32) * 20, (i / 32) * 12, 20, 12)).setTo(color);
  }
  cv::Mat frame, blurred, actual_hsv;
  cv::cvtColor(colors, frame, cv::COLOR_HSV2BGR);
  cv::GaussianBlur(frame, blurred, cv::Size(3, 3), 0);
  cv::cvtColor(blurred, actual_hsv, cv::COLOR_BGR2HSV);
  // Use actual converted values so BGR quantization cannot hide boundary failures.
  const auto contains = [](int value, int first, int last) {
    return first <= last ? value >= first && value <= last
                         : value >= first || value <= last;
  };
  const std::vector<HsvRange> ranges{
      {0, 179, 0, 255, 0, 255}, {10, 20, 10, 200, 10, 200},
      {50, 20, 0, 255, 0, 255}, {170, 10, 10, 150, 40, 230},
      {0, 179, 200, 40, 0, 255}, {0, 179, 0, 255, 230, 80},
      {50, 20, 200, 40, 0, 255}, {50, 20, 0, 255, 230, 80},
      {0, 179, 200, 40, 230, 80}, {50, 20, 200, 40, 230, 80},
      {20, 20, 0, 255, 0, 255}, {0, 179, 20, 20, 0, 255},
      {0, 179, 0, 255, 20, 20}, {0, 0, 0, 255, 0, 255},
      {179, 179, 0, 255, 0, 255}, {0, 179, 0, 0, 0, 255},
      {0, 179, 255, 255, 0, 255}, {0, 179, 0, 255, 0, 0},
      {0, 179, 0, 255, 255, 255}, {179, 0, 255, 0, 255, 0}};
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    auto value = config();
    value.morphology_pixels = 1;  // Isolate thresholding from opening.
    value.road_hsv = ranges[i];
    const auto result = RoadDetector(value).process(frame, i);
    for (int y = 0; y < frame.rows; ++y) {
      for (int x = 0; x < frame.cols; ++x) {
        const auto pixel = actual_hsv.at<cv::Vec3b>(y, x);
        const auto& range = ranges[i];
        const bool expected = contains(pixel[0], range.h_min, range.h_max) &&
                              contains(pixel[1], range.s_min, range.s_max) &&
                              contains(pixel[2], range.v_min, range.v_max);
        require((result.road_mask.at<unsigned char>(y, x) != 0) == expected,
                "HSV interval mismatch in case " + std::to_string(i) +
                " at " + std::to_string(x) + "," + std::to_string(y));
      }
    }
  }
}

void geometry() {
  static_assert(static_cast<int>(RoadType::Straight) == 0);
  static_assert(static_cast<int>(RoadType::RoadEnd) == 1);
  static_assert(static_cast<int>(RoadType::Unknown) == 2);
  const auto centered = detect(road());
  require(centered.valid && centered.type == RoadType::Straight, "centered road must be Straight");
  require(centered.centerline.size() == 6, "all six strips must contribute geometry");
  require(std::abs(centered.lateral_error) < 1e-5 &&
          std::abs(centered.heading_error_rad) < 1e-5, "centered geometry must be zero");
  for (const auto& point : centered.centerline)
    require(std::abs(point.x - 320.0f) < 0.01f, "fit must remain on analytic center");
  require(detect(road(0, 360)).lateral_error > 0.5f, "right offset must be positive");
  require(detect(road(0, 280)).lateral_error < -0.5f, "left offset must be negative");
  cv::Mat angled = cv::Mat::zeros(360, 640, CV_8UC1);
  for (int y = 0; y < angled.rows; ++y) {
    const int x = cvRound(320 + 0.15 * (359 - y));
    angled(cv::Rect(x - 40, y, 81, 1)).setTo(255);
  }
  const auto fit = detect(angled);
  require(fit.valid && fit.type == RoadType::Straight, "slanted road is still a tracking state");
  require(std::abs(fit.heading_error_rad - std::atan(0.15)) < 0.01,
          "heading must match analytic slope");
  const auto empty = detect(cv::Mat::zeros(360, 640, CV_8UC1));
  require(!empty.valid && empty.type == RoadType::Unknown && empty.centerline.empty(),
          "lost road must be Unknown without geometry");
  cv::Mat detached = road();
  detached.rowRange(300, 360).setTo(0);
  require(detect(detached).valid && !detect(detached).road_end_detected,
          "filled narrow masks contribute even without bottom-connected road");
  cv::Mat full(360, 640, CV_8UC1, cv::Scalar(255));
  const auto full_mask = detect(full);
  require(full_mask.valid && full_mask.centerline.size() == 6 &&
          std::abs(full_mask.centerline.back().x - 319.5f) < 0.01f,
          "a full-width mask contributes its geometric centroid");
  cv::Mat clipped = road(0, 40);
  require(detect(clipped).valid && detect(clipped).road_end_detected,
          "edge-touching road supplies centroids and does not veto an empty end box");

  cv::Mat one_strip = cv::Mat::zeros(360, 640, CV_8UC1);
  one_strip(cv::Rect(0, 300, 640, 30)).setTo(255);
  const auto single = detect(one_strip);
  require(single.valid && single.type == RoadType::Straight && single.centerline.size() == 2 &&
          single.scan_blocks[4].found && !single.scan_blocks[3].found,
          "one nonempty strip must produce a centerline without prior history");
  require(std::abs(single.scan_blocks[4].midpoint.x - 319.5f) < 0.01f &&
          std::abs(single.scan_blocks[4].midpoint.y - 314.5f) < 0.01f &&
          std::abs(single.centerline.back().x - single.scan_blocks[4].midpoint.x) < 0.01f,
          "single-strip line must pass through the exact mask centroid");

  cv::Mat t_junction = cv::Mat::zeros(360, 640, CV_8UC1);
  t_junction(cv::Rect(0, 180, 401, 30)).setTo(255);
  t_junction(cv::Rect(300, 180, 41, 180)).setTo(255);
  const auto junction = detect(t_junction);
  require(junction.valid && junction.scan_blocks.front().found && junction.centerline.size() == 6,
          "a wide T-junction mask must contribute even without road-edge pairs");
  const double expected = 200.0;  // The centered stem is contained by the strip crossbar.
  require(std::abs(junction.scan_blocks.front().midpoint.x - expected) < 0.01,
          "T-junction block must use all mask pixels for its centroid");
}

void stripCoverage() {
  RoadDetector detector(config());
  // Anchors fix the mask bounds at 640 * 30: exactly 60% is 11520 pixels.
  for (int count : {11519, 11520, 11521}) {
    cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
    for (int i = 0; i < count - 1; ++i)
      mask.at<unsigned char>(300 + i / 640, i % 640) = 1;
    mask.at<unsigned char>(329, 639) = 1;
    const auto result = detector.processMask(mask, count, true);
    const bool accepted = count > 11520;
    require(result.valid == accepted && result.scan_blocks[4].found == accepted,
            "strip centroid requires strictly greater than 60% fill of mask bounds");
    require(std::abs(result.scan_blocks[4].coverage - count / 19200.0f) < 1e-6f,
            "coverage uses nonzero pixel count divided by mask bounding-box area");
    require(result.type == (accepted ? RoadType::Straight : RoadType::Unknown),
            "no qualified strip and no end evidence must produce Unknown");
    if (!accepted) {
      require(result.centerline.empty(), "rejected strips must not generate fitted geometry");
    } else {
      // 18 complete rows plus pixel (639,329), irrespective of pixel intensity.
      const float expected_x = (319.5f * 11520 + 639) / 11521;
      require(std::abs(result.scan_blocks[4].midpoint.x - expected_x) < 0.001f,
              "qualified strip centroid must include all nonzero mask pixels");
    }
  }
  cv::Mat mixed = cv::Mat::zeros(360, 640, CV_8UC1);
  mixed(cv::Rect(0, 180, 384, 30)).setTo(255);
  mixed.at<unsigned char>(180, 0) = 0;
  mixed.at<unsigned char>(209, 639) = 255;  // Expand bounds: exactly 60%, rejected.
  mixed(cv::Rect(280, 300, 81, 30)).setTo(255);  // Narrow but 100% filled.
  const auto result = detector.processMask(mixed, 12000, true);
  require(result.valid && !result.scan_blocks.front().found && result.scan_blocks[4].found &&
          std::abs(result.centerline.back().x - 320.0f) < 0.01f &&
          std::abs(result.scan_blocks[4].coverage - 1.0f) < 1e-6f,
          "rejected blocks must not influence the remaining qualified centroid");
  mixed(cv::Rect(280, 300, 81, 30)).setTo(0);
  const auto lost = detector.processMask(mixed, 12030, true);
  require(!lost.valid && lost.type == RoadType::Unknown && lost.centerline.empty(),
          "falling below coverage must immediately clear prior geometry");
}

void endpoints() {
  auto value = config();
  value.road_end_roi = {10, 10, 10, 10};  // Independent of the tracking ROI.
  RoadDetector detector(value);
  for (int missing : {79, 80, 81}) {
    cv::Mat mask = road();
    mask(value.road_end_roi).setTo(1);  // Every nonzero value means road.
    for (int i = 0; i < missing; ++i)
      mask.at<unsigned char>(10 + i / 10, 10 + i % 10) = 0;
    const auto result = detector.processMask(mask, missing, true);
    require(result.valid && result.road_end_detected == (missing >= 80) &&
            result.road_end_region.found == (missing >= 80),
            "endpoint requires at least 80% non-mask, including exactly 80%");
    require(std::abs(result.road_end_region.coverage - missing / 100.0f) < 1e-6f,
            "endpoint coverage is the full box non-mask area fraction");
    require(result.type == (missing >= 80 ? RoadType::RoadEnd : RoadType::Straight) &&
            result.raw_type == result.type && result.road_end_boundary.empty(),
            "current-frame area detection does not fit a terminal boundary or debounce");
  }
  for (int points : {0, 1, 2}) {
    cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
    if (points >= 1) mask(cv::Rect(280, 300, 81, 30)).setTo(255);
    if (points >= 2) mask(cv::Rect(280, 330, 81, 30)).setTo(255);
    const auto result = detector.processMask(mask, 100 + points);
    require(std::count_if(result.scan_blocks.begin(), result.scan_blocks.end(),
                         [](const BlockObservation& block) { return block.found; }) == points,
            "gate must count measured strip centroids");
    require(result.road_end_region.coverage == 1.0f &&
            result.type == (points == 0 ? RoadType::Unknown :
                            points == 1 ? RoadType::Straight : RoadType::RoadEnd),
            "an empty end box alone cannot replace two measured centroids");
    if (points == 1)
      require(result.valid && result.centerline.size() == 2 && !result.road_end_detected,
              "single-point vertical fallback must not count as two measured points");
  }
  cv::Mat invalid_second = cv::Mat::zeros(360, 640, CV_8UC1);
  invalid_second(cv::Rect(280, 300, 81, 30)).setTo(255);
  invalid_second.at<unsigned char>(330, 0) = 255;
  invalid_second.at<unsigned char>(359, 639) = 255;
  require(detector.processMask(invalid_second, 200).type == RoadType::Straight,
          "a strip below 60% mask fill cannot supply the second endpoint centroid");

  cv::Mat discontinuous = cv::Mat::zeros(360, 640, CV_8UC1);
  for (int i = 0; i < 6; ++i)
    discontinuous(cv::Rect(i % 2 ? 540 : 0, 180 + i * 30, 81, 30)).setTo(255);
  require(detector.processMask(discontinuous, 201).type == RoadType::RoadEnd,
          "fit residual, disconnection and edge contact must not add endpoint conditions");

  const auto normal = detect(road());
  require(normal.type == RoadType::Straight && !normal.road_end_detected,
          "a box containing road must not trigger");
  const auto end = detect(road(320));
  require(end.type == RoadType::RoadEnd && end.valid,
          "two real centroids and an empty box trigger immediately");
}

void temporal() {
  RoadDetector detector(config());
  const cv::Mat end = road(320);
  for (std::int64_t timestamp : {30, 60, 1000, 900}) {
    const auto result = detector.processMask(end, timestamp, true);
    require(result.type == RoadType::RoadEnd && result.raw_type == RoadType::RoadEnd &&
            result.road_end_detected,
            "RoadEnd is immediate regardless of temporal flag, stream gap or timestamps");
    require(detector.processMask(road(), timestamp + 1, true).type == RoadType::Straight,
            "RoadEnd is not latched after the area condition clears");
    const auto lost = detector.processMask(cv::Mat::zeros(end.size(), CV_8UC1), timestamp + 2, true);
    require(lost.type == RoadType::Unknown && lost.centerline.empty(),
            "an empty frame immediately clears both geometry and RoadEnd");
  }
  require(detector.processMask(end, 1).type == RoadType::RoadEnd,
          "still images use the same immediate conditions");
}

void currentMaskGeometry() {
  RoadDetector detector(config());
  detector.processMask(road(), 30, true);
  const auto current = detector.processMask(road(0, 340), 60, true);
  require(current.valid && std::abs(current.centerline.back().x - 340) < 0.01,
          "each frame geometry must follow its current mask without smoothing");
  detector.processMask(cv::Mat::zeros(360, 640, CV_8UC1), 90, true);
  require(std::abs(detector.processMask(road(0, 340), 120, true).centerline.back().x - 340) < 0.01,
          "reacquisition must not reuse geometry from before loss");
}
}  // namespace

int main() {
  try {
    hsvThresholds();
    geometry();
    stripCoverage();
    endpoints();
    temporal();
    currentMaskGeometry();
    std::cout << "road detector tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "road detector tests failed: " << error.what() << '\n';
    return 1;
  }
}
