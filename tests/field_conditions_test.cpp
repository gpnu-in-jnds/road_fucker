#include "road_finder/road_detector.hpp"
#include "road_finder/preview.hpp"
#include "road_finder/uart_sender.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

road_finder::Config configFor(int threads) {
  road_finder::Config config;
  config.threads = threads;
  config.roi = {0, 360, 1280, 360};
  return config;
}

cv::Rect scaled(const cv::Rect& reference) {
  return {reference.x * 5, reference.y * 4, reference.width * 5, reference.height * 4};
}

cv::Mat junction() {
  cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(mask, {0, 60}, {639, 240}, cv::Scalar(255), -1);
  cv::rectangle(mask, {280, 240}, {360, 359}, cv::Scalar(255), -1);
  return mask;
}

void expectVotes() {
  for (int threads : {1, 2, 4}) {
    const auto config = configFor(threads);
    road_finder::RoadDetector detector(config);
    for (int left = 0; left <= 3; ++left) {
      for (int right = 0; right <= 3; ++right) {
        cv::Mat mask = junction();
        for (int i = left; i < 3; ++i)
          mask(scaled(config.left_marker_rois[i])).setTo(0);
        for (int i = right; i < 3; ++i)
          mask(scaled(config.right_marker_rois[i])).setTo(0);
        const auto result = detector.processMask(mask, 1);
        require(result.left_junction_hits == left && result.right_junction_hits == right,
                "each side must count exactly its observed boxes");
        require(result.type == (left >= 2 && right >= 2 ? road_finder::RoadType::T_JUNCTION
                                                       : road_finder::RoadType::UNKNOWN),
                "topology must not bypass two votes or downgrade a partial T to a turn");
      }
    }
    for (bool left_side : {false, true}) {
      cv::Mat turn = junction();
      turn(cv::Rect(left_side ? 361 : 0, 0, left_side ? 279 : 280, 240)).setTo(0);
      const auto expected = left_side ? road_finder::RoadType::LEFT_TURN
                                     : road_finder::RoadType::RIGHT_TURN;
      const auto single = detector.processMask(turn, 2);
      if (single.type != expected)
        std::cerr << "single exit threads=" << threads << " left=" << left_side
                  << " type=" << road_finder::toString(single.type) << " votes="
                  << single.left_junction_hits << "/" << single.right_junction_hits << '\n';
      require(single.type == expected, "supported single exit should remain a turn");
      const auto& boxes = left_side ? config.left_marker_rois : config.right_marker_rois;
      turn(scaled(boxes[1])).setTo(0);
      turn(scaled(boxes[2])).setTo(0);
      require(detector.processMask(turn, 3).type == road_finder::RoadType::UNKNOWN,
              "single turn must also satisfy two votes");
    }
  }
  auto config = configFor(2);
  config.classification_hold_frames = 5;
  road_finder::RoadDetector video(config);
  require(video.processMask(junction(), 100, true).type == road_finder::RoadType::T_JUNCTION,
          "video must first accept the measured T");
  cv::Mat lost_votes = junction();
  lost_votes(scaled(config.left_marker_rois[1])).setTo(0);
  lost_votes(scaled(config.left_marker_rois[2])).setTo(0);
  const auto rejected = video.processMask(lost_votes, 167, true);
  require(rejected.type == road_finder::RoadType::UNKNOWN && rejected.available_directions == 0,
          "temporal hold must not bypass current votes");
  require(road_finder::encodeRoadPacket(rejected, 1280).size == 3,
          "rejected junction must send the existing UNKNOWN packet");
}

cv::Mat coloredRoad(int top, int bottom, int value = 230) {
  cv::Mat hsv(720, 1280, CV_8UC3, cv::Scalar(30, 120, 130));
  cv::rectangle(hsv, {500, 0}, {780, 719}, cv::Scalar(110, 100, 180), -1);
  if (bottom > top)
    hsv(cv::Rect(500, top, 281, bottom - top)).setTo(cv::Scalar(0, 0, value));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

void expectSideThresholds() {
  auto config = configFor(2);
  config.road_hsv = {90, 125, 80, 170, 120, 230};
  cv::Mat hsv(720, 1280, CV_8UC3, cv::Scalar(30, 80, 150));
  cv::Mat mask;
  cv::resize(junction(), mask, hsv.size(), 0, 0, cv::INTER_NEAREST);
  hsv.setTo(cv::Scalar(110, 120, 190), mask);
  for (const cv::Rect& region : {cv::Rect(0, 0, 360, 720), cv::Rect(920, 0, 360, 720)})
    hsv(region).setTo(cv::Scalar(110, 120, 65), mask(region));
  cv::Mat frame;
  cv::cvtColor(hsv, frame, cv::COLOR_HSV2BGR);
  road_finder::RoadDetector central_only(config);
  const auto missed = central_only.process(frame, 1);
  require(missed.left_junction_hits == 0 && missed.right_junction_hits == 0,
          "central threshold must demonstrate dark-side failure");
  config.side_road_hsv = road_finder::HsvRange{90, 125, 80, 170, 30, 100};
  road_finder::RoadDetector partitioned(config);
  for (bool mirror : {false, true}) {
    cv::Mat input;
    if (mirror) cv::flip(frame, input, 1); else input = frame;
    const auto result = partitioned.process(input, 2);
    require(result.type == road_finder::RoadType::T_JUNCTION &&
                result.left_junction_hits == 3 && result.right_junction_hits == 3,
            "one shared side threshold must recover both dark sides");
    require(result.road_mask.at<unsigned char>(300, 320) != 0,
            "side filtering must not remove the bright central stem");
  }
}

void expectOcclusion() {
  for (int threads : {1, 2, 4}) {
    auto config = configFor(threads);
    road_finder::RoadDetector detector(config);
    const auto repaired = detector.process(coloredRoad(480, 540), 1);
    require(repaired.occlusion_detected && cv::countNonZero(repaired.bridge_mask) > 0,
            "bounded white gap must be associated using both measured road edges");
    require(repaired.valid && repaired.type == road_finder::RoadType::STRAIGHT,
            "bounded speed bump should retain measured straight geometry");
    require(cv::countNonZero(repaired.bridge_mask & repaired.road_mask) == 0,
            "bridge pixels must not enter the measured road mask");
    for (const auto& block : repaired.scan_blocks)
      if (block.found)
        require(repaired.road_mask.at<unsigned char>(cvRound(block.midpoint.y * 0.5f),
                    cvRound(block.midpoint.x * 0.5f)) != 0 &&
                repaired.bridge_mask.at<unsigned char>(cvRound(block.midpoint.y * 0.5f),
                    cvRound(block.midpoint.x * 0.5f)) == 0,
                "white bridge cannot fabricate measured centers");
    const auto unbounded = detector.process(coloredRoad(380, 680), 2);
    require(unbounded.occlusion_detected && cv::countNonZero(unbounded.bridge_mask) == 0 &&
                !unbounded.valid && unbounded.centerline.empty(),
            "large white gap with little road visible must not fabricate geometry");
    require(unbounded.type == road_finder::RoadType::UNKNOWN,
            "unobservable white region must not be a turn or inherit straight geometry");
    const auto shadow = detector.process(coloredRoad(480, 540, 80), 3);
    require(!shadow.occlusion_detected && cv::countNonZero(shadow.bridge_mask) == 0,
            "a dark disconnected gap must not be treated as a white speed bump");
    const auto floor = detector.process(cv::Mat(720, 1280, CV_8UC3, cv::Scalar::all(230)), 4);
    require(!floor.valid && !floor.occlusion_detected && cv::countNonZero(floor.bridge_mask) == 0,
            "white background alone supplies no road corridor");
  }
  auto config = configFor(4);
  road_finder::RoadDetector detector(config);
  const auto bottom = detector.process(coloredRoad(660, 720), 1);
  require(bottom.occlusion_detected && bottom.valid && !bottom.centerline.empty(),
          "visible road may be reacquired across a limited white bottom region");
  float last_measured_y = 0;
  for (const auto& block : bottom.scan_blocks)
    if (block.found) last_measured_y = std::max(last_measured_y, block.midpoint.y);
  require(std::abs(bottom.centerline.back().y - last_measured_y) < 0.01f,
          "reacquisition must end at a measured row rather than extrapolate into white");
  const auto too_far = detector.process(coloredRoad(480, 720), 2);
  require(too_far.occlusion_detected && !too_far.valid,
          "road far beyond the vehicle cannot substitute for a hidden near road");
  // Two visually separated roads must not be joined merely because their
  // endpoints overlap across white; their measured directions disagree.
  cv::Mat opposed_hsv(720, 1280, CV_8UC3, cv::Scalar(30, 120, 130));
  for (int y = 0; y < 720; ++y) {
    const int center = y < 480 ? 640 + (y - 480) / 2 : 640 - (y - 540) / 2;
    if (y >= 480 && y < 540) continue;
    opposed_hsv.row(y).colRange(center - 140, center + 141).setTo(cv::Scalar(110, 100, 180));
  }
  opposed_hsv(cv::Rect(500, 480, 281, 60)).setTo(cv::Scalar(0, 0, 230));
  cv::Mat opposed;
  cv::cvtColor(opposed_hsv, opposed, cv::COLOR_HSV2BGR);
  require(cv::countNonZero(detector.process(opposed, 3).bridge_mask) == 0,
          "white must not connect roads with conflicting measured headings");

  // A white gap above the steering ROI must not become a closed forward exit.
  cv::Mat flare_hsv(720, 1280, CV_8UC3, cv::Scalar(30, 120, 130));
  for (int y = 0; y < 720; ++y) {
    const int half_width = 140 + (720 - y) / 2;
    flare_hsv.row(y).colRange(640 - half_width, 641 + half_width).setTo(cv::Scalar(110, 100, 180));
  }
  flare_hsv.rowRange(100, 160).setTo(cv::Scalar(0, 0, 230));
  cv::Mat flare;
  cv::cvtColor(flare_hsv, flare, cv::COLOR_HSV2BGR);
  const auto far_gap = detector.process(flare, 4);
  require(far_gap.occlusion_detected && far_gap.type != road_finder::RoadType::T_JUNCTION,
          "white outside the steering ROI must not fabricate a closed junction");
  const auto large_white = detector.process(coloredRoad(380, 680), 5);
  require(cv::imwrite("field_unobservable_process.png",
      road_finder::annotateResult(coloredRoad(380, 680), large_white, true)),
      "unobservable diagnostic export must succeed");
  require(cv::imwrite("field_occlusion_process.png",
              road_finder::annotateResult(coloredRoad(480, 540),
                  detector.process(coloredRoad(480, 540), 6), true)),
          "occlusion diagnostic export must succeed");
}
}

int main() {
  try {
    expectVotes(); expectSideThresholds(); expectOcclusion();
    std::cout << "field condition tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "field condition tests failed: " << error.what() << '\n';
    return 1;
  }
}
