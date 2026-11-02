#include "road_finder/road_detector.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

road_finder::Config testConfig() {
  road_finder::Config config;
  config.roi = cv::Rect(0, 0, config.input_width, config.input_height);
  return config;
}

cv::Mat stem() {
  cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(mask, {280, 40}, {360, 359}, cv::Scalar(255), cv::FILLED);
  return mask;
}

road_finder::RoadResult detect(const cv::Mat& mask) {
  road_finder::RoadDetector detector(testConfig());
  return detector.processMask(mask, 42);
}

void expectType(const cv::Mat& mask, road_finder::RoadType expected,
                unsigned directions) {
  const auto result = detect(mask);
  require(result.valid, "expected valid road geometry");
  require(result.type == expected,
          std::string("expected ") + road_finder::toString(expected) + ", got " +
              road_finder::toString(result.type));
  require(result.available_directions == directions, "wrong available directions");
  require(result.centerline.size() >= 2, "missing fitted centerline points");
}

road_finder::HsvRange rangeFor(cv::Scalar bgr) {
  cv::Mat pixel(1, 1, CV_8UC3, bgr);
  cv::Mat hsv;
  cv::cvtColor(pixel, hsv, cv::COLOR_BGR2HSV);
  const cv::Vec3b value = hsv.at<cv::Vec3b>(0, 0);
  const auto lower = [](int channel) { return std::max(0, channel - 3); };
  const auto upper = [](int channel) { return std::min(255, channel + 3); };
  return {lower(value[0]), std::min(179, upper(value[0])), lower(value[1]), upper(value[1]),
          lower(value[2]), upper(value[2])};
}

void expectSampleBandGeometry() {
  const cv::Mat straight = stem();
  const auto centered = detect(straight);
  require(centered.valid && centered.type == road_finder::RoadType::STRAIGHT,
          "centered stem should be a straight road");
  require(std::abs(centered.lateral_error) < 0.05f,
          "centered road should have near-zero normalized offset");
  require(std::abs(centered.heading_error_rad) < 0.01f,
          "vertical road should have near-zero heading");
  require(centered.centerline.front().y < centered.centerline.back().y,
          "centerline points should follow the scan bands from top to bottom");

  cv::Mat shifted = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(shifted, {325, 40}, {405, 359}, cv::Scalar(255), cv::FILLED);
  const auto shifted_result = detect(shifted);
  require(shifted_result.valid && shifted_result.lateral_error > 0.2f,
          "right-shifted road should have positive normalized offset");

  cv::Mat rightward_heading = cv::Mat::zeros(360, 640, CV_8UC1);
  const std::vector<cv::Point> path{{240, 359}, {360, 359}, {420, 40}, {300, 40}};
  cv::fillConvexPoly(rightward_heading, path, cv::Scalar(255));
  const auto angled = detect(rightward_heading);
  require(angled.valid && angled.heading_error_rad > 0.0f,
          "road that shifts right toward the top should have positive heading");

  cv::Mat missing_lower = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(missing_lower, {280, 40}, {360, 285}, cv::Scalar(255), cv::FILLED);
  require(!detect(missing_lower).valid,
          "missing lowest scan band should not produce a steering result");

  cv::Mat no_road = cv::Mat::zeros(360, 640, CV_8UC1);
  require(!detect(no_road).valid, "empty mask should be invalid");
}

void expectMarkerClassification() {
  cv::Mat left = stem();
  left.rowRange(0, 95).setTo(0);
  cv::rectangle(left, {80, 95}, {320, 215}, cv::Scalar(255), cv::FILLED);
  expectType(left, road_finder::RoadType::LEFT_TURN, road_finder::LEFT);

  cv::Mat right = stem();
  right.rowRange(0, 95).setTo(0);
  cv::rectangle(right, {320, 95}, {560, 215}, cv::Scalar(255), cv::FILLED);
  expectType(right, road_finder::RoadType::RIGHT_TURN, road_finder::RIGHT);

  cv::Mat t = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(t, {280, 165}, {360, 359}, cv::Scalar(255), cv::FILLED);
  cv::rectangle(t, {80, 95}, {560, 215}, cv::Scalar(255), cv::FILLED);
  expectType(t, road_finder::RoadType::T_JUNCTION,
             road_finder::LEFT | road_finder::RIGHT);
}

void expectHSVSegmentationAndStopGate() {
  const cv::Scalar background(185, 195, 200);
  const cv::Scalar road(150, 140, 130);
  const cv::Scalar stop(20, 40, 220);
  cv::Mat frame(720, 1280, CV_8UC3, background);
  cv::rectangle(frame, {500, 0}, {780, 719}, road, cv::FILLED);
  cv::rectangle(frame, {550, 400}, {650, 475}, stop, cv::FILLED);

  road_finder::Config config = testConfig();
  config.road_hsv = rangeFor(road);
  config.stop_hsv = rangeFor(stop);
  road_finder::RoadDetector default_detector(config);
  const auto default_result = default_detector.process(frame, 7);
  require(default_result.valid && default_result.type == road_finder::RoadType::STRAIGHT,
          "stop-zone recognition should be disabled by default");

  config.stop_detection_enabled = true;
  road_finder::RoadDetector stop_detector(config);
  const auto stop_result = stop_detector.process(frame, 8);
  require(stop_result.valid && stop_result.type == road_finder::RoadType::UNKNOWN,
          "configured stop-zone blob should override the road type to UNKNOWN");
}

void expectPerspectiveAndFrameValidation() {
  road_finder::Config config = testConfig();
  config.use_perspective = true;
  config.perspective_src = {{{0, 0}, {1279, 0}, {1279, 719}, {0, 719}}};
  config.perspective_dst = config.perspective_src;
  road_finder::RoadDetector detector(config);
  const auto result = detector.processMask(stem(), 9);
  require(result.valid && std::abs(result.lateral_error) < 0.05f,
          "identity perspective should preserve centered offset");

  cv::Mat wrong_size(90, 128, CV_8UC3);
  bool threw = false;
  try {
    (void)detector.process(wrong_size, 10);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "process should reject frames with the wrong configured size");
}

void expectDistractorsRejected() {
  cv::Mat mask = stem();
  cv::rectangle(mask, {0, 0}, {190, 359}, cv::Scalar(255), cv::FILLED);
  cv::rectangle(mask, {500, 0}, {639, 359}, cv::Scalar(255), cv::FILLED);
  const auto centered = detect(mask);
  require(centered.valid && std::abs(centered.lateral_error) < 0.05f,
          "larger border blobs must not replace the central road");
  require(centered.type == road_finder::RoadType::STRAIGHT,
          "disconnected side patches must not be mistaken for turn markers");

  cv::Mat clipped = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(clipped, {530, 0}, {639, 359}, cv::Scalar(255), cv::FILLED);
  const auto lost = detect(clipped);
  require(!lost.valid && lost.confidence == 0.0f && lost.centerline.empty(),
          "a road clipped by the image edge must not produce a confident centerline");

  cv::Mat wire = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(wire, {310, 0}, {318, 359}, cv::Scalar(255), cv::FILLED);
  require(!detect(wire).valid, "thin dark cables must not qualify as the road");

  cv::Mat disconnected = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(disconnected, {280, 300}, {360, 359}, cv::Scalar(255), cv::FILLED);
  cv::rectangle(disconnected, {290, 168}, {370, 290}, cv::Scalar(255), cv::FILLED);
  require(!detect(disconnected).valid,
          "disconnected upper patches must not supply the missing road geometry");

  cv::Mat crooked = cv::Mat::zeros(360, 640, CV_8UC1);
  const std::vector<cv::Point> shape{{270, 150}, {300, 210}, {270, 270}, {300, 359},
                                    {380, 359}, {350, 270}, {380, 210}, {350, 150}};
  cv::fillPoly(crooked, std::vector<std::vector<cv::Point>>{shape}, cv::Scalar(255));
  const auto uncertain = detect(crooked);
  require(uncertain.valid && uncertain.confidence < 0.85f,
          "inconsistent centers must reduce confidence even when all bands are present");
}

void expectAdaptiveDarkRoad() {
  road_finder::Config config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  road_finder::RoadDetector detector(config);
  for (double exposure : {0.5, 0.75, 1.0}) {
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(220, 225, 230));
    const std::vector<cv::Point> road{{520, 0}, {760, 0}, {860, 719}, {420, 719}};
    cv::fillConvexPoly(frame, road, cv::Scalar(165, 125, 135));
    cv::rectangle(frame, {0, 0}, {180, 719}, cv::Scalar(130, 85, 95), cv::FILLED);
    cv::rectangle(frame, {1140, 0}, {1279, 719}, cv::Scalar(130, 85, 95), cv::FILLED);
    cv::line(frame, {260, 0}, {210, 719}, cv::Scalar(20, 20, 20), 5);
    for (int y = 0; y < frame.rows; ++y) {
      auto* row = frame.ptr<cv::Vec3b>(y);
      const double gain = exposure * (0.65 + 0.35 * y / frame.rows);
      for (int x = 0; x < frame.cols; ++x)
        for (int channel = 0; channel < 3; ++channel)
          row[x][channel] = cv::saturate_cast<unsigned char>(row[x][channel] * gain);
    }
    const auto result = detector.process(frame, 100);
    require(result.valid && result.type == road_finder::RoadType::STRAIGHT,
            "adaptive mode should follow a tinted road despite exposure and side clutter");
    require(std::abs(result.lateral_error) < 0.05f &&
                std::abs(result.heading_error_rad) < 0.02f,
            "adaptive geometry should stay centered under changing brightness");
    require(result.left_boundary.back().x > 400 && result.left_boundary.back().x < 470 &&
                result.right_boundary.back().x > 810 && result.right_boundary.back().x < 880,
            "adaptive boundaries should match both actual road edges");
  }

  for (int brightness : {30, 140, 230}) {
    cv::Mat floor(720, 1280, CV_8UC3, cv::Scalar::all(brightness));
    require(!detector.process(floor, 101).valid,
            "a uniform floor must not become a road at any exposure");
  }

  config.use_perspective = true;
  config.perspective_src = {{{0, 0}, {1279, 0}, {1279, 719}, {0, 719}}};
  config.perspective_dst = {{{200, 0}, {1279, 0}, {1279, 719}, {200, 719}}};
  road_finder::RoadDetector warped(config);
  cv::Mat floor(720, 1280, CV_8UC3, cv::Scalar::all(220));
  require(!warped.process(floor, 102).valid,
          "black padding introduced by perspective warping must not be detected as road");

  config.perspective_dst = {{{200, 0}, {1000, 0}, {1279, 719}, {400, 719}}};
  road_finder::RoadDetector clipped_warp(config);
  cv::rectangle(floor, {0, 0}, {300, 719}, cv::Scalar(100, 100, 100), cv::FILLED);
  require(!clipped_warp.process(floor, 103).valid,
          "a perspective support edge must not substitute for an unseen road boundary");
}

cv::Mat wideJunction(int top, int bottom, int half_width) {
  cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(mask, {0, top}, {639, bottom}, cv::Scalar(255), cv::FILLED);
  cv::rectangle(mask, {320 - half_width, bottom}, {320 + half_width, 359},
                 cv::Scalar(255), cv::FILLED);
  return mask;
}

void expectWideJunctions() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  road_finder::RoadDetector detector(config);
  for (const auto& mask : {wideJunction(80, 245, 80), wideJunction(35, 315, 220),
                            wideJunction(30, 330, 290)}) {
    const auto result = detector.processMask(mask, 100);
    require(result.type == road_finder::RoadType::T_JUNCTION &&
                result.available_directions == (road_finder::LEFT | road_finder::RIGHT),
            "a full-width T crossbar must be recognized at different approach distances");
    require(result.classification_confidence > 0.7f,
            "junction topology should have independent classification confidence");
  }
  const auto close = detector.processMask(wideJunction(30, 330, 290), 101);
  require(!close.valid && close.centerline.empty() && close.confidence == 0.0f,
          "a recognized close junction must not invent unmeasurable steering geometry");
  cv::Mat tilted = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::fillConvexPoly(tilted, std::vector<cv::Point>{{0, 60}, {639, 100}, {639, 230}, {0, 190}},
                      cv::Scalar(255));
  cv::rectangle(tilted, {280, 190}, {360, 359}, cv::Scalar(255), cv::FILLED);
  require(detector.processMask(tilted, 101).type == road_finder::RoadType::T_JUNCTION,
          "a slanted crossbar must not become a one-sided turn as its top edge slopes");

  cv::Mat cross = wideJunction(100, 230, 80);
  cv::rectangle(cross, {240, 0}, {400, 100}, cv::Scalar(255), cv::FILLED);
  require(detector.processMask(cross, 102).type != road_finder::RoadType::T_JUNCTION,
          "a visible forward exit must not be classified as a T junction");
  cv::Mat unbounded(360, 640, CV_8UC1, cv::Scalar(255));
  require(detector.processMask(unbounded, 103).type == road_finder::RoadType::UNKNOWN,
          "uniform foreground without a stem and closed forward path is not a T");

  for (bool left : {true, false}) {
    cv::Mat turn = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(turn, {280, 230}, {360, 359}, cv::Scalar(255), cv::FILLED);
    cv::rectangle(turn, {left ? 0 : 280, 100}, {left ? 360 : 639, 230},
                   cv::Scalar(255), cv::FILLED);
    require(detector.processMask(turn, 104).type ==
                (left ? road_finder::RoadType::LEFT_TURN : road_finder::RoadType::RIGHT_TURN),
            "a single lateral exit must not become a T junction");
  }

  const cv::Mat topology = wideJunction(30, 315, 220);
  for (double exposure : {0.5, 0.8, 1.0}) {
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(220, 225, 230));
    cv::Mat full_mask;
    cv::resize(topology, full_mask, frame.size(), 0, 0, cv::INTER_NEAREST);
    frame.setTo(cv::Scalar(165, 125, 135), full_mask);
    frame.convertTo(frame, -1, exposure);
    cv::Mat noise(frame.size(), CV_16SC3);
    cv::RNG random(42);
    random.fill(noise, cv::RNG::NORMAL, 0, 5);
    cv::Mat noisy;
    frame.convertTo(noisy, CV_16SC3);
    cv::add(noisy, noise, noisy);
    noisy.convertTo(frame, CV_8UC3);
    const auto result = detector.process(frame, 105);
    require(result.type == road_finder::RoadType::T_JUNCTION,
            "row-uniform crossbars should survive exposure changes and pixel noise");
  }
}

void expectForwardContinuationNotTurn() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  road_finder::RoadDetector detector(config);
  for (bool mirror : {false, true}) {
    for (int shape : {0, 1, 2}) {
      cv::Mat road = cv::Mat::zeros(360, 640, CV_8UC1);
      // A continuous lane can be off axis or flare toward the horizon.
      // Neither changes its topology into a 90-degree lateral exit.
      const int top_left = shape == 2 ? 497 : (shape == 1 ? 390 : 360);
      const int top_right = shape == 2 ? 503 : (shape == 1 ? 610 : 440);
      cv::fillConvexPoly(road, std::vector<cv::Point>{
          {280, 359}, {360, 359}, {top_right, 0}, {top_left, 0}}, cv::Scalar(255));
      if (mirror) cv::flip(road, road, 1);
      const auto result = detector.processMask(road, 100);
      require(result.valid && result.type == road_finder::RoadType::STRAIGHT &&
                  result.available_directions == road_finder::STRAIGHT,
              "an off-axis forward lane reaching the image top must not become a turn");
    }
  }
}

void expectVeryCloseTJunctionEvidence() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  road_finder::RoadDetector detector(config);
  const cv::Mat very_close = wideJunction(35, 350, 220);
  const auto near_result = detector.processMask(very_close, 100);
  require(near_result.type == road_finder::RoadType::T_JUNCTION,
          "a T with entrance edges visible only in the last image rows should be classified");
  require(!near_result.valid && near_result.centerline.empty(),
          "a short visible entrance must not create unmeasured steering geometry");

  for (bool mirror : {false, true}) {
    cv::Mat fringe = wideJunction(35, 245, 80);
    cv::rectangle(fringe, {4, 0}, {9, 35}, cv::Scalar(255), cv::FILLED);
    if (mirror) cv::flip(fringe, fringe, 1);
    require(detector.processMask(fringe, 101).type == road_finder::RoadType::T_JUNCTION,
            "a thin connected edge artifact above a T must not count as a forward road");

    cv::Mat close_turn = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(close_turn, {120, 350}, {520, 359}, cv::Scalar(255), cv::FILLED);
    cv::rectangle(close_turn, {mirror ? 0 : 120, 35}, {mirror ? 520 : 639, 350},
                   cv::Scalar(255), cv::FILLED);
    require(detector.processMask(close_turn, 102).type ==
                (mirror ? road_finder::RoadType::LEFT_TURN
                        : road_finder::RoadType::RIGHT_TURN),
            "a short entrance with only one arm must remain a one-sided turn");
  }

  cv::Mat no_entrance(360, 640, CV_8UC1, cv::Scalar(255));
  cv::rectangle(no_entrance, {0, 0}, {639, 34}, cv::Scalar(0), cv::FILLED);
  require(detector.processMask(no_entrance, 103).type == road_finder::RoadType::UNKNOWN,
          "a full-width patch without measured entrance edges must remain unknown");
  cv::Mat clipped = wideJunction(35, 350, 220);
  cv::rectangle(clipped, {0, 350}, {520, 359}, cv::Scalar(255), cv::FILLED);
  require(detector.processMask(clipped, 104).type == road_finder::RoadType::UNKNOWN,
          "one missing entrance boundary must not be invented for a close T");

  cv::Mat cross = wideJunction(100, 230, 80);
  cv::fillConvexPoly(cross, std::vector<cv::Point>{
      {240, 100}, {400, 100}, {323, 0}, {317, 0}}, cv::Scalar(255));
  require(detector.processMask(cross, 105).type != road_finder::RoadType::T_JUNCTION,
          "a tapering real forward exit must not be removed as an edge artifact");

  config.process_width = 320;
  road_finder::RoadDetector smaller(config);
  cv::Mat fringe = wideJunction(35, 245, 80);
  cv::rectangle(fringe, {4, 0}, {9, 35}, cv::Scalar(255), cv::FILLED);
  for (const cv::Mat& image : {very_close, fringe}) {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(config.process_width, config.processHeight()),
                0, 0, cv::INTER_NEAREST);
    require(smaller.processMask(resized, 106).type == road_finder::RoadType::T_JUNCTION,
            "close-entry and fringe handling must also work at 320 processing width");
  }
}

void expectAsymmetricCloseJunctions() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  road_finder::RoadDetector detector(config);
  for (bool mirror : {false, true}) {
    const int left = mirror ? 69 : 10;
    const int right = mirror ? 629 : 570;
    cv::Mat t = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(t, {left, 240}, {right, 359}, cv::Scalar(255), cv::FILLED);
    cv::rectangle(t, {0, 80}, {639, 240}, cv::Scalar(255), cv::FILLED);
    require(detector.processMask(t, 100).type == road_finder::RoadType::T_JUNCTION,
            "a close T must retain both exits when one stem edge is near the image edge");

    cv::Mat turn = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(turn, {left, 240}, {right, 359}, cv::Scalar(255), cv::FILLED);
    cv::rectangle(turn, {mirror ? 0 : left, 80}, {mirror ? right : 639, 240},
                   cv::Scalar(255), cv::FILLED);
    const auto expected = mirror ? road_finder::RoadType::LEFT_TURN
                                 : road_finder::RoadType::RIGHT_TURN;
    require(detector.processMask(turn, 101).type == expected,
            "a close one-sided turn must not acquire an unsupported second exit");
    // A short patch at the clipped side is not a sustained lateral exit.
    cv::rectangle(turn, {0, 200}, {639, 202}, cv::Scalar(255), cv::FILLED);
    require(detector.processMask(turn, 102).type == expected,
            "brief edge noise must not promote a one-sided turn to T");
  }
}

void expectClippedFarEdgeAtTJunction() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  for (int width : {320, 640}) {
    config.process_width = width;
    road_finder::RoadDetector detector(config);
    const int height = config.processHeight();
    cv::Mat t = wideJunction(30, 245, 80);
    // A tilted far edge can leave road in both outer corners while a broad
    // floor cap blocks the center. These strips are too narrow to be a lane.
    cv::rectangle(t, {0, 0}, {20, 30}, cv::Scalar(255), cv::FILLED);
    cv::rectangle(t, {619, 0}, {639, 30}, cv::Scalar(255), cv::FILLED);
    cv::Mat scaled;
    cv::resize(t, scaled, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
    require(detector.processMask(scaled, 100).type == road_finder::RoadType::T_JUNCTION,
            "thin outer-corner road remnants must not erase a closed T far edge");

    for (bool mirror : {false, true}) {
      cv::Mat open = wideJunction(100, 245, 80);
      // An actual forward exit at the image edge retains measurable width
      // inside the edge margin, even when its outer boundary is clipped.
      cv::rectangle(open, {580, 0}, {639, 100}, cv::Scalar(255), cv::FILLED);
      if (mirror) cv::flip(open, open, 1);
      cv::resize(open, scaled, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
      require(detector.processMask(scaled, 101).type != road_finder::RoadType::T_JUNCTION,
              "a measurable off-axis forward exit must keep the T far edge open");
    }
  }
}

void expectJunctionColorUnderUnevenLight() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  road_finder::RoadDetector detector(config);
  cv::Mat mask;
  cv::resize(wideJunction(80, 240, 80), mask, cv::Size(1280, 720),
              0, 0, cv::INTER_NEAREST);
  // Exercise two chromatic directions: the model must learn the frame's
  // road/floor contrast instead of hard-coding the captured road's blue hue.
  for (bool red_road : {false, true}) {
    const cv::Scalar road = red_road ? cv::Scalar(125, 125, 165)
                                    : cv::Scalar(165, 125, 135);
    const cv::Scalar floor = red_road ? cv::Scalar(205, 205, 180)
                                     : cv::Scalar(180, 205, 205);
    cv::Mat frame(mask.size(), CV_8UC3, floor);
    frame.setTo(road, mask);
    for (int y = 0; y < frame.rows; ++y) {
      auto* row = frame.ptr<cv::Vec3b>(y);
      const auto* road_pixels = mask.ptr<unsigned char>(y);
      for (int x = 0; x < frame.cols; ++x) {
        const double gain = 1.2 - 0.6 * x / frame.cols;
        const int reflection = road_pixels[x] && x < frame.cols / 2 ? 50 : 0;
        for (int channel = 0; channel < 3; ++channel)
          row[x][channel] = cv::saturate_cast<unsigned char>(
              row[x][channel] * gain + reflection);
      }
    }
    const auto result = detector.process(frame, 150);
    require(result.type == road_finder::RoadType::T_JUNCTION,
            "road reflections and darker floor must not erase one exit");
  }
}

void expectCapturedCloseTJunction(const char* path) {
  const cv::Mat captured = cv::imread(path, cv::IMREAD_COLOR);
  require(!captured.empty(), std::string("captured close T image is missing: ") + path);
  cv::Mat mirrored, resized;
  cv::flip(captured, mirrored, 1);
  cv::resize(captured, resized, cv::Size(1280, 720));
  for (const cv::Mat& image : {captured, mirrored, resized}) {
    for (int process_width : {320, 640}) {
      auto config = testConfig();
      config.input_width = image.cols;
      config.input_height = image.rows;
      config.process_width = process_width;
      config.roi = cv::Rect(0, image.rows / 2, image.cols,
                            image.rows - image.rows / 2);
      config.segmentation_mode = "adaptive_dark";
      road_finder::RoadDetector detector(config);
      const auto result = detector.process(image, 200);
      require(result.type == road_finder::RoadType::T_JUNCTION &&
                  result.available_directions == (road_finder::LEFT | road_finder::RIGHT),
              "captured T must retain both exits across mirroring and processing sizes");

      // The supplied video preset publishes on the first measured T frame.
      // A previously accepted straight must not mask that junction label.
      config.classification_confirm_frames = 1;
      config.classification_hold_frames = 2;
      road_finder::RoadDetector video(config);
      cv::Mat incoming = cv::Mat::zeros(config.processHeight(), process_width, CV_8UC1);
      cv::rectangle(incoming, {process_width * 2 / 5, 0},
                    {process_width * 3 / 5, incoming.rows - 1}, cv::Scalar(255), cv::FILLED);
      require(video.processMask(incoming, 1000, true).type == road_finder::RoadType::STRAIGHT,
              "video fixture must first accept the incoming straight");
      const auto first_t = video.process(image, 1067, true);
      require(first_t.raw_type == road_finder::RoadType::T_JUNCTION &&
                  first_t.type == road_finder::RoadType::T_JUNCTION,
              "single-frame preset must publish the first captured T after straight");
    }
  }
}

void expectTemporalStability() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  config.classification_confirm_frames = 3;
  config.classification_hold_frames = 2;
  config.geometry_smoothing_alpha = 0.35f;
  road_finder::RoadDetector detector(config);
  std::int64_t time = 1000;
  const auto observe = [&](const cv::Mat& mask) {
    time += 67;
    return detector.processMask(mask, time, true);
  };
  require(observe(stem()).type == road_finder::RoadType::UNKNOWN,
          "a new video classification must wait for confirmation");
  observe(stem());
  require(observe(stem()).type == road_finder::RoadType::STRAIGHT,
          "three consistent observations should confirm straight");
  const auto near_t = wideJunction(30, 330, 290);
  const auto transient = observe(near_t);
  require(transient.raw_type == road_finder::RoadType::T_JUNCTION &&
              transient.type == road_finder::RoadType::STRAIGHT && !transient.valid,
          "one noisy T observation must not change the stable type or revive old geometry");
  observe(stem());
  observe(near_t);
  observe(near_t);
  require(observe(near_t).type == road_finder::RoadType::T_JUNCTION,
          "a sustained junction must be confirmed after three video frames");
  cv::Mat blank = cv::Mat::zeros(360, 640, CV_8UC1);
  const auto gap = observe(blank);
  require(gap.type == road_finder::RoadType::UNKNOWN && !gap.valid && gap.centerline.empty(),
          "a junction without current two-box votes must clear immediately");
  observe(blank);
  require(observe(blank).type == road_finder::RoadType::UNKNOWN,
          "held classifications must expire after the configured dropout limit");
  observe(near_t);
  observe(near_t);
  observe(near_t);
  time += 1000;
  require(observe(stem()).type == road_finder::RoadType::UNKNOWN,
          "a discontinuous stream must discard its previous junction state");
  require(detector.processMask(near_t, 20, true).type == road_finder::RoadType::UNKNOWN,
          "a backwards timestamp must reset temporal state");
  require(detector.processMask(near_t, 21).type == road_finder::RoadType::T_JUNCTION,
          "still-image classification must not require video confirmation");

  double raw_motion = 0.0, filtered_motion = 0.0;
  road_finder::RoadDetector smoother(config);
  road_finder::RoadResult previous_raw, previous_filtered;
  for (int i = 0; i < 12; ++i) {
    cv::Mat jitter = cv::Mat::zeros(360, 640, CV_8UC1);
    const int shift = i % 2 ? 12 : -12;
    cv::rectangle(jitter, {280 + shift, 0}, {360 + shift, 359}, cv::Scalar(255), cv::FILLED);
    const auto raw = smoother.processMask(jitter, 2000 + i * 67);
    const auto filtered = smoother.processMask(jitter, 2000 + i * 67, true);
    require(raw.valid && filtered.valid, "small jitter should retain measured geometry");
    if (i > 1) {
      raw_motion += std::abs(raw.centerline.back().x - previous_raw.centerline.back().x);
      filtered_motion += std::abs(filtered.centerline.back().x - previous_filtered.centerline.back().x);
    }
    previous_raw = raw;
    previous_filtered = filtered;
  }
  require(filtered_motion < raw_motion * 0.6,
          "temporal geometry filtering should suppress small alternating position noise");
  const auto lost = smoother.processMask(blank, 3000, true);
  require(!lost.valid && lost.centerline.empty(), "smoothing must not hide lost geometry");
}

void expectStopDuringJunctionDropout() {
  auto config = testConfig();
  config.segmentation_mode = "adaptive_dark";
  config.classification_confirm_frames = 3;
  config.classification_hold_frames = 2;
  config.stop_detection_enabled = true;
  const cv::Scalar stop(20, 40, 220);
  config.stop_hsv = rangeFor(stop);
  road_finder::RoadDetector detector(config);
  cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(220, 225, 230));
  cv::Mat mask;
  cv::resize(wideJunction(30, 330, 290), mask, frame.size(), 0, 0, cv::INTER_NEAREST);
  frame.setTo(cv::Scalar(165, 125, 135), mask);
  detector.process(frame, 1000, true);
  detector.process(frame, 1067, true);
  require(detector.process(frame, 1134, true).type == road_finder::RoadType::T_JUNCTION,
          "junction should be confirmed before testing the stop override");
  cv::rectangle(frame, {500, 400}, {779, 455}, stop, cv::FILLED);
  const auto stopped = detector.process(frame, 1201, true);
  require(stopped.stop_detected && stopped.type == road_finder::RoadType::UNKNOWN &&
              stopped.available_directions == 0,
          "stop evidence must immediately clear a held junction even without steering geometry");
}

}  // namespace

int main() {
  try {
    expectSampleBandGeometry();
    expectMarkerClassification();
    expectHSVSegmentationAndStopGate();
    expectPerspectiveAndFrameValidation();
    expectDistractorsRejected();
    expectAdaptiveDarkRoad();
    expectWideJunctions();
    expectForwardContinuationNotTurn();
    expectVeryCloseTJunctionEvidence();
    expectAsymmetricCloseJunctions();
    expectClippedFarEdgeAtTJunction();
    expectJunctionColorUnderUnevenLight();
    expectCapturedCloseTJunction(ROAD_FUCKER_CLOSE_T_IMAGE);
    expectCapturedCloseTJunction(ROAD_FUCKER_NEAR_T_IMAGE);
    expectCapturedCloseTJunction(ROAD_FUCKER_APPROACH_T_IMAGE);
    expectCapturedCloseTJunction(ROAD_FUCKER_LIVE_T_IMAGE);
    expectTemporalStability();
    expectStopDuringJunctionDropout();
    std::cout << "road detector tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "road detector tests failed: " << error.what() << '\n';
    return 1;
  }
}
