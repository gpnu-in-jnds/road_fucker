#include "road_finder/road_detector.hpp"
#include "road_finder/preview.hpp"
#include "../src/worker_pool.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <atomic>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void expectPool() {
  for (int threads : {1, 2, 4}) {
    road_finder::WorkerPool pool(threads);
    std::vector<int> sequential(137), parallel(137);
    for (int i = 0; i < 137; ++i) sequential[i] = i * i + 3;
    for (int frame = 0; frame < 5; ++frame) {
      pool.run(137, [&](int i) { parallel[i] = i * i + 3; });
      require(sequential == parallel, "pool differs from serial reference");
    }
    bool threw = false;
    try {
      pool.run(100, [](int i) { if (i == 7) throw std::runtime_error("task failure"); });
    } catch (const std::runtime_error&) { threw = true; }
    require(threw, "worker exception must propagate");
    std::atomic<int> completed{0};
    pool.run(100, [&](int) { ++completed; });
    require(completed == 100, "pool must remain usable after failure");
  }
}

void expectGeometry() {
  for (int threads : {1, 2, 4}) {
    road_finder::Config config;
    config.threads = threads;
    config.roi = {0, 362, 1280, 358};  // 179 processing rows; uneven partitions.
    road_finder::RoadDetector detector(config);
    cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(mask, {280, 0}, {360, 359}, cv::Scalar(255), -1);
    const auto first = detector.processMask(mask, 1);
    require(first.valid && first.type == road_finder::RoadType::STRAIGHT, "straight must be valid");
    require(static_cast<int>(first.scan_blocks.size()) == threads * 6, "wrong strip count");
    int y = detector.processRoi().y;
    for (const auto& block : first.scan_blocks) {
      require(block.processed_region.y == y && block.processed_region.height > 0,
              "strips must cover ROI without gaps or overlaps");
      y += block.processed_region.height;
      require(block.found && std::abs(block.midpoint.x - 640.0f) < 0.01f,
              "every strip must measure the analytic road center");
    }
    require(y == 360, "ROI coverage must include bottom row");
    for (int repeat = 0; repeat < 6; ++repeat) {
      const auto result = detector.processMask(mask, repeat + 2);
      require(result.centerline == first.centerline && result.type == first.type &&
                  result.confidence == first.confidence, "parallel detection must be deterministic");
    }
    // A linear road has an analytically known heading, at every partition count.
    cv::Mat slanted = cv::Mat::zeros(mask.size(), mask.type());
    for (int row = 0; row < mask.rows; ++row) {
      const int x = 280 + (360 - row) / 6;
      slanted.row(row).colRange(x, x + 81).setTo(255);
    }
    const auto angled = detector.processMask(slanted, 20);
    require(angled.valid && std::abs(angled.heading_error_rad - std::atan(1.0 / 6)) < 0.02,
            "parallel fit must preserve analytic heading");
    mask.setTo(0);
    const auto empty = detector.processMask(mask, 30);
    require(!empty.valid && empty.centerline.empty(), "empty frame must clear geometry");
    for (const auto& block : empty.junction_blocks)
      require(!block.found && block.polygon.size() == 4, "lost frame must retain red box locations");
  }
}

void expectMarkers() {
  for (int index = 0; index < 6; ++index) {
    road_finder::Config config;
    config.threads = 4;
    config.roi = {0, 360, 1280, 360};
    road_finder::RoadDetector detector(config);
    cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(mask, {280, 0}, {360, 359}, cv::Scalar(255), -1);
    const cv::Rect reference = index < 3 ? config.left_marker_rois[index]
                                         : config.right_marker_rois[index - 3];
    const cv::Rect region(reference.x * 5, reference.y * 4, reference.width * 5,
                          reference.height * 4);
    cv::rectangle(mask, region, cv::Scalar(255), -1);
    const int y = region.y + region.height / 2;
    cv::rectangle(mask, index < 3 ? cv::Rect(region.x, y, 321 - region.x, 10)
                                 : cv::Rect(320, y, region.x + region.width - 320, 10),
                  cv::Scalar(255), -1);
    const auto result = detector.processMask(mask, 1);
    for (int i = 0; i < 6; ++i)
      require(result.junction_blocks[i].found == (i == index), "side boxes must detect independently");
    require(result.type != road_finder::RoadType::LEFT_TURN &&
            result.type != road_finder::RoadType::RIGHT_TURN &&
            result.type != road_finder::RoadType::T_JUNCTION,
            "one box must never publish a junction");
    // Disconnect the lateral patch: a foreground blob alone is insufficient.
    mask.colRange(index < 3 ? 250 : 380, index < 3 ? 270 : 400).setTo(0);
    const auto disconnected = detector.processMask(mask, 2);
    require(!disconnected.junction_blocks[index].found, "disconnected side patch must be rejected");
  }
}

void expectHsvWrap() {
  road_finder::Config config;
  config.roi = {0, 360, 1280, 360};
  config.road_hsv = {170, 10, 180, 255, 120, 255};
  config.morphology_pixels = 1;
  road_finder::RoadDetector detector(config);
  for (int hue : {0, 5, 175, 179}) {
    cv::Mat hsv(720, 1280, CV_8UC3, cv::Scalar(60, 255, 200));
    cv::rectangle(hsv, {500, 0}, {780, 719}, cv::Scalar(hue, 255, 200), -1);
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    const auto result = detector.process(bgr, hue + 1);
    require(result.valid, "wrapped hue interval must retain red on both sides of zero");
    require(result.road_mask.at<unsigned char>(300, 320) != 0 &&
            result.road_mask.at<unsigned char>(300, 20) == 0, "HSV must exclude green floor");
  }
}

void expectFixtureHsv() {
  for (int threads : {1, 2, 4}) {
    for (const char* name : {"t_junction_close.png", "t_junction_near_raw.png",
                             "t_junction_approach_raw.png", "t_junction_live_720p.png"}) {
      cv::Mat frame = cv::imread(std::string(ROAD_FIXTURE_DIR) + "/" + name);
      require(!frame.empty(), "fixture must be materialized from Git LFS");
      road_finder::Config config;
      config.input_width = frame.cols;
      config.input_height = frame.rows;
      config.roi = {0, frame.rows / 2, frame.cols, frame.rows - frame.rows / 2};
      config.threads = threads;
      road_finder::RoadDetector detector(config);
      for (bool mirrored : {false, true}) {
        cv::Mat input;
        if (mirrored) cv::flip(frame, input, 1); else input = frame;
        const auto result = detector.process(input, 1);
        require(result.type == road_finder::RoadType::T_JUNCTION,
                "HSV defaults must recognize original and mirrored junction fixtures");
        bool hit = false;
        for (const auto& block : result.junction_blocks) hit = hit || block.found;
        require(hit, "junction frames, even without geometry, must expose local box evidence");
      }
    }
  }
}

void expectPerspectiveBlocks() {
  road_finder::Config config;
  config.roi = {0, 360, 1280, 360};
  config.use_perspective = true;
  config.perspective_src = {{{0, 0}, {1279, 0}, {1279, 719}, {0, 719}}};
  config.perspective_dst = config.perspective_src;
  for (auto& point : config.perspective_dst) point.x += 40;
  road_finder::RoadDetector detector(config);
  const auto result = detector.processMask(cv::Mat::zeros(360, 640, CV_8UC1), 1);
  require(std::abs(result.junction_blocks[0].polygon[0].x - 60.0f) < 0.01f,
          "junction rectangles must inverse-transform to input coordinates");
  require(std::abs(result.scan_blocks[0].polygon[0].x) < 0.01f,
          "scan rectangles must inverse-transform to input coordinates");
}

void expectRendering() {
  cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar::all(0));
  road_finder::RoadResult result;
  for (int i = 0; i < 6; ++i) {
    auto& block = result.junction_blocks[i];
    const float x = 60.0f + i * 150;
    block.polygon = {{x, 300}, {x + 70, 300}, {x + 70, 370}, {x, 370}};
    block.found = i % 2 == 0;
  }
  result.scan_blocks.resize(1);
  result.scan_blocks[0].found = true;
  result.scan_blocks[0].midpoint = {640, 500};
  const auto view = road_finder::annotateResult(frame, result);
  for (int i = 0; i < 6; ++i) {
    const auto pixel = view.at<cv::Vec3b>(300, 95 + i * 150);
    require(i % 2 == 0 ? pixel[1] > 200 && pixel[2] < 10
                       : pixel[2] > 200 && pixel[1] < 10, "box must use its own red/green state");
  }
  require(view.at<cv::Vec3b>(500, 640) == cv::Vec3b(255, 0, 255),
          "measured points must appear even without valid overall geometry");
  const auto montage = road_finder::annotateResult(frame, result, true);
  require(montage.cols == 2560 && montage.rows == 1440, "process montage must have four panels");
  require(montage.at<cv::Vec3b>(720 + 300, 1280 + 95)[1] > 200,
          "final process panel must contain junction boxes");
}
}

int main() {
  try {
    expectPool(); expectGeometry(); expectMarkers(); expectHsvWrap();
    expectFixtureHsv(); expectPerspectiveBlocks(); expectRendering();
    std::cout << "parallel detection tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
