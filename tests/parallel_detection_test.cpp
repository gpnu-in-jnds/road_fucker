#include "test_support.hpp"
#include "road_finder/preview.hpp"

#include <opencv2/imgcodecs.hpp>

#include <filesystem>
#include <iostream>

namespace {
using namespace road_finder;
using namespace road_finder::test;

void parallelAndScale() {
  for (int width : {160, 320, 640}) {
    RoadResult baseline;
    for (int threads : {1, 2, 4}) {
      auto value = config();
      value.process_width = width;
      value.threads = threads;
      RoadDetector detector(value);
      cv::Mat mask;
      cv::resize(road(), mask, {width, value.processHeight()}, 0, 0, cv::INTER_NEAREST);
      const auto result = detector.processMask(mask, 100);
      require(result.valid && result.type == RoadType::Straight, "parallel geometry must be valid");
      require(static_cast<int>(result.scan_blocks.size()) == threads * 6, "wrong strip count");
      int y = detector.processRoi().y;
      for (const auto& block : result.scan_blocks) {
        require(block.processed_region.y == y && block.processed_region.height > 0,
                "strips must cover the ROI without gaps or overlaps");
        require(block.found && std::abs(block.midpoint.x - 320) < 0.01,
                "each block must measure the analytic source center");
        y += block.processed_region.height;
      }
      require(y == detector.processRoi().br().y, "last strip must end at ROI bottom");
      if (threads == 1) baseline = result;
      require(std::abs(result.heading_error_rad - baseline.heading_error_rad) < 1e-6 &&
              std::abs(result.lateral_error - baseline.lateral_error) < 1e-6,
              "worker count must preserve analytic steering");
      cv::resize(road(300), mask, {width, value.processHeight()}, 0, 0, cv::INTER_NEAREST);
      const auto end = detector.processMask(mask, 200);
      require(end.type == RoadType::RoadEnd && end.road_end_region.polygon.front() == cv::Point2f(280,240),
              "end trigger is independent of strip count and uses source pixels");
    }
  }
}

void perspective() {
  auto value = config();
  value.use_perspective = true;
  value.perspective_src = {{{0,0}, {639,0}, {639,359}, {0,359}}};
  value.perspective_dst = {{{100,0}, {739,0}, {739,359}, {100,359}}};
  RoadDetector detector(value);
  const auto result = detector.process(image(road(300)), 100);
  require(result.type == RoadType::RoadEnd, "translated perspective must preserve end evidence");
  require(result.road_end_region.polygon.front() == cv::Point2f(280,240),
          "trigger polygon must remain in original coordinates");
  require(result.road_end_region.coverage == 1.0f && result.road_end_boundary.empty(),
          "end result must report non-mask area without fabricated boundary points");
  const auto centered = detector.process(image(road()), 100);
  require(centered.valid && std::abs(centered.centerline.back().x - 320) < 1,
          "source centerline must survive perspective translation");
  const cv::Mat overlay = annotateResult(image(road()), centered);
  require(overlay.at<cv::Vec3b>(310, 310)[2] > 80 &&
          overlay.at<cv::Vec3b>(310, 310)[0] < 150 &&
          overlay.at<cv::Vec3b>(310, 410) == cv::Vec3b(160,160,160),
          "detector must provide the source transform for the red mask overlay");
  // Padding pixels supplied by a custom segmenter cannot become measured road.
  cv::Mat padding(360, 640, CV_8UC1, cv::Scalar(255));
  const auto masked_padding = detector.processMask(padding, 100);
  require(masked_padding.valid && std::abs(masked_padding.centerline.back().x - 269.5f) < 1.0f,
          "only valid-view pixels may contribute to the mask centroid");
  // A sheared trigger is a parallelogram, not its axis-aligned bounding box.
  value.perspective_dst = {{{0,-32}, {639,31.9f}, {639,390.9f}, {0,327}}};
  RoadDetector sheared(value);
  require(sheared.process(image(road(300)), 100).type == RoadType::RoadEnd,
          "a sheared view must detect a boundary inside the source trigger");
  require(!sheared.process(image(road()), 100).road_end_detected,
          "only the transformed rectangle polygon supplies area evidence");
  // A box outside the visible warped image must not treat padding as non-road.
  value.perspective_dst = {{{100,0}, {739,0}, {739,359}, {100,359}}};
  value.road_end_roi = {600, 240, 40, 50};
  RoadDetector invisible_box(value);
  const auto invisible = invisible_box.process(image(road()), 100);
  require(invisible.valid && !invisible.road_end_detected &&
          invisible.road_end_region.coverage == 0.0f,
          "an end box with no visible pixels supplies no end evidence");
  // Partially clipped boxes use visible source pixels; padding is never non-mask.
  value.road_end_roi = {500, 240, 140, 50};
  RoadDetector partial_box(value);
  cv::Mat visible_road(360, 640, CV_8UC1, cv::Scalar(255));
  const auto partial = partial_box.processMask(visible_road, 100);
  require(partial.valid && !partial.road_end_detected &&
          partial.road_end_region.coverage == 0.0f,
          "invalid padding must not increase a partially visible box's non-mask fraction");

}

void capturedRoadEnd() {
  const auto base = std::filesystem::path(ROAD_FIXTURE_DIR);
  const auto value = loadConfig((base / "roadend.yaml").string());
  RoadDetector detector(value);
  for (const char* name : {"roadend.png", "t-conner.png"}) {
    const auto frame = cv::imread((base / name).string());
    require(!frame.empty() && frame.size() == cv::Size(640, 480),
            "captured end regression must use the original image aspect ratio");
    const auto result = detector.process(frame, 1, true);
    require(result.type == (std::string(name) == "roadend.png" ?
                           RoadType::RoadEnd : RoadType::Straight),
            "captured best turn pose must trigger immediately; distant pose must not");
    const double expected = std::string(name) == "roadend.png" ? 0.826318 : 0.000683594;
    require(std::abs(result.road_end_region.coverage - expected) < 0.001,
            "captured end area must match the calibrated sample box");
    cv::imwrite(std::string("area_") + name, annotateResult(frame, result));
  }
}

void fixtures() {
  const auto value = Config{};
  for (const char* name : {"frame_784.png", "t_junction_close.png", "t_junction_near_raw.png",
                           "t_junction_approach_raw.png", "t_junction_live_720p.png"}) {
    const cv::Mat frame = cv::imread((std::filesystem::path(ROAD_FIXTURE_DIR) / name).string());
    require(!frame.empty(), "fixture must be materialized from Git LFS: " + std::string(name));
    cv::Mat resized;
    cv::resize(frame, resized, {value.input_width, value.input_height});
    const auto result = RoadDetector(value).process(resized, 100);
    require(result.type == RoadType::Straight || result.type == RoadType::RoadEnd ||
            result.type == RoadType::Unknown, "legacy scene must use the new three-state contract");
    require(std::isfinite(result.lateral_error) && std::isfinite(result.heading_error_rad),
            "camera fixture must not produce non-finite steering");
    require(!result.valid || !result.centerline.empty(), "valid geometry requires a centerline");
    cv::imwrite(std::string("fixture_") + name, annotateResult(resized, result));
  }
}
}  // namespace

int main() {
  try {
    parallelAndScale();
    perspective();
    fixtures();
    capturedRoadEnd();
    std::cout << "parallel detection tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "parallel detection tests failed: " << error.what() << '\n';
    return 1;
  }
}
