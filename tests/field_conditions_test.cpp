#include "test_support.hpp"
#include "road_finder/preview.hpp"

#include <opencv2/imgcodecs.hpp>

#include <iostream>

namespace {
using namespace road_finder;
using namespace road_finder::test;

cv::Scalar hsvColor(int hue, int saturation = 100, int value = 180) {
  const cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, saturation, value));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const auto pixel = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(pixel[0], pixel[1], pixel[2]);
}

cv::Mat mixedHueRoad(int top, int floor_hue, int width = 81) {
  cv::Mat frame = image(road(top, 320, width), hsvColor(floor_hue), hsvColor(0));
  const cv::Scalar high_hue = hsvColor(100);
  // Large real road patches must survive segmentation, not rely on morphology.
  for (int y = top; y < frame.rows; ++y) {
    const int left = (y / 60) % 2 == 0 ? 320 - width / 2 : 321;
    frame(cv::Rect(left, y, width / 2, 1)).setTo(high_hue);
  }
  return frame;
}

void crossZeroRoad() {
  RoadDetector detector(config());
  for (int hue : {0, 10, 70, 100, 179}) {
    const auto result = detector.process(image(road(), hsvColor(35), hsvColor(hue)), 100);
    require(result.valid && result.type == RoadType::Straight,
            "default range must track hue " + std::to_string(hue));
    require(result.road_mask.at<unsigned char>(250, 320) != 0,
            "both inclusive hue intervals must retain road pixels");
  }
  for (int hue : {11, 25, 35, 45, 69}) {
    const auto result = detector.process(image(road(), hsvColor(35), hsvColor(hue)), 100);
    require(!result.valid && result.type == RoadType::Unknown &&
            cv::countNonZero(result.road_mask) == 0,
            "default range must reject hue " + std::to_string(hue));
  }
  for (const auto& color : {hsvColor(0, 0), hsvColor(100, 200),
                            hsvColor(0, 100, 30), hsvColor(100, 100, 250)}) {
    const auto result = detector.process(image(road(), hsvColor(35), color), 100);
    require(!result.valid && cv::countNonZero(result.road_mask) == 0,
            "hue-wrap must still enforce shared saturation and value bounds");
  }
  const cv::Mat straight = mixedHueRoad(0, 35);
  const auto fitted = detector.process(straight, 100);
  require(fitted.valid && fitted.type == RoadType::Straight,
          "large low/high-hue road patches must produce a continuous centerline");
  for (const auto& block : fitted.scan_blocks)
    require(block.found && std::abs(block.midpoint.x - 320) < 1.0f,
            "both hues must contribute to the measured road width and center");
  for (int hue : {25, 35, 45}) {
    const cv::Mat end = mixedHueRoad(300, hue);
    RoadResult result;
    for (int frame = 1; frame <= 3; ++frame) {
      result = detector.process(end, frame * 30, true);
      require(result.road_end_detected && result.raw_type == RoadType::RoadEnd,
              "excluded background hue must provide non-mask evidence in the end box");
      require(result.type == RoadType::RoadEnd,
              "mixed-hue end must publish on the first matching frame");
    }
    require(cv::countNonZero(result.road_mask(cv::Rect(280, 200, 81, 50))) == 0,
            "H25..45 beyond the cap must stay outside the road mask");
    require(result.road_mask.at<unsigned char>(310, 290) &&
            result.road_mask.at<unsigned char>(310, 350),
            "both hue groups must remain measured near the cap");
    if (hue == 35) cv::imwrite("hue_wrap_end_overlay.png", annotateResult(end, result));
  }
  cv::imwrite("hue_wrap_straight_overlay.png", annotateResult(straight, fitted));
}

void fixedHsv() {
  RoadDetector detector(config());
  for (const cv::Scalar& color : {cv::Scalar(150, 100, 80), cv::Scalar(110, 75, 60),
                                  cv::Scalar(190, 130, 105)}) {
    const auto measured = detector.process(image(road(), {160, 160, 160}, color), 100);
    require(measured.valid && measured.type == RoadType::Straight,
            "fixed HSV must track calibrated road colors under bounded light variation");
  }
  require(detector.process(image(road(300)), 100).type == RoadType::RoadEnd,
          "natural end must work through the BGR/HSV path");
  require(detector.process(image(road(300), {220, 220, 220}), 100).type == RoadType::RoadEnd,
          "white floor matching both sides is not automatically an occlusion");
  cv::Mat glare = image(road(300));
  glare(cv::Rect(275, 240, 91, 40)).setTo(cv::Scalar(255, 255, 255));
  require(detector.process(glare, 100).road_end_detected,
          "background color and glare must not add a veto to the two area conditions");
  cv::Mat dim = image(road(), {160, 160, 160}, {20, 13, 10});
  const auto lost = detector.process(dim, 100);
  require(!lost.valid && lost.type == RoadType::Unknown && !lost.road_end_detected,
          "out-of-calibration light fails as Unknown without adaptive compensation");
  auto wrapped = config();
  wrapped.road_hsv = {170, 10, 100, 255, 80, 255};
  require(RoadDetector(wrapped).process(image(road(), {160,160,160}, {0,0,180}), 100).valid,
          "fixed HSV must retain hue-wrap support");
}

void occlusion() {
  RoadDetector detector(config());
  cv::Mat frame = image(road());
  frame(cv::Rect(280, 250, 81, 20)).setTo(cv::Scalar(240, 240, 240));
  const auto bridged = detector.process(frame, 100);
  require(bridged.occlusion_detected && !bridged.road_end_detected,
          "white gap with road beyond it is an occlusion");
  require(bridged.valid, "sufficiently filled narrow masks still contribute geometry");
  require(cv::countNonZero(bridged.bridge_mask) > 0, "a short supported gap can associate road");
  for (const auto& block : bridged.scan_blocks) {
    if (!block.found) continue;
    const cv::Point point(cvRound(block.midpoint.x), cvRound(block.midpoint.y));
    require(bridged.road_mask.at<unsigned char>(point) && !bridged.bridge_mask.at<unsigned char>(point),
            "center points must be real measurements, never white bridge pixels");
  }
  cv::Mat large = image(road());
  large(cv::Rect(280, 210, 81, 100)).setTo(cv::Scalar(240,240,240));
  const auto blocked = detector.process(large, 100);
  require(blocked.road_end_detected,
          "visible continuation and occlusion diagnostics do not veto the requested area rule");
  cv::Mat bottom = image(road());
  bottom(cv::Rect(280, 300, 81, 60)).setTo(cv::Scalar(240,240,240));
  const auto lost = detector.process(bottom, 100);
  require(lost.type == RoadType::Straight && lost.valid && !lost.centerline.empty() &&
          !lost.road_end_detected,
          "remaining filled narrow masks provide geometry without becoming RoadEnd");
  cv::Mat side = image(road());
  side(cv::Rect(0, 200, 100, 100)).setTo(cv::Scalar(255,255,255));
  const auto outside = detector.process(side, 100);
  require(outside.valid && !outside.occlusion_detected && !outside.road_end_detected,
          "white floor outside the corridor must not be treated as an obstruction");
}
}  // namespace

int main() {
  try {
    fixedHsv();
    crossZeroRoad();
    occlusion();
    std::cout << "field conditions tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "field conditions tests failed: " << error.what() << '\n';
    return 1;
  }
}
