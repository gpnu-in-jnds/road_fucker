#include "road_finder/road_detector.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

road_finder::Config testConfig() {
  road_finder::Config config;
  config.junction_vote_frames = 1;
  config.junction_vote_window = 1;
  config.minimum_confidence = 0.1f;
  return config;
}

cv::Mat stem() {
  cv::Mat mask = cv::Mat::zeros(360, 640, CV_8UC1);
  cv::rectangle(mask, {260, 40}, {380, 359}, cv::Scalar(255), cv::FILLED);
  return mask;
}

void expectType(const cv::Mat& mask, road_finder::RoadType expected, unsigned directions) {
  road_finder::RoadDetector detector(testConfig());
  const auto result = detector.processMask(mask, 0);
  require(result.valid, "expected valid road geometry");
  require(result.type == expected,
          std::string("expected ") + road_finder::toString(expected) + ", got " +
              road_finder::toString(result.type));
  require(result.available_directions == directions, "wrong available directions");
  require(!result.centerline.empty(), "missing centerline");
}

float xNearY(const std::vector<cv::Point2f>& points, float y) {
  require(!points.empty(), "missing fitted boundary points");
  const auto closest = std::min_element(points.begin(), points.end(),
      [y](const cv::Point2f& a, const cv::Point2f& b) {
        return std::abs(a.y - y) < std::abs(b.y - y);
      });
  require(std::abs(closest->y - y) <= 12, "boundary does not cover requested row");
  return closest->x;
}

}  // namespace

int main() {
  try {
    expectType(stem(), road_finder::RoadType::STRAIGHT, road_finder::STRAIGHT);

    cv::Mat t = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(t, {260, 165}, {380, 359}, cv::Scalar(255), cv::FILLED);
    cv::rectangle(t, {80, 95}, {560, 215}, cv::Scalar(255), cv::FILLED);
    expectType(t, road_finder::RoadType::T_JUNCTION,
               road_finder::LEFT | road_finder::RIGHT);

    cv::Mat left_branch = stem();
    cv::rectangle(left_branch, {80, 95}, {320, 215}, cv::Scalar(255), cv::FILLED);
    expectType(left_branch, road_finder::RoadType::LEFT_BRANCH,
               road_finder::LEFT | road_finder::STRAIGHT);

    cv::Mat right_branch = stem();
    cv::rectangle(right_branch, {320, 95}, {560, 215}, cv::Scalar(255), cv::FILLED);
    expectType(right_branch, road_finder::RoadType::RIGHT_BRANCH,
               road_finder::RIGHT | road_finder::STRAIGHT);

    cv::Mat left_turn = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(left_turn, {260, 165}, {380, 359}, cv::Scalar(255), cv::FILLED);
    cv::rectangle(left_turn, {80, 95}, {320, 215}, cv::Scalar(255), cv::FILLED);
    expectType(left_turn, road_finder::RoadType::LEFT_TURN, road_finder::LEFT);

    cv::Mat no_road = cv::Mat::zeros(360, 640, CV_8UC1);
    road_finder::RoadDetector detector(testConfig());
    require(!detector.processMask(no_road, 0).valid, "empty mask should be invalid");

    cv::Mat shifted = cv::Mat::zeros(360, 640, CV_8UC1);
    cv::rectangle(shifted, {325, 40}, {445, 359}, cv::Scalar(255), cv::FILLED);
    const auto shifted_result = detector.processMask(shifted, 1);
    require(shifted_result.valid && shifted_result.lateral_error > 0.2f,
            "right-shifted road should have positive lateral error");

    // Full 720p path: low-saturation road and a white transverse speed bump.
    cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(185, 195, 200));
    cv::rectangle(frame, {500, 50}, {780, 719}, cv::Scalar(150, 140, 130), cv::FILLED);
    cv::rectangle(frame, {500, 480}, {780, 494}, cv::Scalar(245, 245, 245), cv::FILLED);
    road_finder::RoadDetector camera_detector(testConfig());
    const auto image_result = camera_detector.process(frame, 2);
    require(image_result.valid, "synthetic 720p road should be found");
    require(std::abs(image_result.lateral_error) < 0.25f,
            "speed bump should not move the road center");
    require(image_result.type == road_finder::RoadType::STRAIGHT,
            "speed bump must not create a junction");

    cv::Mat t_frame(720, 1280, CV_8UC3, cv::Scalar(185, 195, 200));
    cv::rectangle(t_frame, {500, 300}, {780, 719}, cv::Scalar(150, 140, 130), cv::FILLED);
    cv::rectangle(t_frame, {230, 175}, {1050, 420}, cv::Scalar(150, 140, 130), cv::FILLED);
    road_finder::RoadDetector t_detector(testConfig());
    const auto t_image_result = t_detector.process(t_frame, 3);
    require(t_image_result.valid, "synthetic 720p T junction should be found");
    require(t_image_result.type == road_finder::RoadType::T_JUNCTION,
            "synthetic 720p T junction should be classified");

    cv::Mat distracted(720, 1280, CV_8UC3, cv::Scalar(185, 195, 200));
    const std::vector<cv::Point> road_polygon{{430, 100}, {850, 100},
                                               {1180, 719}, {100, 719}};
    cv::fillConvexPoly(distracted, road_polygon, cv::Scalar(150, 140, 130));
    cv::line(distracted, {1050, 170}, {1230, 560}, cv::Scalar(230, 225, 210), 22);
    road_finder::RoadDetector distracted_detector(testConfig());
    const auto distracted_result = distracted_detector.process(distracted, 4);
    require(distracted_result.valid && distracted_result.type == road_finder::RoadType::STRAIGHT,
            "straight road with outside right stripe should remain valid");
    require(std::abs(distracted_result.heading_error_rad) < 0.08f &&
            std::abs(distracted_result.lateral_error) < 0.12f,
            "right-side distraction must not bend the centerline");
    require(std::abs(xNearY(distracted_result.right_boundary, 360) - 989) < 30 &&
            std::abs(xNearY(distracted_result.right_boundary, 500) - 1063) < 30,
            "right boundary should follow the road, not the outside stripe");

    cv::Mat no_right_edge(720, 1280, CV_8UC3, cv::Scalar(185, 195, 200));
    cv::rectangle(no_right_edge, {500, 50}, {1279, 719},
                  cv::Scalar(150, 140, 130), cv::FILLED);
    road_finder::RoadDetector missing_edge_detector(testConfig());
    const auto missing_edge_result = missing_edge_detector.process(no_right_edge, 5);
    require(!missing_edge_result.valid && missing_edge_result.centerline.empty(),
            "missing right boundary must not produce a steering centerline");

    const cv::Mat reference = cv::imread(ROAD_FINDER_SAMPLE_IMAGE, cv::IMREAD_COLOR);
    require(!reference.empty(), "reference road image is missing");
    road_finder::RoadDetector reference_detector(testConfig());
    const auto reference_result = reference_detector.process(reference, 6);
    require(reference_result.valid && reference_result.type == road_finder::RoadType::STRAIGHT,
            "reference straight road should remain valid");
    require(std::abs(reference_result.heading_error_rad) < 0.08f,
            "reference centerline should be approximately straight");
    require(std::abs(xNearY(reference_result.right_boundary, 120) - 935) < 45 &&
            std::abs(xNearY(reference_result.right_boundary, 360) - 1090) < 45 &&
            std::abs(xNearY(reference_result.right_boundary, 600) - 1250) < 45,
            "reference right boundary deviates from the visible road edge");

    road_finder::Config temporal_config = testConfig();
    temporal_config.junction_vote_frames = 3;
    temporal_config.junction_vote_window = 5;
    road_finder::RoadDetector temporal_detector(temporal_config);
    require(temporal_detector.processMask(t, 4).type == road_finder::RoadType::UNKNOWN,
            "junction should wait for temporal confirmation");
    require(temporal_detector.processMask(t, 5).type == road_finder::RoadType::UNKNOWN,
            "junction should wait for three frames");
    require(temporal_detector.processMask(t, 6).type == road_finder::RoadType::T_JUNCTION,
            "junction should be confirmed on the third frame");
    require(temporal_detector.processMask(no_road, 7).type == road_finder::RoadType::UNKNOWN,
            "lost road must not retain the previous turn");

    cv::Mat empty_scene(720, 1280, CV_8UC3, cv::Scalar(180, 180, 180));
    require(!camera_detector.process(empty_scene, 8).valid,
            "uniform scene must not be reported as a road");

    road_finder::Config calibrated = testConfig();
    calibrated.use_perspective = true;
    calibrated.perspective_src = {{{0, 0}, {1279, 0}, {1279, 719}, {0, 719}}};
    calibrated.perspective_dst = calibrated.perspective_src;
    road_finder::RoadDetector calibrated_detector(calibrated);
    const auto calibrated_result = calibrated_detector.processMask(stem(), 9);
    require(calibrated_result.valid && std::abs(calibrated_result.lateral_error) < 0.1f,
            "identity perspective must preserve centered-road error");
    std::cout << "road detector tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "road detector tests failed: " << error.what() << '\n';
    return 1;
  }
}
