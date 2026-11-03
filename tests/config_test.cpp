#include "test_support.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>

namespace {
using namespace road_finder;
using road_finder::test::require;

template <typename Fn>
void rejected(Fn&& fn, const std::string& message) {
  try { fn(); }
  catch (const std::exception& error) {
    require(std::string(error.what()).find(message) != std::string::npos,
            "expected " + message + ", got " + error.what());
    return;
  }
  throw std::runtime_error("expected rejection containing " + message);
}

void loadText(const std::string& body, const std::string& error) {
  const std::string path = "config_test_invalid.yaml";
  { std::ofstream file(path); file << "%YAML:1.0\n---\n" << body; }
  rejected([&] { loadConfig(path); }, error);
  std::remove(path.c_str());
}
}  // namespace

int main() {
  try {
    Config value;
    validateConfig(value);
    require(value.processHeight() == 360, "defaults must retain input aspect ratio");
    for (const cv::Rect rect : {cv::Rect(-1, 540, 200, 50), cv::Rect(400, 700, 200, 50),
                               cv::Rect(400, 540, 0, 50), cv::Rect(1200, 540, 200, 50)}) {
      value = Config{};
      value.road_end_roi = rect;
      rejected([&] { validateConfig(value); }, "road_end_roi");
    }
    value = Config{};
    value.road_end_roi = {400, 0, 200, 50};
    validateConfig(value);
    for (float fraction : {0.0f, 1.1f, std::numeric_limits<float>::quiet_NaN()}) {
      value = Config{}; value.occlusion_max_gap_fraction = fraction;
      rejected([&] { validateConfig(value); }, "fractions");
    }
    for (int threads : {0, -1, 100, std::numeric_limits<int>::max()}) {
      value = Config{}; value.threads = threads;
      rejected([&] { RoadDetector detector(value); }, "threads");
    }
    value = Config{}; value.road_hsv = {170, 10, 0, 255, 0, 255};
    validateConfig(value);
    value.road_hsv.h_max = 180;
    rejected([&] { validateConfig(value); }, "HSV");
    for (const HsvRange range : {HsvRange{50, 20, 200, 40, 230, 80},
                                HsvRange{0, 0, 0, 0, 0, 0},
                                HsvRange{179, 179, 255, 255, 255, 255},
                                HsvRange{0, 179, 0, 255, 0, 255}}) {
      value = Config{};
      value.road_hsv = range;
      validateConfig(value);
    }
    const std::array<int HsvRange::*, 6> endpoints{{
        &HsvRange::h_min, &HsvRange::h_max, &HsvRange::s_min,
        &HsvRange::s_max, &HsvRange::v_min, &HsvRange::v_max}};
    for (std::size_t i = 0; i < endpoints.size(); ++i) {
      for (int invalid : {-1, i < 2 ? 180 : 256}) {
        value = Config{};
        value.road_hsv.*endpoints[i] = invalid;
        rejected([&] { validateConfig(value); }, "HSV");
      }
    }
    value = Config{}; value.min_road_width_fraction = value.max_road_width_fraction;
    rejected([&] { validateConfig(value); }, "width fractions");
    value = Config{}; value.white_v_min = 256;
    rejected([&] { validateConfig(value); }, "white thresholds");
    value = Config{}; value.roi.width = 1281;
    rejected([&] { validateConfig(value); }, "inside the input");
    value = Config{}; value.roi = {0, 0, 1280, 200};
    rejected([&] { validateConfig(value); }, "80%");
    for (const char* key : {"segmentation_mode", "road_contrast_min", "side_road_hsv",
        "side_left_end_reference", "side_right_begin_reference", "marker_min_coverage",
        "classification_confirm_frames", "classification_hold_frames", "left_marker_rois",
        "right_marker_rois", "indicator_roi", "indicator_min_area_reference", "stop_roi",
        "stop_hsv", "stop_min_area_reference", "stop_detection_enabled", "road_lab", "stop_lab",
        "scan_bands", "left_marker_roi", "right_marker_roi"})
      loadText(std::string(key) + ": 1\n", std::string(key) + " is no longer supported");
    loadText("process_height: 360\n", "process_height is no longer used");
    loadText("roi_fraction: [0.02, 0.12, 0.96, 0.85]\n", "[26, 86, 1229, 612]");
    loadText("road_end_roi: [1, 2, 3]\n", "needs [x,y,width,height]");
    loadText("road_hsv: [1, 2, 3]\n", "needs [H_min");
    loadText("perspective_src: [[0,0], [639,0], [639,359], [0,359]]\n", "both perspective");
    value = road_finder::test::config();
    value.use_perspective = true;
    value.perspective_src = {{{0,0}, {639,0}, {639,359}, {0,359}}};
    value.perspective_dst = value.perspective_src;
    value.perspective_dst[0].x = std::numeric_limits<float>::quiet_NaN();
    rejected([&] { validateConfig(value); }, "finite");
    value.perspective_dst = {{{0,0}, {639,359}, {639,0}, {0,359}}};
    rejected([&] { validateConfig(value); }, "convex");
    const std::string path = "config_test_modern.yaml";
    {
      std::ofstream file(path);
      file << "%YAML:1.0\n---\ninput_width: 640\ninput_height: 360\nprocess_width: 320\n"
              "roi: [0,180,640,180]\nroad_end_roi: [10,10,10,10]\n"
              "road_end_confirm_frames: 4\nroad_end_min_coverage: 0.95\n"
              "road_end_support_fraction: 0.1\nroad_hsv: [50,20,200,40,230,80]\n"
              "geometry_min_span_fraction: 0.1\ngeometry_smoothing_alpha: 0.4\n"
              "white_v_min: 180\n";
    }
    const auto loaded = loadConfig(path);
    require(loaded.road_end_roi == cv::Rect(10,10,10,10) && loaded.processHeight() == 180 &&
            loaded.road_hsv.h_min == 50 && loaded.road_hsv.h_max == 20 &&
            loaded.road_hsv.s_min == 200 && loaded.road_hsv.s_max == 40 &&
            loaded.road_hsv.v_min == 230 && loaded.road_hsv.v_max == 80 &&
            loaded.white_v_min == 180,
            "modern calibration must load in original pixels");
    // Old 95% / four-frame tuning must not override the fixed 80% current-frame rule.
    auto legacy = loaded;
    legacy.process_width = 640;
    cv::Mat mask = road_finder::test::road();
    mask(legacy.road_end_roi).setTo(1);
    for (int i = 0; i < 80; ++i)
      mask.at<unsigned char>(10 + i / 10, 10 + i % 10) = 0;
    const auto result = RoadDetector(legacy).processMask(mask, 1, true);
    require(result.type == RoadType::RoadEnd && result.road_end_region.coverage == 0.8f,
            "legacy endpoint keys must be ignored instead of delaying or tightening detection");
    std::remove(path.c_str());
    std::cout << "config tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "config tests failed: " << error.what() << '\n';
    return 1;
  }
}
