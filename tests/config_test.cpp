#include "road_finder/road_detector.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Fn>
void requireThrows(Fn&& fn, const std::string& needle, const std::string& label) {
  try {
    fn();
  } catch (const std::exception& error) {
    const std::string what = error.what();
    require(what.find(needle) != std::string::npos,
            label + ": expected message containing \"" + needle + "\", got: " + what);
    return;
  }
  require(false, label + ": expected an exception");
}

std::string writeConfig(const std::string& name, const std::string& body) {
  std::ofstream out(name);
  out << "%YAML:1.0\n---\n" << body;
  return name;
}

void expectDefaultsAndValidation() {
  road_finder::Config config;
  require(config.processHeight() == 360, "default process size should be 640x360");
  require(config.threads == 1, "one worker should be configured");
  require(!config.stop_detection_enabled, "stop recognition should default off");
  road_finder::validateConfig(config);

  road_finder::Config hd;
  hd.input_width = 1920;
  hd.input_height = 1080;
  hd.roi = cv::Rect(40, 130, 1840, 918);
  require(hd.processHeight() == 360, "1080p input should retain the aspect ratio");
  road_finder::validateConfig(hd);

  road_finder::Config outside;
  outside.roi = cv::Rect(100, 100, 1200, 612);
  requireThrows([&] { road_finder::validateConfig(outside); }, "inside the input",
                "ROI outside image");

  road_finder::Config short_roi;
  short_roi.roi = cv::Rect(26, 86, 1229, 300);
  requireThrows([&] { road_finder::validateConfig(short_roi); }, "80%",
                "ROI not reaching the lower image");

  road_finder::Config invalid_lab;
  invalid_lab.road_hsv.v_min = 200;
  invalid_lab.road_hsv.v_max = 100;
  requireThrows([&] { road_finder::validateConfig(invalid_lab); }, "invalid HSV range",
                "reversed HSV bounds");

  road_finder::Config invalid_band;
  invalid_band.left_marker_rois[0] = cv::Rect(0, 85, 128, 7);
  requireThrows([&] { road_finder::validateConfig(invalid_band); }, "128x90",
                "scan band outside reference image");

  invalid_lab.road_hsv = {170, 10, 0, 255, 0, 255};
  road_finder::validateConfig(invalid_lab);
  invalid_lab.road_hsv.h_max = 180;
  requireThrows([&] { road_finder::validateConfig(invalid_lab); }, "invalid HSV", "H upper bound");
  for (int threads : {0, -1, std::numeric_limits<int>::max(), 100}) {
    config.threads = threads;
    requireThrows([&] { road_finder::RoadDetector detector(config); }, "threads", "invalid threads");
  }

  road_finder::Config field_config;
  field_config.side_road_hsv = road_finder::HsvRange{0, 180, 0, 255, 0, 255};
  requireThrows([&] { road_finder::validateConfig(field_config); }, "invalid HSV", "side range");
  field_config.side_road_hsv.reset();
  field_config.side_right_begin_reference = field_config.side_left_end_reference;
  requireThrows([&] { road_finder::validateConfig(field_config); }, "boundaries", "overlapping sides");
  field_config = road_finder::Config{};
  field_config.marker_min_coverage = 0;
  requireThrows([&] { road_finder::validateConfig(field_config); }, "fractions", "zero coverage");
  field_config.marker_min_coverage = 0.35f;
  field_config.occlusion_max_gap_fraction = std::numeric_limits<float>::quiet_NaN();
  requireThrows([&] { road_finder::validateConfig(field_config); }, "fractions", "NaN gap");
  field_config.occlusion_max_gap_fraction = 0.25f;
  field_config.white_v_min = 256;
  requireThrows([&] { road_finder::validateConfig(field_config); }, "white thresholds", "white bounds");

  road_finder::Config invalid_mode;
  invalid_mode.segmentation_mode = "unknown";
  requireThrows([&] { road_finder::validateConfig(invalid_mode); }, "segmentation_mode",
                "unsupported segmentation");
  road_finder::Config invalid_contrast;
  invalid_contrast.road_contrast_min = 0;
  requireThrows([&] { road_finder::validateConfig(invalid_contrast); }, "road_contrast_min",
                "zero contrast threshold");
  road_finder::Config invalid_width;
  invalid_width.min_road_width_fraction = invalid_width.max_road_width_fraction;
  requireThrows([&] { road_finder::validateConfig(invalid_width); }, "width fractions",
                "reversed width limits");
  invalid_width.min_road_width_fraction = std::numeric_limits<float>::quiet_NaN();
  requireThrows([&] { road_finder::validateConfig(invalid_width); }, "width fractions",
                "non-finite width limit");
  road_finder::Config invalid_temporal;
  invalid_temporal.classification_confirm_frames = 0;
  requireThrows([&] { road_finder::validateConfig(invalid_temporal); }, "frame counts",
                "zero confirmation frames");
  invalid_temporal.classification_confirm_frames = 3;
  invalid_temporal.classification_hold_frames = 11;
  requireThrows([&] { road_finder::validateConfig(invalid_temporal); }, "frame counts",
                "excessive hold frames");
  invalid_temporal.classification_hold_frames = 2;
  invalid_temporal.geometry_smoothing_alpha = std::numeric_limits<float>::quiet_NaN();
  requireThrows([&] { road_finder::validateConfig(invalid_temporal); }, "smoothing_alpha",
                "non-finite smoothing coefficient");
}

void expectConfigFile() {
  const std::string modern = writeConfig(
      "config_test_modern.yaml",
      "input_width: 1280\ninput_height: 720\nprocess_width: 640\n"
      "roi: [10, 20, 300, 700]\nroad_hsv: [120, 170, 120, 150, 120, 150]\n"
      "segmentation_mode: adaptive_dark\nroad_contrast_min: 22\n"
      "min_road_width_fraction: 0.08\nmax_road_width_fraction: 0.75\n"
      "classification_confirm_frames: 4\nclassification_hold_frames: 2\n"
      "geometry_smoothing_alpha: 0.4\n"
      "stop_detection_enabled: 1\nleft_marker_rois: [[11, 40, 10, 10], [11, 50, 10, 10], [11, 60, 10, 10]]\n");
  const road_finder::Config loaded = road_finder::loadConfig(modern);
  require(loaded.roi == cv::Rect(10, 20, 300, 700), "roi should be read in input pixels");
  require(loaded.road_hsv.h_min == 120 && loaded.road_hsv.v_max == 150,
          "road HSV range should be read");
  require(loaded.stop_detection_enabled, "stop recognition toggle should be read");
  require(loaded.left_marker_rois[1] == cv::Rect(11, 50, 10, 10),
          "sample ROI override should be read");
  require(loaded.segmentation_mode == "adaptive_dark" && loaded.road_contrast_min == 22,
          "adaptive segmentation options should be read");
  require(std::abs(loaded.min_road_width_fraction - 0.08f) < 1e-6f &&
              std::abs(loaded.max_road_width_fraction - 0.75f) < 1e-6f,
          "road width limits should be read");
  require(loaded.classification_confirm_frames == 4 && loaded.classification_hold_frames == 2 &&
              std::abs(loaded.geometry_smoothing_alpha - 0.4f) < 1e-6f,
          "video confirmation and smoothing options should be read");
  std::remove(modern.c_str());

  const auto field = writeConfig("config_field.yaml",
      "road_hsv: [90, 120, 20, 140, 100, 230]\n"
      "side_road_hsv: [95, 130, 15, 180, 20, 160]\n"
      "side_left_end_reference: 35\nside_right_begin_reference: 93\n"
      "marker_min_coverage: 0.45\nwhite_s_max: 35\nwhite_v_min: 180\n"
      "occlusion_max_gap_fraction: 0.2\ngeometry_min_span_fraction: 0.25\n");
  const auto field_loaded = road_finder::loadConfig(field);
  require(field_loaded.side_road_hsv && field_loaded.side_road_hsv->v_min == 20 &&
              field_loaded.road_hsv.v_min == 100, "central and shared side HSV must be independent");
  require(field_loaded.side_left_end_reference == 35 && field_loaded.side_right_begin_reference == 93 &&
              std::abs(field_loaded.marker_min_coverage - 0.45f) < 1e-6f &&
              field_loaded.white_s_max == 35 && field_loaded.white_v_min == 180 &&
              std::abs(field_loaded.occlusion_max_gap_fraction - 0.2f) < 1e-6f &&
              std::abs(field_loaded.geometry_min_span_fraction - 0.25f) < 1e-6f,
          "field calibration controls must load");
  std::remove(field.c_str());
  const auto inherited = writeConfig("config_inherited.yaml", "road_hsv: [90,120,20,140,100,230]\n");
  require(!road_finder::loadConfig(inherited).side_road_hsv,
          "omitted side range must inherit the configured central range");
  std::remove(inherited.c_str());

  const std::string bad_height =
      writeConfig("config_test_height.yaml", "process_height: 360\n");
  requireThrows([&] { road_finder::loadConfig(bad_height); },
                "process_height is no longer used", "deprecated process_height");
  std::remove(bad_height.c_str());

  const std::string bad_fraction = writeConfig(
      "config_test_fraction.yaml", "roi_fraction: [0.02, 0.12, 0.96, 0.85]\n");
  requireThrows([&] { road_finder::loadConfig(bad_fraction); },
                "[26, 86, 1229, 612]", "deprecated roi_fraction");
  std::remove(bad_fraction.c_str());

  for (const char* key : {"road_lab", "stop_lab", "scan_bands", "left_marker_roi"}) {
    const auto legacy = writeConfig("config_legacy.yaml", std::string(key) + ": [0, 255]\n");
    requireThrows([&] { road_finder::loadConfig(legacy); }, "no longer supported", "legacy key");
    std::remove(legacy.c_str());
  }

  const std::string bad_lab =
      writeConfig("config_test_lab.yaml", "road_hsv: [200, 100, 0, 255, 0, 255]\n");
  requireThrows([&] { road_finder::loadConfig(bad_lab); }, "invalid HSV range",
                "reversed road HSV range");
  std::remove(bad_lab.c_str());
}

}  // namespace

int main() {
  try {
    expectDefaultsAndValidation();
    expectConfigFile();
    std::cout << "config tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "config tests failed: " << error.what() << '\n';
    return 1;
  }
}
