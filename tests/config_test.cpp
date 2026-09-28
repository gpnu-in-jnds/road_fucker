#include "road_finder/road_detector.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
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
            label + ": expected a message containing \"" + needle + "\", got: " + what);
    return;
  }
  require(false, label + ": expected an exception");
}

std::string writeConfig(const std::string& name, const std::string& body) {
  std::ofstream out(name);
  out << "%YAML:1.0\n---\n" << body;
  return name;
}

void expectDefaults() {
  const road_finder::Config config;
  require(config.processHeight() == 360, "the default process width must give a 360 px height");
  require(config.roi == cv::Rect(26, 86, 1229, 612), "unexpected default roi");
  road_finder::validateConfig(config);
}

void expectDerivedHeight() {
  road_finder::Config hd;
  hd.input_width = 1920;
  hd.input_height = 1080;
  hd.roi = cv::Rect(40, 130, 1840, 918);
  require(hd.processHeight() == 360, "1080p input must give a 360 px processing height");
  road_finder::validateConfig(hd);

  road_finder::Config rounded;
  rounded.process_width = 500;
  require(rounded.processHeight() == 281, "500/1280x720 must round to 281");
  road_finder::validateConfig(rounded);
}

void expectValidation() {
  road_finder::Config tiny;
  tiny.process_width = 40;
  requireThrows([&] { road_finder::validateConfig(tiny); }, "process_width", "tiny process width");

  road_finder::Config short_height;
  short_height.process_width = 100;  // 100/1280x720 gives a 56 px processing height.
  requireThrows([&] { road_finder::validateConfig(short_height); }, "too small",
                "process width whose height is too small");

  road_finder::Config outside;
  outside.roi = cv::Rect(100, 100, 1200, 612);
  requireThrows([&] { road_finder::validateConfig(outside); }, "must stay inside",
                "roi outside the input image");

  road_finder::Config floating;
  floating.roi = cv::Rect(26, 86, 1229, 300);
  requireThrows([&] { road_finder::validateConfig(floating); }, "at least 80%",
                "roi that stops above the bottom");
}

void expectConfigFile() {
  const std::string modern = writeConfig(
      "config_test_modern.yaml",
      "input_width: 1280\ninput_height: 720\nprocess_width: 640\nroi: [10, 20, 300, 700]\n");
  const road_finder::Config loaded = road_finder::loadConfig(modern);
  require(loaded.roi == cv::Rect(10, 20, 300, 700), "roi must be read as input pixels");
  require(loaded.processHeight() == 360, "process height must follow the input aspect ratio");
  std::remove(modern.c_str());

  const std::string with_height = writeConfig(
      "config_test_height.yaml", "input_width: 1280\ninput_height: 720\nprocess_height: 360\n");
  requireThrows([&] { road_finder::loadConfig(with_height); }, "process_height is no longer used",
                "deprecated process_height");
  std::remove(with_height.c_str());

  const std::string with_fraction = writeConfig(
      "config_test_fraction.yaml",
      "input_width: 1280\ninput_height: 720\nroi_fraction: [0.02, 0.12, 0.96, 0.85]\n");
  requireThrows([&] { road_finder::loadConfig(with_fraction); }, "[26, 86, 1229, 612]",
                "deprecated roi_fraction must name the replacement");
  std::remove(with_fraction.c_str());
}

}  // namespace

int main() {
  try {
    expectDefaults();
    expectDerivedHeight();
    expectValidation();
    expectConfigFile();
    std::cout << "config tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "config tests failed: " << error.what() << '\n';
    return 1;
  }
}
