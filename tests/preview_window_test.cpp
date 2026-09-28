#include "road_finder/preview.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using road_finder::PreviewWindowState;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

// The GTK backend does not implement cv::WND_PROP_VISIBLE: OpenCV reports -1 for
// every frame. The preview must keep running instead of quitting after the first
// frame.
void expectUnusableReadings(double reading, const std::string& label) {
  PreviewWindowState state;
  for (int frame = 0; frame < 100; ++frame) {
    require(state.observeVisible(reading) == PreviewWindowState::Decision::kKeepRunning,
            label + ": the preview must keep running");
    require(!state.visible_seen(), label + ": no visible window may be reported");
  }
  require(state.unusable_readings() == 100, label + ": unusable readings must be counted");
  require(state.visibility_reporting_unavailable(),
          label + ": unsupported visibility reporting must be detected");
}

void expectLateWindowMapping() {
  PreviewWindowState state;
  for (int frame = 0; frame < 20; ++frame)
    require(state.observeVisible(0.0) == PreviewWindowState::Decision::kKeepRunning,
            "a window that is not mapped yet must not be treated as closed");
  require(!state.visible_seen(), "the startup readings must not arm close detection");
  require(state.observeVisible(1.0) == PreviewWindowState::Decision::kKeepRunning,
          "a visible window must keep the preview running");
  require(state.visible_seen(), "a visible reading must arm close detection");
  require(state.observeVisible(0.0) == PreviewWindowState::Decision::kWindowClosed,
          "a window that became invisible must be treated as closed");
  require(state.observeVisible(1.0) == PreviewWindowState::Decision::kWindowClosed,
          "a closed window must stay closed");
}

void expectVisibleThreshold() {
  PreviewWindowState state;
  require(state.observeVisible(0.9) == PreviewWindowState::Decision::kKeepRunning,
          "a visible reading must keep the preview running");
  require(state.visible_seen(), "a reading above the threshold must arm close detection");
  require(state.observeVisible(0.5) == PreviewWindowState::Decision::kKeepRunning,
          "the threshold reading must be treated as visible");
}

void expectProbeWindow() {
  PreviewWindowState state;
  for (int frame = 0; frame + 1 < PreviewWindowState::kUnavailableProbeFrames; ++frame) {
    require(state.observeVisible(-1.0) == PreviewWindowState::Decision::kKeepRunning,
            "an unusable reading must keep the preview running");
    require(!state.visibility_reporting_unavailable(),
            "the backend must be reported as unsupported only after enough frames");
  }
  require(state.observeVisible(-1.0) == PreviewWindowState::Decision::kKeepRunning,
          "an unusable reading must keep the preview running");
  require(state.visibility_reporting_unavailable(),
          "enough unusable readings must report the backend as unsupported");
}

}  // namespace

int main() {
  try {
    expectUnusableReadings(-1.0, "-1.0");
    expectUnusableReadings(std::numeric_limits<double>::quiet_NaN(), "NaN");
    expectUnusableReadings(-0.25, "-0.25");
    expectLateWindowMapping();
    expectVisibleThreshold();
    expectProbeWindow();
    std::cout << "preview window tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "preview window tests failed: " << error.what() << '\n';
    return 1;
  }
}
