#pragma once

#include <cmath>

namespace road_finder {

// Decides whether the highgui preview window must be treated as closed.
//
// cv::WND_PROP_VISIBLE is only implemented by some highgui backends: the GTK
// backend does not implement it (opencv/opencv#25346), so cv::getWindowProperty
// reports -1 there (OpenCV maps the backend's quiet_NaN to -1). A reading that
// carries no information must never be mistaken for a closed window, which is
// what made the preview quit right after the first frame on GTK-based builds.
class PreviewWindowState {
 public:
  enum class Decision { kKeepRunning, kWindowClosed };

  // Feeds one cv::getWindowProperty(..., cv::WND_PROP_VISIBLE) reading.
  Decision observeVisible(double reading) {
    if (closed_) return Decision::kWindowClosed;
    if (!std::isfinite(reading) || reading < 0.0) {
      // -1, NaN or any other negative value: the backend cannot answer.
      ++unusable_readings_;
      return Decision::kKeepRunning;
    }
    if (reading < 0.5) {
      // 0.0: the window is not visible. Before the backend has ever reported a
      // visible window this is just the startup phase, in which the window may
      // not be mapped yet.
      if (!visible_seen_) return Decision::kKeepRunning;
      closed_ = true;
      return Decision::kWindowClosed;
    }
    visible_seen_ = true;
    return Decision::kKeepRunning;
  }

  bool visible_seen() const { return visible_seen_; }

  int unusable_readings() const { return unusable_readings_; }

  // True once enough frames have passed without any usable visibility reading:
  // the backend does not support window-close detection, so the preview must be
  // stopped with Esc, Q or Ctrl+C instead.
  bool visibility_reporting_unavailable() const {
    return !visible_seen_ && unusable_readings_ >= kUnavailableProbeFrames;
  }

  static constexpr int kUnavailableProbeFrames = 30;

 private:
  bool visible_seen_ = false;
  bool closed_ = false;
  int unusable_readings_ = 0;
};

}  // namespace road_finder
