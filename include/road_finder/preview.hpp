#pragma once

#include "road_finder/road_detector.hpp"

#include <opencv2/core.hpp>

#include <chrono>
#include <cmath>
#include <string>

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

// Draws the boundaries, the centerline and the road-condition panel, without the
// frame-rate line. Used for the single-frame preview and for --overlay exports.
cv::Mat annotateResult(const cv::Mat& frame, const RoadResult& result);

// Owns the preview window: creation, drawing, event loop and close detection.
class Preview {
 public:
  // image_size is the size of the frames handed to show(). scale shrinks the
  // preview and the window: the OpenCV GTK backend converts every displayed
  // frame to RGB in software and blits it with cairo, so on a weak SoC a smaller
  // preview is much cheaper. refresh_hz > 0 limits how often the window is
  // redrawn while detection keeps running on every frame (0 = every frame).
  // Throws when the window cannot be created (no display, no GUI backend).
  Preview(cv::Size image_size, double scale = 1.0, double refresh_hz = 0.0);

  // Draws one frame and pumps the GUI event loop. wait_ms is passed to
  // cv::waitKey: 1 while streaming, 0 to block on a single image. Returns false
  // when the preview must stop; stopReason() then explains why.
  bool show(const cv::Mat& frame, const RoadResult& result, int wait_ms = 1);

  const std::string& stopReason() const { return stop_reason_; }

  bool takeSnapshotRequest() {
    const bool requested = snapshot_requested_;
    snapshot_requested_ = false;
    return requested;
  }

 private:
  void frameCompleted();
  bool refreshDue(bool forced) const;

  PreviewWindowState state_;
  bool unusable_visibility_reported_ = false;
  std::string stop_reason_;
  cv::Mat canvas_;
  double scale_ = 1.0;
  double refresh_hz_ = 0.0;
  std::chrono::steady_clock::time_point last_draw_{};
  bool drawn_once_ = false;
  bool snapshot_requested_ = false;
  std::chrono::steady_clock::time_point fps_window_start_ = std::chrono::steady_clock::now();
  int fps_frames_ = 0;
  double fps_ = -1.0;
};

}  // namespace road_finder
