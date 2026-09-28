#include "road_finder/preview.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <limits>
#include <stdexcept>

namespace road_finder {
namespace {

constexpr const char* kPreviewWindow = "road_finder preview";

const char* roadCondition(const RoadResult& result) {
  if (!result.valid) return "LOST";
  switch (result.type) {
    case RoadType::STRAIGHT: return "STRAIGHT";
    case RoadType::LEFT_TURN: return "LEFT TURN";
    case RoadType::RIGHT_TURN: return "RIGHT TURN";
    case RoadType::LEFT_BRANCH: return "LEFT BRANCH";
    case RoadType::RIGHT_BRANCH: return "RIGHT BRANCH";
    case RoadType::T_JUNCTION: return "T JUNCTION";
    case RoadType::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

cv::Mat draw(const cv::Mat& frame, const RoadResult& result, bool show_fps, double fps) {
  cv::Mat output = frame.clone();
  const int panel_height = show_fps ? 120 : 88;
  const cv::Rect panel(12, 12, 440, panel_height);
  cv::Mat panel_pixels = output(panel);
  cv::Mat black(panel.size(), output.type(), cv::Scalar::all(0));
  cv::addWeighted(panel_pixels, 0.4, black, 0.6, 0.0, panel_pixels);
  const cv::Scalar white(255, 255, 255);
  cv::putText(output, std::string("ROAD: ") + roadCondition(result), {24, 43},
              cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2, cv::LINE_AA);
  const std::string confidence = result.valid ? cv::format("%.2f", result.confidence) : "--";
  cv::putText(output, "CONFIDENCE: " + confidence, {24, 76},
              cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2, cv::LINE_AA);
  if (show_fps) {
    const std::string label = fps < 0.0 ? "MEASURING" : cv::format("%.1f", fps);
    cv::putText(output, "FPS: " + label, {24, 109},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2, cv::LINE_AA);
  }
  // Detection window actually used for this frame, in input-image pixels.
  if (result.roi_polygon.size() == 4) {
    std::vector<cv::Point> window;
    window.reserve(result.roi_polygon.size());
    for (const cv::Point2f& point : result.roi_polygon)
      window.emplace_back(cvRound(point.x), cvRound(point.y));
    cv::polylines(output, std::vector<std::vector<cv::Point>>{window}, /*isClosed=*/true,
                  cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
  const auto draw_line = [&](const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (std::size_t i = 1; i < points.size(); ++i)
      cv::line(output, points[i - 1], points[i], color, 3, cv::LINE_AA);
  };
  if (result.valid) {
    draw_line(result.left_boundary, {255, 255, 0});  // Cyan in BGR.
    draw_line(result.right_boundary, {0, 165, 255}); // Orange in BGR.
    draw_line(result.centerline, {0, 255, 0});
  }
  return output;
}

}  // namespace

cv::Mat annotateResult(const cv::Mat& frame, const RoadResult& result) {
  return draw(frame, result, false, -1.0);
}

Preview::Preview(cv::Size image_size) {
  try {
    cv::namedWindow(kPreviewWindow, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO |
                                       cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(kPreviewWindow, image_size);
  } catch (const cv::Exception& error) {
    throw std::runtime_error(
        std::string("cannot create the preview window: ") + error.what() +
        "; check DISPLAY and that OpenCV highgui has a GUI backend, or use --no-display");
  }
}

void Preview::frameCompleted() {
  ++fps_frames_;
  const auto now = std::chrono::steady_clock::now();
  const double seconds = std::chrono::duration<double>(now - fps_window_start_).count();
  if (seconds >= 1.0) {
    fps_ = fps_frames_ / seconds;
    fps_frames_ = 0;
    fps_window_start_ = now;
  }
}

bool Preview::show(const cv::Mat& frame, const RoadResult& result, int wait_ms) {
  cv::imshow(kPreviewWindow, draw(frame, result, true, fps_));
  const int key = cv::waitKey(wait_ms);
  frameCompleted();
  if (key == 27 || key == 'q' || key == 'Q') {
    stop_reason_ = "preview stopped by key";
    return false;
  }
  // A blocking wait already returned from a key press; a single image is done.
  if (wait_ms <= 0) return true;

  double visible = std::numeric_limits<double>::quiet_NaN();
  try {
    visible = cv::getWindowProperty(kPreviewWindow, cv::WND_PROP_VISIBLE);
  } catch (const cv::Exception&) {
    // A backend that cannot answer is not a closed window; the GTK backend
    // reports -1 here because it does not implement WND_PROP_VISIBLE.
  }
  if (state_.observeVisible(visible) == PreviewWindowState::Decision::kWindowClosed) {
    stop_reason_ = "preview window closed";
    return false;
  }
  if (!unusable_visibility_reported_ && state_.visibility_reporting_unavailable()) {
    std::cerr << "\npreview backend does not report WND_PROP_VISIBLE (the OpenCV GTK "
                 "backend does not implement it); window-close detection is disabled, "
                 "stop the preview with Esc or Q\n";
    unusable_visibility_reported_ = true;
  }
  return true;
}

}  // namespace road_finder
