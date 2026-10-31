#include "road_finder/preview.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace road_finder {
namespace {

constexpr const char* kPreviewWindow = "road_fucker preview";

const char* roadCondition(const RoadResult& result) {
  if (!result.valid && result.type == RoadType::UNKNOWN) return "LOST";
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

// Draws into a caller-owned canvas so a streaming preview does not allocate a
// full frame every iteration. All geometry is scaled by canvas/frame so a
// downscaled preview keeps the same layout.
void drawInto(const cv::Mat& frame, const RoadResult& result, bool show_fps, double fps,
              double scale, cv::Mat& canvas) {
  if (scale >= 1.0) {
    frame.copyTo(canvas);
  } else {
    cv::resize(frame, canvas,
               cv::Size(std::max(1, cvRound(frame.cols * scale)),
                        std::max(1, cvRound(frame.rows * scale))),
               0, 0, cv::INTER_AREA);
  }
  const double s = static_cast<double>(canvas.cols) / std::max(1, frame.cols);
  const auto at = [s](int x, int y) { return cv::Point(cvRound(x * s), cvRound(y * s)); };
  const double font = std::max(0.4, 0.7 * s);
  const int text_thickness = std::max(1, cvRound(2 * s));
  const int line_thickness = std::max(1, cvRound(3 * s));

  const bool classification_only = !result.valid && result.type != RoadType::UNKNOWN;
  const int panel_height = (show_fps ? 120 : 88) + (classification_only ? 32 : 0);
  const cv::Rect panel(cvRound(12 * s), cvRound(12 * s), cvRound(440 * s),
                       cvRound(panel_height * s));
  cv::Mat panel_pixels = canvas(panel);
  cv::Mat black(panel.size(), canvas.type(), cv::Scalar::all(0));
  cv::addWeighted(panel_pixels, 0.4, black, 0.6, 0.0, panel_pixels);
  const cv::Scalar white(255, 255, 255);
  cv::putText(canvas, std::string("ROAD: ") + roadCondition(result), at(24, 43),
              cv::FONT_HERSHEY_SIMPLEX, font, white, text_thickness, cv::LINE_AA);
  const float displayed_confidence = result.type != RoadType::UNKNOWN
      ? result.classification_confidence : result.confidence;
  const std::string confidence = result.valid || classification_only
      ? cv::format("%.2f", displayed_confidence) : "--";
  cv::putText(canvas, "CONFIDENCE: " + confidence, at(24, 76), cv::FONT_HERSHEY_SIMPLEX, font,
              white, text_thickness, cv::LINE_AA);
  if (show_fps) {
    const std::string label = fps < 0.0 ? "MEASURING" : cv::format("%.1f", fps);
    cv::putText(canvas, "FPS: " + label, at(24, 109), cv::FONT_HERSHEY_SIMPLEX, font, white,
                text_thickness, cv::LINE_AA);
  }
  if (classification_only)
    cv::putText(canvas, "GEOMETRY: UNAVAILABLE", at(24, show_fps ? 141 : 108),
                cv::FONT_HERSHEY_SIMPLEX, font * 0.8, white, text_thickness, cv::LINE_AA);
  // Detection window actually used for this frame, in input-image pixels.
  if (result.roi_polygon.size() == 4) {
    std::vector<cv::Point> window;
    window.reserve(result.roi_polygon.size());
    for (const cv::Point2f& point : result.roi_polygon)
      window.emplace_back(cvRound(point.x * s), cvRound(point.y * s));
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{window}, /*isClosed=*/true,
                  cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
  const auto draw_line = [&](const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (std::size_t i = 1; i < points.size(); ++i)
      cv::line(canvas, at(cvRound(points[i - 1].x), cvRound(points[i - 1].y)),
               at(cvRound(points[i].x), cvRound(points[i].y)), color, line_thickness, cv::LINE_AA);
  };
  if (result.valid) {
    draw_line(result.left_boundary, {255, 255, 0});  // Cyan in BGR.
    draw_line(result.right_boundary, {0, 165, 255}); // Orange in BGR.
    draw_line(result.centerline, {0, 255, 0});
  }
}

}  // namespace

cv::Mat annotateResult(const cv::Mat& frame, const RoadResult& result) {
  cv::Mat output;
  drawInto(frame, result, false, -1.0, 1.0, output);
  return output;
}

Preview::Preview(cv::Size image_size, double scale, double refresh_hz)
    : scale_(scale > 0.0 ? scale : 1.0), refresh_hz_(refresh_hz > 0.0 ? refresh_hz : 0.0) {
  const cv::Size window_size(std::max(1, cvRound(image_size.width * scale_)),
                             std::max(1, cvRound(image_size.height * scale_)));
  try {
    cv::namedWindow(kPreviewWindow, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO |
                                       cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(kPreviewWindow, window_size);
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

bool Preview::refreshDue(bool forced) const {
  if (forced || refresh_hz_ <= 0.0 || !drawn_once_) return true;
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - last_draw_).count();
  return elapsed_ms >= 1000.0 / refresh_hz_;
}

bool Preview::show(const cv::Mat& frame, const RoadResult& result, int wait_ms) {
  // wait_ms == 0 means a single image is on screen and must be drawn once.
  if (refreshDue(wait_ms <= 0)) {
    drawInto(frame, result, true, fps_, scale_, canvas_);
    cv::imshow(kPreviewWindow, canvas_);
    last_draw_ = std::chrono::steady_clock::now();
    drawn_once_ = true;
  }
  const int key = cv::waitKey(wait_ms);
  if (key == 's' || key == 'S') snapshot_requested_ = true;
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
