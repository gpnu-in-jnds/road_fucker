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
  const int panel_height = (show_fps ? 176 : 144) + (classification_only ? 32 : 0);
  const cv::Rect panel(cvRound(12 * s), cvRound(12 * s), cvRound(440 * s),
                       cvRound(panel_height * s));
  const cv::Rect clipped_panel = panel & cv::Rect(0, 0, canvas.cols, canvas.rows);
  cv::Mat panel_pixels = canvas(clipped_panel);
  cv::Mat black(clipped_panel.size(), canvas.type(), cv::Scalar::all(0));
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
  const int diagnostics_y = (show_fps ? 141 : 108) + (classification_only ? 32 : 0);
  const auto votes = [](const char* side, int hits) {
    return std::string(side) + " " + std::to_string(hits) + "/3 " +
        (hits >= 2 ? "VALID" : "INVALID");
  };
  cv::putText(canvas, votes("LEFT", result.left_junction_hits) + "  " +
              votes("RIGHT", result.right_junction_hits), at(24, diagnostics_y),
              cv::FONT_HERSHEY_SIMPLEX, font * 0.7, white, 1, cv::LINE_AA);
  cv::putText(canvas, result.occlusion_detected ? "OCCLUSION: DETECTED" : "OCCLUSION: NONE",
              at(24, diagnostics_y + 28), cv::FONT_HERSHEY_SIMPLEX, font * 0.7,
              result.occlusion_detected ? cv::Scalar(0, 165, 255) : white, 1, cv::LINE_AA);
  // Detection window actually used for this frame, in input-image pixels.
  if (result.roi_polygon.size() == 4) {
    std::vector<cv::Point> window;
    window.reserve(result.roi_polygon.size());
    for (const cv::Point2f& point : result.roi_polygon)
      window.emplace_back(cvRound(point.x * s), cvRound(point.y * s));
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{window}, /*isClosed=*/true,
                  cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
  const auto draw_polygon = [&](const std::vector<cv::Point2f>& polygon,
                                 const cv::Scalar& color) {
    if (polygon.size() != 4) return;
    std::vector<cv::Point> points;
    for (const auto& point : polygon)
      points.emplace_back(cvRound(point.x * s), cvRound(point.y * s));
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{points}, true, color,
                  std::max(1, cvRound(2 * s)), cv::LINE_AA);
  };
  for (const auto& block : result.scan_blocks) {
    if (block.found)
      cv::circle(canvas, at(cvRound(block.midpoint.x), cvRound(block.midpoint.y)),
                 std::max(2, cvRound(5 * s)), {255, 0, 255}, -1, cv::LINE_AA);
  }
  for (std::size_t i = 0; i < result.junction_blocks.size(); ++i) {
    const auto& block = result.junction_blocks[i];
    const cv::Scalar color = block.found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    draw_polygon(block.polygon, color);
    if (!block.polygon.empty()) {
      const auto& point = block.polygon.front();
      cv::putText(canvas, std::string(i < 3 ? "L" : "R") + std::to_string(i % 3 + 1),
                  at(cvRound(point.x), cvRound(point.y) - 3),
                  cv::FONT_HERSHEY_SIMPLEX, std::max(0.3, 0.45 * s), color, 1, cv::LINE_AA);
    }
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
    for (const auto& point : result.centerline)
      cv::circle(canvas, at(cvRound(point.x), cvRound(point.y)),
                 std::max(2, cvRound(5 * s)), {0, 255, 255}, 1, cv::LINE_AA);
  }
}

}  // namespace

cv::Mat annotateResult(const cv::Mat& frame, const RoadResult& result, bool full_process) {
  cv::Mat output;
  drawInto(frame, result, false, -1.0, 1.0, output);
  if (!full_process) return output;

  // Each panel has input-frame dimensions. Mask stays in processing/warped
  // coordinates; source overlays use inverse-transformed polygons and points.
  cv::Mat montage(frame.rows * 2, frame.cols * 2, frame.type(), cv::Scalar::all(0));
  frame.copyTo(montage(cv::Rect(0, 0, frame.cols, frame.rows)));
  if (!result.road_mask.empty()) {
    cv::Mat color_mask(result.road_mask.size(), CV_8UC3, cv::Scalar::all(0)), resized;
    color_mask.setTo(cv::Scalar(0, 200, 0), result.road_mask);
    for (int side = 0; side < 2; ++side) {
      const int left = side == 0 ? 0 : result.hsv_region_edges[1];
      const int right = side == 0 ? result.hsv_region_edges[0] : color_mask.cols;
      if (left < right && left >= 0 && right <= color_mask.cols)
        color_mask.colRange(left, right).setTo(cv::Scalar(200, 200, 0),
            result.road_mask.colRange(left, right));
    }
    if (!result.white_mask.empty()) color_mask.setTo(cv::Scalar(160, 160, 160), result.white_mask);
    if (!result.occlusion_mask.empty()) color_mask.setTo(cv::Scalar(180, 0, 180), result.occlusion_mask);
    if (!result.bridge_mask.empty()) color_mask.setTo(cv::Scalar(0, 165, 255), result.bridge_mask);
    for (int edge : result.hsv_region_edges)
      if (edge > 0 && edge < color_mask.cols)
        cv::line(color_mask, {edge, 0}, {edge, color_mask.rows - 1}, {255, 255, 255}, 1);
    cv::resize(color_mask, resized, frame.size(), 0, 0, cv::INTER_NEAREST);
    resized.copyTo(montage(cv::Rect(frame.cols, 0, frame.cols, frame.rows)));
  }
  cv::Mat blocks = frame.clone();
  for (const auto& block : result.scan_blocks) {
    std::vector<cv::Point> polygon;
    for (const auto& point : block.polygon) polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    if (!polygon.empty())
      cv::polylines(blocks, std::vector<std::vector<cv::Point>>{polygon}, true,
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    if (block.found)
      cv::circle(blocks, block.midpoint, 5, {255, 0, 255}, -1, cv::LINE_AA);
  }
  blocks.copyTo(montage(cv::Rect(0, frame.rows, frame.cols, frame.rows)));
  output.copyTo(montage(cv::Rect(frame.cols, frame.rows, frame.cols, frame.rows)));
  const char* titles[] = {"INPUT", "ROAD: GREEN/CYAN | WHITE: GRAY | UNKNOWN: PURPLE | BRIDGE: ORANGE", "BLOCKS / MEASURED CENTERS",
                          "FIT / JUNCTION BOXES"};
  for (int i = 0; i < 4; ++i) {
    const cv::Point origin((i % 2) * frame.cols, (i / 2) * frame.rows);
    cv::rectangle(montage, cv::Rect(origin.x, origin.y, frame.cols, std::min(28, frame.rows)),
                  cv::Scalar::all(0), -1);
    cv::putText(montage, titles[i], origin + cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX,
                0.5, {255, 255, 255}, 1, cv::LINE_AA);
  }
  return montage;
}

Preview::Preview(cv::Size image_size, double scale, double refresh_hz, bool full_process)
    : full_process_(full_process), scale_(scale > 0.0 ? scale : 1.0), refresh_hz_(refresh_hz > 0.0 ? refresh_hz : 0.0) {
  if (full_process_) image_size = cv::Size(image_size.width * 2, image_size.height * 2);
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
    if (full_process_) {
      const cv::Mat process_view = annotateResult(frame, result, true);
      cv::resize(process_view, canvas_, cv::Size(std::max(1, cvRound(process_view.cols * scale_)),
          std::max(1, cvRound(process_view.rows * scale_))), 0, 0, cv::INTER_AREA);
      cv::putText(canvas_, fps_ < 0 ? "FPS: MEASURING" : cv::format("FPS: %.1f", fps_),
                  {10, std::max(20, canvas_.rows - 10)}, cv::FONT_HERSHEY_SIMPLEX,
                  0.5, {255, 255, 255}, 1, cv::LINE_AA);
    } else {
      drawInto(frame, result, true, fps_, scale_, canvas_);
    }
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
