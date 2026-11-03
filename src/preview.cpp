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
constexpr double kRoadOverlayAlpha = 0.35;

// Blend every threshold-selected pixel, including disconnected candidates.
// Map the processing/warped mask directly to the current preview resolution.
void drawRoadMask(const RoadResult& result, cv::Mat& canvas, double sx, double sy) {
  if (result.road_mask.empty()) return;
  cv::Mat mask;
  if (result.road_mask_to_source.empty()) {
    cv::resize(result.road_mask, mask, canvas.size(), 0, 0, cv::INTER_NEAREST);
  } else {
    cv::Mat transform = result.road_mask_to_source.clone();
    transform.row(0) *= sx;
    transform.row(1) *= sy;
    cv::warpPerspective(result.road_mask, mask, transform, canvas.size(),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  }
  for (int y = 0; y < canvas.rows; ++y) {
    const auto* selected = mask.ptr<unsigned char>(y);
    auto* pixels = canvas.ptr<cv::Vec3b>(y);
    for (int x = 0; x < canvas.cols; ++x) {
      if (!selected[x]) continue;
      for (int c = 0; c < 3; ++c)
        pixels[x][c] = cv::saturate_cast<unsigned char>(
            pixels[x][c] * (1.0 - kRoadOverlayAlpha) + (c == 2 ? 255 * kRoadOverlayAlpha : 0));
    }
  }
}

// Draw the mask first, keeping status, center points and end ROI unobscured.
void drawInto(const cv::Mat& frame, const RoadResult& result, double scale,
              cv::Mat& canvas) {
  if (scale >= 1.0) frame.copyTo(canvas);
  else cv::resize(frame, canvas,
      cv::Size(std::max(1, cvRound(frame.cols * scale)),
               std::max(1, cvRound(frame.rows * scale))), 0, 0, cv::INTER_AREA);
  const double sx = static_cast<double>(canvas.cols) / frame.cols;
  const double sy = static_cast<double>(canvas.rows) / frame.rows;
  drawRoadMask(result, canvas, sx, sy);
  const auto at = [sx, sy](const cv::Point2f& point) {
    return cv::Point(cvRound(point.x * sx), cvRound(point.y * sy));
  };
  const auto line = [&](const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (std::size_t i = 1; i < points.size(); ++i)
      cv::line(canvas, at(points[i - 1]), at(points[i]), color,
               std::max(1, cvRound(2 * sx)), cv::LINE_AA);
  };
  for (const auto& block : result.scan_blocks)
    if (block.found)
      cv::circle(canvas, at(block.midpoint), std::max(2, cvRound(4 * sx)),
                 {255, 0, 255}, -1, cv::LINE_AA);
  if (result.valid) {
    line(result.centerline, {0, 255, 0});
    for (const auto& point : result.centerline)
      cv::circle(canvas, at(point), std::max(2, cvRound(4 * sx)),
                 {0, 255, 255}, 1, cv::LINE_AA);
  }
  if (result.road_end_region.polygon.size() == 4) {
    std::vector<cv::Point> points;
    for (const auto& point : result.road_end_region.polygon) points.push_back(at(point));
    const cv::Scalar color = result.type == RoadType::RoadEnd ? cv::Scalar(0, 255, 0)
                                                           : cv::Scalar(0, 0, 255);
    cv::polylines(canvas, std::vector<std::vector<cv::Point>>{points}, true,
                  color, std::max(1, cvRound(2 * sx)), cv::LINE_AA);
  }
  const std::string status = std::string("ROAD: ") + toString(result.type);
  const double font = std::max(0.4, 0.7 * sx);
  const int thickness = std::max(1, cvRound(2 * sx));
  int baseline = 0;
  const cv::Size label = cv::getTextSize(status, cv::FONT_HERSHEY_SIMPLEX,
                                        font, thickness, &baseline);
  const cv::Point origin(std::max(2, cvRound(16 * sx)),
                          std::max(label.height + 2, cvRound(40 * sy)));
  const cv::Rect panel = cv::Rect(origin.x - 2, origin.y - label.height - 2,
      label.width + 4, label.height + baseline + 4) & cv::Rect(0, 0, canvas.cols, canvas.rows);
  cv::rectangle(canvas, panel, cv::Scalar::all(0), -1);
  cv::putText(canvas, status, origin, cv::FONT_HERSHEY_SIMPLEX,
              font, {255, 255, 255}, thickness, cv::LINE_AA);
}

}  // namespace

cv::Mat annotateResult(const cv::Mat& frame, const RoadResult& result, bool full_process) {
  cv::Mat output;
  drawInto(frame, result, 1.0, output);
  if (!full_process) return output;

  // Each panel has input-frame dimensions. Mask stays in processing/warped
  // coordinates; source overlays use inverse-transformed polygons and points.
  cv::Mat montage(frame.rows * 2, frame.cols * 2, frame.type(), cv::Scalar::all(0));
  frame.copyTo(montage(cv::Rect(0, 0, frame.cols, frame.rows)));
  if (!result.road_mask.empty()) {
    cv::Mat color_mask(result.road_mask.size(), CV_8UC3, cv::Scalar::all(0)), resized;
    color_mask.setTo(cv::Scalar(0, 200, 0), result.road_mask);
    if (!result.white_mask.empty()) color_mask.setTo(cv::Scalar(160, 160, 160), result.white_mask);
    if (!result.occlusion_mask.empty()) color_mask.setTo(cv::Scalar(180, 0, 180), result.occlusion_mask);
    if (!result.bridge_mask.empty()) color_mask.setTo(cv::Scalar(0, 165, 255), result.bridge_mask);
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
  const char* titles[] = {"INPUT", "ROAD: GREEN | WHITE: GRAY | OCCLUSION: PURPLE | BRIDGE: ORANGE", "BLOCKS / MEASURED CENTERS",
                          "CENTERLINE / ROAD END"};
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
    } else {
      drawInto(frame, result, scale_, canvas_);
    }
    cv::imshow(kPreviewWindow, canvas_);
    last_draw_ = std::chrono::steady_clock::now();
    drawn_once_ = true;
  }
  const int key = cv::waitKey(wait_ms);
  if (key == 's' || key == 'S') snapshot_requested_ = true;
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
