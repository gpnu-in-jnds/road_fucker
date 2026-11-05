#include "road_finder/preview.hpp"
#include "test_support.hpp"

#include <opencv2/imgcodecs.hpp>

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

void expectMinimalOverlay() {
  using namespace road_finder;
  cv::Mat frame = cv::Mat::zeros(360, 640, CV_8UC3);
  RoadResult unknown;
  const auto unknown_view = annotateResult(frame, unknown);
  RoadResult end;
  end.type = RoadType::RoadEnd;
  const auto end_view = annotateResult(frame, end);
  require(cv::norm(unknown_view, end_view, cv::NORM_L1) > 1000.0,
          "RoadEnd without geometry must have its own status label");
  require(cv::countNonZero(end_view(cv::Rect(0,100,640,260)).reshape(1)) == 0,
          "classification alone must not draw fabricated geometry");
  end.road_end_region.polygon = {{200,270}, {440,270}, {440,330}, {200,330}};
  end.left_boundary = {{50,180}, {50,359}};
  end.right_boundary = {{590,180}, {590,359}};
  end.roi_polygon = {{0,180}, {639,180}, {639,359}, {0,359}};
  end.valid = true;
  end.centerline = {{320,200}, {320,350}};
  end.scan_blocks.resize(1);
  end.scan_blocks[0].found = true;
  end.scan_blocks[0].fit_used = true;
  end.scan_blocks[0].midpoint = {320,240};
  const auto normal = annotateResult(frame, end);
  require(normal.at<cv::Vec3b>(270,200)[1] > 200, "confirmed end box must be green");
  require(normal.at<cv::Vec3b>(200,50) == cv::Vec3b(0,0,0) &&
          normal.at<cv::Vec3b>(200,590) == cv::Vec3b(0,0,0),
          "left/right road boundaries must not appear");
  require(normal.at<cv::Vec3b>(180,100)[1] > 200,
          "tracking ROI must appear in green");
  require(cv::countNonZero(normal(cv::Rect(310,230,20,20)).reshape(1)) > 0,
          "measured/fitted center points must remain visible");
  end.type = RoadType::Straight;
  end.road_end_detected = true;
  require(annotateResult(frame, end).at<cv::Vec3b>(270,200)[2] > 200,
          "a non-end result has a red box without a pending-confirmation state");
  end.road_end_detected = false;
  require(annotateResult(frame, end).at<cv::Vec3b>(270,200)[2] > 200,
          "unsupported end box must be red");
  const auto montage = annotateResult(frame, end, true);
  require(montage.size() == cv::Size(1280,720), "full process must retain four panels");
  require(cv::norm(montage(cv::Rect(640,460,640,260)),
                   annotateResult(frame,end)(cv::Rect(0,100,640,260)), cv::NORM_INF) == 0,
          "final process panel must share the normal overlay");
  cv::imwrite("forward_overlay.png", normal);
  cv::imwrite("forward_process.png", montage);
}

void expectRoadMaskOverlay() {
  using namespace road_finder;
  const cv::Mat frame(360, 640, CV_8UC3, cv::Scalar(100, 100, 100));
  RoadResult result;
  result.road_mask = cv::Mat::zeros(180, 320, CV_8UC1);
  result.road_mask(cv::Rect(140, 100, 20, 50)).setTo(255);
  result.road_mask(cv::Rect(20, 120, 10, 10)).setTo(255);
  const cv::Mat original_mask = result.road_mask.clone();
  const auto output = annotateResult(frame, result);
  require(output.at<cv::Vec3b>(220, 290) == cv::Vec3b(65, 65, 154),
          "threshold pixels must show translucent red at the source resolution");
  require(output.at<cv::Vec3b>(250, 50) == cv::Vec3b(65, 65, 154),
          "disconnected threshold candidates must remain visible without geometry");
  require(output.at<cv::Vec3b>(220, 350) == cv::Vec3b(100, 100, 100),
          "pixels outside the binary mask must remain unchanged");
  require(frame.at<cv::Vec3b>(220, 290) == cv::Vec3b(100, 100, 100) &&
          cv::norm(result.road_mask, original_mask, cv::NORM_INF) == 0,
          "overlay rendering must not modify detector input or its mask");
  result.road_mask_to_source = cv::Mat::eye(3, 3, CV_64F);
  result.road_mask_to_source.at<double>(0, 0) = 2;
  result.road_mask_to_source.at<double>(1, 1) = 2;
  result.road_mask_to_source.at<double>(0, 2) = -100;
  const auto mapped = annotateResult(frame, result);
  require(mapped.at<cv::Vec3b>(220, 190) == cv::Vec3b(65, 65, 154) &&
          mapped.at<cv::Vec3b>(220, 290) == cv::Vec3b(100, 100, 100),
          "warped mask must map back onto the original road, without a shifted ghost");
  cv::imwrite("threshold_overlay.png", output);
}

void expectRejectedCenters() {
  using namespace road_finder;
  const cv::Mat frame(360, 640, CV_8UC3, cv::Scalar::all(40));
  RoadResult result;
  result.valid = true;
  result.type = RoadType::Straight;
  result.road_mask = cv::Mat(360, 640, CV_8UC1, cv::Scalar(255));
  result.centerline = {{320,180}, {320,320}};
  result.road_end_region.polygon = {{200,240}, {440,240}, {440,330}, {200,330}};
  result.scan_blocks.resize(3);
  result.scan_blocks[0].found = result.scan_blocks[0].fit_used = true;
  result.scan_blocks[0].midpoint = {240,280};
  result.scan_blocks[1].found = true;
  result.scan_blocks[1].midpoint = {320,240};  // Overlaps both line and end-box edge.
  result.scan_blocks[2].midpoint = {480,280};  // Missing observations must not be drawn.
  for (double scale : {1.0, 0.5, 0.25}) {
    const auto normal = annotateResult(frame, result, false, scale);
    const auto montage = annotateResult(frame, result, true, scale);
    const int x = cvRound(320 * scale), y = cvRound(240 * scale);
    require(normal.at<cv::Vec3b>(y,x) == cv::Vec3b(0,0,0),
            "rejected black center must cover road tint, green line and end-box edge at every scale");
    const int radius = std::max(2, cvRound(4 * scale));
    const auto outline = normal.at<cv::Vec3b>(y, x + radius + 1);
    require(outline[0] > 150 && outline[1] > 150 && outline[2] > 150,
            "black points need a light outline on dark backgrounds");
    require(normal.at<cv::Vec3b>(cvRound(280 * scale), cvRound(240 * scale)) == cv::Vec3b(255,0,255),
            "retained centers remain purple");
    require(normal.at<cv::Vec3b>(cvRound(280 * scale), cvRound(480 * scale)) != cv::Vec3b(0,0,0),
            "missing observations must not appear as black points");
    require(montage.at<cv::Vec3b>(normal.rows + y,x) == cv::Vec3b(0,0,0) &&
            montage.at<cv::Vec3b>(normal.rows + y,normal.cols + x) == cv::Vec3b(0,0,0),
            "both measurement and final process panels must show rejected points");
    const cv::Rect below_title(0, cvRound(100 * scale), normal.cols, normal.rows - cvRound(100 * scale));
    require(cv::norm(normal(below_title),
                     montage(below_title + cv::Point(normal.cols, normal.rows)), cv::NORM_INF) == 0,
            "scaled final montage must share normal preview rendering");
  }
  result.valid = false;
  result.type = RoadType::Unknown;
  result.scan_blocks[0].fit_used = false;
  const auto unknown = annotateResult(frame, result);
  require(unknown.at<cv::Vec3b>(240,320) == cv::Vec3b(0,0,0) &&
          unknown.at<cv::Vec3b>(280,240) == cv::Vec3b(0,0,0),
          "Unknown must still show all excluded observations");
}

void expectRoiOverlay() {
  using namespace road_finder;
  const cv::Mat frame(360, 640, CV_8UC3, cv::Scalar::all(80));
  RoadResult result;
  // Non-axis-aligned source polygon, as returned after inverse perspective mapping.
  result.roi_polygon = {{80,120}, {560,160}, {520,320}, {120,280}};
  result.scan_blocks.resize(1);
  result.scan_blocks[0].found = true;
  result.scan_blocks[0].midpoint = {320,140};
  for (double scale : {1.0, 0.5}) {
    const auto output = annotateResult(frame, result, false, scale);
    const auto process = annotateResult(frame, result, true, scale);
    const int x = cvRound(200 * scale), y = cvRound(130 * scale);
    require(output.at<cv::Vec3b>(y,x)[1] > 200 &&
            process.at<cv::Vec3b>(y + output.rows,x)[1] > 200 &&
            process.at<cv::Vec3b>(y + output.rows,x + output.cols)[1] > 200,
            "source ROI polygon must align in normal, block and final panels at preview scale");
    require(output.at<cv::Vec3b>(cvRound(140 * scale),cvRound(320 * scale)) == cv::Vec3b(0,0,0),
            "excluded black point must remain on top of green ROI");
  }
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
    expectMinimalOverlay();
    expectRoadMaskOverlay();
    expectRejectedCenters();
    expectRoiOverlay();
    std::cout << "preview window tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "preview window tests failed: " << error.what() << '\n';
    return 1;
  }
}
