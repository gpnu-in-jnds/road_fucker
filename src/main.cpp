#include "road_finder/road_detector.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

struct Options {
  std::string source = "0";
  std::string config = "config/road_finder.yaml";
  std::string overlay_path;
  std::string mask_path;
  std::string camera_fourcc;
  double camera_fps = 30.0;
  bool display = false;
  bool no_json = false;
};

constexpr const char* kPreviewWindow = "road_finder preview";

void usage() {
  std::cout << "road_finder --source <camera index|video|image|GStreamer pipeline> "
               "[--config file] [--display] [--no-json] [--camera-fourcc MJPG] "
               "[--camera-fps 30] "
               "[--overlay file] [--mask file]\n";
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") { usage(); std::exit(0); }
    if (arg == "--display") { options.display = true; continue; }
    if (arg == "--no-json") { options.no_json = true; continue; }
    if (i + 1 >= argc) throw std::runtime_error("missing value after " + arg);
    const std::string value = argv[++i];
    if (arg == "--source") options.source = value;
    else if (arg == "--config") options.config = value;
    else if (arg == "--camera-fourcc") options.camera_fourcc = value;
    else if (arg == "--camera-fps") {
      std::size_t parsed = 0;
      options.camera_fps = std::stod(value, &parsed);
      if (parsed != value.size() || !std::isfinite(options.camera_fps) ||
          options.camera_fps <= 0.0)
        throw std::runtime_error("--camera-fps must be a positive number");
    } else if (arg == "--overlay") options.overlay_path = value;
    else if (arg == "--mask") options.mask_path = value;
    else throw std::runtime_error("unknown argument: " + arg);
  }
  return options;
}

bool isImage(const std::string& source) {
  const auto dot = source.find_last_of('.');
  if (dot == std::string::npos) return false;
  std::string extension = source.substr(dot);
  for (char& ch : extension) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  return extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
         extension == ".bmp" || extension == ".webp";
}

bool isCameraIndex(const std::string& source) {
  return !source.empty() &&
         source.find_first_not_of("0123456789") == std::string::npos;
}

bool isVideoDevice(const std::string& source) {
  constexpr const char* prefix = "/dev/video";
  const std::string suffix = source.rfind(prefix, 0) == 0 ? source.substr(10) : "";
  return !suffix.empty() && suffix.find_first_not_of("0123456789") == std::string::npos;
}

cv::Mat prepareVideoFrame(const cv::Mat& frame, const road_finder::Config& config,
                          bool camera_source, bool& resize_reported) {
  if (frame.empty()) throw std::runtime_error("video source returned an empty frame");
  cv::Mat bgr;
  if (frame.depth() != CV_8U)
    throw std::runtime_error("video source must return 8-bit frames");
  if (frame.channels() == 3) bgr = frame;
  else if (frame.channels() == 1) cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
  else if (frame.channels() == 4) cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
  else
    throw std::runtime_error("unsupported camera frame: " +
                             std::to_string(frame.channels()) + " channels");

  const cv::Size expected(config.input_width, config.input_height);
  if (bgr.size() == expected) return bgr;
  const std::string actual = std::to_string(bgr.cols) + "x" + std::to_string(bgr.rows);
  const std::string wanted = std::to_string(expected.width) + "x" +
                             std::to_string(expected.height);
  const double aspect_error = std::abs(static_cast<double>(bgr.cols) / bgr.rows -
                                       static_cast<double>(expected.width) / expected.height);
  if (!camera_source || aspect_error > 0.01) {
    const std::string advice = camera_source
        ? "; select a 16:9 camera mode with v4l2-ctl or change the input size in config"
        : "; change the input size in config";
    throw std::runtime_error("source delivered " + actual + ", expected " + wanted + advice);
  }
  if (!resize_reported) {
    std::cerr << "camera delivered " << actual << "; resizing to " << wanted
              << " for detection. Calibrate against the actual camera resolution.\n";
    resize_reported = true;
  }
  cv::Mat resized;
  cv::resize(bgr, resized, expected, 0, 0, cv::INTER_LINEAR);
  return resized;
}

void printResult(const road_finder::RoadResult& r) {
  const auto printPoints = [](const std::vector<cv::Point2f>& points) {
    std::cout << '[';
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << '[' << points[i].x << ',' << points[i].y << ']';
    }
    std::cout << ']';
  };
  std::cout << std::fixed << std::setprecision(4)
            << "{\"timestamp_ms\":" << r.timestamp_ms
            << ",\"valid\":" << (r.valid ? "true" : "false")
            << ",\"type\":\"" << road_finder::toString(r.type) << "\""
            << ",\"directions\":[";
  bool first = true;
  const std::array<std::pair<unsigned, const char*>, 3> directions{{
      {road_finder::LEFT, "left"},
      {road_finder::STRAIGHT, "straight"},
      {road_finder::RIGHT, "right"}}};
  for (const auto& item : directions) {
    if (r.available_directions & item.first) {
      if (!first) std::cout << ',';
      std::cout << '"' << item.second << '"';
      first = false;
    }
  }
  std::cout << "]"
            << ",\"confidence\":" << r.confidence
            << ",\"lateral_error\":" << r.lateral_error
            << ",\"heading_error_rad\":" << r.heading_error_rad
            << ",\"left_boundary\":";
  printPoints(r.left_boundary);
  std::cout << ",\"right_boundary\":";
  printPoints(r.right_boundary);
  std::cout << ",\"centerline\":";
  printPoints(r.centerline);
  std::cout << '}' << std::endl;
}

const char* roadCondition(const road_finder::RoadResult& result) {
  if (!result.valid) return "LOST";
  switch (result.type) {
    case road_finder::RoadType::STRAIGHT: return "STRAIGHT";
    case road_finder::RoadType::LEFT_TURN: return "LEFT TURN";
    case road_finder::RoadType::RIGHT_TURN: return "RIGHT TURN";
    case road_finder::RoadType::LEFT_BRANCH: return "LEFT BRANCH";
    case road_finder::RoadType::RIGHT_BRANCH: return "RIGHT BRANCH";
    case road_finder::RoadType::T_JUNCTION: return "T JUNCTION";
    case road_finder::RoadType::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

class DisplayFps {
 public:
  void frameCompleted() {
    ++frames_;
    const auto now = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now - window_start_).count();
    if (seconds >= 1.0) {
      fps_ = frames_ / seconds;
      frames_ = 0;
      window_start_ = now;
    }
  }

  double value() const { return fps_; }

 private:
  std::chrono::steady_clock::time_point window_start_ = std::chrono::steady_clock::now();
  int frames_ = 0;
  double fps_ = -1.0;
};

cv::Mat annotate(const cv::Mat& frame, const road_finder::RoadResult& result,
                 double fps = -1.0, bool show_fps = false) {
  cv::Mat output = frame.clone();
  const int panel_height = show_fps ? 120 : 88;
  const cv::Rect panel(12, 12, 440, panel_height);
  cv::Mat panel_pixels = output(panel);
  cv::Mat black(panel.size(), output.type(), cv::Scalar::all(0));
  cv::addWeighted(panel_pixels, 0.4, black, 0.6, 0.0, panel_pixels);
  const cv::Scalar white(255, 255, 255);
  cv::putText(output, std::string("ROAD: ") + roadCondition(result), {24, 43},
              cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2, cv::LINE_AA);
  const std::string confidence = result.valid ? cv::format("%.2f", result.confidence)
                                               : "--";
  cv::putText(output, "CONFIDENCE: " + confidence, {24, 76},
              cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2, cv::LINE_AA);
  if (show_fps) {
    const std::string fps_label = fps < 0.0 ? "MEASURING" : cv::format("%.1f", fps);
    cv::putText(output, "FPS: " + fps_label, {24, 109},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2, cv::LINE_AA);
  }
  const auto draw = [&](const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (std::size_t i = 1; i < points.size(); ++i)
      cv::line(output, points[i - 1], points[i], color, 3, cv::LINE_AA);
  };
  if (result.valid) {
    draw(result.left_boundary, {255, 255, 0});  // Cyan in BGR.
    draw(result.right_boundary, {0, 165, 255}); // Orange in BGR.
    draw(result.centerline, {0, 255, 0});
  }
  return output;
}

void openPreviewWindow(const cv::Size& image_size) {
  cv::namedWindow(kPreviewWindow, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO |
                                     cv::WINDOW_GUI_NORMAL);
  cv::resizeWindow(kPreviewWindow, image_size);
}

std::int64_t nowMilliseconds() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    const auto config = road_finder::loadConfig(options.config);
    road_finder::RoadDetector detector(config);
    if (isImage(options.source)) {
      const cv::Mat frame = cv::imread(options.source, cv::IMREAD_COLOR);
      if (frame.empty()) throw std::runtime_error("cannot read image: " + options.source);
      const auto result = detector.process(frame, nowMilliseconds());
      if (!options.no_json) printResult(result);
      if (!options.overlay_path.empty() &&
          !cv::imwrite(options.overlay_path, annotate(frame, result)))
        throw std::runtime_error("cannot write overlay: " + options.overlay_path);
      if (!options.mask_path.empty() && !cv::imwrite(options.mask_path, result.road_mask))
        throw std::runtime_error("cannot write mask: " + options.mask_path);
      if (options.display) {
        openPreviewWindow(frame.size());
        cv::imshow(kPreviewWindow, annotate(frame, result));
        cv::waitKey(0);
      }
      return 0;
    }
    if (!options.overlay_path.empty() || !options.mask_path.empty())
      throw std::runtime_error("--overlay and --mask are available for image sources only");
    const bool camera_source = isCameraIndex(options.source) || isVideoDevice(options.source);
    if (!options.camera_fourcc.empty() && !camera_source)
      throw std::runtime_error("--camera-fourcc requires a camera index or /dev/videoN source");
    if (!options.camera_fourcc.empty() && options.camera_fourcc.size() != 4)
      throw std::runtime_error("--camera-fourcc must contain exactly four characters");
    cv::VideoCapture capture;
    if (isCameraIndex(options.source)) {
      capture.open(std::stoi(options.source), cv::CAP_ANY);
    } else if (isVideoDevice(options.source)) {
      const int device_index = std::stoi(options.source.substr(10));
      capture.open(device_index, cv::CAP_V4L2);
    } else if (options.source.find('!') != std::string::npos) {
      capture.open(options.source, cv::CAP_GSTREAMER);
    } else {
      capture.open(options.source, cv::CAP_ANY);
    }
    if (!capture.isOpened())
      throw std::runtime_error("cannot open source: " + options.source +
                               (isVideoDevice(options.source) ? " with V4L2" : ""));
    if (camera_source) {
      // UVC cameras often provide 720p at full frame rate only in MJPG mode.
      const std::string code = options.camera_fourcc.empty() && isVideoDevice(options.source)
                                   ? "MJPG" : options.camera_fourcc;
      if (!code.empty()) {
        if (!capture.set(cv::CAP_PROP_FOURCC,
                         cv::VideoWriter::fourcc(code[0], code[1], code[2], code[3])))
          std::cerr << "camera did not accept FOURCC " << code << '\n';
      }
      capture.set(cv::CAP_PROP_FRAME_WIDTH, config.input_width);
      capture.set(cv::CAP_PROP_FRAME_HEIGHT, config.input_height);
      if (!capture.set(cv::CAP_PROP_FPS, options.camera_fps))
        std::cerr << "camera did not accept requested " << options.camera_fps << " FPS\n";
      capture.set(cv::CAP_PROP_CONVERT_RGB, 1);
      std::cerr << "camera requested " << config.input_width << 'x' << config.input_height
                << ", negotiated " << static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH))
                << 'x' << static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT))
                << " at " << capture.get(cv::CAP_PROP_FPS) << " FPS via "
                << capture.getBackendName() << '\n';
    }
    if (options.display) openPreviewWindow({config.input_width, config.input_height});
    DisplayFps display_fps;
    cv::Mat frame;
    bool resize_reported = false;
    bool received_frame = false;
    while (true) {
      if (!capture.read(frame) || frame.empty()) {
        if (camera_source)
          throw std::runtime_error("camera did not deliver a frame from " + options.source +
                                   "; test the device with v4l2-ctl -d " + options.source +
                                   " --stream-mmap --stream-count=10");
        if (!received_frame) throw std::runtime_error("video source contains no frames");
        break;
      }
      received_frame = true;
      const auto start = std::chrono::steady_clock::now();
      const cv::Mat bgr = prepareVideoFrame(frame, config, camera_source, resize_reported);
      const auto result = detector.process(bgr, nowMilliseconds());
      if (!options.no_json) printResult(result);
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start).count();
      std::cerr << "frame processing: " << elapsed << " ms\r";
      if (options.display) {
        cv::imshow(kPreviewWindow, annotate(bgr, result, display_fps.value(), true));
        const int key = cv::waitKey(1);
        display_fps.frameCompleted();
        if (key == 27 || key == 'q' || key == 'Q') break;
        try {
          if (cv::getWindowProperty(kPreviewWindow, cv::WND_PROP_VISIBLE) < 1) break;
        } catch (const cv::Exception&) {
          break; // Some GUI backends throw after the window is closed.
        }
      }
    }
    std::cerr << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "road_finder: " << error.what() << '\n';
    return 1;
  }
}
