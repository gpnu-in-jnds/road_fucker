#include "road_finder/camera.hpp"
#include "road_finder/preview.hpp"
#include "road_finder/road_detector.hpp"

#include <opencv2/imgcodecs.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr const char* kDefaultSource = "/dev/video0";
constexpr const char* kDefaultConfig = "config/road_finder.yaml";
constexpr double kDefaultCameraFps = 30.0;

struct Options {
  std::string source = kDefaultSource;
  std::string config = kDefaultConfig;
  std::string camera_fourcc;
  std::string overlay_path;
  std::string mask_path;
  double camera_fps = kDefaultCameraFps;
  bool display = true;
  bool json = false;
};

void usage() {
  std::cout << "road_finder [--source /dev/video0|0|frame.png] [--config file] [--no-display] "
               "[--json] [--camera-fourcc MJPG] [--camera-fps 30] [--overlay file] "
               "[--mask file]\n"
               "Defaults: --source "
            << kDefaultSource << ", --config " << kDefaultConfig
            << ", preview window open, no JSON output.\n";
}

double parsePositiveDouble(const std::string& value, const char* flag) {
  try {
    std::size_t parsed = 0;
    const double result = std::stod(value, &parsed);
    if (parsed != value.size() || !std::isfinite(result) || result <= 0.0)
      throw std::runtime_error("not a positive number");
    return result;
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " must be a positive number");
  }
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") { usage(); std::exit(0); }
    else if (arg == "--display") options.display = true;    // Default; kept for old scripts.
    else if (arg == "--no-display") options.display = false;
    else if (arg == "--json") options.json = true;
    else if (arg == "--no-json") options.json = false;      // Default; kept for old scripts.
    else if (arg == "--source" || arg == "--config" || arg == "--camera-fourcc" ||
             arg == "--camera-fps" || arg == "--overlay" || arg == "--mask") {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + arg);
      const std::string value = argv[++i];
      if (arg == "--source") options.source = value;
      else if (arg == "--config") options.config = value;
      else if (arg == "--camera-fourcc") options.camera_fourcc = value;
      else if (arg == "--camera-fps") options.camera_fps = parsePositiveDouble(value, arg.c_str());
      else if (arg == "--overlay") options.overlay_path = value;
      else options.mask_path = value;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (!options.camera_fourcc.empty() && options.camera_fourcc.size() != 4)
    throw std::runtime_error("--camera-fourcc must contain exactly four characters");
  return options;
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

// One line that says what the configuration resolved to, so a typo in
// process_width or roi shows up immediately instead of as missing detections.
void logGeometry(const road_finder::Config& config, const road_finder::RoadDetector& detector) {
  const cv::Rect2f& roi = detector.processRoi();
  std::cerr << "geometry: input " << config.input_width << 'x' << config.input_height
            << ", process " << config.process_width << 'x' << config.processHeight()
            << ", roi [" << config.roi.x << ',' << config.roi.y << ',' << config.roi.width << ','
            << config.roi.height << "] input px -> [" << roi.x << ',' << roi.y << ',' << roi.width
            << ',' << roi.height << "] "
            << (config.use_perspective ? "warped px" : "process px") << '\n';
}

std::int64_t nowMilliseconds() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// One image: detect, optionally export the overlay/mask, optionally show it.
int runImage(const Options& options, road_finder::RoadDetector& detector) {
  const cv::Mat frame = cv::imread(options.source, cv::IMREAD_COLOR);
  if (frame.empty()) throw std::runtime_error("cannot read image: " + options.source);
  const road_finder::RoadResult result = detector.process(frame, nowMilliseconds());
  if (options.json) printResult(result);
  if (!options.overlay_path.empty() &&
      !cv::imwrite(options.overlay_path, road_finder::annotateResult(frame, result)))
    throw std::runtime_error("cannot write overlay: " + options.overlay_path);
  if (!options.mask_path.empty() && !cv::imwrite(options.mask_path, result.road_mask))
    throw std::runtime_error("cannot write mask: " + options.mask_path);
  if (options.display) {
    road_finder::Preview preview(frame.size());
    preview.show(frame, result, 0);  // Blocks until a key is pressed.
  }
  return 0;
}

// The camera stream: read, detect, optionally print JSON, optionally show.
int runCamera(const Options& options, const road_finder::Config& config,
              road_finder::RoadDetector& detector) {
  road_finder::Camera camera(options.source, {options.camera_fourcc, options.camera_fps}, config);
  std::unique_ptr<road_finder::Preview> preview;
  if (options.display)
    preview = std::make_unique<road_finder::Preview>(
        cv::Size(config.input_width, config.input_height));

  std::string stop_reason;
  cv::Mat frame;
  while (true) {
    camera.read(frame);
    const auto start = std::chrono::steady_clock::now();
    const road_finder::RoadResult result = detector.process(frame, nowMilliseconds());
    if (options.json) printResult(result);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    std::cerr << "frame processing: " << elapsed << " ms\r";
    if (preview && !preview->show(frame, result)) {
      stop_reason = preview->stopReason();
      break;
    }
  }
  std::cerr << '\n';
  if (!stop_reason.empty()) std::cerr << stop_reason << '\n';
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    const bool camera_source = road_finder::isCameraSource(options.source);
    if (!camera_source && !road_finder::isImageSource(options.source))
      throw std::runtime_error("unsupported source: " + options.source +
                               "; use a camera index, /dev/videoN or an image file "
                               "(.png/.jpg/.jpeg/.bmp/.webp)");
    if (!camera_source && !options.camera_fourcc.empty())
      throw std::runtime_error("--camera-fourcc requires a camera index or /dev/videoN source");
    if (camera_source && (!options.overlay_path.empty() || !options.mask_path.empty()))
      throw std::runtime_error("--overlay and --mask are available for image sources only");

    const road_finder::Config config = road_finder::loadConfig(options.config);
    road_finder::RoadDetector detector(config);
    logGeometry(config, detector);
    return camera_source ? runCamera(options, config, detector) : runImage(options, detector);
  } catch (const std::exception& error) {
    std::cerr << "road_finder: " << error.what() << '\n';
    return 1;
  }
}
