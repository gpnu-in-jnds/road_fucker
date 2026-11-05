#include "road_finder/camera.hpp"
#include "road_finder/preview.hpp"
#include "road_finder/road_detector.hpp"
#include "road_finder/uart_sender.hpp"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr const char* kDefaultSource = "/dev/video0";
constexpr const char* kDefaultConfig = "config/road_finder.yaml";
constexpr const char* kDefaultSerialDevice = "/dev/ttyS1";
constexpr double kDefaultCameraFps = 30.0;

struct Options {
  std::string source = kDefaultSource;
  std::string config = kDefaultConfig;
  std::string camera_fourcc;
  std::string serial_device = kDefaultSerialDevice;
  std::string overlay_path;
  std::string mask_path;
  double camera_fps = kDefaultCameraFps;
  double preview_scale = 1.0;  // < 1 shrinks the preview to cut display cost.
  double preview_fps = 0.0;    // > 0 redraws the preview at most that often.
  int threads = 1;             // Detector workers; ROI strips = threads * 6.
  int benchmark_frames = 0;    // > 0 exits after that many frames.
  bool display = false;
  bool full_process = false;
  bool json = false;
  bool serial_enabled = true;
  bool serial_device_explicit = false;
};

void usage() {
  std::cout << "road_fucker [--source /dev/video0|0|frame.png] [--config file] [--display] [--full-process] "
               "[--json] [--camera-fourcc MJPG] [--camera-fps 30] [--preview-scale 1.0] "
               "[--preview-fps 0] [--threads N] [--benchmark frames] [--overlay file] "
               "[--mask file] [--serial-device /dev/ttyS1|--no-serial]\n"
               "Defaults: --source "
            << kDefaultSource << ", --config " << kDefaultConfig
            << ", preview off, threads 1 (6 ROI strips), no JSON output, UART "
            << kDefaultSerialDevice << " at 115200 8N1 for camera sources.\n"
               "--preview-scale shrinks the preview window (0.5 is much cheaper to draw on "
               "weak GPUs); --preview-fps limits how often it is redrawn (detection keeps "
               "running every frame); --benchmark runs a fixed number of frames and prints "
               "per-stage timings. --threads N uses N detector workers and 6*N ROI strips. "
               "--full-process selects a four-panel process view for --display/--overlay. Press S in the camera preview to save raw/mask/overlay "
               "images under build/captures/.\n";
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

double parseNonNegativeDouble(const std::string& value, const char* flag) {
  try {
    std::size_t parsed = 0;
    const double result = std::stod(value, &parsed);
    if (parsed != value.size() || !std::isfinite(result) || result < 0.0)
      throw std::runtime_error("not a non-negative number");
    return result;
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " must be a non-negative number");
  }
}

int parsePositiveInt(const std::string& value, const char* flag) {
  try {
    std::size_t parsed = 0;
    const long result = std::stol(value, &parsed);
    if (parsed != value.size() || result <= 0)
      throw std::runtime_error("not a positive integer");
    if (result > 1000000L) throw std::runtime_error("integer too large");
    return static_cast<int>(result);
  } catch (const std::exception&) {
    throw std::runtime_error(std::string(flag) + " must be a positive integer");
  }
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") { usage(); std::exit(0); }
    else if (arg == "--display") options.display = true;
    else if (arg == "--full-process") options.full_process = true;
    else if (arg == "--no-display") options.display = false;
    else if (arg == "--json") options.json = true;
    else if (arg == "--no-json") options.json = false;      // Default; kept for old scripts.
    else if (arg == "--no-serial") options.serial_enabled = false;
    else if (arg == "--source" || arg == "--config" || arg == "--camera-fourcc" ||
             arg == "--camera-fps" || arg == "--preview-scale" || arg == "--preview-fps" ||
             arg == "--threads" || arg == "--benchmark" || arg == "--overlay" ||
             arg == "--mask" || arg == "--serial-device") {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + arg);
      const std::string value = argv[++i];
      if (arg == "--source") options.source = value;
      else if (arg == "--config") options.config = value;
      else if (arg == "--camera-fourcc") options.camera_fourcc = value;
      else if (arg == "--serial-device") {
        options.serial_device = value;
        options.serial_device_explicit = true;
        options.serial_enabled = true;
      }
      else if (arg == "--camera-fps") options.camera_fps = parsePositiveDouble(value, arg.c_str());
      else if (arg == "--preview-scale") {
        options.preview_scale = parsePositiveDouble(value, arg.c_str());
        if (options.preview_scale < 0.1 || options.preview_scale > 1.0)
          throw std::runtime_error("--preview-scale must be between 0.1 and 1.0");
      }
      else if (arg == "--preview-fps") options.preview_fps = parseNonNegativeDouble(value, arg.c_str());
      else if (arg == "--threads") options.threads = parsePositiveInt(value, arg.c_str());
      else if (arg == "--benchmark") options.benchmark_frames = parsePositiveInt(value, arg.c_str());
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
            << ",\"raw_type\":\"" << road_finder::toString(r.raw_type) << "\""
            << ",\"road_end_detected\":" << (r.road_end_detected ? "true" : "false")
            << ",\"road_end_coverage\":" << r.road_end_region.coverage
            << ",\"occlusion_detected\":" << (r.occlusion_detected ? "true" : "false")
            << ",\"classification_confidence\":" << r.classification_confidence
            << ",\"confidence\":" << r.confidence
            << ",\"raw_point_count\":" << r.raw_point_count
            << ",\"fit_point_count\":" << r.fit_point_count
            << ",\"centerline_filter_status\":\"" << r.centerline_filter_status << "\""
            << ",\"fit_residual_px\":" << r.fit_residual_px
            << ",\"lateral_error\":" << r.lateral_error
            << ",\"heading_error_rad\":" << r.heading_error_rad
            << ",\"left_boundary\":";
  printPoints(r.left_boundary);
  std::cout << ",\"right_boundary\":";
  printPoints(r.right_boundary);
  std::cout << ",\"centerline\":";
  printPoints(r.centerline);
  std::cout << ",\"scan_blocks\":[";
  for (std::size_t i = 0; i < r.scan_blocks.size(); ++i) {
    if (i) std::cout << ',';
    const auto& block = r.scan_blocks[i];
    std::cout << "{\"found\":" << (block.found ? "true" : "false")
              << ",\"fit_used\":" << (block.fit_used ? "true" : "false")
              << ",\"midpoint\":";
    if (block.found) std::cout << '[' << block.midpoint.x << ',' << block.midpoint.y << ']';
    else std::cout << "null";
    std::cout << '}';
  }
  std::cout << ']';
  std::cout << '}' << std::endl;
}

// Per-stage timings: on a slow board the interesting question is whether the
// camera, the detector or the preview owns the frame time.
class StageReport {
 public:
  void add(double read_ms, double detect_ms, double preview_ms) {
    read_ += read_ms;
    detect_ += detect_ms;
    preview_ += preview_ms;
    ++frames_;
  }

  int frames() const { return frames_; }

  // Prints one summary line once a second (or for the final report).
  void report(bool final_report) {
    const auto now = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now - window_start_).count();
    if (frames_ == 0) return;
    if (!final_report && seconds < 1.0) return;
    const double frames = frames_;
    std::ostringstream line;
    line << std::fixed << std::setprecision(1) << "fps " << frames / seconds
         << " | read " << read_ / frames << " ms | detect " << detect_ / frames
         << " ms | preview " << preview_ / frames << " ms | frames " << frames_;
    if (final_report) line << " | loop " << 1000.0 * seconds / frames << " ms/frame";
    std::cerr << line.str() << '\n';
    read_ = detect_ = preview_ = 0.0;
    frames_ = 0;
    window_start_ = now;
  }

 private:
  std::chrono::steady_clock::time_point window_start_ = std::chrono::steady_clock::now();
  double read_ = 0.0;
  double detect_ = 0.0;
  double preview_ = 0.0;
  int frames_ = 0;
};

double elapsedMs(const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
      .count();
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
std::unique_ptr<road_finder::Preview> makePreview(const Options& options, cv::Size size) {
  if (!options.display) return nullptr;
  return std::make_unique<road_finder::Preview>(size, options.preview_scale, options.preview_fps, options.full_process);
}

int runImage(const Options& options, road_finder::RoadDetector& detector,
             road_finder::UartSender* serial) {
  const cv::Mat frame = cv::imread(options.source, cv::IMREAD_COLOR);
  if (frame.empty()) throw std::runtime_error("cannot read image: " + options.source);
  const std::unique_ptr<road_finder::Preview> preview = makePreview(options, frame.size());
  if (options.benchmark_frames > 0) {
    // Repeats the same frame so detection throughput can be measured without a
    // camera; the preview is included when it is enabled.
    StageReport report;
    for (int i = 0; i < options.benchmark_frames; ++i) {
      const auto detect_start = std::chrono::steady_clock::now();
      const road_finder::RoadResult result = detector.process(frame, nowMilliseconds());
      if (serial) serial->send(result, frame.cols);
      const double detect_ms = elapsedMs(detect_start);
      double preview_ms = 0.0;
      if (preview) {
        const auto preview_start = std::chrono::steady_clock::now();
        preview->show(frame, result);
        preview_ms = elapsedMs(preview_start);
      }
      report.add(0.0, detect_ms, preview_ms);
    }
    report.report(/*final_report=*/true);
    return 0;
  }
  const road_finder::RoadResult result = detector.process(frame, nowMilliseconds());
  if (serial) serial->send(result, frame.cols);
  if (options.json) printResult(result);
  if (!options.overlay_path.empty() &&
      !cv::imwrite(options.overlay_path, road_finder::annotateResult(frame, result, options.full_process)))
    throw std::runtime_error("cannot write overlay: " + options.overlay_path);
  if (!options.mask_path.empty() && !cv::imwrite(options.mask_path, result.road_mask))
    throw std::runtime_error("cannot write mask: " + options.mask_path);
  if (preview) preview->show(frame, result, 0);  // Blocks until a key is pressed.
  return 0;
}

void saveSnapshot(const cv::Mat& frame, const road_finder::RoadResult& result, bool full_process) {
  const std::filesystem::path directory("build/captures");
  std::filesystem::create_directories(directory);
  const std::string prefix = (directory / std::to_string(result.timestamp_ms)).string();
  if (!cv::imwrite(prefix + "_raw.png", frame) ||
      !cv::imwrite(prefix + "_mask.png", result.road_mask) ||
      !cv::imwrite(prefix + "_overlay.png", road_finder::annotateResult(frame, result)))
    throw std::runtime_error("cannot write snapshot images");
  if (full_process && !cv::imwrite(prefix + "_process.png",
      road_finder::annotateResult(frame, result, true)))
    throw std::runtime_error("cannot write process snapshot");
  std::cerr << "snapshot saved: " << prefix << "_{raw,mask,overlay}.png"
            << " | type " << road_finder::toString(result.type)
            << " | raw_type " << road_finder::toString(result.raw_type)
            << " | geometry " << (result.valid ? "measured" : "unavailable") << '\n';
}

// The camera stream: read, detect, optionally print JSON, optionally show.
int runCamera(const Options& options, const road_finder::Config& config,
              road_finder::RoadDetector& detector, road_finder::UartSender* serial) {
  road_finder::Camera camera(options.source, {options.camera_fourcc, options.camera_fps}, config);
  const std::unique_ptr<road_finder::Preview> preview =
      makePreview(options, cv::Size(config.input_width, config.input_height));

  StageReport report;
  std::string stop_reason;
  cv::Mat frame;
  while (true) {
    auto start = std::chrono::steady_clock::now();
    camera.read(frame);
    const double read_ms = elapsedMs(start);
    start = std::chrono::steady_clock::now();
    const road_finder::RoadResult result = detector.process(frame, nowMilliseconds(), true);
    if (serial) serial->send(result, config.input_width);
    if (options.json) printResult(result);
    const double detect_ms = elapsedMs(start);
    double preview_ms = 0.0;
    if (preview) {
      start = std::chrono::steady_clock::now();
      const bool keep_running = preview->show(frame, result);
      preview_ms = elapsedMs(start);
      if (!keep_running) stop_reason = preview->stopReason();
      if (preview->takeSnapshotRequest()) {
        try {
          saveSnapshot(frame, result, options.full_process);
        } catch (const std::exception& error) {
          std::cerr << "snapshot failed: " << error.what() << '\n';
        }
      }
    }
    report.add(read_ms, detect_ms, preview_ms);
    if (!stop_reason.empty()) break;
    if (options.benchmark_frames > 0 && report.frames() >= options.benchmark_frames) {
      report.report(/*final_report=*/true);
      return 0;
    }
    report.report(/*final_report=*/false);
  }
  report.report(/*final_report=*/true);
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

    cv::setNumThreads(1);
    road_finder::Config config = road_finder::loadConfig(options.config);
    config.threads = options.threads;
    road_finder::RoadDetector detector(config);
    const bool use_serial = options.serial_enabled &&
                            (camera_source || options.serial_device_explicit);
    std::unique_ptr<road_finder::UartSender> serial;
    if (use_serial) serial = std::make_unique<road_finder::UartSender>(options.serial_device);
    logGeometry(config, detector);
    {
      std::ostringstream runtime;
      runtime << "runtime: segmentation " << "hsv"
              << ", RoadEnd >=2 retained points and >=80% non-mask (current frame)"
              << ", centerline filter " << (config.centerline_filter.enabled ? "on" : "off")
              << " (residual <= " << config.centerline_filter.max_residual_fraction * config.process_width << " px)"
              << ", threads " << config.threads << ", ROI blocks " << config.threads * 6;
      if (options.display)
        runtime << ", preview " << cvRound(config.input_width * options.preview_scale * (options.full_process ? 2 : 1)) << 'x'
                << cvRound(config.input_height * options.preview_scale * (options.full_process ? 2 : 1));
      else
        runtime << ", preview off";
      runtime << ", UART " << (serial ? options.serial_device : "off");
      if (options.preview_fps > 0.0) runtime << ", refresh " << options.preview_fps << " Hz";
      if (options.benchmark_frames > 0)
        runtime << ", benchmark " << options.benchmark_frames << " frames";
      std::cerr << runtime.str() << '\n';
    }
    return camera_source ? runCamera(options, config, detector, serial.get())
                         : runImage(options, detector, serial.get());
  } catch (const std::exception& error) {
    std::cerr << "road_fucker: " << error.what() << '\n';
    return 1;
  }
}
