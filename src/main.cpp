#include "road_finder/road_detector.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <array>
#include <cctype>
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
  bool display = false;
};

void usage() {
  std::cout << "road_finder --source <camera index|video|image|GStreamer pipeline> "
               "[--config file] [--display] [--overlay file] [--mask file]\n";
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") { usage(); std::exit(0); }
    if (arg == "--display") { options.display = true; continue; }
    if (i + 1 >= argc) throw std::runtime_error("missing value after " + arg);
    const std::string value = argv[++i];
    if (arg == "--source") options.source = value;
    else if (arg == "--config") options.config = value;
    else if (arg == "--overlay") options.overlay_path = value;
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

cv::Mat annotate(const cv::Mat& frame, const road_finder::RoadResult& result) {
  cv::Mat output = frame.clone();
  const auto draw = [&](const std::vector<cv::Point2f>& points, const cv::Scalar& color) {
    for (std::size_t i = 1; i < points.size(); ++i)
      cv::line(output, points[i - 1], points[i], color, 3, cv::LINE_AA);
  };
  draw(result.left_boundary, {0, 255, 255});
  draw(result.right_boundary, {0, 255, 255});
  draw(result.centerline, {0, 255, 0});
  const std::string label = std::string(road_finder::toString(result.type)) +
                            " confidence=" + cv::format("%.2f", result.confidence);
  cv::putText(output, label, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
              {0, 0, 255}, 2, cv::LINE_AA);
  return output;
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
      printResult(result);
      if (!options.overlay_path.empty() &&
          !cv::imwrite(options.overlay_path, annotate(frame, result)))
        throw std::runtime_error("cannot write overlay: " + options.overlay_path);
      if (!options.mask_path.empty() && !cv::imwrite(options.mask_path, result.road_mask))
        throw std::runtime_error("cannot write mask: " + options.mask_path);
      if (options.display) {
        cv::imshow("road_finder", annotate(frame, result));
        cv::waitKey(0);
      }
      return 0;
    }
    if (!options.overlay_path.empty() || !options.mask_path.empty())
      throw std::runtime_error("--overlay and --mask are available for image sources only");
    cv::VideoCapture capture;
    if (isCameraIndex(options.source)) {
      capture.open(std::stoi(options.source), cv::CAP_ANY);
      capture.set(cv::CAP_PROP_FRAME_WIDTH, config.input_width);
      capture.set(cv::CAP_PROP_FRAME_HEIGHT, config.input_height);
    } else if (options.source.find('!') != std::string::npos) {
      capture.open(options.source, cv::CAP_GSTREAMER);
    } else {
      capture.open(options.source, cv::CAP_ANY);
    }
    if (!capture.isOpened()) throw std::runtime_error("cannot open source: " + options.source);
    cv::Mat frame;
    while (capture.read(frame)) {
      const auto start = std::chrono::steady_clock::now();
      const auto result = detector.process(frame, nowMilliseconds());
      printResult(result);
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start).count();
      std::cerr << "frame processing: " << elapsed << " ms\r";
      if (options.display) {
        cv::imshow("road_finder", annotate(frame, result));
        if (cv::waitKey(1) == 27) break;
      }
    }
    std::cerr << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "road_finder: " << error.what() << '\n';
    return 1;
  }
}
