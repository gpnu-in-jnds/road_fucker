#include "road_finder/camera.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
  try {
    require(road_finder::isCameraSource("/dev/video0"), "/dev/video0 is a camera");
    require(road_finder::isCameraSource("/dev/video12"), "/dev/video12 is a camera");
    require(road_finder::isCameraSource("0"), "a camera index is a camera");
    require(road_finder::isCameraSource("12"), "a camera index is a camera");
    require(!road_finder::isCameraSource("/dev/video"), "a device path needs an index");
    require(!road_finder::isCameraSource("frame.png"), "an image is not a camera");

    require(road_finder::isImageSource("frame.png"), "png is an image");
    require(road_finder::isImageSource("FRAME.JPG"), "the extension is case insensitive");
    require(road_finder::isImageSource("tests/fixtures/frame_784.png"), "paths keep extensions");
    require(road_finder::isImageSource("a.b.jpeg"), "jpeg is an image");
    require(road_finder::isImageSource("frame.bmp"), "bmp is an image");
    require(road_finder::isImageSource("frame.webp"), "webp is an image");
    require(!road_finder::isImageSource(""), "an empty source is not an image");
    require(!road_finder::isImageSource("/tmp/frame.bar/baz"), "only trailing extensions count");

    require(!road_finder::isImageSource("clip.mp4"), "video files are not supported");
    require(!road_finder::isCameraSource("clip.mp4"), "video files are not supported");
    require(!road_finder::isImageSource("recording.avi"), "video files are not supported");
    require(!road_finder::isCameraSource("nvarguscamerasrc ! appsink"),
            "GStreamer pipelines are not supported");
    require(!road_finder::isImageSource("nvarguscamerasrc ! appsink"),
            "GStreamer pipelines are not supported");

    std::cout << "source tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "source tests failed: " << error.what() << '\n';
    return 1;
  }
}
