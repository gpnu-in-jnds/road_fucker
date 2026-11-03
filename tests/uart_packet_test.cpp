#include "test_support.hpp"
#include "road_finder/uart_sender.hpp"

#include <initializer_list>
#include <iostream>
#include <limits>

namespace {
using namespace road_finder;
using road_finder::test::require;

void expect(const RoadResult& result, std::initializer_list<std::uint8_t> expected,
            const std::string& label, int input_width = 1280) {
  const auto packet = encodeRoadPacket(result, input_width);
  require(packet.size == expected.size(), label + ": wrong length");
  std::size_t i = 0;
  for (auto byte : expected) {
    require(packet.bytes[i] == byte, label + ": wrong byte " + std::to_string(i));
    ++i;
  }
}
}  // namespace

int main() {
  try {
    RoadResult result;
    expect(result, {0xFF, 0x7A, 0x02}, "Unknown");
    result.type = RoadType::Straight;
    expect(result, {0xFF, 0x7A, 0x02}, "Straight without geometry");
    result.valid = true;
    result.centerline = {{640, 600}, {640, 300}};
    expect(result, {0xFF, 0x7A, 0x00, 0, 0, 0, 0}, "centered Straight");
    result.centerline = {{740, 600}, {100, 100}};
    result.heading_error_rad = CV_PI / 180.0;
    expect(result, {0xFF, 0x7A, 0x00, 10, 0, 1, 0}, "positive bottommost offset");
    result.centerline = {{540, 600}};
    result.heading_error_rad = -CV_PI / 180.0;
    expect(result, {0xFF, 0x7A, 0x00, 0xF6, 0xFF, 0xFF, 0xFF}, "negative Straight");
    result.type = RoadType::RoadEnd;
    expect(result, {0xFF, 0x7A, 0x01, 0xF6, 0xFF, 0xFF, 0xFF}, "RoadEnd with geometry");
    result.valid = false;
    expect(result, {0xFF, 0x7A, 0x01, 0, 0, 0, 0}, "RoadEnd ignores stale measurements");
    result.centerline.clear();
    expect(result, {0xFF, 0x7A, 0x01, 0, 0, 0, 0}, "RoadEnd without geometry");
    result.type = RoadType::Unknown;
    result.valid = true;
    expect(result, {0xFF, 0x7A, 0x02}, "Unknown regardless of geometry");
    result.type = static_cast<RoadType>(255);
    expect(result, {0xFF, 0x7A, 0x02}, "unrecognized enum is Unknown");
    result.type = RoadType::Straight;
    result.centerline = {{1e7f, 10}};
    expect(result, {0xFF, 0x7A, 0, 0xFF, 0x7F, 0xFF, 0xFF}, "positive offset clamp");
    bool threw = false;
    try { (void)encodeRoadPacket(result, 0); } catch (const std::invalid_argument&) { threw = true; }
    require(threw, "zero input width must be rejected");
    result.centerline.clear();
    threw = false;
    try { (void)encodeRoadPacket(result, 1280); } catch (const std::invalid_argument&) { threw = true; }
    require(threw, "valid geometry must have a centerline");
    const auto close = road_finder::test::detect(road_finder::test::road(320));
    expect(close, {0xFF, 0x7A, 1, 0, 0, 0, 0}, "detected close end reaches UART", 640);
    std::cout << "UART packet tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "UART packet tests failed: " << error.what() << '\n';
    return 1;
  }
}
