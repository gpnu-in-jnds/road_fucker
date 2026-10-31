#include "road_finder/uart_sender.hpp"

#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void expectPacket(const road_finder::RoadPacket& packet,
                  std::initializer_list<std::uint8_t> expected,
                  const std::string& label) {
  require(packet.size == expected.size(), label + ": wrong packet length");
  std::size_t i = 0;
  for (const std::uint8_t byte : expected) {
    require(packet.bytes[i] == byte, label + ": byte mismatch at index " +
                                        std::to_string(i));
    ++i;
  }
}

}  // namespace

int main() {
  try {
    road_finder::RoadResult straight;
    straight.valid = true;
    straight.type = road_finder::RoadType::STRAIGHT;
    straight.centerline = {{640.0f, 600.0f}, {640.0f, 300.0f}};
    expectPacket(road_finder::encodeRoadPacket(straight, 1280),
                 {0xFF, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00},
                 "centered straight");

    straight.centerline = {{740.0f, 600.0f}};
    straight.heading_error_rad = 1.0f * CV_PI / 180.0f;
    expectPacket(road_finder::encodeRoadPacket(straight, 1280),
                 {0xFF, 0x7A, 0x00, 0x0A, 0x00, 0x01, 0x00},
                 "positive offset and angle");

    straight.centerline = {{540.0f, 600.0f}};
    straight.heading_error_rad = -1.0f * CV_PI / 180.0f;
    expectPacket(road_finder::encodeRoadPacket(straight, 1280),
                 {0xFF, 0x7A, 0x00, 0xF6, 0xFF, 0xFF, 0xFF},
                 "negative offset and angle");

    road_finder::RoadResult left;
    left.valid = true;
    left.type = road_finder::RoadType::LEFT_TURN;
    left.centerline = {{640.0f, 600.0f}};
    expectPacket(road_finder::encodeRoadPacket(left, 1280),
                 {0xFF, 0x7A, 0x01, 0x00, 0x00, 0x00, 0x00},
                 "left turn");

    road_finder::RoadResult t;
    t.valid = true;
    t.type = road_finder::RoadType::T_JUNCTION;
    t.centerline = {{640.0f, 600.0f}};
    expectPacket(road_finder::encodeRoadPacket(t, 1280),
                 {0xFF, 0x7A, 0x03, 0x00, 0x00, 0x00, 0x00},
                 "T junction");
    t.valid = false;
    t.centerline.clear();
    expectPacket(road_finder::encodeRoadPacket(t, 1280),
                 {0xFF, 0x7A, 0x04}, "recognized junction without measured steering geometry");

    road_finder::RoadResult unknown;
    expectPacket(road_finder::encodeRoadPacket(unknown, 1280),
                 {0xFF, 0x7A, 0x04}, "unknown");
    unknown.valid = true;
    unknown.type = road_finder::RoadType::RIGHT_BRANCH;
    expectPacket(road_finder::encodeRoadPacket(unknown, 1280),
                 {0xFF, 0x7A, 0x04}, "unsupported branch type");

    bool threw = false;
    try {
      (void)road_finder::encodeRoadPacket(straight, 0);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    require(threw, "serializer must reject zero input width");

    std::cout << "UART packet tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "UART packet tests failed: " << error.what() << '\n';
    return 1;
  }
}
