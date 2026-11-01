#include "road_finder/uart_sender.hpp"

#include <opencv2/imgcodecs.hpp>

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
                 {0xFF, 0x7A, 0x03, 0x00, 0x00, 0x00, 0x00},
                 "recognized T without measured steering geometry");

    for (const auto type : {road_finder::RoadType::LEFT_TURN,
                            road_finder::RoadType::RIGHT_TURN,
                            road_finder::RoadType::T_JUNCTION}) {
      t.type = type;
      // Invalid geometry must ignore even leftover nonzero measurements.
      t.centerline = {{740.0f, 600.0f}};
      t.heading_error_rad = -0.5f;
      const auto wire_type = static_cast<std::uint8_t>(
          type == road_finder::RoadType::LEFT_TURN ? 1 :
          (type == road_finder::RoadType::RIGHT_TURN ? 2 : 3));
      expectPacket(road_finder::encodeRoadPacket(t, 1280),
                   {0xFF, 0x7A, wire_type, 0x00, 0x00, 0x00, 0x00},
                   "junction type survives unavailable geometry with zero fields");
    }
    t.type = road_finder::RoadType::STRAIGHT;
    expectPacket(road_finder::encodeRoadPacket(t, 1280),
                 {0xFF, 0x7A, 0x04}, "straight still requires measured geometry");

    const cv::Mat captured = cv::imread(ROAD_FUCKER_NEAR_T_IMAGE, cv::IMREAD_COLOR);
    require(!captured.empty(), "captured near T image is missing");
    for (int process_width : {320, 640}) {
      road_finder::Config config;
      config.input_width = captured.cols;
      config.input_height = captured.rows;
      config.process_width = process_width;
      config.roi = {0, captured.rows / 2, captured.cols, captured.rows - captured.rows / 2};
      config.segmentation_mode = "adaptive_dark";
      road_finder::RoadDetector detector(config);
      const auto result = detector.process(captured, 100);
      require(result.type == road_finder::RoadType::T_JUNCTION && !result.valid &&
                  result.centerline.empty(), "captured T must reproduce missing geometry");
      expectPacket(road_finder::encodeRoadPacket(result, captured.cols),
                   {0xFF, 0x7A, 0x03, 0x00, 0x00, 0x00, 0x00},
                   "captured T must reach STM32 as type 03");
    }

    road_finder::RoadResult unknown;
    expectPacket(road_finder::encodeRoadPacket(unknown, 1280),
                 {0xFF, 0x7A, 0x04}, "unknown");
    for (bool valid : {false, true}) {
      unknown.valid = valid;
      for (const auto type : {road_finder::RoadType::UNKNOWN,
                              road_finder::RoadType::LEFT_BRANCH,
                              road_finder::RoadType::RIGHT_BRANCH}) {
        unknown.type = type;
        expectPacket(road_finder::encodeRoadPacket(unknown, 1280),
                     {0xFF, 0x7A, 0x04}, "unknown and unsupported types");
      }
    }

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
