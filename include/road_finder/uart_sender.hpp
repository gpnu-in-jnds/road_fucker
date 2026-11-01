#pragma once

#include "road_finder/road_detector.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace road_finder {

struct RoadPacket {
  std::array<std::uint8_t, 7> bytes{};
  std::size_t size = 0;
};

// Encodes the fixed STM32 XDK42 contract. input_width is used to convert the
// fitted source-image center to the sample's 128-pixel reference coordinates.
// Known left/right/T junctions without geometry keep their type and use zero
// offset/angle. Straight still requires geometry; UNKNOWN remains three bytes.
RoadPacket encodeRoadPacket(const RoadResult& result, int input_width);

class UartSender {
 public:
  explicit UartSender(const std::string& device);
  ~UartSender();

  UartSender(const UartSender&) = delete;
  UartSender& operator=(const UartSender&) = delete;

  void send(const RoadResult& result, int input_width);
  const std::string& device() const { return device_; }

 private:
  int fd_ = -1;
  std::string device_;
};

}  // namespace road_finder
