#include "road_finder/uart_sender.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <system_error>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace road_finder {
namespace {

constexpr double kReferenceWidth = 128.0;
constexpr double kRadiansToDegrees = 180.0 / CV_PI;

std::uint8_t wireType(const RoadResult& result) {
  if (!result.valid) return 0x04;
  switch (result.type) {
    case RoadType::STRAIGHT: return 0x00;
    case RoadType::LEFT_TURN: return 0x01;
    case RoadType::RIGHT_TURN: return 0x02;
    case RoadType::T_JUNCTION: return 0x03;
    case RoadType::UNKNOWN:
    case RoadType::LEFT_BRANCH:
    case RoadType::RIGHT_BRANCH:
      return 0x04;
  }
  return 0x04;
}

std::int16_t boundedInt16(long value) {
  return static_cast<std::int16_t>(
      std::clamp(value,
                 static_cast<long>(std::numeric_limits<std::int16_t>::lowest()),
                 static_cast<long>(std::numeric_limits<std::int16_t>::max())));
}

void appendInt16(std::array<std::uint8_t, 7>& bytes, std::size_t offset,
                 std::int16_t value) {
  const std::uint16_t bits = static_cast<std::uint16_t>(value);
  bytes[offset] = static_cast<std::uint8_t>(bits & 0xFFU);
  bytes[offset + 1] = static_cast<std::uint8_t>((bits >> 8) & 0xFFU);
}

}  // namespace

RoadPacket encodeRoadPacket(const RoadResult& result, int input_width) {
  if (input_width <= 0) throw std::invalid_argument("input_width must be positive");
  RoadPacket packet;
  packet.bytes[0] = 0xFF;
  packet.bytes[1] = 0x7A;
  packet.bytes[2] = wireType(result);
  if (packet.bytes[2] == 0x04) {
    packet.size = 3;
    return packet;
  }

  if (result.centerline.empty())
    throw std::invalid_argument("valid road result has no centerline for UART offset");
  const auto bottom = std::max_element(
      result.centerline.begin(), result.centerline.end(),
      [](const cv::Point2f& lhs, const cv::Point2f& rhs) { return lhs.y < rhs.y; });
  const double center_x_reference = bottom->x * kReferenceWidth / input_width;
  const auto offset = boundedInt16(std::lround(center_x_reference - kReferenceWidth / 2.0));
  const auto angle = boundedInt16(std::lround(result.heading_error_rad * kRadiansToDegrees));
  appendInt16(packet.bytes, 3, offset);
  appendInt16(packet.bytes, 5, angle);
  packet.size = packet.bytes.size();
  return packet;
}

UartSender::UartSender(const std::string& device) : device_(device) {
#ifdef _WIN32
  throw std::runtime_error("UART output currently requires a POSIX target");
#else
  fd_ = ::open(device.c_str(), O_WRONLY | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0)
    throw std::system_error(errno, std::generic_category(), "cannot open UART " + device);

  termios settings{};
  if (::tcgetattr(fd_, &settings) != 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(error, std::generic_category(), "cannot read UART settings");
  }
  if (::cfsetispeed(&settings, B115200) != 0 ||
      ::cfsetospeed(&settings, B115200) != 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(error, std::generic_category(), "cannot set UART baud rate");
  }
  settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                        ICRNL | IXON | IXOFF | IXANY);
  settings.c_oflag &= ~OPOST;
  settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
#ifdef CRTSCTS
  settings.c_cflag &= ~CRTSCTS;
#endif
  settings.c_cflag |= CS8 | CLOCAL | CREAD;
  settings.c_iflag &= ~(IXON | IXOFF | IXANY);
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::tcsetattr(fd_, TCSANOW, &settings) != 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw std::system_error(error, std::generic_category(), "cannot configure UART");
  }
#endif
}

UartSender::~UartSender() {
#ifndef _WIN32
  if (fd_ >= 0) ::close(fd_);
#endif
}

void UartSender::send(const RoadResult& result, int input_width) {
  const RoadPacket packet = encodeRoadPacket(result, input_width);
#ifdef _WIN32
  (void)packet;
  throw std::runtime_error("UART output currently requires a POSIX target");
#else
  std::size_t written = 0;
  while (written < packet.size) {
    const ssize_t count = ::write(fd_, packet.bytes.data() + written, packet.size - written);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0)
      throw std::system_error(count < 0 ? errno : EIO, std::generic_category(),
                              "cannot write UART " + device_);
    written += static_cast<std::size_t>(count);
  }
  if (::tcdrain(fd_) != 0)
    throw std::system_error(errno, std::generic_category(), "cannot drain UART " + device_);
#endif
}

}  // namespace road_finder
