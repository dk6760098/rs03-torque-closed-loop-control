#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kTypeMotion = 0x01;
constexpr uint8_t kTypeFeedback = 0x02;
constexpr uint8_t kTypeEnable = 0x03;
constexpr uint8_t kTypeStop = 0x04;
constexpr uint8_t kTypeReadParam = 0x11;
constexpr uint8_t kTypeWriteParam = 0x12;
constexpr uint8_t kTypeProtocolSwitch = 0x19;
constexpr uint16_t kRunMode = 0x7005;
constexpr float kProtocolTorqueMaxNm = 60.0F;
constexpr float kPositionMaxRad = 4.0F * static_cast<float>(M_PI);
// RS03 protocol Type-1 command / Type-2 feedback velocity mapping is
// -20.0 .. +20.0 rad/s. Using a +/-50 range from other motor variants makes
// decoded feedback 2.5x too large and can cause false overspeed trips.
constexpr float kVelocityMaxRadS = 20.0F;
constexpr float kKpMax = 5000.0F;
constexpr float kKdMax = 100.0F;
constexpr uint8_t kSerialExtendedFrameFlag = 0x04;

uint16_t encode_u16(float value, float low, float high) {
  value = std::clamp(value, low, high);
  return static_cast<uint16_t>(std::lround((value - low) * 65535.0F / (high - low)));
}

float decode_u16(uint16_t value, float low, float high) {
  return static_cast<float>(value) * (high - low) / 65535.0F + low;
}

uint16_t encode_u12(float value, float low, float high) {
  value = std::clamp(value, low, high);
  return static_cast<uint16_t>(
      std::lround((value - low) * 4095.0F / (high - low)));
}

float decode_u12(uint16_t value, float low, float high) {
  return static_cast<float>(value & 0x0fffU) * (high - low) / 4095.0F + low;
}

float cyclic_position_error(float absolute_target, float cyclic_feedback) {
  // Type-2 position feedback cycles over -4pi..+4pi, whereas mechPos/loc_ref
  // are absolute multi-turn positions. Compare them modulo the 8pi period.
  return std::remainder(absolute_target - cyclic_feedback,
                        2.0F * kPositionMaxRad);
}
}  // namespace

class Rs03Can {
 public:
  Rs03Can(const std::string &transport, const std::string &iface,
          const std::string &serial_device, int serial_baud,
          bool serial_debug, uint8_t master_id, uint8_t motor_id,
          int receive_timeout_ms, const std::string &motor_protocol)
      : serial_mode_(transport == "serial"),
        mit_protocol_(motor_protocol == "mit"),
        serial_debug_(serial_debug), receive_timeout_ms_(receive_timeout_ms),
        master_id_(master_id), motor_id_(motor_id) {
    if (serial_mode_) {
      if (serial_baud != 921600)
        throw std::invalid_argument("the official CH340 transport currently requires serial_baud=921600");
      fd_ = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (fd_ < 0)
        throw std::runtime_error("cannot open serial device " + serial_device +
                                 ": " + std::strerror(errno));
      termios tty{};
      if (tcgetattr(fd_, &tty) != 0) {
        const std::string error = std::strerror(errno);
        close(fd_); fd_ = -1;
        throw std::runtime_error("cannot read serial settings: " + error);
      }
      cfmakeraw(&tty);
      cfsetispeed(&tty, B921600);
      cfsetospeed(&tty, B921600);
      tty.c_cflag |= CLOCAL | CREAD;
      tty.c_cflag &= ~CRTSCTS;
      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 0;
      if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        const std::string error = std::strerror(errno);
        close(fd_); fd_ = -1;
        throw std::runtime_error("cannot configure serial device: " + error);
      }
      tcflush(fd_, TCIOFLUSH);
      return;
    }

    fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0)
      throw std::runtime_error(std::string("cannot create CAN socket: ") +
                               std::strerror(errno));

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("CAN interface not found: " + iface + ": " +
                               std::strerror(errno));
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("cannot bind CAN interface: " + iface + ": " +
                               std::strerror(errno));
    }
    timeval timeout{};
    timeout.tv_sec = receive_timeout_ms / 1000;
    timeout.tv_usec = (receive_timeout_ms % 1000) * 1000;
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  ~Rs03Can() { if (fd_ >= 0) close(fd_); }

  void set_mode(uint8_t mode) {
    if (mit_protocol_)
      throw std::logic_error("private run_mode write requested in MIT protocol");
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(kRunMode & 0xff);
    data[1] = static_cast<uint8_t>(kRunMode >> 8);
    data[4] = mode;
    send(kTypeWriteParam, master_id_, data);
  }

  bool read_u8_parameter(uint16_t index, uint8_t &value) {
    if (mit_protocol_) return false;
    std::array<uint8_t, 8> request{};
    request[0] = static_cast<uint8_t>(index & 0xff);
    request[1] = static_cast<uint8_t>(index >> 8);
    send(kTypeReadParam, master_id_, request);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(receive_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      can_frame frame{};
      if (!read_frame(frame, deadline)) return false;
      if (!(frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8) continue;
      const uint32_t id = frame.can_id & CAN_EFF_MASK;
      if (((id >> 24) & 0x1f) != kTypeReadParam ||
          (id & 0xff) != master_id_ || ((id >> 8) & 0xff) != motor_id_)
        continue;
      const uint16_t response_index =
          static_cast<uint16_t>(frame.data[0]) |
          (static_cast<uint16_t>(frame.data[1]) << 8);
      if (response_index != index) continue;
      value = frame.data[4];
      return true;
    }
    return false;
  }

  void set_torque(float torque_nm) {
    if (mit_protocol_)
      throw std::logic_error("private torque frame requested in MIT protocol");
    set_private_motion(0.0F, 0.0F, 0.0F, 0.0F, torque_nm);
  }

  void set_private_motion(float position_rad, float velocity_rad_s, float kp,
                          float kd, float feedforward_torque_nm) {
    can_frame frame{};
    const uint16_t torque_raw = encode_u16(
        feedforward_torque_nm, -kProtocolTorqueMaxNm, kProtocolTorqueMaxNm);
    frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(kTypeMotion) << 24) |
                   (static_cast<uint32_t>(torque_raw) << 8) | motor_id_;
    frame.can_dlc = 8;
    const uint16_t pos = encode_u16(
        position_rad, -kPositionMaxRad, kPositionMaxRad);
    const uint16_t vel = encode_u16(
        velocity_rad_s, -kVelocityMaxRadS, kVelocityMaxRadS);
    const uint16_t kp_raw = encode_u16(kp, 0.0F, kKpMax);
    const uint16_t kd_raw = encode_u16(kd, 0.0F, kKdMax);
    frame.data[0] = pos >> 8; frame.data[1] = pos & 0xff;
    frame.data[2] = vel >> 8; frame.data[3] = vel & 0xff;
    frame.data[4] = kp_raw >> 8;  frame.data[5] = kp_raw & 0xff;
    frame.data[6] = kd_raw >> 8;  frame.data[7] = kd_raw & 0xff;
    write_frame(frame);
  }

  void set_mit(float position_rad, float velocity_rad_s, float kp, float kd,
               float feedforward_torque_nm) {
    if (!mit_protocol_)
      throw std::logic_error("standard MIT frame requested in private protocol");
    can_frame frame{};
    frame.can_id = static_cast<canid_t>(motor_id_);  // mode bits 10..8 = 0
    frame.can_dlc = 8;
    const uint16_t p = encode_u16(
        position_rad, -kPositionMaxRad, kPositionMaxRad);
    const uint16_t v = encode_u12(
        velocity_rad_s, -kVelocityMaxRadS, kVelocityMaxRadS);
    const uint16_t kp_raw = encode_u12(kp, 0.0F, kKpMax);
    const uint16_t kd_raw = encode_u12(kd, 0.0F, kKdMax);
    const uint16_t t = encode_u12(
        feedforward_torque_nm, -kProtocolTorqueMaxNm, kProtocolTorqueMaxNm);
    frame.data[0] = p >> 8;
    frame.data[1] = p & 0xff;
    frame.data[2] = v >> 4;
    frame.data[3] = static_cast<uint8_t>(((v & 0x0f) << 4) | (kp_raw >> 8));
    frame.data[4] = kp_raw & 0xff;
    frame.data[5] = kd_raw >> 4;
    frame.data[6] = static_cast<uint8_t>(((kd_raw & 0x0f) << 4) | (t >> 8));
    frame.data[7] = t & 0xff;
    write_frame(frame);
  }

  void set_mit_mode() {
    if (!mit_protocol_)
      throw std::logic_error("MIT mode command requested in private protocol");
    std::array<uint8_t, 8> data{};
    data.fill(0xff);
    data[6] = 0x00;
    data[7] = 0xfc;
    send_standard(motor_id_, data);
  }

  void switch_protocol(const std::string &target) {
    const uint8_t value = target == "mit" ? 2 : 0;
    if (!mit_protocol_) {
      std::array<uint8_t, 8> data{};
      data[0] = 1; data[1] = 2; data[2] = 3;
      data[3] = 4; data[4] = 5; data[5] = 6;
      data[6] = value;
      send(kTypeProtocolSwitch, master_id_, data);
    } else {
      std::array<uint8_t, 8> data{};
      data.fill(0xff);
      data[6] = value;
      data[7] = 0xfd;
      send_standard(motor_id_, data);
    }
  }

  void enable() {
    if (!mit_protocol_) {
      send(kTypeEnable, master_id_, {});
      return;
    }
    std::array<uint8_t, 8> data{};
    data.fill(0xff);
    data[7] = 0xfc;
    send_standard(motor_id_, data);
  }

  void stop() {
    if (!mit_protocol_) {
      send(kTypeStop, master_id_, {});
      return;
    }
    std::array<uint8_t, 8> data{};
    data.fill(0xff);
    data[7] = 0xfd;
    send_standard(motor_id_, data);
  }

  struct Feedback { float position_rad, velocity_rad_s, torque_nm, temperature_c; };
  bool receive_feedback(Feedback &out) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(receive_timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      can_frame frame{};
      if (!read_frame(frame, deadline)) return false;
      if (mit_protocol_) {
        if ((frame.can_id & CAN_EFF_FLAG) || frame.can_dlc < 8 ||
            (frame.can_id & CAN_SFF_MASK) != master_id_ ||
            frame.data[0] != motor_id_)
          continue;
        const uint16_t p =
            (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
        const uint16_t v =
            (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
        const uint16_t t =
            (static_cast<uint16_t>(frame.data[4] & 0x0f) << 8) | frame.data[5];
        const uint16_t temp =
            (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];
        out = {decode_u16(p, -kPositionMaxRad, kPositionMaxRad),
               decode_u12(v, -kVelocityMaxRadS, kVelocityMaxRadS),
               decode_u12(t, -kProtocolTorqueMaxNm, kProtocolTorqueMaxNm),
               temp > 200 ? static_cast<float>(temp) * 0.1F
                          : static_cast<float>(temp)};
        return true;
      }
      if (!(frame.can_id & CAN_EFF_FLAG)) continue;
      const uint32_t id = frame.can_id & CAN_EFF_MASK;
      if (((id >> 24) & 0x1f) != kTypeFeedback || frame.can_dlc < 8 ||
          (id & 0xff) != master_id_ || ((id >> 8) & 0xff) != motor_id_)
        continue;
      const uint16_t p = (frame.data[0] << 8) | frame.data[1];
      const uint16_t v = (frame.data[2] << 8) | frame.data[3];
      const uint16_t t = (frame.data[4] << 8) | frame.data[5];
      const uint16_t temp = (frame.data[6] << 8) | frame.data[7];
      out = {decode_u16(p, -kPositionMaxRad, kPositionMaxRad),
             decode_u16(v, -kVelocityMaxRadS, kVelocityMaxRadS),
             decode_u16(t, -kProtocolTorqueMaxNm, kProtocolTorqueMaxNm),
             static_cast<float>(temp) * 0.1F};
      return true;
    }
    return false;
  }

 private:
  void send(uint8_t type, uint16_t extra, const std::array<uint8_t, 8> &data) {
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(type) << 24) |
                   (static_cast<uint32_t>(extra) << 8) | motor_id_;
    frame.can_dlc = 8;
    std::copy(data.begin(), data.end(), frame.data);
    write_frame(frame);
  }

  void send_standard(uint16_t id, const std::array<uint8_t, 8> &data) {
    can_frame frame{};
    frame.can_id = id & CAN_SFF_MASK;
    frame.can_dlc = 8;
    std::copy(data.begin(), data.end(), frame.data);
    write_frame(frame);
  }

  void write_frame(const can_frame &frame) {
    if (!serial_mode_) {
      if (write(fd_, &frame, sizeof(frame)) != sizeof(frame))
        throw std::runtime_error(std::string("CAN frame write failed: ") +
                                 std::strerror(errno));
      return;
    }
    const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    const uint32_t can_id = extended
        ? (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
    const uint32_t serial_id =
        (can_id << 3) | (extended ? kSerialExtendedFrameFlag : 0U);
    std::array<uint8_t, 17> packet{};
    packet[0] = 'A'; packet[1] = 'T';
    packet[2] = static_cast<uint8_t>(serial_id >> 24);
    packet[3] = static_cast<uint8_t>(serial_id >> 16);
    packet[4] = static_cast<uint8_t>(serial_id >> 8);
    packet[5] = static_cast<uint8_t>(serial_id);
    packet[6] = frame.can_dlc;
    std::copy(frame.data, frame.data + frame.can_dlc, packet.begin() + 7);
    packet[7 + frame.can_dlc] = '\r';
    packet[8 + frame.can_dlc] = '\n';
    const size_t length = 9 + frame.can_dlc;
    if (serial_debug_) {
      std::fprintf(stderr, "RS03 serial TX (%zu):", length);
      for (size_t i = 0; i < length; ++i) std::fprintf(stderr, " %02X", packet[i]);
      std::fprintf(stderr, "\n");
    }
    size_t offset = 0;
    while (offset < length) {
      const ssize_t count = write(fd_, packet.data() + offset, length - offset);
      if (count > 0) { offset += static_cast<size_t>(count); continue; }
      if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      throw std::runtime_error(std::string("CAN frame write failed: ") +
                               std::strerror(errno));
    }
  }

  using Deadline = std::chrono::steady_clock::time_point;

  bool read_exact(uint8_t *data, size_t length, const Deadline &deadline) {
    size_t offset = 0;
    while (offset < length && std::chrono::steady_clock::now() < deadline) {
      fd_set set;
      FD_ZERO(&set); FD_SET(fd_, &set);
      const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
          deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) return false;
      timeval timeout{static_cast<time_t>(remaining.count() / 1000000),
                      static_cast<suseconds_t>(remaining.count() % 1000000)};
      const int ready = select(fd_ + 1, &set, nullptr, nullptr, &timeout);
      if (ready == 0) return false;
      if (ready < 0) { if (errno == EINTR) continue; return false; }
      const ssize_t count = read(fd_, data + offset, length - offset);
      if (count > 0) {
        if (serial_debug_) {
          std::fprintf(stderr, "RS03 serial RX (%zd):", count);
          for (ssize_t i = 0; i < count; ++i)
            std::fprintf(stderr, " %02X", data[offset + static_cast<size_t>(i)]);
          std::fprintf(stderr, "\n");
        }
        offset += static_cast<size_t>(count);
      }
      else if (count < 0 && errno != EINTR && errno != EAGAIN) return false;
    }
    return offset == length;
  }

  bool read_frame(can_frame &frame, const Deadline &deadline) {
    if (!serial_mode_) {
      const ssize_t count = recv(fd_, &frame, sizeof(frame), 0);
      return count == sizeof(frame);
    }
    bool found_a = false;
    while (std::chrono::steady_clock::now() < deadline) {
      uint8_t byte = 0;
      if (!read_exact(&byte, 1, deadline)) return false;
      if (!found_a) { found_a = byte == 'A'; continue; }
      if (byte != 'T') { found_a = byte == 'A'; continue; }
      uint8_t header[5]{};
      if (!read_exact(header, sizeof(header), deadline)) return false;
      const uint32_t serial_id = (static_cast<uint32_t>(header[0]) << 24) |
                                 (static_cast<uint32_t>(header[1]) << 16) |
                                 (static_cast<uint32_t>(header[2]) << 8) |
                                 header[3];
      const uint8_t dlc = header[4];
      if (dlc > 8) { found_a = false; continue; }
      uint8_t payload[10]{};
      if (!read_exact(payload, dlc + 2, deadline)) return false;
      const uint8_t frame_flags = serial_id & 0x07;
      if ((frame_flags != 0 && frame_flags != kSerialExtendedFrameFlag) ||
          payload[dlc] != '\r' || payload[dlc + 1] != '\n') {
        found_a = false; continue;
      }
      frame = {};
      frame.can_id = (serial_id >> 3) |
          (frame_flags == kSerialExtendedFrameFlag ? CAN_EFF_FLAG : 0U);
      frame.can_dlc = dlc;
      std::copy(payload, payload + dlc, frame.data);
      return true;
    }
    return false;
  }

  int fd_{-1};
  bool serial_mode_{false};
  bool mit_protocol_{false};
  bool serial_debug_{false};
  int receive_timeout_ms_{20};
  uint8_t master_id_;
  uint8_t motor_id_;
};

class Rs03TorqueClosedLoopNode final : public rclcpp::Node {
 public:
  Rs03TorqueClosedLoopNode() : Node("rs03_torque_closed_loop") {
    const auto transport = declare_parameter("transport", "serial");
    const auto iface = declare_parameter("can_interface", "can0");
    const auto serial_device = declare_parameter("serial_device", "/dev/ttyUSB0");
    const auto serial_baud = declare_parameter("serial_baud", 921600);
    const auto serial_debug = declare_parameter("serial_debug", false);
    const auto motor_id = declare_parameter("motor_id", 1);
    const auto master_id = declare_parameter("master_id", 255);
    const auto mit_host_id = declare_parameter("mit_host_id", 0xfd);
    motor_protocol_ = declare_parameter("motor_protocol", "private");
    protocol_switch_target_ =
        declare_parameter("protocol_switch_target", "none");
    mode_ = declare_parameter("control_mode", "velocity_pi");
    auto_enable_ = declare_parameter("auto_enable", false);
    timeout_s_ = declare_parameter("command_timeout_s", 0.30);
    max_torque_nm_ = std::clamp(declare_parameter("max_torque_nm", 0.5),
                                0.0, static_cast<double>(kProtocolTorqueMaxNm));
    torque_slew_rate_ = declare_parameter("torque_slew_rate_nm_s", 2.0);
    max_velocity_command_rad_s_ =
        declare_parameter("max_velocity_command_rad_s", 0.30);
    position_max_offset_rad_ = declare_parameter("position_max_offset_rad", 0.15);
    position_tracking_error_rad_ =
        declare_parameter("position_tracking_error_rad", 0.25);
    velocity_kp_ = declare_parameter("velocity_kp", 0.40);
    velocity_ki_ = declare_parameter("velocity_ki", 0.30);
    velocity_integral_limit_nm_ =
        declare_parameter("velocity_integral_limit_nm", 0.20);
    velocity_feedforward_nm_ =
        declare_parameter("velocity_feedforward_nm", 0.0);
    position_kp_ = declare_parameter("position_kp", 2.0);
    position_ki_ = declare_parameter("position_ki", 0.0);
    position_kd_ = declare_parameter("position_kd", 0.20);
    position_integral_limit_nm_ =
        declare_parameter("position_integral_limit_nm", 0.10);
    position_feedforward_nm_ =
        declare_parameter("position_feedforward_nm", 0.0);
    mit_kp_ = declare_parameter("mit_kp", 2.0);
    mit_kd_ = declare_parameter("mit_kd", 0.20);
    mit_feedforward_torque_nm_ =
        declare_parameter("mit_feedforward_torque_nm", 0.0);
    mit_position_slew_rate_rad_s_ =
        declare_parameter("mit_position_slew_rate_rad_s", 0.05);
    velocity_filter_alpha_ = declare_parameter("velocity_filter_alpha", 0.20);
    max_velocity_rad_s_ = declare_parameter("max_velocity_rad_s", 0.80);
    velocity_trip_samples_ = declare_parameter("velocity_trip_samples", 5);
    max_temperature_c_ = declare_parameter("max_temperature_c", 60.0);
    feedback_miss_limit_ = declare_parameter("feedback_miss_limit", 10);
    const auto receive_timeout = declare_parameter("receive_timeout_ms", 20);
    if (motor_id < 0 || motor_id > 255 || master_id < 0 || master_id > 255 ||
        mit_host_id < 0 || mit_host_id > 255)
      throw std::invalid_argument(
          "motor_id, master_id, and mit_host_id must be in [0, 255]");
    if (timeout_s_ <= 0.0 || receive_timeout < 0)
      throw std::invalid_argument("timeouts must be positive");
    if (max_torque_nm_ <= 0.0 || torque_slew_rate_ <= 0.0 ||
        max_velocity_command_rad_s_ <= 0.0 || position_max_offset_rad_ <= 0.0 ||
        position_tracking_error_rad_ <= position_max_offset_rad_ ||
        velocity_kp_ < 0.0 || velocity_ki_ < 0.0 ||
        velocity_integral_limit_nm_ < 0.0 || velocity_feedforward_nm_ < 0.0 ||
        position_kp_ < 0.0 || position_ki_ < 0.0 || position_kd_ < 0.0 ||
        position_integral_limit_nm_ < 0.0 ||
        std::abs(position_feedforward_nm_) > max_torque_nm_ ||
        mit_kp_ < 0.0 || mit_kp_ > kKpMax ||
        mit_kd_ < 0.0 || mit_kd_ > kKdMax ||
        std::abs(mit_feedforward_torque_nm_) > max_torque_nm_ ||
        mit_position_slew_rate_rad_s_ <= 0.0 ||
        velocity_filter_alpha_ <= 0.0 || velocity_filter_alpha_ > 1.0 ||
        max_velocity_rad_s_ <= max_velocity_command_rad_s_ ||
        max_temperature_c_ <= 0.0)
      throw std::invalid_argument("controller gains or safety limits are invalid");
    if (velocity_trip_samples_ < 1 || feedback_miss_limit_ < 1)
      throw std::invalid_argument("trip sample counts must be at least 1");
    if (mode_ != "velocity_pi" && mode_ != "position_pid" &&
        mode_ != "mit_impedance")
      throw std::invalid_argument(
          "control_mode must be velocity_pi, position_pid, or mit_impedance");
    if (transport != "serial" && transport != "socketcan")
      throw std::invalid_argument("transport must be serial or socketcan");
    if (motor_protocol_ != "private" && motor_protocol_ != "mit")
      throw std::invalid_argument("motor_protocol must be private or mit");
    if (protocol_switch_target_ != "none" &&
        protocol_switch_target_ != "private" &&
        protocol_switch_target_ != "mit")
      throw std::invalid_argument(
          "protocol_switch_target must be none, private, or mit");
    if (protocol_switch_target_ == "none" &&
        ((mode_ == "mit_impedance") != (motor_protocol_ == "mit")))
      throw std::invalid_argument(
          "mit_impedance requires motor_protocol=mit; velocity_pi and "
          "position_pid require motor_protocol=private");
    if (protocol_switch_target_ != "none" && auto_enable_)
      throw std::invalid_argument(
          "protocol switching requires auto_enable=false");
    can_ = std::make_unique<Rs03Can>(
        transport, iface, serial_device, serial_baud, serial_debug,
        static_cast<uint8_t>(motor_protocol_ == "mit" ? mit_host_id : master_id),
        static_cast<uint8_t>(motor_id),
        receive_timeout, motor_protocol_);

    const std::string command_topic = mode_ == "velocity_pi"
        ? "~/velocity_command_rad_s" : "~/position_offset_command_rad";
    command_sub_ = create_subscription<std_msgs::msg::Float32>(
        command_topic, 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
          if (!std::isfinite(msg->data)) {
            RCLCPP_ERROR(get_logger(), "rejected non-finite command");
            return;
          }
          const bool first_command = !command_seen_;
          if (first_command)
            RCLCPP_INFO(get_logger(), "first %s command received: %.3f",
                        mode_.c_str(), msg->data);
          command_ = msg->data;
          last_command_ = now();
          command_seen_ = true;
          if (armed_waiting_for_command_) {
            if (!auto_enable_) return;
            startup_position_rad_ = last_position_feedback_rad_;
            applied_command_ = 0.0F;
            integral_torque_nm_ = 0.0F;
            mit_applied_offset_rad_ = 0.0F;
            send_neutral_command();
            can_->enable();
            enabled_ = true;
            armed_waiting_for_command_ = false;
            last_update_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(get_logger(),
                        "%s enabled on first command; startup position=%.3f rad",
                        mode_.c_str(), startup_position_rad_);
          }
        });
    torque_pub_ = create_publisher<std_msgs::msg::Float32>("~/estimated_torque_nm", 10);
    position_pub_ = create_publisher<std_msgs::msg::Float32>("~/position_rad", 10);
    velocity_pub_ = create_publisher<std_msgs::msg::Float32>("~/velocity_rad_s", 10);
    temperature_pub_ = create_publisher<std_msgs::msg::Float32>("~/temperature_c", 10);
    commanded_torque_pub_ =
        create_publisher<std_msgs::msg::Float32>("~/commanded_torque_nm", 10);
    control_error_pub_ =
        create_publisher<std_msgs::msg::Float32>("~/control_error", 10);
    timer_ = create_wall_timer(10ms, [this] { update(); });

    can_->stop();
    Rs03Can::Feedback probe{};
    bool motor_online = can_->receive_feedback(probe);
    if (!motor_online && motor_protocol_ == "mit") {
      // The RS03 MIT disable frame does not produce status feedback.  Probe
      // with a zero-gain, zero-feedforward command, then immediately disable.
      // With Kp=Kd=tau_ff=0 this command cannot request holding or motion
      // torque, regardless of the encoded position and velocity fields.
      RCLCPP_INFO(get_logger(),
                  "MIT stop frame returned no feedback; running zero-output "
                  "feedback probe");
      can_->set_mit(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
      can_->enable();
      can_->set_mit(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
      motor_online = can_->receive_feedback(probe);
      can_->stop();
    }
    if (motor_online) {
      startup_position_rad_ = probe.position_rad;
      last_position_feedback_rad_ = probe.position_rad;
      last_velocity_feedback_rad_s_ = probe.velocity_rad_s;
      filtered_velocity_rad_s_ = probe.velocity_rad_s;
      has_position_feedback_ = true;
      RCLCPP_INFO(get_logger(),
                  "RS03 feedback received: position=%.3f rad, velocity=%.3f rad/s, "
                  "estimated_torque=%.3f Nm, temperature=%.1f C",
                  probe.position_rad, probe.velocity_rad_s, probe.torque_nm,
                  probe.temperature_c);
    }
    else
      RCLCPP_WARN(get_logger(), "no RS03 feedback after safe zero-output probe");

    if (protocol_switch_target_ != "none") {
      if (protocol_switch_target_ == motor_protocol_) {
        throw std::invalid_argument(
            "protocol_switch_target already matches motor_protocol");
      }
      can_->switch_protocol(protocol_switch_target_);
      switch_only_ = true;
      RCLCPP_FATAL(
          get_logger(),
          "protocol switch command sent: %s -> %s. Disconnect motor power, "
          "wait, power it again, then launch with motor_protocol:=%s. "
          "Do not send motion commands in this process.",
          motor_protocol_.c_str(), protocol_switch_target_.c_str(),
          protocol_switch_target_.c_str());
      return;
    }

    if (auto_enable_) {
      if (!motor_online)
        throw std::runtime_error("refusing to enable: no valid RS03 feedback");
      if (motor_protocol_ == "private") {
        const uint8_t run_mode = 0;
        can_->set_mode(run_mode);
        uint8_t confirmed_run_mode = 0xff;
        if (!can_->read_u8_parameter(kRunMode, confirmed_run_mode) ||
            confirmed_run_mode != run_mode) {
          throw std::runtime_error(
              "refusing to enable: RS03 run_mode write could not be verified");
        }
        RCLCPP_INFO(get_logger(), "RS03 private run_mode confirmed: %u",
                    static_cast<unsigned>(confirmed_run_mode));
      } else {
        can_->set_mit_mode();
        RCLCPP_INFO(get_logger(), "RS03 standard-frame MIT mode selected");
      }
      send_neutral_command();
      armed_waiting_for_command_ = true;
      RCLCPP_WARN(get_logger(),
                  "%s armed; motor remains disabled until the first valid command",
                  mode_.c_str());
      last_command_ = now();
      last_update_ = std::chrono::steady_clock::now();
    } else {
      RCLCPP_WARN(get_logger(), "auto_enable=false: motor remains stopped; set true only after bench checks");
    }
  }

  ~Rs03TorqueClosedLoopNode() override {
    try { zero_and_stop(); } catch (...) {}
  }

 private:
  void update() {
    if (switch_only_) return;
    if (!enabled_) return;
    const auto ros_now = now();
    const auto update_time = std::chrono::steady_clock::now();
    const double dt = std::clamp(
        std::chrono::duration<double>(update_time - last_update_).count(),
        0.0, 0.1);
    last_update_ = update_time;
    const bool fresh = command_seen_ && (ros_now - last_command_).seconds() <= timeout_s_;
    if (!fresh && command_seen_ && !timeout_reported_) {
      send_zero_command();
      can_->stop();
      enabled_ = false;
      applied_command_ = 0.0F;
      timeout_reported_ = true;
      RCLCPP_FATAL(get_logger(),
                   "command timeout: output forced to zero and motor stopped; "
                   "restart node to re-enable");
      return;
    }
    const float target = mode_ == "velocity_pi"
        ? std::clamp(command_, -static_cast<float>(max_velocity_command_rad_s_),
                     static_cast<float>(max_velocity_command_rad_s_))
        : std::clamp(command_, -static_cast<float>(position_max_offset_rad_),
                     static_cast<float>(position_max_offset_rad_));
    float base_torque = 0.0F;
    float ki = 0.0F;
    float integral_limit = 0.0F;
    if (mode_ == "velocity_pi") {
      control_error_ = target - filtered_velocity_rad_s_;
      base_torque = static_cast<float>(velocity_kp_) * control_error_;
      if (std::abs(target) > 1e-4F)
        base_torque += std::copysign(
            static_cast<float>(velocity_feedforward_nm_), target);
      ki = static_cast<float>(velocity_ki_);
      integral_limit = static_cast<float>(velocity_integral_limit_nm_);
    } else if (mode_ == "position_pid") {
      const float absolute_target = startup_position_rad_ + target;
      control_error_ = cyclic_position_error(
          absolute_target, last_position_feedback_rad_);
      base_torque = static_cast<float>(position_kp_) * control_error_ -
          static_cast<float>(position_kd_) * filtered_velocity_rad_s_ +
          static_cast<float>(position_feedforward_nm_);
      ki = static_cast<float>(position_ki_);
      integral_limit = static_cast<float>(position_integral_limit_nm_);
    } else {
      const float max_offset_step =
          static_cast<float>(mit_position_slew_rate_rad_s_ * dt);
      mit_applied_offset_rad_ += std::clamp(
          target - mit_applied_offset_rad_, -max_offset_step, max_offset_step);
      const float absolute_target = startup_position_rad_ + mit_applied_offset_rad_;
      control_error_ = cyclic_position_error(
          absolute_target, last_position_feedback_rad_);
      const float estimated_pd_torque =
          static_cast<float>(mit_kp_) * control_error_ -
          static_cast<float>(mit_kd_) * filtered_velocity_rad_s_;
      const float raw_estimated_torque = estimated_pd_torque +
          static_cast<float>(mit_feedforward_torque_nm_);
      desired_torque_nm_ = std::clamp(
          raw_estimated_torque,
          -static_cast<float>(max_torque_nm_),
          static_cast<float>(max_torque_nm_));
      applied_command_ = desired_torque_nm_;
      // MIT has no separate torque-limit field. Offset the feed-forward term so
      // the estimated P+D+FF demand stays inside the configured bench limit.
      // The independent velocity/temperature/tracking trips remain mandatory.
      mit_sent_feedforward_nm_ = std::clamp(
          static_cast<float>(mit_feedforward_torque_nm_) +
              (desired_torque_nm_ - raw_estimated_torque),
          -kProtocolTorqueMaxNm, kProtocolTorqueMaxNm);
      const float cyclic_target = std::remainder(
          absolute_target, 2.0F * kPositionMaxRad);
      can_->set_mit(
          cyclic_target, 0.0F, static_cast<float>(mit_kp_),
          static_cast<float>(mit_kd_),
          mit_sent_feedforward_nm_);
    }

    if (mode_ != "mit_impedance") {
      const float proposed_integral = std::clamp(
          integral_torque_nm_ + ki * control_error_ * static_cast<float>(dt),
          -integral_limit, integral_limit);
      const float proposed_unsaturated = base_torque + proposed_integral;
      const float torque_limit = static_cast<float>(max_torque_nm_);
      const bool drives_further_high =
          proposed_unsaturated > torque_limit && ki * control_error_ > 0.0F;
      const bool drives_further_low =
          proposed_unsaturated < -torque_limit && ki * control_error_ < 0.0F;
      if (!drives_further_high && !drives_further_low)
        integral_torque_nm_ = proposed_integral;
      desired_torque_nm_ = std::clamp(
          base_torque + integral_torque_nm_, -torque_limit, torque_limit);

      const float max_step = static_cast<float>(torque_slew_rate_ * dt);
      applied_command_ += std::clamp(
          desired_torque_nm_ - applied_command_, -max_step, max_step);
      can_->set_torque(applied_command_);
    }
    if (fresh) timeout_reported_ = false;

    Rs03Can::Feedback fb{};
    if (can_->receive_feedback(fb)) {
      feedback_miss_count_ = 0;
      last_position_feedback_rad_ = fb.position_rad;
      last_velocity_feedback_rad_s_ = fb.velocity_rad_s;
      filtered_velocity_rad_s_ += static_cast<float>(velocity_filter_alpha_) *
          (fb.velocity_rad_s - filtered_velocity_rad_s_);
      has_position_feedback_ = true;
      std_msgs::msg::Float32 msg;
      msg.data = fb.torque_nm;
      torque_pub_->publish(msg);
      msg.data = fb.position_rad;
      position_pub_->publish(msg);
      msg.data = fb.velocity_rad_s;
      velocity_pub_->publish(msg);
      msg.data = fb.temperature_c;
      temperature_pub_->publish(msg);
      msg.data = applied_command_;
      commanded_torque_pub_->publish(msg);
      msg.data = control_error_;
      control_error_pub_->publish(msg);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "%s: target=%.3f, error=%.3f, torque=%.3f Nm, position=%.3f rad, "
          "velocity=%.3f rad/s",
          mode_.c_str(), target, control_error_, applied_command_,
          fb.position_rad, fb.velocity_rad_s);

      if (std::abs(fb.velocity_rad_s) > max_velocity_rad_s_)
        ++velocity_trip_count_;
      else
        velocity_trip_count_ = 0;
      const bool velocity_trip = velocity_trip_count_ >= velocity_trip_samples_;
      const float position_error = mode_ != "velocity_pi"
          ? cyclic_position_error(
                startup_position_rad_ + (mode_ == "mit_impedance"
                    ? mit_applied_offset_rad_
                    : std::clamp(
                          command_, -static_cast<float>(position_max_offset_rad_),
                          static_cast<float>(position_max_offset_rad_))),
                fb.position_rad)
          : 0.0F;
      const bool position_trip = mode_ != "velocity_pi" && command_seen_ &&
          std::abs(position_error) > position_tracking_error_rad_;
      if (velocity_trip || position_trip || fb.temperature_c > max_temperature_c_) {
        send_zero_command();
        can_->stop();
        enabled_ = false;
        applied_command_ = 0.0F;
        RCLCPP_FATAL(get_logger(),
                     "safety stop: velocity=%.3f rad/s (limit %.3f), "
                     "position_error=%.3f rad (limit %.3f), temperature=%.1f C (limit %.1f)",
                     fb.velocity_rad_s, max_velocity_rad_s_,
                     position_error,
                     position_tracking_error_rad_,
                     fb.temperature_c, max_temperature_c_);
      }
    } else if (++feedback_miss_count_ >= feedback_miss_limit_) {
      send_zero_command();
      can_->stop();
      enabled_ = false;
      applied_command_ = 0.0F;
      RCLCPP_FATAL(get_logger(),
                   "feedback timeout: %ld consecutive receive failures; motor stopped",
                   static_cast<long>(feedback_miss_count_));
    }
  }

  void zero_and_stop() {
    if (!can_) return;
    send_zero_command();
    can_->stop();
    enabled_ = false;
  }

  void send_zero_command() {
    send_neutral_command();
  }

  void send_neutral_command() {
    if (motor_protocol_ == "mit") {
      can_->set_mit(last_position_feedback_rad_, 0.0F, 0.0F, 0.0F, 0.0F);
    } else {
      can_->set_torque(0.0F);
    }
  }

  std::unique_ptr<Rs03Can> can_;
  std::string mode_;
  std::string motor_protocol_{"private"};
  std::string protocol_switch_target_{"none"};
  double timeout_s_{0.30}, max_torque_nm_{0.5}, torque_slew_rate_{2.0};
  double max_velocity_command_rad_s_{0.30};
  double position_max_offset_rad_{0.15}, position_tracking_error_rad_{0.25};
  double velocity_kp_{0.40}, velocity_ki_{0.30};
  double velocity_integral_limit_nm_{0.20}, velocity_feedforward_nm_{0.0};
  double position_kp_{2.0}, position_ki_{0.0}, position_kd_{0.20};
  double position_integral_limit_nm_{0.10}, position_feedforward_nm_{0.0};
  double mit_kp_{2.0}, mit_kd_{0.20}, mit_feedforward_torque_nm_{0.0};
  double mit_position_slew_rate_rad_s_{0.05};
  double velocity_filter_alpha_{0.20};
  double max_velocity_rad_s_{0.80}, max_temperature_c_{60.0};
  int64_t velocity_trip_samples_{5};
  int64_t velocity_trip_count_{0};
  int64_t feedback_miss_limit_{10}, feedback_miss_count_{0};
  float command_{0.0F}, applied_command_{0.0F}, startup_position_rad_{0.0F};
  float last_position_feedback_rad_{0.0F}, last_velocity_feedback_rad_s_{0.0F};
  float filtered_velocity_rad_s_{0.0F};
  float integral_torque_nm_{0.0F}, desired_torque_nm_{0.0F};
  float mit_applied_offset_rad_{0.0F};
  float mit_sent_feedforward_nm_{0.0F};
  float control_error_{0.0F};
  bool has_position_feedback_{false};
  bool auto_enable_{false}, armed_waiting_for_command_{false};
  bool switch_only_{false};
  bool enabled_{false}, command_seen_{false}, timeout_reported_{false};
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_update_{std::chrono::steady_clock::now()};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr commanded_torque_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Rs03TorqueClosedLoopNode>()); }
  catch (const std::exception &e) {
    std::fprintf(stderr, "RS03 controller fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
