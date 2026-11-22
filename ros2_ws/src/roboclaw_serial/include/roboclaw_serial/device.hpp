// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace roboclaw_serial
{

class SerialDevice
{
public:
  typedef std::shared_ptr<SerialDevice> SharedPtr;

  // RoboClaw factory-default packet serial baud rate.
  static constexpr int kDefaultBaudRate = 38400;

  SerialDevice() = default;

  // BILLEE: baud_rate added. Ignored by USB (CDC-ACM) ports, but a real UART
  // (e.g. Jetson /dev/ttyTHSx) must match the RoboClaw's Motion Studio setting.
  explicit SerialDevice(const std::string device, int baud_rate = kDefaultBaudRate)
  : speed_(toSpeed(baud_rate))
  {
    connect(device);
  }
  virtual ~SerialDevice() {disconnect();}

  // Map a numeric baud rate onto a termios speed. Covers every rate the
  // RoboClaw's packet serial mode supports.
  static speed_t toSpeed(int baud_rate)
  {
    switch (baud_rate) {
      case 2400: return B2400;
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      default:
        throw std::invalid_argument(
                "Unsupported RoboClaw baud rate " + std::to_string(baud_rate) +
                " (use 2400, 9600, 19200, 38400, 57600, 115200, 230400 or 460800)");
    }
  }

  // BILLEE: discard any unread input (e.g. the tail of a late reply) so the
  // next request starts in sync.
  virtual void flushInput()
  {
    if (connected_) {
      tcflush(fd_, TCIFLUSH);
    }
  }

  virtual bool connect(const std::string & device)
  {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
    connected_ = fd_ != -1;

    if (connected_) {
      setSerialDeviceOptions();
    } else {
      std::cerr << "Failed to open serial device: " << device << std::endl;
      perror("Error");
    }

    return connected_;
  }

  virtual void disconnect()
  {
    if (connected_) {
      close(fd_);
      connected_ = false;
    }
  }

  bool connected() const {return connected_;}

  virtual std::size_t write(const std::byte * buffer, std::size_t count)
  {
    ssize_t result = ::write(fd_, buffer, count);
    if (result < 0) {
      // Error writing to device
      throw std::range_error("Error writing to the device!");
    }
    return static_cast<std::size_t>(result) == count;
  }

  virtual std::size_t read(std::byte * buffer, std::size_t count)
  {
    fd_set set;
    struct timeval timeout;

    /* Initialize the file descriptor set. */
    FD_ZERO(&set);
    FD_SET(fd_, &set);

    /* Initialize the timeout data structure. */
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;  // 10ms

    /* select returns 0 if timeout, 1 if input available, -1 if error. */
    int res = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
    if (res < 0) {
      throw std::range_error("Error reading from the serial device!");
    } else if (res == 0) {
      throw std::runtime_error("Read timeout!");
    }
    ssize_t result = ::read(fd_, buffer, count);
    if (result < 0) {
      // Error reading from the device
      throw std::range_error("Error reading from the serial device!");
    }

    return static_cast<std::size_t>(result);
  }

protected:
  bool connected_ = false;

private:
  void setSerialDeviceOptions()
  {
    struct termios options;
    tcgetattr(fd_, &options);
    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    // BILLEE: upstream never set a speed, which left the port at B0. Harmless
    // over USB, wrong on a real UART.
    cfsetispeed(&options, speed_);
    cfsetospeed(&options, speed_);
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &options);

    // Set the file descriptor to non-blocking mode
    fcntl(fd_, F_SETFL, O_NONBLOCK);
  }

  int fd_ = -1;
  speed_t speed_ = B38400;
};

}  // namespace roboclaw_serial
