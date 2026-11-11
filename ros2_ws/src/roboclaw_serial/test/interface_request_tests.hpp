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

// Including necessary libraries and files.
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dummy_serial_device.hpp"
#include "roboclaw_serial/command.hpp"
#include "roboclaw_serial/interface.hpp"

// Helper function to create a byte vector from char vector
std::vector<std::byte> create_byte_vector(std::vector<unsigned char> list)
{
  std::vector<std::byte> result;
  for (auto val : list) {
    result.push_back(static_cast<std::byte>(val));
  }
  return result;
}

struct Data
{
  std::string requestType;  // "read" or "write"
  const std::vector<unsigned char> writeBytes;
  const std::vector<unsigned char> readBytes;
};

template<typename RequestType>
struct TestData
{
  typename RequestType::ArgsTuple args;
  Data data;
};

class TestExecutor : public ::testing::Test
{
protected:
  // Method to execute test
  // max_bytes_per_read > 0 splits the reply across reads, like a real UART.
  template<typename RequestType>
  void executeTest(const TestData<RequestType> & p, std::size_t max_bytes_per_read = 0)
  {
    // Create a dummy device
    auto device = std::make_shared<DummyTestDevice>(
      create_byte_vector(p.data.readBytes),
      create_byte_vector(p.data.writeBytes));
    device->setMaxBytesPerRead(max_bytes_per_read);

    // Update the interface to use the dummy device for this test
    interface_.setDevice(device);

    // Initialize a request for communication
    RequestType request;

    // If this is a write request, set the request arguments to the test values.
    // (Compiled only for writable requests: read-only ones such as
    // FirmwareVersion carry a std::string, which can't be serialized.)
    if constexpr (RequestType::write_command != uint8_t(roboclaw_serial::Command::NONE)) {
      if (p.data.requestType == "write") {
        request.fields = p.args;
        ASSERT_NO_THROW(interface_.write(request));
      }
    }

    if (p.data.requestType == "read") {
      // Test that values can be "read" from the device
      interface_.read(request);
      ASSERT_TRUE(p.args == request.fields);
    }

    // Disconnect the device
    device->disconnect();
  }

private:
  roboclaw_serial::Interface interface_;
};

TEST_F(TestExecutor, WriteVelocityPIDConstantsM1SerializationTest)
{
  this->executeTest(
    TestData<roboclaw_serial::VelocityPIDConstantsM1>{
    {
      0,
      8631,
      878,
      201562
    },  // request arguments
    {
      "write",  // requestType
      {
        0x80,  // Address byte
        0x1c,  // Command byte
        0x00, 0x00, 0x00, 0x00,  // P parameter
        0x00, 0x00, 0x21, 0xb7,  // I parameter
        0x00, 0x00, 0x03, 0x6e,  // D parameter
        0x00, 0x03, 0x13, 0x5a,  // QPPS value
        0x7d, 0x8e
      },  // writeBytes
      {0xff}  // readBytes
    }});
}

TEST_F(TestExecutor, ReadEncoderCountersSerializationTest)
{
  this->executeTest(
    TestData<roboclaw_serial::EncoderCounters>{
    {12528, 53212},  // request arguments
    {
      "read",  // requestType
      {0x80, 0x4e},  // writeBytes
      {0x00, 0x00, 0x30, 0xf0, 0x00, 0x00, 0xcf, 0xdc, 0xd4, 0xdb}  // readBytes
    }});
}

// ---- BILLEE: UART behaviour (reply split across reads, variable length) ----

const TestData<roboclaw_serial::EncoderCounters> kEncoderCounters{
  {12528, 53212},
  {"read", {0x80, 0x4e}, {0x00, 0x00, 0x30, 0xf0, 0x00, 0x00, 0xcf, 0xdc, 0xd4, 0xdb}}};

const TestData<roboclaw_serial::FirmwareVersion> kFirmwareVersion{
  {std::string("USB Roboclaw 2x15a v4.1.34\n\0", 28)},
  {"read", {0x80, 0x15},
    {0x55, 0x53, 0x42, 0x20, 0x52, 0x6f, 0x62, 0x6f, 0x63, 0x6c, 0x61, 0x77, 0x20, 0x32,
      0x78, 0x31, 0x35, 0x61, 0x20, 0x76, 0x34, 0x2e, 0x31, 0x2e, 0x33, 0x34, 0x0a, 0x00,
      0x34, 0x6a}}};

TEST_F(TestExecutor, ReadEncoderCountersOneBytePerReadTest)
{
  this->executeTest(kEncoderCounters, 1);
}

TEST_F(TestExecutor, ReadEncoderCountersThreeBytesPerReadTest)
{
  this->executeTest(kEncoderCounters, 3);
}

TEST_F(TestExecutor, ReadFirmwareVersionTest)
{
  this->executeTest(kFirmwareVersion);
}

TEST_F(TestExecutor, ReadFirmwareVersionSplitReplyTest)
{
  this->executeTest(kFirmwareVersion, 4);
}

TEST(InterfaceErrors, ReadWithNoReplyThrows)
{
  // The dummy only answers a request matching writeBytes; 0x81 != 0x80, so
  // the reply never comes. Must throw, not hang or parse garbage.
  auto device = std::make_shared<DummyTestDevice>(
    create_byte_vector({0x00, 0x00, 0x30, 0xf0, 0x00, 0x00, 0xcf, 0xdc, 0xd4, 0xdb}),
    create_byte_vector({0x80, 0x4e}));
  roboclaw_serial::Interface interface(device);
  roboclaw_serial::EncoderCounters request;
  EXPECT_THROW(interface.read(request, 0x81), std::runtime_error);
}

TEST(InterfaceErrors, UnsupportedBaudRateThrows)
{
  EXPECT_THROW(roboclaw_serial::SerialDevice::toSpeed(12345), std::invalid_argument);
  EXPECT_EQ(roboclaw_serial::SerialDevice::toSpeed(115200), static_cast<speed_t>(B115200));
}
