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

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "roboclaw_serial/device.hpp"

class DummyTestDevice : public roboclaw_serial::SerialDevice
{
public:
  DummyTestDevice(
    const std::vector<std::byte> && read_buffer, const std::vector<std::byte> && write_buffer)
  : expected_read_buffer_(std::move(read_buffer)), expected_write_buffer_(std::move(write_buffer))
  {
    connect("dummy device");
  }

  bool connect(const std::string &) override
  {
    connected_ = true;
    return true;
  }

  void disconnect() override {connected_ = false;}

  std::size_t write(const std::byte * buffer, std::size_t count) override
  {
    // Track if the data we received matched what is expected
    write_okay_ = verifyWriteBuffer(buffer, count);

    // Return the number of bytes written
    return count;
  }

  std::size_t read(std::byte * buffer, std::size_t count) override
  {
    if (write_okay_) {
      // Update the buffer and return the number of bytes read
      return updateReadBuffer(buffer, count);
    } else {
      // We don't return any data if the input data was garbage
      // There is nothing to read... simulate a block!
      return 0;
    }
  }

private:
  // Track whether the data received from the interface is what was expected
  bool write_okay_ = false;

  // buffers containing expected values
  const std::vector<std::byte> expected_read_buffer_;
  const std::vector<std::byte> expected_write_buffer_;

  /* Ensure that the contents of a buffer and expected buffer match */
  bool verifyWriteBuffer(const void * buffer, size_t count)
  {
    // Check that the size of the serialized data matches the expected size
    bool valid = (expected_write_buffer_.size() == count);

    if (valid) {
      const std::byte * byte_ptr = static_cast<const std::byte *>(buffer);
      for (size_t i = 0; i < count; ++i) {
        if (expected_write_buffer_[i] != byte_ptr[i]) {
          valid = false;
          break;
        }
      }
    }

    return valid;
  }

  size_t updateReadBuffer(void * buffer, size_t count)
  {
    // Cast the buffer to a bytes array
    std::byte * byte_ptr = static_cast<std::byte *>(buffer);

    // Determine how many bytes to write; this will be the minimum of how many we should (count)
    // and how many we can (expected_read_buffer_.size())
    size_t n_bytes = std::min(count, expected_read_buffer_.size());

    for (size_t i = 0; i < n_bytes; ++i) {
      // Write the response byte to the buffer
      byte_ptr[i] = expected_read_buffer_[i];
    }

    // Return the number of bytes written
    return n_bytes;
  }
};
