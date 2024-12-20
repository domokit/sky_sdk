// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_BYTE_BUFFER_STREAMS_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_BYTE_BUFFER_STREAMS_H_

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "include/flutter/byte_streams.h"

namespace flutter {

// Implementation of ByteStreamReader base on a byte array.
class ByteBufferStreamReader : public ByteStreamReader {
 public:
  // Createa a reader reading from |bytes|, which must have a length of |size|.
  // |bytes| must remain valid for the lifetime of this object.
  explicit ByteBufferStreamReader(const uint8_t* bytes, size_t size)
      : bytes_(bytes), size_(size) {}

  virtual ~ByteBufferStreamReader() = default;

  // |ByteStreamReader|
  uint8_t ReadByte() override {
    if (location_ >= size_) {
      std::cerr << "Invalid read in StandardCodecByteStreamReader" << std::endl;
      return 0;
    }
    return bytes_[location_++];
  }

  // |ByteStreamReader|
  void ReadBytes(uint8_t* buffer, size_t length) override {
    if (location_ + length > size_) {
      std::cerr << "Invalid read in StandardCodecByteStreamReader" << std::endl;
      return;
    }
    std::memcpy(buffer, &bytes_[location_], length);
    location_ += length;
  }

  // |ByteStreamReader|
  void ReadAlignment(uint8_t alignment) override {
    uint8_t mod = location_ % alignment;
    if (mod) {
      location_ += alignment - mod;
    }
  }

 private:
  // The buffer to read from.
  const uint8_t* bytes_;
  // The total size of the buffer.
  size_t size_;
  // The current read location.
  size_t location_ = 0;
};

// Implementation of ByteStreamWriter based on a byte array.
class ByteBufferStreamWriter : public ByteStreamWriter {
 public:
  // Creates a writer that writes into |buffer|.
  // |buffer| must remain valid for the lifetime of this object.
  explicit ByteBufferStreamWriter(std::vector<uint8_t>* buffer)
      : bytes_(buffer) {
    assert(buffer);
  }

  virtual ~ByteBufferStreamWriter() = default;

  // |ByteStreamWriter|
  void WriteByte(uint8_t byte) { bytes_->push_back(byte); }

  // |ByteStreamWriter|
  void WriteBytes(const uint8_t* bytes, size_t length) {
    assert(length > 0);
    bytes_->insert(bytes_->end(), bytes, bytes + length);
  }

  // |ByteStreamWriter|
  void WriteAlignment(uint8_t alignment) {
    uint8_t mod = bytes_->size() % alignment;
    if (mod) {
      for (int i = 0; i < alignment - mod; ++i) {
        WriteByte(0);
      }
    }
  }

 private:
  // The buffer to write to.
  std::vector<uint8_t>* bytes_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_BYTE_BUFFER_STREAMS_H_
