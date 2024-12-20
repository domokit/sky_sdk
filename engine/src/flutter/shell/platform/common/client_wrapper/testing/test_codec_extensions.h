// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_TESTING_TEST_CODEC_EXTENSIONS_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_TESTING_TEST_CODEC_EXTENSIONS_H_

#include "flutter/shell/platform/common/client_wrapper/include/flutter/encodable_value.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_codec_serializer.h"

namespace flutter {

// A representation of a point, for custom type testing of a simple type.
class Point {
 public:
  Point(int x, int y) : x_(x), y_(y) {}
  ~Point() = default;

  int x() const { return x_; }
  int y() const { return y_; }

  bool operator==(const Point& other) const {
    return x_ == other.x_ && y_ == other.y_;
  }

 private:
  int x_;
  int y_;
};

// A typed binary data object with extra fields, for custom type testing of a
// variable-length type that includes types handled by the core standard codec.
class SomeData {
 public:
  SomeData(const std::string& label, const std::vector<uint8_t>& data)
      : label_(label), data_(data) {}
  ~SomeData() = default;

  const std::string& label() const { return label_; }
  const std::vector<uint8_t>& data() const { return data_; }

 private:
  std::string label_;
  std::vector<uint8_t> data_;
};

// Codec extension for Point.
class PointExtensionSerializer : public StandardCodecSerializer {
 public:
  PointExtensionSerializer();
  virtual ~PointExtensionSerializer();

  static const PointExtensionSerializer& GetInstance();

  // |TestCodecSerializer|
  EncodableValue ReadValueOfType(uint8_t type,
                                 ByteStreamReader* stream) const override;

  // |TestCodecSerializer|
  void WriteValue(const EncodableValue& value,
                  ByteStreamWriter* stream) const override;

 private:
  static constexpr uint8_t kPointType = 128;
};

// Codec extension for SomeData.
class SomeDataExtensionSerializer : public StandardCodecSerializer {
 public:
  SomeDataExtensionSerializer();
  virtual ~SomeDataExtensionSerializer();

  static const SomeDataExtensionSerializer& GetInstance();

  // |TestCodecSerializer|
  EncodableValue ReadValueOfType(uint8_t type,
                                 ByteStreamReader* stream) const override;

  // |TestCodecSerializer|
  void WriteValue(const EncodableValue& value,
                  ByteStreamWriter* stream) const override;

 private:
  static constexpr uint8_t kSomeDataType = 129;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_TESTING_TEST_CODEC_EXTENSIONS_H_
