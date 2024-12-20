// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/testing/test_codec_extensions.h"

namespace flutter {

PointExtensionSerializer::PointExtensionSerializer() = default;
PointExtensionSerializer::~PointExtensionSerializer() = default;

// static
const PointExtensionSerializer& PointExtensionSerializer::GetInstance() {
  static PointExtensionSerializer sInstance;
  return sInstance;
}

EncodableValue PointExtensionSerializer::ReadValueOfType(
    uint8_t type,
    ByteStreamReader* stream) const {
  if (type == kPointType) {
    int32_t x = stream->ReadInt32();
    int32_t y = stream->ReadInt32();
    return CustomEncodableValue(Point(x, y));
  }
  return StandardCodecSerializer::ReadValueOfType(type, stream);
}

void PointExtensionSerializer::WriteValue(const EncodableValue& value,
                                          ByteStreamWriter* stream) const {
  auto custom_value = std::get_if<CustomEncodableValue>(&value);
  if (!custom_value) {
    StandardCodecSerializer::WriteValue(value, stream);
    return;
  }
  stream->WriteByte(kPointType);
  const Point& point = std::any_cast<Point>(*custom_value);
  stream->WriteInt32(point.x());
  stream->WriteInt32(point.y());
}

SomeDataExtensionSerializer::SomeDataExtensionSerializer() = default;
SomeDataExtensionSerializer::~SomeDataExtensionSerializer() = default;

// static
const SomeDataExtensionSerializer& SomeDataExtensionSerializer::GetInstance() {
  static SomeDataExtensionSerializer sInstance;
  return sInstance;
}

EncodableValue SomeDataExtensionSerializer::ReadValueOfType(
    uint8_t type,
    ByteStreamReader* stream) const {
  if (type == kSomeDataType) {
    size_t size = ReadSize(stream);
    std::vector<uint8_t> data;
    data.resize(size);
    stream->ReadBytes(data.data(), size);
    EncodableValue label = ReadValue(stream);
    return CustomEncodableValue(SomeData(std::get<std::string>(label), data));
  }
  return StandardCodecSerializer::ReadValueOfType(type, stream);
}

void SomeDataExtensionSerializer::WriteValue(const EncodableValue& value,
                                             ByteStreamWriter* stream) const {
  auto custom_value = std::get_if<CustomEncodableValue>(&value);
  if (!custom_value) {
    StandardCodecSerializer::WriteValue(value, stream);
    return;
  }
  stream->WriteByte(kSomeDataType);
  const SomeData& some_data = std::any_cast<SomeData>(*custom_value);
  size_t data_size = some_data.data().size();
  WriteSize(data_size, stream);
  stream->WriteBytes(some_data.data().data(), data_size);
  WriteValue(EncodableValue(some_data.label()), stream);
}

}  // namespace flutter
