// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file contains what would normally be standard_codec_serializer.cc,
// standard_message_codec.cc, and standard_method_codec.cc. They are grouped
// together to simplify use of the client wrapper, since the common case is
// that any client that needs one of these files needs all three.

#include <cassert>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "byte_buffer_streams.h"
#include "include/flutter/standard_codec_serializer.h"
#include "include/flutter/standard_message_codec.h"
#include "include/flutter/standard_method_codec.h"

namespace flutter {

// ===== standard_codec_serializer.h =====

namespace {

// The order/values here must match the constants in message_codecs.dart.
enum class EncodedType {
  kNull = 0,
  kTrue,
  kFalse,
  kInt32,
  kInt64,
  kLargeInt,  // No longer used. If encountered, treat as kString.
  kFloat64,
  kString,
  kUInt8List,
  kInt32List,
  kInt64List,
  kFloat64List,
  kList,
  kMap,
  kFloat32List,
};

// Returns the encoded type that should be written when serializing |value|.
EncodedType EncodedTypeForValue(const EncodableValue& value) {
  switch (value.index()) {
    case 0:
      return EncodedType::kNull;
    case 1:
      return std::get<bool>(value) ? EncodedType::kTrue : EncodedType::kFalse;
    case 2:
      return EncodedType::kInt32;
    case 3:
      return EncodedType::kInt64;
    case 4:
      return EncodedType::kFloat64;
    case 5:
      return EncodedType::kString;
    case 6:
      return EncodedType::kUInt8List;
    case 7:
      return EncodedType::kInt32List;
    case 8:
      return EncodedType::kInt64List;
    case 9:
      return EncodedType::kFloat64List;
    case 10:
      return EncodedType::kList;
    case 11:
      return EncodedType::kMap;
    case 13:
      return EncodedType::kFloat32List;
  }
  assert(false);
  return EncodedType::kNull;
}

}  // namespace

StandardCodecSerializer::StandardCodecSerializer() = default;

StandardCodecSerializer::~StandardCodecSerializer() = default;

const StandardCodecSerializer& StandardCodecSerializer::GetInstance() {
  static StandardCodecSerializer sInstance;
  return sInstance;
};

EncodableValue StandardCodecSerializer::ReadValue(
    ByteStreamReader* stream) const {
  uint8_t type = stream->ReadByte();
  return ReadValueOfType(type, stream);
}

void StandardCodecSerializer::WriteValue(const EncodableValue& value,
                                         ByteStreamWriter* stream) const {
  stream->WriteByte(static_cast<uint8_t>(EncodedTypeForValue(value)));
  // TODO(cbracken): Consider replacing this with std::visit.
  switch (value.index()) {
    case 0:
    case 1:
      // Null and bool are encoded directly in the type.
      break;
    case 2:
      stream->WriteInt32(std::get<int32_t>(value));
      break;
    case 3:
      stream->WriteInt64(std::get<int64_t>(value));
      break;
    case 4:
      stream->WriteAlignment(8);
      stream->WriteDouble(std::get<double>(value));
      break;
    case 5: {
      const auto& string_value = std::get<std::string>(value);
      size_t size = string_value.size();
      WriteSize(size, stream);
      if (size > 0) {
        stream->WriteBytes(
            reinterpret_cast<const uint8_t*>(string_value.data()), size);
      }
      break;
    }
    case 6:
      WriteVector(std::get<std::vector<uint8_t>>(value), stream);
      break;
    case 7:
      WriteVector(std::get<std::vector<int32_t>>(value), stream);
      break;
    case 8:
      WriteVector(std::get<std::vector<int64_t>>(value), stream);
      break;
    case 9:
      WriteVector(std::get<std::vector<double>>(value), stream);
      break;
    case 10: {
      const auto& list = std::get<EncodableList>(value);
      WriteSize(list.size(), stream);
      for (const auto& item : list) {
        WriteValue(item, stream);
      }
      break;
    }
    case 11: {
      const auto& map = std::get<EncodableMap>(value);
      WriteSize(map.size(), stream);
      for (const auto& pair : map) {
        WriteValue(pair.first, stream);
        WriteValue(pair.second, stream);
      }
      break;
    }
    case 12:
      std::cerr
          << "Unhandled custom type in StandardCodecSerializer::WriteValue. "
          << "Custom types require codec extensions." << std::endl;
      break;
    case 13: {
      WriteVector(std::get<std::vector<float>>(value), stream);
      break;
    }
  }
}

EncodableValue StandardCodecSerializer::ReadValueOfType(
    uint8_t type,
    ByteStreamReader* stream) const {
  switch (static_cast<EncodedType>(type)) {
    case EncodedType::kNull:
      return EncodableValue();
    case EncodedType::kTrue:
      return EncodableValue(true);
    case EncodedType::kFalse:
      return EncodableValue(false);
    case EncodedType::kInt32:
      return EncodableValue(stream->ReadInt32());
    case EncodedType::kInt64:
      return EncodableValue(stream->ReadInt64());
    case EncodedType::kFloat64:
      stream->ReadAlignment(8);
      return EncodableValue(stream->ReadDouble());
    case EncodedType::kLargeInt:
    case EncodedType::kString: {
      size_t size = ReadSize(stream);
      std::string string_value;
      string_value.resize(size);
      stream->ReadBytes(reinterpret_cast<uint8_t*>(&string_value[0]), size);
      return EncodableValue(string_value);
    }
    case EncodedType::kUInt8List:
      return ReadVector<uint8_t>(stream);
    case EncodedType::kInt32List:
      return ReadVector<int32_t>(stream);
    case EncodedType::kInt64List:
      return ReadVector<int64_t>(stream);
    case EncodedType::kFloat64List:
      return ReadVector<double>(stream);
    case EncodedType::kList: {
      size_t length = ReadSize(stream);
      EncodableList list_value;
      list_value.reserve(length);
      for (size_t i = 0; i < length; ++i) {
        list_value.push_back(ReadValue(stream));
      }
      return EncodableValue(list_value);
    }
    case EncodedType::kMap: {
      size_t length = ReadSize(stream);
      EncodableMap map_value;
      for (size_t i = 0; i < length; ++i) {
        EncodableValue key = ReadValue(stream);
        EncodableValue value = ReadValue(stream);
        map_value.emplace(std::move(key), std::move(value));
      }
      return EncodableValue(map_value);
    }
    case EncodedType::kFloat32List: {
      return ReadVector<float>(stream);
    }
  }
  std::cerr << "Unknown type in StandardCodecSerializer::ReadValueOfType: "
            << static_cast<int>(type) << std::endl;
  return EncodableValue();
}

size_t StandardCodecSerializer::ReadSize(ByteStreamReader* stream) const {
  uint8_t byte = stream->ReadByte();
  if (byte < 254) {
    return byte;
  } else if (byte == 254) {
    uint16_t value = 0;
    stream->ReadBytes(reinterpret_cast<uint8_t*>(&value), 2);
    return value;
  } else {
    uint32_t value = 0;
    stream->ReadBytes(reinterpret_cast<uint8_t*>(&value), 4);
    return value;
  }
}

void StandardCodecSerializer::WriteSize(size_t size,
                                        ByteStreamWriter* stream) const {
  if (size < 254) {
    stream->WriteByte(static_cast<uint8_t>(size));
  } else if (size <= 0xffff) {
    stream->WriteByte(254);
    uint16_t value = static_cast<uint16_t>(size);
    stream->WriteBytes(reinterpret_cast<uint8_t*>(&value), 2);
  } else {
    stream->WriteByte(255);
    uint32_t value = static_cast<uint32_t>(size);
    stream->WriteBytes(reinterpret_cast<uint8_t*>(&value), 4);
  }
}

template <typename T>
EncodableValue StandardCodecSerializer::ReadVector(
    ByteStreamReader* stream) const {
  size_t count = ReadSize(stream);
  std::vector<T> vector;
  vector.resize(count);
  uint8_t type_size = static_cast<uint8_t>(sizeof(T));
  if (type_size > 1) {
    stream->ReadAlignment(type_size);
  }
  stream->ReadBytes(reinterpret_cast<uint8_t*>(vector.data()),
                    count * type_size);
  return EncodableValue(vector);
}

template <typename T>
void StandardCodecSerializer::WriteVector(const std::vector<T> vector,
                                          ByteStreamWriter* stream) const {
  size_t count = vector.size();
  WriteSize(count, stream);
  if (count == 0) {
    return;
  }
  uint8_t type_size = static_cast<uint8_t>(sizeof(T));
  if (type_size > 1) {
    stream->WriteAlignment(type_size);
  }
  stream->WriteBytes(reinterpret_cast<const uint8_t*>(vector.data()),
                     count * type_size);
}

// ===== standard_message_codec.h =====

// static
const StandardMessageCodec& StandardMessageCodec::GetInstance(
    const StandardCodecSerializer* serializer) {
  if (!serializer) {
    serializer = &StandardCodecSerializer::GetInstance();
  }
  static auto* sInstances = new std::map<const StandardCodecSerializer*,
                                         std::unique_ptr<StandardMessageCodec>>;
  auto it = sInstances->find(serializer);
  if (it == sInstances->end()) {
    // Uses new due to private constructor (to prevent API clients from
    // accidentally passing temporary codec instances to channels).
    auto emplace_result = sInstances->emplace(
        serializer, std::unique_ptr<StandardMessageCodec>(
                        new StandardMessageCodec(serializer)));
    it = emplace_result.first;
  }
  return *(it->second);
}

StandardMessageCodec::StandardMessageCodec(
    const StandardCodecSerializer* serializer)
    : serializer_(serializer) {}

StandardMessageCodec::~StandardMessageCodec() = default;

std::unique_ptr<EncodableValue> StandardMessageCodec::DecodeMessageInternal(
    const uint8_t* binary_message,
    size_t message_size) const {
  if (!binary_message) {
    return std::make_unique<EncodableValue>();
  }
  ByteBufferStreamReader stream(binary_message, message_size);
  return std::make_unique<EncodableValue>(serializer_->ReadValue(&stream));
}

std::unique_ptr<std::vector<uint8_t>>
StandardMessageCodec::EncodeMessageInternal(
    const EncodableValue& message) const {
  auto encoded = std::make_unique<std::vector<uint8_t>>();
  ByteBufferStreamWriter stream(encoded.get());
  serializer_->WriteValue(message, &stream);
  return encoded;
}

// ===== standard_method_codec.h =====

// static
const StandardMethodCodec& StandardMethodCodec::GetInstance(
    const StandardCodecSerializer* serializer) {
  if (!serializer) {
    serializer = &StandardCodecSerializer::GetInstance();
  }
  static auto* sInstances = new std::map<const StandardCodecSerializer*,
                                         std::unique_ptr<StandardMethodCodec>>;
  auto it = sInstances->find(serializer);
  if (it == sInstances->end()) {
    // Uses new due to private constructor (to prevent API clients from
    // accidentally passing temporary codec instances to channels).
    auto emplace_result = sInstances->emplace(
        serializer, std::unique_ptr<StandardMethodCodec>(
                        new StandardMethodCodec(serializer)));
    it = emplace_result.first;
  }
  return *(it->second);
}

StandardMethodCodec::StandardMethodCodec(
    const StandardCodecSerializer* serializer)
    : serializer_(serializer) {}

StandardMethodCodec::~StandardMethodCodec() = default;

std::unique_ptr<MethodCall<EncodableValue>>
StandardMethodCodec::DecodeMethodCallInternal(const uint8_t* message,
                                              size_t message_size) const {
  ByteBufferStreamReader stream(message, message_size);
  EncodableValue method_name_value = serializer_->ReadValue(&stream);
  const auto* method_name = std::get_if<std::string>(&method_name_value);
  if (!method_name) {
    std::cerr << "Invalid method call; method name is not a string."
              << std::endl;
    return nullptr;
  }
  auto arguments =
      std::make_unique<EncodableValue>(serializer_->ReadValue(&stream));
  return std::make_unique<MethodCall<EncodableValue>>(*method_name,
                                                      std::move(arguments));
}

std::unique_ptr<std::vector<uint8_t>>
StandardMethodCodec::EncodeMethodCallInternal(
    const MethodCall<EncodableValue>& method_call) const {
  auto encoded = std::make_unique<std::vector<uint8_t>>();
  ByteBufferStreamWriter stream(encoded.get());
  serializer_->WriteValue(EncodableValue(method_call.method_name()), &stream);
  if (method_call.arguments()) {
    serializer_->WriteValue(*method_call.arguments(), &stream);
  } else {
    serializer_->WriteValue(EncodableValue(), &stream);
  }
  return encoded;
}

std::unique_ptr<std::vector<uint8_t>>
StandardMethodCodec::EncodeSuccessEnvelopeInternal(
    const EncodableValue* result) const {
  auto encoded = std::make_unique<std::vector<uint8_t>>();
  ByteBufferStreamWriter stream(encoded.get());
  stream.WriteByte(0);
  if (result) {
    serializer_->WriteValue(*result, &stream);
  } else {
    serializer_->WriteValue(EncodableValue(), &stream);
  }
  return encoded;
}

std::unique_ptr<std::vector<uint8_t>>
StandardMethodCodec::EncodeErrorEnvelopeInternal(
    const std::string& error_code,
    const std::string& error_message,
    const EncodableValue* error_details) const {
  auto encoded = std::make_unique<std::vector<uint8_t>>();
  ByteBufferStreamWriter stream(encoded.get());
  stream.WriteByte(1);
  serializer_->WriteValue(EncodableValue(error_code), &stream);
  if (error_message.empty()) {
    serializer_->WriteValue(EncodableValue(), &stream);
  } else {
    serializer_->WriteValue(EncodableValue(error_message), &stream);
  }
  if (error_details) {
    serializer_->WriteValue(*error_details, &stream);
  } else {
    serializer_->WriteValue(EncodableValue(), &stream);
  }
  return encoded;
}

bool StandardMethodCodec::DecodeAndProcessResponseEnvelopeInternal(
    const uint8_t* response,
    size_t response_size,
    MethodResult<EncodableValue>* result) const {
  ByteBufferStreamReader stream(response, response_size);
  uint8_t flag = stream.ReadByte();
  switch (flag) {
    case 0: {
      EncodableValue value = serializer_->ReadValue(&stream);
      if (value.IsNull()) {
        result->Success();
      } else {
        result->Success(value);
      }
      return true;
    }
    case 1: {
      EncodableValue code = serializer_->ReadValue(&stream);
      EncodableValue message = serializer_->ReadValue(&stream);
      EncodableValue details = serializer_->ReadValue(&stream);
      const std::string& message_string =
          message.IsNull() ? "" : std::get<std::string>(message);
      if (details.IsNull()) {
        result->Error(std::get<std::string>(code), message_string);
      } else {
        result->Error(std::get<std::string>(code), message_string, details);
      }
      return true;
    }
    default:
      return false;
  }
}

}  // namespace flutter
