// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_JSON_METHOD_CODEC_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_JSON_METHOD_CODEC_H_

#include <rapidjson/document.h>

#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_call.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_codec.h"

namespace flutter {

// An implementation of MethodCodec that uses JSON strings as the serialization.
class JsonMethodCodec : public MethodCodec<rapidjson::Document> {
 public:
  // Returns the shared instance of the codec.
  static const JsonMethodCodec& GetInstance();

  ~JsonMethodCodec() = default;

  // Prevent copying.
  JsonMethodCodec(JsonMethodCodec const&) = delete;
  JsonMethodCodec& operator=(JsonMethodCodec const&) = delete;

 protected:
  // Instances should be obtained via GetInstance.
  JsonMethodCodec() = default;

  // |flutter::MethodCodec|
  std::unique_ptr<MethodCall<rapidjson::Document>> DecodeMethodCallInternal(
      const uint8_t* message,
      const size_t message_size) const override;

  // |flutter::MethodCodec|
  std::unique_ptr<std::vector<uint8_t>> EncodeMethodCallInternal(
      const MethodCall<rapidjson::Document>& method_call) const override;

  // |flutter::MethodCodec|
  std::unique_ptr<std::vector<uint8_t>> EncodeSuccessEnvelopeInternal(
      const rapidjson::Document* result) const override;

  // |flutter::MethodCodec|
  std::unique_ptr<std::vector<uint8_t>> EncodeErrorEnvelopeInternal(
      const std::string& error_code,
      const std::string& error_message,
      const rapidjson::Document* error_details) const override;

  // |flutter::MethodCodec|
  bool DecodeAndProcessResponseEnvelopeInternal(
      const uint8_t* response,
      const size_t response_size,
      MethodResult<rapidjson::Document>* result) const override;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_JSON_METHOD_CODEC_H_
