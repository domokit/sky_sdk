// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_STANDARD_MESSAGE_CODEC_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_STANDARD_MESSAGE_CODEC_H_

#include <memory>

#include "encodable_value.h"
#include "message_codec.h"
#include "standard_codec_serializer.h"

namespace flutter {

// A binary message encoding/decoding mechanism for communications to/from the
// Flutter engine via message channels.
class StandardMessageCodec : public MessageCodec<EncodableValue> {
 public:
  // Returns an instance of the codec, optionally using a custom serializer to
  // add support for more types.
  //
  // If provided, |serializer| must be long-lived. If no serializer is provided,
  // the default will be used.
  //
  // The instance returned for a given |serializer| will be shared, and
  // any instance returned from this will be long-lived, and can be safely
  // passed to, e.g., channel constructors.
  static const StandardMessageCodec& GetInstance(
      const StandardCodecSerializer* serializer = nullptr);

  ~StandardMessageCodec();

  // Prevent copying.
  StandardMessageCodec(StandardMessageCodec const&) = delete;
  StandardMessageCodec& operator=(StandardMessageCodec const&) = delete;

 protected:
  // |flutter::MessageCodec|
  std::unique_ptr<EncodableValue> DecodeMessageInternal(
      const uint8_t* binary_message,
      const size_t message_size) const override;

  // |flutter::MessageCodec|
  std::unique_ptr<std::vector<uint8_t>> EncodeMessageInternal(
      const EncodableValue& message) const override;

 private:
  // Instances should be obtained via GetInstance.
  explicit StandardMessageCodec(const StandardCodecSerializer* serializer);

  const StandardCodecSerializer* serializer_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_STANDARD_MESSAGE_CODEC_H_
