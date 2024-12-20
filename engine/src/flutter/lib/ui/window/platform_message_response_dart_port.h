// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_PLATFORM_MESSAGE_RESPONSE_DART_PORT_H_
#define FLUTTER_LIB_UI_WINDOW_PLATFORM_MESSAGE_RESPONSE_DART_PORT_H_

#include "flutter/fml/message_loop.h"
#include "flutter/lib/ui/window/platform_message_response.h"
#include "third_party/tonic/dart_persistent_value.h"

namespace flutter {

/// A \ref PlatformMessageResponse that will respond over a Dart port.
class PlatformMessageResponseDartPort : public PlatformMessageResponse {
  FML_FRIEND_MAKE_REF_COUNTED(PlatformMessageResponseDartPort);

 public:
  // Callable on any thread.
  void Complete(std::unique_ptr<fml::Mapping> data) override;
  void CompleteEmpty() override;

 protected:
  explicit PlatformMessageResponseDartPort(Dart_Port send_port,
                                           int64_t identifier,
                                           const std::string& channel);

  Dart_Port send_port_;
  int64_t identifier_;
  const std::string channel_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_PLATFORM_MESSAGE_RESPONSE_DART_PORT_H_
