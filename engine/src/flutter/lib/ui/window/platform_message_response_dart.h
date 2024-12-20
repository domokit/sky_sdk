// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_PLATFORM_MESSAGE_RESPONSE_DART_H_
#define FLUTTER_LIB_UI_WINDOW_PLATFORM_MESSAGE_RESPONSE_DART_H_

#include "flutter/fml/message_loop.h"
#include "flutter/lib/ui/window/platform_message_response.h"
#include "third_party/tonic/dart_persistent_value.h"

namespace flutter {

class PlatformMessageResponseDart : public PlatformMessageResponse {
  FML_FRIEND_MAKE_REF_COUNTED(PlatformMessageResponseDart);

 public:
  // Callable on any thread.
  void Complete(std::unique_ptr<fml::Mapping> data) override;
  void CompleteEmpty() override;

 protected:
  explicit PlatformMessageResponseDart(
      tonic::DartPersistentValue callback,
      fml::RefPtr<fml::TaskRunner> ui_task_runner,
      const std::string& channel);
  ~PlatformMessageResponseDart() override;

  tonic::DartPersistentValue callback_;
  fml::RefPtr<fml::TaskRunner> ui_task_runner_;
  const std::string channel_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_PLATFORM_MESSAGE_RESPONSE_DART_H_
