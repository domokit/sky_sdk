// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_PLATFORM_MESSAGE_RESPONSE_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_PLATFORM_MESSAGE_RESPONSE_H_

#include "flutter/fml/macros.h"
#include "flutter/fml/task_runner.h"
#include "flutter/lib/ui/window/platform_message_response.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      The platform message response subclass for responses to messages
///             from the embedder to the framework. Message responses are
///             fulfilled by the framework.
class EmbedderPlatformMessageResponse : public PlatformMessageResponse {
 public:
  using Callback = std::function<void(const uint8_t* data, size_t size)>;

  //----------------------------------------------------------------------------
  /// @param[in]  runner    The task runner on which to execute the callback.
  ///                       The response will be initiated by the framework on
  ///                       the UI thread.
  /// @param[in]  callback  The callback that communicates to the embedder the
  ///                       contents of the response sent by the framework back
  ///                       to the emebder.
  EmbedderPlatformMessageResponse(fml::RefPtr<fml::TaskRunner> runner,
                                  const Callback& callback);

  //----------------------------------------------------------------------------
  /// @brief      Destroys the message response. Can be called on any thread.
  ///             Does not execute unfulfilled callbacks.
  ///
  ~EmbedderPlatformMessageResponse() override;

 private:
  fml::RefPtr<fml::TaskRunner> runner_;
  Callback callback_;

  // |PlatformMessageResponse|
  void Complete(std::unique_ptr<fml::Mapping> data) override;

  // |PlatformMessageResponse|
  void CompleteEmpty() override;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderPlatformMessageResponse);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_PLATFORM_MESSAGE_RESPONSE_H_
