// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_METHOD_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_METHOD_CHANNEL_H_

#include <iostream>
#include <string>

#include "basic_message_channel.h"
#include "binary_messenger.h"
#include "engine_method_result.h"
#include "method_call.h"
#include "method_codec.h"
#include "method_result.h"

namespace flutter {

class EncodableValue;

// A handler for receiving a method call from the Flutter engine.
//
// Implementations must asynchronously call exactly one of the methods on
// |result| to indicate the result of the method call.
template <typename T>
using MethodCallHandler =
    std::function<void(const MethodCall<T>& call,
                       std::unique_ptr<MethodResult<T>> result)>;

// A channel for communicating with the Flutter engine using invocation of
// asynchronous methods.
template <typename T = EncodableValue>
class MethodChannel {
 public:
  // Creates an instance that sends and receives method calls on the channel
  // named |name|, encoded with |codec| and dispatched via |messenger|.
  MethodChannel(BinaryMessenger* messenger,
                const std::string& name,
                const MethodCodec<T>* codec)
      : messenger_(messenger), name_(name), codec_(codec) {}

  ~MethodChannel() = default;

  // Prevent copying.
  MethodChannel(MethodChannel const&) = delete;
  MethodChannel& operator=(MethodChannel const&) = delete;

  // Sends a message to the Flutter engine on this channel.
  //
  // If |result| is provided, one of its methods will be invoked with the
  // response from the engine.
  void InvokeMethod(const std::string& method,
                    std::unique_ptr<T> arguments,
                    std::unique_ptr<MethodResult<T>> result = nullptr) {
    MethodCall<T> method_call(method, std::move(arguments));
    std::unique_ptr<std::vector<uint8_t>> message =
        codec_->EncodeMethodCall(method_call);
    if (!result) {
      messenger_->Send(name_, message->data(), message->size(), nullptr);
      return;
    }

    // std::function requires a copyable lambda, so convert to a shared pointer.
    // This is safe since only one copy of the shared_pointer will ever be
    // accessed.
    std::shared_ptr<MethodResult<T>> shared_result(result.release());
    const auto* codec = codec_;
    std::string channel_name = name_;
    BinaryReply reply_handler = [shared_result, codec, channel_name](
                                    const uint8_t* reply, size_t reply_size) {
      if (reply_size == 0) {
        shared_result->NotImplemented();
        return;
      }
      // Use this channel's codec to decode and handle the
      // reply.
      bool decoded = codec->DecodeAndProcessResponseEnvelope(
          reply, reply_size, shared_result.get());
      if (!decoded) {
        std::cerr << "Unable to decode reply to method "
                     "invocation on channel "
                  << channel_name << std::endl;
        shared_result->NotImplemented();
      }
    };

    messenger_->Send(name_, message->data(), message->size(),
                     std::move(reply_handler));
  }

  // Registers a handler that should be called any time a method call is
  // received on this channel. A null handler will remove any previous handler.
  //
  // The handler will be owned by the underlying BinaryMessageHandler.
  // Destroying the MethodChannel will not unregister the handler, so
  // the caller is responsible for unregistering explicitly if the handler
  // stops being valid before the engine is destroyed.
  void SetMethodCallHandler(MethodCallHandler<T> handler) const {
    if (!handler) {
      messenger_->SetMessageHandler(name_, nullptr);
      return;
    }
    const auto* codec = codec_;
    std::string channel_name = name_;
    BinaryMessageHandler binary_handler = [handler, codec, channel_name](
                                              const uint8_t* message,
                                              size_t message_size,
                                              BinaryReply reply) {
      // Use this channel's codec to decode the call and build a result handler.
      auto result =
          std::make_unique<EngineMethodResult<T>>(std::move(reply), codec);
      std::unique_ptr<MethodCall<T>> method_call =
          codec->DecodeMethodCall(message, message_size);
      if (!method_call) {
        std::cerr << "Unable to construct method call from message on channel "
                  << channel_name << std::endl;
        result->NotImplemented();
        return;
      }
      handler(*method_call, std::move(result));
    };
    messenger_->SetMessageHandler(name_, std::move(binary_handler));
  }

  // Adjusts the number of messages that will get buffered when sending messages
  // to channels that aren't fully set up yet. For example, the engine isn't
  // running yet or the channel's message handler isn't set up on the Dart side
  // yet.
  void Resize(int new_size) {
    internal::ResizeChannel(messenger_, name_, new_size);
  }

  // Defines whether the channel should show warning messages when discarding
  // messages due to overflow.
  //
  // When |warns| is false, the channel is expected to overflow and warning
  // messages will not be shown.
  void SetWarnsOnOverflow(bool warns) {
    internal::SetChannelWarnsOnOverflow(messenger_, name_, warns);
  }

 private:
  BinaryMessenger* messenger_;
  std::string name_;
  const MethodCodec<T>* codec_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_METHOD_CHANNEL_H_
