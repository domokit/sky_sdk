// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_CHANNEL_H_

#include <iostream>
#include <memory>
#include <string>

#include "binary_messenger.h"
#include "engine_method_result.h"
#include "event_sink.h"
#include "event_stream_handler.h"

namespace flutter {

class EncodableValue;

// A named channel for communicating with the Flutter application using
// asynchronous event streams. Incoming requests for event stream setup are
// decoded from binary on receipt, and C++ responses and events are encoded into
// binary before being transmitted back to Flutter. The MethodCodec used must be
// compatible with the one used by the Flutter application. This can be achieved
// by creating an EventChannel
// ("https://api.flutter.dev/flutter/services/EventChannel-class.html")
// counterpart of this channel on the Dart side.
// The C++ type of stream configuration arguments, events, and error details are
// templated, but only values supported by the specified MethodCodec can be
// used.
template <typename T = EncodableValue>
class EventChannel {
 public:
  // Creates an instance that sends and receives event handler on the channel
  // named |name|, encoded with |codec| and dispatched via |messenger|.
  EventChannel(BinaryMessenger* messenger,
               const std::string& name,
               const MethodCodec<T>* codec)
      : messenger_(messenger), name_(name), codec_(codec) {}
  ~EventChannel() = default;

  // Prevent copying.
  EventChannel(EventChannel const&) = delete;
  EventChannel& operator=(EventChannel const&) = delete;

  // Registers a stream handler on this channel.
  // If no handler has been registered, any incoming stream setup requests will
  // be handled silently by providing an empty stream.
  //
  // Note that the EventChannel does not own the handler and will not
  // unregister it on destruction. The caller is responsible for unregistering
  // the handler if it should no longer be called.
  void SetStreamHandler(std::unique_ptr<StreamHandler<T>> handler) {
    if (!handler) {
      messenger_->SetMessageHandler(name_, nullptr);
      return;
    }

    // std::function requires a copyable lambda, so convert to a shared pointer.
    // This is safe since only one copy of the shared_pointer will ever be
    // accessed.
    std::shared_ptr<StreamHandler<T>> shared_handler(handler.release());
    const MethodCodec<T>* codec = codec_;
    const std::string channel_name = name_;
    const BinaryMessenger* messenger = messenger_;
    BinaryMessageHandler binary_handler =
        [shared_handler, codec, channel_name, messenger,
         // Mutable state to track the handler's listening status.
         is_listening = false](const uint8_t* message,
                               const size_t message_size,
                               const BinaryReply& reply) mutable {
          constexpr char kOnListenMethod[] = "listen";
          constexpr char kOnCancelMethod[] = "cancel";

          std::unique_ptr<MethodCall<T>> method_call =
              codec->DecodeMethodCall(message, message_size);
          if (!method_call) {
            std::cerr
                << "Unable to construct method call from message on channel: "
                << channel_name << std::endl;
            reply(nullptr, 0);
            return;
          }

          const std::string& method = method_call->method_name();
          if (method.compare(kOnListenMethod) == 0) {
            if (is_listening) {
              std::unique_ptr<StreamHandlerError<T>> error =
                  shared_handler->OnCancel(nullptr);
              if (error) {
                std::cerr << "Failed to cancel existing stream: "
                          << (error->error_code) << ", "
                          << (error->error_message) << ", "
                          << (error->error_details);
              }
            }
            is_listening = true;

            std::unique_ptr<std::vector<uint8_t>> result;
            auto sink = std::make_unique<EventSinkImplementation>(
                messenger, channel_name, codec);
            std::unique_ptr<StreamHandlerError<T>> error =
                shared_handler->OnListen(method_call->arguments(),
                                         std::move(sink));
            if (error) {
              result = codec->EncodeErrorEnvelope(error->error_code,
                                                  error->error_message,
                                                  error->error_details.get());
            } else {
              result = codec->EncodeSuccessEnvelope();
            }
            reply(result->data(), result->size());
          } else if (method.compare(kOnCancelMethod) == 0) {
            std::unique_ptr<std::vector<uint8_t>> result;
            if (is_listening) {
              std::unique_ptr<StreamHandlerError<T>> error =
                  shared_handler->OnCancel(method_call->arguments());
              if (error) {
                result = codec->EncodeErrorEnvelope(error->error_code,
                                                    error->error_message,
                                                    error->error_details.get());
              } else {
                result = codec->EncodeSuccessEnvelope();
              }
              is_listening = false;
            } else {
              result = codec->EncodeErrorEnvelope(
                  "error", "No active stream to cancel", nullptr);
            }
            reply(result->data(), result->size());
          } else {
            reply(nullptr, 0);
          }
        };
    messenger_->SetMessageHandler(name_, std::move(binary_handler));
  }

 private:
  class EventSinkImplementation : public EventSink<T> {
   public:
    EventSinkImplementation(const BinaryMessenger* messenger,
                            const std::string& name,
                            const MethodCodec<T>* codec)
        : messenger_(messenger), name_(name), codec_(codec) {}
    ~EventSinkImplementation() = default;

    // Prevent copying.
    EventSinkImplementation(EventSinkImplementation const&) = delete;
    EventSinkImplementation& operator=(EventSinkImplementation const&) = delete;

   private:
    const BinaryMessenger* messenger_;
    const std::string name_;
    const MethodCodec<T>* codec_;

   protected:
    void SuccessInternal(const T* event = nullptr) override {
      auto result = codec_->EncodeSuccessEnvelope(event);
      messenger_->Send(name_, result->data(), result->size());
    }

    void ErrorInternal(const std::string& error_code,
                       const std::string& error_message,
                       const T* error_details) override {
      auto result =
          codec_->EncodeErrorEnvelope(error_code, error_message, error_details);
      messenger_->Send(name_, result->data(), result->size());
    }

    void EndOfStreamInternal() override { messenger_->Send(name_, nullptr, 0); }
  };

  BinaryMessenger* messenger_;
  const std::string name_;
  const MethodCodec<T>* codec_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_CHANNEL_H_
