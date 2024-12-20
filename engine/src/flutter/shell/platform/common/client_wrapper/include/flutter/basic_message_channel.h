// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_BASIC_MESSAGE_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_BASIC_MESSAGE_CHANNEL_H_

#include <iostream>
#include <string>
#include <utility>

#include "binary_messenger.h"
#include "message_codec.h"

namespace flutter {

namespace internal {
// Internal helper functions used by BasicMessageChannel and MethodChannel.

// Adjusts the number of messages that will get buffered when sending messages
// to channels that aren't fully set up yet. For example, the engine isn't
// running yet or the channel's message handler isn't set up on the Dart side
// yet.
void ResizeChannel(BinaryMessenger* messenger, std::string name, int new_size);

// Defines whether the channel should show warning messages when discarding
// messages due to overflow.
//
// When |warns| is false, the channel is expected to overflow and warning
// messages will not be shown.
void SetChannelWarnsOnOverflow(BinaryMessenger* messenger,
                               std::string name,
                               bool warns);

}  // namespace internal

class EncodableValue;

// A message reply callback.
//
// Used for submitting a reply back to a Flutter message sender.
template <typename T>
using MessageReply = std::function<void(const T& reply)>;

// A handler for receiving a message from the Flutter engine.
//
// Implementations must asynchronously call reply exactly once with the reply
// to the message.
template <typename T>
using MessageHandler =
    std::function<void(const T& message, const MessageReply<T>& reply)>;

// A channel for communicating with the Flutter engine by sending asynchronous
// messages.
template <typename T = EncodableValue>
class BasicMessageChannel {
 public:
  // Creates an instance that sends and receives method calls on the channel
  // named |name|, encoded with |codec| and dispatched via |messenger|.
  BasicMessageChannel(BinaryMessenger* messenger,
                      const std::string& name,
                      const MessageCodec<T>* codec)
      : messenger_(messenger), name_(name), codec_(codec) {}

  ~BasicMessageChannel() = default;

  // Prevent copying.
  BasicMessageChannel(BasicMessageChannel const&) = delete;
  BasicMessageChannel& operator=(BasicMessageChannel const&) = delete;

  // Sends a message to the Flutter engine on this channel.
  void Send(const T& message) {
    std::unique_ptr<std::vector<uint8_t>> raw_message =
        codec_->EncodeMessage(message);
    messenger_->Send(name_, raw_message->data(), raw_message->size());
  }

  // Sends a message to the Flutter engine on this channel expecting a reply.
  void Send(const T& message, BinaryReply reply) {
    std::unique_ptr<std::vector<uint8_t>> raw_message =
        codec_->EncodeMessage(message);
    messenger_->Send(name_, raw_message->data(), raw_message->size(),
                     std::move(reply));
  }

  // Registers a handler that should be called any time a message is
  // received on this channel. A null handler will remove any previous handler.
  //
  // Note that the BasicMessageChannel does not own the handler, and will not
  // unregister it on destruction, so the caller is responsible for
  // unregistering explicitly if it should no longer be called.
  void SetMessageHandler(const MessageHandler<T>& handler) const {
    if (!handler) {
      messenger_->SetMessageHandler(name_, nullptr);
      return;
    }
    const auto* codec = codec_;
    std::string channel_name = name_;
    BinaryMessageHandler binary_handler = [handler, codec, channel_name](
                                              const uint8_t* binary_message,
                                              const size_t binary_message_size,
                                              const BinaryReply& binary_reply) {
      // Use this channel's codec to decode the message and build a reply
      // handler.
      std::unique_ptr<T> message =
          codec->DecodeMessage(binary_message, binary_message_size);
      if (!message) {
        std::cerr << "Unable to decode message on channel " << channel_name
                  << std::endl;
        binary_reply(nullptr, 0);
        return;
      }

      MessageReply<T> unencoded_reply = [binary_reply,
                                         codec](const T& unencoded_response) {
        auto binary_response = codec->EncodeMessage(unencoded_response);
        binary_reply(binary_response->data(), binary_response->size());
      };
      handler(*message, std::move(unencoded_reply));
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
  const MessageCodec<T>* codec_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_BASIC_MESSAGE_CHANNEL_H_
