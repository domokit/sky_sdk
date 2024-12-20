// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/include/flutter/event_channel.h"

#include <memory>
#include <string>

#include "flutter/shell/platform/common/client_wrapper/include/flutter/binary_messenger.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/event_stream_handler_functions.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

class TestBinaryMessenger : public BinaryMessenger {
 public:
  void Send(const std::string& channel,
            const uint8_t* message,
            const size_t message_size,
            BinaryReply reply) const override {}

  void SetMessageHandler(const std::string& channel,
                         BinaryMessageHandler handler) override {
    last_message_handler_channel_ = channel;
    last_message_handler_ = handler;
  }

  std::string last_message_handler_channel() {
    return last_message_handler_channel_;
  }

  const BinaryMessageHandler& last_message_handler() {
    return last_message_handler_;
  }

 private:
  std::string last_message_handler_channel_;
  BinaryMessageHandler last_message_handler_;
};

}  // namespace

// Tests that SetStreamHandler sets a handler that correctly interacts with
// the binary messenger.
TEST(EventChannelTest, Registration) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  const StandardMethodCodec& codec = StandardMethodCodec::GetInstance();
  EventChannel channel(&messenger, channel_name, &codec);

  bool on_listen_called = false;
  auto handler = std::make_unique<StreamHandlerFunctions<>>(
      [&on_listen_called](const EncodableValue* arguments,
                          std::unique_ptr<EventSink<>>&& events)
          -> std::unique_ptr<StreamHandlerError<>> {
        on_listen_called = true;
        return nullptr;
      },
      [](const EncodableValue* arguments)
          -> std::unique_ptr<StreamHandlerError<>> { return nullptr; });
  channel.SetStreamHandler(std::move(handler));
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);

  // Send test listen message.
  MethodCall<> call("listen", nullptr);
  auto message = codec.EncodeMethodCall(call);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});

  // Check results.
  EXPECT_EQ(on_listen_called, true);
}

// Tests that SetStreamHandler with a null handler unregisters the handler.
TEST(EventChannelTest, Unregistration) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  const StandardMethodCodec& codec = StandardMethodCodec::GetInstance();
  EventChannel channel(&messenger, channel_name, &codec);

  auto handler = std::make_unique<StreamHandlerFunctions<>>(
      [](const EncodableValue* arguments, std::unique_ptr<EventSink<>>&& events)
          -> std::unique_ptr<StreamHandlerError<>> { return nullptr; },
      [](const EncodableValue* arguments)
          -> std::unique_ptr<StreamHandlerError<>> { return nullptr; });
  channel.SetStreamHandler(std::move(handler));
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);

  channel.SetStreamHandler(nullptr);
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_EQ(messenger.last_message_handler(), nullptr);
}

// Test that OnCancel callback sequence.
TEST(EventChannelTest, Cancel) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  const StandardMethodCodec& codec = StandardMethodCodec::GetInstance();
  EventChannel channel(&messenger, channel_name, &codec);

  bool on_listen_called = false;
  bool on_cancel_called = false;
  auto handler = std::make_unique<StreamHandlerFunctions<>>(
      [&on_listen_called](const EncodableValue* arguments,
                          std::unique_ptr<EventSink<>>&& events)
          -> std::unique_ptr<StreamHandlerError<>> {
        on_listen_called = true;
        return nullptr;
      },
      [&on_cancel_called](const EncodableValue* arguments)
          -> std::unique_ptr<StreamHandlerError<>> {
        on_cancel_called = true;
        return nullptr;
      });
  channel.SetStreamHandler(std::move(handler));
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);

  // Send test listen message.
  MethodCall<> call_listen("listen", nullptr);
  auto message = codec.EncodeMethodCall(call_listen);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});
  EXPECT_EQ(on_listen_called, true);

  // Send test cancel message.
  MethodCall<> call_cancel("cancel", nullptr);
  message = codec.EncodeMethodCall(call_cancel);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});

  // Check results.
  EXPECT_EQ(on_cancel_called, true);
}

// Tests that OnCancel in not called on registration.
TEST(EventChannelTest, ListenNotCancel) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  const StandardMethodCodec& codec = StandardMethodCodec::GetInstance();
  EventChannel channel(&messenger, channel_name, &codec);

  bool on_listen_called = false;
  bool on_cancel_called = false;
  auto handler = std::make_unique<StreamHandlerFunctions<>>(
      [&on_listen_called](const EncodableValue* arguments,
                          std::unique_ptr<EventSink<>>&& events)
          -> std::unique_ptr<StreamHandlerError<>> {
        on_listen_called = true;
        return nullptr;
      },
      [&on_cancel_called](const EncodableValue* arguments)
          -> std::unique_ptr<StreamHandlerError<>> {
        on_cancel_called = true;
        return nullptr;
      });
  channel.SetStreamHandler(std::move(handler));
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);

  // Send test listen message.
  MethodCall<> call_listen("listen", nullptr);
  auto message = codec.EncodeMethodCall(call_listen);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});

  // Check results.
  EXPECT_EQ(on_listen_called, true);
  EXPECT_EQ(on_cancel_called, false);
}

// Pseudo test when user re-registers or call OnListen to the same channel.
// Confirm that OnCancel is called and OnListen is called again
// when user re-registers the same channel that has already started
// communication.
TEST(EventChannelTest, ReRegistration) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  const StandardMethodCodec& codec = StandardMethodCodec::GetInstance();
  EventChannel channel(&messenger, channel_name, &codec);

  bool on_listen_called = false;
  bool on_cancel_called = false;
  auto handler = std::make_unique<StreamHandlerFunctions<>>(
      [&on_listen_called](const EncodableValue* arguments,
                          std::unique_ptr<EventSink<>>&& events)
          -> std::unique_ptr<StreamHandlerError<>> {
        on_listen_called = true;
        return nullptr;
      },
      [&on_cancel_called](const EncodableValue* arguments)
          -> std::unique_ptr<StreamHandlerError<>> {
        on_cancel_called = true;
        return nullptr;
      });
  channel.SetStreamHandler(std::move(handler));
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);

  // Send test listen message.
  MethodCall<> call("listen", nullptr);
  auto message = codec.EncodeMethodCall(call);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});
  EXPECT_EQ(on_listen_called, true);

  // Send second test message to test StreamHandler's OnCancel
  // method is called before OnListen method is called.
  on_listen_called = false;
  message = codec.EncodeMethodCall(call);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});

  // Check results.
  EXPECT_EQ(on_cancel_called, true);
  EXPECT_EQ(on_listen_called, true);
}

// Test that the handler is called even if the event channel is destroyed.
TEST(EventChannelTest, HandlerOutlivesEventChannel) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  const StandardMethodCodec& codec = StandardMethodCodec::GetInstance();

  bool on_listen_called = false;
  bool on_cancel_called = false;
  {
    EventChannel channel(&messenger, channel_name, &codec);
    auto handler = std::make_unique<StreamHandlerFunctions<>>(
        [&on_listen_called](const EncodableValue* arguments,
                            std::unique_ptr<EventSink<>>&& events)
            -> std::unique_ptr<StreamHandlerError<>> {
          on_listen_called = true;
          return nullptr;
        },
        [&on_cancel_called](const EncodableValue* arguments)
            -> std::unique_ptr<StreamHandlerError<>> {
          on_cancel_called = true;
          return nullptr;
        });
    channel.SetStreamHandler(std::move(handler));
  }

  // The event channel was destroyed but the handler should still be alive.
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);

  // Send test listen message.
  MethodCall<> call_listen("listen", nullptr);
  auto message = codec.EncodeMethodCall(call_listen);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});
  EXPECT_EQ(on_listen_called, true);

  // Send test cancel message.
  MethodCall<> call_cancel("cancel", nullptr);
  message = codec.EncodeMethodCall(call_cancel);
  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, const size_t reply_size) {});
  EXPECT_EQ(on_cancel_called, true);
}

TEST(EventChannelTest, StreamHandlerErrorPassByValue) {
  std::unique_ptr<StreamHandlerError<>> error = nullptr;

  {
    std::string code = "Code";
    std::string msg = "Message";
    std::unique_ptr<EncodableValue> details =
        std::make_unique<EncodableValue>("Details");
    error =
        std::make_unique<StreamHandlerError<>>(code, msg, std::move(details));
  }

  ASSERT_NE(error.get(), nullptr);
  EXPECT_EQ(error->error_code, "Code");
  EXPECT_EQ(error->error_message, "Message");
  EXPECT_EQ(std::get<std::string>(*error->error_details), "Details");
}

TEST(EventChannelTest, StreamHandlerErrorNullptr) {
  std::unique_ptr<StreamHandlerError<>> error =
      std::make_unique<StreamHandlerError<>>("Code", "Message", nullptr);

  ASSERT_NE(error.get(), nullptr);
  EXPECT_FALSE(error->error_details);
}

}  // namespace flutter
