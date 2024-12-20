// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_channel.h"

#include <memory>
#include <string>

#include "flutter/shell/platform/common/client_wrapper/include/flutter/binary_messenger.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_result_functions.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

class TestBinaryMessenger : public BinaryMessenger {
 public:
  void Send(const std::string& channel,
            const uint8_t* message,
            size_t message_size,
            BinaryReply reply) const override {
    send_called_ = true;
    last_reply_handler_ = reply;
    int length = static_cast<int>(message_size);
    last_message_ =
        std::vector<uint8_t>(message, message + length * sizeof(uint8_t));
  }

  void SetMessageHandler(const std::string& channel,
                         BinaryMessageHandler handler) override {
    last_message_handler_channel_ = channel;
    last_message_handler_ = handler;
  }

  bool send_called() { return send_called_; }

  BinaryReply last_reply_handler() { return last_reply_handler_; }

  std::string last_message_handler_channel() {
    return last_message_handler_channel_;
  }

  BinaryMessageHandler last_message_handler() { return last_message_handler_; }

  std::vector<uint8_t> last_message() { return last_message_; }

 private:
  mutable bool send_called_ = false;
  mutable BinaryReply last_reply_handler_;
  std::string last_message_handler_channel_;
  BinaryMessageHandler last_message_handler_;
  mutable std::vector<uint8_t> last_message_;
};

}  // namespace

// Tests that SetMethodCallHandler sets a handler that correctly interacts with
// the binary messenger.
TEST(MethodChannelTest, Registration) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  const StandardMethodCodec& codec = StandardMethodCodec::GetInstance();
  MethodChannel channel(&messenger, channel_name, &codec);

  bool callback_called = false;
  const std::string method_name("hello");
  channel.SetMethodCallHandler(
      [&callback_called, method_name](const auto& call, auto result) {
        callback_called = true;
        // Ensure that the wrapper received a correctly decoded call and a
        // result.
        EXPECT_EQ(call.method_name(), method_name);
        EXPECT_NE(result, nullptr);
        result->Success();
      });
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);
  // Send a test message to trigger the handler test assertions.
  MethodCall<> call(method_name, nullptr);
  auto message = codec.EncodeMethodCall(call);

  messenger.last_message_handler()(
      message->data(), message->size(),
      [](const uint8_t* reply, size_t reply_size) {});
  EXPECT_TRUE(callback_called);
}

// Tests that SetMethodCallHandler with a null handler unregisters the handler.
TEST(MethodChannelTest, Unregistration) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  MethodChannel channel(&messenger, channel_name,
                        &StandardMethodCodec::GetInstance());

  channel.SetMethodCallHandler([](const auto& call, auto result) {});
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_NE(messenger.last_message_handler(), nullptr);

  channel.SetMethodCallHandler(nullptr);
  EXPECT_EQ(messenger.last_message_handler_channel(), channel_name);
  EXPECT_EQ(messenger.last_message_handler(), nullptr);
}

TEST(MethodChannelTest, InvokeWithoutResponse) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  MethodChannel channel(&messenger, channel_name,
                        &StandardMethodCodec::GetInstance());

  channel.InvokeMethod("foo", nullptr);
  EXPECT_TRUE(messenger.send_called());
  EXPECT_EQ(messenger.last_reply_handler(), nullptr);
}

TEST(MethodChannelTest, InvokeWithResponse) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  MethodChannel channel(&messenger, channel_name,
                        &StandardMethodCodec::GetInstance());

  bool received_reply = false;
  const std::string reply = "bar";
  auto result_handler = std::make_unique<MethodResultFunctions<>>(
      [&received_reply, reply](const EncodableValue* success_value) {
        received_reply = true;
        EXPECT_EQ(std::get<std::string>(*success_value), reply);
      },
      nullptr, nullptr);

  channel.InvokeMethod("foo", nullptr, std::move(result_handler));
  EXPECT_TRUE(messenger.send_called());
  ASSERT_NE(messenger.last_reply_handler(), nullptr);

  // Call the underlying reply handler to ensure it's processed correctly.
  EncodableValue reply_value(reply);
  std::unique_ptr<std::vector<uint8_t>> encoded_reply =
      StandardMethodCodec::GetInstance().EncodeSuccessEnvelope(&reply_value);
  messenger.last_reply_handler()(encoded_reply->data(), encoded_reply->size());
  EXPECT_TRUE(received_reply);
}

TEST(MethodChannelTest, InvokeNotImplemented) {
  TestBinaryMessenger messenger;
  const std::string channel_name("some_channel");
  MethodChannel channel(&messenger, channel_name,
                        &StandardMethodCodec::GetInstance());

  bool received_not_implemented = false;
  auto result_handler = std::make_unique<MethodResultFunctions<>>(
      nullptr, nullptr,
      [&received_not_implemented]() { received_not_implemented = true; });

  channel.InvokeMethod("foo", nullptr, std::move(result_handler));
  EXPECT_EQ(messenger.send_called(), true);
  ASSERT_NE(messenger.last_reply_handler(), nullptr);

  // Call the underlying reply handler to ensure it's reported as unimplemented.
  messenger.last_reply_handler()(nullptr, 0);
  EXPECT_TRUE(received_not_implemented);
}

// Tests that calling Resize generates the binary message expected by the Dart
// implementation.
TEST(MethodChannelTest, Resize) {
  TestBinaryMessenger messenger;
  const std::string channel_name("flutter/test");
  MethodChannel channel(&messenger, channel_name,
                        &StandardMethodCodec::GetInstance());

  channel.Resize(3);

  // Because the Dart implementation for the control channel implements its own
  // custom deserialization logic, this test compares the generated bytes array
  // to the expected one (for instance, the deserialization logic expects the
  // size parameter of the resize method call to be an uint32).
  //
  // The expected content was created from the following Dart code:
  //   MethodCall call = MethodCall('resize', ['flutter/test',3]);
  //   StandardMethodCodec().encodeMethodCall(call).buffer.asUint8List();
  const int expected_message_size = 29;

  EXPECT_EQ(messenger.send_called(), true);
  EXPECT_EQ(static_cast<int>(messenger.last_message().size()),
            expected_message_size);

  int expected[expected_message_size] = {
      7,   6,   114, 101, 115, 105, 122, 101, 12,  2, 7, 12, 102, 108, 117,
      116, 116, 101, 114, 47,  116, 101, 115, 116, 3, 3, 0,  0,   0};
  for (int i = 0; i < expected_message_size; i++) {
    EXPECT_EQ(messenger.last_message()[i], expected[i]);
  }
}

// Tests that calling SetWarnsOnOverflow generates the binary message expected
// by the Dart implementation.
TEST(MethodChannelTest, SetWarnsOnOverflow) {
  TestBinaryMessenger messenger;

  const std::string channel_name("flutter/test");
  MethodChannel channel(&messenger, channel_name,
                        &StandardMethodCodec::GetInstance());

  channel.SetWarnsOnOverflow(false);

  // The expected content was created from the following Dart code:
  //   MethodCall call = MethodCall('overflow',['flutter/test', true]);
  //   StandardMethodCodec().encodeMethodCall(call).buffer.asUint8List();
  const int expected_message_size = 27;

  EXPECT_EQ(messenger.send_called(), true);
  EXPECT_EQ(static_cast<int>(messenger.last_message().size()),
            expected_message_size);

  int expected[expected_message_size] = {
      7,   8,   111, 118, 101, 114, 102, 108, 111, 119, 12,  2,   7, 12,
      102, 108, 117, 116, 116, 101, 114, 47,  116, 101, 115, 116, 1};
  for (int i = 0; i < expected_message_size; i++) {
    EXPECT_EQ(messenger.last_message()[i], expected[i]);
  }
}

}  // namespace flutter
