// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "flutter/shell/platform/windows/keyboard_key_channel_handler.h"

#include <memory>

#include "flutter/shell/platform/common/json_message_codec.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/shell/platform/windows/testing/test_binary_messenger.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {
static constexpr char kScanCodeKey[] = "scanCode";
static constexpr char kCharacterCodePointKey[] = "characterCodePoint";
static constexpr int kHandledScanCode = 0x14;
static constexpr int kUnhandledScanCode = 0x15;
static constexpr int kUnhandledScanCodeExtended = 0xe015;

static std::unique_ptr<std::vector<uint8_t>> CreateResponse(bool handled) {
  auto response_doc =
      std::make_unique<rapidjson::Document>(rapidjson::kObjectType);
  auto& allocator = response_doc->GetAllocator();
  response_doc->AddMember("handled", handled, allocator);
  return JsonMessageCodec::GetInstance().EncodeMessage(*response_doc);
}
}  // namespace

TEST(KeyboardKeyChannelHandlerTest, KeyboardHookHandling) {
  auto handled_message = CreateResponse(true);
  auto unhandled_message = CreateResponse(false);
  int received_scancode = 0;

  TestBinaryMessenger messenger(
      [&received_scancode, &handled_message, &unhandled_message](
          const std::string& channel, const uint8_t* message,
          size_t message_size, BinaryReply reply) {
        if (channel == "flutter/keyevent") {
          auto message_doc = JsonMessageCodec::GetInstance().DecodeMessage(
              message, message_size);
          received_scancode = (*message_doc)[kScanCodeKey].GetInt();
          if (received_scancode == kHandledScanCode) {
            reply(handled_message->data(), handled_message->size());
          } else {
            reply(unhandled_message->data(), unhandled_message->size());
          }
        }
      });

  KeyboardKeyChannelHandler handler(&messenger);
  bool last_handled = false;

  handler.KeyboardHook(
      64, kHandledScanCode, WM_KEYDOWN, L'a', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(received_scancode, kHandledScanCode);
  EXPECT_EQ(last_handled, true);

  received_scancode = 0;

  handler.KeyboardHook(
      64, kUnhandledScanCode, WM_KEYDOWN, L'b', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(received_scancode, kUnhandledScanCode);
  EXPECT_EQ(last_handled, false);

  received_scancode = 0;

  handler.KeyboardHook(
      64, kHandledScanCode, WM_SYSKEYDOWN, L'a', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(received_scancode, kHandledScanCode);
  EXPECT_EQ(last_handled, true);

  received_scancode = 0;

  handler.KeyboardHook(
      64, kUnhandledScanCode, WM_SYSKEYDOWN, L'c', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(received_scancode, kUnhandledScanCode);
  EXPECT_EQ(last_handled, false);
}

TEST(KeyboardKeyChannelHandlerTest, ExtendedKeysAreSentToRedispatch) {
  auto handled_message = CreateResponse(true);
  auto unhandled_message = CreateResponse(false);
  int received_scancode = 0;

  TestBinaryMessenger messenger(
      [&received_scancode, &handled_message, &unhandled_message](
          const std::string& channel, const uint8_t* message,
          size_t message_size, BinaryReply reply) {
        if (channel == "flutter/keyevent") {
          auto message_doc = JsonMessageCodec::GetInstance().DecodeMessage(
              message, message_size);
          received_scancode = (*message_doc)[kScanCodeKey].GetInt();
          if (received_scancode == kHandledScanCode) {
            reply(handled_message->data(), handled_message->size());
          } else {
            reply(unhandled_message->data(), unhandled_message->size());
          }
        }
      });

  KeyboardKeyChannelHandler handler(&messenger);
  bool last_handled = true;

  // Extended key flag is passed to redispatched events if set.
  handler.KeyboardHook(
      64, kUnhandledScanCode, WM_KEYDOWN, L'b', true, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(received_scancode, kUnhandledScanCodeExtended);

  last_handled = true;
  // Extended key flag is not passed to redispatched events if not set.
  handler.KeyboardHook(
      64, kUnhandledScanCode, WM_KEYDOWN, L'b', false, false,
      [&last_handled](bool handled) { last_handled = handled; });
  EXPECT_EQ(last_handled, false);
  EXPECT_EQ(received_scancode, kUnhandledScanCode);
}

TEST(KeyboardKeyChannelHandlerTest, DeadKeysDoNotCrash) {
  bool received = false;
  TestBinaryMessenger messenger(
      [&received](const std::string& channel, const uint8_t* message,
                  size_t message_size, BinaryReply reply) {
        if (channel == "flutter/keyevent") {
          auto message_doc = JsonMessageCodec::GetInstance().DecodeMessage(
              message, message_size);
          uint32_t character = (*message_doc)[kCharacterCodePointKey].GetUint();
          EXPECT_EQ(character, (uint32_t)'^');
          received = true;
        }
        return true;
      });

  KeyboardKeyChannelHandler handler(&messenger);
  // Extended key flag is passed to redispatched events if set.
  handler.KeyboardHook(0xDD, 0x1a, WM_KEYDOWN, 0x8000005E, false, false,
                       [](bool handled) {});

  // EXPECT is done during the callback above.
  EXPECT_TRUE(received);
}

TEST(KeyboardKeyChannelHandlerTest, EmptyResponsesDoNotCrash) {
  bool received = false;
  TestBinaryMessenger messenger(
      [&received](const std::string& channel, const uint8_t* message,
                  size_t message_size, BinaryReply reply) {
        if (channel == "flutter/keyevent") {
          std::string empty_message = "";
          std::vector<uint8_t> empty_response(empty_message.begin(),
                                              empty_message.end());
          reply(empty_response.data(), empty_response.size());
          received = true;
        }
        return true;
      });

  KeyboardKeyChannelHandler handler(&messenger);
  handler.KeyboardHook(64, kUnhandledScanCode, WM_KEYDOWN, L'b', false, false,
                       [](bool handled) {});

  // Passes if it does not crash.
  EXPECT_TRUE(received);
}

}  // namespace testing
}  // namespace flutter
