// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/platform_handler.h"

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/json_method_codec.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/flutter_windows_engine_builder.h"
#include "flutter/shell/platform/windows/testing/mock_window_binding_handler.h"
#include "flutter/shell/platform/windows/testing/test_binary_messenger.h"
#include "flutter/shell/platform/windows/testing/windows_test.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rapidjson/document.h"

namespace flutter {
namespace testing {

namespace {
using ::testing::_;
using ::testing::NiceMock;
using ::testing::Return;

static constexpr char kChannelName[] = "flutter/platform";

static constexpr char kClipboardGetDataMessage[] =
    "{\"method\":\"Clipboard.getData\",\"args\":\"text/plain\"}";
static constexpr char kClipboardGetDataFakeContentTypeMessage[] =
    "{\"method\":\"Clipboard.getData\",\"args\":\"text/madeupcontenttype\"}";
static constexpr char kClipboardHasStringsMessage[] =
    "{\"method\":\"Clipboard.hasStrings\",\"args\":\"text/plain\"}";
static constexpr char kClipboardHasStringsFakeContentTypeMessage[] =
    "{\"method\":\"Clipboard.hasStrings\",\"args\":\"text/madeupcontenttype\"}";
static constexpr char kClipboardSetDataMessage[] =
    "{\"method\":\"Clipboard.setData\",\"args\":{\"text\":\"hello\"}}";
static constexpr char kClipboardSetDataNullTextMessage[] =
    "{\"method\":\"Clipboard.setData\",\"args\":{\"text\":null}}";
static constexpr char kClipboardSetDataUnknownTypeMessage[] =
    "{\"method\":\"Clipboard.setData\",\"args\":{\"madeuptype\":\"hello\"}}";
static constexpr char kSystemSoundTypeAlertMessage[] =
    "{\"method\":\"SystemSound.play\",\"args\":\"SystemSoundType.alert\"}";
static constexpr char kSystemExitApplicationRequiredMessage[] =
    "{\"method\":\"System.exitApplication\",\"args\":{\"type\":\"required\","
    "\"exitCode\":1}}";
static constexpr char kSystemExitApplicationCancelableMessage[] =
    "{\"method\":\"System.exitApplication\",\"args\":{\"type\":\"cancelable\","
    "\"exitCode\":2}}";
static constexpr char kExitResponseCancelMessage[] =
    "[{\"response\":\"cancel\"}]";
static constexpr char kExitResponseExitMessage[] = "[{\"response\":\"exit\"}]";

static constexpr int kAccessDeniedErrorCode = 5;
static constexpr int kErrorSuccess = 0;
static constexpr int kArbitraryErrorCode = 1;

// Test implementation of PlatformHandler to allow testing the PlatformHandler
// logic.
class MockPlatformHandler : public PlatformHandler {
 public:
  explicit MockPlatformHandler(
      BinaryMessenger* messenger,
      FlutterWindowsEngine* engine,
      std::optional<std::function<std::unique_ptr<ScopedClipboardInterface>()>>
          scoped_clipboard_provider = std::nullopt)
      : PlatformHandler(messenger, engine, scoped_clipboard_provider) {}

  virtual ~MockPlatformHandler() = default;

  MOCK_METHOD(void,
              GetPlainText,
              (std::unique_ptr<MethodResult<rapidjson::Document>>,
               std::string_view key),
              (override));
  MOCK_METHOD(void,
              GetHasStrings,
              (std::unique_ptr<MethodResult<rapidjson::Document>>),
              (override));
  MOCK_METHOD(void,
              SetPlainText,
              (const std::string&,
               std::unique_ptr<MethodResult<rapidjson::Document>>),
              (override));
  MOCK_METHOD(void,
              SystemSoundPlay,
              (const std::string&,
               std::unique_ptr<MethodResult<rapidjson::Document>>),
              (override));

  MOCK_METHOD(void,
              QuitApplication,
              (std::optional<HWND> hwnd,
               std::optional<WPARAM> wparam,
               std::optional<LPARAM> lparam,
               UINT exit_code),
              (override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockPlatformHandler);
};

// A test version of the private ScopedClipboard.
class MockScopedClipboard : public ScopedClipboardInterface {
 public:
  MockScopedClipboard() = default;
  virtual ~MockScopedClipboard() = default;

  MOCK_METHOD(int, Open, (HWND window), (override));
  MOCK_METHOD(bool, HasString, (), (override));
  MOCK_METHOD((std::variant<std::wstring, int>), GetString, (), (override));
  MOCK_METHOD(int, SetString, (const std::wstring string), (override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockScopedClipboard);
};

std::string SimulatePlatformMessage(TestBinaryMessenger* messenger,
                                    std::string message) {
  std::string result;
  EXPECT_TRUE(messenger->SimulateEngineMessage(
      kChannelName, reinterpret_cast<const uint8_t*>(message.c_str()),
      message.size(),
      [result = &result](const uint8_t* reply, size_t reply_size) {
        std::string response(reinterpret_cast<const char*>(reply), reply_size);

        *result = response;
      }));

  return result;
}

}  // namespace

class PlatformHandlerTest : public WindowsTest {
 public:
  PlatformHandlerTest() = default;
  virtual ~PlatformHandlerTest() = default;

 protected:
  FlutterWindowsEngine* engine() { return engine_.get(); }

  void UseHeadlessEngine() {
    FlutterWindowsEngineBuilder builder{GetContext()};

    engine_ = builder.Build();
  }

  void UseEngineWithView() {
    FlutterWindowsEngineBuilder builder{GetContext()};

    auto window = std::make_unique<NiceMock<MockWindowBindingHandler>>();

    engine_ = builder.Build();
    view_ = std::make_unique<FlutterWindowsView>(kImplicitViewId, engine_.get(),
                                                 std::move(window));

    EngineModifier modifier{engine_.get()};
    modifier.SetImplicitView(view_.get());
  }

 private:
  std::unique_ptr<FlutterWindowsEngine> engine_;
  std::unique_ptr<FlutterWindowsView> view_;

  FML_DISALLOW_COPY_AND_ASSIGN(PlatformHandlerTest);
};

TEST_F(PlatformHandlerTest, GetClipboardData) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kErrorSuccess));
    EXPECT_CALL(*clipboard.get(), HasString).Times(1).WillOnce(Return(true));
    EXPECT_CALL(*clipboard.get(), GetString)
        .Times(1)
        .WillOnce(Return(std::wstring(L"Hello world")));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardGetDataMessage);

  EXPECT_EQ(result, "[{\"text\":\"Hello world\"}]");
}

TEST_F(PlatformHandlerTest, GetClipboardDataRejectsUnknownContentType) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine());

  // Requesting an unknown content type is an error.
  std::string result = SimulatePlatformMessage(
      &messenger, kClipboardGetDataFakeContentTypeMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unknown clipboard format\",null]");
}

TEST_F(PlatformHandlerTest, GetClipboardDataRequiresView) {
  UseHeadlessEngine();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine());

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardGetDataMessage);

  EXPECT_EQ(result,
            "[\"Clipboard error\",\"Clipboard is not available in "
            "Windows headless mode\",null]");
}

TEST_F(PlatformHandlerTest, GetClipboardDataReportsOpenFailure) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kArbitraryErrorCode));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardGetDataMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unable to open clipboard\",1]");
}

TEST_F(PlatformHandlerTest, GetClipboardDataReportsGetDataFailure) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kErrorSuccess));
    EXPECT_CALL(*clipboard.get(), HasString).Times(1).WillOnce(Return(true));
    EXPECT_CALL(*clipboard.get(), GetString)
        .Times(1)
        .WillOnce(Return(kArbitraryErrorCode));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardGetDataMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unable to get clipboard data\",1]");
}

TEST_F(PlatformHandlerTest, ClipboardHasStrings) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kErrorSuccess));
    EXPECT_CALL(*clipboard.get(), HasString).Times(1).WillOnce(Return(true));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardHasStringsMessage);

  EXPECT_EQ(result, "[{\"value\":true}]");
}

TEST_F(PlatformHandlerTest, ClipboardHasStringsReturnsFalse) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kErrorSuccess));
    EXPECT_CALL(*clipboard.get(), HasString).Times(1).WillOnce(Return(false));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardHasStringsMessage);

  EXPECT_EQ(result, "[{\"value\":false}]");
}

TEST_F(PlatformHandlerTest, ClipboardHasStringsRejectsUnknownContentType) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine());

  std::string result = SimulatePlatformMessage(
      &messenger, kClipboardHasStringsFakeContentTypeMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unknown clipboard format\",null]");
}

TEST_F(PlatformHandlerTest, ClipboardHasStringsRequiresView) {
  UseHeadlessEngine();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine());

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardHasStringsMessage);

  EXPECT_EQ(result,
            "[\"Clipboard error\",\"Clipboard is not available in Windows "
            "headless mode\",null]");
}

// Regression test for https://github.com/flutter/flutter/issues/95817.
TEST_F(PlatformHandlerTest, ClipboardHasStringsIgnoresPermissionErrors) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kAccessDeniedErrorCode));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardHasStringsMessage);

  EXPECT_EQ(result, "[{\"value\":false}]");
}

TEST_F(PlatformHandlerTest, ClipboardHasStringsReportsErrors) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kArbitraryErrorCode));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardHasStringsMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unable to open clipboard\",1]");
}

TEST_F(PlatformHandlerTest, ClipboardSetData) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kErrorSuccess));
    EXPECT_CALL(*clipboard.get(), SetString)
        .Times(1)
        .WillOnce([](std::wstring string) {
          EXPECT_EQ(string, L"hello");
          return kErrorSuccess;
        });

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardSetDataMessage);

  EXPECT_EQ(result, "[null]");
}

// Regression test for: https://github.com/flutter/flutter/issues/121976
TEST_F(PlatformHandlerTest, ClipboardSetDataTextMustBeString) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine());

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardSetDataNullTextMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unknown clipboard format\",null]");
}

TEST_F(PlatformHandlerTest, ClipboardSetDataUnknownType) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine());

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardSetDataUnknownTypeMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unknown clipboard format\",null]");
}

TEST_F(PlatformHandlerTest, ClipboardSetDataRequiresView) {
  UseHeadlessEngine();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine());

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardSetDataMessage);

  EXPECT_EQ(result,
            "[\"Clipboard error\",\"Clipboard is not available in Windows "
            "headless mode\",null]");
}

TEST_F(PlatformHandlerTest, ClipboardSetDataReportsOpenFailure) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kArbitraryErrorCode));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardSetDataMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unable to open clipboard\",1]");
}

TEST_F(PlatformHandlerTest, ClipboardSetDataReportsSetDataFailure) {
  UseEngineWithView();

  TestBinaryMessenger messenger;
  PlatformHandler platform_handler(&messenger, engine(), []() {
    auto clipboard = std::make_unique<MockScopedClipboard>();

    EXPECT_CALL(*clipboard.get(), Open)
        .Times(1)
        .WillOnce(Return(kErrorSuccess));
    EXPECT_CALL(*clipboard.get(), SetString)
        .Times(1)
        .WillOnce(Return(kArbitraryErrorCode));

    return clipboard;
  });

  std::string result =
      SimulatePlatformMessage(&messenger, kClipboardSetDataMessage);

  EXPECT_EQ(result, "[\"Clipboard error\",\"Unable to set clipboard data\",1]");
}

TEST_F(PlatformHandlerTest, PlaySystemSound) {
  UseHeadlessEngine();

  TestBinaryMessenger messenger;
  MockPlatformHandler platform_handler(&messenger, engine());

  EXPECT_CALL(platform_handler, SystemSoundPlay("SystemSoundType.alert", _))
      .WillOnce([](const std::string& sound,
                   std::unique_ptr<MethodResult<rapidjson::Document>> result) {
        result->Success();
      });

  std::string result =
      SimulatePlatformMessage(&messenger, kSystemSoundTypeAlertMessage);

  EXPECT_EQ(result, "[null]");
}

TEST_F(PlatformHandlerTest, SystemExitApplicationRequired) {
  UseHeadlessEngine();
  UINT exit_code = 0;

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t size,
                                   BinaryReply reply) {});
  MockPlatformHandler platform_handler(&messenger, engine());

  ON_CALL(platform_handler, QuitApplication)
      .WillByDefault([&exit_code](std::optional<HWND> hwnd,
                                  std::optional<WPARAM> wparam,
                                  std::optional<LPARAM> lparam,
                                  UINT ec) { exit_code = ec; });
  EXPECT_CALL(platform_handler, QuitApplication).Times(1);

  std::string result = SimulatePlatformMessage(
      &messenger, kSystemExitApplicationRequiredMessage);
  EXPECT_EQ(result, "[{\"response\":\"exit\"}]");
  EXPECT_EQ(exit_code, 1);
}

TEST_F(PlatformHandlerTest, SystemExitApplicationCancelableCancel) {
  UseHeadlessEngine();
  bool called_cancel = false;

  TestBinaryMessenger messenger(
      [&called_cancel](const std::string& channel, const uint8_t* message,
                       size_t size, BinaryReply reply) {
        reply(reinterpret_cast<const uint8_t*>(kExitResponseCancelMessage),
              sizeof(kExitResponseCancelMessage));
        called_cancel = true;
      });
  MockPlatformHandler platform_handler(&messenger, engine());

  EXPECT_CALL(platform_handler, QuitApplication).Times(0);

  std::string result = SimulatePlatformMessage(
      &messenger, kSystemExitApplicationCancelableMessage);
  EXPECT_EQ(result, "[{\"response\":\"cancel\"}]");
  EXPECT_TRUE(called_cancel);
}

TEST_F(PlatformHandlerTest, SystemExitApplicationCancelableExit) {
  UseHeadlessEngine();
  bool called_cancel = false;
  UINT exit_code = 0;

  TestBinaryMessenger messenger(
      [&called_cancel](const std::string& channel, const uint8_t* message,
                       size_t size, BinaryReply reply) {
        reply(reinterpret_cast<const uint8_t*>(kExitResponseExitMessage),
              sizeof(kExitResponseExitMessage));
        called_cancel = true;
      });
  MockPlatformHandler platform_handler(&messenger, engine());

  ON_CALL(platform_handler, QuitApplication)
      .WillByDefault([&exit_code](std::optional<HWND> hwnd,
                                  std::optional<WPARAM> wparam,
                                  std::optional<LPARAM> lparam,
                                  UINT ec) { exit_code = ec; });
  EXPECT_CALL(platform_handler, QuitApplication).Times(1);

  std::string result = SimulatePlatformMessage(
      &messenger, kSystemExitApplicationCancelableMessage);
  EXPECT_EQ(result, "[{\"response\":\"cancel\"}]");
  EXPECT_TRUE(called_cancel);
  EXPECT_EQ(exit_code, 2);
}

}  // namespace testing
}  // namespace flutter
