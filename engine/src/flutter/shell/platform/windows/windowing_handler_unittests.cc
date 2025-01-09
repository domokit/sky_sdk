// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "flutter/shell/platform/windows/windowing_handler.h"

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_result_functions.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "flutter/shell/platform/windows/flutter_host_window_controller.h"
#include "flutter/shell/platform/windows/testing/flutter_windows_engine_builder.h"
#include "flutter/shell/platform/windows/testing/test_binary_messenger.h"
#include "flutter/shell/platform/windows/testing/windows_test.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {
using ::testing::_;
using ::testing::Eq;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::StrEq;

static constexpr char kChannelName[] = "flutter/windowing";

static constexpr char kCreateWindowMethod[] = "createWindow";
static constexpr char kDestroyWindowMethod[] = "destroyWindow";

void SimulateWindowingMessage(TestBinaryMessenger* messenger,
                              const std::string& method_name,
                              std::unique_ptr<EncodableValue> arguments,
                              MethodResult<EncodableValue>* result_handler) {
  MethodCall<> call(method_name, std::move(arguments));

  auto message = StandardMethodCodec::GetInstance().EncodeMethodCall(call);

  EXPECT_TRUE(messenger->SimulateEngineMessage(
      kChannelName, message->data(), message->size(),
      [&result_handler](const uint8_t* reply, size_t reply_size) {
        StandardMethodCodec::GetInstance().DecodeAndProcessResponseEnvelope(
            reply, reply_size, result_handler);
      }));
}

class MockFlutterHostWindowController : public FlutterHostWindowController {
 public:
  MockFlutterHostWindowController(FlutterWindowsEngine* engine)
      : FlutterHostWindowController(engine) {}
  ~MockFlutterHostWindowController() = default;

  MOCK_METHOD(std::optional<WindowMetadata>,
              CreateHostWindow,
              (std::wstring const& title,
               WindowSize const& size,
               WindowArchetype archetype,
               std::optional<WindowPositioner> positioner,
               std::optional<FlutterViewId> parent_view_id),
              (override));
  MOCK_METHOD(bool, DestroyHostWindow, (FlutterViewId view_id), (override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockFlutterHostWindowController);
};

}  // namespace

class WindowingHandlerTest : public WindowsTest {
 public:
  WindowingHandlerTest() = default;
  virtual ~WindowingHandlerTest() = default;

 protected:
  void SetUp() override {
    FlutterWindowsEngineBuilder builder(GetContext());
    engine_ = builder.Build();

    mock_controller_ =
        std::make_unique<NiceMock<MockFlutterHostWindowController>>(
            engine_.get());

    ON_CALL(*mock_controller_, CreateHostWindow)
        .WillByDefault(Return(WindowMetadata{}));
    ON_CALL(*mock_controller_, DestroyHostWindow).WillByDefault(Return(true));
  }

  MockFlutterHostWindowController* controller() {
    return mock_controller_.get();
  }

 private:
  std::unique_ptr<FlutterWindowsEngine> engine_;
  std::unique_ptr<NiceMock<MockFlutterHostWindowController>> mock_controller_;

  FML_DISALLOW_COPY_AND_ASSIGN(WindowingHandlerTest);
};

TEST_F(WindowingHandlerTest, HandleCreateRegularWindow) {
  TestBinaryMessenger messenger;
  WindowingHandler windowing_handler(&messenger, controller());

  WindowSize const size = {800, 600};
  EncodableMap const arguments = {
      {EncodableValue("size"),
       EncodableValue(EncodableList{EncodableValue(size.width),
                                    EncodableValue(size.height)})},
  };

  bool success = false;
  MethodResultFunctions<> result_handler(
      [&success](const EncodableValue* result) { success = true; }, nullptr,
      nullptr);

  EXPECT_CALL(
      *controller(),
      CreateHostWindow(StrEq(L"regular"), size, WindowArchetype::regular,
                       Eq(std::nullopt), Eq(std::nullopt)))
      .Times(1);

  SimulateWindowingMessage(&messenger, kCreateWindowMethod,
                           std::make_unique<EncodableValue>(arguments),
                           &result_handler);

  EXPECT_TRUE(success);
}

TEST_F(WindowingHandlerTest, HandleDestroyWindow) {
  TestBinaryMessenger messenger;
  WindowingHandler windowing_handler(&messenger, controller());

  EncodableMap const arguments = {
      {EncodableValue("viewId"), EncodableValue(1)},
  };

  bool success = false;
  MethodResultFunctions<> result_handler(
      [&success](const EncodableValue* result) { success = true; }, nullptr,
      nullptr);

  EXPECT_CALL(*controller(), DestroyHostWindow(1)).Times(1);

  SimulateWindowingMessage(&messenger, kDestroyWindowMethod,
                           std::make_unique<EncodableValue>(arguments),
                           &result_handler);

  EXPECT_TRUE(success);
}

}  // namespace testing
}  // namespace flutter
