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

constexpr char kChannelName[] = "flutter/windowing";
constexpr char kCreateRegularMethod[] = "createRegular";
constexpr char kDestroyWindowMethod[] = "destroyWindow";
constexpr char kModifyRegularMethod[] = "modifyRegular";

constexpr char kMaxSizeKey[] = "maxSize";
constexpr char kMinSizeKey[] = "minSize";
constexpr char kSizeKey[] = "size";
constexpr char kStateKey[] = "state";
constexpr char kTitleKey[] = "title";
constexpr char kViewIdKey[] = "viewId";

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
              (WindowCreationSettings const& settings),
              (override));
  MOCK_METHOD(bool,
              ModifyHostWindow,
              (FlutterViewId view_id,
               WindowModificationSettings const& settings),
              (override, const));
  MOCK_METHOD(bool,
              DestroyHostWindow,
              (FlutterViewId view_id),
              (override, const));

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
        .WillByDefault([](WindowCreationSettings const& settings) {
          return WindowMetadata{
              .size = settings.size,
              .state = WindowState::kRestored,
          };
        });
    ON_CALL(*mock_controller_, ModifyHostWindow).WillByDefault(Return(true));
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

MATCHER_P(WindowCreationSettingsMatches,
          expected,
          "Matches WindowCreationSettings") {
  return arg.archetype == expected.archetype && arg.size == expected.size &&
         arg.min_size == expected.min_size &&
         arg.max_size == expected.max_size && arg.title == expected.title &&
         arg.state == expected.state;
}

MATCHER_P(WindowModificationSettingsMatches,
          expected,
          "Matches WindowModificationSettings") {
  return arg.size == expected.size && arg.title == expected.title &&
         arg.state == expected.state;
}

TEST_F(WindowingHandlerTest, HandleCreateRegular) {
  TestBinaryMessenger messenger;
  WindowingHandler windowing_handler(&messenger, controller());

  WindowCreationSettings const settings = {
      .archetype = WindowArchetype::kRegular,
      .size = Size{800.0, 600.0},
      .min_size = Size{640.0, 480.0},
      .max_size = Size{1024.0, 768.0},
      .title = "regular",
      .state = WindowState::kRestored,
  };

  EncodableMap const arguments = {
      {EncodableValue(kSizeKey),
       EncodableValue(EncodableList{EncodableValue(settings.size.width()),
                                    EncodableValue(settings.size.height())})},
      {EncodableValue(kMinSizeKey),
       EncodableValue(
           EncodableList{EncodableValue(settings.min_size->width()),
                         EncodableValue(settings.min_size->height())})},
      {EncodableValue(kMaxSizeKey),
       EncodableValue(
           EncodableList{EncodableValue(settings.max_size->width()),
                         EncodableValue(settings.max_size->height())})},
      {EncodableValue(kTitleKey), EncodableValue(*settings.title)},
      {EncodableValue(kStateKey),
       EncodableValue(WindowStateToString(*settings.state))},
  };

  bool success = false;
  MethodResultFunctions<> result_handler(
      [&success](const EncodableValue* result) { success = true; }, nullptr,
      nullptr);

  EXPECT_CALL(*controller(),
              CreateHostWindow(WindowCreationSettingsMatches(settings)))
      .Times(1);

  SimulateWindowingMessage(&messenger, kCreateRegularMethod,
                           std::make_unique<EncodableValue>(arguments),
                           &result_handler);

  EXPECT_TRUE(success);
}

TEST_F(WindowingHandlerTest, HandleModifyRegular) {
  TestBinaryMessenger messenger;
  WindowingHandler windowing_handler(&messenger, controller());

  FlutterViewId const view_id = 1;
  WindowModificationSettings const settings = {
      .size = Size{800.0, 600.0},
      .title = "regular",
      .state = WindowState::kRestored,
  };

  EncodableMap const arguments = {
      {EncodableValue(kViewIdKey), EncodableValue(static_cast<int>(view_id))},
      {EncodableValue(kSizeKey),
       EncodableValue(EncodableList{EncodableValue(settings.size->width()),
                                    EncodableValue(settings.size->height())})},
      {EncodableValue(kTitleKey), EncodableValue(*settings.title)},
      {EncodableValue(kStateKey),
       EncodableValue(WindowStateToString(*settings.state))},
  };

  bool success = false;
  MethodResultFunctions<> result_handler(
      [&success](const EncodableValue* result) { success = true; }, nullptr,
      nullptr);

  EXPECT_CALL(
      *controller(),
      ModifyHostWindow(view_id, WindowModificationSettingsMatches(settings)))
      .Times(1);

  SimulateWindowingMessage(&messenger, kModifyRegularMethod,
                           std::make_unique<EncodableValue>(arguments),
                           &result_handler);

  EXPECT_TRUE(success);
}

TEST_F(WindowingHandlerTest, HandleDestroyWindow) {
  TestBinaryMessenger messenger;
  WindowingHandler windowing_handler(&messenger, controller());

  FlutterViewId const view_id = 1;

  EncodableMap const arguments = {
      {EncodableValue(kViewIdKey), EncodableValue(static_cast<int>(view_id))},
  };

  bool success = false;
  MethodResultFunctions<> result_handler(
      [&success](const EncodableValue* result) { success = true; }, nullptr,
      nullptr);

  EXPECT_CALL(*controller(), DestroyHostWindow(view_id)).Times(1);

  SimulateWindowingMessage(&messenger, kDestroyWindowMethod,
                           std::make_unique<EncodableValue>(arguments),
                           &result_handler);

  EXPECT_TRUE(success);
}

}  // namespace testing
}  // namespace flutter
