// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "flutter/shell/platform/windows/text_input_plugin.h"

#include <rapidjson/document.h>
#include <windows.h>
#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/json_message_codec.h"
#include "flutter/shell/platform/common/json_method_codec.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/flutter_windows_engine_builder.h"
#include "flutter/shell/platform/windows/testing/mock_window_binding_handler.h"
#include "flutter/shell/platform/windows/testing/test_binary_messenger.h"
#include "flutter/shell/platform/windows/testing/windows_test.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {
using ::testing::Return;

static constexpr char kScanCodeKey[] = "scanCode";
static constexpr int kHandledScanCode = 20;
static constexpr int kUnhandledScanCode = 21;
static constexpr char kTextPlainFormat[] = "text/plain";
static constexpr int kDefaultClientId = 42;
// Should be identical to constants in text_input_plugin.cc.
static constexpr char kChannelName[] = "flutter/textinput";
static constexpr char kEnableDeltaModel[] = "enableDeltaModel";
static constexpr char kSetClientMethod[] = "TextInput.setClient";
static constexpr char kAffinityDownstream[] = "TextAffinity.downstream";
static constexpr char kTextKey[] = "text";
static constexpr char kSelectionBaseKey[] = "selectionBase";
static constexpr char kSelectionExtentKey[] = "selectionExtent";
static constexpr char kSelectionAffinityKey[] = "selectionAffinity";
static constexpr char kSelectionIsDirectionalKey[] = "selectionIsDirectional";
static constexpr char kComposingBaseKey[] = "composingBase";
static constexpr char kComposingExtentKey[] = "composingExtent";
static constexpr char kUpdateEditingStateMethod[] =
    "TextInputClient.updateEditingState";

static std::unique_ptr<std::vector<uint8_t>> CreateResponse(bool handled) {
  auto response_doc =
      std::make_unique<rapidjson::Document>(rapidjson::kObjectType);
  auto& allocator = response_doc->GetAllocator();
  response_doc->AddMember("handled", handled, allocator);
  return JsonMessageCodec::GetInstance().EncodeMessage(*response_doc);
}

static std::unique_ptr<rapidjson::Document> EncodedClientConfig(
    std::string type_name,
    std::string input_action) {
  auto arguments = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = arguments->GetAllocator();
  arguments->PushBack(kDefaultClientId, allocator);

  rapidjson::Value config(rapidjson::kObjectType);
  config.AddMember("inputAction", input_action, allocator);
  config.AddMember(kEnableDeltaModel, false, allocator);
  rapidjson::Value type_info(rapidjson::kObjectType);
  type_info.AddMember("name", type_name, allocator);
  config.AddMember("inputType", type_info, allocator);
  arguments->PushBack(config, allocator);

  return arguments;
}

static std::unique_ptr<rapidjson::Document> EncodedEditingState(
    std::string text,
    TextRange selection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText(text);
  model->SetSelection(selection);

  auto arguments = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = arguments->GetAllocator();
  arguments->PushBack(kDefaultClientId, allocator);

  rapidjson::Value editing_state(rapidjson::kObjectType);
  editing_state.AddMember(kSelectionAffinityKey, kAffinityDownstream,
                          allocator);
  editing_state.AddMember(kSelectionBaseKey, selection.base(), allocator);
  editing_state.AddMember(kSelectionExtentKey, selection.extent(), allocator);
  editing_state.AddMember(kSelectionIsDirectionalKey, false, allocator);

  int composing_base =
      model->composing() ? model->composing_range().base() : -1;
  int composing_extent =
      model->composing() ? model->composing_range().extent() : -1;
  editing_state.AddMember(kComposingBaseKey, composing_base, allocator);
  editing_state.AddMember(kComposingExtentKey, composing_extent, allocator);
  editing_state.AddMember(kTextKey,
                          rapidjson::Value(model->GetText(), allocator).Move(),
                          allocator);
  arguments->PushBack(editing_state, allocator);

  return arguments;
}

class MockFlutterWindowsView : public FlutterWindowsView {
 public:
  MockFlutterWindowsView(FlutterWindowsEngine* engine,
                         std::unique_ptr<WindowBindingHandler> window)
      : FlutterWindowsView(kImplicitViewId, engine, std::move(window)) {}
  virtual ~MockFlutterWindowsView() = default;

  MOCK_METHOD(void, OnCursorRectUpdated, (const Rect&), (override));
  MOCK_METHOD(void, OnResetImeComposing, (), (override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockFlutterWindowsView);
};

}  // namespace

class TextInputPluginTest : public WindowsTest {
 public:
  TextInputPluginTest() = default;
  virtual ~TextInputPluginTest() = default;

 protected:
  FlutterWindowsEngine* engine() { return engine_.get(); }
  MockFlutterWindowsView* view() { return view_.get(); }
  MockWindowBindingHandler* window() { return window_; }

  void UseHeadlessEngine() {
    FlutterWindowsEngineBuilder builder{GetContext()};

    engine_ = builder.Build();
  }

  void UseEngineWithView() {
    FlutterWindowsEngineBuilder builder{GetContext()};

    auto window = std::make_unique<MockWindowBindingHandler>();

    window_ = window.get();
    EXPECT_CALL(*window_, SetView).Times(1);
    EXPECT_CALL(*window, GetWindowHandle).WillRepeatedly(Return(nullptr));

    engine_ = builder.Build();
    view_ = std::make_unique<MockFlutterWindowsView>(engine_.get(),
                                                     std::move(window));

    EngineModifier modifier{engine_.get()};
    modifier.SetImplicitView(view_.get());
  }

 private:
  std::unique_ptr<FlutterWindowsEngine> engine_;
  std::unique_ptr<MockFlutterWindowsView> view_;
  MockWindowBindingHandler* window_;

  FML_DISALLOW_COPY_AND_ASSIGN(TextInputPluginTest);
};

TEST_F(TextInputPluginTest, TextMethodsWorksWithEmptyModel) {
  UseEngineWithView();

  auto handled_message = CreateResponse(true);
  auto unhandled_message = CreateResponse(false);
  int received_scancode = 0;

  TestBinaryMessenger messenger(
      [&received_scancode, &handled_message, &unhandled_message](
          const std::string& channel, const uint8_t* message,
          size_t message_size, BinaryReply reply) {});

  int redispatch_scancode = 0;
  TextInputPlugin handler(&messenger, engine());

  handler.KeyboardHook(VK_RETURN, 100, WM_KEYDOWN, '\n', false, false);
  handler.ComposeBeginHook();
  std::u16string text;
  text.push_back('\n');
  handler.ComposeChangeHook(text, 1);
  handler.ComposeEndHook();

  // Passes if it did not crash
}

TEST_F(TextInputPluginTest, ClearClientResetsComposing) {
  UseEngineWithView();

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t message_size,
                                   BinaryReply reply) {});
  BinaryReply reply_handler = [](const uint8_t* reply, size_t reply_size) {};

  TextInputPlugin handler(&messenger, engine());

  EXPECT_CALL(*view(), OnResetImeComposing());

  auto& codec = JsonMethodCodec::GetInstance();
  auto message = codec.EncodeMethodCall({"TextInput.clearClient", nullptr});
  messenger.SimulateEngineMessage(kChannelName, message->data(),
                                  message->size(), reply_handler);
}

// Verify that clear client fails if in headless mode.
TEST_F(TextInputPluginTest, ClearClientRequiresView) {
  UseHeadlessEngine();

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t message_size,
                                   BinaryReply reply) {});

  std::string reply;
  BinaryReply reply_handler = [&reply](const uint8_t* reply_bytes,
                                       size_t reply_size) {
    reply = std::string(reinterpret_cast<const char*>(reply_bytes), reply_size);
  };

  TextInputPlugin handler(&messenger, engine());

  auto& codec = JsonMethodCodec::GetInstance();
  auto message = codec.EncodeMethodCall({"TextInput.clearClient", nullptr});
  messenger.SimulateEngineMessage(kChannelName, message->data(),
                                  message->size(), reply_handler);

  EXPECT_EQ(reply,
            "[\"Internal Consistency Error\",\"Text input is not available in "
            "Windows headless mode\",null]");
}

// Verify that the embedder sends state update messages to the framework during
// IME composing.
TEST_F(TextInputPluginTest, VerifyComposingSendStateUpdate) {
  UseEngineWithView();

  bool sent_message = false;
  TestBinaryMessenger messenger(
      [&sent_message](const std::string& channel, const uint8_t* message,
                      size_t message_size,
                      BinaryReply reply) { sent_message = true; });
  BinaryReply reply_handler = [](const uint8_t* reply, size_t reply_size) {};

  TextInputPlugin handler(&messenger, engine());

  auto& codec = JsonMethodCodec::GetInstance();

  // Call TextInput.setClient to initialize the TextInputModel.
  auto arguments = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = arguments->GetAllocator();
  arguments->PushBack(kDefaultClientId, allocator);
  rapidjson::Value config(rapidjson::kObjectType);
  config.AddMember("inputAction", "done", allocator);
  config.AddMember("inputType", "text", allocator);
  config.AddMember(kEnableDeltaModel, false, allocator);
  arguments->PushBack(config, allocator);
  auto message =
      codec.EncodeMethodCall({"TextInput.setClient", std::move(arguments)});
  messenger.SimulateEngineMessage("flutter/textinput", message->data(),
                                  message->size(), reply_handler);

  // ComposeBeginHook should send state update.
  sent_message = false;
  handler.ComposeBeginHook();
  EXPECT_TRUE(sent_message);

  // ComposeChangeHook should send state update.
  sent_message = false;
  handler.ComposeChangeHook(u"4", 1);
  EXPECT_TRUE(sent_message);

  // ComposeCommitHook should NOT send state update.
  //
  // Commit messages are always immediately followed by a change message or an
  // end message, both of which will send an update. Sending intermediate state
  // with a collapsed composing region will trigger the framework to assume
  // composing has ended, which is not the case until a WM_IME_ENDCOMPOSING
  // event is received in the main event loop, which will trigger a call to
  // ComposeEndHook.
  sent_message = false;
  handler.ComposeCommitHook();
  EXPECT_FALSE(sent_message);

  // ComposeEndHook should send state update.
  sent_message = false;
  handler.ComposeEndHook();
  EXPECT_TRUE(sent_message);
}

TEST_F(TextInputPluginTest, VerifyInputActionNewlineInsertNewLine) {
  UseEngineWithView();

  // Store messages as std::string for convenience.
  std::vector<std::string> messages;

  TestBinaryMessenger messenger(
      [&messages](const std::string& channel, const uint8_t* message,
                  size_t message_size, BinaryReply reply) {
        std::string last_message(reinterpret_cast<const char*>(message),
                                 message_size);
        messages.push_back(last_message);
      });
  BinaryReply reply_handler = [](const uint8_t* reply, size_t reply_size) {};

  TextInputPlugin handler(&messenger, engine());

  auto& codec = JsonMethodCodec::GetInstance();

  // Call TextInput.setClient to initialize the TextInputModel.
  auto set_client_arguments =
      EncodedClientConfig("TextInputType.multiline", "TextInputAction.newline");
  auto message = codec.EncodeMethodCall(
      {"TextInput.setClient", std::move(set_client_arguments)});
  messenger.SimulateEngineMessage("flutter/textinput", message->data(),
                                  message->size(), reply_handler);

  // Simulate a key down event for '\n'.
  handler.KeyboardHook(VK_RETURN, 100, WM_KEYDOWN, '\n', false, false);

  // Two messages are expected, the first is TextInput.updateEditingState and
  // the second is TextInputClient.performAction.
  EXPECT_EQ(messages.size(), 2);

  // Editing state should have been updated.
  auto encoded_arguments = EncodedEditingState("\n", TextRange(1));
  auto update_state_message = codec.EncodeMethodCall(
      {kUpdateEditingStateMethod, std::move(encoded_arguments)});

  EXPECT_TRUE(std::equal(update_state_message->begin(),
                         update_state_message->end(),
                         messages.front().begin()));

  // TextInputClient.performAction should have been called.
  auto arguments = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = arguments->GetAllocator();
  arguments->PushBack(kDefaultClientId, allocator);
  arguments->PushBack(
      rapidjson::Value("TextInputAction.newline", allocator).Move(), allocator);
  auto invoke_action_message = codec.EncodeMethodCall(
      {"TextInputClient.performAction", std::move(arguments)});

  EXPECT_TRUE(std::equal(invoke_action_message->begin(),
                         invoke_action_message->end(),
                         messages.back().begin()));
}

// Regression test for https://github.com/flutter/flutter/issues/125879.
TEST_F(TextInputPluginTest, VerifyInputActionSendDoesNotInsertNewLine) {
  UseEngineWithView();

  std::vector<std::vector<uint8_t>> messages;

  TestBinaryMessenger messenger(
      [&messages](const std::string& channel, const uint8_t* message,
                  size_t message_size, BinaryReply reply) {
        int length = static_cast<int>(message_size);
        std::vector<uint8_t> last_message(length);
        memcpy(&last_message[0], &message[0], length * sizeof(uint8_t));
        messages.push_back(last_message);
      });
  BinaryReply reply_handler = [](const uint8_t* reply, size_t reply_size) {};

  TextInputPlugin handler(&messenger, engine());

  auto& codec = JsonMethodCodec::GetInstance();

  // Call TextInput.setClient to initialize the TextInputModel.
  auto set_client_arguments =
      EncodedClientConfig("TextInputType.multiline", "TextInputAction.send");
  auto message = codec.EncodeMethodCall(
      {"TextInput.setClient", std::move(set_client_arguments)});
  messenger.SimulateEngineMessage("flutter/textinput", message->data(),
                                  message->size(), reply_handler);

  // Simulate a key down event for '\n'.
  handler.KeyboardHook(VK_RETURN, 100, WM_KEYDOWN, '\n', false, false);

  // Only a call to TextInputClient.performAction is expected.
  EXPECT_EQ(messages.size(), 1);

  // TextInputClient.performAction should have been called.
  auto arguments = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = arguments->GetAllocator();
  arguments->PushBack(kDefaultClientId, allocator);
  arguments->PushBack(
      rapidjson::Value("TextInputAction.send", allocator).Move(), allocator);
  auto invoke_action_message = codec.EncodeMethodCall(
      {"TextInputClient.performAction", std::move(arguments)});

  EXPECT_TRUE(std::equal(invoke_action_message->begin(),
                         invoke_action_message->end(),
                         messages.front().begin()));
}

TEST_F(TextInputPluginTest, TextEditingWorksWithDeltaModel) {
  UseEngineWithView();

  auto handled_message = CreateResponse(true);
  auto unhandled_message = CreateResponse(false);
  int received_scancode = 0;

  TestBinaryMessenger messenger(
      [&received_scancode, &handled_message, &unhandled_message](
          const std::string& channel, const uint8_t* message,
          size_t message_size, BinaryReply reply) {});

  int redispatch_scancode = 0;
  TextInputPlugin handler(&messenger, engine());

  auto args = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = args->GetAllocator();
  args->PushBack(123, allocator);  // client_id

  rapidjson::Value client_config(rapidjson::kObjectType);
  client_config.AddMember(kEnableDeltaModel, true, allocator);

  args->PushBack(client_config, allocator);
  auto encoded = JsonMethodCodec::GetInstance().EncodeMethodCall(
      MethodCall<rapidjson::Document>(kSetClientMethod, std::move(args)));

  EXPECT_TRUE(messenger.SimulateEngineMessage(
      kChannelName, encoded->data(), encoded->size(),
      [](const uint8_t* reply, size_t reply_size) {}));

  handler.KeyboardHook(VK_RETURN, 100, WM_KEYDOWN, '\n', false, false);
  handler.ComposeBeginHook();
  std::u16string text;
  text.push_back('\n');
  handler.ComposeChangeHook(text, 1);
  handler.ComposeEndHook();

  handler.KeyboardHook(0x4E, 100, WM_KEYDOWN, 'n', false, false);
  handler.ComposeBeginHook();
  std::u16string textN;
  text.push_back('n');
  handler.ComposeChangeHook(textN, 1);
  handler.KeyboardHook(0x49, 100, WM_KEYDOWN, 'i', false, false);
  std::u16string textNi;
  text.push_back('n');
  text.push_back('i');
  handler.ComposeChangeHook(textNi, 2);
  handler.KeyboardHook(VK_RETURN, 100, WM_KEYDOWN, '\n', false, false);
  std::u16string textChineseCharacter;
  text.push_back(u'\u4F60');
  handler.ComposeChangeHook(textChineseCharacter, 1);
  handler.ComposeCommitHook();
  handler.ComposeEndHook();

  // Passes if it did not crash
}

// Regression test for https://github.com/flutter/flutter/issues/123749
TEST_F(TextInputPluginTest, CompositionCursorPos) {
  UseEngineWithView();

  int selection_base = -1;
  TestBinaryMessenger messenger([&](const std::string& channel,
                                    const uint8_t* message, size_t size,
                                    BinaryReply reply) {
    auto method = JsonMethodCodec::GetInstance().DecodeMethodCall(
        std::vector<uint8_t>(message, message + size));
    if (method->method_name() == kUpdateEditingStateMethod) {
      const auto& args = *method->arguments();
      const auto& editing_state = args[1];
      auto base = editing_state.FindMember(kSelectionBaseKey);
      auto extent = editing_state.FindMember(kSelectionExtentKey);
      ASSERT_NE(base, editing_state.MemberEnd());
      ASSERT_TRUE(base->value.IsInt());
      ASSERT_NE(extent, editing_state.MemberEnd());
      ASSERT_TRUE(extent->value.IsInt());
      selection_base = base->value.GetInt();
      EXPECT_EQ(extent->value.GetInt(), selection_base);
    }
  });

  TextInputPlugin plugin(&messenger, engine());

  auto args = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = args->GetAllocator();
  args->PushBack(123, allocator);  // client_id
  rapidjson::Value client_config(rapidjson::kObjectType);
  args->PushBack(client_config, allocator);
  auto encoded = JsonMethodCodec::GetInstance().EncodeMethodCall(
      MethodCall<rapidjson::Document>(kSetClientMethod, std::move(args)));
  EXPECT_TRUE(messenger.SimulateEngineMessage(
      kChannelName, encoded->data(), encoded->size(),
      [](const uint8_t* reply, size_t reply_size) {}));

  plugin.ComposeBeginHook();
  EXPECT_EQ(selection_base, 0);
  plugin.ComposeChangeHook(u"abc", 3);
  EXPECT_EQ(selection_base, 3);

  plugin.ComposeCommitHook();
  plugin.ComposeEndHook();
  EXPECT_EQ(selection_base, 3);

  plugin.ComposeBeginHook();
  plugin.ComposeChangeHook(u"1", 1);
  EXPECT_EQ(selection_base, 4);

  plugin.ComposeChangeHook(u"12", 2);
  EXPECT_EQ(selection_base, 5);

  plugin.ComposeChangeHook(u"12", 1);
  EXPECT_EQ(selection_base, 4);

  plugin.ComposeChangeHook(u"12", 2);
  EXPECT_EQ(selection_base, 5);
}

TEST_F(TextInputPluginTest, TransformCursorRect) {
  UseEngineWithView();

  // A position of `EditableText`.
  double view_x = 100;
  double view_y = 200;

  // A position and size of marked text, in `EditableText` local coordinates.
  double ime_x = 3;
  double ime_y = 4;
  double ime_width = 50;
  double ime_height = 60;

  // Transformation matrix.
  std::array<std::array<double, 4>, 4> editabletext_transform = {
      1.0, 0.0, 0.0, view_x,  //
      0.0, 1.0, 0.0, view_y,  //
      0.0, 0.0, 0.0, 0.0,     //
      0.0, 0.0, 0.0, 1.0};

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t message_size,
                                   BinaryReply reply) {});
  BinaryReply reply_handler = [](const uint8_t* reply, size_t reply_size) {};

  TextInputPlugin handler(&messenger, engine());

  auto& codec = JsonMethodCodec::GetInstance();

  EXPECT_CALL(*view(), OnCursorRectUpdated(Rect{{view_x, view_y}, {0, 0}}));

  {
    auto arguments =
        std::make_unique<rapidjson::Document>(rapidjson::kObjectType);
    auto& allocator = arguments->GetAllocator();

    rapidjson::Value transoform(rapidjson::kArrayType);
    for (int i = 0; i < 4 * 4; i++) {
      // Pack 2-dimensional array by column-major order.
      transoform.PushBack(editabletext_transform[i % 4][i / 4], allocator);
    }

    arguments->AddMember("transform", transoform, allocator);

    auto message = codec.EncodeMethodCall(
        {"TextInput.setEditableSizeAndTransform", std::move(arguments)});
    messenger.SimulateEngineMessage(kChannelName, message->data(),
                                    message->size(), reply_handler);
  }

  EXPECT_CALL(*view(),
              OnCursorRectUpdated(Rect{{view_x + ime_x, view_y + ime_y},
                                       {ime_width, ime_height}}));

  {
    auto arguments =
        std::make_unique<rapidjson::Document>(rapidjson::kObjectType);
    auto& allocator = arguments->GetAllocator();

    arguments->AddMember("x", ime_x, allocator);
    arguments->AddMember("y", ime_y, allocator);
    arguments->AddMember("width", ime_width, allocator);
    arguments->AddMember("height", ime_height, allocator);

    auto message = codec.EncodeMethodCall(
        {"TextInput.setMarkedTextRect", std::move(arguments)});
    messenger.SimulateEngineMessage(kChannelName, message->data(),
                                    message->size(), reply_handler);
  }
}

TEST_F(TextInputPluginTest, SetMarkedTextRectRequiresView) {
  UseHeadlessEngine();

  TestBinaryMessenger messenger([](const std::string& channel,
                                   const uint8_t* message, size_t message_size,
                                   BinaryReply reply) {});

  std::string reply;
  BinaryReply reply_handler = [&reply](const uint8_t* reply_bytes,
                                       size_t reply_size) {
    reply = std::string(reinterpret_cast<const char*>(reply_bytes), reply_size);
  };

  TextInputPlugin handler(&messenger, engine());

  auto& codec = JsonMethodCodec::GetInstance();

  auto arguments =
      std::make_unique<rapidjson::Document>(rapidjson::kObjectType);
  auto& allocator = arguments->GetAllocator();

  arguments->AddMember("x", 0, allocator);
  arguments->AddMember("y", 0, allocator);
  arguments->AddMember("width", 0, allocator);
  arguments->AddMember("height", 0, allocator);

  auto message = codec.EncodeMethodCall(
      {"TextInput.setMarkedTextRect", std::move(arguments)});
  messenger.SimulateEngineMessage(kChannelName, message->data(),
                                  message->size(), reply_handler);

  EXPECT_EQ(reply,
            "[\"Internal Consistency Error\",\"Text input is not available in "
            "Windows headless mode\",null]");
}

}  // namespace testing
}  // namespace flutter
