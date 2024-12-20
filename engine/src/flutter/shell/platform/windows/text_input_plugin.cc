// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/text_input_plugin.h"

#include <windows.h>

#include <cstdint>

#include "flutter/fml/string_conversion.h"
#include "flutter/shell/platform/common/json_method_codec.h"
#include "flutter/shell/platform/common/text_editing_delta.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"

static constexpr char kSetEditingStateMethod[] = "TextInput.setEditingState";
static constexpr char kClearClientMethod[] = "TextInput.clearClient";
static constexpr char kSetClientMethod[] = "TextInput.setClient";
static constexpr char kShowMethod[] = "TextInput.show";
static constexpr char kHideMethod[] = "TextInput.hide";
static constexpr char kSetMarkedTextRect[] = "TextInput.setMarkedTextRect";
static constexpr char kSetEditableSizeAndTransform[] =
    "TextInput.setEditableSizeAndTransform";

static constexpr char kMultilineInputType[] = "TextInputType.multiline";

static constexpr char kUpdateEditingStateMethod[] =
    "TextInputClient.updateEditingState";
static constexpr char kUpdateEditingStateWithDeltasMethod[] =
    "TextInputClient.updateEditingStateWithDeltas";
static constexpr char kPerformActionMethod[] = "TextInputClient.performAction";

static constexpr char kDeltaOldTextKey[] = "oldText";
static constexpr char kDeltaTextKey[] = "deltaText";
static constexpr char kDeltaStartKey[] = "deltaStart";
static constexpr char kDeltaEndKey[] = "deltaEnd";
static constexpr char kDeltasKey[] = "deltas";
static constexpr char kEnableDeltaModel[] = "enableDeltaModel";
static constexpr char kTextInputAction[] = "inputAction";
static constexpr char kTextInputType[] = "inputType";
static constexpr char kTextInputTypeName[] = "name";
static constexpr char kComposingBaseKey[] = "composingBase";
static constexpr char kComposingExtentKey[] = "composingExtent";
static constexpr char kSelectionAffinityKey[] = "selectionAffinity";
static constexpr char kAffinityDownstream[] = "TextAffinity.downstream";
static constexpr char kSelectionBaseKey[] = "selectionBase";
static constexpr char kSelectionExtentKey[] = "selectionExtent";
static constexpr char kSelectionIsDirectionalKey[] = "selectionIsDirectional";
static constexpr char kTextKey[] = "text";
static constexpr char kXKey[] = "x";
static constexpr char kYKey[] = "y";
static constexpr char kWidthKey[] = "width";
static constexpr char kHeightKey[] = "height";
static constexpr char kTransformKey[] = "transform";

static constexpr char kChannelName[] = "flutter/textinput";

static constexpr char kBadArgumentError[] = "Bad Arguments";
static constexpr char kInternalConsistencyError[] =
    "Internal Consistency Error";

static constexpr char kInputActionNewline[] = "TextInputAction.newline";

namespace flutter {

void TextInputPlugin::TextHook(const std::u16string& text) {
  if (active_model_ == nullptr) {
    return;
  }
  std::u16string text_before_change =
      fml::Utf8ToUtf16(active_model_->GetText());
  TextRange selection_before_change = active_model_->selection();
  active_model_->AddText(text);

  if (enable_delta_model) {
    TextEditingDelta delta =
        TextEditingDelta(text_before_change, selection_before_change, text);
    SendStateUpdateWithDelta(*active_model_, &delta);
  } else {
    SendStateUpdate(*active_model_);
  }
}

void TextInputPlugin::KeyboardHook(int key,
                                   int scancode,
                                   int action,
                                   char32_t character,
                                   bool extended,
                                   bool was_down) {
  if (active_model_ == nullptr) {
    return;
  }
  if (action == WM_KEYDOWN || action == WM_SYSKEYDOWN) {
    // Most editing keys (arrow keys, backspace, delete, etc.) are handled in
    // the framework, so don't need to be handled at this layer.
    switch (key) {
      case VK_RETURN:
        EnterPressed(active_model_.get());
        break;
      default:
        break;
    }
  }
}

TextInputPlugin::TextInputPlugin(flutter::BinaryMessenger* messenger,
                                 FlutterWindowsEngine* engine)
    : channel_(std::make_unique<flutter::MethodChannel<rapidjson::Document>>(
          messenger,
          kChannelName,
          &flutter::JsonMethodCodec::GetInstance())),
      engine_(engine),
      active_model_(nullptr) {
  channel_->SetMethodCallHandler(
      [this](
          const flutter::MethodCall<rapidjson::Document>& call,
          std::unique_ptr<flutter::MethodResult<rapidjson::Document>> result) {
        HandleMethodCall(call, std::move(result));
      });
}

TextInputPlugin::~TextInputPlugin() = default;

void TextInputPlugin::ComposeBeginHook() {
  if (active_model_ == nullptr) {
    return;
  }
  active_model_->BeginComposing();
  if (enable_delta_model) {
    std::string text = active_model_->GetText();
    TextRange selection = active_model_->selection();
    TextEditingDelta delta = TextEditingDelta(text);
    SendStateUpdateWithDelta(*active_model_, &delta);
  } else {
    SendStateUpdate(*active_model_);
  }
}

void TextInputPlugin::ComposeCommitHook() {
  if (active_model_ == nullptr) {
    return;
  }
  std::string text_before_change = active_model_->GetText();
  TextRange selection_before_change = active_model_->selection();
  TextRange composing_before_change = active_model_->composing_range();
  std::string composing_text_before_change = text_before_change.substr(
      composing_before_change.start(), composing_before_change.length());
  active_model_->CommitComposing();

  // We do not trigger SendStateUpdate here.
  //
  // Until a WM_IME_ENDCOMPOSING event, the user is still composing from the OS
  // point of view. Commit events are always immediately followed by another
  // composing event or an end composing event. However, in the brief window
  // between the commit event and the following event, the composing region is
  // collapsed. Notifying the framework of this intermediate state will trigger
  // any framework code designed to execute at the end of composing, such as
  // input formatters, which may try to update the text and send a message back
  // to the engine with changes.
  //
  // This is a particular problem with Korean IMEs, which build up one
  // character at a time in their composing region until a keypress that makes
  // no sense for the in-progress character. At that point, the result
  // character is committed and a compose event is immedidately received with
  // the new composing region.
  //
  // In the case where this event is immediately followed by a composing event,
  // the state will be sent in ComposeChangeHook.
  //
  // In the case where this event is immediately followed by an end composing
  // event, the state will be sent in ComposeEndHook.
}

void TextInputPlugin::ComposeEndHook() {
  if (active_model_ == nullptr) {
    return;
  }
  std::string text_before_change = active_model_->GetText();
  TextRange selection_before_change = active_model_->selection();
  active_model_->CommitComposing();
  active_model_->EndComposing();
  if (enable_delta_model) {
    std::string text = active_model_->GetText();
    TextEditingDelta delta = TextEditingDelta(text);
    SendStateUpdateWithDelta(*active_model_, &delta);
  } else {
    SendStateUpdate(*active_model_);
  }
}

void TextInputPlugin::ComposeChangeHook(const std::u16string& text,
                                        int cursor_pos) {
  if (active_model_ == nullptr) {
    return;
  }
  std::string text_before_change = active_model_->GetText();
  TextRange composing_before_change = active_model_->composing_range();
  active_model_->AddText(text);
  active_model_->UpdateComposingText(text, TextRange(cursor_pos, cursor_pos));
  std::string text_after_change = active_model_->GetText();
  if (enable_delta_model) {
    TextEditingDelta delta = TextEditingDelta(
        fml::Utf8ToUtf16(text_before_change), composing_before_change, text);
    SendStateUpdateWithDelta(*active_model_, &delta);
  } else {
    SendStateUpdate(*active_model_);
  }
}

void TextInputPlugin::HandleMethodCall(
    const flutter::MethodCall<rapidjson::Document>& method_call,
    std::unique_ptr<flutter::MethodResult<rapidjson::Document>> result) {
  const std::string& method = method_call.method_name();

  if (method.compare(kShowMethod) == 0 || method.compare(kHideMethod) == 0) {
    // These methods are no-ops.
  } else if (method.compare(kClearClientMethod) == 0) {
    // TODO(loicsharma): Remove implicit view assumption.
    // https://github.com/flutter/flutter/issues/142845
    FlutterWindowsView* view = engine_->view(kImplicitViewId);
    if (view == nullptr) {
      result->Error(kInternalConsistencyError,
                    "Text input is not available in Windows headless mode");
      return;
    }
    if (active_model_ != nullptr && active_model_->composing()) {
      active_model_->CommitComposing();
      active_model_->EndComposing();
      SendStateUpdate(*active_model_);
    }
    view->OnResetImeComposing();
    active_model_ = nullptr;
  } else if (method.compare(kSetClientMethod) == 0) {
    if (!method_call.arguments() || method_call.arguments()->IsNull()) {
      result->Error(kBadArgumentError, "Method invoked without args");
      return;
    }
    const rapidjson::Document& args = *method_call.arguments();

    const rapidjson::Value& client_id_json = args[0];
    const rapidjson::Value& client_config = args[1];
    if (client_id_json.IsNull()) {
      result->Error(kBadArgumentError, "Could not set client, ID is null.");
      return;
    }
    if (client_config.IsNull()) {
      result->Error(kBadArgumentError,
                    "Could not set client, missing arguments.");
      return;
    }
    client_id_ = client_id_json.GetInt();
    auto enable_delta_model_json = client_config.FindMember(kEnableDeltaModel);
    if (enable_delta_model_json != client_config.MemberEnd() &&
        enable_delta_model_json->value.IsBool()) {
      enable_delta_model = enable_delta_model_json->value.GetBool();
    }
    input_action_ = "";
    auto input_action_json = client_config.FindMember(kTextInputAction);
    if (input_action_json != client_config.MemberEnd() &&
        input_action_json->value.IsString()) {
      input_action_ = input_action_json->value.GetString();
    }
    input_type_ = "";
    auto input_type_info_json = client_config.FindMember(kTextInputType);
    if (input_type_info_json != client_config.MemberEnd() &&
        input_type_info_json->value.IsObject()) {
      auto input_type_json =
          input_type_info_json->value.FindMember(kTextInputTypeName);
      if (input_type_json != input_type_info_json->value.MemberEnd() &&
          input_type_json->value.IsString()) {
        input_type_ = input_type_json->value.GetString();
      }
    }
    active_model_ = std::make_unique<TextInputModel>();
  } else if (method.compare(kSetEditingStateMethod) == 0) {
    if (!method_call.arguments() || method_call.arguments()->IsNull()) {
      result->Error(kBadArgumentError, "Method invoked without args");
      return;
    }
    const rapidjson::Document& args = *method_call.arguments();

    if (active_model_ == nullptr) {
      result->Error(
          kInternalConsistencyError,
          "Set editing state has been invoked, but no client is set.");
      return;
    }
    auto text = args.FindMember(kTextKey);
    if (text == args.MemberEnd() || text->value.IsNull()) {
      result->Error(kBadArgumentError,
                    "Set editing state has been invoked, but without text.");
      return;
    }
    auto base = args.FindMember(kSelectionBaseKey);
    auto extent = args.FindMember(kSelectionExtentKey);
    if (base == args.MemberEnd() || base->value.IsNull() ||
        extent == args.MemberEnd() || extent->value.IsNull()) {
      result->Error(kInternalConsistencyError,
                    "Selection base/extent values invalid.");
      return;
    }
    // Flutter uses -1/-1 for invalid; translate that to 0/0 for the model.
    int selection_base = base->value.GetInt();
    int selection_extent = extent->value.GetInt();
    if (selection_base == -1 && selection_extent == -1) {
      selection_base = selection_extent = 0;
    }
    active_model_->SetText(text->value.GetString());
    active_model_->SetSelection(TextRange(selection_base, selection_extent));

    base = args.FindMember(kComposingBaseKey);
    extent = args.FindMember(kComposingExtentKey);
    if (base == args.MemberEnd() || base->value.IsNull() ||
        extent == args.MemberEnd() || extent->value.IsNull()) {
      result->Error(kInternalConsistencyError,
                    "Composing base/extent values invalid.");
      return;
    }
    int composing_base = base->value.GetInt();
    int composing_extent = base->value.GetInt();
    if (composing_base == -1 && composing_extent == -1) {
      active_model_->EndComposing();
    } else {
      int composing_start = std::min(composing_base, composing_extent);
      int cursor_offset = selection_base - composing_start;
      active_model_->SetComposingRange(
          TextRange(composing_base, composing_extent), cursor_offset);
    }
  } else if (method.compare(kSetMarkedTextRect) == 0) {
    // TODO(loicsharma): Remove implicit view assumption.
    // https://github.com/flutter/flutter/issues/142845
    FlutterWindowsView* view = engine_->view(kImplicitViewId);
    if (view == nullptr) {
      result->Error(kInternalConsistencyError,
                    "Text input is not available in Windows headless mode");
      return;
    }
    if (!method_call.arguments() || method_call.arguments()->IsNull()) {
      result->Error(kBadArgumentError, "Method invoked without args");
      return;
    }
    const rapidjson::Document& args = *method_call.arguments();
    auto x = args.FindMember(kXKey);
    auto y = args.FindMember(kYKey);
    auto width = args.FindMember(kWidthKey);
    auto height = args.FindMember(kHeightKey);
    if (x == args.MemberEnd() || x->value.IsNull() ||          //
        y == args.MemberEnd() || y->value.IsNull() ||          //
        width == args.MemberEnd() || width->value.IsNull() ||  //
        height == args.MemberEnd() || height->value.IsNull()) {
      result->Error(kInternalConsistencyError,
                    "Composing rect values invalid.");
      return;
    }
    composing_rect_ = {{x->value.GetDouble(), y->value.GetDouble()},
                       {width->value.GetDouble(), height->value.GetDouble()}};

    Rect transformed_rect = GetCursorRect();
    view->OnCursorRectUpdated(transformed_rect);
  } else if (method.compare(kSetEditableSizeAndTransform) == 0) {
    // TODO(loicsharma): Remove implicit view assumption.
    // https://github.com/flutter/flutter/issues/142845
    FlutterWindowsView* view = engine_->view(kImplicitViewId);
    if (view == nullptr) {
      result->Error(kInternalConsistencyError,
                    "Text input is not available in Windows headless mode");
      return;
    }
    if (!method_call.arguments() || method_call.arguments()->IsNull()) {
      result->Error(kBadArgumentError, "Method invoked without args");
      return;
    }
    const rapidjson::Document& args = *method_call.arguments();
    auto transform = args.FindMember(kTransformKey);
    if (transform == args.MemberEnd() || transform->value.IsNull() ||
        !transform->value.IsArray() || transform->value.Size() != 16) {
      result->Error(kInternalConsistencyError,
                    "EditableText transform invalid.");
      return;
    }
    size_t i = 0;
    for (auto& entry : transform->value.GetArray()) {
      if (entry.IsNull()) {
        result->Error(kInternalConsistencyError,
                      "EditableText transform contains null value.");
        return;
      }
      editabletext_transform_[i / 4][i % 4] = entry.GetDouble();
      ++i;
    }
    Rect transformed_rect = GetCursorRect();
    view->OnCursorRectUpdated(transformed_rect);
  } else {
    result->NotImplemented();
    return;
  }
  // All error conditions return early, so if nothing has gone wrong indicate
  // success.
  result->Success();
}

Rect TextInputPlugin::GetCursorRect() const {
  Point transformed_point = {
      composing_rect_.left() * editabletext_transform_[0][0] +
          composing_rect_.top() * editabletext_transform_[1][0] +
          editabletext_transform_[3][0],
      composing_rect_.left() * editabletext_transform_[0][1] +
          composing_rect_.top() * editabletext_transform_[1][1] +
          editabletext_transform_[3][1]};
  return {transformed_point, composing_rect_.size()};
}

void TextInputPlugin::SendStateUpdate(const TextInputModel& model) {
  auto args = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = args->GetAllocator();
  args->PushBack(client_id_, allocator);

  TextRange selection = model.selection();
  rapidjson::Value editing_state(rapidjson::kObjectType);
  editing_state.AddMember(kSelectionAffinityKey, kAffinityDownstream,
                          allocator);
  editing_state.AddMember(kSelectionBaseKey, selection.base(), allocator);
  editing_state.AddMember(kSelectionExtentKey, selection.extent(), allocator);
  editing_state.AddMember(kSelectionIsDirectionalKey, false, allocator);

  int composing_base = model.composing() ? model.composing_range().base() : -1;
  int composing_extent =
      model.composing() ? model.composing_range().extent() : -1;
  editing_state.AddMember(kComposingBaseKey, composing_base, allocator);
  editing_state.AddMember(kComposingExtentKey, composing_extent, allocator);
  editing_state.AddMember(
      kTextKey, rapidjson::Value(model.GetText(), allocator).Move(), allocator);
  args->PushBack(editing_state, allocator);

  channel_->InvokeMethod(kUpdateEditingStateMethod, std::move(args));
}

void TextInputPlugin::SendStateUpdateWithDelta(const TextInputModel& model,
                                               const TextEditingDelta* delta) {
  auto args = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = args->GetAllocator();
  args->PushBack(client_id_, allocator);

  rapidjson::Value object(rapidjson::kObjectType);
  rapidjson::Value deltas(rapidjson::kArrayType);
  rapidjson::Value deltaJson(rapidjson::kObjectType);

  deltaJson.AddMember(kDeltaOldTextKey, delta->old_text(), allocator);
  deltaJson.AddMember(kDeltaTextKey, delta->delta_text(), allocator);
  deltaJson.AddMember(kDeltaStartKey, delta->delta_start(), allocator);
  deltaJson.AddMember(kDeltaEndKey, delta->delta_end(), allocator);

  TextRange selection = model.selection();
  deltaJson.AddMember(kSelectionAffinityKey, kAffinityDownstream, allocator);
  deltaJson.AddMember(kSelectionBaseKey, selection.base(), allocator);
  deltaJson.AddMember(kSelectionExtentKey, selection.extent(), allocator);
  deltaJson.AddMember(kSelectionIsDirectionalKey, false, allocator);

  int composing_base = model.composing() ? model.composing_range().base() : -1;
  int composing_extent =
      model.composing() ? model.composing_range().extent() : -1;
  deltaJson.AddMember(kComposingBaseKey, composing_base, allocator);
  deltaJson.AddMember(kComposingExtentKey, composing_extent, allocator);

  deltas.PushBack(deltaJson, allocator);
  object.AddMember(kDeltasKey, deltas, allocator);
  args->PushBack(object, allocator);

  channel_->InvokeMethod(kUpdateEditingStateWithDeltasMethod, std::move(args));
}

void TextInputPlugin::EnterPressed(TextInputModel* model) {
  if (input_type_ == kMultilineInputType &&
      input_action_ == kInputActionNewline) {
    std::u16string text_before_change = fml::Utf8ToUtf16(model->GetText());
    TextRange selection_before_change = model->selection();
    model->AddText(u"\n");
    if (enable_delta_model) {
      TextEditingDelta delta(text_before_change, selection_before_change,
                             u"\n");
      SendStateUpdateWithDelta(*model, &delta);
    } else {
      SendStateUpdate(*model);
    }
  }
  auto args = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = args->GetAllocator();
  args->PushBack(client_id_, allocator);
  args->PushBack(rapidjson::Value(input_action_, allocator).Move(), allocator);

  channel_->InvokeMethod(kPerformActionMethod, std::move(args));
}

}  // namespace flutter
