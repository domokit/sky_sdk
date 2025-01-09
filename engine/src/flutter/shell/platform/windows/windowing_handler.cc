// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/windowing_handler.h"

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"

namespace {

// Name of the windowing channel.
constexpr char kChannelName[] = "flutter/windowing";

// Methods for creating different types of windows.
constexpr char kCreateWindowMethod[] = "createWindow";

// The method to destroy a window.
constexpr char kDestroyWindowMethod[] = "destroyWindow";

// Keys used in method calls.
constexpr char kAnchorRectKey[] = "anchorRect";
constexpr char kArchetypeKey[] = "archetype";
constexpr char kParentKey[] = "parent";
constexpr char kParentViewIdKey[] = "parentViewId";
constexpr char kPositionerChildAnchorKey[] = "positionerChildAnchor";
constexpr char kPositionerConstraintAdjustmentKey[] =
    "positionerConstraintAdjustment";
constexpr char kPositionerOffsetKey[] = "positionerOffset";
constexpr char kPositionerParentAnchorKey[] = "positionerParentAnchor";
constexpr char kSizeKey[] = "size";
constexpr char kViewIdKey[] = "viewId";

// Error codes used for responses.
constexpr char kInvalidValueError[] = "Invalid Value";
constexpr char kUnavailableError[] = "Unavailable";

// Retrieves the value associated with |key| from |map|, ensuring it matches
// the expected type |T|. Returns the value if found and correctly typed,
// otherwise logs an error in |result| and returns std::nullopt.
template <typename T>
std::optional<T> GetSingleValueForKeyOrSendError(
    std::string const& key,
    flutter::EncodableMap const* map,
    flutter::MethodResult<>& result) {
  if (auto const it = map->find(flutter::EncodableValue(key));
      it != map->end()) {
    if (auto const* const value = std::get_if<T>(&it->second)) {
      return *value;
    } else {
      result.Error(kInvalidValueError, "Value for '" + key +
                                           "' key must be of type '" +
                                           typeid(T).name() + "'.");
    }
  } else {
    result.Error(kInvalidValueError,
                 "Map does not contain required '" + key + "' key.");
  }
  return std::nullopt;
}

// Retrieves a list of values associated with |key| from |map|, ensuring the
// list has |Size| elements, all of type |T|. Returns the list if found and
// valid, otherwise logs an error in |result| and returns std::nullopt.
template <typename T, size_t Size>
std::optional<std::vector<T>> GetListValuesForKeyOrSendError(
    std::string const& key,
    flutter::EncodableMap const* map,
    flutter::MethodResult<>& result) {
  if (auto const it = map->find(flutter::EncodableValue(key));
      it != map->end()) {
    if (auto const* const array =
            std::get_if<std::vector<flutter::EncodableValue>>(&it->second)) {
      if (array->size() != Size) {
        result.Error(kInvalidValueError, "Array for '" + key +
                                             "' key must have " +
                                             std::to_string(Size) + " values.");
        return std::nullopt;
      }
      std::vector<T> decoded_values;
      for (flutter::EncodableValue const& value : *array) {
        if (std::holds_alternative<T>(value)) {
          decoded_values.push_back(std::get<T>(value));
        } else {
          result.Error(kInvalidValueError,
                       "Array for '" + key +
                           "' key must only have values of type '" +
                           typeid(T).name() + "'.");
          return std::nullopt;
        }
      }
      return decoded_values;
    } else {
      result.Error(kInvalidValueError,
                   "Value for '" + key + "' key must be an array.");
    }
  } else {
    result.Error(kInvalidValueError,
                 "Map does not contain required '" + key + "' key.");
  }
  return std::nullopt;
}

// Converts a |flutter::WindowArchetype| to its corresponding wide string
// representation.
std::wstring ArchetypeToWideString(flutter::WindowArchetype archetype) {
  switch (archetype) {
    case flutter::WindowArchetype::regular:
      return L"regular";
  }
  FML_UNREACHABLE();
}

}  // namespace

namespace flutter {

WindowingHandler::WindowingHandler(BinaryMessenger* messenger,
                                   FlutterHostWindowController* controller)
    : channel_(std::make_shared<MethodChannel<EncodableValue>>(
          messenger,
          kChannelName,
          &StandardMethodCodec::GetInstance())),
      controller_(controller) {
  channel_->SetMethodCallHandler(
      [this](const MethodCall<EncodableValue>& call,
             std::unique_ptr<MethodResult<EncodableValue>> result) {
        HandleMethodCall(call, std::move(result));
      });
  controller_->SetMethodChannel(channel_);
}

void WindowingHandler::HandleMethodCall(
    const MethodCall<EncodableValue>& method_call,
    std::unique_ptr<MethodResult<EncodableValue>> result) {
  const std::string& method = method_call.method_name();

  if (method == kCreateWindowMethod) {
    HandleCreateWindow(WindowArchetype::regular, method_call, *result);
  } else if (method == kDestroyWindowMethod) {
    HandleDestroyWindow(method_call, *result);
  } else {
    result->NotImplemented();
  }
}

void WindowingHandler::HandleCreateWindow(WindowArchetype archetype,
                                          MethodCall<> const& call,
                                          MethodResult<>& result) {
  auto const* const arguments = call.arguments();
  auto const* const map = std::get_if<EncodableMap>(arguments);
  if (!map) {
    result.Error(kInvalidValueError, "Method call argument is not a map.");
    return;
  }

  std::wstring const title = ArchetypeToWideString(archetype);

  auto const size_list =
      GetListValuesForKeyOrSendError<int, 2>(kSizeKey, map, result);
  if (!size_list) {
    return;
  }
  if (size_list->at(0) < 0 || size_list->at(1) < 0) {
    result.Error(kInvalidValueError,
                 "Values for '" + std::string(kSizeKey) + "' key (" +
                     std::to_string(size_list->at(0)) + ", " +
                     std::to_string(size_list->at(1)) +
                     ") must be nonnegative.");
    return;
  }

  if (std::optional<WindowMetadata> const data_opt =
          controller_->CreateHostWindow(
              title, {.width = size_list->at(0), .height = size_list->at(1)},
              archetype)) {
    WindowMetadata const& data = data_opt.value();
    result.Success(EncodableValue(EncodableMap{
        {EncodableValue(kViewIdKey), EncodableValue(data.view_id)},
        {EncodableValue(kArchetypeKey),
         EncodableValue(static_cast<int>(data.archetype))},
        {EncodableValue(kSizeKey),
         EncodableValue(EncodableList{EncodableValue(data.size.width),
                                      EncodableValue(data.size.height)})},
        {EncodableValue(kParentViewIdKey),
         data.parent_id ? EncodableValue(data.parent_id.value())
                        : EncodableValue()}}));
  } else {
    result.Error(kUnavailableError, "Can't create window.");
  }
}

void WindowingHandler::HandleDestroyWindow(MethodCall<> const& call,
                                           MethodResult<>& result) {
  auto const* const arguments = call.arguments();
  auto const* const map = std::get_if<EncodableMap>(arguments);
  if (!map) {
    result.Error(kInvalidValueError, "Method call argument is not a map.");
    return;
  }

  auto const view_id =
      GetSingleValueForKeyOrSendError<int>(kViewIdKey, map, result);
  if (!view_id) {
    return;
  }
  if (view_id.value() < 0) {
    result.Error(kInvalidValueError,
                 "Value for '" + std::string(kViewIdKey) + "' (" +
                     std::to_string(view_id.value()) + ") cannot be negative.");
    return;
  }

  if (!controller_->DestroyHostWindow(view_id.value())) {
    result.Error(kInvalidValueError,
                 "Can't find window with '" + std::string(kViewIdKey) + "' (" +
                     std::to_string(view_id.value()) + ").");
    return;
  }

  result.Success();
}

}  // namespace flutter
