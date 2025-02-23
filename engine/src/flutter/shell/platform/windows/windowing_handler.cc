// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/windowing_handler.h"

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "flutter/shell/platform/common/windowing.h"

namespace {

// Name of the windowing channel.
constexpr char kChannelName[] = "flutter/windowing";

// Methods for creating different types of windows.
constexpr char kCreateRegularMethod[] = "createRegular";

// Methods for modifying the attributes of windows.
constexpr char kModifyRegularMethod[] = "modifyRegular";

// The method to destroy a window.
constexpr char kDestroyWindowMethod[] = "destroyWindow";

// Keys used in method calls.
constexpr char kMaxSizeKey[] = "maxSize";
constexpr char kMinSizeKey[] = "minSize";
constexpr char kSizeKey[] = "size";
constexpr char kStateKey[] = "state";
constexpr char kTitleKey[] = "title";
constexpr char kViewIdKey[] = "viewId";

// Error codes used for responses.
constexpr char kBadArgumentsError[] = "Bad Arguments";
constexpr char kUnavailableError[] = "Unavailable";

// Retrieves the value associated with |key| from |map|, ensuring it matches
// the expected type |T| or std::monostate. Returns a pair where the first
// element is the value if found and correctly typed (or std::nullopt if the
// value is null), and the second element is a boolean indicating success (true)
// or error (false). If |key| doesn't exist, or if it exists but is not of
// either type |T| or std::monostate, an error is logged in |result| and the
// function returns {std::nullopt, false}.
template <typename T>
std::pair<std::optional<T>, bool> GetSingleValueForKeyOrSendError(
    std::string const& key,
    flutter::EncodableMap const* map,
    flutter::MethodResult<>& result) {
  auto const it = map->find(flutter::EncodableValue(key));
  if (it == map->end()) {
    result.Error(kBadArgumentsError,
                 "Map does not contain required '" + key + "' key.");
    return {std::nullopt, false};
  }
  if (auto const* const value = std::get_if<T>(&it->second)) {
    return {*value, true};
  }
  if (std::holds_alternative<std::monostate>(it->second)) {
    return {std::nullopt, true};
  }

  result.Error(kBadArgumentsError, "Value for '" + key +
                                       "' key must be of type '" +
                                       typeid(T).name() + "'.");
  return {std::nullopt, false};
}

// Retrieves a list of values associated with |key| from |map|, ensuring the
// list has only elements of type |T|. If |Size| is provided and greater than 0,
// the number of elements in the list is also checked. Returns a pair where the
// first element is the list of values and the second element is a boolean
// indicating success (true) or error (false). The first element of the pair is
// set to std::nullopt on error or if the value for the key is null. If |key|
// doesn't exist, or if it exists but is not of either type std::vector<T> or
// std::monostate, an error is logged in |result| and the function returns
// {std::nullopt, false}.
template <typename T, size_t Size = 0>
std::pair<std::optional<std::vector<T>>, bool> GetListOfValuesForKeyOrSendError(
    std::string const& key,
    flutter::EncodableMap const* map,
    flutter::MethodResult<>& result) {
  auto const it = map->find(flutter::EncodableValue(key));
  if (it == map->end()) {
    result.Error(kBadArgumentsError,
                 "Map does not contain required '" + key + "' key.");
    return {std::nullopt, false};
  }
  if (std::holds_alternative<std::monostate>(it->second)) {
    return {std::nullopt, true};
  }
  if (auto const* const array =
          std::get_if<std::vector<flutter::EncodableValue>>(&it->second)) {
    if (Size && array->size() != Size) {
      result.Error(kBadArgumentsError, "Array for '" + key +
                                           "' key must have " +
                                           std::to_string(Size) + " values.");
      return {std::nullopt, false};
    }
    std::vector<T> decoded_values;
    if (Size) {
      decoded_values.reserve(Size);
    }
    for (flutter::EncodableValue const& value : *array) {
      if (std::holds_alternative<T>(value)) {
        decoded_values.push_back(std::get<T>(value));
      } else {
        result.Error(kBadArgumentsError,
                     "Array for '" + key +
                         "' key must only have values of type '" +
                         typeid(T).name() + "'.");
        return {std::nullopt, false};
      }
    }
    return {std::move(decoded_values), true};
  }

  result.Error(kBadArgumentsError,
               "Value for key '" + key + "' key must be an array.");
  return {std::nullopt, false};
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

  if (method == kCreateRegularMethod) {
    HandleCreateWindow(WindowArchetype::kRegular, method_call, *result);
  } else if (method == kModifyRegularMethod) {
    HandleModifyWindow(WindowArchetype::kRegular, method_call, *result);
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
    result.Error(kBadArgumentsError, "Method call argument is not a map.");
    return;
  }

  WindowCreationSettings settings;

  // Get value for the 'size' key (non-nullable).
  if (auto const [size_list, success] =
          GetListOfValuesForKeyOrSendError<double, 2>(kSizeKey, map, result);
      success) {
    if (!size_list.has_value()) {
      result.Error(kBadArgumentsError, "Value for the '" +
                                           std::string(kSizeKey) +
                                           "' key must not be null.");
      return;
    }
    settings.size = {size_list->at(0), size_list->at(1)};
  } else {
    return;
  }

  if (archetype == WindowArchetype::kRegular) {
    // Get value for the 'minSize' key (nullable).
    if (auto const [list, success] =
            GetListOfValuesForKeyOrSendError<double, 2>(kMinSizeKey, map,
                                                        result);
        success) {
      if (list.has_value()) {
        settings.min_size = {list->at(0), list->at(1)};
      }
    } else {
      return;
    }
    // Get value for the 'maxSize' key (nullable).
    if (auto const [list, success] =
            GetListOfValuesForKeyOrSendError<double, 2>(kMaxSizeKey, map,
                                                        result);
        success) {
      if (list.has_value()) {
        settings.max_size = {list->at(0), list->at(1)};
      }
    } else {
      return;
    }
    // Get value for the 'title' key (nullable).
    if (auto const [title, success] =
            GetSingleValueForKeyOrSendError<std::string>(kTitleKey, map,
                                                         result);
        success) {
      settings.title = title;
    } else {
      return;
    }
    // Get value for the 'state' key (nullable).
    if (auto const [state, success] =
            GetSingleValueForKeyOrSendError<std::string>(kStateKey, map,
                                                         result);
        success) {
      if (state) {
        settings.state = StringToWindowState(*state);
      }
    } else {
      return;
    }
  }

  if (std::optional<WindowMetadata> const data_opt =
          controller_->CreateHostWindow(settings)) {
    WindowMetadata const& data = data_opt.value();
    EncodableMap map;

    if (archetype == WindowArchetype::kRegular) {
      map.insert({EncodableValue(kViewIdKey), EncodableValue(data.view_id)});
      map.insert(
          {EncodableValue(kSizeKey),
           EncodableValue(EncodableList{EncodableValue(data.size.width()),
                                        EncodableValue(data.size.height())})});
      assert(data.state.has_value());
      map.insert({EncodableValue(kStateKey),
                  EncodableValue(WindowStateToString(data.state.value()))});
    }

    result.Success(EncodableValue(map));
  } else {
    result.Error(kUnavailableError, "Can't create window.");
  }
}

void WindowingHandler::HandleModifyWindow(WindowArchetype archetype,
                                          MethodCall<> const& call,
                                          MethodResult<>& result) {
  auto const* const arguments = call.arguments();
  auto const* const map = std::get_if<EncodableMap>(arguments);
  if (!map) {
    result.Error(kBadArgumentsError, "Method call argument is not a map.");
    return;
  }

  FlutterViewId view_id = {};
  WindowModificationSettings settings;

  // Get value for the 'viewId' key (non-nullable)
  if (auto const [data, success] =
          GetSingleValueForKeyOrSendError<int>(kViewIdKey, map, result);
      success) {
    if (!data.has_value()) {
      result.Error(kBadArgumentsError, "Value for the '" +
                                           std::string(kViewIdKey) +
                                           "' key must not be null.");
      return;
    }
    view_id = *data;
  } else {
    return;
  }
  // Get value for the 'size' key (nullable).
  if (auto const [data, success] =
          GetListOfValuesForKeyOrSendError<double, 2>(kSizeKey, map, result);
      success) {
    if (data.has_value()) {
      settings.size = {data->at(0), data->at(1)};
    }
  } else {
    return;
  }
  // Get value for the 'title' key (nullable).
  if (auto const [data, success] =
          GetSingleValueForKeyOrSendError<std::string>(kTitleKey, map, result);
      success) {
    settings.title = data;
  } else {
    return;
  }
  // Get value for the 'state' key (nullable).
  if (auto const [data, success] =
          GetSingleValueForKeyOrSendError<std::string>(kStateKey, map, result);
      success) {
    if (data) {
      settings.state = StringToWindowState(*data);
    }
  } else {
    return;
  }

  if (!controller_->ModifyHostWindow(view_id, settings)) {
    result.Error(kBadArgumentsError, "Can't find window with view ID " +
                                         std::to_string(view_id) + ".");
  }

  result.Success();
}

void WindowingHandler::HandleDestroyWindow(MethodCall<> const& call,
                                           MethodResult<>& result) {
  auto const* const arguments = call.arguments();
  auto const* const map = std::get_if<EncodableMap>(arguments);
  if (!map) {
    result.Error(kBadArgumentsError, "Method call argument is not a map.");
    return;
  }

  // Get value for the 'viewId' key (non-nullable).
  if (auto const [view_id_opt, success] =
          GetSingleValueForKeyOrSendError<int>(kViewIdKey, map, result);
      success) {
    if (!view_id_opt.has_value()) {
      result.Error(kBadArgumentsError, "Value for the '" +
                                           std::string(kViewIdKey) +
                                           "' key must not be null.");
      return;
    }
    if (*view_id_opt < 0) {
      result.Error(kBadArgumentsError,
                   "Value for '" + std::string(kViewIdKey) + "' (" +
                       std::to_string(*view_id_opt) + ") cannot be negative.");
      return;
    }
    if (!controller_->DestroyHostWindow(*view_id_opt)) {
      result.Error(kBadArgumentsError, "Can't find window with view ID " +
                                           std::to_string(*view_id_opt) + ".");
      return;
    }
    result.Success();
  } else {
    return;
  }
}

}  // namespace flutter