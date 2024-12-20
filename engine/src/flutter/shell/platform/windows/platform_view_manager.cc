// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/platform_view_manager.h"

#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"

namespace flutter {

namespace {
constexpr char kChannelName[] = "flutter/platform_views";
constexpr char kCreateMethod[] = "create";
constexpr char kFocusMethod[] = "focus";
constexpr char kViewTypeParameter[] = "viewType";
constexpr char kIdParameter[] = "id";
constexpr char kDirectionParameter[] = "direction";
constexpr char kFocusParameter[] = "focus";
}  // namespace

PlatformViewManager::PlatformViewManager(BinaryMessenger* binary_messenger)
    : channel_(std::make_unique<MethodChannel<EncodableValue>>(
          binary_messenger,
          kChannelName,
          &StandardMethodCodec::GetInstance())) {
  channel_->SetMethodCallHandler(
      [this](const MethodCall<EncodableValue>& call,
             std::unique_ptr<MethodResult<EncodableValue>> result) {
        const auto& args = std::get<EncodableMap>(*call.arguments());
        if (call.method_name() == kCreateMethod) {
          const auto& type_itr = args.find(EncodableValue(kViewTypeParameter));
          const auto& id_itr = args.find(EncodableValue(kIdParameter));
          if (type_itr == args.end()) {
            result->Error("AddPlatformView", "Parameter viewType is required");
            return;
          }
          if (id_itr == args.end()) {
            result->Error("AddPlatformView", "Parameter id is required");
            return;
          }
          const auto& type = std::get<std::string>(type_itr->second);
          const auto& id = std::get<std::int32_t>(id_itr->second);
          if (AddPlatformView(id, type)) {
            result->Success();
          } else {
            result->Error("AddPlatformView", "Failed to add platform view");
          }
          return;
        } else if (call.method_name() == kFocusMethod) {
          const auto& id_itr = args.find(EncodableValue(kIdParameter));
          const auto& direction_itr =
              args.find(EncodableValue(kDirectionParameter));
          const auto& focus_itr = args.find(EncodableValue(kFocusParameter));
          if (id_itr == args.end()) {
            result->Error("FocusPlatformView", "Parameter id is required");
            return;
          }
          if (direction_itr == args.end()) {
            result->Error("FocusPlatformView",
                          "Parameter direction is required");
            return;
          }
          if (focus_itr == args.end()) {
            result->Error("FocusPlatformView", "Parameter focus is required");
            return;
          }
          const auto& id = std::get<std::int32_t>(id_itr->second);
          const auto& direction = std::get<std::int32_t>(direction_itr->second);
          const auto& focus = std::get<bool>(focus_itr->second);
          if (FocusPlatformView(
                  id, static_cast<FocusChangeDirection>(direction), focus)) {
            result->Success();
          } else {
            result->Error("FocusPlatformView", "Failed to focus platform view");
          }
          return;
        }
        result->NotImplemented();
      });
}

PlatformViewManager::~PlatformViewManager() {}

}  // namespace flutter
