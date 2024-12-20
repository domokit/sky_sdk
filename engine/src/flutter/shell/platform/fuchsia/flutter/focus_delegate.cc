// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ostream>

#include "focus_delegate.h"

namespace flutter_runner {

void FocusDelegate::WatchLoop(std::function<void(bool)> callback) {
  if (watch_loop_) {
    FML_LOG(ERROR) << "FocusDelegate::WatchLoop() must be called once.";
    return;
  }

  watch_loop_ = [this, /*copy*/ callback](auto focus_state) {
    callback(is_focused_ = focus_state.focused());
    Complete(std::exchange(next_focus_request_, nullptr),
             is_focused_ ? "[true]" : "[false]");
    view_ref_focused_->Watch(/*copy*/ watch_loop_);
  };

  view_ref_focused_->Watch(/*copy*/ watch_loop_);
}

bool FocusDelegate::HandlePlatformMessage(
    rapidjson::Value request,
    fml::RefPtr<flutter::PlatformMessageResponse> response) {
  auto method = request.FindMember("method");
  if (method == request.MemberEnd() || !method->value.IsString()) {
    return false;
  }

  if (method->value == "View.focus.getCurrent") {
    Complete(std::move(response), is_focused_ ? "[true]" : "[false]");
  } else if (method->value == "View.focus.getNext") {
    if (next_focus_request_) {
      FML_LOG(ERROR) << "An outstanding PlatformMessageResponse already exists "
                        "for the next focus state!";
      Complete(std::move(response), "[null]");
    } else {
      next_focus_request_ = std::move(response);
    }
  } else if (method->value == "View.focus.request") {
    auto args_it = request.FindMember("args");
    if (args_it == request.MemberEnd() || !args_it->value.IsObject()) {
      FML_LOG(ERROR) << "No arguments found.";
      return false;
    }
    const auto& args = args_it->value;

    auto view_ref = args.FindMember("viewRef");
    if (!view_ref->value.IsUint64()) {
      FML_LOG(ERROR) << "Argument 'viewRef' is not a uint64";
      return false;
    }

    zx_handle_t handle = view_ref->value.GetUint64();
    zx_handle_t out_handle;
    zx_status_t status =
        zx_handle_duplicate(handle, ZX_RIGHT_SAME_RIGHTS, &out_handle);
    if (status != ZX_OK) {
      FML_LOG(ERROR) << "Argument 'viewRef' is not valid";
      return false;
    }
    auto ref = fuchsia::ui::views::ViewRef({
        .reference = zx::eventpair(out_handle),
    });
    return RequestFocusByViewRef(std::move(ref), std::move(response));

  } else if (method->value == "View.focus.requestById") {
    auto args_it = request.FindMember("args");
    if (args_it == request.MemberEnd() || !args_it->value.IsObject()) {
      FML_LOG(ERROR) << "No arguments found.";
      return false;
    }
    const auto& args = args_it->value;

    auto view_id = args.FindMember("viewId");
    if (!view_id->value.IsUint64()) {
      FML_LOG(ERROR) << "Argument 'viewId' is not a uint64";
      return false;
    }

    auto id = view_id->value.GetUint64();
    if (child_view_view_refs_.count(id) != 1) {
      FML_LOG(ERROR) << "Argument 'viewId' (" << id
                     << ") does not refer to a valid ChildView";
      Complete(std::move(response), "[1]");
      return true;
    }

    return RequestFocusById(id, std::move(response));
  } else {
    return false;
  }
  // All of our methods complete the platform message response.
  return true;
}

void FocusDelegate::OnChildViewViewRef(uint64_t view_id,
                                       fuchsia::ui::views::ViewRef view_ref) {
  FML_CHECK(child_view_view_refs_.count(view_id) == 0);
  child_view_view_refs_[view_id] = std::move(view_ref);
}

void FocusDelegate::OnDisposeChildView(uint64_t view_id) {
  FML_CHECK(child_view_view_refs_.count(view_id) == 1);
  child_view_view_refs_.erase(view_id);
}

void FocusDelegate::Complete(
    fml::RefPtr<flutter::PlatformMessageResponse> response,
    std::string value) {
  if (response) {
    response->Complete(std::make_unique<fml::DataMapping>(
        std::vector<uint8_t>(value.begin(), value.end())));
  }
}

bool FocusDelegate::RequestFocusById(
    uint64_t view_id,
    fml::RefPtr<flutter::PlatformMessageResponse> response) {
  fuchsia::ui::views::ViewRef ref;
  auto status = child_view_view_refs_[view_id].Clone(&ref);
  if (status != ZX_OK) {
    FML_LOG(ERROR) << "Failed to clone ViewRef";
    return false;
  }

  return RequestFocusByViewRef(std::move(ref), std::move(response));
}

bool FocusDelegate::RequestFocusByViewRef(
    fuchsia::ui::views::ViewRef view_ref,
    fml::RefPtr<flutter::PlatformMessageResponse> response) {
  focuser_->RequestFocus(
      std::move(view_ref),
      [this, response = std::move(response)](
          fuchsia::ui::views::Focuser_RequestFocus_Result result) {
        int result_code =
            result.is_err()
                ? static_cast<
                      std::underlying_type_t<fuchsia::ui::views::Error>>(
                      result.err())
                : 0;
        Complete(std::move(response), "[" + std::to_string(result_code) + "]");
      });
  return true;
}

}  // namespace flutter_runner
