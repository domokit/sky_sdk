// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_platform_node.h"

#include "ax/ax_node_data.h"
#include "ax_build/build_config.h"
#include "ax_platform_node_delegate.h"

namespace ui {

std::vector<AXModeObserver*> AXPlatformNode::ax_mode_observers_;

std::function<AXPlatformNode::NativeWindowHandlerCallback>
    AXPlatformNode::native_window_handler_;
// static
AXMode AXPlatformNode::ax_mode_;

// static
gfx::NativeViewAccessible AXPlatformNode::popup_focus_override_ = nullptr;

// static
AXPlatformNode* AXPlatformNode::FromNativeWindow(
    gfx::NativeWindow native_window) {
  if (native_window_handler_)
    return native_window_handler_(native_window);
  return nullptr;
}

void AXPlatformNode::RegisterNativeWindowHandler(
    std::function<AXPlatformNode::NativeWindowHandlerCallback> handler) {
  native_window_handler_ = handler;
}

AXPlatformNode::AXPlatformNode() {}

AXPlatformNode::~AXPlatformNode() {}

void AXPlatformNode::Destroy() {}

int32_t AXPlatformNode::GetUniqueId() const {
  BASE_DCHECK(GetDelegate());
  return GetDelegate() ? GetDelegate()->GetUniqueId().Get() : -1;
}

void AXPlatformNode::SetIsPrimaryWebContentsForWindow(bool is_primary) {
  is_primary_web_contents_for_window_ = is_primary;
}

bool AXPlatformNode::IsPrimaryWebContentsForWindow() const {
  return is_primary_web_contents_for_window_;
}

std::string AXPlatformNode::ToString() {
  return GetDelegate() ? GetDelegate()->ToString() : "No delegate";
}

std::string AXPlatformNode::SubtreeToString() {
  return GetDelegate() ? GetDelegate()->SubtreeToString() : "No delegate";
}

std::ostream& operator<<(std::ostream& stream, AXPlatformNode& node) {
  return stream << node.ToString();
}

// static
void AXPlatformNode::AddAXModeObserver(AXModeObserver* observer) {
  ax_mode_observers_.push_back(observer);
}

// static
void AXPlatformNode::RemoveAXModeObserver(AXModeObserver* observer) {
  ax_mode_observers_.erase(std::find(ax_mode_observers_.begin(),
                                     ax_mode_observers_.end(), observer));
}

// static
void AXPlatformNode::NotifyAddAXModeFlags(AXMode mode_flags) {
  // Note: this is only called on Windows.
  AXMode new_ax_mode(ax_mode_);
  new_ax_mode |= mode_flags;

  if (new_ax_mode == ax_mode_)
    return;  // No change.

  ax_mode_ = new_ax_mode;
  for (AXModeObserver* observer : ax_mode_observers_)
    observer->OnAXModeAdded(mode_flags);
}

// static
void AXPlatformNode::SetPopupFocusOverride(
    gfx::NativeViewAccessible popup_focus_override) {
  popup_focus_override_ = popup_focus_override;
}

// static
gfx::NativeViewAccessible AXPlatformNode::GetPopupFocusOverride() {
  return popup_focus_override_;
}

}  // namespace ui
