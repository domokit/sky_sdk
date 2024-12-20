// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_H_

#include <functional>
#include <ostream>
#include <string>

#include "ax/ax_enums.h"
#include "ax/ax_export.h"
#include "ax/ax_mode.h"
#include "ax/ax_mode_observer.h"
#include "ax_build/build_config.h"
#include "base/macros.h"
#include "gfx/native_widget_types.h"

namespace ui {

class AXPlatformNodeDelegate;

// AXPlatformNode is the abstract base class for an implementation of
// native accessibility APIs on supported platforms (e.g. Windows, Mac OS X).
// An object that wants to be accessible can derive from AXPlatformNodeDelegate
// and then call AXPlatformNode::Create. The delegate implementation should
// own the AXPlatformNode instance (or otherwise manage its lifecycle).
class AX_EXPORT AXPlatformNode {
 public:
  typedef AXPlatformNode* NativeWindowHandlerCallback(gfx::NativeWindow);

  // Create an appropriate platform-specific instance. The delegate owns the
  // AXPlatformNode instance (or manages its lifecycle in some other way).
  static AXPlatformNode* Create(AXPlatformNodeDelegate* delegate);

  // Cast a gfx::NativeViewAccessible to an AXPlatformNode if it is one,
  // or return nullptr if it's not an instance of this class.
  static AXPlatformNode* FromNativeViewAccessible(
      gfx::NativeViewAccessible accessible);

  // Return the AXPlatformNode at the root of the tree for a native window.
  static AXPlatformNode* FromNativeWindow(gfx::NativeWindow native_window);

  // Provide a function that returns the AXPlatformNode at the root of the
  // tree for a native window.
  static void RegisterNativeWindowHandler(
      std::function<NativeWindowHandlerCallback> handler);

  // Register and unregister to receive notifications about AXMode changes
  // for this node.
  static void AddAXModeObserver(AXModeObserver* observer);
  static void RemoveAXModeObserver(AXModeObserver* observer);

  // Convenience method to get the current accessibility mode.
  static AXMode GetAccessibilityMode() { return ax_mode_; }

  // Helper static function to notify all global observers about
  // the addition of an AXMode flag.
  static void NotifyAddAXModeFlags(AXMode mode_flags);

  // Return the focused object in any UI popup overlaying content, or null.
  static gfx::NativeViewAccessible GetPopupFocusOverride();

  // Set the focused object withn any UI popup overlaying content, or null.
  // The focus override is the perceived focus within the popup, and it changes
  // each time a user navigates to a new item within the popup.
  static void SetPopupFocusOverride(gfx::NativeViewAccessible focus_override);

  // Call Destroy rather than deleting this, because the subclass may
  // use reference counting.
  virtual void Destroy();

  // Get the platform-specific accessible object type for this instance.
  // On some platforms this is just a type cast, on others it may be a
  // wrapper object or handle.
  virtual gfx::NativeViewAccessible GetNativeViewAccessible() = 0;

  // Fire a platform-specific notification that an event has occurred on
  // this object.
  virtual void NotifyAccessibilityEvent(ax::mojom::Event event_type) = 0;

#if defined(OS_APPLE)
  // Fire a platform-specific notification to announce |text|.
  virtual void AnnounceText(const std::u16string& text) = 0;
#endif

  // Return this object's delegate.
  virtual AXPlatformNodeDelegate* GetDelegate() const = 0;

  // Return true if this object is equal to or a descendant of |ancestor|.
  virtual bool IsDescendantOf(AXPlatformNode* ancestor) const = 0;

  // Set |this| as the primary web contents for the window.
  void SetIsPrimaryWebContentsForWindow(bool is_primary);
  bool IsPrimaryWebContentsForWindow() const;

  // Return the unique ID.
  int32_t GetUniqueId() const;

  // Creates a string representation of this node's data.
  std::string ToString();

  // Returns a string representation of the subtree of nodes rooted at this
  // node.
  std::string SubtreeToString();

  friend std::ostream& operator<<(std::ostream& stream, AXPlatformNode& node);
  virtual ~AXPlatformNode();

 protected:
  AXPlatformNode();

 private:
  static std::vector<AXModeObserver*> ax_mode_observers_;
  static std::function<NativeWindowHandlerCallback> native_window_handler_;

  static AXMode ax_mode_;

  // This allows UI menu popups like to act as if they are focused in the
  // exposed platform accessibility API, even though actual focus remains in
  // underlying content.
  static gfx::NativeViewAccessible popup_focus_override_;

  bool is_primary_web_contents_for_window_ = false;

  BASE_DISALLOW_COPY_AND_ASSIGN(AXPlatformNode);
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_H_
