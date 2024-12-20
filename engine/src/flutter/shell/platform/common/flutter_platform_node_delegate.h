// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_FLUTTER_PLATFORM_NODE_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_FLUTTER_PLATFORM_NODE_DELEGATE_H_

#include "flutter/fml/mapping.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/third_party/accessibility/ax/ax_event_generator.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node_delegate_base.h"

namespace flutter {

typedef ui::AXNode::AXID AccessibilityNodeId;

//------------------------------------------------------------------------------
/// The platform node delegate to be used in accessibility bridge. This
/// class is responsible for providing native accessibility object with
/// appropriate information, such as accessibility label/value/bounds.
///
/// While most methods have default implementations and are ready to be used
/// as-is, the subclasses must override the GetNativeViewAccessible to return
/// native accessibility objects. To do that, subclasses should create and
/// maintain AXPlatformNode[s] which delegate their accessibility attributes to
/// this class.
///
/// For desktop platforms, subclasses also need to override the GetBoundsRect
/// to apply window-to-screen transform.
///
/// This class transforms bounds assuming the device pixel ratio is 1.0. See
/// the https://github.com/flutter/flutter/issues/74283 for more information.
class FlutterPlatformNodeDelegate : public ui::AXPlatformNodeDelegateBase {
 public:
  //----------------------------------------------------------------------------
  /// The required interface to be able to own the flutter platform node
  /// delegate.
  class OwnerBridge {
   public:
    virtual ~OwnerBridge() = default;

    //---------------------------------------------------------------------------
    /// @brief      Gets the rectangular bounds of the ax node relative to
    ///             global coordinate
    ///
    /// @param[in]  node        The ax node to look up.
    /// @param[in]  offscreen   the bool reference to hold the result whether
    ///                         the ax node is outside of its ancestors' bounds.
    /// @param[in]  clip_bounds whether to clip the result if the ax node cannot
    ///                         be fully contained in its ancestors' bounds.
    virtual gfx::RectF RelativeToGlobalBounds(const ui::AXNode* node,
                                              bool& offscreen,
                                              bool clip_bounds) = 0;

   protected:
    friend class FlutterPlatformNodeDelegate;

    //---------------------------------------------------------------------------
    /// @brief      Dispatch accessibility action back to the Flutter framework.
    ///             These actions are generated in the native accessibility
    ///             system when users interact with the assistive technologies.
    ///             For example, a
    ///             FlutterSemanticsAction::kFlutterSemanticsActionTap is
    ///             fired when user click or touch the screen.
    ///
    /// @param[in]  target              The semantics node id of the action
    ///                                 target.
    /// @param[in]  action              The generated flutter semantics action.
    /// @param[in]  data                Additional data associated with the
    ///                                 action.
    virtual void DispatchAccessibilityAction(AccessibilityNodeId target,
                                             FlutterSemanticsAction action,
                                             fml::MallocMapping data) = 0;

    //---------------------------------------------------------------------------
    /// @brief      Get the native accessibility node with the given id.
    ///
    /// @param[in]  id           The id of the native accessibility node you
    ///                          want to retrieve.
    virtual gfx::NativeViewAccessible GetNativeAccessibleFromId(
        AccessibilityNodeId id) = 0;

    //---------------------------------------------------------------------------
    /// @brief      Get the last id of the node that received accessibility
    ///             focus.
    virtual AccessibilityNodeId GetLastFocusedId() = 0;

    //---------------------------------------------------------------------------
    /// @brief      Update the id of the node that is currently foucsed by the
    ///             native accessibility system.
    ///
    /// @param[in]  node_id     The id of the focused node.
    virtual void SetLastFocusedId(AccessibilityNodeId node_id) = 0;
  };

  FlutterPlatformNodeDelegate();

  // |ui::AXPlatformNodeDelegateBase|
  virtual ~FlutterPlatformNodeDelegate() override;

  // |ui::AXPlatformNodeDelegateBase|
  const ui::AXUniqueId& GetUniqueId() const override { return unique_id_; }

  // |ui::AXPlatformNodeDelegateBase|
  const ui::AXNodeData& GetData() const override;

  // |ui::AXPlatformNodeDelegateBase|
  bool AccessibilityPerformAction(const ui::AXActionData& data) override;

  // |ui::AXPlatformNodeDelegateBase|
  gfx::NativeViewAccessible GetParent() override;

  // |ui::AXPlatformNodeDelegateBase|
  gfx::NativeViewAccessible GetFocus() override;

  // |ui::AXPlatformNodeDelegateBase|
  int GetChildCount() const override;

  // |ui::AXPlatformNodeDelegateBase|
  gfx::NativeViewAccessible ChildAtIndex(int index) override;

  // |ui::AXPlatformNodeDelegateBase|
  gfx::Rect GetBoundsRect(
      const ui::AXCoordinateSystem coordinate_system,
      const ui::AXClippingBehavior clipping_behavior,
      ui::AXOffscreenResult* offscreen_result) const override;

  // |ui::AXPlatformNodeDelegateBase|
  gfx::NativeViewAccessible GetLowestPlatformAncestor() const override;

  // |ui::AXPlatformNodeDelegateBase|
  ui::AXNodePosition::AXPositionInstance CreateTextPositionAt(
      int offset) const override;

  //------------------------------------------------------------------------------
  /// @brief      Called only once, immediately after construction. The
  ///             constructor doesn't take any arguments because in the Windows
  ///             subclass we use a special function to construct a COM object.
  ///             Subclasses must call super.
  virtual void Init(std::weak_ptr<OwnerBridge> bridge, ui::AXNode* node);

  //------------------------------------------------------------------------------
  // @brief       Called when node was updated. Subclasses can override this
  //              to update platform nodes.
  virtual void NodeDataChanged(const ui::AXNodeData& old_node_data,
                               const ui::AXNodeData& new_node_data) {}

  //------------------------------------------------------------------------------
  /// @brief      Gets the underlying ax node for this platform node delegate.
  ui::AXNode* GetAXNode() const;

  //------------------------------------------------------------------------------
  /// @brief      Gets the owner of this platform node delegate. This is useful
  ///             when you want to get the information about surrounding nodes
  ///             of this platform node delegate, e.g. the global rect of this
  ///             platform node delegate. This pointer is only safe in the
  ///             platform thread.
  std::weak_ptr<OwnerBridge> GetOwnerBridge() const;

  // Get the platform node represented by this delegate.
  virtual ui::AXPlatformNode* GetPlatformNode() const;

  // |ui::AXPlatformNodeDelegateBase|
  virtual ui::AXPlatformNode* GetFromNodeID(int32_t id) override;

  // |ui::AXPlatformNodeDelegateBase|
  virtual ui::AXPlatformNode* GetFromTreeIDAndNodeID(
      const ui::AXTreeID& tree_id,
      int32_t node_id) override;

  // |ui::AXPlatformNodeDelegateBase|
  virtual const ui::AXTree::Selection GetUnignoredSelection() const override;

 private:
  ui::AXNode* ax_node_;
  std::weak_ptr<OwnerBridge> bridge_;
  ui::AXUniqueId unique_id_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_FLUTTER_PLATFORM_NODE_DELEGATE_H_
