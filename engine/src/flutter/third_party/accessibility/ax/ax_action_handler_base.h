// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_ACTION_HANDLER_BASE_H_
#define UI_ACCESSIBILITY_AX_ACTION_HANDLER_BASE_H_

#include "ax_export.h"
#include "ax_tree_id.h"

namespace ui {

struct AXActionData;

// Classes that host an accessibility tree in the browser process that also wish
// to become visible to accessibility clients (e.g. for relaying targets to
// source accessibility trees), can subclass this class. However, unless you
// need to have more control over how |tree_id_| is set, most classes will want
// to inherit from AXActionHandler instead, which manages it automatically.
//
// Subclasses can use |tree_id| when annotating their |AXNodeData| for clients
// to respond with the appropriate target node id.
class AX_EXPORT AXActionHandlerBase {
 public:
  virtual ~AXActionHandlerBase();

  // Handle an action from an accessibility client.
  virtual void PerformAction(const AXActionData& data) = 0;

  // Returns whether this handler expects points in pixels (true) or dips
  // (false) for data passed to |PerformAction|.
  virtual bool RequiresPerformActionPointInPixels() const;

  // A tree id appropriate for annotating events sent to an accessibility
  // client.
  const AXTreeID& ax_tree_id() const { return tree_id_; }

 protected:
  // Initializes the AXActionHandlerBase subclass with ui::AXTreeIDUnknown().
  AXActionHandlerBase();

  // Initializes the AXActionHandlerBase subclass with |ax_tree_id|. It is Ok to
  // pass ui::AXTreeIDUnknown() and then call SetAXTreeID() at a later point.
  explicit AXActionHandlerBase(const AXTreeID& ax_tree_id);

  // Change the AXTreeID.
  void SetAXTreeID(AXTreeID new_ax_tree_id);

 private:
  // Register or unregister this class with |AXTreeIDRegistry|.
  void UpdateActiveState(bool active);

  // Manually set in this base class, but automatically set by instances of the
  // subclass AXActionHandler, which most classes inherit from.
  AXTreeID tree_id_;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_ACTION_HANDLER_BASE_H_
