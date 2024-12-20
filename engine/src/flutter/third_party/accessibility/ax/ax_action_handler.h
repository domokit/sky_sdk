// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_ACTION_HANDLER_H_
#define UI_ACCESSIBILITY_AX_ACTION_HANDLER_H_

#include "ax_action_handler_base.h"
#include "ax_export.h"

namespace ui {

// The class you normally want to inherit from other classes when you want to
// make them visible to accessibility clients, since it automatically registers
// a valid AXTreeID with the AXTreeIDRegistry when constructing the instance.
//
// If you need more control over how the AXTreeID associated to this class is
// set, please inherit directly from AXActionHandlerBase instead.
class AX_EXPORT AXActionHandler : public AXActionHandlerBase {
 protected:
  AXActionHandler();
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_ACTION_HANDLER_H_
