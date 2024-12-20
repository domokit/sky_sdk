// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_action_handler.h"

#include "ax_tree_id_registry.h"

namespace ui {

AXActionHandler::AXActionHandler()
    : AXActionHandlerBase(
          AXTreeIDRegistry::GetInstance().GetOrCreateAXTreeID(this)) {}

}  // namespace ui
