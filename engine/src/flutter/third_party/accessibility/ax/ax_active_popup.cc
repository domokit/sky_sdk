// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_active_popup.h"

#include "base/logging.h"

namespace ui {
// Represents a global storage for the view accessibility for an
// autofill popup. It is a singleton wrapper around the ax unique id of the
// autofill popup. This singleton is used for communicating the live status of
// the autofill popup between web contents and views.
// The assumption here is that only one autofill popup can exist at a time.
static std::optional<int32_t> g_active_popup_ax_unique_id;

std::optional<int32_t> GetActivePopupAxUniqueId() {
  return g_active_popup_ax_unique_id;
}

void SetActivePopupAxUniqueId(std::optional<int32_t> ax_unique_id) {
  // When an instance of autofill popup hides, the caller of popup hide should
  // make sure g_active_popup_ax_unique_id is cleared. The assumption is that
  // there can only be one active autofill popup existing at a time. If on
  // popup showing, we encounter g_active_popup_ax_unique_id is already set,
  // this would indicate two autofill popups are showing at the same time or
  // previous on popup hide call did not clear the variable, so we should fail
  // DCHECK here.
  BASE_DCHECK(!GetActivePopupAxUniqueId());

  g_active_popup_ax_unique_id = ax_unique_id;
}

void ClearActivePopupAxUniqueId() {
  g_active_popup_ax_unique_id = std::nullopt;
}

}  // namespace ui
