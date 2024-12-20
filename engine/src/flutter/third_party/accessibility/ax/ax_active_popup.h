// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_ACTIVE_POPUP_H_
#define UI_ACCESSIBILITY_AX_ACTIVE_POPUP_H_

#include <optional>

#include "ax/ax_export.h"
#include "base/macros.h"

namespace ui {
AX_EXPORT std::optional<int32_t> GetActivePopupAxUniqueId();

AX_EXPORT void SetActivePopupAxUniqueId(std::optional<int32_t> ax_unique_id);

AX_EXPORT void ClearActivePopupAxUniqueId();

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_ACTIVE_POPUP_H_
