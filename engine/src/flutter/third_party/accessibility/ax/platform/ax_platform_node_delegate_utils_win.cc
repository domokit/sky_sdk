// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_platform_node_delegate_utils_win.h"

#include "ax/ax_node_data.h"
#include "ax/ax_role_properties.h"
#include "ax/platform/ax_platform_node.h"
#include "ax/platform/ax_platform_node_delegate.h"

namespace ui {

bool IsValuePatternSupported(AXPlatformNodeDelegate* delegate) {
  // https://docs.microsoft.com/en-us/windows/desktop/winauto/uiauto-implementingvalue
  // The Value control pattern is used to support controls that have an
  // intrinsic value not spanning a range and that can be represented as
  // a string.
  //
  // IValueProvider must be implemented by controls such as the color picker
  // selection control [...]

  // https://www.w3.org/TR/html-aam-1.0/
  // The HTML AAM maps "href [a; area]" to UIA Value.Value
  return delegate->GetData().IsRangeValueSupported() ||
         IsReadOnlySupported(delegate->GetData().role) ||
         IsLink(delegate->GetData().role) ||
         delegate->GetData().role == ax::mojom::Role::kColorWell ||
         delegate->IsCellOrHeaderOfARIAGrid();
}

}  // namespace ui
