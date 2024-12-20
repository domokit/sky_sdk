// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "compute_attributes.h"

#include <cstddef>

#include "ax/ax_enums.h"
#include "ax/ax_node_data.h"
#include "ax_platform_node_delegate.h"

namespace ui {
namespace {

std::optional<int32_t> GetCellAttribute(
    const ui::AXPlatformNodeDelegate* delegate,
    ax::mojom::IntAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntAttribute::kAriaCellColumnIndex:
      return delegate->GetTableCellAriaColIndex();
    case ax::mojom::IntAttribute::kAriaCellRowIndex:
      return delegate->GetTableCellAriaRowIndex();
    case ax::mojom::IntAttribute::kTableCellColumnIndex:
      return delegate->GetTableCellColIndex();
    case ax::mojom::IntAttribute::kTableCellRowIndex:
      return delegate->GetTableCellRowIndex();
    case ax::mojom::IntAttribute::kTableCellColumnSpan:
      return delegate->GetTableCellColSpan();
    case ax::mojom::IntAttribute::kTableCellRowSpan:
      return delegate->GetTableCellRowSpan();
    default:
      return std::nullopt;
  }
}

std::optional<int32_t> GetRowAttribute(
    const ui::AXPlatformNodeDelegate* delegate,
    ax::mojom::IntAttribute attribute) {
  if (attribute == ax::mojom::IntAttribute::kTableRowIndex) {
    return delegate->GetTableRowRowIndex();
  }
  return std::nullopt;
}

std::optional<int32_t> GetTableAttribute(
    const ui::AXPlatformNodeDelegate* delegate,
    ax::mojom::IntAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntAttribute::kTableColumnCount:
      return delegate->GetTableColCount();
    case ax::mojom::IntAttribute::kTableRowCount:
      return delegate->GetTableRowCount();
    case ax::mojom::IntAttribute::kAriaColumnCount:
      return delegate->GetTableAriaColCount();
    case ax::mojom::IntAttribute::kAriaRowCount:
      return delegate->GetTableAriaRowCount();
    default:
      return std::nullopt;
  }
}

std::optional<int> GetOrderedSetItemAttribute(
    const ui::AXPlatformNodeDelegate* delegate,
    ax::mojom::IntAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntAttribute::kPosInSet:
      return delegate->GetPosInSet();
    case ax::mojom::IntAttribute::kSetSize:
      return delegate->GetSetSize();
    default:
      return std::nullopt;
  }
}

std::optional<int> GetOrderedSetAttribute(
    const ui::AXPlatformNodeDelegate* delegate,
    ax::mojom::IntAttribute attribute) {
  switch (attribute) {
    case ax::mojom::IntAttribute::kSetSize:
      return delegate->GetSetSize();
    default:
      return std::nullopt;
  }
}

std::optional<int32_t> GetFromData(const ui::AXPlatformNodeDelegate* delegate,
                                   ax::mojom::IntAttribute attribute) {
  int32_t value;
  if (delegate->GetData().GetIntAttribute(attribute, &value)) {
    return value;
  }
  return std::nullopt;
}

}  // namespace

std::optional<int32_t> ComputeAttribute(
    const ui::AXPlatformNodeDelegate* delegate,
    ax::mojom::IntAttribute attribute) {
  std::optional<int32_t> maybe_value = std::nullopt;
  // Table-related nodes.
  if (delegate->IsTableCellOrHeader())
    maybe_value = GetCellAttribute(delegate, attribute);
  else if (delegate->IsTableRow())
    maybe_value = GetRowAttribute(delegate, attribute);
  else if (delegate->IsTable())
    maybe_value = GetTableAttribute(delegate, attribute);
  // Ordered-set-related nodes.
  else if (delegate->IsOrderedSetItem())
    maybe_value = GetOrderedSetItemAttribute(delegate, attribute);
  else if (delegate->IsOrderedSet())
    maybe_value = GetOrderedSetAttribute(delegate, attribute);

  if (!maybe_value.has_value()) {
    return GetFromData(delegate, attribute);
  }
  return maybe_value;
}

}  // namespace ui
