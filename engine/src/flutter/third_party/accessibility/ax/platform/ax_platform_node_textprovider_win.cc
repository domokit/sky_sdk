// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax/platform/ax_platform_node_textprovider_win.h"

#include <wrl/client.h>

#include "base/win/scoped_safearray.h"

#include "ax/platform/ax_platform_node_textrangeprovider_win.h"

#define UIA_VALIDATE_TEXTPROVIDER_CALL() \
  if (!owner()->GetDelegate())           \
    return UIA_E_ELEMENTNOTAVAILABLE;
#define UIA_VALIDATE_TEXTPROVIDER_CALL_1_ARG(arg) \
  if (!owner()->GetDelegate())                    \
    return UIA_E_ELEMENTNOTAVAILABLE;             \
  if (!arg)                                       \
    return E_INVALIDARG;

namespace ui {

AXPlatformNodeTextProviderWin::AXPlatformNodeTextProviderWin() {}

AXPlatformNodeTextProviderWin::~AXPlatformNodeTextProviderWin() {}

// static
AXPlatformNodeTextProviderWin* AXPlatformNodeTextProviderWin::Create(
    AXPlatformNodeWin* owner) {
  CComObject<AXPlatformNodeTextProviderWin>* text_provider = nullptr;
  if (SUCCEEDED(CComObject<AXPlatformNodeTextProviderWin>::CreateInstance(
          &text_provider))) {
    BASE_DCHECK(text_provider);
    text_provider->owner_ = owner;
    text_provider->AddRef();
    return text_provider;
  }

  return nullptr;
}

// static
void AXPlatformNodeTextProviderWin::CreateIUnknown(AXPlatformNodeWin* owner,
                                                   IUnknown** unknown) {
  Microsoft::WRL::ComPtr<AXPlatformNodeTextProviderWin> text_provider(
      Create(owner));
  if (text_provider)
    *unknown = text_provider.Detach();
}

//
// ITextProvider methods.
//

HRESULT AXPlatformNodeTextProviderWin::GetSelection(SAFEARRAY** selection) {
  UIA_VALIDATE_TEXTPROVIDER_CALL();

  *selection = nullptr;

  AXPlatformNodeDelegate* delegate = owner()->GetDelegate();
  AXTree::Selection unignored_selection = delegate->GetUnignoredSelection();

  AXPlatformNode* anchor_object =
      delegate->GetFromNodeID(unignored_selection.anchor_object_id);
  AXPlatformNode* focus_object =
      delegate->GetFromNodeID(unignored_selection.focus_object_id);

  // anchor_offset corresponds to the selection start index
  // and focus_offset is where the selection ends.
  auto start_offset = unignored_selection.anchor_offset;
  auto end_offset = unignored_selection.focus_offset;

  // If there's no selected object, return success and don't fill the SAFEARRAY.
  if (!anchor_object || !focus_object)
    return S_OK;

  AXNodePosition::AXPositionInstance start =
      anchor_object->GetDelegate()->CreateTextPositionAt(start_offset);
  AXNodePosition::AXPositionInstance end =
      focus_object->GetDelegate()->CreateTextPositionAt(end_offset);

  BASE_DCHECK(!start->IsNullPosition());
  BASE_DCHECK(!end->IsNullPosition());

  // Reverse start and end if the selection goes backwards
  if (*start > *end)
    std::swap(start, end);

  Microsoft::WRL::ComPtr<ITextRangeProvider> text_range_provider =
      AXPlatformNodeTextRangeProviderWin::CreateTextRangeProvider(
          std::move(start), std::move(end));
  if (&text_range_provider == nullptr)
    return E_OUTOFMEMORY;

  // Since we don't support disjoint text ranges, the SAFEARRAY returned
  // will always have one element
  base::win::ScopedSafearray selections_to_return(
      SafeArrayCreateVector(VT_UNKNOWN /* element type */, 0 /* lower bound */,
                            1 /* number of elements */));

  if (!selections_to_return.Get())
    return E_OUTOFMEMORY;

  LONG index = 0;
  HRESULT hr = SafeArrayPutElement(selections_to_return.Get(), &index,
                                   text_range_provider.Get());
  BASE_DCHECK(SUCCEEDED(hr));

  // Since BASE_DCHECK only happens in debug builds, return immediately to
  // ensure that we're not leaking the SAFEARRAY on release builds
  if (FAILED(hr))
    return E_FAIL;

  *selection = selections_to_return.Release();

  return S_OK;
}

HRESULT AXPlatformNodeTextProviderWin::GetVisibleRanges(
    SAFEARRAY** visible_ranges) {
  UIA_VALIDATE_TEXTPROVIDER_CALL();

  const AXPlatformNodeDelegate* delegate = owner()->GetDelegate();

  // Get the Clipped Frame Bounds of the current node, not from the root,
  // so if this node is wrapped with overflow styles it will have the
  // correct bounds
  const gfx::Rect frame_rect = delegate->GetBoundsRect(
      AXCoordinateSystem::kFrame, AXClippingBehavior::kClipped);

  const auto start = delegate->CreateTextPositionAt(0);
  const auto end = start->CreatePositionAtEndOfAnchor();
  BASE_DCHECK(start->GetAnchor() == end->GetAnchor());

  // SAFEARRAYs are not dynamic, so fill the visible ranges in a vector
  // and then transfer to an appropriately-sized SAFEARRAY
  std::vector<Microsoft::WRL::ComPtr<ITextRangeProvider>> ranges;

  auto current_line_start = start->Clone();
  while (!current_line_start->IsNullPosition() && *current_line_start < *end) {
    auto current_line_end = current_line_start->CreateNextLineEndPosition(
        AXBoundaryBehavior::CrossBoundary);
    if (current_line_end->IsNullPosition() || *current_line_end > *end)
      current_line_end = end->Clone();

    gfx::Rect current_rect = delegate->GetInnerTextRangeBoundsRect(
        current_line_start->text_offset(), current_line_end->text_offset(),
        AXCoordinateSystem::kFrame, AXClippingBehavior::kUnclipped);

    if (frame_rect.Contains(current_rect)) {
      Microsoft::WRL::ComPtr<ITextRangeProvider> text_range_provider =
          AXPlatformNodeTextRangeProviderWin::CreateTextRangeProvider(
              current_line_start->Clone(), current_line_end->Clone());

      ranges.emplace_back(text_range_provider);
    }

    current_line_start = current_line_start->CreateNextLineStartPosition(
        AXBoundaryBehavior::CrossBoundary);
  }

  base::win::ScopedSafearray scoped_visible_ranges(
      SafeArrayCreateVector(VT_UNKNOWN /* element type */, 0 /* lower bound */,
                            ranges.size() /* number of elements */));

  if (!scoped_visible_ranges.Get())
    return E_OUTOFMEMORY;

  LONG index = 0;
  for (Microsoft::WRL::ComPtr<ITextRangeProvider>& current_provider : ranges) {
    HRESULT hr = SafeArrayPutElement(scoped_visible_ranges.Get(), &index,
                                     current_provider.Get());
    BASE_DCHECK(SUCCEEDED(hr));

    // Since BASE_DCHECK only happens in debug builds, return immediately to
    // ensure that we're not leaking the SAFEARRAY on release builds
    if (FAILED(hr))
      return E_FAIL;

    ++index;
  }

  *visible_ranges = scoped_visible_ranges.Release();

  return S_OK;
}

HRESULT AXPlatformNodeTextProviderWin::RangeFromChild(
    IRawElementProviderSimple* child,
    ITextRangeProvider** range) {
  UIA_VALIDATE_TEXTPROVIDER_CALL_1_ARG(child);

  *range = nullptr;

  Microsoft::WRL::ComPtr<ui::AXPlatformNodeWin> child_platform_node;
  if (!SUCCEEDED(child->QueryInterface(IID_PPV_ARGS(&child_platform_node))))
    return UIA_E_INVALIDOPERATION;

  if (!owner()->IsDescendant(child_platform_node.Get()))
    return E_INVALIDARG;

  *range = GetRangeFromChild(owner(), child_platform_node.Get());

  return S_OK;
}

HRESULT AXPlatformNodeTextProviderWin::RangeFromPoint(
    UiaPoint uia_point,
    ITextRangeProvider** range) {
  UIA_VALIDATE_TEXTPROVIDER_CALL();
  *range = nullptr;

  gfx::Point point(uia_point.x, uia_point.y);
  // Retrieve the closest accessibility node. No coordinate unit conversion is
  // needed, hit testing input is also in screen coordinates.

  AXPlatformNodeWin* nearest_node =
      static_cast<AXPlatformNodeWin*>(owner()->NearestLeafToPoint(point));
  BASE_DCHECK(nearest_node);
  BASE_DCHECK(nearest_node->IsLeaf());

  AXNodePosition::AXPositionInstance start, end;
  start = nearest_node->GetDelegate()->CreateTextPositionAt(
      nearest_node->NearestTextIndexToPoint(point));
  BASE_DCHECK(!start->IsNullPosition());
  end = start->Clone();

  *range = AXPlatformNodeTextRangeProviderWin::CreateTextRangeProvider(
      std::move(start), std::move(end));
  return S_OK;
}

HRESULT AXPlatformNodeTextProviderWin::get_DocumentRange(
    ITextRangeProvider** range) {
  UIA_VALIDATE_TEXTPROVIDER_CALL();

  // Get range from child, where child is the current node. In other words,
  // getting the text range of the current owner AxPlatformNodeWin node.
  *range = GetRangeFromChild(owner(), owner());

  return S_OK;
}

HRESULT AXPlatformNodeTextProviderWin::get_SupportedTextSelection(
    enum SupportedTextSelection* text_selection) {
  UIA_VALIDATE_TEXTPROVIDER_CALL();

  *text_selection = SupportedTextSelection_Single;
  return S_OK;
}

//
// ITextEditProvider methods.
//

HRESULT AXPlatformNodeTextProviderWin::GetActiveComposition(
    ITextRangeProvider** range) {
  UIA_VALIDATE_TEXTPROVIDER_CALL();

  *range = nullptr;
  return GetTextRangeProviderFromActiveComposition(range);
}

HRESULT AXPlatformNodeTextProviderWin::GetConversionTarget(
    ITextRangeProvider** range) {
  UIA_VALIDATE_TEXTPROVIDER_CALL();

  *range = nullptr;
  return GetTextRangeProviderFromActiveComposition(range);
}

ITextRangeProvider* AXPlatformNodeTextProviderWin::GetRangeFromChild(
    ui::AXPlatformNodeWin* ancestor,
    ui::AXPlatformNodeWin* descendant) {
  BASE_DCHECK(ancestor);
  BASE_DCHECK(descendant);
  BASE_DCHECK(descendant->GetDelegate());
  BASE_DCHECK(ancestor->IsDescendant(descendant));

  // Start and end should be leaf text positions that span the beginning and end
  // of text content within a node. The start position should be the directly
  // first child and the end position should be the deepest last child node.
  AXNodePosition::AXPositionInstance start =
      descendant->GetDelegate()->CreateTextPositionAt(0)->AsLeafTextPosition();

  AXNodePosition::AXPositionInstance end;
  if (descendant->GetChildCount() == 0) {
    end = descendant->GetDelegate()
              ->CreateTextPositionAt(0)
              ->CreatePositionAtEndOfAnchor()
              ->AsLeafTextPosition();
  } else {
    AXPlatformNodeBase* deepest_last_child = descendant->GetLastChild();
    while (deepest_last_child && deepest_last_child->GetChildCount() > 0)
      deepest_last_child = deepest_last_child->GetLastChild();

    end = deepest_last_child->GetDelegate()
              ->CreateTextPositionAt(0)
              ->CreatePositionAtEndOfAnchor()
              ->AsLeafTextPosition();
  }

  return AXPlatformNodeTextRangeProviderWin::CreateTextRangeProvider(
      std::move(start), std::move(end));
}

ITextRangeProvider* AXPlatformNodeTextProviderWin::CreateDegenerateRangeAtStart(
    ui::AXPlatformNodeWin* node) {
  BASE_DCHECK(node);
  BASE_DCHECK(node->GetDelegate());

  // Create a degenerate range positioned at the node's start.
  AXNodePosition::AXPositionInstance start, end;
  start = node->GetDelegate()->CreateTextPositionAt(0)->AsLeafTextPosition();
  end = start->Clone();
  return AXPlatformNodeTextRangeProviderWin::CreateTextRangeProvider(
      std::move(start), std::move(end));
}

ui::AXPlatformNodeWin* AXPlatformNodeTextProviderWin::owner() const {
  return owner_.Get();
}

HRESULT
AXPlatformNodeTextProviderWin::GetTextRangeProviderFromActiveComposition(
    ITextRangeProvider** range) {
  *range = nullptr;
  // We fetch the start and end offset of an active composition only if
  // this object has focus and TSF is in composition mode.
  // The offsets here refer to the character positions in a plain text
  // view of the DOM tree. Ex: if the active composition in an element
  // has "abc" then the range will be (0,3) in both TSF and accessibility
  if ((AXPlatformNode::FromNativeViewAccessible(
           owner()->GetDelegate()->GetFocus()) ==
       static_cast<AXPlatformNode*>(owner())) &&
      owner()->HasActiveComposition()) {
    gfx::Range active_composition_offset =
        owner()->GetActiveCompositionOffsets();
    AXNodePosition::AXPositionInstance start =
        owner()->GetDelegate()->CreateTextPositionAt(
            /*offset*/ active_composition_offset.start());
    AXNodePosition::AXPositionInstance end =
        owner()->GetDelegate()->CreateTextPositionAt(
            /*offset*/ active_composition_offset.end());

    *range = AXPlatformNodeTextRangeProviderWin::CreateTextRangeProvider(
        std::move(start), std::move(end));
  }

  return S_OK;
}

}  // namespace ui
