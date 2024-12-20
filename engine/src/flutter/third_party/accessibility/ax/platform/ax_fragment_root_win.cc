// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_fragment_root_win.h"

#include <unordered_map>

#include "ax_fragment_root_delegate_win.h"
#include "ax_platform_node_win.h"
#include "base/win/atl_module.h"
#include "uia_registrar_win.h"

namespace ui {

class AXFragmentRootPlatformNodeWin : public AXPlatformNodeWin,
                                      public IItemContainerProvider,
                                      public IRawElementProviderFragmentRoot,
                                      public IRawElementProviderAdviseEvents {
 public:
  BEGIN_COM_MAP(AXFragmentRootPlatformNodeWin)
  COM_INTERFACE_ENTRY(IItemContainerProvider)
  COM_INTERFACE_ENTRY(IRawElementProviderFragmentRoot)
  COM_INTERFACE_ENTRY(IRawElementProviderAdviseEvents)
  COM_INTERFACE_ENTRY_CHAIN(AXPlatformNodeWin)
  END_COM_MAP()

  static AXFragmentRootPlatformNodeWin* Create(
      AXPlatformNodeDelegate* delegate) {
    // Make sure ATL is initialized in this module.
    win::CreateATLModuleIfNeeded();

    CComObject<AXFragmentRootPlatformNodeWin>* instance = nullptr;
    HRESULT hr =
        CComObject<AXFragmentRootPlatformNodeWin>::CreateInstance(&instance);
    BASE_DCHECK(SUCCEEDED(hr));
    instance->Init(delegate);
    instance->AddRef();
    return instance;
  }

  //
  // IItemContainerProvider methods.
  //
  IFACEMETHODIMP FindItemByProperty(
      IRawElementProviderSimple* start_after_element,
      PROPERTYID property_id,
      VARIANT value,
      IRawElementProviderSimple** result) override {
    UIA_VALIDATE_CALL_1_ARG(result);
    *result = nullptr;

    // We currently only support the custom UIA property ID for unique id.
    if (property_id ==
            UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId() &&
        value.vt == VT_BSTR) {
      int32_t ax_unique_id;

      // TODO(gw280): https://github.com/flutter/flutter/issues/78802
      // detect and handle errors
      ax_unique_id = std::stoi(value.bstrVal);

      // In the Windows accessibility platform implementation, id 0 represents
      // self; a positive id represents the immediate descendants; and a
      // negative id represents a unique id that can be mapped to any node.
      if (AXPlatformNodeWin* result_platform_node =
              static_cast<AXPlatformNodeWin*>(GetFromUniqueId(-ax_unique_id))) {
        if (start_after_element) {
          Microsoft::WRL::ComPtr<AXPlatformNodeWin> start_after_platform_node;
          if (!SUCCEEDED(start_after_element->QueryInterface(
                  IID_PPV_ARGS(&start_after_platform_node))))
            return E_INVALIDARG;

          // We want |result| to be nullptr if it comes before or is equal to
          // |start_after_element|.
          if (start_after_platform_node->CompareTo(*result_platform_node) >= 0)
            return S_OK;
        }

        return result_platform_node->QueryInterface(IID_PPV_ARGS(result));
      }
    }

    return E_INVALIDARG;
  }

  //
  // IRawElementProviderSimple methods.
  //

  IFACEMETHODIMP get_HostRawElementProvider(
      IRawElementProviderSimple** host_element_provider) override {
    UIA_VALIDATE_CALL_1_ARG(host_element_provider);

    HWND hwnd = GetDelegate()->GetTargetForNativeAccessibilityEvent();
    return UiaHostProviderFromHwnd(hwnd, host_element_provider);
  }

  IFACEMETHODIMP GetPatternProvider(PATTERNID pattern_id,
                                    IUnknown** result) override {
    UIA_VALIDATE_CALL_1_ARG(result);
    *result = nullptr;

    if (pattern_id == UIA_ItemContainerPatternId) {
      AddRef();
      *result = static_cast<IItemContainerProvider*>(this);
      return S_OK;
    }

    return AXPlatformNodeWin::GetPatternProviderImpl(pattern_id, result);
  }

  IFACEMETHODIMP GetPropertyValue(PROPERTYID property_id,
                                  VARIANT* result) override {
    UIA_VALIDATE_CALL_1_ARG(result);

    switch (property_id) {
      default:
        // UIA has a built-in provider that will expose values for several
        // properties based on the HWND. This information is useful to someone
        // examining the accessibility tree using tools such as Inspect. Return
        // VT_EMPTY for most properties so that we don't override values from
        // the default provider with blank data.
        result->vt = VT_EMPTY;
        break;

      case UIA_IsControlElementPropertyId:
      case UIA_IsContentElementPropertyId:
        // Override IsControlElement and IsContentElement to fine tune which
        // fragment roots appear in the control and content views.
        result->vt = VT_BOOL;
        result->boolVal =
            static_cast<AXFragmentRootWin*>(GetDelegate())->IsControlElement()
                ? VARIANT_TRUE
                : VARIANT_FALSE;
        break;
    }

    return S_OK;
  }

  //
  // IRawElementProviderFragment methods.
  //

  IFACEMETHODIMP get_FragmentRoot(
      IRawElementProviderFragmentRoot** fragment_root) override {
    UIA_VALIDATE_CALL_1_ARG(fragment_root);

    QueryInterface(IID_PPV_ARGS(fragment_root));
    return S_OK;
  }

  //
  // IRawElementProviderFragmentRoot methods.
  //
  IFACEMETHODIMP ElementProviderFromPoint(
      double screen_physical_pixel_x,
      double screen_physical_pixel_y,
      IRawElementProviderFragment** element_provider) override {
    UIA_VALIDATE_CALL_1_ARG(element_provider);

    *element_provider = nullptr;

    gfx::NativeViewAccessible hit_element = nullptr;

    // Descend the tree until we get a non-hit or can't go any further.
    AXPlatformNode* node_to_test = this;
    do {
      gfx::NativeViewAccessible test_result =
          node_to_test->GetDelegate()->HitTestSync(screen_physical_pixel_x,
                                                   screen_physical_pixel_y);
      if (test_result != nullptr && test_result != hit_element) {
        hit_element = test_result;
        node_to_test = AXPlatformNode::FromNativeViewAccessible(test_result);
      } else {
        node_to_test = nullptr;
      }
    } while (node_to_test);

    if (hit_element)
      hit_element->QueryInterface(element_provider);

    return S_OK;
  }

  IFACEMETHODIMP GetFocus(IRawElementProviderFragment** focus) override {
    UIA_VALIDATE_CALL_1_ARG(focus);

    *focus = nullptr;

    gfx::NativeViewAccessible focused_element = nullptr;

    // GetFocus() can return a node at the root of a subtree, for example when
    // transitioning from Views into web content. In such cases we want to
    // continue drilling to retrieve the actual focused element.
    AXPlatformNode* node_to_test = this;
    do {
      gfx::NativeViewAccessible test_result =
          node_to_test->GetDelegate()->GetFocus();
      if (test_result != nullptr && test_result != focused_element) {
        focused_element = test_result;
        node_to_test =
            AXPlatformNode::FromNativeViewAccessible(focused_element);
      } else {
        node_to_test = nullptr;
      }
    } while (node_to_test);

    if (focused_element)
      focused_element->QueryInterface(IID_PPV_ARGS(focus));

    return S_OK;
  }

  //
  // IRawElementProviderAdviseEvents methods.
  //
  IFACEMETHODIMP AdviseEventAdded(EVENTID event_id,
                                  SAFEARRAY* property_ids) override {
    if (event_id == UIA_LiveRegionChangedEventId) {
      live_region_change_listeners_++;

      if (live_region_change_listeners_ == 1) {
        // Fire a LiveRegionChangedEvent for each live-region to tell the
        // newly-attached assistive technology about the regions.
        //
        // Ideally we'd be able to direct these events to only the
        // newly-attached AT, but we don't have that capability, so we only
        // fire events when the *first* AT attaches. (A common scenario will
        // be an attached screen-reader, then a software-keyboard attaches to
        // handle an input field; we don't want the screen-reader to announce
        // that every live-region has changed.) There isn't a perfect solution,
        // but this heuristic seems to work well in practice.
        FireLiveRegionChangeRecursive();
      }
    }
    return S_OK;
  }

  IFACEMETHODIMP AdviseEventRemoved(EVENTID event_id,
                                    SAFEARRAY* property_ids) override {
    if (event_id == UIA_LiveRegionChangedEventId) {
      BASE_DCHECK(live_region_change_listeners_ > 0);
      live_region_change_listeners_--;
    }
    return S_OK;
  }

 private:
  int32_t live_region_change_listeners_ = 0;
};

class AXFragmentRootMapWin {
 public:
  static AXFragmentRootMapWin& GetInstance() {
    static base::NoDestructor<AXFragmentRootMapWin> instance;
    return *instance;
  }

  void AddFragmentRoot(gfx::AcceleratedWidget widget,
                       AXFragmentRootWin* fragment_root) {
    map_[widget] = fragment_root;
  }

  void RemoveFragmentRoot(gfx::AcceleratedWidget widget) { map_.erase(widget); }

  ui::AXFragmentRootWin* GetFragmentRoot(gfx::AcceleratedWidget widget) const {
    const auto& entry = map_.find(widget);
    if (entry != map_.end())
      return entry->second;

    return nullptr;
  }

  ui::AXFragmentRootWin* GetFragmentRootParentOf(
      gfx::NativeViewAccessible accessible) const {
    for (const auto& entry : map_) {
      AXPlatformNodeDelegate* child = entry.second->GetChildNodeDelegate();
      if (child && (child->GetNativeViewAccessible() == accessible))
        return entry.second;
    }
    return nullptr;
  }

 private:
  std::unordered_map<gfx::AcceleratedWidget, AXFragmentRootWin*> map_;
};

AXFragmentRootWin::AXFragmentRootWin(gfx::AcceleratedWidget widget,
                                     AXFragmentRootDelegateWin* delegate)
    : widget_(widget), delegate_(delegate), alert_node_(nullptr) {
  platform_node_ = ui::AXFragmentRootPlatformNodeWin::Create(this);
  AXFragmentRootMapWin::GetInstance().AddFragmentRoot(widget, this);
}

AXFragmentRootWin::~AXFragmentRootWin() {
  AXFragmentRootMapWin::GetInstance().RemoveFragmentRoot(widget_);
  platform_node_->Destroy();
  platform_node_ = nullptr;
}

AXFragmentRootWin* AXFragmentRootWin::GetForAcceleratedWidget(
    gfx::AcceleratedWidget widget) {
  return AXFragmentRootMapWin::GetInstance().GetFragmentRoot(widget);
}

// static
AXFragmentRootWin* AXFragmentRootWin::GetFragmentRootParentOf(
    gfx::NativeViewAccessible accessible) {
  return AXFragmentRootMapWin::GetInstance().GetFragmentRootParentOf(
      accessible);
}

gfx::NativeViewAccessible AXFragmentRootWin::GetNativeViewAccessible() {
  return platform_node_.Get();
}

bool AXFragmentRootWin::IsControlElement() {
  return delegate_->IsAXFragmentRootAControlElement();
}

gfx::NativeViewAccessible AXFragmentRootWin::GetParent() {
  return delegate_->GetParentOfAXFragmentRoot();
}

int AXFragmentRootWin::GetChildCount() const {
  return delegate_->GetChildOfAXFragmentRoot() ? 1 : 0;
}

gfx::NativeViewAccessible AXFragmentRootWin::ChildAtIndex(int index) {
  if (index == 0) {
    return delegate_->GetChildOfAXFragmentRoot();
  } else if (index == 1 && alert_node_) {
    return alert_node_;
  }

  return nullptr;
}

gfx::NativeViewAccessible AXFragmentRootWin::GetNextSibling() {
  int child_index = GetIndexInParentOfChild();
  if (child_index >= 0) {
    AXPlatformNodeDelegate* parent = GetParentNodeDelegate();
    if (parent && child_index < (parent->GetChildCount() - 1))
      return GetParentNodeDelegate()->ChildAtIndex(child_index + 1);
  }

  return nullptr;
}

gfx::NativeViewAccessible AXFragmentRootWin::GetPreviousSibling() {
  int child_index = GetIndexInParentOfChild();
  if (child_index > 0)
    return GetParentNodeDelegate()->ChildAtIndex(child_index - 1);

  return nullptr;
}

gfx::NativeViewAccessible AXFragmentRootWin::HitTestSync(int x, int y) const {
  AXPlatformNodeDelegate* child_delegate = GetChildNodeDelegate();
  if (child_delegate)
    return child_delegate->HitTestSync(x, y);

  return nullptr;
}

gfx::NativeViewAccessible AXFragmentRootWin::GetFocus() {
  AXPlatformNodeDelegate* child_delegate = GetChildNodeDelegate();
  if (child_delegate)
    return child_delegate->GetFocus();

  return nullptr;
}

const ui::AXUniqueId& AXFragmentRootWin::GetUniqueId() const {
  return unique_id_;
}

gfx::AcceleratedWidget
AXFragmentRootWin::GetTargetForNativeAccessibilityEvent() {
  return widget_;
}

AXPlatformNode* AXFragmentRootWin::GetFromTreeIDAndNodeID(
    const ui::AXTreeID& ax_tree_id,
    int32_t node_id) {
  AXPlatformNodeDelegate* child_delegate = GetChildNodeDelegate();
  if (child_delegate)
    return child_delegate->GetFromTreeIDAndNodeID(ax_tree_id, node_id);

  return nullptr;
}

AXPlatformNodeDelegate* AXFragmentRootWin::GetParentNodeDelegate() const {
  gfx::NativeViewAccessible parent = delegate_->GetParentOfAXFragmentRoot();
  if (parent)
    return ui::AXPlatformNode::FromNativeViewAccessible(parent)->GetDelegate();

  return nullptr;
}

AXPlatformNodeDelegate* AXFragmentRootWin::GetChildNodeDelegate() const {
  gfx::NativeViewAccessible child = delegate_->GetChildOfAXFragmentRoot();
  if (child)
    return ui::AXPlatformNode::FromNativeViewAccessible(child)->GetDelegate();

  return nullptr;
}

int AXFragmentRootWin::GetIndexInParentOfChild() const {
  AXPlatformNodeDelegate* parent = GetParentNodeDelegate();

  if (!parent)
    return 0;

  AXPlatformNodeDelegate* child = GetChildNodeDelegate();
  if (child) {
    int child_count = parent->GetChildCount();
    for (int child_index = 0; child_index < child_count; child_index++) {
      if (ui::AXPlatformNode::FromNativeViewAccessible(
              parent->ChildAtIndex(child_index))
              ->GetDelegate() == child)
        return child_index;
    }
  }
  return 0;
}

void AXFragmentRootWin::SetAlertNode(AXPlatformNodeWin* alert_node) {
  alert_node_ = alert_node;
}

gfx::Rect AXFragmentRootWin::GetBoundsRect(AXCoordinateSystem sys,
                                           AXClippingBehavior clip,
                                           AXOffscreenResult* result) const {
  AXPlatformNodeDelegate* child = GetChildNodeDelegate();
  if (!child) {
    return gfx::Rect();
  }
  return child->GetBoundsRect(sys, clip, result);
}

}  // namespace ui
