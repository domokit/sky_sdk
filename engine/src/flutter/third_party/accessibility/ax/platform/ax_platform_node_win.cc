// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_platform_node_win.h"

#include <wrl/client.h>
#include <wrl/implements.h>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>

#include "ax/ax_action_data.h"
#include "ax/ax_active_popup.h"
#include "ax/ax_enum_util.h"
#include "ax/ax_mode_observer.h"
#include "ax/ax_node_data.h"
#include "ax/ax_node_position.h"
#include "ax/ax_role_properties.h"
#include "ax/ax_tree_data.h"
#include "base/win/enum_variant.h"
#include "base/win/scoped_bstr.h"
#include "base/win/scoped_safearray.h"
#include "base/win/scoped_variant.h"
#include "base/win/variant_vector.h"

#include "ax_fragment_root_win.h"
#include "ax_platform_node_delegate.h"
#include "ax_platform_node_delegate_utils_win.h"
#include "ax_platform_node_textprovider_win.h"
#include "shellscalingapi.h"
#include "uia_registrar_win.h"

#include "base/logging.h"
#include "base/win/atl_module.h"
#include "base/win/display.h"
#include "flutter/fml/platform/win/wstring_conversion.h"
#include "gfx/geometry/rect_conversions.h"

// From ax.constants.mojom
namespace ax {
namespace mojom {
const int32_t kUnknownAriaColumnOrRowCount = -1;
}
}  // namespace ax

//
// Macros to use at the top of any AXPlatformNodeWin function that implements
// a non-UIA COM interface. Because COM objects are reference counted and
// clients are completely untrusted, it's important to always first check that
// our object is still valid, and then check that all pointer arguments are not
// NULL.
//
#define COM_OBJECT_VALIDATE() \
  if (!GetDelegate())         \
    return E_FAIL;
#define COM_OBJECT_VALIDATE_1_ARG(arg) \
  if (!GetDelegate())                  \
    return E_FAIL;                     \
  if (!arg)                            \
    return E_INVALIDARG;               \
  *arg = {};
#define COM_OBJECT_VALIDATE_2_ARGS(arg1, arg2) \
  if (!GetDelegate())                          \
    return E_FAIL;                             \
  if (!arg1)                                   \
    return E_INVALIDARG;                       \
  *arg1 = {};                                  \
  if (!arg2)                                   \
    return E_INVALIDARG;                       \
  *arg2 = {};
#define COM_OBJECT_VALIDATE_3_ARGS(arg1, arg2, arg3) \
  if (!GetDelegate())                                \
    return E_FAIL;                                   \
  if (!arg1)                                         \
    return E_INVALIDARG;                             \
  *arg1 = {};                                        \
  if (!arg2)                                         \
    return E_INVALIDARG;                             \
  *arg2 = {};                                        \
  if (!arg3)                                         \
    return E_INVALIDARG;                             \
  *arg3 = {};
#define COM_OBJECT_VALIDATE_4_ARGS(arg1, arg2, arg3, arg4) \
  if (!GetDelegate())                                      \
    return E_FAIL;                                         \
  if (!arg1)                                               \
    return E_INVALIDARG;                                   \
  *arg1 = {};                                              \
  if (!arg2)                                               \
    return E_INVALIDARG;                                   \
  *arg2 = {};                                              \
  if (!arg3)                                               \
    return E_INVALIDARG;                                   \
  *arg3 = {};                                              \
  if (!arg4)                                               \
    return E_INVALIDARG;                                   \
  *arg4 = {};
#define COM_OBJECT_VALIDATE_5_ARGS(arg1, arg2, arg3, arg4, arg5) \
  if (!GetDelegate())                                            \
    return E_FAIL;                                               \
  if (!arg1)                                                     \
    return E_INVALIDARG;                                         \
  *arg1 = {};                                                    \
  if (!arg2)                                                     \
    return E_INVALIDARG;                                         \
  *arg2 = {};                                                    \
  if (!arg3)                                                     \
    return E_INVALIDARG;                                         \
  *arg3 = {};                                                    \
  if (!arg4)                                                     \
    return E_INVALIDARG;                                         \
  *arg4 = {};                                                    \
  if (!arg5)                                                     \
    return E_INVALIDARG;                                         \
  *arg5 = {};
#define COM_OBJECT_VALIDATE_VAR_ID_AND_GET_TARGET(var_id, target) \
  if (!GetDelegate())                                             \
    return E_FAIL;                                                \
  target = GetTargetFromChildID(var_id);                          \
  if (!target)                                                    \
    return E_INVALIDARG;                                          \
  if (!target->GetDelegate())                                     \
    return E_INVALIDARG;
#define COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, arg, target) \
  if (!GetDelegate())                                                        \
    return E_FAIL;                                                           \
  if (!arg)                                                                  \
    return E_INVALIDARG;                                                     \
  *arg = {};                                                                 \
  target = GetTargetFromChildID(var_id);                                     \
  if (!target)                                                               \
    return E_INVALIDARG;                                                     \
  if (!target->GetDelegate())                                                \
    return E_INVALIDARG;
#define COM_OBJECT_VALIDATE_VAR_ID_2_ARGS_AND_GET_TARGET(var_id, arg1, arg2, \
                                                         target)             \
  if (!GetDelegate())                                                        \
    return E_FAIL;                                                           \
  if (!arg1)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg1 = {};                                                                \
  if (!arg2)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg2 = {};                                                                \
  target = GetTargetFromChildID(var_id);                                     \
  if (!target)                                                               \
    return E_INVALIDARG;                                                     \
  if (!target->GetDelegate())                                                \
    return E_INVALIDARG;
#define COM_OBJECT_VALIDATE_VAR_ID_3_ARGS_AND_GET_TARGET(var_id, arg1, arg2, \
                                                         arg3, target)       \
  if (!GetDelegate())                                                        \
    return E_FAIL;                                                           \
  if (!arg1)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg1 = {};                                                                \
  if (!arg2)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg2 = {};                                                                \
  if (!arg3)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg3 = {};                                                                \
  target = GetTargetFromChildID(var_id);                                     \
  if (!target)                                                               \
    return E_INVALIDARG;                                                     \
  if (!target->GetDelegate())                                                \
    return E_INVALIDARG;
#define COM_OBJECT_VALIDATE_VAR_ID_4_ARGS_AND_GET_TARGET(var_id, arg1, arg2, \
                                                         arg3, arg4, target) \
  if (!GetDelegate())                                                        \
    return E_FAIL;                                                           \
  if (!arg1)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg1 = {};                                                                \
  if (!arg2)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg2 = {};                                                                \
  if (!arg3)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg3 = {};                                                                \
  if (!arg4)                                                                 \
    return E_INVALIDARG;                                                     \
  *arg4 = {};                                                                \
  target = GetTargetFromChildID(var_id);                                     \
  if (!target)                                                               \
    return E_INVALIDARG;                                                     \
  if (!target->GetDelegate())                                                \
    return E_INVALIDARG;

namespace ui {

namespace {

typedef std::unordered_set<AXPlatformNodeWin*> AXPlatformNodeWinSet;

// Sets the multiplier by which large changes to a RangeValueProvider are
// greater than small changes.
constexpr int kLargeChangeScaleFactor = 10;

// The amount to scroll when UI Automation asks to scroll by a small increment.
// Value is in device independent pixels and is the same used by Blink when
// cursor keys are used to scroll a webpage.
constexpr float kSmallScrollIncrement = 40.0f;

// The filename of the DLL to load for UIA.
constexpr wchar_t kUIADLLFilename[] = L"uiautomationcore.dll";

void AppendTextToString(std::u16string extra_text, std::u16string* string) {
  if (extra_text.empty())
    return;

  if (string->empty()) {
    *string = extra_text;
    return;
  }

  *string += std::u16string(u". ") + extra_text;
}

// Helper function to GetPatternProviderFactoryMethod that, given a node,
// will return a pattern interface through result based on the provided type T.
template <typename T>
void PatternProvider(AXPlatformNodeWin* node, IUnknown** result) {
  node->AddRef();
  *result = static_cast<T*>(node);
}

}  // namespace

void AXPlatformNodeWin::AddAttributeToList(const char* name,
                                           const char* value,
                                           PlatformAttributeList* attributes) {
  std::string str_value = value;
  SanitizeStringAttribute(str_value, &str_value);
  attributes->push_back(base::UTF8ToUTF16(name) + u":" +
                        base::UTF8ToUTF16(str_value));
}

// There is no easy way to decouple |kScreenReader| and |kHTML| accessibility
// modes when Windows screen readers are used. For example, certain roles use
// the HTML tag name. Input fields require their type attribute to be exposed.
const uint32_t kScreenReaderAndHTMLAccessibilityModes =
    AXMode::kScreenReader | AXMode::kHTML;

//
// AXPlatformNode::Create
//

// static
AXPlatformNode* AXPlatformNode::Create(AXPlatformNodeDelegate* delegate) {
  // Make sure ATL is initialized in this module.
  win::CreateATLModuleIfNeeded();

  CComObject<AXPlatformNodeWin>* instance = nullptr;
  HRESULT hr = CComObject<AXPlatformNodeWin>::CreateInstance(&instance);
  BASE_DCHECK(SUCCEEDED(hr));
  instance->Init(delegate);
  instance->AddRef();
  return instance;
}

// static
AXPlatformNode* AXPlatformNode::FromNativeViewAccessible(
    gfx::NativeViewAccessible accessible) {
  if (!accessible)
    return nullptr;
  Microsoft::WRL::ComPtr<AXPlatformNodeWin> ax_platform_node;
  accessible->QueryInterface(IID_PPV_ARGS(&ax_platform_node));
  return ax_platform_node.Get();
}

//
// AXPlatformNodeWin
//

AXPlatformNodeWin::AXPlatformNodeWin() {}

AXPlatformNodeWin::~AXPlatformNodeWin() {}

void AXPlatformNodeWin::Init(AXPlatformNodeDelegate* delegate) {
  AXPlatformNodeBase::Init(delegate);
}

// Static
void AXPlatformNodeWin::SanitizeStringAttributeForUIAAriaProperty(
    const std::u16string& input,
    std::u16string* output) {
  BASE_DCHECK(output);
  // According to the UIA Spec, these characters need to be escaped with a
  // backslash in an AriaProperties string: backslash, equals and semicolon.
  // Note that backslash must be replaced first.
  base::ReplaceChars(input, u"\\", u"\\\\", output);
  base::ReplaceChars(*output, u"=", u"\\=", output);
  base::ReplaceChars(*output, u";", u"\\;", output);
}

void AXPlatformNodeWin::StringAttributeToUIAAriaProperty(
    std::vector<std::u16string>& properties,
    ax::mojom::StringAttribute attribute,
    const char* uia_aria_property) {
  std::u16string value;
  if (GetString16Attribute(attribute, &value)) {
    SanitizeStringAttributeForUIAAriaProperty(value, &value);
    properties.push_back(base::ASCIIToUTF16(uia_aria_property) + u"=" + value);
  }
}

void AXPlatformNodeWin::BoolAttributeToUIAAriaProperty(
    std::vector<std::u16string>& properties,
    ax::mojom::BoolAttribute attribute,
    const char* uia_aria_property) {
  bool value;
  if (GetBoolAttribute(attribute, &value)) {
    properties.push_back((base::ASCIIToUTF16(uia_aria_property) + u"=") +
                         (value ? u"true" : u"false"));
  }
}

void AXPlatformNodeWin::IntAttributeToUIAAriaProperty(
    std::vector<std::u16string>& properties,
    ax::mojom::IntAttribute attribute,
    const char* uia_aria_property) {
  int value;
  if (GetIntAttribute(attribute, &value)) {
    properties.push_back(base::ASCIIToUTF16(uia_aria_property) + u"=" +
                         base::NumberToString16(value));
  }
}

void AXPlatformNodeWin::FloatAttributeToUIAAriaProperty(
    std::vector<std::u16string>& properties,
    ax::mojom::FloatAttribute attribute,
    const char* uia_aria_property) {
  float value;
  if (GetFloatAttribute(attribute, &value)) {
    properties.push_back(base::ASCIIToUTF16(uia_aria_property) + u"=" +
                         base::NumberToString16(value));
  }
}

void AXPlatformNodeWin::StateToUIAAriaProperty(
    std::vector<std::u16string>& properties,
    ax::mojom::State state,
    const char* uia_aria_property) {
  const AXNodeData& data = GetData();
  bool value = data.HasState(state);
  properties.push_back((base::ASCIIToUTF16(uia_aria_property) + u"=") +
                       (value ? u"true" : u"false"));
}

void AXPlatformNodeWin::HtmlAttributeToUIAAriaProperty(
    std::vector<std::u16string>& properties,
    const char* html_attribute_name,
    const char* uia_aria_property) {
  std::u16string html_attribute_value;
  if (GetData().GetHtmlAttribute(html_attribute_name, &html_attribute_value)) {
    SanitizeStringAttributeForUIAAriaProperty(html_attribute_value,
                                              &html_attribute_value);
    properties.push_back(base::ASCIIToUTF16(uia_aria_property) + u"=" +
                         html_attribute_value);
  }
}

std::vector<AXPlatformNodeWin*>
AXPlatformNodeWin::CreatePlatformNodeVectorFromRelationIdVector(
    std::vector<int32_t>& relation_id_list) {
  std::vector<AXPlatformNodeWin*> platform_node_list;

  for (int32_t id : relation_id_list) {
    AXPlatformNode* platform_node = GetDelegate()->GetFromNodeID(id);
    if (IsValidUiaRelationTarget(platform_node)) {
      platform_node_list.push_back(
          static_cast<AXPlatformNodeWin*>(platform_node));
    }
  }

  return platform_node_list;
}

SAFEARRAY* AXPlatformNodeWin::CreateUIAElementsSafeArray(
    std::vector<AXPlatformNodeWin*>& platform_node_list) {
  if (platform_node_list.empty())
    return nullptr;

  SAFEARRAY* uia_array =
      SafeArrayCreateVector(VT_UNKNOWN, 0, platform_node_list.size());
  LONG i = 0;

  for (AXPlatformNodeWin* platform_node : platform_node_list) {
    // All incoming ids should already be validated to have a valid relation
    // targets so that this function does not need to re-check before allocating
    // the SAFEARRAY.
    BASE_DCHECK(IsValidUiaRelationTarget(platform_node));
    SafeArrayPutElement(uia_array, &i,
                        static_cast<IRawElementProviderSimple*>(platform_node));
    ++i;
  }

  return uia_array;
}

SAFEARRAY* AXPlatformNodeWin::CreateUIAControllerForArray() {
  std::vector<int32_t> relation_id_list =
      GetIntListAttribute(ax::mojom::IntListAttribute::kControlsIds);

  std::vector<AXPlatformNodeWin*> platform_node_list =
      CreatePlatformNodeVectorFromRelationIdVector(relation_id_list);

  if (GetActivePopupAxUniqueId() != std::nullopt) {
    AXPlatformNodeWin* view_popup_node_win = static_cast<AXPlatformNodeWin*>(
        GetFromUniqueId(GetActivePopupAxUniqueId().value()));

    if (IsValidUiaRelationTarget(view_popup_node_win))
      platform_node_list.push_back(view_popup_node_win);
  }

  return CreateUIAElementsSafeArray(platform_node_list);
}

SAFEARRAY* AXPlatformNodeWin::CreateUIAElementsArrayForRelation(
    const ax::mojom::IntListAttribute& attribute) {
  std::vector<int32_t> relation_id_list = GetIntListAttribute(attribute);

  std::vector<AXPlatformNodeWin*> platform_node_list =
      CreatePlatformNodeVectorFromRelationIdVector(relation_id_list);

  return CreateUIAElementsSafeArray(platform_node_list);
}

SAFEARRAY* AXPlatformNodeWin::CreateUIAElementsArrayForReverseRelation(
    const ax::mojom::IntListAttribute& attribute) {
  std::set<AXPlatformNode*> reverse_relations =
      GetDelegate()->GetReverseRelations(attribute);

  std::vector<int32_t> id_list;
  std::transform(
      reverse_relations.cbegin(), reverse_relations.cend(),
      std::back_inserter(id_list), [](AXPlatformNode* platform_node) {
        return static_cast<AXPlatformNodeWin*>(platform_node)->GetData().id;
      });

  std::vector<AXPlatformNodeWin*> platform_node_list =
      CreatePlatformNodeVectorFromRelationIdVector(id_list);

  return CreateUIAElementsSafeArray(platform_node_list);
}

SAFEARRAY* AXPlatformNodeWin::CreateClickablePointArray() {
  SAFEARRAY* clickable_point_array = SafeArrayCreateVector(VT_R8, 0, 2);
  gfx::Point center = GetDelegate()
                          ->GetBoundsRect(AXCoordinateSystem::kScreenDIPs,
                                          AXClippingBehavior::kUnclipped)
                          .CenterPoint();

  double* double_array;
  SafeArrayAccessData(clickable_point_array,
                      reinterpret_cast<void**>(&double_array));
  double_array[0] = center.x();
  double_array[1] = center.y();
  SafeArrayUnaccessData(clickable_point_array);

  return clickable_point_array;
}

gfx::Vector2d AXPlatformNodeWin::CalculateUIAScrollPoint(
    const ScrollAmount horizontal_amount,
    const ScrollAmount vertical_amount) const {
  if (!GetDelegate() || !IsScrollable())
    return {};

  const gfx::Rect bounds = GetDelegate()->GetBoundsRect(
      AXCoordinateSystem::kScreenDIPs, AXClippingBehavior::kClipped);
  const int large_horizontal_change = bounds.width();
  const int large_vertical_change = bounds.height();

  const HWND hwnd = GetDelegate()->GetTargetForNativeAccessibilityEvent();
  BASE_DCHECK(hwnd);
  const float scale_factor = base::win::GetScaleFactorForHWND(hwnd);
  const int small_change =
      base::ClampRound(kSmallScrollIncrement * scale_factor);

  const int x_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMin);
  const int x_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMax);
  const int y_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMin);
  const int y_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMax);

  int x = GetIntAttribute(ax::mojom::IntAttribute::kScrollX);
  int y = GetIntAttribute(ax::mojom::IntAttribute::kScrollY);

  switch (horizontal_amount) {
    case ScrollAmount_LargeDecrement:
      x -= large_horizontal_change;
      break;
    case ScrollAmount_LargeIncrement:
      x += large_horizontal_change;
      break;
    case ScrollAmount_NoAmount:
      break;
    case ScrollAmount_SmallDecrement:
      x -= small_change;
      break;
    case ScrollAmount_SmallIncrement:
      x += small_change;
      break;
  }
  x = std::min(x, x_max);
  x = std::max(x, x_min);

  switch (vertical_amount) {
    case ScrollAmount_LargeDecrement:
      y -= large_vertical_change;
      break;
    case ScrollAmount_LargeIncrement:
      y += large_vertical_change;
      break;
    case ScrollAmount_NoAmount:
      break;
    case ScrollAmount_SmallDecrement:
      y -= small_change;
      break;
    case ScrollAmount_SmallIncrement:
      y += small_change;
      break;
  }
  y = std::min(y, y_max);
  y = std::max(y, y_min);

  return {x, y};
}

//
// AXPlatformNodeBase implementation.
//

void AXPlatformNodeWin::Dispose() {
  Release();
}

void AXPlatformNodeWin::Destroy() {
  RemoveAlertTarget();

  // This will end up calling Dispose() which may result in deleting this object
  // if there are no more outstanding references.
  AXPlatformNodeBase::Destroy();
}

//
// AXPlatformNode implementation.
//

gfx::NativeViewAccessible AXPlatformNodeWin::GetNativeViewAccessible() {
  return this;
}

void AXPlatformNodeWin::NotifyAccessibilityEvent(ax::mojom::Event event_type) {
  AXPlatformNodeBase::NotifyAccessibilityEvent(event_type);
  // Menu items fire selection events but Windows screen readers work reliably
  // with focus events. Remap here.
  if (event_type == ax::mojom::Event::kSelection) {
    // A menu item could have something other than a role of
    // |ROLE_SYSTEM_MENUITEM|. Zoom modification controls for example have a
    // role of button.
    auto* parent =
        static_cast<AXPlatformNodeWin*>(FromNativeViewAccessible(GetParent()));
    int role = MSAARole();
    if (role == ROLE_SYSTEM_MENUITEM) {
      event_type = ax::mojom::Event::kFocus;
    } else if (role == ROLE_SYSTEM_LISTITEM) {
      if (AXPlatformNodeBase* container = GetSelectionContainer()) {
        const AXNodeData& data = container->GetData();
        if (data.role == ax::mojom::Role::kListBox &&
            !data.HasState(ax::mojom::State::kMultiselectable) &&
            GetDelegate()->GetFocus() == GetNativeViewAccessible()) {
          event_type = ax::mojom::Event::kFocus;
        }
      }
    } else if (parent) {
      int parent_role = parent->MSAARole();
      if (parent_role == ROLE_SYSTEM_MENUPOPUP ||
          parent_role == ROLE_SYSTEM_LIST) {
        event_type = ax::mojom::Event::kFocus;
      }
    }
  }

  if (event_type == ax::mojom::Event::kValueChanged) {
    // For the IAccessibleText interface to work on non-web content nodes, we
    // need to update the nodes' hypertext
    // when the value changes. Otherwise, for web and PDF content, this is
    // handled by "BrowserAccessibilityComWin".
    if (!GetDelegate()->IsWebContent())
      UpdateComputedHypertext();
  }

  if (std::optional<DWORD> native_event = MojoEventToMSAAEvent(event_type)) {
    HWND hwnd = GetDelegate()->GetTargetForNativeAccessibilityEvent();
    if (!hwnd)
      return;

    ::NotifyWinEvent((*native_event), hwnd, OBJID_CLIENT, -GetUniqueId());
  }

  if (std::optional<PROPERTYID> uia_property =
          MojoEventToUIAProperty(event_type)) {
    // For this event, we're not concerned with the old value.
    base::win::ScopedVariant old_value;
    ::VariantInit(old_value.Receive());
    base::win::ScopedVariant new_value;
    ::VariantInit(new_value.Receive());
    GetPropertyValueImpl((*uia_property), new_value.Receive());
    ::UiaRaiseAutomationPropertyChangedEvent(this, (*uia_property), old_value,
                                             new_value);
  }

  if (std::optional<EVENTID> uia_event = MojoEventToUIAEvent(event_type))
    ::UiaRaiseAutomationEvent(this, (*uia_event));

  // Keep track of objects that are a target of an alert event.
  if (event_type == ax::mojom::Event::kAlert)
    AddAlertTarget();
}

bool AXPlatformNodeWin::HasActiveComposition() const {
  return active_composition_range_.end() > active_composition_range_.start();
}

gfx::Range AXPlatformNodeWin::GetActiveCompositionOffsets() const {
  return active_composition_range_;
}

void AXPlatformNodeWin::OnActiveComposition(
    const gfx::Range& range,
    const std::u16string& active_composition_text,
    bool is_composition_committed) {
  // Cache the composition range that will be used when
  // GetActiveComposition and GetConversionTarget is called in
  // AXPlatformNodeTextProviderWin
  active_composition_range_ = range;
  // Fire the UiaTextEditTextChangedEvent
  FireUiaTextEditTextChangedEvent(range, active_composition_text,
                                  is_composition_committed);
}

void AXPlatformNodeWin::FireUiaTextEditTextChangedEvent(
    const gfx::Range& range,
    const std::u16string& active_composition_text,
    bool is_composition_committed) {
  // This API is only supported from Win8.1 onwards
  // Check if the function pointer is valid or not
  using UiaRaiseTextEditTextChangedEventFunction = HRESULT(WINAPI*)(
      IRawElementProviderSimple*, TextEditChangeType, SAFEARRAY*);
  UiaRaiseTextEditTextChangedEventFunction text_edit_text_changed_func =
      reinterpret_cast<UiaRaiseTextEditTextChangedEventFunction>(
          ::GetProcAddress(GetModuleHandleW(kUIADLLFilename),
                           "UiaRaiseTextEditTextChangedEvent"));
  if (!text_edit_text_changed_func) {
    return;
  }

  TextEditChangeType text_edit_change_type =
      is_composition_committed ? TextEditChangeType_CompositionFinalized
                               : TextEditChangeType_Composition;

  // Composition has been finalized by TSF
  base::win::ScopedBstr composition_text(
      (wchar_t*)active_composition_text.data());
  base::win::ScopedSafearray changed_data(
      ::SafeArrayCreateVector(VT_BSTR /* element type */, 0 /* lower bound */,
                              1 /* number of elements */));
  if (!changed_data.Get()) {
    return;
  }

  LONG index = 0;
  HRESULT hr =
      SafeArrayPutElement(changed_data.Get(), &index, composition_text.Get());

  if (FAILED(hr)) {
    return;
  } else {
    // Fire the UiaRaiseTextEditTextChangedEvent
    text_edit_text_changed_func(this, text_edit_change_type,
                                changed_data.Release());
  }
}

bool AXPlatformNodeWin::IsValidUiaRelationTarget(
    AXPlatformNode* ax_platform_node) {
  if (!ax_platform_node)
    return false;
  if (!ax_platform_node->GetDelegate())
    return false;

  // This is needed for get_FragmentRoot.
  if (!ax_platform_node->GetDelegate()->GetTargetForNativeAccessibilityEvent())
    return false;

  return true;
}

//
// IAccessible implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::accHitTest(LONG screen_physical_pixel_x,
                                             LONG screen_physical_pixel_y,
                                             VARIANT* child) {
  COM_OBJECT_VALIDATE_1_ARG(child);

  gfx::Point point(screen_physical_pixel_x, screen_physical_pixel_y);
  if (!GetDelegate()
           ->GetBoundsRect(AXCoordinateSystem::kScreenPhysicalPixels,
                           AXClippingBehavior::kClipped)
           .Contains(point)) {
    // Return S_FALSE and VT_EMPTY when outside the object's boundaries.
    child->vt = VT_EMPTY;
    return S_FALSE;
  }

  AXPlatformNode* current_result = this;
  while (true) {
    gfx::NativeViewAccessible hit_child =
        current_result->GetDelegate()->HitTestSync(screen_physical_pixel_x,
                                                   screen_physical_pixel_y);
    if (!hit_child) {
      child->vt = VT_EMPTY;
      return S_FALSE;
    }

    AXPlatformNode* hit_child_node =
        AXPlatformNode::FromNativeViewAccessible(hit_child);
    if (!hit_child_node)
      break;

    // If we get the same node, we're done.
    if (hit_child_node == current_result)
      break;

    // Prevent cycles / loops.
    //
    // This is a workaround for a bug where a hit test in web content might
    // return a node that's not a strict descendant. To catch that case
    // without disallowing other valid cases of hit testing, add the
    // following check:
    //
    // If the hit child comes from the same HWND, but it's not a descendant,
    // just ignore the result and stick with the current result. Note that
    // GetTargetForNativeAccessibilityEvent returns a node's owning HWND.
    //
    // Ideally this shouldn't happen - see http://crbug.com/1061323
    bool is_descendant = hit_child_node->IsDescendantOf(current_result);
    bool is_same_hwnd =
        hit_child_node->GetDelegate()->GetTargetForNativeAccessibilityEvent() ==
        current_result->GetDelegate()->GetTargetForNativeAccessibilityEvent();
    if (!is_descendant && is_same_hwnd)
      break;

    // Continue to check recursively. That's because HitTestSync may have
    // returned the best result within a particular accessibility tree,
    // but we might need to recurse further in a tree of a different type
    // (for example, from Views to Web).
    current_result = hit_child_node;
  }

  if (current_result == this) {
    // This object is the best match, so return CHILDID_SELF. It's tempting to
    // simplify the logic and use VT_DISPATCH everywhere, but the Windows
    // call AccessibleObjectFromPoint will keep calling accHitTest until some
    // object returns CHILDID_SELF.
    child->vt = VT_I4;
    child->lVal = CHILDID_SELF;
    return S_OK;
  }

  child->vt = VT_DISPATCH;
  child->pdispVal = static_cast<AXPlatformNodeWin*>(current_result);
  // Always increment ref when returning a reference to a COM object.
  child->pdispVal->AddRef();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::accDoDefaultAction(VARIANT var_id) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_AND_GET_TARGET(var_id, target);
  AXActionData data;
  data.action = ax::mojom::Action::kDoDefault;

  if (target->GetDelegate()->AccessibilityPerformAction(data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::accLocation(LONG* physical_pixel_left,
                                              LONG* physical_pixel_top,
                                              LONG* width,
                                              LONG* height,
                                              VARIANT var_id) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_4_ARGS_AND_GET_TARGET(
      var_id, physical_pixel_left, physical_pixel_top, width, height, target);

  gfx::Rect bounds = target->GetDelegate()->GetBoundsRect(
      AXCoordinateSystem::kScreenPhysicalPixels,
      AXClippingBehavior::kUnclipped);
  *physical_pixel_left = bounds.x();
  *physical_pixel_top = bounds.y();
  *width = bounds.width();
  *height = bounds.height();

  if (bounds.IsEmpty())
    return S_FALSE;

  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::accNavigate(LONG nav_dir,
                                              VARIANT start,
                                              VARIANT* end) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(start, end, target);
  end->vt = VT_EMPTY;
  if ((nav_dir == NAVDIR_FIRSTCHILD || nav_dir == NAVDIR_LASTCHILD) &&
      V_VT(&start) == VT_I4 && V_I4(&start) != CHILDID_SELF) {
    // MSAA states that navigating to first/last child can only be from self.
    return E_INVALIDARG;
  }

  IAccessible* result = nullptr;
  switch (nav_dir) {
    case NAVDIR_FIRSTCHILD:
      if (GetDelegate()->GetChildCount() > 0)
        result = GetDelegate()->GetFirstChild();
      break;

    case NAVDIR_LASTCHILD:
      if (GetDelegate()->GetChildCount() > 0)
        result = GetDelegate()->GetLastChild();
      break;

    case NAVDIR_NEXT: {
      AXPlatformNodeBase* next = target->GetNextSibling();
      if (next)
        result = next->GetNativeViewAccessible();
      break;
    }

    case NAVDIR_PREVIOUS: {
      AXPlatformNodeBase* previous = target->GetPreviousSibling();
      if (previous)
        result = previous->GetNativeViewAccessible();
      break;
    }

    case NAVDIR_DOWN: {
      // This direction is not implemented except in tables.
      if (!GetTableRow() || !GetTableRowSpan() || !GetTableColumn())
        return E_NOTIMPL;

      AXPlatformNodeBase* next = target->GetTableCell(
          *GetTableRow() + *GetTableRowSpan(), *GetTableColumn());
      if (!next)
        return S_OK;

      result = next->GetNativeViewAccessible();
      break;
    }

    case NAVDIR_UP: {
      // This direction is not implemented except in tables.
      if (!GetTableRow() || !GetTableColumn())
        return E_NOTIMPL;

      AXPlatformNodeBase* next =
          target->GetTableCell(*GetTableRow() - 1, *GetTableColumn());
      if (!next)
        return S_OK;

      result = next->GetNativeViewAccessible();
      break;
    }

    case NAVDIR_LEFT: {
      // This direction is not implemented except in tables.
      if (!GetTableRow() || !GetTableColumn())
        return E_NOTIMPL;

      AXPlatformNodeBase* next =
          target->GetTableCell(*GetTableRow(), *GetTableColumn() - 1);
      if (!next)
        return S_OK;

      result = next->GetNativeViewAccessible();
      break;
    }

    case NAVDIR_RIGHT: {
      // This direction is not implemented except in tables.

      if (!GetTableRow() || !GetTableColumn() || !GetTableColumnSpan())
        return E_NOTIMPL;

      AXPlatformNodeBase* next = target->GetTableCell(
          *GetTableRow(), *GetTableColumn() + *GetTableColumnSpan());
      if (!next)
        return S_OK;

      result = next->GetNativeViewAccessible();
      break;
    }
  }

  if (!result)
    return S_FALSE;

  end->vt = VT_DISPATCH;
  end->pdispVal = result;
  // Always increment ref when returning a reference to a COM object.
  end->pdispVal->AddRef();

  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accChild(VARIANT var_child,
                                               IDispatch** disp_child) {
  *disp_child = nullptr;
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_AND_GET_TARGET(var_child, target);

  *disp_child = target;
  (*disp_child)->AddRef();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accChildCount(LONG* child_count) {
  COM_OBJECT_VALIDATE_1_ARG(child_count);
  *child_count = GetDelegate()->GetChildCount();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accDefaultAction(VARIANT var_id,
                                                       BSTR* def_action) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, def_action, target);
  AXPlatformNode::NotifyAddAXModeFlags(kScreenReaderAndHTMLAccessibilityModes);

  int action;
  if (!target->GetIntAttribute(ax::mojom::IntAttribute::kDefaultActionVerb,
                               &action)) {
    *def_action = nullptr;
    return S_FALSE;
  }

  // TODO(gw280): https://github.com/flutter/flutter/issues/78799
  // Use localized strings
  std::u16string action_verb = base::UTF8ToUTF16(
      ui::ToString(static_cast<ax::mojom::DefaultActionVerb>(action)));
  if (action_verb.empty()) {
    *def_action = nullptr;
    return S_FALSE;
  }

  *def_action = ::SysAllocString(fml::Utf16ToWideString(action_verb).c_str());
  BASE_DCHECK(def_action);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accDescription(VARIANT var_id,
                                                     BSTR* desc) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, desc, target);

  return target->GetStringAttributeAsBstr(
      ax::mojom::StringAttribute::kDescription, desc);
}

IFACEMETHODIMP AXPlatformNodeWin::get_accFocus(VARIANT* focus_child) {
  COM_OBJECT_VALIDATE_1_ARG(focus_child);
  gfx::NativeViewAccessible focus_accessible = GetDelegate()->GetFocus();
  if (focus_accessible == this) {
    focus_child->vt = VT_I4;
    focus_child->lVal = CHILDID_SELF;
  } else if (focus_accessible) {
    Microsoft::WRL::ComPtr<IDispatch> focus_idispatch;
    if (FAILED(
            focus_accessible->QueryInterface(IID_PPV_ARGS(&focus_idispatch)))) {
      focus_child->vt = VT_EMPTY;
      return E_FAIL;
    }

    focus_child->vt = VT_DISPATCH;
    focus_child->pdispVal = focus_idispatch.Detach();
  } else {
    focus_child->vt = VT_EMPTY;
  }

  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accKeyboardShortcut(VARIANT var_id,
                                                          BSTR* acc_key) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, acc_key, target);

  return target->GetStringAttributeAsBstr(
      ax::mojom::StringAttribute::kKeyShortcuts, acc_key);
}

IFACEMETHODIMP AXPlatformNodeWin::get_accName(VARIANT var_id, BSTR* name_bstr) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, name_bstr, target);

  if (!IsNameExposed())
    return S_FALSE;

  bool has_name = target->HasStringAttribute(ax::mojom::StringAttribute::kName);
  std::u16string name = target->GetNameAsString16();

  // Simply appends the tooltip, if any, to the end of the MSAA name.
  const std::u16string tooltip =
      target->GetString16Attribute(ax::mojom::StringAttribute::kTooltip);
  if (!tooltip.empty()) {
    AppendTextToString(tooltip, &name);
  }

  auto status = GetData().GetImageAnnotationStatus();
  switch (status) {
    case ax::mojom::ImageAnnotationStatus::kNone:
    case ax::mojom::ImageAnnotationStatus::kWillNotAnnotateDueToScheme:
    case ax::mojom::ImageAnnotationStatus::kIneligibleForAnnotation:
    case ax::mojom::ImageAnnotationStatus::kSilentlyEligibleForAnnotation:
      break;

    case ax::mojom::ImageAnnotationStatus::kEligibleForAnnotation:
    case ax::mojom::ImageAnnotationStatus::kAnnotationPending:
    case ax::mojom::ImageAnnotationStatus::kAnnotationEmpty:
    case ax::mojom::ImageAnnotationStatus::kAnnotationAdult:
    case ax::mojom::ImageAnnotationStatus::kAnnotationProcessFailed:
      AppendTextToString(
          GetDelegate()->GetLocalizedStringForImageAnnotationStatus(status),
          &name);
      break;

    case ax::mojom::ImageAnnotationStatus::kAnnotationSucceeded:
      AppendTextToString(
          GetString16Attribute(ax::mojom::StringAttribute::kImageAnnotation),
          &name);
      break;
  }

  if (name.empty() && !has_name)
    return S_FALSE;

  *name_bstr = ::SysAllocString(fml::Utf16ToWideString(name).c_str());
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accParent(IDispatch** disp_parent) {
  COM_OBJECT_VALIDATE_1_ARG(disp_parent);
  *disp_parent = GetParent();
  if (*disp_parent) {
    (*disp_parent)->AddRef();
    return S_OK;
  }
  IRawElementProviderFragmentRoot* root;
  if (SUCCEEDED(get_FragmentRoot(&root))) {
    gfx::NativeViewAccessible parent;
    if (SUCCEEDED(root->QueryInterface(IID_PPV_ARGS(&parent)))) {
      if (parent && parent != GetNativeViewAccessible()) {
        *disp_parent = parent;
        parent->AddRef();
        return S_OK;
      }
    }
  }
  return S_FALSE;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accRole(VARIANT var_id, VARIANT* role) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, role, target);

  role->vt = VT_I4;
  role->lVal = target->MSAARole();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accState(VARIANT var_id, VARIANT* state) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, state, target);
  state->vt = VT_I4;
  state->lVal = target->MSAAState();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accHelp(VARIANT var_id, BSTR* help) {
  COM_OBJECT_VALIDATE_1_ARG(help);
  return S_FALSE;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accValue(VARIANT var_id, BSTR* value) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_1_ARG_AND_GET_TARGET(var_id, value, target);
  *value = GetValueAttributeAsBstr(target);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::put_accValue(VARIANT var_id, BSTR new_value) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_AND_GET_TARGET(var_id, target);
  if (!new_value)
    return E_INVALIDARG;

  std::u16string new_value_utf16((char16_t*)new_value);
  AXActionData data;
  data.action = ax::mojom::Action::kSetValue;
  data.value = base::UTF16ToUTF8(new_value_utf16);
  if (target->GetDelegate()->AccessibilityPerformAction(data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accSelection(VARIANT* selected) {
  COM_OBJECT_VALIDATE_1_ARG(selected);
  std::vector<Microsoft::WRL::ComPtr<IDispatch>> selected_nodes;
  for (int i = 0; i < GetDelegate()->GetChildCount(); ++i) {
    auto* node = static_cast<AXPlatformNodeWin*>(
        FromNativeViewAccessible(GetDelegate()->ChildAtIndex(i)));
    if (node &&
        node->GetData().GetBoolAttribute(ax::mojom::BoolAttribute::kSelected)) {
      Microsoft::WRL::ComPtr<IDispatch> node_idispatch;
      if (SUCCEEDED(node->QueryInterface(IID_PPV_ARGS(&node_idispatch))))
        selected_nodes.push_back(node_idispatch);
    }
  }

  if (selected_nodes.empty()) {
    selected->vt = VT_EMPTY;
    return S_OK;
  }

  if (selected_nodes.size() == 1) {
    selected->vt = VT_DISPATCH;
    selected->pdispVal = selected_nodes[0].Detach();
    return S_OK;
  }

  // Multiple items are selected.
  LONG selected_count = static_cast<LONG>(selected_nodes.size());
  Microsoft::WRL::ComPtr<base::win::EnumVariant> enum_variant =
      Microsoft::WRL::Make<base::win::EnumVariant>(selected_count);
  for (LONG i = 0; i < selected_count; ++i) {
    enum_variant->ItemAt(i)->vt = VT_DISPATCH;
    enum_variant->ItemAt(i)->pdispVal = selected_nodes[i].Detach();
  }
  selected->vt = VT_UNKNOWN;
  return enum_variant.CopyTo(IID_PPV_ARGS(&V_UNKNOWN(selected)));
}

IFACEMETHODIMP AXPlatformNodeWin::accSelect(LONG flagsSelect, VARIANT var_id) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_AND_GET_TARGET(var_id, target);

  if (flagsSelect & SELFLAG_TAKEFOCUS) {
    AXActionData action_data;
    action_data.action = ax::mojom::Action::kFocus;
    target->GetDelegate()->AccessibilityPerformAction(action_data);
    return S_OK;
  }

  return S_FALSE;
}

IFACEMETHODIMP AXPlatformNodeWin::get_accHelpTopic(BSTR* help_file,
                                                   VARIANT var_id,
                                                   LONG* topic_id) {
  AXPlatformNodeWin* target;
  COM_OBJECT_VALIDATE_VAR_ID_2_ARGS_AND_GET_TARGET(var_id, help_file, topic_id,
                                                   target);
  if (help_file) {
    *help_file = nullptr;
  }
  if (topic_id) {
    *topic_id = static_cast<LONG>(-1);
  }
  return E_NOTIMPL;
}

IFACEMETHODIMP AXPlatformNodeWin::put_accName(VARIANT var_id, BSTR put_name) {
  // TODO(dougt): We may want to collect an API histogram here.
  // Deprecated.
  return E_NOTIMPL;
}

//
// IExpandCollapseProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::Collapse() {
  UIA_VALIDATE_CALL();
  if (GetData().GetRestriction() == ax::mojom::Restriction::kDisabled)
    return UIA_E_ELEMENTNOTAVAILABLE;

  if (GetData().HasState(ax::mojom::State::kCollapsed))
    return UIA_E_INVALIDOPERATION;

  AXActionData action_data;
  action_data.action = ax::mojom::Action::kDoDefault;
  if (GetDelegate()->AccessibilityPerformAction(action_data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::Expand() {
  UIA_VALIDATE_CALL();
  if (GetData().GetRestriction() == ax::mojom::Restriction::kDisabled)
    return UIA_E_ELEMENTNOTAVAILABLE;

  if (GetData().HasState(ax::mojom::State::kExpanded))
    return UIA_E_INVALIDOPERATION;

  AXActionData action_data;
  action_data.action = ax::mojom::Action::kDoDefault;
  if (GetDelegate()->AccessibilityPerformAction(action_data))
    return S_OK;
  return E_FAIL;
}

ExpandCollapseState AXPlatformNodeWin::ComputeExpandCollapseState() const {
  const AXNodeData& data = GetData();

  // Since a menu button implies there is a popup and it is either expanded or
  // collapsed, and it should not support ExpandCollapseState_LeafNode.
  // According to the UIA spec, ExpandCollapseState_LeafNode indicates that the
  // element neither expands nor collapses.
  if (data.IsMenuButton()) {
    if (data.IsButtonPressed())
      return ExpandCollapseState_Expanded;
    return ExpandCollapseState_Collapsed;
  }

  if (data.HasState(ax::mojom::State::kExpanded)) {
    return ExpandCollapseState_Expanded;
  } else if (data.HasState(ax::mojom::State::kCollapsed)) {
    return ExpandCollapseState_Collapsed;
  } else {
    return ExpandCollapseState_LeafNode;
  }
}

IFACEMETHODIMP AXPlatformNodeWin::get_ExpandCollapseState(
    ExpandCollapseState* result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  *result = ComputeExpandCollapseState();

  return S_OK;
}

//
// IGridItemProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::get_Column(int* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  std::optional<int> column = GetTableColumn();
  if (!column)
    return E_FAIL;
  *result = *column;
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_ColumnSpan(int* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  std::optional<int> column_span = GetTableColumnSpan();
  if (!column_span)
    return E_FAIL;
  *result = *column_span;
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_ContainingGrid(
    IRawElementProviderSimple** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  AXPlatformNodeBase* table = GetTable();
  if (!table)
    return E_FAIL;

  auto* node_win = static_cast<AXPlatformNodeWin*>(table);
  node_win->AddRef();
  *result = static_cast<IRawElementProviderSimple*>(node_win);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_Row(int* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  std::optional<int> row = GetTableRow();
  if (!row)
    return E_FAIL;
  *result = *row;
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_RowSpan(int* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  std::optional<int> row_span = GetTableRowSpan();
  if (!row_span)
    return E_FAIL;
  *result = *row_span;
  return S_OK;
}

//
// IGridProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::GetItem(int row,
                                          int column,
                                          IRawElementProviderSimple** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  AXPlatformNodeBase* cell = GetTableCell(row, column);
  if (!cell)
    return E_INVALIDARG;

  auto* node_win = static_cast<AXPlatformNodeWin*>(cell);
  node_win->AddRef();
  *result = static_cast<IRawElementProviderSimple*>(node_win);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_RowCount(int* result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  std::optional<int> row_count = GetTableAriaRowCount();
  if (!row_count)
    row_count = GetTableRowCount();

  if (!row_count || *row_count == ax::mojom::kUnknownAriaColumnOrRowCount)
    return E_UNEXPECTED;
  *result = *row_count;
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_ColumnCount(int* result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  std::optional<int> column_count = GetTableAriaColumnCount();
  if (!column_count)
    column_count = GetTableColumnCount();

  if (!column_count ||
      *column_count == ax::mojom::kUnknownAriaColumnOrRowCount) {
    return E_UNEXPECTED;
  }
  *result = *column_count;
  return S_OK;
}

//
// IInvokeProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::Invoke() {
  UIA_VALIDATE_CALL();

  if (GetData().GetRestriction() == ax::mojom::Restriction::kDisabled)
    return UIA_E_ELEMENTNOTENABLED;

  AXActionData action_data;
  action_data.action = ax::mojom::Action::kDoDefault;
  GetDelegate()->AccessibilityPerformAction(action_data);

  return S_OK;
}

//
// IScrollItemProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::ScrollIntoView() {
  UIA_VALIDATE_CALL();
  gfx::Rect r = gfx::ToEnclosingRect(GetData().relative_bounds.bounds);
  r -= r.OffsetFromOrigin();

  AXActionData action_data;
  action_data.target_node_id = GetData().id;
  action_data.target_rect = r;
  action_data.horizontal_scroll_alignment =
      ax::mojom::ScrollAlignment::kScrollAlignmentCenter;
  action_data.vertical_scroll_alignment =
      ax::mojom::ScrollAlignment::kScrollAlignmentCenter;
  action_data.scroll_behavior =
      ax::mojom::ScrollBehavior::kDoNotScrollIfVisible;
  if (GetDelegate()->AccessibilityPerformAction(action_data))
    return S_OK;
  return E_FAIL;
}

//
// IScrollProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::Scroll(ScrollAmount horizontal_amount,
                                         ScrollAmount vertical_amount) {
  UIA_VALIDATE_CALL();
  if (!IsScrollable())
    return E_FAIL;

  AXActionData action_data;
  action_data.target_node_id = GetData().id;
  action_data.action = ax::mojom::Action::kSetScrollOffset;
  action_data.target_point = gfx::PointAtOffsetFromOrigin(
      CalculateUIAScrollPoint(horizontal_amount, vertical_amount));
  if (GetDelegate()->AccessibilityPerformAction(action_data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::SetScrollPercent(double horizontal_percent,
                                                   double vertical_percent) {
  UIA_VALIDATE_CALL();
  if (!IsScrollable())
    return E_FAIL;

  const double x_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMin);
  const double x_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMax);
  const double y_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMin);
  const double y_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMax);
  const int x =
      base::ClampRound(horizontal_percent / 100.0 * (x_max - x_min) + x_min);
  const int y =
      base::ClampRound(vertical_percent / 100.0 * (y_max - y_min) + y_min);
  const gfx::Point scroll_to(x, y);

  AXActionData action_data;
  action_data.target_node_id = GetData().id;
  action_data.action = ax::mojom::Action::kSetScrollOffset;
  action_data.target_point = scroll_to;
  if (GetDelegate()->AccessibilityPerformAction(action_data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_HorizontallyScrollable(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = IsHorizontallyScrollable();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_HorizontalScrollPercent(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = GetHorizontalScrollPercent();
  return S_OK;
}

// Horizontal size of the viewable region as a percentage of the total content
// area.
IFACEMETHODIMP AXPlatformNodeWin::get_HorizontalViewSize(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  if (!IsHorizontallyScrollable()) {
    *result = 100.;
    return S_OK;
  }

  gfx::RectF clipped_bounds(GetDelegate()->GetBoundsRect(
      AXCoordinateSystem::kScreenDIPs, AXClippingBehavior::kClipped));
  float x_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMin);
  float x_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMax);
  float total_width = clipped_bounds.width() + x_max - x_min;
  BASE_DCHECK(clipped_bounds.width() <= total_width);
  *result = 100.0 * clipped_bounds.width() / total_width;
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_VerticallyScrollable(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = IsVerticallyScrollable();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_VerticalScrollPercent(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = GetVerticalScrollPercent();
  return S_OK;
}

// Vertical size of the viewable region as a percentage of the total content
// area.
IFACEMETHODIMP AXPlatformNodeWin::get_VerticalViewSize(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  if (!IsVerticallyScrollable()) {
    *result = 100.0;
    return S_OK;
  }

  gfx::RectF clipped_bounds(GetDelegate()->GetBoundsRect(
      AXCoordinateSystem::kScreenDIPs, AXClippingBehavior::kClipped));
  float y_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMin);
  float y_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMax);
  float total_height = clipped_bounds.height() + y_max - y_min;
  BASE_DCHECK(clipped_bounds.height() <= total_height);
  *result = 100.0 * clipped_bounds.height() / total_height;
  return S_OK;
}

//
// ISelectionItemProvider implementation.
//

HRESULT AXPlatformNodeWin::ISelectionItemProviderSetSelected(
    bool selected) const {
  UIA_VALIDATE_CALL();
  if (GetData().GetRestriction() == ax::mojom::Restriction::kDisabled)
    return UIA_E_ELEMENTNOTENABLED;

  // The platform implements selection follows focus for single-selection
  // container elements. Focus action can change a node's accessibility selected
  // state, but does not cause the actual control to be selected.
  // https://www.w3.org/TR/wai-aria-practices-1.1/#kbd_selection_follows_focus
  // https://www.w3.org/TR/core-aam-1.2/#mapping_events_selection
  //
  // We don't want to perform |Action::kDoDefault| for an ax node that has
  // |kSelected=true| and |kSelectedFromFocus=false|, because perform
  // |Action::kDoDefault| may cause the control to be unselected. However, if an
  // ax node is selected due to focus, i.e. |kSelectedFromFocus=true|, we need
  // to perform |Action::kDoDefault| on the ax node, since focus action only
  // changes an ax node's accessibility selected state to |kSelected=true| and
  // no |Action::kDoDefault| was performed on that node yet. So we need to
  // perform |Action::kDoDefault| on the ax node to cause its associated control
  // to be selected.
  if (selected == ISelectionItemProviderIsSelected() &&
      !GetBoolAttribute(ax::mojom::BoolAttribute::kSelectedFromFocus))
    return S_OK;

  AXActionData data;
  data.action = ax::mojom::Action::kDoDefault;
  if (GetDelegate()->AccessibilityPerformAction(data))
    return S_OK;
  return UIA_E_INVALIDOPERATION;
}

bool AXPlatformNodeWin::ISelectionItemProviderIsSelected() const {
  // https://www.w3.org/TR/core-aam-1.1/#mapping_state-property_table
  // SelectionItem.IsSelected is set according to the True or False value of
  // aria-checked for 'radio' and 'menuitemradio' roles.
  if (GetData().role == ax::mojom::Role::kRadioButton ||
      GetData().role == ax::mojom::Role::kMenuItemRadio)
    return GetData().GetCheckedState() == ax::mojom::CheckedState::kTrue;

  // https://www.w3.org/TR/wai-aria-1.1/#aria-selected
  // SelectionItem.IsSelected is set according to the True or False value of
  // aria-selected.
  return GetBoolAttribute(ax::mojom::BoolAttribute::kSelected);
}

IFACEMETHODIMP AXPlatformNodeWin::AddToSelection() {
  return ISelectionItemProviderSetSelected(true);
}

IFACEMETHODIMP AXPlatformNodeWin::RemoveFromSelection() {
  return ISelectionItemProviderSetSelected(false);
}

IFACEMETHODIMP AXPlatformNodeWin::Select() {
  return ISelectionItemProviderSetSelected(true);
}

IFACEMETHODIMP AXPlatformNodeWin::get_IsSelected(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = ISelectionItemProviderIsSelected();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_SelectionContainer(
    IRawElementProviderSimple** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  auto* node_win = static_cast<AXPlatformNodeWin*>(GetSelectionContainer());
  if (!node_win)
    return E_FAIL;

  node_win->AddRef();
  *result = static_cast<IRawElementProviderSimple*>(node_win);
  return S_OK;
}

//
// ISelectionProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::GetSelection(SAFEARRAY** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  std::vector<AXPlatformNodeBase*> selected_children;
  int max_items = GetMaxSelectableItems();
  if (max_items)
    GetSelectedItems(max_items, &selected_children);

  LONG selected_children_count = selected_children.size();
  *result = SafeArrayCreateVector(VT_UNKNOWN, 0, selected_children_count);
  if (!*result)
    return E_OUTOFMEMORY;

  for (LONG i = 0; i < selected_children_count; ++i) {
    AXPlatformNodeWin* children =
        static_cast<AXPlatformNodeWin*>(selected_children[i]);
    HRESULT hr = SafeArrayPutElement(
        *result, &i, static_cast<IRawElementProviderSimple*>(children));
    if (FAILED(hr)) {
      SafeArrayDestroy(*result);
      *result = nullptr;
      return hr;
    }
  }
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_CanSelectMultiple(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = GetData().HasState(ax::mojom::State::kMultiselectable);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_IsSelectionRequired(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = GetData().HasState(ax::mojom::State::kRequired);
  return S_OK;
}

//
// ITableItemProvider methods.
//

IFACEMETHODIMP AXPlatformNodeWin::GetColumnHeaderItems(SAFEARRAY** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  std::optional<int> column = GetTableColumn();
  if (!column)
    return E_FAIL;

  std::vector<int32_t> column_header_ids =
      GetDelegate()->GetColHeaderNodeIds(*column);

  std::vector<AXPlatformNodeWin*> platform_node_list =
      CreatePlatformNodeVectorFromRelationIdVector(column_header_ids);

  *result = CreateUIAElementsSafeArray(platform_node_list);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::GetRowHeaderItems(SAFEARRAY** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  std::optional<int> row = GetTableRow();
  if (!row)
    return E_FAIL;

  std::vector<int32_t> row_header_ids =
      GetDelegate()->GetRowHeaderNodeIds(*row);

  std::vector<AXPlatformNodeWin*> platform_node_list =
      CreatePlatformNodeVectorFromRelationIdVector(row_header_ids);

  *result = CreateUIAElementsSafeArray(platform_node_list);
  return S_OK;
}

//
// ITableProvider methods.
//

IFACEMETHODIMP AXPlatformNodeWin::GetColumnHeaders(SAFEARRAY** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  std::vector<int32_t> column_header_ids = GetDelegate()->GetColHeaderNodeIds();

  std::vector<AXPlatformNodeWin*> platform_node_list =
      CreatePlatformNodeVectorFromRelationIdVector(column_header_ids);

  *result = CreateUIAElementsSafeArray(platform_node_list);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::GetRowHeaders(SAFEARRAY** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  std::vector<int32_t> row_header_ids = GetDelegate()->GetRowHeaderNodeIds();

  std::vector<AXPlatformNodeWin*> platform_node_list =
      CreatePlatformNodeVectorFromRelationIdVector(row_header_ids);

  *result = CreateUIAElementsSafeArray(platform_node_list);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_RowOrColumnMajor(
    RowOrColumnMajor* result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  // Tables and ARIA grids are always in row major order
  // see AXPlatformNodeBase::GetTableCell
  *result = RowOrColumnMajor_RowMajor;
  return S_OK;
}

//
// IToggleProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::Toggle() {
  UIA_VALIDATE_CALL();
  AXActionData action_data;
  action_data.action = ax::mojom::Action::kDoDefault;

  if (GetDelegate()->AccessibilityPerformAction(action_data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_ToggleState(ToggleState* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  const auto checked_state = GetData().GetCheckedState();
  if (checked_state == ax::mojom::CheckedState::kTrue) {
    *result = ToggleState_On;
  } else if (checked_state == ax::mojom::CheckedState::kMixed) {
    *result = ToggleState_Indeterminate;
  } else {
    *result = ToggleState_Off;
  }
  return S_OK;
}

//
// IValueProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::SetValue(LPCWSTR value) {
  UIA_VALIDATE_CALL();
  if (!value)
    return E_INVALIDARG;

  if (GetData().IsReadOnlyOrDisabled())
    return UIA_E_ELEMENTNOTENABLED;

  AXActionData data;
  data.action = ax::mojom::Action::kSetValue;
  data.value = base::UTF16ToUTF8(fml::WideStringToUtf16(value));
  if (GetDelegate()->AccessibilityPerformAction(data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_IsReadOnly(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = GetData().IsReadOnlyOrDisabled();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_Value(BSTR* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  *result = GetValueAttributeAsBstr(this);
  return S_OK;
}

//
// IWindowProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::SetVisualState(
    WindowVisualState window_visual_state) {
  UIA_VALIDATE_CALL();
  return UIA_E_NOTSUPPORTED;
}

IFACEMETHODIMP AXPlatformNodeWin::Close() {
  UIA_VALIDATE_CALL();
  return UIA_E_NOTSUPPORTED;
}

IFACEMETHODIMP AXPlatformNodeWin::WaitForInputIdle(int milliseconds,
                                                   BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  return UIA_E_NOTSUPPORTED;
}

IFACEMETHODIMP AXPlatformNodeWin::get_CanMaximize(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  return UIA_E_NOTSUPPORTED;
}

IFACEMETHODIMP AXPlatformNodeWin::get_CanMinimize(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  return UIA_E_NOTSUPPORTED;
}

IFACEMETHODIMP AXPlatformNodeWin::get_IsModal(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  *result = GetBoolAttribute(ax::mojom::BoolAttribute::kModal);

  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_WindowVisualState(
    WindowVisualState* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  return UIA_E_NOTSUPPORTED;
}

IFACEMETHODIMP AXPlatformNodeWin::get_WindowInteractionState(
    WindowInteractionState* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  return UIA_E_NOTSUPPORTED;
}

IFACEMETHODIMP AXPlatformNodeWin::get_IsTopmost(BOOL* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  return UIA_E_NOTSUPPORTED;
}

//
// IRangeValueProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::SetValue(double value) {
  UIA_VALIDATE_CALL();
  AXActionData data;
  data.action = ax::mojom::Action::kSetValue;
  data.value = base::NumberToString(value);
  if (GetDelegate()->AccessibilityPerformAction(data))
    return S_OK;
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_LargeChange(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  float attribute;
  if (GetFloatAttribute(ax::mojom::FloatAttribute::kStepValueForRange,
                        &attribute)) {
    *result = attribute * kLargeChangeScaleFactor;
    return S_OK;
  }
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_Maximum(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  float attribute;
  if (GetFloatAttribute(ax::mojom::FloatAttribute::kMaxValueForRange,
                        &attribute)) {
    *result = attribute;
    return S_OK;
  }
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_Minimum(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  float attribute;
  if (GetFloatAttribute(ax::mojom::FloatAttribute::kMinValueForRange,
                        &attribute)) {
    *result = attribute;
    return S_OK;
  }
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_SmallChange(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  float attribute;
  if (GetFloatAttribute(ax::mojom::FloatAttribute::kStepValueForRange,
                        &attribute)) {
    *result = attribute;
    return S_OK;
  }
  return E_FAIL;
}

IFACEMETHODIMP AXPlatformNodeWin::get_Value(double* result) {
  UIA_VALIDATE_CALL_1_ARG(result);
  float attribute;
  if (GetFloatAttribute(ax::mojom::FloatAttribute::kValueForRange,
                        &attribute)) {
    *result = attribute;
    return S_OK;
  }
  return E_FAIL;
}

//
// IRawElementProviderFragment implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::Navigate(
    NavigateDirection direction,
    IRawElementProviderFragment** element_provider) {
  UIA_VALIDATE_CALL_1_ARG(element_provider);

  *element_provider = nullptr;

  //
  // Navigation to a fragment root node:
  //
  // In order for the platform-neutral accessibility tree to support IA2 and UIA
  // simultaneously, we handle navigation to and from fragment roots in UIA
  // specific code. Consider the following platform-neutral tree:
  //
  //         N1
  //   _____/ \_____
  //  /             \
  // N2---N3---N4---N5
  //     / \       / \
  //   N6---N7   N8---N9
  //
  // N3 and N5 are nodes for which we need a fragment root. This will correspond
  // to the following tree in UIA:
  //
  //         U1
  //   _____/ \_____
  //  /             \
  // U2---R3---U4---R5
  //      |         |
  //      U3        U5
  //     / \       / \
  //   U6---U7   U8---U9
  //
  // Ux is the platform node for Nx.
  // R3 and R5 are the fragment root nodes for U3 and U5 respectively.
  //
  // Navigation has the following behaviors:
  //
  // 1. Parent navigation: If source node Ux is the child of a fragment root,
  //    return Rx. Otherwise, consult the platform-neutral tree.
  // 2. First/last child navigation: If target node Ux is the child of a
  //    fragment root and the source node isn't Rx, return Rx. Otherwise, return
  //    Ux.
  // 3. Next/previous sibling navigation:
  //    a. If source node Ux is the child of a fragment root, return nullptr.
  //    b. If target node Ux is the child of a fragment root, return Rx.
  //       Otherwise, return Ux.
  //
  // Note that the condition in 3b is a special case of the condition in 2. In
  // 3b, the source node is never Rx. So in the code, we collapse them to a
  // common implementation.
  //
  // Navigation from an Rx node is set up by delegate APIs on AXFragmentRootWin.
  //
  gfx::NativeViewAccessible neighbor = nullptr;
  switch (direction) {
    case NavigateDirection_Parent: {
      // 1. If source node Ux is the child of a fragment root, return Rx.
      // Otherwise, consult the platform-neutral tree.
      AXFragmentRootWin* fragment_root =
          AXFragmentRootWin::GetFragmentRootParentOf(GetNativeViewAccessible());
      if (BASE_UNLIKELY(fragment_root)) {
        neighbor = fragment_root->GetNativeViewAccessible();
      } else {
        neighbor = GetParent();
      }
    } break;

    case NavigateDirection_FirstChild:
      if (GetChildCount() > 0)
        neighbor = GetFirstChild()->GetNativeViewAccessible();
      break;

    case NavigateDirection_LastChild:
      if (GetChildCount() > 0)
        neighbor = GetLastChild()->GetNativeViewAccessible();
      break;

    case NavigateDirection_NextSibling:
      // 3a. If source node Ux is the child of a fragment root, return nullptr.
      if (AXFragmentRootWin::GetFragmentRootParentOf(
              GetNativeViewAccessible()) == nullptr) {
        AXPlatformNodeBase* neighbor_node = GetNextSibling();
        if (neighbor_node)
          neighbor = neighbor_node->GetNativeViewAccessible();
      }
      break;

    case NavigateDirection_PreviousSibling:
      // 3a. If source node Ux is the child of a fragment root, return nullptr.
      if (AXFragmentRootWin::GetFragmentRootParentOf(
              GetNativeViewAccessible()) == nullptr) {
        AXPlatformNodeBase* neighbor_node = GetPreviousSibling();
        if (neighbor_node)
          neighbor = neighbor_node->GetNativeViewAccessible();
      }
      break;

    default:
      BASE_UNREACHABLE();
      break;
  }

  if (neighbor) {
    if (direction != NavigateDirection_Parent) {
      // 2 / 3b. If target node Ux is the child of a fragment root and the
      // source node isn't Rx, return Rx.
      AXFragmentRootWin* fragment_root =
          AXFragmentRootWin::GetFragmentRootParentOf(neighbor);
      if (BASE_UNLIKELY(fragment_root && fragment_root != GetDelegate()))
        neighbor = fragment_root->GetNativeViewAccessible();
    }
    neighbor->QueryInterface(IID_PPV_ARGS(element_provider));
  }

  return S_OK;
}

void AXPlatformNodeWin::GetRuntimeIdArray(
    AXPlatformNodeWin::RuntimeIdArray& runtime_id) {
  runtime_id[0] = UiaAppendRuntimeId;
  runtime_id[1] = GetUniqueId();
}

IFACEMETHODIMP AXPlatformNodeWin::GetRuntimeId(SAFEARRAY** runtime_id) {
  UIA_VALIDATE_CALL_1_ARG(runtime_id);

  RuntimeIdArray id_array;
  GetRuntimeIdArray(id_array);
  *runtime_id = ::SafeArrayCreateVector(VT_I4, 0, id_array.size());

  int* array_data = nullptr;
  ::SafeArrayAccessData(*runtime_id, reinterpret_cast<void**>(&array_data));

  size_t runtime_id_byte_count = id_array.size() * sizeof(int);
  memcpy_s(array_data, runtime_id_byte_count, id_array.data(),
           runtime_id_byte_count);

  ::SafeArrayUnaccessData(*runtime_id);

  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_BoundingRectangle(
    UiaRect* screen_physical_pixel_bounds) {
  UIA_VALIDATE_CALL_1_ARG(screen_physical_pixel_bounds);

  gfx::Rect bounds =
      delegate_->GetBoundsRect(AXCoordinateSystem::kScreenPhysicalPixels,
                               AXClippingBehavior::kUnclipped);
  screen_physical_pixel_bounds->left = bounds.x();
  screen_physical_pixel_bounds->top = bounds.y();
  screen_physical_pixel_bounds->width = bounds.width();
  screen_physical_pixel_bounds->height = bounds.height();
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::GetEmbeddedFragmentRoots(
    SAFEARRAY** embedded_fragment_roots) {
  UIA_VALIDATE_CALL_1_ARG(embedded_fragment_roots);

  *embedded_fragment_roots = nullptr;
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::SetFocus() {
  UIA_VALIDATE_CALL();

  AXActionData action_data;
  action_data.action = ax::mojom::Action::kFocus;
  delegate_->AccessibilityPerformAction(action_data);
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_FragmentRoot(
    IRawElementProviderFragmentRoot** fragment_root) {
  UIA_VALIDATE_CALL_1_ARG(fragment_root);

  gfx::AcceleratedWidget widget =
      delegate_->GetTargetForNativeAccessibilityEvent();
  if (widget) {
    AXFragmentRootWin* root =
        AXFragmentRootWin::GetForAcceleratedWidget(widget);
    if (root != nullptr) {
      root->GetNativeViewAccessible()->QueryInterface(
          IID_PPV_ARGS(fragment_root));
      BASE_DCHECK(*fragment_root);
      return S_OK;
    }
  }

  *fragment_root = nullptr;
  return UIA_E_ELEMENTNOTAVAILABLE;
}

//
// IRawElementProviderSimple implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::GetPatternProvider(PATTERNID pattern_id,
                                                     IUnknown** result) {
  return GetPatternProviderImpl(pattern_id, result);
}

HRESULT AXPlatformNodeWin::GetPatternProviderImpl(PATTERNID pattern_id,
                                                  IUnknown** result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  *result = nullptr;

  PatternProviderFactoryMethod factory_method =
      GetPatternProviderFactoryMethod(pattern_id);
  if (factory_method)
    (*factory_method)(this, result);

  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::GetPropertyValue(PROPERTYID property_id,
                                                   VARIANT* result) {
  return GetPropertyValueImpl(property_id, result);
}

HRESULT AXPlatformNodeWin::GetPropertyValueImpl(PROPERTYID property_id,
                                                VARIANT* result) {
  UIA_VALIDATE_CALL_1_ARG(result);

  result->vt = VT_EMPTY;

  int int_attribute;
  const AXNodeData& data = GetData();

  // Default UIA Property Ids.
  switch (property_id) {
    case UIA_AriaPropertiesPropertyId:
      result->vt = VT_BSTR;
      result->bstrVal = ::SysAllocString(
          fml::Utf16ToWideString(ComputeUIAProperties()).c_str());
      break;

    case UIA_AriaRolePropertyId:
      result->vt = VT_BSTR;
      result->bstrVal =
          ::SysAllocString(fml::Utf16ToWideString(UIAAriaRole()).c_str());
      break;

    case UIA_AutomationIdPropertyId:
      V_VT(result) = VT_BSTR;
      V_BSTR(result) = ::SysAllocString(
          fml::Utf16ToWideString(GetDelegate()->GetAuthorUniqueId()).c_str());
      break;

    case UIA_ClassNamePropertyId:
      result->vt = VT_BSTR;
      GetStringAttributeAsBstr(ax::mojom::StringAttribute::kClassName,
                               &result->bstrVal);
      break;

    case UIA_ClickablePointPropertyId:
      result->vt = VT_ARRAY | VT_R8;
      result->parray = CreateClickablePointArray();
      break;

    case UIA_ControllerForPropertyId:
      result->vt = VT_ARRAY | VT_UNKNOWN;
      result->parray = CreateUIAControllerForArray();
      break;

    case UIA_ControlTypePropertyId:
      result->vt = VT_I4;
      result->lVal = ComputeUIAControlType();
      break;

    case UIA_CulturePropertyId: {
      std::optional<LCID> lcid = GetCultureAttributeAsLCID();
      if (!lcid)
        return E_FAIL;
      result->vt = VT_I4;
      result->lVal = lcid.value();
      break;
    }

    case UIA_DescribedByPropertyId:
      result->vt = VT_ARRAY | VT_UNKNOWN;
      result->parray = CreateUIAElementsArrayForRelation(
          ax::mojom::IntListAttribute::kDescribedbyIds);
      break;

    case UIA_FlowsFromPropertyId:
      V_VT(result) = VT_ARRAY | VT_UNKNOWN;
      V_ARRAY(result) = CreateUIAElementsArrayForReverseRelation(
          ax::mojom::IntListAttribute::kFlowtoIds);
      break;

    case UIA_FlowsToPropertyId:
      result->vt = VT_ARRAY | VT_UNKNOWN;
      result->parray = CreateUIAElementsArrayForRelation(
          ax::mojom::IntListAttribute::kFlowtoIds);
      break;

    case UIA_FrameworkIdPropertyId:
      V_VT(result) = VT_BSTR;
      V_BSTR(result) = SysAllocString(FRAMEWORK_ID);
      break;

    case UIA_HasKeyboardFocusPropertyId:
      result->vt = VT_BOOL;
      result->boolVal = (delegate_->GetFocus() == GetNativeViewAccessible())
                            ? VARIANT_TRUE
                            : VARIANT_FALSE;
      break;

    case UIA_FullDescriptionPropertyId:
      result->vt = VT_BSTR;
      GetStringAttributeAsBstr(ax::mojom::StringAttribute::kDescription,
                               &result->bstrVal);
      break;

    case UIA_HelpTextPropertyId:
      if (HasStringAttribute(ax::mojom::StringAttribute::kPlaceholder)) {
        V_VT(result) = VT_BSTR;
        GetStringAttributeAsBstr(ax::mojom::StringAttribute::kPlaceholder,
                                 &V_BSTR(result));
      } else if (data.GetNameFrom() == ax::mojom::NameFrom::kPlaceholder ||
                 data.GetNameFrom() == ax::mojom::NameFrom::kTitle) {
        V_VT(result) = VT_BSTR;
        GetNameAsBstr(&V_BSTR(result));
      } else if (HasStringAttribute(ax::mojom::StringAttribute::kTooltip)) {
        V_VT(result) = VT_BSTR;
        GetStringAttributeAsBstr(ax::mojom::StringAttribute::kTooltip,
                                 &V_BSTR(result));
      }
      break;

    case UIA_IsContentElementPropertyId:
    case UIA_IsControlElementPropertyId:
      result->vt = VT_BOOL;
      result->boolVal = IsUIAControl() ? VARIANT_TRUE : VARIANT_FALSE;
      break;

    case UIA_IsDataValidForFormPropertyId:
      if (data.GetIntAttribute(ax::mojom::IntAttribute::kInvalidState,
                               &int_attribute)) {
        result->vt = VT_BOOL;
        result->boolVal =
            (static_cast<int>(ax::mojom::InvalidState::kFalse) == int_attribute)
                ? VARIANT_TRUE
                : VARIANT_FALSE;
      }
      break;

    case UIA_IsDialogPropertyId:
      result->vt = VT_BOOL;
      result->boolVal = IsDialog(data.role) ? VARIANT_TRUE : VARIANT_FALSE;
      break;

    case UIA_IsKeyboardFocusablePropertyId:
      result->vt = VT_BOOL;
      result->boolVal =
          ShouldNodeHaveFocusableState(data) ? VARIANT_TRUE : VARIANT_FALSE;
      break;

    case UIA_IsOffscreenPropertyId:
      result->vt = VT_BOOL;
      result->boolVal =
          GetDelegate()->IsOffscreen() ? VARIANT_TRUE : VARIANT_FALSE;
      break;

    case UIA_IsRequiredForFormPropertyId:
      result->vt = VT_BOOL;
      if (data.HasState(ax::mojom::State::kRequired)) {
        result->boolVal = VARIANT_TRUE;
      } else {
        result->boolVal = VARIANT_FALSE;
      }
      break;

    case UIA_ItemStatusPropertyId: {
      // https://www.w3.org/TR/core-aam-1.1/#mapping_state-property_table
      // aria-sort='ascending|descending|other' is mapped for the
      // HeaderItem Control Type.
      int32_t sort_direction;
      if (IsTableHeader(data.role) &&
          GetIntAttribute(ax::mojom::IntAttribute::kSortDirection,
                          &sort_direction)) {
        switch (static_cast<ax::mojom::SortDirection>(sort_direction)) {
          case ax::mojom::SortDirection::kNone:
          case ax::mojom::SortDirection::kUnsorted:
            break;
          case ax::mojom::SortDirection::kAscending:
            V_VT(result) = VT_BSTR;
            V_BSTR(result) = SysAllocString(L"ascending");
            break;
          case ax::mojom::SortDirection::kDescending:
            V_VT(result) = VT_BSTR;
            V_BSTR(result) = SysAllocString(L"descending");
            break;
          case ax::mojom::SortDirection::kOther:
            V_VT(result) = VT_BSTR;
            V_BSTR(result) = SysAllocString(L"other");
            break;
        }
      }
      break;
    }

    case UIA_LabeledByPropertyId:
      if (AXPlatformNodeWin* node = ComputeUIALabeledBy()) {
        result->vt = VT_UNKNOWN;
        result->punkVal = node->GetNativeViewAccessible();
        result->punkVal->AddRef();
      }
      break;

    case UIA_LocalizedControlTypePropertyId: {
      std::u16string localized_control_type = GetRoleDescription();
      if (!localized_control_type.empty()) {
        result->vt = VT_BSTR;
        result->bstrVal = ::SysAllocString(
            fml::Utf16ToWideString(localized_control_type).c_str());
      }
      // If a role description has not been provided, leave as VT_EMPTY.
      // UIA core handles Localized Control type for some built-in types and
      // also has a mapping for ARIA roles. To get these defaults, we need to
      // have returned VT_EMPTY.
    } break;

    case UIA_NamePropertyId:
      if (IsNameExposed()) {
        result->vt = VT_BSTR;
        GetNameAsBstr(&result->bstrVal);
      }
      break;

    case UIA_OrientationPropertyId:
      if (SupportsOrientation(data.role)) {
        if (data.HasState(ax::mojom::State::kHorizontal) &&
            data.HasState(ax::mojom::State::kVertical)) {
          BASE_UNREACHABLE();  // << "An accessibility object cannot have a
                               // horizontal "
                               //"and a vertical orientation at the same time.";
        }
        if (data.HasState(ax::mojom::State::kHorizontal)) {
          result->vt = VT_I4;
          result->intVal = OrientationType_Horizontal;
        }
        if (data.HasState(ax::mojom::State::kVertical)) {
          result->vt = VT_I4;
          result->intVal = OrientationType_Vertical;
        }
      } else {
        result->vt = VT_I4;
        result->intVal = OrientationType_None;
      }
      break;

    case UIA_IsEnabledPropertyId:
      V_VT(result) = VT_BOOL;
      switch (data.GetRestriction()) {
        case ax::mojom::Restriction::kDisabled:
          V_BOOL(result) = VARIANT_FALSE;
          break;

        case ax::mojom::Restriction::kNone:
        case ax::mojom::Restriction::kReadOnly:
          V_BOOL(result) = VARIANT_TRUE;
          break;
      }
      break;

    case UIA_IsPasswordPropertyId:
      result->vt = VT_BOOL;
      result->boolVal = data.HasState(ax::mojom::State::kProtected)
                            ? VARIANT_TRUE
                            : VARIANT_FALSE;
      break;

    case UIA_AcceleratorKeyPropertyId:
      if (data.HasStringAttribute(ax::mojom::StringAttribute::kKeyShortcuts)) {
        result->vt = VT_BSTR;
        GetStringAttributeAsBstr(ax::mojom::StringAttribute::kKeyShortcuts,
                                 &result->bstrVal);
      }
      break;

    case UIA_AccessKeyPropertyId:
      if (data.HasStringAttribute(ax::mojom::StringAttribute::kAccessKey)) {
        result->vt = VT_BSTR;
        GetStringAttributeAsBstr(ax::mojom::StringAttribute::kAccessKey,
                                 &result->bstrVal);
      }
      break;

    case UIA_IsPeripheralPropertyId:
      result->vt = VT_BOOL;
      result->boolVal = VARIANT_FALSE;
      break;

    case UIA_LevelPropertyId:
      if (data.GetIntAttribute(ax::mojom::IntAttribute::kHierarchicalLevel,
                               &int_attribute)) {
        result->vt = VT_I4;
        result->intVal = int_attribute;
      }
      break;

    case UIA_LiveSettingPropertyId: {
      result->vt = VT_I4;
      result->intVal = LiveSetting::Off;

      std::string string_attribute;
      if (data.GetStringAttribute(ax::mojom::StringAttribute::kLiveStatus,
                                  &string_attribute)) {
        if (string_attribute == "polite")
          result->intVal = LiveSetting::Polite;
        else if (string_attribute == "assertive")
          result->intVal = LiveSetting::Assertive;
      }
      break;
    }

    case UIA_OptimizeForVisualContentPropertyId:
      result->vt = VT_BOOL;
      result->boolVal = VARIANT_FALSE;
      break;

    case UIA_PositionInSetPropertyId: {
      std::optional<int> pos_in_set = GetPosInSet();
      if (pos_in_set) {
        result->vt = VT_I4;
        result->intVal = *pos_in_set;
      }
    } break;

    case UIA_ScrollHorizontalScrollPercentPropertyId: {
      V_VT(result) = VT_R8;
      V_R8(result) = GetHorizontalScrollPercent();
      break;
    }

    case UIA_ScrollVerticalScrollPercentPropertyId: {
      V_VT(result) = VT_R8;
      V_R8(result) = GetVerticalScrollPercent();
      break;
    }

    case UIA_SizeOfSetPropertyId: {
      std::optional<int> set_size = GetSetSize();
      if (set_size) {
        result->vt = VT_I4;
        result->intVal = *set_size;
      }
      break;
    }

    case UIA_LandmarkTypePropertyId: {
      std::optional<LONG> landmark_type = ComputeUIALandmarkType();
      if (landmark_type) {
        result->vt = VT_I4;
        result->intVal = landmark_type.value();
      }
      break;
    }

    case UIA_LocalizedLandmarkTypePropertyId: {
      std::u16string localized_landmark_type =
          GetDelegate()->GetLocalizedStringForLandmarkType();
      if (!localized_landmark_type.empty()) {
        result->vt = VT_BSTR;
        result->bstrVal = ::SysAllocString(
            fml::Utf16ToWideString(localized_landmark_type).c_str());
      }
      break;
    }

    case UIA_ExpandCollapseExpandCollapseStatePropertyId:
      result->vt = VT_I4;
      result->intVal = static_cast<int>(ComputeExpandCollapseState());
      break;

    case UIA_ToggleToggleStatePropertyId: {
      ToggleState state;
      get_ToggleState(&state);
      result->vt = VT_I4;
      result->lVal = state;
      break;
    }

    case UIA_ValueValuePropertyId:
      result->vt = VT_BSTR;
      result->bstrVal = GetValueAttributeAsBstr(this);
      break;

    // Not currently implemented.
    case UIA_AnnotationObjectsPropertyId:
    case UIA_AnnotationTypesPropertyId:
    case UIA_CenterPointPropertyId:
    case UIA_FillColorPropertyId:
    case UIA_FillTypePropertyId:
    case UIA_HeadingLevelPropertyId:
    case UIA_ItemTypePropertyId:
    case UIA_OutlineColorPropertyId:
    case UIA_OutlineThicknessPropertyId:
    case UIA_RotationPropertyId:
    case UIA_SizePropertyId:
    case UIA_VisualEffectsPropertyId:
      break;

    // Provided by UIA Core; we should not implement.
    case UIA_BoundingRectanglePropertyId:
    case UIA_NativeWindowHandlePropertyId:
    case UIA_ProcessIdPropertyId:
    case UIA_ProviderDescriptionPropertyId:
    case UIA_RuntimeIdPropertyId:
      break;
  }  // End of default UIA property ids.

  // Custom UIA Property Ids.
  if (property_id ==
      UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId()) {
    // We want to negate the unique id for it to be consistent across different
    // Windows accessiblity APIs. The negative unique id convention originated
    // from ::NotifyWinEvent() takes an hwnd and a child id. A 0 child id means
    // self, and a positive child id means child #n. In order to fire an event
    // for an arbitrary descendant of the window, Firefox started the practice
    // of using a negative unique id. We follow the same negative unique id
    // convention here and when we fire events via ::NotifyWinEvent().
    result->vt = VT_BSTR;
    result->bstrVal = ::SysAllocString(
        fml::Utf16ToWideString(base::NumberToString16(-GetUniqueId())).c_str());
  }

  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_ProviderOptions(ProviderOptions* ret) {
  UIA_VALIDATE_CALL_1_ARG(ret);

  *ret = ProviderOptions_ServerSideProvider | ProviderOptions_UseComThreading |
         ProviderOptions_RefuseNonClientSupport |
         ProviderOptions_HasNativeIAccessible;
  return S_OK;
}

IFACEMETHODIMP AXPlatformNodeWin::get_HostRawElementProvider(
    IRawElementProviderSimple** provider) {
  UIA_VALIDATE_CALL_1_ARG(provider);

  *provider = nullptr;
  return S_OK;
}

//
// IRawElementProviderSimple2 implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::ShowContextMenu() {
  UIA_VALIDATE_CALL();

  AXActionData action_data;
  action_data.action = ax::mojom::Action::kShowContextMenu;
  delegate_->AccessibilityPerformAction(action_data);
  return S_OK;
}

//
// IServiceProvider implementation.
//

IFACEMETHODIMP AXPlatformNodeWin::QueryService(REFGUID guidService,
                                               REFIID riid,
                                               void** object) {
  COM_OBJECT_VALIDATE_1_ARG(object);

  if (guidService == IID_IAccessible) {
    return QueryInterface(riid, object);
  }

  // TODO(suproteem): Include IAccessibleEx in the list, potentially checking
  // for version.

  *object = nullptr;
  return E_FAIL;
}

//
// Methods used by the ATL COM map.
//

// static
STDMETHODIMP AXPlatformNodeWin::InternalQueryInterface(
    void* this_ptr,
    const _ATL_INTMAP_ENTRY* entries,
    REFIID riid,
    void** object) {
  if (!object)
    return E_INVALIDARG;
  *object = nullptr;
  AXPlatformNodeWin* accessible =
      reinterpret_cast<AXPlatformNodeWin*>(this_ptr);
  BASE_DCHECK(accessible);

  return CComObjectRootBase::InternalQueryInterface(this_ptr, entries, riid,
                                                    object);
}

HRESULT AXPlatformNodeWin::GetTextAttributeValue(
    TEXTATTRIBUTEID attribute_id,
    const std::optional<int>& start_offset,
    const std::optional<int>& end_offset,
    base::win::VariantVector* result) {
  BASE_DCHECK(!start_offset || start_offset.value() >= 0);
  BASE_DCHECK(!end_offset || end_offset.value() >= 0);

  switch (attribute_id) {
    case UIA_AnnotationTypesAttributeId:
      return GetAnnotationTypesAttribute(start_offset, end_offset, result);
    case UIA_BackgroundColorAttributeId:
      result->Insert<VT_I4>(
          GetIntAttributeAsCOLORREF(ax::mojom::IntAttribute::kBackgroundColor));
      break;
    case UIA_BulletStyleAttributeId:
      result->Insert<VT_I4>(ComputeUIABulletStyle());
      break;
    case UIA_CultureAttributeId: {
      std::optional<LCID> lcid = GetCultureAttributeAsLCID();
      if (!lcid)
        return E_FAIL;
      result->Insert<VT_I4>(lcid.value());
      break;
    }
    case UIA_FontNameAttributeId:
      result->Insert<VT_BSTR>(GetFontNameAttributeAsBSTR());
      break;
    case UIA_FontSizeAttributeId: {
      std::optional<float> font_size_in_points = GetFontSizeInPoints();
      if (font_size_in_points) {
        result->Insert<VT_R8>(*font_size_in_points);
      }
      break;
    }
    case UIA_FontWeightAttributeId:
      result->Insert<VT_I4>(
          GetFloatAttribute(ax::mojom::FloatAttribute::kFontWeight));
      break;
    case UIA_ForegroundColorAttributeId:
      result->Insert<VT_I4>(
          GetIntAttributeAsCOLORREF(ax::mojom::IntAttribute::kColor));
      break;
    case UIA_IsHiddenAttributeId:
      result->Insert<VT_BOOL>(IsInvisibleOrIgnored());
      break;
    case UIA_IsItalicAttributeId:
      result->Insert<VT_BOOL>(
          GetData().HasTextStyle(ax::mojom::TextStyle::kItalic));
      break;
    case UIA_IsReadOnlyAttributeId:
      // Placeholder text should return the enclosing element's read-only value.
      if (IsPlaceholderText()) {
        AXPlatformNodeWin* parent_platform_node =
            static_cast<AXPlatformNodeWin*>(
                FromNativeViewAccessible(GetParent()));
        return parent_platform_node->GetTextAttributeValue(
            attribute_id, start_offset, end_offset, result);
      }
      result->Insert<VT_BOOL>(GetData().IsReadOnlyOrDisabled());
      break;
    case UIA_IsSubscriptAttributeId:
      result->Insert<VT_BOOL>(GetData().GetTextPosition() ==
                              ax::mojom::TextPosition::kSubscript);
      break;
    case UIA_IsSuperscriptAttributeId:
      result->Insert<VT_BOOL>(GetData().GetTextPosition() ==
                              ax::mojom::TextPosition::kSuperscript);
      break;
    case UIA_OverlineStyleAttributeId:
      result->Insert<VT_I4>(GetUIATextDecorationStyle(
          ax::mojom::IntAttribute::kTextOverlineStyle));
      break;
    case UIA_StrikethroughStyleAttributeId:
      result->Insert<VT_I4>(GetUIATextDecorationStyle(
          ax::mojom::IntAttribute::kTextStrikethroughStyle));
      break;
    case UIA_StyleNameAttributeId:
      result->Insert<VT_BSTR>(GetStyleNameAttributeAsBSTR());
      break;
    case UIA_StyleIdAttributeId:
      result->Insert<VT_I4>(ComputeUIAStyleId());
      break;
    case UIA_HorizontalTextAlignmentAttributeId: {
      std::optional<HorizontalTextAlignment> horizontal_text_alignment =
          AXTextAlignToUIAHorizontalTextAlignment(GetData().GetTextAlign());
      if (horizontal_text_alignment)
        result->Insert<VT_I4>(*horizontal_text_alignment);
      break;
    }
    case UIA_UnderlineStyleAttributeId:
      result->Insert<VT_I4>(GetUIATextDecorationStyle(
          ax::mojom::IntAttribute::kTextUnderlineStyle));
      break;
    case UIA_TextFlowDirectionsAttributeId:
      result->Insert<VT_I4>(
          TextDirectionToFlowDirections(GetData().GetTextDirection()));
      break;
    default: {
      Microsoft::WRL::ComPtr<IUnknown> not_supported_value;
      HRESULT hr = ::UiaGetReservedNotSupportedValue(&not_supported_value);
      if (SUCCEEDED(hr))
        result->Insert<VT_UNKNOWN>(not_supported_value.Get());
      return hr;
    } break;
  }

  return S_OK;
}

HRESULT AXPlatformNodeWin::GetAnnotationTypesAttribute(
    const std::optional<int>& start_offset,
    const std::optional<int>& end_offset,
    base::win::VariantVector* result) {
  base::win::VariantVector variant_vector;

  MarkerTypeRangeResult grammar_result = MarkerTypeRangeResult::kNone;
  MarkerTypeRangeResult spelling_result = MarkerTypeRangeResult::kNone;

  if (IsText() || IsPlainTextField()) {
    grammar_result = GetMarkerTypeFromRange(start_offset, end_offset,
                                            ax::mojom::MarkerType::kGrammar);
    spelling_result = GetMarkerTypeFromRange(start_offset, end_offset,
                                             ax::mojom::MarkerType::kSpelling);
  }

  if (grammar_result == MarkerTypeRangeResult::kMixed ||
      spelling_result == MarkerTypeRangeResult::kMixed) {
    Microsoft::WRL::ComPtr<IUnknown> mixed_attribute_value;
    HRESULT hr = ::UiaGetReservedMixedAttributeValue(&mixed_attribute_value);
    if (SUCCEEDED(hr))
      result->Insert<VT_UNKNOWN>(mixed_attribute_value.Get());
    return hr;
  }

  if (spelling_result == MarkerTypeRangeResult::kMatch)
    result->Insert<VT_I4>(AnnotationType_SpellingError);
  if (grammar_result == MarkerTypeRangeResult::kMatch)
    result->Insert<VT_I4>(AnnotationType_GrammarError);

  return S_OK;
}

std::optional<LCID> AXPlatformNodeWin::GetCultureAttributeAsLCID() const {
  const std::u16string language =
      GetInheritedString16Attribute(ax::mojom::StringAttribute::kLanguage);
  const LCID lcid =
      LocaleNameToLCID((wchar_t*)language.c_str(), LOCALE_ALLOW_NEUTRAL_NAMES);
  if (!lcid)
    return std::nullopt;

  return lcid;
}

COLORREF AXPlatformNodeWin::GetIntAttributeAsCOLORREF(
    ax::mojom::IntAttribute attribute) const {
  uint32_t color = GetIntAttribute(attribute);
  // From skia_utils_win.cc
  return (_byteswap_ulong(color) >> 8);
}

BulletStyle AXPlatformNodeWin::ComputeUIABulletStyle() const {
  // UIA expects the list style of a non-list-item to be none however the
  // default list style cascaded is disc not none. Therefore we must ensure that
  // this node is contained within a list-item to distinguish non-list-items and
  // disc styled list items.
  const AXPlatformNodeBase* current_node = this;
  while (current_node &&
         current_node->GetData().role != ax::mojom::Role::kListItem) {
    current_node = FromNativeViewAccessible(current_node->GetParent());
  }

  const ax::mojom::ListStyle list_style =
      current_node ? current_node->GetData().GetListStyle()
                   : ax::mojom::ListStyle::kNone;

  switch (list_style) {
    case ax::mojom::ListStyle::kNone:
      return BulletStyle::BulletStyle_None;
    case ax::mojom::ListStyle::kCircle:
      return BulletStyle::BulletStyle_HollowRoundBullet;
    case ax::mojom::ListStyle::kDisc:
      return BulletStyle::BulletStyle_FilledRoundBullet;
    case ax::mojom::ListStyle::kImage:
      return BulletStyle::BulletStyle_Other;
    case ax::mojom::ListStyle::kNumeric:
    case ax::mojom::ListStyle::kOther:
      return BulletStyle::BulletStyle_None;
    case ax::mojom::ListStyle::kSquare:
      return BulletStyle::BulletStyle_FilledSquareBullet;
  }
}

LONG AXPlatformNodeWin::ComputeUIAStyleId() const {
  const AXPlatformNodeBase* current_node = this;
  do {
    switch (current_node->GetData().role) {
      case ax::mojom::Role::kHeading:
        return AXHierarchicalLevelToUIAStyleId(current_node->GetIntAttribute(
            ax::mojom::IntAttribute::kHierarchicalLevel));
      case ax::mojom::Role::kListItem:
        return AXListStyleToUIAStyleId(current_node->GetData().GetListStyle());
      case ax::mojom::Role::kMark:
        return StyleId_Custom;
      case ax::mojom::Role::kBlockquote:
        return StyleId_Quote;
      default:
        break;
    }
    current_node = FromNativeViewAccessible(current_node->GetParent());
  } while (current_node);

  return StyleId_Normal;
}

// static
std::optional<HorizontalTextAlignment>
AXPlatformNodeWin::AXTextAlignToUIAHorizontalTextAlignment(
    ax::mojom::TextAlign text_align) {
  switch (text_align) {
    case ax::mojom::TextAlign::kNone:
      return std::nullopt;
    case ax::mojom::TextAlign::kLeft:
      return HorizontalTextAlignment_Left;
    case ax::mojom::TextAlign::kRight:
      return HorizontalTextAlignment_Right;
    case ax::mojom::TextAlign::kCenter:
      return HorizontalTextAlignment_Centered;
    case ax::mojom::TextAlign::kJustify:
      return HorizontalTextAlignment_Justified;
  }
}

// static
LONG AXPlatformNodeWin::AXHierarchicalLevelToUIAStyleId(
    int32_t hierarchical_level) {
  switch (hierarchical_level) {
    case 0:
      return StyleId_Normal;
    case 1:
      return StyleId_Heading1;
    case 2:
      return StyleId_Heading2;
    case 3:
      return StyleId_Heading3;
    case 4:
      return StyleId_Heading4;
    case 5:
      return StyleId_Heading5;
    case 6:
      return StyleId_Heading6;
    case 7:
      return StyleId_Heading7;
    case 8:
      return StyleId_Heading8;
    case 9:
      return StyleId_Heading9;
    default:
      return StyleId_Custom;
  }
}

// static
LONG AXPlatformNodeWin::AXListStyleToUIAStyleId(
    ax::mojom::ListStyle list_style) {
  switch (list_style) {
    case ax::mojom::ListStyle::kNone:
      return StyleId_Normal;
    case ax::mojom::ListStyle::kCircle:
    case ax::mojom::ListStyle::kDisc:
    case ax::mojom::ListStyle::kImage:
    case ax::mojom::ListStyle::kSquare:
      return StyleId_BulletedList;
    case ax::mojom::ListStyle::kNumeric:
    case ax::mojom::ListStyle::kOther:
      return StyleId_NumberedList;
  }
}

// static
FlowDirections AXPlatformNodeWin::TextDirectionToFlowDirections(
    ax::mojom::WritingDirection text_direction) {
  switch (text_direction) {
    case ax::mojom::WritingDirection::kNone:
      return FlowDirections::FlowDirections_Default;
    case ax::mojom::WritingDirection::kLtr:
      return FlowDirections::FlowDirections_Default;
    case ax::mojom::WritingDirection::kRtl:
      return FlowDirections::FlowDirections_RightToLeft;
    case ax::mojom::WritingDirection::kTtb:
      return FlowDirections::FlowDirections_Vertical;
    case ax::mojom::WritingDirection::kBtt:
      return FlowDirections::FlowDirections_BottomToTop;
  }
}

// static
void AXPlatformNodeWin::AggregateRangesForMarkerType(
    AXPlatformNodeBase* node,
    ax::mojom::MarkerType marker_type,
    int offset_ranges_amount,
    std::vector<std::pair<int, int>>* ranges) {
  BASE_DCHECK(node->IsText());
  const std::vector<int32_t>& marker_types =
      node->GetIntListAttribute(ax::mojom::IntListAttribute::kMarkerTypes);
  const std::vector<int>& marker_starts =
      node->GetIntListAttribute(ax::mojom::IntListAttribute::kMarkerStarts);
  const std::vector<int>& marker_ends =
      node->GetIntListAttribute(ax::mojom::IntListAttribute::kMarkerEnds);

  for (size_t i = 0; i < marker_types.size(); ++i) {
    if (static_cast<ax::mojom::MarkerType>(marker_types[i]) != marker_type)
      continue;

    const int marker_start = marker_starts[i] + offset_ranges_amount;
    const int marker_end = marker_ends[i] + offset_ranges_amount;
    ranges->emplace_back(std::make_pair(marker_start, marker_end));
  }
}

AXPlatformNodeWin::MarkerTypeRangeResult
AXPlatformNodeWin::GetMarkerTypeFromRange(
    const std::optional<int>& start_offset,
    const std::optional<int>& end_offset,
    ax::mojom::MarkerType marker_type) {
  BASE_DCHECK(IsText() || IsPlainTextField());
  std::vector<std::pair<int, int>> relevant_ranges;

  if (IsText()) {
    AggregateRangesForMarkerType(this, marker_type, /*offset_ranges_amount=*/0,
                                 &relevant_ranges);
  } else if (IsPlainTextField()) {
    int offset_ranges_amount = 0;
    for (AXPlatformNodeBase* static_text = GetFirstTextOnlyDescendant();
         static_text; static_text = static_text->GetNextSibling()) {
      const int child_offset_ranges_amount = offset_ranges_amount;
      if (start_offset || end_offset) {
        // Break if the current node is after the desired |end_offset|.
        if (end_offset && child_offset_ranges_amount > end_offset.value())
          break;

        // Skip over nodes preceding the desired |start_offset|.
        offset_ranges_amount += static_text->GetHypertext().length();
        if (start_offset && offset_ranges_amount < start_offset.value())
          continue;
      }

      AggregateRangesForMarkerType(static_text, marker_type,
                                   child_offset_ranges_amount,
                                   &relevant_ranges);
    }
  }

  // Sort the ranges by their start offset.
  const auto sort_ranges_by_start_offset = [](const std::pair<int, int>& a,
                                              const std::pair<int, int>& b) {
    return a.first < b.first;
  };
  std::sort(relevant_ranges.begin(), relevant_ranges.end(),
            sort_ranges_by_start_offset);

  // Validate that the desired range has a contiguous MarkerType.
  std::optional<std::pair<int, int>> contiguous_range;
  for (const std::pair<int, int>& range : relevant_ranges) {
    if (end_offset && range.first > end_offset.value())
      break;
    if (start_offset && range.second < start_offset.value())
      continue;

    if (!contiguous_range) {
      contiguous_range = range;
      continue;
    }

    // If there is a gap, then the range must be mixed.
    if ((range.first - contiguous_range->second) > 1)
      return MarkerTypeRangeResult::kMixed;

    // Expand the range if possible.
    contiguous_range->second = std::max(contiguous_range->second, range.second);
  }

  // The desired range does not overlap with |marker_type|.
  if (!contiguous_range)
    return MarkerTypeRangeResult::kNone;

  // If there is a partial overlap, then the desired range must be mixed.
  // 1. The |start_offset| is not specified, treat it as offset 0.
  if (!start_offset && contiguous_range->first > 0)
    return MarkerTypeRangeResult::kMixed;
  // 2. The |end_offset| is not specified, treat it as max text offset.
  if (!end_offset && contiguous_range->second < GetHypertext().length())
    return MarkerTypeRangeResult::kMixed;
  // 3. The |start_offset| is specified, but is before the first matching range.
  if (start_offset && start_offset.value() < contiguous_range->first)
    return MarkerTypeRangeResult::kMixed;
  // 4. The |end_offset| is specified, but is after the last matching range.
  if (end_offset && end_offset.value() > contiguous_range->second)
    return MarkerTypeRangeResult::kMixed;

  // The desired range is a complete match for |marker_type|.
  return MarkerTypeRangeResult::kMatch;
}

// IRawElementProviderSimple support methods.

bool AXPlatformNodeWin::IsPatternProviderSupported(PATTERNID pattern_id) {
  return GetPatternProviderFactoryMethod(pattern_id);
}

//
// Private member functions.
//
int AXPlatformNodeWin::MSAARole() {
  // If this is a web area for a presentational iframe, give it a role of
  // something other than DOCUMENT so that the fact that it's a separate doc
  // is not exposed to AT.
  if (IsWebAreaForPresentationalIframe())
    return ROLE_SYSTEM_GROUPING;

  switch (GetData().role) {
    case ax::mojom::Role::kAlert:
      return ROLE_SYSTEM_ALERT;

    case ax::mojom::Role::kAlertDialog:
      return ROLE_SYSTEM_DIALOG;

    case ax::mojom::Role::kAnchor:
      return ROLE_SYSTEM_LINK;

    case ax::mojom::Role::kComment:
    case ax::mojom::Role::kSuggestion:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kApplication:
      return ROLE_SYSTEM_APPLICATION;

    case ax::mojom::Role::kArticle:
      return ROLE_SYSTEM_DOCUMENT;

    case ax::mojom::Role::kAudio:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kBanner:
    case ax::mojom::Role::kHeader:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kBlockquote:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kButton:
      return ROLE_SYSTEM_PUSHBUTTON;

    case ax::mojom::Role::kCanvas:
      return ROLE_SYSTEM_GRAPHIC;

    case ax::mojom::Role::kCaption:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kCaret:
      return ROLE_SYSTEM_CARET;

    case ax::mojom::Role::kCell:
      return ROLE_SYSTEM_CELL;

    case ax::mojom::Role::kCheckBox:
      return ROLE_SYSTEM_CHECKBUTTON;

    case ax::mojom::Role::kClient:
      return ROLE_SYSTEM_PANE;

    case ax::mojom::Role::kColorWell:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kColumn:
      return ROLE_SYSTEM_COLUMN;

    case ax::mojom::Role::kColumnHeader:
      return ROLE_SYSTEM_COLUMNHEADER;

    case ax::mojom::Role::kComboBoxGrouping:
    case ax::mojom::Role::kComboBoxMenuButton:
      return ROLE_SYSTEM_COMBOBOX;

    case ax::mojom::Role::kComplementary:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kContentDeletion:
    case ax::mojom::Role::kContentInsertion:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kContentInfo:
    case ax::mojom::Role::kFooter:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kDate:
    case ax::mojom::Role::kDateTime:
      return ROLE_SYSTEM_DROPLIST;

    case ax::mojom::Role::kDefinition:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kDescriptionListDetail:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kDescriptionList:
      return ROLE_SYSTEM_LIST;

    case ax::mojom::Role::kDescriptionListTerm:
      return ROLE_SYSTEM_LISTITEM;

    case ax::mojom::Role::kDesktop:
      return ROLE_SYSTEM_PANE;

    case ax::mojom::Role::kDetails:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kDialog:
      return ROLE_SYSTEM_DIALOG;

    case ax::mojom::Role::kDisclosureTriangle:
      return ROLE_SYSTEM_PUSHBUTTON;

    case ax::mojom::Role::kDirectory:
      return ROLE_SYSTEM_LIST;

    case ax::mojom::Role::kDocCover:
      return ROLE_SYSTEM_GRAPHIC;

    case ax::mojom::Role::kDocBackLink:
    case ax::mojom::Role::kDocBiblioRef:
    case ax::mojom::Role::kDocGlossRef:
    case ax::mojom::Role::kDocNoteRef:
      return ROLE_SYSTEM_LINK;

    case ax::mojom::Role::kDocBiblioEntry:
    case ax::mojom::Role::kDocEndnote:
    case ax::mojom::Role::kDocFootnote:
      return ROLE_SYSTEM_LISTITEM;

    case ax::mojom::Role::kDocPageBreak:
      return ROLE_SYSTEM_SEPARATOR;

    case ax::mojom::Role::kDocAbstract:
    case ax::mojom::Role::kDocAcknowledgments:
    case ax::mojom::Role::kDocAfterword:
    case ax::mojom::Role::kDocAppendix:
    case ax::mojom::Role::kDocBibliography:
    case ax::mojom::Role::kDocChapter:
    case ax::mojom::Role::kDocColophon:
    case ax::mojom::Role::kDocConclusion:
    case ax::mojom::Role::kDocCredit:
    case ax::mojom::Role::kDocCredits:
    case ax::mojom::Role::kDocDedication:
    case ax::mojom::Role::kDocEndnotes:
    case ax::mojom::Role::kDocEpigraph:
    case ax::mojom::Role::kDocEpilogue:
    case ax::mojom::Role::kDocErrata:
    case ax::mojom::Role::kDocExample:
    case ax::mojom::Role::kDocForeword:
    case ax::mojom::Role::kDocGlossary:
    case ax::mojom::Role::kDocIndex:
    case ax::mojom::Role::kDocIntroduction:
    case ax::mojom::Role::kDocNotice:
    case ax::mojom::Role::kDocPageList:
    case ax::mojom::Role::kDocPart:
    case ax::mojom::Role::kDocPreface:
    case ax::mojom::Role::kDocPrologue:
    case ax::mojom::Role::kDocPullquote:
    case ax::mojom::Role::kDocQna:
    case ax::mojom::Role::kDocSubtitle:
    case ax::mojom::Role::kDocTip:
    case ax::mojom::Role::kDocToc:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kDocument:
    case ax::mojom::Role::kRootWebArea:
    case ax::mojom::Role::kWebArea:
      return ROLE_SYSTEM_DOCUMENT;

    case ax::mojom::Role::kEmbeddedObject:
      // Even though the HTML-AAM has ROLE_SYSTEM_CLIENT for <embed>, we are
      // forced to use ROLE_SYSTEM_GROUPING when the <embed> has children in the
      // accessibility tree.
      // https://www.w3.org/TR/html-aam-1.0/#html-element-role-mappings
      //
      // Screen readers Jaws and NVDA do not "see" any of the <embed>'s contents
      // if they are represented as its children in the accessibility tree. For
      // example, one of the places that would be negatively impacted is the
      // reading of PDFs.
      if (GetDelegate()->GetChildCount()) {
        return ROLE_SYSTEM_GROUPING;
      } else {
        return ROLE_SYSTEM_CLIENT;
      }

    case ax::mojom::Role::kFigcaption:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kFigure:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kFeed:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kFooterAsNonLandmark:
    case ax::mojom::Role::kHeaderAsNonLandmark:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kForm:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kGenericContainer:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kGraphicsDocument:
      return ROLE_SYSTEM_DOCUMENT;

    case ax::mojom::Role::kGraphicsObject:
      return ROLE_SYSTEM_PANE;

    case ax::mojom::Role::kGraphicsSymbol:
      return ROLE_SYSTEM_GRAPHIC;

    case ax::mojom::Role::kGrid:
      return ROLE_SYSTEM_TABLE;

    case ax::mojom::Role::kGroup:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kHeading:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kIframe:
      return ROLE_SYSTEM_DOCUMENT;

    case ax::mojom::Role::kIframePresentational:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kImage:
    case ax::mojom::Role::kImageMap:
      return ROLE_SYSTEM_GRAPHIC;

    case ax::mojom::Role::kInputTime:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kInlineTextBox:
      return ROLE_SYSTEM_STATICTEXT;

    case ax::mojom::Role::kLabelText:
    case ax::mojom::Role::kLegend:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kLayoutTable:
      return ROLE_SYSTEM_TABLE;

    case ax::mojom::Role::kLayoutTableCell:
      return ROLE_SYSTEM_CELL;

    case ax::mojom::Role::kLayoutTableRow:
      return ROLE_SYSTEM_ROW;

    case ax::mojom::Role::kLink:
      return ROLE_SYSTEM_LINK;

    case ax::mojom::Role::kList:
      return ROLE_SYSTEM_LIST;

    case ax::mojom::Role::kListBox:
      return ROLE_SYSTEM_LIST;

    case ax::mojom::Role::kListBoxOption:
      return ROLE_SYSTEM_LISTITEM;

    case ax::mojom::Role::kListGrid:
      return ROLE_SYSTEM_LIST;

    case ax::mojom::Role::kListItem:
      return ROLE_SYSTEM_LISTITEM;

    case ax::mojom::Role::kListMarker:
      if (!GetDelegate()->GetChildCount()) {
        // There's only a name attribute when using Legacy layout. With Legacy
        // layout, list markers have no child and are considered as StaticText.
        // We consider a list marker as a group in LayoutNG since it has
        // a text child node.
        return ROLE_SYSTEM_STATICTEXT;
      }
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kLog:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kMain:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kMark:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kMarquee:
      return ROLE_SYSTEM_ANIMATION;

    case ax::mojom::Role::kMath:
      return ROLE_SYSTEM_EQUATION;

    case ax::mojom::Role::kMenu:
      return ROLE_SYSTEM_MENUPOPUP;

    case ax::mojom::Role::kMenuBar:
      return ROLE_SYSTEM_MENUBAR;

    case ax::mojom::Role::kMenuItem:
    case ax::mojom::Role::kMenuItemCheckBox:
    case ax::mojom::Role::kMenuItemRadio:
      return ROLE_SYSTEM_MENUITEM;

    case ax::mojom::Role::kMenuListPopup:
      return ROLE_SYSTEM_LIST;

    case ax::mojom::Role::kMenuListOption:
      return ROLE_SYSTEM_LISTITEM;

    case ax::mojom::Role::kMeter:
      return ROLE_SYSTEM_PROGRESSBAR;

    case ax::mojom::Role::kNavigation:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kNote:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kParagraph:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kPdfActionableHighlight:
      return ROLE_SYSTEM_PUSHBUTTON;

    case ax::mojom::Role::kPluginObject:
      // See also case ax::mojom::Role::kEmbeddedObject.
      if (GetDelegate()->GetChildCount()) {
        return ROLE_SYSTEM_GROUPING;
      } else {
        return ROLE_SYSTEM_CLIENT;
      }

    case ax::mojom::Role::kPopUpButton: {
      std::string html_tag =
          GetData().GetStringAttribute(ax::mojom::StringAttribute::kHtmlTag);
      if (html_tag == "select")
        return ROLE_SYSTEM_COMBOBOX;
      return ROLE_SYSTEM_BUTTONMENU;
    }

    case ax::mojom::Role::kPortal:
      return ROLE_SYSTEM_PUSHBUTTON;

    case ax::mojom::Role::kPre:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kProgressIndicator:
      return ROLE_SYSTEM_PROGRESSBAR;

    case ax::mojom::Role::kRadioButton:
      return ROLE_SYSTEM_RADIOBUTTON;

    case ax::mojom::Role::kRadioGroup:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kRegion:
      return ROLE_SYSTEM_PANE;

    case ax::mojom::Role::kRow: {
      // Role changes depending on whether row is inside a treegrid
      // https://www.w3.org/TR/core-aam-1.1/#role-map-row
      return IsInTreeGrid() ? ROLE_SYSTEM_OUTLINEITEM : ROLE_SYSTEM_ROW;
    }

    case ax::mojom::Role::kRowGroup:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kRowHeader:
      return ROLE_SYSTEM_ROWHEADER;

    case ax::mojom::Role::kRuby:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kSection: {
      if (GetNameAsString16().empty()) {
        // Do not use ARIA mapping for nameless <section>.
        return ROLE_SYSTEM_GROUPING;
      }
      // Use ARIA mapping.
      return ROLE_SYSTEM_PANE;
    }

    case ax::mojom::Role::kScrollBar:
      return ROLE_SYSTEM_SCROLLBAR;

    case ax::mojom::Role::kScrollView:
      return ROLE_SYSTEM_PANE;

    case ax::mojom::Role::kSearch:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kSlider:
      return ROLE_SYSTEM_SLIDER;

    case ax::mojom::Role::kSliderThumb:
      return ROLE_SYSTEM_SLIDER;

    case ax::mojom::Role::kSpinButton:
      return ROLE_SYSTEM_SPINBUTTON;

    case ax::mojom::Role::kSwitch:
      return ROLE_SYSTEM_CHECKBUTTON;

    case ax::mojom::Role::kRubyAnnotation:
    case ax::mojom::Role::kStaticText:
      return ROLE_SYSTEM_STATICTEXT;

    case ax::mojom::Role::kStatus:
      return ROLE_SYSTEM_STATUSBAR;

    case ax::mojom::Role::kSplitter:
      return ROLE_SYSTEM_SEPARATOR;

    case ax::mojom::Role::kSvgRoot:
      return ROLE_SYSTEM_GRAPHIC;

    case ax::mojom::Role::kTab:
      return ROLE_SYSTEM_PAGETAB;

    case ax::mojom::Role::kTable:
      return ROLE_SYSTEM_TABLE;

    case ax::mojom::Role::kTableHeaderContainer:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kTabList:
      return ROLE_SYSTEM_PAGETABLIST;

    case ax::mojom::Role::kTabPanel:
      return ROLE_SYSTEM_PROPERTYPAGE;

    case ax::mojom::Role::kTerm:
      return ROLE_SYSTEM_LISTITEM;

    case ax::mojom::Role::kTitleBar:
      return ROLE_SYSTEM_TITLEBAR;

    case ax::mojom::Role::kToggleButton:
      return ROLE_SYSTEM_CHECKBUTTON;

    case ax::mojom::Role::kTextField:
    case ax::mojom::Role::kSearchBox:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kTextFieldWithComboBox:
      return ROLE_SYSTEM_COMBOBOX;

    case ax::mojom::Role::kAbbr:
    case ax::mojom::Role::kCode:
    case ax::mojom::Role::kEmphasis:
    case ax::mojom::Role::kStrong:
    case ax::mojom::Role::kTime:
      return ROLE_SYSTEM_TEXT;

    case ax::mojom::Role::kTimer:
      return ROLE_SYSTEM_CLOCK;

    case ax::mojom::Role::kToolbar:
      return ROLE_SYSTEM_TOOLBAR;

    case ax::mojom::Role::kTooltip:
      return ROLE_SYSTEM_TOOLTIP;

    case ax::mojom::Role::kTree:
      return ROLE_SYSTEM_OUTLINE;

    case ax::mojom::Role::kTreeGrid:
      return ROLE_SYSTEM_OUTLINE;

    case ax::mojom::Role::kTreeItem:
      return ROLE_SYSTEM_OUTLINEITEM;

    case ax::mojom::Role::kLineBreak:
      return ROLE_SYSTEM_WHITESPACE;

    case ax::mojom::Role::kVideo:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kWebView:
      return ROLE_SYSTEM_CLIENT;

    case ax::mojom::Role::kPane:
    case ax::mojom::Role::kWindow:
      // Do not return ROLE_SYSTEM_WINDOW as that is a special MSAA system
      // role used to indicate a real native window object. It is
      // automatically created by oleacc.dll as a parent of the root of our
      // hierarchy, matching the HWND.
      return ROLE_SYSTEM_PANE;

    case ax::mojom::Role::kImeCandidate:
    case ax::mojom::Role::kIgnored:
    case ax::mojom::Role::kKeyboard:
    case ax::mojom::Role::kNone:
    case ax::mojom::Role::kPresentational:
    case ax::mojom::Role::kUnknown:
      return ROLE_SYSTEM_PANE;
  }

  BASE_UNREACHABLE();
  return ROLE_SYSTEM_GROUPING;
}

bool AXPlatformNodeWin::IsWebAreaForPresentationalIframe() {
  if (GetData().role != ax::mojom::Role::kWebArea &&
      GetData().role != ax::mojom::Role::kRootWebArea) {
    return false;
  }

  AXPlatformNodeBase* parent = FromNativeViewAccessible(GetParent());
  if (!parent)
    return false;

  return parent->GetData().role == ax::mojom::Role::kIframePresentational;
}

std::u16string AXPlatformNodeWin::UIAAriaRole() {
  // If this is a web area for a presentational iframe, give it a role of
  // something other than document so that the fact that it's a separate doc
  // is not exposed to AT.
  if (IsWebAreaForPresentationalIframe())
    return u"group";

  switch (GetData().role) {
    case ax::mojom::Role::kAlert:
      return u"alert";

    case ax::mojom::Role::kAlertDialog:
      // Our MSAA implementation suggests the use of
      // "alert", not "alertdialog" because some
      // Windows screen readers are not compatible with
      // |ax::mojom::Role::kAlertDialog| yet.
      return u"alert";

    case ax::mojom::Role::kAnchor:
      return u"link";

    case ax::mojom::Role::kComment:
    case ax::mojom::Role::kSuggestion:
      return u"group";

    case ax::mojom::Role::kApplication:
      return u"application";

    case ax::mojom::Role::kArticle:
      return u"article";

    case ax::mojom::Role::kAudio:
      return u"group";

    case ax::mojom::Role::kBanner:
    case ax::mojom::Role::kHeader:
      return u"banner";

    case ax::mojom::Role::kBlockquote:
      return u"group";

    case ax::mojom::Role::kButton:
      return u"button";

    case ax::mojom::Role::kCanvas:
      return u"img";

    case ax::mojom::Role::kCaption:
      return u"description";

    case ax::mojom::Role::kCaret:
      return u"region";

    case ax::mojom::Role::kCell:
      return u"gridcell";

    case ax::mojom::Role::kCode:
      return u"code";

    case ax::mojom::Role::kCheckBox:
      return u"checkbox";

    case ax::mojom::Role::kClient:
      return u"region";

    case ax::mojom::Role::kColorWell:
      return u"textbox";

    case ax::mojom::Role::kColumn:
      return u"region";

    case ax::mojom::Role::kColumnHeader:
      return u"columnheader";

    case ax::mojom::Role::kComboBoxGrouping:
    case ax::mojom::Role::kComboBoxMenuButton:
      return u"combobox";

    case ax::mojom::Role::kComplementary:
      return u"complementary";

    case ax::mojom::Role::kContentDeletion:
    case ax::mojom::Role::kContentInsertion:
      return u"group";

    case ax::mojom::Role::kContentInfo:
    case ax::mojom::Role::kFooter:
      return u"contentinfo";

    case ax::mojom::Role::kDate:
    case ax::mojom::Role::kDateTime:
      return u"textbox";

    case ax::mojom::Role::kDefinition:
      return u"definition";

    case ax::mojom::Role::kDescriptionListDetail:
      return u"description";

    case ax::mojom::Role::kDescriptionList:
      return u"list";

    case ax::mojom::Role::kDescriptionListTerm:
      return u"listitem";

    case ax::mojom::Role::kDesktop:
      return u"document";

    case ax::mojom::Role::kDetails:
      return u"group";

    case ax::mojom::Role::kDialog:
      return u"dialog";

    case ax::mojom::Role::kDisclosureTriangle:
      return u"button";

    case ax::mojom::Role::kDirectory:
      return u"directory";

    case ax::mojom::Role::kDocCover:
      return u"img";

    case ax::mojom::Role::kDocBackLink:
    case ax::mojom::Role::kDocBiblioRef:
    case ax::mojom::Role::kDocGlossRef:
    case ax::mojom::Role::kDocNoteRef:
      return u"link";

    case ax::mojom::Role::kDocBiblioEntry:
    case ax::mojom::Role::kDocEndnote:
    case ax::mojom::Role::kDocFootnote:
      return u"listitem";

    case ax::mojom::Role::kDocPageBreak:
      return u"separator";

    case ax::mojom::Role::kDocAbstract:
    case ax::mojom::Role::kDocAcknowledgments:
    case ax::mojom::Role::kDocAfterword:
    case ax::mojom::Role::kDocAppendix:
    case ax::mojom::Role::kDocBibliography:
    case ax::mojom::Role::kDocChapter:
    case ax::mojom::Role::kDocColophon:
    case ax::mojom::Role::kDocConclusion:
    case ax::mojom::Role::kDocCredit:
    case ax::mojom::Role::kDocCredits:
    case ax::mojom::Role::kDocDedication:
    case ax::mojom::Role::kDocEndnotes:
    case ax::mojom::Role::kDocEpigraph:
    case ax::mojom::Role::kDocEpilogue:
    case ax::mojom::Role::kDocErrata:
    case ax::mojom::Role::kDocExample:
    case ax::mojom::Role::kDocForeword:
    case ax::mojom::Role::kDocGlossary:
    case ax::mojom::Role::kDocIndex:
    case ax::mojom::Role::kDocIntroduction:
    case ax::mojom::Role::kDocNotice:
    case ax::mojom::Role::kDocPageList:
    case ax::mojom::Role::kDocPart:
    case ax::mojom::Role::kDocPreface:
    case ax::mojom::Role::kDocPrologue:
    case ax::mojom::Role::kDocPullquote:
    case ax::mojom::Role::kDocQna:
    case ax::mojom::Role::kDocSubtitle:
    case ax::mojom::Role::kDocTip:
    case ax::mojom::Role::kDocToc:
      return u"group";

    case ax::mojom::Role::kDocument:
    case ax::mojom::Role::kRootWebArea:
    case ax::mojom::Role::kWebArea:
      return u"document";

    case ax::mojom::Role::kEmbeddedObject:
      if (GetDelegate()->GetChildCount()) {
        return u"group";
      } else {
        return u"document";
      }

    case ax::mojom::Role::kEmphasis:
      return u"emphasis";

    case ax::mojom::Role::kFeed:
      return u"group";

    case ax::mojom::Role::kFigcaption:
      return u"description";

    case ax::mojom::Role::kFigure:
      return u"group";

    case ax::mojom::Role::kFooterAsNonLandmark:
    case ax::mojom::Role::kHeaderAsNonLandmark:
      return u"group";

    case ax::mojom::Role::kForm:
      return u"form";

    case ax::mojom::Role::kGenericContainer:
      return u"group";

    case ax::mojom::Role::kGraphicsDocument:
      return u"document";

    case ax::mojom::Role::kGraphicsObject:
      return u"region";

    case ax::mojom::Role::kGraphicsSymbol:
      return u"img";

    case ax::mojom::Role::kGrid:
      return u"grid";

    case ax::mojom::Role::kGroup:
      return u"group";

    case ax::mojom::Role::kHeading:
      return u"heading";

    case ax::mojom::Role::kIframe:
      return u"document";

    case ax::mojom::Role::kIframePresentational:
      return u"group";

    case ax::mojom::Role::kImage:
      return u"img";

    case ax::mojom::Role::kImageMap:
      return u"document";

    case ax::mojom::Role::kImeCandidate:
      // Internal role, not used on Windows.
      return u"group";

    case ax::mojom::Role::kInputTime:
      return u"group";

    case ax::mojom::Role::kInlineTextBox:
      return u"textbox";

    case ax::mojom::Role::kKeyboard:
      return u"group";

    case ax::mojom::Role::kLabelText:
    case ax::mojom::Role::kLegend:
      return u"description";

    case ax::mojom::Role::kLayoutTable:
      return u"grid";

    case ax::mojom::Role::kLayoutTableCell:
      return u"gridcell";

    case ax::mojom::Role::kLayoutTableRow:
      return u"row";

    case ax::mojom::Role::kLink:
      return u"link";

    case ax::mojom::Role::kList:
      return u"list";

    case ax::mojom::Role::kListBox:
      return u"listbox";

    case ax::mojom::Role::kListBoxOption:
      return u"option";

    case ax::mojom::Role::kListGrid:
      return u"listview";

    case ax::mojom::Role::kListItem:
      return u"listitem";

    case ax::mojom::Role::kListMarker:
      if (!GetDelegate()->GetChildCount()) {
        // There's only a name attribute when using Legacy layout. With Legacy
        // layout, list markers have no child and are considered as StaticText.
        // We consider a list marker as a group in LayoutNG since it has
        // a text child node.
        return u"description";
      }
      return u"group";

    case ax::mojom::Role::kLog:
      return u"log";

    case ax::mojom::Role::kMain:
      return u"main";

    case ax::mojom::Role::kMark:
      return u"description";

    case ax::mojom::Role::kMarquee:
      return u"marquee";

    case ax::mojom::Role::kMath:
      return u"group";

    case ax::mojom::Role::kMenu:
      return u"menu";

    case ax::mojom::Role::kMenuBar:
      return u"menubar";

    case ax::mojom::Role::kMenuItem:
      return u"menuitem";

    case ax::mojom::Role::kMenuItemCheckBox:
      return u"menuitemcheckbox";

    case ax::mojom::Role::kMenuItemRadio:
      return u"menuitemradio";

    case ax::mojom::Role::kMenuListPopup:
      return u"list";

    case ax::mojom::Role::kMenuListOption:
      return u"listitem";

    case ax::mojom::Role::kMeter:
      return u"progressbar";

    case ax::mojom::Role::kNavigation:
      return u"navigation";

    case ax::mojom::Role::kNote:
      return u"note";

    case ax::mojom::Role::kParagraph:
      return u"group";

    case ax::mojom::Role::kPdfActionableHighlight:
      return u"button";

    case ax::mojom::Role::kPluginObject:
      if (GetDelegate()->GetChildCount()) {
        return u"group";
      } else {
        return u"document";
      }

    case ax::mojom::Role::kPopUpButton: {
      std::string html_tag =
          GetData().GetStringAttribute(ax::mojom::StringAttribute::kHtmlTag);
      if (html_tag == "select")
        return u"combobox";
      return u"button";
    }

    case ax::mojom::Role::kPortal:
      return u"button";

    case ax::mojom::Role::kPre:
      return u"region";

    case ax::mojom::Role::kProgressIndicator:
      return u"progressbar";

    case ax::mojom::Role::kRadioButton:
      return u"radio";

    case ax::mojom::Role::kRadioGroup:
      return u"radiogroup";

    case ax::mojom::Role::kRegion:
      return u"region";

    case ax::mojom::Role::kRow: {
      // Role changes depending on whether row is inside a treegrid
      // https://www.w3.org/TR/core-aam-1.1/#role-map-row
      return IsInTreeGrid() ? u"treeitem" : u"row";
    }

    case ax::mojom::Role::kRowGroup:
      return u"rowgroup";

    case ax::mojom::Role::kRowHeader:
      return u"rowheader";

    case ax::mojom::Role::kRuby:
      return u"region";

    case ax::mojom::Role::kSection: {
      if (GetNameAsString16().empty()) {
        // Do not use ARIA mapping for nameless <section>.
        return u"group";
      }
      // Use ARIA mapping.
      return u"region";
    }

    case ax::mojom::Role::kScrollBar:
      return u"scrollbar";

    case ax::mojom::Role::kScrollView:
      return u"region";

    case ax::mojom::Role::kSearch:
      return u"search";

    case ax::mojom::Role::kSlider:
      return u"slider";

    case ax::mojom::Role::kSliderThumb:
      return u"slider";

    case ax::mojom::Role::kSpinButton:
      return u"spinbutton";

    case ax::mojom::Role::kStrong:
      return u"strong";

    case ax::mojom::Role::kSwitch:
      return u"switch";

    case ax::mojom::Role::kRubyAnnotation:
    case ax::mojom::Role::kStaticText:
      return u"description";

    case ax::mojom::Role::kStatus:
      return u"status";

    case ax::mojom::Role::kSplitter:
      return u"separator";

    case ax::mojom::Role::kSvgRoot:
      return u"img";

    case ax::mojom::Role::kTab:
      return u"tab";

    case ax::mojom::Role::kTable:
      return u"grid";

    case ax::mojom::Role::kTableHeaderContainer:
      return u"group";

    case ax::mojom::Role::kTabList:
      return u"tablist";

    case ax::mojom::Role::kTabPanel:
      return u"tabpanel";

    case ax::mojom::Role::kTerm:
      return u"listitem";

    case ax::mojom::Role::kTitleBar:
      return u"document";

    case ax::mojom::Role::kToggleButton:
      return u"button";

    case ax::mojom::Role::kTextField:
      return u"textbox";

    case ax::mojom::Role::kSearchBox:
      return u"searchbox";

    case ax::mojom::Role::kTextFieldWithComboBox:
      return u"combobox";

    case ax::mojom::Role::kAbbr:
      return u"description";

    case ax::mojom::Role::kTime:
      return u"time";

    case ax::mojom::Role::kTimer:
      return u"timer";

    case ax::mojom::Role::kToolbar:
      return u"toolbar";

    case ax::mojom::Role::kTooltip:
      return u"tooltip";

    case ax::mojom::Role::kTree:
      return u"tree";

    case ax::mojom::Role::kTreeGrid:
      return u"treegrid";

    case ax::mojom::Role::kTreeItem:
      return u"treeitem";

    case ax::mojom::Role::kLineBreak:
      return u"separator";

    case ax::mojom::Role::kVideo:
      return u"group";

    case ax::mojom::Role::kWebView:
      return u"document";

    case ax::mojom::Role::kPane:
    case ax::mojom::Role::kWindow:
    case ax::mojom::Role::kIgnored:
    case ax::mojom::Role::kNone:
    case ax::mojom::Role::kPresentational:
    case ax::mojom::Role::kUnknown:
      return u"region";
  }

  BASE_UNREACHABLE();
  return u"document";
}

std::u16string AXPlatformNodeWin::ComputeUIAProperties() {
  std::vector<std::u16string> properties;
  const AXNodeData& data = GetData();

  BoolAttributeToUIAAriaProperty(
      properties, ax::mojom::BoolAttribute::kLiveAtomic, "atomic");
  BoolAttributeToUIAAriaProperty(properties, ax::mojom::BoolAttribute::kBusy,
                                 "busy");

  switch (data.GetCheckedState()) {
    case ax::mojom::CheckedState::kNone:
      break;
    case ax::mojom::CheckedState::kFalse:
      if (data.role == ax::mojom::Role::kToggleButton) {
        properties.emplace_back(u"pressed=false");
      } else if (data.role == ax::mojom::Role::kSwitch) {
        // ARIA switches are exposed to Windows accessibility as toggle
        // buttons. For maximum compatibility with ATs, we expose both the
        // pressed and checked states.
        properties.emplace_back(u"pressed=false");
        properties.emplace_back(u"checked=false");
      } else {
        properties.emplace_back(u"checked=false");
      }
      break;
    case ax::mojom::CheckedState::kTrue:
      if (data.role == ax::mojom::Role::kToggleButton) {
        properties.emplace_back(u"pressed=true");
      } else if (data.role == ax::mojom::Role::kSwitch) {
        // ARIA switches are exposed to Windows accessibility as toggle
        // buttons. For maximum compatibility with ATs, we expose both the
        // pressed and checked states.
        properties.emplace_back(u"pressed=true");
        properties.emplace_back(u"checked=true");
      } else {
        properties.emplace_back(u"checked=true");
      }
      break;
    case ax::mojom::CheckedState::kMixed:
      if (data.role == ax::mojom::Role::kToggleButton) {
        properties.emplace_back(u"pressed=mixed");
      } else if (data.role == ax::mojom::Role::kSwitch) {
        // This is disallowed both by the ARIA standard and by Blink.
        BASE_UNREACHABLE();
      } else {
        properties.emplace_back(u"checked=mixed");
      }
      break;
  }

  const auto restriction = static_cast<ax::mojom::Restriction>(
      GetIntAttribute(ax::mojom::IntAttribute::kRestriction));
  if (restriction == ax::mojom::Restriction::kDisabled) {
    properties.push_back(u"disabled=true");
  } else {
    // The readonly property is complex on Windows. We set "readonly=true"
    // on *some* document structure roles such as paragraph, heading or list
    // even if the node data isn't marked as read only, as long as the
    // node is not editable.
    if (GetData().IsReadOnlyOrDisabled())
      properties.push_back(u"readonly=true");
  }

  // aria-dropeffect is deprecated in WAI-ARIA 1.1.
  if (data.HasIntAttribute(ax::mojom::IntAttribute::kDropeffect)) {
    properties.push_back(u"dropeffect=" +
                         base::UTF8ToUTF16(data.DropeffectBitfieldToString()));
  }
  StateToUIAAriaProperty(properties, ax::mojom::State::kExpanded, "expanded");
  BoolAttributeToUIAAriaProperty(properties, ax::mojom::BoolAttribute::kGrabbed,
                                 "grabbed");

  switch (static_cast<ax::mojom::HasPopup>(
      data.GetIntAttribute(ax::mojom::IntAttribute::kHasPopup))) {
    case ax::mojom::HasPopup::kFalse:
      break;
    case ax::mojom::HasPopup::kTrue:
      properties.push_back(u"haspopup=true");
      break;
    case ax::mojom::HasPopup::kMenu:
      properties.push_back(u"haspopup=menu");
      break;
    case ax::mojom::HasPopup::kListbox:
      properties.push_back(u"haspopup=listbox");
      break;
    case ax::mojom::HasPopup::kTree:
      properties.push_back(u"haspopup=tree");
      break;
    case ax::mojom::HasPopup::kGrid:
      properties.push_back(u"haspopup=grid");
      break;
    case ax::mojom::HasPopup::kDialog:
      properties.push_back(u"haspopup=dialog");
      break;
  }

  if (IsInvisibleOrIgnored())
    properties.push_back(u"hidden=true");

  if (HasIntAttribute(ax::mojom::IntAttribute::kInvalidState) &&
      GetIntAttribute(ax::mojom::IntAttribute::kInvalidState) !=
          static_cast<int32_t>(ax::mojom::InvalidState::kFalse)) {
    properties.push_back(u"invalid=true");
  }

  IntAttributeToUIAAriaProperty(
      properties, ax::mojom::IntAttribute::kHierarchicalLevel, "level");
  StringAttributeToUIAAriaProperty(
      properties, ax::mojom::StringAttribute::kLiveStatus, "live");
  StateToUIAAriaProperty(properties, ax::mojom::State::kMultiline, "multiline");
  StateToUIAAriaProperty(properties, ax::mojom::State::kMultiselectable,
                         "multiselectable");
  IntAttributeToUIAAriaProperty(properties, ax::mojom::IntAttribute::kPosInSet,
                                "posinset");
  StringAttributeToUIAAriaProperty(
      properties, ax::mojom::StringAttribute::kLiveRelevant, "relevant");
  StateToUIAAriaProperty(properties, ax::mojom::State::kRequired, "required");
  BoolAttributeToUIAAriaProperty(
      properties, ax::mojom::BoolAttribute::kSelected, "selected");
  IntAttributeToUIAAriaProperty(properties, ax::mojom::IntAttribute::kSetSize,
                                "setsize");

  int32_t sort_direction;
  if (IsTableHeader(data.role) &&
      GetIntAttribute(ax::mojom::IntAttribute::kSortDirection,
                      &sort_direction)) {
    switch (static_cast<ax::mojom::SortDirection>(sort_direction)) {
      case ax::mojom::SortDirection::kNone:
        break;
      case ax::mojom::SortDirection::kUnsorted:
        properties.push_back(u"sort=none");
        break;
      case ax::mojom::SortDirection::kAscending:
        properties.push_back(u"sort=ascending");
        break;
      case ax::mojom::SortDirection::kDescending:
        properties.push_back(u"sort=descending");
        break;
      case ax::mojom::SortDirection::kOther:
        properties.push_back(u"sort=other");
        break;
    }
  }

  if (data.IsRangeValueSupported()) {
    FloatAttributeToUIAAriaProperty(
        properties, ax::mojom::FloatAttribute::kMaxValueForRange, "valuemax");
    FloatAttributeToUIAAriaProperty(
        properties, ax::mojom::FloatAttribute::kMinValueForRange, "valuemin");
    StringAttributeToUIAAriaProperty(
        properties, ax::mojom::StringAttribute::kValue, "valuetext");

    std::u16string value_now = GetRangeValueText();
    SanitizeStringAttributeForUIAAriaProperty(value_now, &value_now);
    if (!value_now.empty())
      properties.push_back(u"valuenow=" + value_now);
  }

  std::u16string result = base::JoinString(properties, u";");
  return result;
}

LONG AXPlatformNodeWin::ComputeUIAControlType() {  // NOLINT(runtime/int)
  // If this is a web area for a presentational iframe, give it a role of
  // something other than document so that the fact that it's a separate doc
  // is not exposed to AT.
  if (IsWebAreaForPresentationalIframe())
    return UIA_GroupControlTypeId;

  switch (GetData().role) {
    case ax::mojom::Role::kAlert:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kAlertDialog:
      // Our MSAA implementation suggests the use of
      // |UIA_TextControlTypeId|, not |UIA_PaneControlTypeId| because some
      // Windows screen readers are not compatible with
      // |ax::mojom::Role::kAlertDialog| yet.
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kAnchor:
      return UIA_HyperlinkControlTypeId;

    case ax::mojom::Role::kComment:
    case ax::mojom::Role::kSuggestion:
      return ROLE_SYSTEM_GROUPING;

    case ax::mojom::Role::kApplication:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kArticle:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kAudio:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kBanner:
    case ax::mojom::Role::kHeader:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kBlockquote:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kButton:
      return UIA_ButtonControlTypeId;

    case ax::mojom::Role::kCanvas:
      return UIA_ImageControlTypeId;

    case ax::mojom::Role::kCaption:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kCaret:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kCell:
      return UIA_DataItemControlTypeId;

    case ax::mojom::Role::kCheckBox:
      return UIA_CheckBoxControlTypeId;

    case ax::mojom::Role::kClient:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kCode:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kColorWell:
      return UIA_ButtonControlTypeId;

    case ax::mojom::Role::kColumn:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kColumnHeader:
      return UIA_DataItemControlTypeId;

    case ax::mojom::Role::kComboBoxGrouping:
    case ax::mojom::Role::kComboBoxMenuButton:
      return UIA_ComboBoxControlTypeId;

    case ax::mojom::Role::kComplementary:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kContentDeletion:
    case ax::mojom::Role::kContentInsertion:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kContentInfo:
    case ax::mojom::Role::kFooter:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kDate:
    case ax::mojom::Role::kDateTime:
      return UIA_EditControlTypeId;

    case ax::mojom::Role::kDefinition:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kDescriptionListDetail:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kDescriptionList:
      return UIA_ListControlTypeId;

    case ax::mojom::Role::kDescriptionListTerm:
      return UIA_ListItemControlTypeId;

    case ax::mojom::Role::kDesktop:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kDetails:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kDialog:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kDisclosureTriangle:
      return UIA_ButtonControlTypeId;

    case ax::mojom::Role::kDirectory:
      return UIA_ListControlTypeId;

    case ax::mojom::Role::kDocCover:
      return UIA_ImageControlTypeId;

    case ax::mojom::Role::kDocBackLink:
    case ax::mojom::Role::kDocBiblioRef:
    case ax::mojom::Role::kDocGlossRef:
    case ax::mojom::Role::kDocNoteRef:
      return UIA_HyperlinkControlTypeId;

    case ax::mojom::Role::kDocBiblioEntry:
    case ax::mojom::Role::kDocEndnote:
    case ax::mojom::Role::kDocFootnote:
      return UIA_ListItemControlTypeId;

    case ax::mojom::Role::kDocPageBreak:
      return UIA_SeparatorControlTypeId;

    case ax::mojom::Role::kDocAbstract:
    case ax::mojom::Role::kDocAcknowledgments:
    case ax::mojom::Role::kDocAfterword:
    case ax::mojom::Role::kDocAppendix:
    case ax::mojom::Role::kDocBibliography:
    case ax::mojom::Role::kDocChapter:
    case ax::mojom::Role::kDocColophon:
    case ax::mojom::Role::kDocConclusion:
    case ax::mojom::Role::kDocCredit:
    case ax::mojom::Role::kDocCredits:
    case ax::mojom::Role::kDocDedication:
    case ax::mojom::Role::kDocEndnotes:
    case ax::mojom::Role::kDocEpigraph:
    case ax::mojom::Role::kDocEpilogue:
    case ax::mojom::Role::kDocErrata:
    case ax::mojom::Role::kDocExample:
    case ax::mojom::Role::kDocForeword:
    case ax::mojom::Role::kDocGlossary:
    case ax::mojom::Role::kDocIndex:
    case ax::mojom::Role::kDocIntroduction:
    case ax::mojom::Role::kDocNotice:
    case ax::mojom::Role::kDocPageList:
    case ax::mojom::Role::kDocPart:
    case ax::mojom::Role::kDocPreface:
    case ax::mojom::Role::kDocPrologue:
    case ax::mojom::Role::kDocPullquote:
    case ax::mojom::Role::kDocQna:
    case ax::mojom::Role::kDocSubtitle:
    case ax::mojom::Role::kDocTip:
    case ax::mojom::Role::kDocToc:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kDocument:
    case ax::mojom::Role::kRootWebArea:
    case ax::mojom::Role::kWebArea:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kEmbeddedObject:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kEmphasis:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kFeed:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kFigcaption:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kFigure:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kFooterAsNonLandmark:
    case ax::mojom::Role::kHeaderAsNonLandmark:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kForm:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kGenericContainer:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kGraphicsDocument:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kGraphicsObject:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kGraphicsSymbol:
      return UIA_ImageControlTypeId;

    case ax::mojom::Role::kGrid:
      return UIA_DataGridControlTypeId;

    case ax::mojom::Role::kGroup:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kHeading:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kIframe:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kIframePresentational:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kImage:
      return UIA_ImageControlTypeId;

    case ax::mojom::Role::kImageMap:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kInputTime:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kInlineTextBox:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kLabelText:
    case ax::mojom::Role::kLegend:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kLayoutTable:
      return UIA_TableControlTypeId;

    case ax::mojom::Role::kLayoutTableCell:
      return UIA_DataItemControlTypeId;

    case ax::mojom::Role::kLayoutTableRow:
      return UIA_DataItemControlTypeId;

    case ax::mojom::Role::kLink:
      return UIA_HyperlinkControlTypeId;

    case ax::mojom::Role::kList:
      return UIA_ListControlTypeId;

    case ax::mojom::Role::kListBox:
      return UIA_ListControlTypeId;

    case ax::mojom::Role::kListBoxOption:
      return UIA_ListItemControlTypeId;

    case ax::mojom::Role::kListGrid:
      return UIA_DataGridControlTypeId;

    case ax::mojom::Role::kListItem:
      return UIA_ListItemControlTypeId;

    case ax::mojom::Role::kListMarker:
      if (!GetDelegate()->GetChildCount()) {
        // There's only a name attribute when using Legacy layout. With Legacy
        // layout, list markers have no child and are considered as StaticText.
        // We consider a list marker as a group in LayoutNG since it has
        // a text child node.
        return UIA_TextControlTypeId;
      }
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kLog:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kMain:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kMark:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kMarquee:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kMath:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kMenu:
      return UIA_MenuControlTypeId;

    case ax::mojom::Role::kMenuBar:
      return UIA_MenuBarControlTypeId;

    case ax::mojom::Role::kMenuItem:
      return UIA_MenuItemControlTypeId;

    case ax::mojom::Role::kMenuItemCheckBox:
      return UIA_CheckBoxControlTypeId;

    case ax::mojom::Role::kMenuItemRadio:
      return UIA_RadioButtonControlTypeId;

    case ax::mojom::Role::kMenuListPopup:
      return UIA_ListControlTypeId;

    case ax::mojom::Role::kMenuListOption:
      return UIA_ListItemControlTypeId;

    case ax::mojom::Role::kMeter:
      return UIA_ProgressBarControlTypeId;

    case ax::mojom::Role::kNavigation:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kNote:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kParagraph:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kPdfActionableHighlight:
      return UIA_CustomControlTypeId;

    case ax::mojom::Role::kPluginObject:
      if (GetDelegate()->GetChildCount()) {
        return UIA_GroupControlTypeId;
      } else {
        return UIA_DocumentControlTypeId;
      }

    case ax::mojom::Role::kPopUpButton: {
      std::string html_tag =
          GetData().GetStringAttribute(ax::mojom::StringAttribute::kHtmlTag);
      if (html_tag == "select")
        return UIA_ComboBoxControlTypeId;
      return UIA_ButtonControlTypeId;
    }

    case ax::mojom::Role::kPortal:
      return UIA_ButtonControlTypeId;

    case ax::mojom::Role::kPre:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kProgressIndicator:
      return UIA_ProgressBarControlTypeId;

    case ax::mojom::Role::kRadioButton:
      return UIA_RadioButtonControlTypeId;

    case ax::mojom::Role::kRadioGroup:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kRegion:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kRow: {
      // Role changes depending on whether row is inside a treegrid
      // https://www.w3.org/TR/core-aam-1.1/#role-map-row
      return IsInTreeGrid() ? UIA_TreeItemControlTypeId
                            : UIA_DataItemControlTypeId;
    }

    case ax::mojom::Role::kRowGroup:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kRowHeader:
      return UIA_DataItemControlTypeId;

    case ax::mojom::Role::kRuby:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kSection:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kScrollBar:
      return UIA_ScrollBarControlTypeId;

    case ax::mojom::Role::kScrollView:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kSearch:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kSlider:
      return UIA_SliderControlTypeId;

    case ax::mojom::Role::kSliderThumb:
      return UIA_SliderControlTypeId;

    case ax::mojom::Role::kSpinButton:
      return UIA_SpinnerControlTypeId;

    case ax::mojom::Role::kSwitch:
      return UIA_ButtonControlTypeId;

    case ax::mojom::Role::kRubyAnnotation:
    case ax::mojom::Role::kStaticText:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kStatus:
      return UIA_StatusBarControlTypeId;

    case ax::mojom::Role::kStrong:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kSplitter:
      return UIA_SeparatorControlTypeId;

    case ax::mojom::Role::kSvgRoot:
      return UIA_ImageControlTypeId;

    case ax::mojom::Role::kTab:
      return UIA_TabItemControlTypeId;

    case ax::mojom::Role::kTable:
      return UIA_TableControlTypeId;

    case ax::mojom::Role::kTableHeaderContainer:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kTabList:
      return UIA_TabControlTypeId;

    case ax::mojom::Role::kTabPanel:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kTerm:
      return UIA_ListItemControlTypeId;

    case ax::mojom::Role::kTitleBar:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kToggleButton:
      return UIA_ButtonControlTypeId;

    case ax::mojom::Role::kTextField:
    case ax::mojom::Role::kSearchBox:
      return UIA_EditControlTypeId;

    case ax::mojom::Role::kTextFieldWithComboBox:
      return UIA_ComboBoxControlTypeId;

    case ax::mojom::Role::kAbbr:
    case ax::mojom::Role::kTime:
      return UIA_TextControlTypeId;

    case ax::mojom::Role::kTimer:
      return UIA_PaneControlTypeId;

    case ax::mojom::Role::kToolbar:
      return UIA_ToolBarControlTypeId;

    case ax::mojom::Role::kTooltip:
      return UIA_ToolTipControlTypeId;

    case ax::mojom::Role::kTree:
      return UIA_TreeControlTypeId;

    case ax::mojom::Role::kTreeGrid:
      return UIA_DataGridControlTypeId;

    case ax::mojom::Role::kTreeItem:
      return UIA_TreeItemControlTypeId;

    case ax::mojom::Role::kLineBreak:
      return UIA_SeparatorControlTypeId;

    case ax::mojom::Role::kVideo:
      return UIA_GroupControlTypeId;

    case ax::mojom::Role::kWebView:
      return UIA_DocumentControlTypeId;

    case ax::mojom::Role::kPane:
    case ax::mojom::Role::kWindow:
    case ax::mojom::Role::kIgnored:
    case ax::mojom::Role::kImeCandidate:
    case ax::mojom::Role::kKeyboard:
    case ax::mojom::Role::kNone:
    case ax::mojom::Role::kPresentational:
    case ax::mojom::Role::kUnknown:
      return UIA_PaneControlTypeId;
  }

  BASE_UNREACHABLE();
  return UIA_DocumentControlTypeId;
}

AXPlatformNodeWin* AXPlatformNodeWin::ComputeUIALabeledBy() {
  // Not all control types expect a value for this property.
  if (!CanHaveUIALabeledBy())
    return nullptr;

  // This property only accepts static text elements to be returned. Find the
  // first static text used to label this node.
  for (int32_t id : GetData().GetIntListAttribute(
           ax::mojom::IntListAttribute::kLabelledbyIds)) {
    auto* node_win =
        static_cast<AXPlatformNodeWin*>(GetDelegate()->GetFromNodeID(id));
    if (!node_win)
      continue;

    // If this node is a static text, then simply return the node itself.
    if (IsValidUiaRelationTarget(node_win) &&
        node_win->GetData().role == ax::mojom::Role::kStaticText) {
      return node_win;
    }

    // Otherwise, find the first static text node in its descendants.
    for (auto iter = node_win->GetDelegate()->ChildrenBegin();
         *iter != *node_win->GetDelegate()->ChildrenEnd(); ++(*iter)) {
      AXPlatformNodeWin* child = static_cast<AXPlatformNodeWin*>(
          AXPlatformNode::FromNativeViewAccessible(
              iter->GetNativeViewAccessible()));
      if (IsValidUiaRelationTarget(child) &&
          child->GetData().role == ax::mojom::Role::kStaticText) {
        return child;
      }
    }
  }

  return nullptr;
}

bool AXPlatformNodeWin::CanHaveUIALabeledBy() {
  // Not all control types expect a value for this property. See
  // https://docs.microsoft.com/en-us/windows/win32/winauto/uiauto-supportinguiautocontroltypes
  // for a complete list of control types. Each one of them has specific
  // expectations regarding the UIA_LabeledByPropertyId.
  switch (ComputeUIAControlType()) {
    case UIA_ButtonControlTypeId:
    case UIA_CheckBoxControlTypeId:
    case UIA_DataItemControlTypeId:
    case UIA_MenuControlTypeId:
    case UIA_MenuBarControlTypeId:
    case UIA_RadioButtonControlTypeId:
    case UIA_ScrollBarControlTypeId:
    case UIA_SeparatorControlTypeId:
    case UIA_StatusBarControlTypeId:
    case UIA_TabItemControlTypeId:
    case UIA_TextControlTypeId:
    case UIA_ToolBarControlTypeId:
    case UIA_ToolTipControlTypeId:
    case UIA_TreeItemControlTypeId:
      return false;
    default:
      return true;
  }
}

bool AXPlatformNodeWin::IsNameExposed() const {
  const AXNodeData& data = GetData();
  switch (data.role) {
    case ax::mojom::Role::kListMarker:
      return !GetDelegate()->GetChildCount();
    default:
      return true;
  }
}

bool AXPlatformNodeWin::IsUIAControl() const {
  // UIA provides multiple "views": raw, content and control. We only want to
  // populate the content and control views with items that make sense to
  // traverse over.

  if (GetDelegate()->IsWebContent()) {
    // Invisible or ignored elements should not show up in control view at all.
    if (IsInvisibleOrIgnored())
      return false;

    if (IsText()) {
      // A text leaf can be a UIAControl, but text inside of a heading, link,
      // button, etc. where the role allows the name to be generated from the
      // content is not. We want to avoid reading out a button, moving to the
      // next item, and then reading out the button's text child, causing the
      // text to be effectively repeated.
      auto* parent = FromNativeViewAccessible(GetDelegate()->GetParent());
      while (parent) {
        const AXNodeData& data = parent->GetData();
        if (IsCellOrTableHeader(data.role))
          return false;
        switch (data.role) {
          case ax::mojom::Role::kButton:
          case ax::mojom::Role::kCheckBox:
          case ax::mojom::Role::kGroup:
          case ax::mojom::Role::kHeading:
          case ax::mojom::Role::kLineBreak:
          case ax::mojom::Role::kLink:
          case ax::mojom::Role::kListBoxOption:
          case ax::mojom::Role::kListItem:
          case ax::mojom::Role::kMenuItem:
          case ax::mojom::Role::kMenuItemCheckBox:
          case ax::mojom::Role::kMenuItemRadio:
          case ax::mojom::Role::kMenuListOption:
          case ax::mojom::Role::kPdfActionableHighlight:
          case ax::mojom::Role::kRadioButton:
          case ax::mojom::Role::kRow:
          case ax::mojom::Role::kRowGroup:
          case ax::mojom::Role::kStaticText:
          case ax::mojom::Role::kSwitch:
          case ax::mojom::Role::kTab:
          case ax::mojom::Role::kTooltip:
          case ax::mojom::Role::kTreeItem:
            return false;
          default:
            break;
        }
        parent = FromNativeViewAccessible(parent->GetParent());
      }
    }  // end of text only case.

    const AXNodeData& data = GetData();
    // https://docs.microsoft.com/en-us/windows/win32/winauto/uiauto-treeoverview#control-view
    // The control view also includes noninteractive UI items that contribute
    // to the logical structure of the UI.
    if (IsControl(data.role) || ComputeUIALandmarkType() ||
        IsTableLike(data.role) || IsList(data.role)) {
      return true;
    }
    if (IsImage(data.role)) {
      // If the author provides an explicitly empty alt text attribute then
      // the image is decorational and should not be considered as a control.
      if (data.role == ax::mojom::Role::kImage &&
          data.GetNameFrom() ==
              ax::mojom::NameFrom::kAttributeExplicitlyEmpty) {
        return false;
      }
      return true;
    }
    switch (data.role) {
      case ax::mojom::Role::kArticle:
      case ax::mojom::Role::kBlockquote:
      case ax::mojom::Role::kDetails:
      case ax::mojom::Role::kFigure:
      case ax::mojom::Role::kFooter:
      case ax::mojom::Role::kFooterAsNonLandmark:
      case ax::mojom::Role::kHeader:
      case ax::mojom::Role::kHeaderAsNonLandmark:
      case ax::mojom::Role::kLabelText:
      case ax::mojom::Role::kListBoxOption:
      case ax::mojom::Role::kListItem:
      case ax::mojom::Role::kMeter:
      case ax::mojom::Role::kProgressIndicator:
      case ax::mojom::Role::kSection:
      case ax::mojom::Role::kSplitter:
      case ax::mojom::Role::kTime:
        return true;
      default:
        break;
    }
    // Classify generic containers that are not clickable or focusable and have
    // no name, description, landmark type, and is not the root of editable
    // content as not controls.
    // Doing so helps Narrator find all the content of live regions.
    if (!data.GetBoolAttribute(ax::mojom::BoolAttribute::kHasAriaAttribute) &&
        !data.GetBoolAttribute(ax::mojom::BoolAttribute::kEditableRoot) &&
        GetNameAsString16().empty() &&
        data.GetStringAttribute(ax::mojom::StringAttribute::kDescription)
            .empty() &&
        !data.HasState(ax::mojom::State::kFocusable) && !data.IsClickable()) {
      return false;
    }

    return true;
  }  // end of web-content only case.

  const AXNodeData& data = GetData();
  return !((IsReadOnlySupported(data.role) && data.IsReadOnlyOrDisabled()) ||
           data.HasState(ax::mojom::State::kInvisible) ||
           (data.IsIgnored() && !data.HasState(ax::mojom::State::kFocusable)));
}

std::optional<LONG> AXPlatformNodeWin::ComputeUIALandmarkType() const {
  const AXNodeData& data = GetData();
  switch (data.role) {
    case ax::mojom::Role::kBanner:
    case ax::mojom::Role::kComplementary:
    case ax::mojom::Role::kContentInfo:
    case ax::mojom::Role::kFooter:
    case ax::mojom::Role::kHeader:
      return UIA_CustomLandmarkTypeId;

    case ax::mojom::Role::kForm:
      // https://www.w3.org/TR/html-aam-1.0/#html-element-role-mappings
      // https://w3c.github.io/core-aam/#mapping_role_table
      // While the HTML-AAM spec states that <form> without an accessible name
      // should have no corresponding role, removing the role breaks both
      // aria-setsize and aria-posinset.
      // The only other difference for UIA is that it should not be a landmark.
      // If the author provided an accessible name, or the role was explicit,
      // then allow the form landmark.
      if (data.HasStringAttribute(ax::mojom::StringAttribute::kName) ||
          data.HasStringAttribute(ax::mojom::StringAttribute::kRole)) {
        return UIA_FormLandmarkTypeId;
      }
      return {};

    case ax::mojom::Role::kMain:
      return UIA_MainLandmarkTypeId;

    case ax::mojom::Role::kNavigation:
      return UIA_NavigationLandmarkTypeId;

    case ax::mojom::Role::kSearch:
      return UIA_SearchLandmarkTypeId;

    case ax::mojom::Role::kRegion:
    case ax::mojom::Role::kSection:
      if (data.HasStringAttribute(ax::mojom::StringAttribute::kName))
        return UIA_CustomLandmarkTypeId;
      BASE_FALLTHROUGH;

    default:
      return {};
  }
}

bool AXPlatformNodeWin::IsInaccessibleDueToAncestor() const {
  AXPlatformNodeWin* parent = static_cast<AXPlatformNodeWin*>(
      AXPlatformNode::FromNativeViewAccessible(GetParent()));
  while (parent) {
    if (parent->ShouldHideChildrenForUIA())
      return true;
    parent = static_cast<AXPlatformNodeWin*>(
        FromNativeViewAccessible(parent->GetParent()));
  }
  return false;
}

bool AXPlatformNodeWin::ShouldHideChildrenForUIA() const {
  if (IsPlainTextField())
    return true;

  auto role = GetData().role;
  if (HasPresentationalChildren(role))
    return true;

  switch (role) {
    // Other elements that are expected by UIA to hide their children without
    // having "Children Presentational: True".
    //
    // TODO(bebeaudr): We might be able to remove ax::mojom::Role::kLink once
    // http://crbug.com/1054514 is fixed. Links should not have to hide their
    // children.
    // TODO(virens): |kPdfActionableHighlight| needs to follow a fix similar to
    // links. At present Pdf highlghts have text nodes as children. But, we may
    // enable pdf highlights to have complex children like links based on user
    // feedback.
    case ax::mojom::Role::kLink:
      // Links with a single text-only child should hide their subtree.
      if (GetChildCount() == 1) {
        AXPlatformNodeBase* only_child = GetFirstChild();
        return only_child && only_child->IsText();
      }
      return false;
    case ax::mojom::Role::kPdfActionableHighlight:
      return true;
    default:
      return false;
  }
}

std::u16string AXPlatformNodeWin::GetValue() const {
  std::u16string value = AXPlatformNodeBase::GetValue();

  // If this doesn't have a value and is linked then set its value to the URL
  // attribute. This allows screen readers to read an empty link's
  // destination.
  // TODO(dougt): Look into ensuring that on click handlers correctly provide
  // a value here.
  if (value.empty() && (MSAAState() & STATE_SYSTEM_LINKED))
    value = GetString16Attribute(ax::mojom::StringAttribute::kUrl);

  return value;
}

bool AXPlatformNodeWin::IsPlatformCheckable() const {
  if (GetData().role == ax::mojom::Role::kToggleButton)
    return false;

  return AXPlatformNodeBase::IsPlatformCheckable();
}

bool AXPlatformNodeWin::ShouldNodeHaveFocusableState(
    const AXNodeData& data) const {
  switch (data.role) {
    case ax::mojom::Role::kDocument:
    case ax::mojom::Role::kGraphicsDocument:
    case ax::mojom::Role::kWebArea:
      return true;

    case ax::mojom::Role::kRootWebArea: {
      AXPlatformNodeBase* parent = FromNativeViewAccessible(GetParent());
      return !parent || parent->GetData().role != ax::mojom::Role::kPortal;
    }

    case ax::mojom::Role::kIframe:
      return false;

    case ax::mojom::Role::kListBoxOption:
    case ax::mojom::Role::kMenuListOption:
      if (data.HasBoolAttribute(ax::mojom::BoolAttribute::kSelected))
        return true;
      break;

    default:
      break;
  }

  return data.HasState(ax::mojom::State::kFocusable);
}

int AXPlatformNodeWin::MSAAState() const {
  const AXNodeData& data = GetData();
  int msaa_state = 0;

  // Map the ax::mojom::State to MSAA state. Note that some of the states are
  // not currently handled.

  if (data.GetBoolAttribute(ax::mojom::BoolAttribute::kBusy))
    msaa_state |= STATE_SYSTEM_BUSY;

  if (data.HasState(ax::mojom::State::kCollapsed))
    msaa_state |= STATE_SYSTEM_COLLAPSED;

  if (data.HasState(ax::mojom::State::kDefault))
    msaa_state |= STATE_SYSTEM_DEFAULT;

  // TODO(dougt) unhandled ux::ax::mojom::State::kEditable

  if (data.HasState(ax::mojom::State::kExpanded))
    msaa_state |= STATE_SYSTEM_EXPANDED;

  if (ShouldNodeHaveFocusableState(data))
    msaa_state |= STATE_SYSTEM_FOCUSABLE;

  // Built-in autofill and autocomplete wil also set has popup.
  if (data.HasIntAttribute(ax::mojom::IntAttribute::kHasPopup))
    msaa_state |= STATE_SYSTEM_HASPOPUP;

  // TODO(dougt) unhandled ux::ax::mojom::State::kHorizontal

  if (data.HasState(ax::mojom::State::kHovered)) {
    // Expose whether or not the mouse is over an element, but suppress
    // this for tests because it can make the test results flaky depending
    // on the position of the mouse.
    if (GetDelegate()->ShouldIgnoreHoveredStateForTesting())
      msaa_state |= STATE_SYSTEM_HOTTRACKED;
  }

  // If the role is IGNORED, we want these elements to be invisible so that
  // these nodes are hidden from the screen reader.
  if (IsInvisibleOrIgnored())
    msaa_state |= STATE_SYSTEM_INVISIBLE;

  if (data.HasState(ax::mojom::State::kLinked))
    msaa_state |= STATE_SYSTEM_LINKED;

  // TODO(dougt) unhandled ux::ax::mojom::State::kMultiline

  if (data.HasState(ax::mojom::State::kMultiselectable)) {
    msaa_state |= STATE_SYSTEM_EXTSELECTABLE;
    msaa_state |= STATE_SYSTEM_MULTISELECTABLE;
  }

  if (GetDelegate()->IsOffscreen())
    msaa_state |= STATE_SYSTEM_OFFSCREEN;

  if (data.HasState(ax::mojom::State::kProtected))
    msaa_state |= STATE_SYSTEM_PROTECTED;

  // TODO(dougt) unhandled ux::ax::mojom::State::kRequired
  // TODO(dougt) unhandled ux::ax::mojom::State::kRichlyEditable

  if (data.IsSelectable())
    msaa_state |= STATE_SYSTEM_SELECTABLE;

  if (data.GetBoolAttribute(ax::mojom::BoolAttribute::kSelected))
    msaa_state |= STATE_SYSTEM_SELECTED;

  // TODO(dougt) unhandled VERTICAL

  if (data.HasState(ax::mojom::State::kVisited))
    msaa_state |= STATE_SYSTEM_TRAVERSED;

  //
  // Checked state
  //

  switch (data.GetCheckedState()) {
    case ax::mojom::CheckedState::kNone:
    case ax::mojom::CheckedState::kFalse:
      break;
    case ax::mojom::CheckedState::kTrue:
      if (data.role == ax::mojom::Role::kToggleButton) {
        msaa_state |= STATE_SYSTEM_PRESSED;
      } else if (data.role == ax::mojom::Role::kSwitch) {
        // ARIA switches are exposed to Windows accessibility as toggle
        // buttons. For maximum compatibility with ATs, we expose both the
        // pressed and checked states.
        msaa_state |= STATE_SYSTEM_PRESSED | STATE_SYSTEM_CHECKED;
      } else {
        msaa_state |= STATE_SYSTEM_CHECKED;
      }
      break;
    case ax::mojom::CheckedState::kMixed:
      msaa_state |= STATE_SYSTEM_MIXED;
      break;
  }

  const auto restriction = static_cast<ax::mojom::Restriction>(
      GetIntAttribute(ax::mojom::IntAttribute::kRestriction));
  switch (restriction) {
    case ax::mojom::Restriction::kDisabled:
      msaa_state |= STATE_SYSTEM_UNAVAILABLE;
      break;
    case ax::mojom::Restriction::kReadOnly:
      msaa_state |= STATE_SYSTEM_READONLY;
      break;
    default:
      // READONLY state is complex on Windows.  We set STATE_SYSTEM_READONLY
      // on *some* document structure roles such as paragraph, heading or list
      // even if the node data isn't marked as read only, as long as the
      // node is not editable.
      if (!data.HasState(ax::mojom::State::kRichlyEditable) &&
          ShouldHaveReadonlyStateByDefault(data.role)) {
        msaa_state |= STATE_SYSTEM_READONLY;
      }
      break;
  }

  // Windowless plugins should have STATE_SYSTEM_UNAVAILABLE.
  //
  // (All of our plugins are windowless.)
  if (data.role == ax::mojom::Role::kPluginObject ||
      data.role == ax::mojom::Role::kEmbeddedObject) {
    msaa_state |= STATE_SYSTEM_UNAVAILABLE;
  }

  //
  // Handle STATE_SYSTEM_FOCUSED
  //
  gfx::NativeViewAccessible focus = GetDelegate()->GetFocus();
  if (focus == const_cast<AXPlatformNodeWin*>(this)->GetNativeViewAccessible())
    msaa_state |= STATE_SYSTEM_FOCUSED;

  // In focused single selection UI menus and listboxes, mirror item selection
  // to focus. This helps NVDA read the selected option as it changes.
  if ((data.role == ax::mojom::Role::kListBoxOption || IsMenuItem(data.role)) &&
      data.GetBoolAttribute(ax::mojom::BoolAttribute::kSelected)) {
    AXPlatformNodeBase* container = FromNativeViewAccessible(GetParent());
    if (container && container->GetParent() == focus) {
      AXNodeData container_data = container->GetData();
      if ((container_data.role == ax::mojom::Role::kListBox ||
           container_data.role == ax::mojom::Role::kMenu) &&
          !container_data.HasState(ax::mojom::State::kMultiselectable)) {
        msaa_state |= STATE_SYSTEM_FOCUSED;
      }
    }
  }

  // On Windows, the "focus" bit should be set on certain containers, like
  // menu bars, when visible.
  //
  // TODO(dmazzoni): this should probably check if focus is actually inside
  // the menu bar, but we don't currently track focus inside menu pop-ups,
  // and Chrome only has one menu visible at a time so this works for now.
  if (data.role == ax::mojom::Role::kMenuBar &&
      !(data.HasState(ax::mojom::State::kInvisible))) {
    msaa_state |= STATE_SYSTEM_FOCUSED;
  }

  // Handle STATE_SYSTEM_LINKED
  if (GetData().role == ax::mojom::Role::kLink)
    msaa_state |= STATE_SYSTEM_LINKED;

  // Special case for indeterminate progressbar.
  if (GetData().role == ax::mojom::Role::kProgressIndicator &&
      !HasFloatAttribute(ax::mojom::FloatAttribute::kValueForRange))
    msaa_state |= STATE_SYSTEM_MIXED;

  return msaa_state;
}

// static
std::optional<DWORD> AXPlatformNodeWin::MojoEventToMSAAEvent(
    ax::mojom::Event event) {
  switch (event) {
    case ax::mojom::Event::kAlert:
      return EVENT_SYSTEM_ALERT;
    case ax::mojom::Event::kCheckedStateChanged:
    case ax::mojom::Event::kExpandedChanged:
    case ax::mojom::Event::kStateChanged:
      return EVENT_OBJECT_STATECHANGE;
    case ax::mojom::Event::kFocus:
    case ax::mojom::Event::kFocusContext:
      return EVENT_OBJECT_FOCUS;
    case ax::mojom::Event::kLiveRegionChanged:
      return EVENT_OBJECT_LIVEREGIONCHANGED;
    case ax::mojom::Event::kMenuStart:
      return EVENT_SYSTEM_MENUSTART;
    case ax::mojom::Event::kMenuEnd:
      return EVENT_SYSTEM_MENUEND;
    case ax::mojom::Event::kMenuPopupStart:
      return EVENT_SYSTEM_MENUPOPUPSTART;
    case ax::mojom::Event::kMenuPopupEnd:
      return EVENT_SYSTEM_MENUPOPUPEND;
    case ax::mojom::Event::kSelection:
      return EVENT_OBJECT_SELECTION;
    case ax::mojom::Event::kSelectionAdd:
      return EVENT_OBJECT_SELECTIONADD;
    case ax::mojom::Event::kSelectionRemove:
      return EVENT_OBJECT_SELECTIONREMOVE;
    case ax::mojom::Event::kTextChanged:
      return EVENT_OBJECT_NAMECHANGE;
    case ax::mojom::Event::kTooltipClosed:
      return EVENT_OBJECT_HIDE;
    case ax::mojom::Event::kTooltipOpened:
      return EVENT_OBJECT_SHOW;
    case ax::mojom::Event::kValueChanged:
      return EVENT_OBJECT_VALUECHANGE;
    case ax::mojom::Event::kDocumentSelectionChanged:
      return EVENT_OBJECT_TEXTSELECTIONCHANGED;
    default:
      return std::nullopt;
  }
}

// static
std::optional<EVENTID> AXPlatformNodeWin::MojoEventToUIAEvent(
    ax::mojom::Event event) {
  switch (event) {
    case ax::mojom::Event::kAlert:
      return UIA_SystemAlertEventId;
    case ax::mojom::Event::kDocumentSelectionChanged:
      return UIA_Text_TextChangedEventId;
    case ax::mojom::Event::kFocus:
    case ax::mojom::Event::kFocusContext:
    case ax::mojom::Event::kFocusAfterMenuClose:
      return UIA_AutomationFocusChangedEventId;
    case ax::mojom::Event::kLiveRegionChanged:
      return UIA_LiveRegionChangedEventId;
    case ax::mojom::Event::kSelection:
      return UIA_SelectionItem_ElementSelectedEventId;
    case ax::mojom::Event::kSelectionAdd:
      return UIA_SelectionItem_ElementAddedToSelectionEventId;
    case ax::mojom::Event::kSelectionRemove:
      return UIA_SelectionItem_ElementRemovedFromSelectionEventId;
    case ax::mojom::Event::kTooltipClosed:
      return UIA_ToolTipClosedEventId;
    case ax::mojom::Event::kTooltipOpened:
      return UIA_ToolTipOpenedEventId;
    default:
      return std::nullopt;
  }
}

std::optional<PROPERTYID> AXPlatformNodeWin::MojoEventToUIAProperty(
    ax::mojom::Event event) {
  switch (event) {
    case ax::mojom::Event::kControlsChanged:
      return UIA_ControllerForPropertyId;
    case ax::mojom::Event::kCheckedStateChanged:
      return UIA_ToggleToggleStatePropertyId;
    case ax::mojom::Event::kRowCollapsed:
    case ax::mojom::Event::kRowExpanded:
      return UIA_ExpandCollapseExpandCollapseStatePropertyId;
    case ax::mojom::Event::kSelection:
    case ax::mojom::Event::kSelectionAdd:
    case ax::mojom::Event::kSelectionRemove:
      return UIA_SelectionItemIsSelectedPropertyId;
    case ax::mojom::Event::kValueChanged:
      if (SupportsToggle(GetData().role)) {
        return UIA_ToggleToggleStatePropertyId;
      }
      return std::nullopt;
    default:
      return std::nullopt;
  }
}

// static
BSTR AXPlatformNodeWin::GetValueAttributeAsBstr(AXPlatformNodeWin* target) {
  // GetValueAttributeAsBstr() has two sets of special cases depending on the
  // node's role.
  // The first set apply without regard for the nodes |value| attribute. That is
  // the nodes value attribute isn't consider for the first set of special
  // cases. For example, if the node role is ax::mojom::Role::kColorWell, we do
  // not care at all about the node's ax::mojom::StringAttribute::kValue
  // attribute. The second set of special cases only apply if the value
  // attribute for the node is empty.  That is, if
  // ax::mojom::StringAttribute::kValue is empty, we do something special.
  std::u16string result;

  //
  // Color Well special case (Use ax::mojom::IntAttribute::kColorValue)
  //
  if (target->GetData().role == ax::mojom::Role::kColorWell) {
    // static cast because SkColor is a 4-byte unsigned int
    unsigned int color = static_cast<unsigned int>(
        target->GetIntAttribute(ax::mojom::IntAttribute::kColorValue));

    // This is just ARGB
    unsigned int red = (((color) >> 16) & 0xFF);
    unsigned int green = (((color) >> 8) & 0xFF);
    unsigned int blue = (((color) >> 0) & 0xFF);
    std::u16string value_text;
    value_text = base::NumberToString16(red * 100 / 255) + u"% red " +
                 base::NumberToString16(green * 100 / 255) + u"% green " +
                 base::NumberToString16(blue * 100 / 255) + u"% blue";
    BSTR value = ::SysAllocString(fml::Utf16ToWideString(value_text).c_str());
    BASE_DCHECK(value);
    return value;
  }

  //
  // Document special case (Use the document's URL)
  //
  if (target->GetData().role == ax::mojom::Role::kRootWebArea ||
      target->GetData().role == ax::mojom::Role::kWebArea) {
    result = base::UTF8ToUTF16(target->GetDelegate()->GetTreeData().url);
    BSTR value = ::SysAllocString(fml::Utf16ToWideString(result).c_str());
    BASE_DCHECK(value);
    return value;
  }

  //
  // Links (Use ax::mojom::StringAttribute::kUrl)
  //
  if (target->GetData().role == ax::mojom::Role::kLink) {
    result = target->GetString16Attribute(ax::mojom::StringAttribute::kUrl);
    BSTR value = ::SysAllocString(fml::Utf16ToWideString(result).c_str());
    BASE_DCHECK(value);
    return value;
  }

  // For range controls, e.g. sliders and spin buttons, |ax_attr_value| holds
  // the aria-valuetext if present but not the inner text. The actual value,
  // provided either via aria-valuenow or the actual control's value is held in
  // |ax::mojom::FloatAttribute::kValueForRange|.
  result = target->GetString16Attribute(ax::mojom::StringAttribute::kValue);
  if (result.empty() && target->GetData().IsRangeValueSupported()) {
    float fval;
    if (target->GetFloatAttribute(ax::mojom::FloatAttribute::kValueForRange,
                                  &fval)) {
      result = base::NumberToString16(fval);
      BSTR value = ::SysAllocString(fml::Utf16ToWideString(result).c_str());
      BASE_DCHECK(value);
      return value;
    }
  }

  if (result.empty() && target->IsRichTextField())
    result = target->GetInnerText();

  BSTR value = ::SysAllocString(fml::Utf16ToWideString(result).c_str());
  BASE_DCHECK(value);
  return value;
}

HRESULT AXPlatformNodeWin::GetStringAttributeAsBstr(
    ax::mojom::StringAttribute attribute,
    BSTR* value_bstr) const {
  std::u16string str;

  if (!GetString16Attribute(attribute, &str))
    return S_FALSE;

  *value_bstr = ::SysAllocString(fml::Utf16ToWideString(str).c_str());
  BASE_DCHECK(*value_bstr);

  return S_OK;
}

HRESULT AXPlatformNodeWin::GetNameAsBstr(BSTR* value_bstr) const {
  std::u16string str = GetNameAsString16();
  *value_bstr = ::SysAllocString(fml::Utf16ToWideString(str).c_str());
  BASE_DCHECK(*value_bstr);
  return S_OK;
}

// TODO(gw280): https://github.com/flutter/flutter/issues/78800
// Alert targets
void AXPlatformNodeWin::AddAlertTarget() {}

void AXPlatformNodeWin::RemoveAlertTarget() {}

AXPlatformNodeWin* AXPlatformNodeWin::GetTargetFromChildID(
    const VARIANT& var_id) {
  if (V_VT(&var_id) != VT_I4)
    return nullptr;

  LONG child_id = V_I4(&var_id);
  if (child_id == CHILDID_SELF)
    return this;

  if (child_id >= 1 && child_id <= GetDelegate()->GetChildCount()) {
    // Positive child ids are a 1-based child index, used by clients
    // that want to enumerate all immediate children.
    AXPlatformNodeBase* base =
        FromNativeViewAccessible(GetDelegate()->ChildAtIndex(child_id - 1));
    return static_cast<AXPlatformNodeWin*>(base);
  }

  if (child_id >= 0)
    return nullptr;

  // Negative child ids can be used to map to any descendant.
  AXPlatformNode* node = GetFromUniqueId(-child_id);
  if (!node)
    return nullptr;

  AXPlatformNodeBase* base =
      FromNativeViewAccessible(node->GetNativeViewAccessible());
  if (base && !base->IsDescendantOf(this))
    base = nullptr;

  return static_cast<AXPlatformNodeWin*>(base);
}

bool AXPlatformNodeWin::IsInTreeGrid() {
  AXPlatformNodeBase* container = FromNativeViewAccessible(GetParent());

  // If parent was a rowgroup, we need to look at the grandparent
  if (container && container->GetData().role == ax::mojom::Role::kRowGroup)
    container = FromNativeViewAccessible(container->GetParent());

  if (!container)
    return false;

  return container->GetData().role == ax::mojom::Role::kTreeGrid;
}

HRESULT AXPlatformNodeWin::AllocateComArrayFromVector(
    std::vector<LONG>& results,
    LONG max,
    LONG** selected,
    LONG* n_selected) {
  BASE_DCHECK(max > 0);
  BASE_DCHECK(selected);
  BASE_DCHECK(n_selected);

  auto count = std::min((LONG)results.size(), max);
  *n_selected = count;
  *selected = static_cast<LONG*>(CoTaskMemAlloc(sizeof(LONG) * count));

  for (LONG i = 0; i < count; i++)
    (*selected)[i] = results[i];
  return S_OK;
}

bool AXPlatformNodeWin::IsPlaceholderText() const {
  if (GetData().role != ax::mojom::Role::kStaticText)
    return false;
  AXPlatformNodeWin* parent =
      static_cast<AXPlatformNodeWin*>(FromNativeViewAccessible(GetParent()));
  // Static text nodes are always expected to have a parent.
  BASE_DCHECK(parent);
  return parent->IsTextField() &&
         parent->HasStringAttribute(ax::mojom::StringAttribute::kPlaceholder);
}

double AXPlatformNodeWin::GetHorizontalScrollPercent() {
  if (!IsHorizontallyScrollable())
    return UIA_ScrollPatternNoScroll;

  float x_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMin);
  float x_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollXMax);
  float x = GetIntAttribute(ax::mojom::IntAttribute::kScrollX);
  return 100.0 * (x - x_min) / (x_max - x_min);
}

double AXPlatformNodeWin::GetVerticalScrollPercent() {
  if (!IsVerticallyScrollable())
    return UIA_ScrollPatternNoScroll;

  float y_min = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMin);
  float y_max = GetIntAttribute(ax::mojom::IntAttribute::kScrollYMax);
  float y = GetIntAttribute(ax::mojom::IntAttribute::kScrollY);
  return 100.0 * (y - y_min) / (y_max - y_min);
}

BSTR AXPlatformNodeWin::GetFontNameAttributeAsBSTR() const {
  const std::u16string string =
      GetInheritedString16Attribute(ax::mojom::StringAttribute::kFontFamily);

  return ::SysAllocString(fml::Utf16ToWideString(string).c_str());
}

BSTR AXPlatformNodeWin::GetStyleNameAttributeAsBSTR() const {
  std::u16string style_name =
      GetDelegate()->GetStyleNameAttributeAsLocalizedString();

  return ::SysAllocString(fml::Utf16ToWideString(style_name).c_str());
}

TextDecorationLineStyle AXPlatformNodeWin::GetUIATextDecorationStyle(
    const ax::mojom::IntAttribute int_attribute) const {
  const ax::mojom::TextDecorationStyle text_decoration_style =
      static_cast<ax::mojom::TextDecorationStyle>(
          GetIntAttribute(int_attribute));

  switch (text_decoration_style) {
    case ax::mojom::TextDecorationStyle::kNone:
      return TextDecorationLineStyle::TextDecorationLineStyle_None;
    case ax::mojom::TextDecorationStyle::kDotted:
      return TextDecorationLineStyle::TextDecorationLineStyle_Dot;
    case ax::mojom::TextDecorationStyle::kDashed:
      return TextDecorationLineStyle::TextDecorationLineStyle_Dash;
    case ax::mojom::TextDecorationStyle::kSolid:
      return TextDecorationLineStyle::TextDecorationLineStyle_Single;
    case ax::mojom::TextDecorationStyle::kDouble:
      return TextDecorationLineStyle::TextDecorationLineStyle_Double;
    case ax::mojom::TextDecorationStyle::kWavy:
      return TextDecorationLineStyle::TextDecorationLineStyle_Wavy;
  }
}

// IRawElementProviderSimple support methods.

AXPlatformNodeWin::PatternProviderFactoryMethod
AXPlatformNodeWin::GetPatternProviderFactoryMethod(PATTERNID pattern_id) {
  const AXNodeData& data = GetData();

  switch (pattern_id) {
    case UIA_ExpandCollapsePatternId:
      if (data.SupportsExpandCollapse()) {
        return &PatternProvider<IExpandCollapseProvider>;
      }
      break;

    case UIA_GridPatternId:
      if (IsTableLike(data.role)) {
        return &PatternProvider<IGridProvider>;
      }
      break;

    case UIA_GridItemPatternId:
      if (IsCellOrTableHeader(data.role)) {
        return &PatternProvider<IGridItemProvider>;
      }
      break;

    case UIA_InvokePatternId:
      if (data.IsInvocable()) {
        return &PatternProvider<IInvokeProvider>;
      }
      break;

    case UIA_RangeValuePatternId:
      if (data.IsRangeValueSupported()) {
        return &PatternProvider<IRangeValueProvider>;
      }
      break;

    case UIA_ScrollPatternId:
      if (IsScrollable()) {
        return &PatternProvider<IScrollProvider>;
      }
      break;

    case UIA_ScrollItemPatternId:
      return &PatternProvider<IScrollItemProvider>;
      break;

    case UIA_SelectionItemPatternId:
      if (IsSelectionItemSupported()) {
        return &PatternProvider<ISelectionItemProvider>;
      }
      break;

    case UIA_SelectionPatternId:
      if (IsContainerWithSelectableChildren(data.role)) {
        return &PatternProvider<ISelectionProvider>;
      }
      break;

    case UIA_TablePatternId:
      // https://docs.microsoft.com/en-us/windows/win32/api/uiautomationcore/nn-uiautomationcore-itableprovider
      // This control pattern is analogous to IGridProvider with the distinction
      // that any control implementing ITableProvider must also expose a column
      // and/or row header relationship for each child element.
      if (IsTableLike(data.role)) {
        std::optional<bool> table_has_headers =
            GetDelegate()->GetTableHasColumnOrRowHeaderNode();
        if (table_has_headers.has_value() && table_has_headers.value()) {
          return &PatternProvider<ITableProvider>;
        }
      }
      break;

    case UIA_TableItemPatternId:
      // https://docs.microsoft.com/en-us/windows/win32/api/uiautomationcore/nn-uiautomationcore-itableitemprovider
      // This control pattern is analogous to IGridItemProvider with the
      // distinction that any control implementing ITableItemProvider must
      // expose the relationship between the individual cell and its row and
      // column information.
      if (IsCellOrTableHeader(data.role)) {
        std::optional<bool> table_has_headers =
            GetDelegate()->GetTableHasColumnOrRowHeaderNode();
        if (table_has_headers.has_value() && table_has_headers.value()) {
          return &PatternProvider<ITableItemProvider>;
        }
      }
      break;

    case UIA_TextEditPatternId:
    case UIA_TextPatternId:
      if (IsText() || IsTextField() ||
          data.role == ax::mojom::Role::kRootWebArea) {
        return &AXPlatformNodeTextProviderWin::CreateIUnknown;
      }
      break;

    case UIA_TogglePatternId:
      if (SupportsToggle(data.role)) {
        return &PatternProvider<IToggleProvider>;
      }
      break;

    case UIA_ValuePatternId:
      if (IsValuePatternSupported(GetDelegate())) {
        return &PatternProvider<IValueProvider>;
      }
      break;

    case UIA_WindowPatternId:
      if (HasBoolAttribute(ax::mojom::BoolAttribute::kModal)) {
        return &PatternProvider<IWindowProvider>;
      }
      break;

    // Not currently implemented.
    case UIA_AnnotationPatternId:
    case UIA_CustomNavigationPatternId:
    case UIA_DockPatternId:
    case UIA_DragPatternId:
    case UIA_DropTargetPatternId:
    case UIA_ItemContainerPatternId:
    case UIA_MultipleViewPatternId:
    case UIA_ObjectModelPatternId:
    case UIA_SpreadsheetPatternId:
    case UIA_SpreadsheetItemPatternId:
    case UIA_StylesPatternId:
    case UIA_SynchronizedInputPatternId:
    case UIA_TextPattern2Id:
    case UIA_TransformPatternId:
    case UIA_TransformPattern2Id:
    case UIA_VirtualizedItemPatternId:
      break;

    // Provided by UIA Core; we should not implement.
    case UIA_LegacyIAccessiblePatternId:
      break;
  }
  return nullptr;
}

void AXPlatformNodeWin::FireLiveRegionChangeRecursive() {
  const auto live_status_attr = ax::mojom::StringAttribute::kLiveStatus;
  if (HasStringAttribute(live_status_attr) &&
      GetStringAttribute(live_status_attr) != "off") {
    BASE_DCHECK(GetDelegate()->IsWebContent());
    ::UiaRaiseAutomationEvent(this, UIA_LiveRegionChangedEventId);
    return;
  }

  for (int index = 0; index < GetChildCount(); ++index) {
    auto* child = static_cast<AXPlatformNodeWin*>(
        FromNativeViewAccessible(ChildAtIndex(index)));

    // We assume that only web-content will have live regions; also because
    // this will be called on each fragment-root, there is no need to walk
    // through non-content nodes.
    if (child->GetDelegate()->IsWebContent())
      child->FireLiveRegionChangeRecursive();
  }
}

AXPlatformNodeWin* AXPlatformNodeWin::GetLowestAccessibleElement() {
  if (!IsInaccessibleDueToAncestor())
    return this;

  AXPlatformNodeWin* parent = static_cast<AXPlatformNodeWin*>(
      AXPlatformNode::FromNativeViewAccessible(GetParent()));
  while (parent) {
    if (parent->ShouldHideChildrenForUIA())
      return parent;
    parent = static_cast<AXPlatformNodeWin*>(
        AXPlatformNode::FromNativeViewAccessible(parent->GetParent()));
  }

  BASE_UNREACHABLE();
  return nullptr;
}

AXPlatformNodeWin* AXPlatformNodeWin::GetFirstTextOnlyDescendant() {
  for (auto* child = static_cast<AXPlatformNodeWin*>(GetFirstChild()); child;
       child = static_cast<AXPlatformNodeWin*>(child->GetNextSibling())) {
    if (child->IsText())
      return child;
    if (AXPlatformNodeWin* descendant = child->GetFirstTextOnlyDescendant())
      return descendant;
  }
  return nullptr;
}

bool AXPlatformNodeWin::IsDescendantOf(AXPlatformNode* ancestor) const {
  if (!ancestor) {
    return false;
  }

  if (AXPlatformNodeBase::IsDescendantOf(ancestor)) {
    return true;
  }

  // Test if the ancestor is an IRawElementProviderFragmentRoot and if it
  // matches this node's root fragment.
  IRawElementProviderFragmentRoot* root;
  if (SUCCEEDED(
          const_cast<AXPlatformNodeWin*>(this)->get_FragmentRoot(&root))) {
    AXPlatformNodeWin* root_win;
    if (SUCCEEDED(root->QueryInterface(__uuidof(AXPlatformNodeWin),
                                       reinterpret_cast<void**>(&root_win)))) {
      return ancestor == static_cast<AXPlatformNode*>(root_win);
    }
  }

  return false;
}

}  // namespace ui
