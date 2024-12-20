//
// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_platform_node_win_unittest.h"

#include <oleacc.h>
#include <wrl/client.h>

#include <memory>

#include "ax_fragment_root_win.h"
#include "ax_platform_node_win.h"
#include "base/auto_reset.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "test_ax_node_wrapper.h"
#include "third_party/accessibility/ax/ax_enums.h"
#include "third_party/accessibility/ax/ax_node_data.h"
#include "third_party/accessibility/base/win/atl_module.h"
#include "third_party/accessibility/base/win/scoped_bstr.h"
#include "third_party/accessibility/base/win/scoped_safearray.h"
#include "third_party/accessibility/base/win/scoped_variant.h"

using base::win::ScopedBstr;
using base::win::ScopedVariant;
using Microsoft::WRL::ComPtr;

namespace ui {

const std::u16string AXPlatformNodeWinTest::kEmbeddedCharacterAsString = {
    ui::AXPlatformNodeBase::kEmbeddedCharacter};

namespace {

// Most IAccessible functions require a VARIANT set to CHILDID_SELF as
// the first argument.
ScopedVariant SELF(CHILDID_SELF);

}  // namespace

// Helper macros for UIAutomation HRESULT expectations
#define EXPECT_UIA_ELEMENTNOTAVAILABLE(expr) \
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE), (expr))
#define EXPECT_UIA_INVALIDOPERATION(expr) \
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_INVALIDOPERATION), (expr))
#define EXPECT_UIA_ELEMENTNOTENABLED(expr) \
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTENABLED), (expr))
#define EXPECT_UIA_NOTSUPPORTED(expr) \
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED), (expr))

#define ASSERT_UIA_ELEMENTNOTAVAILABLE(expr) \
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE), (expr))
#define ASSERT_UIA_INVALIDOPERATION(expr) \
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_INVALIDOPERATION), (expr))
#define ASSERT_UIA_ELEMENTNOTENABLED(expr) \
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTENABLED), (expr))
#define ASSERT_UIA_NOTSUPPORTED(expr) \
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED), (expr))

// Helper macros for testing UIAutomation property values and maintain
// correct stack tracing and failure causality.
//
// WARNING: These aren't intended to be generic EXPECT_BSTR_EQ macros
// as the logic is specific to extracting and comparing UIA property
// values.
#define EXPECT_UIA_EMPTY(node, property_id)                     \
  {                                                             \
    ScopedVariant actual;                                       \
    ASSERT_HRESULT_SUCCEEDED(                                   \
        node->GetPropertyValue(property_id, actual.Receive())); \
    EXPECT_EQ(VT_EMPTY, actual.type());                         \
  }

#define EXPECT_UIA_VALUE_EQ(node, property_id, expectedVariant) \
  {                                                             \
    ScopedVariant actual;                                       \
    ASSERT_HRESULT_SUCCEEDED(                                   \
        node->GetPropertyValue(property_id, actual.Receive())); \
    EXPECT_EQ(0, actual.Compare(expectedVariant));              \
  }

#define EXPECT_UIA_BSTR_EQ(node, property_id, expected)                  \
  {                                                                      \
    ScopedVariant expectedVariant(expected);                             \
    ASSERT_EQ(VT_BSTR, expectedVariant.type());                          \
    ASSERT_NE(nullptr, expectedVariant.ptr()->bstrVal);                  \
    ScopedVariant actual;                                                \
    ASSERT_HRESULT_SUCCEEDED(                                            \
        node->GetPropertyValue(property_id, actual.Receive()));          \
    ASSERT_EQ(VT_BSTR, actual.type());                                   \
    ASSERT_NE(nullptr, actual.ptr()->bstrVal);                           \
    EXPECT_STREQ(expectedVariant.ptr()->bstrVal, actual.ptr()->bstrVal); \
  }

#define EXPECT_UIA_BOOL_EQ(node, property_id, expected)               \
  {                                                                   \
    ScopedVariant expectedVariant(expected);                          \
    ASSERT_EQ(VT_BOOL, expectedVariant.type());                       \
    ScopedVariant actual;                                             \
    ASSERT_HRESULT_SUCCEEDED(                                         \
        node->GetPropertyValue(property_id, actual.Receive()));       \
    EXPECT_EQ(expectedVariant.ptr()->boolVal, actual.ptr()->boolVal); \
  }

#define EXPECT_UIA_DOUBLE_ARRAY_EQ(node, array_property_id,                 \
                                   expected_property_values)                \
  {                                                                         \
    ScopedVariant array;                                                    \
    ASSERT_HRESULT_SUCCEEDED(                                               \
        node->GetPropertyValue(array_property_id, array.Receive()));        \
    ASSERT_EQ(VT_ARRAY | VT_R8, array.type());                              \
    ASSERT_EQ(1u, SafeArrayGetDim(array.ptr()->parray));                    \
    LONG array_lower_bound;                                                 \
    ASSERT_HRESULT_SUCCEEDED(                                               \
        SafeArrayGetLBound(array.ptr()->parray, 1, &array_lower_bound));    \
    LONG array_upper_bound;                                                 \
    ASSERT_HRESULT_SUCCEEDED(                                               \
        SafeArrayGetUBound(array.ptr()->parray, 1, &array_upper_bound));    \
    double* array_data;                                                     \
    ASSERT_HRESULT_SUCCEEDED(::SafeArrayAccessData(                         \
        array.ptr()->parray, reinterpret_cast<void**>(&array_data)));       \
    size_t count = array_upper_bound - array_lower_bound + 1;               \
    ASSERT_EQ(expected_property_values.size(), count);                      \
    for (size_t i = 0; i < count; ++i) {                                    \
      EXPECT_EQ(array_data[i], expected_property_values[i]);                \
    }                                                                       \
    ASSERT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(array.ptr()->parray)); \
  }

#define EXPECT_UIA_INT_EQ(node, property_id, expected)              \
  {                                                                 \
    ScopedVariant expectedVariant(expected);                        \
    ASSERT_EQ(VT_I4, expectedVariant.type());                       \
    ScopedVariant actual;                                           \
    ASSERT_HRESULT_SUCCEEDED(                                       \
        node->GetPropertyValue(property_id, actual.Receive()));     \
    EXPECT_EQ(expectedVariant.ptr()->intVal, actual.ptr()->intVal); \
  }

#define EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(array, element_test_property_id,     \
                                         expected_property_values)            \
  {                                                                           \
    ASSERT_EQ(1u, SafeArrayGetDim(array));                                    \
    LONG array_lower_bound;                                                   \
    ASSERT_HRESULT_SUCCEEDED(                                                 \
        SafeArrayGetLBound(array, 1, &array_lower_bound));                    \
    LONG array_upper_bound;                                                   \
    ASSERT_HRESULT_SUCCEEDED(                                                 \
        SafeArrayGetUBound(array, 1, &array_upper_bound));                    \
    IUnknown** array_data;                                                    \
    ASSERT_HRESULT_SUCCEEDED(                                                 \
        ::SafeArrayAccessData(array, reinterpret_cast<void**>(&array_data))); \
    size_t count = array_upper_bound - array_lower_bound + 1;                 \
    ASSERT_EQ(expected_property_values.size(), count);                        \
    for (size_t i = 0; i < count; ++i) {                                      \
      ComPtr<IRawElementProviderSimple> element;                              \
      ASSERT_HRESULT_SUCCEEDED(                                               \
          array_data[i]->QueryInterface(IID_PPV_ARGS(&element)));             \
      EXPECT_UIA_BSTR_EQ(element, element_test_property_id,                   \
                         expected_property_values[i].c_str());                \
    }                                                                         \
    ASSERT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(array));                 \
  }

#define EXPECT_UIA_PROPERTY_ELEMENT_ARRAY_BSTR_EQ(node, array_property_id,  \
                                                  element_test_property_id, \
                                                  expected_property_values) \
  {                                                                         \
    ScopedVariant array;                                                    \
    ASSERT_HRESULT_SUCCEEDED(                                               \
        node->GetPropertyValue(array_property_id, array.Receive()));        \
    ASSERT_EQ(VT_ARRAY | VT_UNKNOWN, array.type());                         \
    EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(array.ptr()->parray,                   \
                                     element_test_property_id,              \
                                     expected_property_values);             \
  }

#define EXPECT_UIA_PROPERTY_UNORDERED_ELEMENT_ARRAY_BSTR_EQ(                   \
    node, array_property_id, element_test_property_id,                         \
    expected_property_values)                                                  \
  {                                                                            \
    ScopedVariant array;                                                       \
    ASSERT_HRESULT_SUCCEEDED(                                                  \
        node->GetPropertyValue(array_property_id, array.Receive()));           \
    ASSERT_EQ(VT_ARRAY | VT_UNKNOWN, array.type());                            \
    ASSERT_EQ(1u, SafeArrayGetDim(array.ptr()->parray));                       \
    LONG array_lower_bound;                                                    \
    ASSERT_HRESULT_SUCCEEDED(                                                  \
        SafeArrayGetLBound(array.ptr()->parray, 1, &array_lower_bound));       \
    LONG array_upper_bound;                                                    \
    ASSERT_HRESULT_SUCCEEDED(                                                  \
        SafeArrayGetUBound(array.ptr()->parray, 1, &array_upper_bound));       \
    IUnknown** array_data;                                                     \
    ASSERT_HRESULT_SUCCEEDED(::SafeArrayAccessData(                            \
        array.ptr()->parray, reinterpret_cast<void**>(&array_data)));          \
    size_t count = array_upper_bound - array_lower_bound + 1;                  \
    ASSERT_EQ(expected_property_values.size(), count);                         \
    std::vector<std::wstring> property_values;                                 \
    for (size_t i = 0; i < count; ++i) {                                       \
      ComPtr<IRawElementProviderSimple> element;                               \
      ASSERT_HRESULT_SUCCEEDED(                                                \
          array_data[i]->QueryInterface(IID_PPV_ARGS(&element)));              \
      ScopedVariant actual;                                                    \
      ASSERT_HRESULT_SUCCEEDED(element->GetPropertyValue(                      \
          element_test_property_id, actual.Receive()));                        \
      ASSERT_EQ(VT_BSTR, actual.type());                                       \
      ASSERT_NE(nullptr, actual.ptr()->bstrVal);                               \
      property_values.push_back(std::wstring(                                  \
          V_BSTR(actual.ptr()), SysStringLen(V_BSTR(actual.ptr()))));          \
    }                                                                          \
    ASSERT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(array.ptr()->parray));    \
    EXPECT_THAT(property_values,                                               \
                testing::UnorderedElementsAreArray(expected_property_values)); \
  }

MockIRawElementProviderSimple::MockIRawElementProviderSimple() = default;
MockIRawElementProviderSimple::~MockIRawElementProviderSimple() = default;

HRESULT
MockIRawElementProviderSimple::CreateMockIRawElementProviderSimple(
    IRawElementProviderSimple** provider) {
  CComObject<MockIRawElementProviderSimple>* raw_element_provider = nullptr;
  HRESULT hr = CComObject<MockIRawElementProviderSimple>::CreateInstance(
      &raw_element_provider);
  if (SUCCEEDED(hr)) {
    *provider = raw_element_provider;
  }

  return hr;
}

//
// IRawElementProviderSimple methods.
//
IFACEMETHODIMP MockIRawElementProviderSimple::GetPatternProvider(
    PATTERNID pattern_id,
    IUnknown** result) {
  return E_NOTIMPL;
}

IFACEMETHODIMP MockIRawElementProviderSimple::GetPropertyValue(
    PROPERTYID property_id,
    VARIANT* result) {
  return E_NOTIMPL;
}

IFACEMETHODIMP
MockIRawElementProviderSimple::get_ProviderOptions(enum ProviderOptions* ret) {
  return E_NOTIMPL;
}

IFACEMETHODIMP MockIRawElementProviderSimple::get_HostRawElementProvider(
    IRawElementProviderSimple** provider) {
  return E_NOTIMPL;
}

AXPlatformNodeWinTest::AXPlatformNodeWinTest() {
  //  scoped_feature_list_.InitAndEnableFeature(features::kIChromeAccessible);
}

AXPlatformNodeWinTest::~AXPlatformNodeWinTest() {}

void AXPlatformNodeWinTest::SetUp() {
  win::CreateATLModuleIfNeeded();
}

void AXPlatformNodeWinTest::TearDown() {
  // Destroy the tree and make sure we're not leaking any objects.
  ax_fragment_root_.reset(nullptr);
  DestroyTree();
  TestAXNodeWrapper::SetGlobalIsWebContent(false);
  TestAXNodeWrapper::ClearHitTestResults();
  ASSERT_EQ(0U, AXPlatformNodeBase::GetInstanceCountForTesting());
}

AXPlatformNode* AXPlatformNodeWinTest::AXPlatformNodeFromNode(AXNode* node) {
  const TestAXNodeWrapper* wrapper =
      TestAXNodeWrapper::GetOrCreate(GetTree(), node);
  return wrapper ? wrapper->ax_platform_node() : nullptr;
}

template <typename T>
ComPtr<T> AXPlatformNodeWinTest::QueryInterfaceFromNodeId(AXNode::AXID id) {
  return QueryInterfaceFromNode<T>(GetNodeFromTree(id));
}

template <typename T>
ComPtr<T> AXPlatformNodeWinTest::QueryInterfaceFromNode(AXNode* node) {
  AXPlatformNode* ax_platform_node = AXPlatformNodeFromNode(node);
  if (!ax_platform_node)
    return ComPtr<T>();
  ComPtr<T> result;
  EXPECT_HRESULT_SUCCEEDED(
      ax_platform_node->GetNativeViewAccessible()->QueryInterface(__uuidof(T),
                                                                  &result));
  return result;
}

ComPtr<IRawElementProviderSimple>
AXPlatformNodeWinTest::GetRootIRawElementProviderSimple() {
  return QueryInterfaceFromNode<IRawElementProviderSimple>(GetRootAsAXNode());
}

ComPtr<IRawElementProviderSimple>
AXPlatformNodeWinTest::GetIRawElementProviderSimpleFromChildIndex(
    int child_index) {
  if (!GetRootAsAXNode() || child_index < 0 ||
      static_cast<size_t>(child_index) >=
          GetRootAsAXNode()->children().size()) {
    return ComPtr<IRawElementProviderSimple>();
  }

  return QueryInterfaceFromNode<IRawElementProviderSimple>(
      GetRootAsAXNode()->children()[static_cast<size_t>(child_index)]);
}

Microsoft::WRL::ComPtr<IRawElementProviderSimple>
AXPlatformNodeWinTest::GetIRawElementProviderSimpleFromTree(
    const ui::AXTreeID tree_id,
    const AXNode::AXID node_id) {
  return QueryInterfaceFromNode<IRawElementProviderSimple>(
      GetNodeFromTree(tree_id, node_id));
}

ComPtr<IRawElementProviderFragment>
AXPlatformNodeWinTest::GetRootIRawElementProviderFragment() {
  return QueryInterfaceFromNode<IRawElementProviderFragment>(GetRootAsAXNode());
}

Microsoft::WRL::ComPtr<IRawElementProviderFragment>
AXPlatformNodeWinTest::IRawElementProviderFragmentFromNode(AXNode* node) {
  AXPlatformNode* platform_node = AXPlatformNodeFromNode(node);
  gfx::NativeViewAccessible native_view =
      platform_node->GetNativeViewAccessible();
  ComPtr<IUnknown> unknown_node = native_view;
  ComPtr<IRawElementProviderFragment> fragment_node;
  unknown_node.As(&fragment_node);

  return fragment_node;
}

ComPtr<IAccessible> AXPlatformNodeWinTest::IAccessibleFromNode(AXNode* node) {
  return QueryInterfaceFromNode<IAccessible>(node);
}

ComPtr<IAccessible> AXPlatformNodeWinTest::GetRootIAccessible() {
  return IAccessibleFromNode(GetRootAsAXNode());
}

void AXPlatformNodeWinTest::CheckVariantHasName(const ScopedVariant& variant,
                                                const wchar_t* expected_name) {
  ASSERT_NE(nullptr, variant.ptr());
  ComPtr<IAccessible> accessible;
  ASSERT_HRESULT_SUCCEEDED(
      V_DISPATCH(variant.ptr())->QueryInterface(IID_PPV_ARGS(&accessible)));
  ScopedBstr name;
  EXPECT_EQ(S_OK, accessible->get_accName(SELF, name.Receive()));
  EXPECT_STREQ(expected_name, name.Get());
}

void AXPlatformNodeWinTest::InitFragmentRoot() {
  test_fragment_root_delegate_ = std::make_unique<TestFragmentRootDelegate>();
  ax_fragment_root_.reset(InitNodeAsFragmentRoot(
      GetRootAsAXNode(), test_fragment_root_delegate_.get()));
}

AXFragmentRootWin* AXPlatformNodeWinTest::InitNodeAsFragmentRoot(
    AXNode* node,
    TestFragmentRootDelegate* delegate) {
  delegate->child_ = AXPlatformNodeFromNode(node)->GetNativeViewAccessible();
  if (node->parent())
    delegate->parent_ =
        AXPlatformNodeFromNode(node->parent())->GetNativeViewAccessible();

  return new AXFragmentRootWin(gfx::kMockAcceleratedWidget, delegate);
}

ComPtr<IRawElementProviderFragmentRoot>
AXPlatformNodeWinTest::GetFragmentRoot() {
  ComPtr<IRawElementProviderFragmentRoot> fragment_root_provider;
  ax_fragment_root_->GetNativeViewAccessible()->QueryInterface(
      IID_PPV_ARGS(&fragment_root_provider));
  return fragment_root_provider;
}

AXPlatformNodeWinTest::PatternSet
AXPlatformNodeWinTest::GetSupportedPatternsFromNodeId(AXNode::AXID id) {
  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      QueryInterfaceFromNodeId<IRawElementProviderSimple>(id);
  PatternSet supported_patterns;
  static const std::vector<LONG> all_supported_patterns_ = {
      UIA_TextChildPatternId,  UIA_TextEditPatternId,
      UIA_TextPatternId,       UIA_WindowPatternId,
      UIA_InvokePatternId,     UIA_ExpandCollapsePatternId,
      UIA_GridPatternId,       UIA_GridItemPatternId,
      UIA_RangeValuePatternId, UIA_ScrollPatternId,
      UIA_ScrollItemPatternId, UIA_TablePatternId,
      UIA_TableItemPatternId,  UIA_SelectionItemPatternId,
      UIA_SelectionPatternId,  UIA_TogglePatternId,
      UIA_ValuePatternId,
  };
  for (LONG property_id : all_supported_patterns_) {
    ComPtr<IUnknown> provider;
    if (SUCCEEDED(raw_element_provider_simple->GetPatternProvider(property_id,
                                                                  &provider)) &&
        provider) {
      supported_patterns.insert(property_id);
    }
  }
  return supported_patterns;
}

TestFragmentRootDelegate::TestFragmentRootDelegate() = default;

TestFragmentRootDelegate::~TestFragmentRootDelegate() = default;

gfx::NativeViewAccessible TestFragmentRootDelegate::GetChildOfAXFragmentRoot() {
  return child_;
}

gfx::NativeViewAccessible
TestFragmentRootDelegate::GetParentOfAXFragmentRoot() {
  return parent_;
}

bool TestFragmentRootDelegate::IsAXFragmentRootAControlElement() {
  return is_control_element_;
}

TEST_F(AXPlatformNodeWinTest, IAccessibleDetachedObject) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("Name");
  Init(root);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ScopedBstr name;
  EXPECT_EQ(S_OK, root_obj->get_accName(SELF, name.Receive()));
  EXPECT_STREQ(L"Name", name.Get());

  // Create an empty tree.
  SetTree(std::make_unique<AXTree>());
  ScopedBstr name2;
  EXPECT_EQ(E_FAIL, root_obj->get_accName(SELF, name2.Receive()));
}

TEST_F(AXPlatformNodeWinTest, IAccessibleHitTest) {
  AXNodeData root;
  root.id = 1;
  root.relative_bounds.bounds = gfx::RectF(0, 0, 40, 40);

  AXNodeData node1;
  node1.id = 2;
  node1.role = ax::mojom::Role::kGenericContainer;
  node1.relative_bounds.bounds = gfx::RectF(0, 0, 10, 10);
  node1.SetName("Name1");
  root.child_ids.push_back(node1.id);

  AXNodeData node2;
  node2.id = 3;
  node2.role = ax::mojom::Role::kGenericContainer;
  node2.relative_bounds.bounds = gfx::RectF(20, 20, 20, 20);
  node2.SetName("Name2");
  root.child_ids.push_back(node2.id);

  Init(root, node1, node2);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());

  // This is way outside of the root node.
  ScopedVariant obj_1;
  EXPECT_EQ(S_FALSE, root_obj->accHitTest(50, 50, obj_1.Receive()));
  EXPECT_EQ(VT_EMPTY, obj_1.type());

  // This is directly on node 1.
  EXPECT_EQ(S_OK, root_obj->accHitTest(5, 5, obj_1.Receive()));
  ASSERT_NE(nullptr, obj_1.ptr());
  CheckVariantHasName(obj_1, L"Name1");

  // This is directly on node 2 with a scale factor of 1.5.
  ScopedVariant obj_2;
  std::unique_ptr<base::AutoReset<float>> scale_factor_reset =
      TestAXNodeWrapper::SetScaleFactor(1.5);
  EXPECT_EQ(S_OK, root_obj->accHitTest(38, 38, obj_2.Receive()));
  ASSERT_NE(nullptr, obj_2.ptr());
  CheckVariantHasName(obj_2, L"Name2");
}

TEST_F(AXPlatformNodeWinTest, IAccessibleHitTestDoesNotLoopForever) {
  AXNodeData root;
  root.id = 1;
  root.relative_bounds.bounds = gfx::RectF(0, 0, 40, 40);

  AXNodeData node1;
  node1.id = 2;
  node1.role = ax::mojom::Role::kGenericContainer;
  node1.relative_bounds.bounds = gfx::RectF(0, 0, 10, 10);
  node1.SetName("Name1");
  root.child_ids.push_back(node1.id);

  Init(root, node1);

  // Set up the endless loop.
  TestAXNodeWrapper::SetHitTestResult(1, 2);
  TestAXNodeWrapper::SetHitTestResult(2, 1);

  // Hit testing on the root returns the child. Hit testing on the
  // child returns the root, but that should be rejected rather than
  // looping endlessly.
  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ScopedVariant obj_1;
  EXPECT_EQ(S_OK, root_obj->accHitTest(5, 5, obj_1.Receive()));
  ASSERT_NE(nullptr, obj_1.ptr());
  CheckVariantHasName(obj_1, L"Name1");
}

TEST_F(AXPlatformNodeWinTest, IAccessibleName) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("Name");
  Init(root);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ScopedBstr name;
  EXPECT_EQ(S_OK, root_obj->get_accName(SELF, name.Receive()));
  EXPECT_STREQ(L"Name", name.Get());

  EXPECT_EQ(E_INVALIDARG, root_obj->get_accName(SELF, nullptr));
  ScopedVariant bad_id(999);
  ScopedBstr name2;
  EXPECT_EQ(E_INVALIDARG, root_obj->get_accName(bad_id, name2.Receive()));
}

TEST_F(AXPlatformNodeWinTest, IAccessibleDescription) {
  AXNodeData root;
  root.id = 1;
  root.AddStringAttribute(ax::mojom::StringAttribute::kDescription,
                          "Description");
  Init(root);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ScopedBstr description;
  EXPECT_EQ(S_OK, root_obj->get_accDescription(SELF, description.Receive()));
  EXPECT_STREQ(L"Description", description.Get());

  EXPECT_EQ(E_INVALIDARG, root_obj->get_accDescription(SELF, nullptr));
  ScopedVariant bad_id(999);
  ScopedBstr d2;
  EXPECT_EQ(E_INVALIDARG, root_obj->get_accDescription(bad_id, d2.Receive()));
}

TEST_F(AXPlatformNodeWinTest, IAccessibleAccValue) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTextField;
  root.AddStringAttribute(ax::mojom::StringAttribute::kValue, "Value");
  Init(root);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ScopedBstr value;
  EXPECT_EQ(S_OK, root_obj->get_accValue(SELF, value.Receive()));
  EXPECT_STREQ(L"Value", value.Get());

  EXPECT_EQ(E_INVALIDARG, root_obj->get_accValue(SELF, nullptr));
  ScopedVariant bad_id(999);
  ScopedBstr v2;
  EXPECT_EQ(E_INVALIDARG, root_obj->get_accValue(bad_id, v2.Receive()));
}

TEST_F(AXPlatformNodeWinTest, IAccessibleShortcut) {
  AXNodeData root;
  root.id = 1;
  root.AddStringAttribute(ax::mojom::StringAttribute::kKeyShortcuts,
                          "Shortcut");
  Init(root);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ScopedBstr shortcut;
  EXPECT_EQ(S_OK, root_obj->get_accKeyboardShortcut(SELF, shortcut.Receive()));
  EXPECT_STREQ(L"Shortcut", shortcut.Get());

  EXPECT_EQ(E_INVALIDARG, root_obj->get_accKeyboardShortcut(SELF, nullptr));
  ScopedVariant bad_id(999);
  ScopedBstr k2;
  EXPECT_EQ(E_INVALIDARG,
            root_obj->get_accKeyboardShortcut(bad_id, k2.Receive()));
}

TEST_F(AXPlatformNodeWinTest,
       IAccessibleSelectionListBoxOptionNothingSelected) {
  AXNodeData list;
  list.id = 1;
  list.role = ax::mojom::Role::kListBox;

  AXNodeData list_item_1;
  list_item_1.id = 2;
  list_item_1.role = ax::mojom::Role::kListBoxOption;
  list_item_1.SetName("Name1");

  AXNodeData list_item_2;
  list_item_2.id = 3;
  list_item_2.role = ax::mojom::Role::kListBoxOption;
  list_item_2.SetName("Name2");

  list.child_ids.push_back(list_item_1.id);
  list.child_ids.push_back(list_item_2.id);

  Init(list, list_item_1, list_item_2);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ScopedVariant selection;
  EXPECT_EQ(S_OK, root_obj->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_EMPTY, selection.type());
}

TEST_F(AXPlatformNodeWinTest, IAccessibleSelectionListBoxOptionOneSelected) {
  AXNodeData list;
  list.id = 1;
  list.role = ax::mojom::Role::kListBox;

  AXNodeData list_item_1;
  list_item_1.id = 2;
  list_item_1.role = ax::mojom::Role::kListBoxOption;
  list_item_1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  list_item_1.SetName("Name1");

  AXNodeData list_item_2;
  list_item_2.id = 3;
  list_item_2.role = ax::mojom::Role::kListBoxOption;
  list_item_2.SetName("Name2");

  list.child_ids.push_back(list_item_1.id);
  list.child_ids.push_back(list_item_2.id);

  Init(list, list_item_1, list_item_2);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ScopedVariant selection;
  EXPECT_EQ(S_OK, root_obj->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_DISPATCH, selection.type());

  CheckVariantHasName(selection, L"Name1");
}

TEST_F(AXPlatformNodeWinTest,
       IAccessibleSelectionListBoxOptionMultipleSelected) {
  AXNodeData list;
  list.id = 1;
  list.role = ax::mojom::Role::kListBox;

  AXNodeData list_item_1;
  list_item_1.id = 2;
  list_item_1.role = ax::mojom::Role::kListBoxOption;
  list_item_1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  list_item_1.SetName("Name1");

  AXNodeData list_item_2;
  list_item_2.id = 3;
  list_item_2.role = ax::mojom::Role::kListBoxOption;
  list_item_2.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  list_item_2.SetName("Name2");

  AXNodeData list_item_3;
  list_item_3.id = 4;
  list_item_3.role = ax::mojom::Role::kListBoxOption;
  list_item_3.SetName("Name3");

  list.child_ids.push_back(list_item_1.id);
  list.child_ids.push_back(list_item_2.id);
  list.child_ids.push_back(list_item_3.id);

  Init(list, list_item_1, list_item_2, list_item_3);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ScopedVariant selection;
  EXPECT_EQ(S_OK, root_obj->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_UNKNOWN, selection.type());
  ASSERT_NE(nullptr, selection.ptr());

  // Loop through the selections and  make sure we have the right ones.
  ComPtr<IEnumVARIANT> accessibles;
  ASSERT_HRESULT_SUCCEEDED(
      V_UNKNOWN(selection.ptr())->QueryInterface(IID_PPV_ARGS(&accessibles)));
  ULONG retrieved_count;

  // Check out the first selected item.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_OK, hr);

    ComPtr<IAccessible> accessible;
    ASSERT_HRESULT_SUCCEEDED(
        V_DISPATCH(item.ptr())->QueryInterface(IID_PPV_ARGS(&accessible)));
    ScopedBstr name;
    EXPECT_EQ(S_OK, accessible->get_accName(SELF, name.Receive()));
    EXPECT_STREQ(L"Name1", name.Get());
  }

  // And the second selected element.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_OK, hr);

    ComPtr<IAccessible> accessible;
    ASSERT_HRESULT_SUCCEEDED(
        V_DISPATCH(item.ptr())->QueryInterface(IID_PPV_ARGS(&accessible)));
    ScopedBstr name;
    EXPECT_EQ(S_OK, accessible->get_accName(SELF, name.Receive()));
    EXPECT_STREQ(L"Name2", name.Get());
  }

  // There shouldn't be any more selected.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_FALSE, hr);
  }
}

TEST_F(AXPlatformNodeWinTest, IAccessibleSelectionTableNothingSelected) {
  Init(Build3X3Table());

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ScopedVariant selection;
  EXPECT_EQ(S_OK, root_obj->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_EMPTY, selection.type());
}

TEST_F(AXPlatformNodeWinTest, IAccessibleSelectionTableRowOneSelected) {
  AXTreeUpdate update = Build3X3Table();

  // 5 == table_row_1
  update.nodes[5].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);

  Init(update);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ScopedVariant selection;
  EXPECT_EQ(S_OK, root_obj->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_DISPATCH, selection.type());
  ASSERT_NE(nullptr, selection.ptr());

  ComPtr<IAccessible> row;
  ASSERT_HRESULT_SUCCEEDED(
      V_DISPATCH(selection.ptr())->QueryInterface(IID_PPV_ARGS(&row)));

  ScopedVariant role;
  EXPECT_HRESULT_SUCCEEDED(row->get_accRole(SELF, role.Receive()));
  EXPECT_EQ(ROLE_SYSTEM_ROW, V_I4(role.ptr()));
}

TEST_F(AXPlatformNodeWinTest, IAccessibleSelectionTableRowMultipleSelected) {
  AXTreeUpdate update = Build3X3Table();

  // 5 == table_row_1
  // 9 == table_row_2
  update.nodes[5].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  update.nodes[9].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);

  Init(update);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ScopedVariant selection;
  EXPECT_EQ(S_OK, root_obj->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_UNKNOWN, selection.type());
  ASSERT_NE(nullptr, selection.ptr());

  // Loop through the selections and  make sure we have the right ones.
  ComPtr<IEnumVARIANT> accessibles;
  ASSERT_HRESULT_SUCCEEDED(
      V_UNKNOWN(selection.ptr())->QueryInterface(IID_PPV_ARGS(&accessibles)));
  ULONG retrieved_count;

  // Check out the first selected row.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_OK, hr);

    ComPtr<IAccessible> accessible;
    ASSERT_HRESULT_SUCCEEDED(
        V_DISPATCH(item.ptr())->QueryInterface(IID_PPV_ARGS(&accessible)));
    ScopedVariant role;
    EXPECT_HRESULT_SUCCEEDED(accessible->get_accRole(SELF, role.Receive()));
    EXPECT_EQ(ROLE_SYSTEM_ROW, V_I4(role.ptr()));
  }

  // And the second selected element.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_OK, hr);

    ComPtr<IAccessible> accessible;
    ASSERT_HRESULT_SUCCEEDED(
        V_DISPATCH(item.ptr())->QueryInterface(IID_PPV_ARGS(&accessible)));
    ScopedVariant role;
    EXPECT_HRESULT_SUCCEEDED(accessible->get_accRole(SELF, role.Receive()));
    EXPECT_EQ(ROLE_SYSTEM_ROW, V_I4(role.ptr()));
  }

  // There shouldn't be any more selected.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_FALSE, hr);
  }
}

TEST_F(AXPlatformNodeWinTest, IAccessibleSelectionTableCellOneSelected) {
  AXTreeUpdate update = Build3X3Table();

  // 7 == table_cell_1
  update.nodes[7].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);

  Init(update);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ComPtr<IDispatch> row2;
  ASSERT_HRESULT_SUCCEEDED(root_obj->get_accChild(ScopedVariant(2), &row2));
  ComPtr<IAccessible> row2_accessible;
  ASSERT_HRESULT_SUCCEEDED(row2.As(&row2_accessible));

  ScopedVariant selection;
  EXPECT_EQ(S_OK, row2_accessible->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_DISPATCH, selection.type());
  ASSERT_NE(nullptr, selection.ptr());

  ComPtr<IAccessible> cell;
  ASSERT_HRESULT_SUCCEEDED(
      V_DISPATCH(selection.ptr())->QueryInterface(IID_PPV_ARGS(&cell)));

  ScopedVariant role;
  EXPECT_HRESULT_SUCCEEDED(cell->get_accRole(SELF, role.Receive()));
  EXPECT_EQ(ROLE_SYSTEM_CELL, V_I4(role.ptr()));

  ScopedBstr name;
  EXPECT_EQ(S_OK, cell->get_accName(SELF, name.Receive()));
  EXPECT_STREQ(L"1", name.Get());
}

TEST_F(AXPlatformNodeWinTest, IAccessibleSelectionTableCellMultipleSelected) {
  AXTreeUpdate update = Build3X3Table();

  // 11 == table_cell_3
  // 12 == table_cell_4
  update.nodes[11].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  update.nodes[12].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);

  Init(update);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());
  ASSERT_NE(nullptr, root_obj.Get());

  ComPtr<IDispatch> row3;
  ASSERT_HRESULT_SUCCEEDED(root_obj->get_accChild(ScopedVariant(3), &row3));
  ComPtr<IAccessible> row3_accessible;
  ASSERT_HRESULT_SUCCEEDED(row3.As(&row3_accessible));

  ScopedVariant selection;
  EXPECT_EQ(S_OK, row3_accessible->get_accSelection(selection.Receive()));
  EXPECT_EQ(VT_UNKNOWN, selection.type());
  ASSERT_NE(nullptr, selection.ptr());

  // Loop through the selections and  make sure we have the right ones.
  ComPtr<IEnumVARIANT> accessibles;
  ASSERT_HRESULT_SUCCEEDED(
      V_UNKNOWN(selection.ptr())->QueryInterface(IID_PPV_ARGS(&accessibles)));
  ULONG retrieved_count;

  // Check out the first selected cell.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_OK, hr);

    ComPtr<IAccessible> accessible;
    ASSERT_HRESULT_SUCCEEDED(
        V_DISPATCH(item.ptr())->QueryInterface(IID_PPV_ARGS(&accessible)));
    ScopedBstr name;
    EXPECT_EQ(S_OK, accessible->get_accName(SELF, name.Receive()));
    EXPECT_STREQ(L"3", name.Get());
  }

  // And the second selected cell.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_OK, hr);

    ComPtr<IAccessible> accessible;
    ASSERT_HRESULT_SUCCEEDED(
        V_DISPATCH(item.ptr())->QueryInterface(IID_PPV_ARGS(&accessible)));
    ScopedBstr name;
    EXPECT_EQ(S_OK, accessible->get_accName(SELF, name.Receive()));
    EXPECT_STREQ(L"4", name.Get());
  }

  // There shouldn't be any more selected.
  {
    ScopedVariant item;
    HRESULT hr = accessibles->Next(1, item.Receive(), &retrieved_count);
    EXPECT_EQ(S_FALSE, hr);
  }
}

TEST_F(AXPlatformNodeWinTest, IAccessibleRole) {
  AXNodeData root;
  root.id = 1;
  root.child_ids.push_back(2);

  AXNodeData child;
  child.id = 2;

  Init(root, child);
  AXNode* child_node = GetRootAsAXNode()->children()[0];
  ComPtr<IAccessible> child_iaccessible(IAccessibleFromNode(child_node));

  ScopedVariant role;

  child.role = ax::mojom::Role::kAlert;
  child_node->SetData(child);
  EXPECT_EQ(S_OK, child_iaccessible->get_accRole(SELF, role.Receive()));
  EXPECT_EQ(ROLE_SYSTEM_ALERT, V_I4(role.ptr()));

  child.role = ax::mojom::Role::kButton;
  child_node->SetData(child);
  EXPECT_EQ(S_OK, child_iaccessible->get_accRole(SELF, role.Receive()));
  EXPECT_EQ(ROLE_SYSTEM_PUSHBUTTON, V_I4(role.ptr()));

  child.role = ax::mojom::Role::kPopUpButton;
  child_node->SetData(child);
  EXPECT_EQ(S_OK, child_iaccessible->get_accRole(SELF, role.Receive()));
  EXPECT_EQ(ROLE_SYSTEM_BUTTONMENU, V_I4(role.ptr()));

  EXPECT_EQ(E_INVALIDARG, child_iaccessible->get_accRole(SELF, nullptr));
  ScopedVariant bad_id(999);
  EXPECT_EQ(E_INVALIDARG,
            child_iaccessible->get_accRole(bad_id, role.Receive()));
}

TEST_F(AXPlatformNodeWinTest, IAccessibleLocation) {
  AXNodeData root;
  root.id = 1;
  root.relative_bounds.bounds = gfx::RectF(10, 40, 800, 600);
  Init(root);

  TestAXNodeWrapper::SetGlobalCoordinateOffset(gfx::Vector2d(100, 200));

  LONG x_left, y_top, width, height;
  EXPECT_EQ(S_OK, GetRootIAccessible()->accLocation(&x_left, &y_top, &width,
                                                    &height, SELF));
  EXPECT_EQ(110, x_left);
  EXPECT_EQ(240, y_top);
  EXPECT_EQ(800, width);
  EXPECT_EQ(600, height);

  EXPECT_EQ(E_INVALIDARG, GetRootIAccessible()->accLocation(
                              nullptr, &y_top, &width, &height, SELF));
  EXPECT_EQ(E_INVALIDARG, GetRootIAccessible()->accLocation(
                              &x_left, nullptr, &width, &height, SELF));
  EXPECT_EQ(E_INVALIDARG, GetRootIAccessible()->accLocation(
                              &x_left, &y_top, nullptr, &height, SELF));
  EXPECT_EQ(E_INVALIDARG, GetRootIAccessible()->accLocation(
                              &x_left, &y_top, &width, nullptr, SELF));
  ScopedVariant bad_id(999);
  EXPECT_EQ(E_INVALIDARG, GetRootIAccessible()->accLocation(
                              &x_left, &y_top, &width, &height, bad_id));

  // Un-set the global offset so that it doesn't affect subsequent tests.
  TestAXNodeWrapper::SetGlobalCoordinateOffset(gfx::Vector2d(0, 0));
}

TEST_F(AXPlatformNodeWinTest, IAccessibleChildAndParent) {
  AXNodeData root;
  root.id = 1;
  root.child_ids.push_back(2);
  root.child_ids.push_back(3);

  AXNodeData button;
  button.role = ax::mojom::Role::kButton;
  button.id = 2;

  AXNodeData checkbox;
  checkbox.role = ax::mojom::Role::kCheckBox;
  checkbox.id = 3;

  Init(root, button, checkbox);
  AXNode* button_node = GetRootAsAXNode()->children()[0];
  AXNode* checkbox_node = GetRootAsAXNode()->children()[1];
  ComPtr<IAccessible> root_iaccessible(GetRootIAccessible());
  ComPtr<IAccessible> button_iaccessible(IAccessibleFromNode(button_node));
  ComPtr<IAccessible> checkbox_iaccessible(IAccessibleFromNode(checkbox_node));

  LONG child_count;
  EXPECT_EQ(S_OK, root_iaccessible->get_accChildCount(&child_count));
  EXPECT_EQ(2L, child_count);
  EXPECT_EQ(S_OK, button_iaccessible->get_accChildCount(&child_count));
  EXPECT_EQ(0L, child_count);
  EXPECT_EQ(S_OK, checkbox_iaccessible->get_accChildCount(&child_count));
  EXPECT_EQ(0L, child_count);

  {
    ComPtr<IDispatch> result;
    EXPECT_EQ(S_OK, root_iaccessible->get_accChild(SELF, &result));
    EXPECT_EQ(result.Get(), root_iaccessible.Get());
  }

  {
    ComPtr<IDispatch> result;
    ScopedVariant child1(1);
    EXPECT_EQ(S_OK, root_iaccessible->get_accChild(child1, &result));
    EXPECT_EQ(result.Get(), button_iaccessible.Get());
  }

  {
    ComPtr<IDispatch> result;
    ScopedVariant child2(2);
    EXPECT_EQ(S_OK, root_iaccessible->get_accChild(child2, &result));
    EXPECT_EQ(result.Get(), checkbox_iaccessible.Get());
  }

  {
    // Asking for child id 3 should fail.
    ComPtr<IDispatch> result;
    ScopedVariant child3(3);
    EXPECT_EQ(E_INVALIDARG, root_iaccessible->get_accChild(child3, &result));
  }

  // Now check parents.
  {
    ComPtr<IDispatch> result;
    EXPECT_EQ(S_OK, button_iaccessible->get_accParent(&result));
    EXPECT_EQ(result.Get(), root_iaccessible.Get());
  }

  {
    ComPtr<IDispatch> result;
    EXPECT_EQ(S_OK, checkbox_iaccessible->get_accParent(&result));
    EXPECT_EQ(result.Get(), root_iaccessible.Get());
  }

  {
    ComPtr<IDispatch> result;
    EXPECT_EQ(S_FALSE, root_iaccessible->get_accParent(&result));
  }
}

TEST_F(AXPlatformNodeWinTest, AccNavigate) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kStaticText;
  root.child_ids.push_back(2);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kStaticText;
  root.child_ids.push_back(3);

  Init(root, child1, child2);
  ComPtr<IAccessible> ia_root(GetRootIAccessible());
  ComPtr<IDispatch> disp_root;
  ASSERT_HRESULT_SUCCEEDED(ia_root.As(&disp_root));
  ScopedVariant var_root(disp_root.Get());
  ComPtr<IAccessible> ia_child1(
      IAccessibleFromNode(GetRootAsAXNode()->children()[0]));
  ComPtr<IDispatch> disp_child1;
  ASSERT_HRESULT_SUCCEEDED(ia_child1.As(&disp_child1));
  ScopedVariant var_child1(disp_child1.Get());
  ComPtr<IAccessible> ia_child2(
      IAccessibleFromNode(GetRootAsAXNode()->children()[1]));
  ComPtr<IDispatch> disp_child2;
  ASSERT_HRESULT_SUCCEEDED(ia_child2.As(&disp_child2));
  ScopedVariant var_child2(disp_child2.Get());
  ScopedVariant end;

  // Invalid arguments.
  EXPECT_EQ(
      E_INVALIDARG,
      ia_root->accNavigate(NAVDIR_NEXT, ScopedVariant::kEmptyVariant, nullptr));
  EXPECT_EQ(E_INVALIDARG,
            ia_child1->accNavigate(NAVDIR_NEXT, ScopedVariant::kEmptyVariant,
                                   end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());

  // Navigating to first/last child should only be from self.
  EXPECT_EQ(E_INVALIDARG,
            ia_root->accNavigate(NAVDIR_FIRSTCHILD, var_root, end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());
  EXPECT_EQ(E_INVALIDARG,
            ia_root->accNavigate(NAVDIR_LASTCHILD, var_root, end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());

  // Spatial directions are not supported.
  EXPECT_EQ(E_NOTIMPL, ia_child1->accNavigate(NAVDIR_UP, SELF, end.AsInput()));
  EXPECT_EQ(E_NOTIMPL, ia_root->accNavigate(NAVDIR_DOWN, SELF, end.AsInput()));
  EXPECT_EQ(E_NOTIMPL,
            ia_child1->accNavigate(NAVDIR_RIGHT, SELF, end.AsInput()));
  EXPECT_EQ(E_NOTIMPL,
            ia_child2->accNavigate(NAVDIR_LEFT, SELF, end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());

  // Logical directions should be supported.
  EXPECT_EQ(S_OK, ia_root->accNavigate(NAVDIR_FIRSTCHILD, SELF, end.AsInput()));
  EXPECT_EQ(VT_DISPATCH, end.type());
  EXPECT_EQ(V_DISPATCH(var_child1.ptr()), V_DISPATCH(end.ptr()));

  EXPECT_EQ(S_OK, ia_root->accNavigate(NAVDIR_LASTCHILD, SELF, end.AsInput()));
  EXPECT_EQ(VT_DISPATCH, end.type());
  EXPECT_EQ(V_DISPATCH(var_child2.ptr()), V_DISPATCH(end.ptr()));

  EXPECT_EQ(S_OK, ia_child1->accNavigate(NAVDIR_NEXT, SELF, end.AsInput()));
  EXPECT_EQ(VT_DISPATCH, end.type());
  EXPECT_EQ(V_DISPATCH(var_child2.ptr()), V_DISPATCH(end.ptr()));

  EXPECT_EQ(S_OK, ia_child2->accNavigate(NAVDIR_PREVIOUS, SELF, end.AsInput()));
  EXPECT_EQ(VT_DISPATCH, end.type());
  EXPECT_EQ(V_DISPATCH(var_child1.ptr()), V_DISPATCH(end.ptr()));

  // Child indices can also be passed by variant.
  // Indices are one-based.
  EXPECT_EQ(S_OK,
            ia_root->accNavigate(NAVDIR_NEXT, ScopedVariant(1), end.AsInput()));
  EXPECT_EQ(VT_DISPATCH, end.type());
  EXPECT_EQ(V_DISPATCH(var_child2.ptr()), V_DISPATCH(end.ptr()));

  EXPECT_EQ(S_OK, ia_root->accNavigate(NAVDIR_PREVIOUS, ScopedVariant(2),
                                       end.AsInput()));
  EXPECT_EQ(VT_DISPATCH, end.type());
  EXPECT_EQ(V_DISPATCH(var_child1.ptr()), V_DISPATCH(end.ptr()));

  // Test out-of-bounds.
  EXPECT_EQ(S_FALSE,
            ia_child1->accNavigate(NAVDIR_PREVIOUS, SELF, end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());
  EXPECT_EQ(S_FALSE, ia_child2->accNavigate(NAVDIR_NEXT, SELF, end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());

  EXPECT_EQ(S_FALSE, ia_root->accNavigate(NAVDIR_PREVIOUS, ScopedVariant(1),
                                          end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());
  EXPECT_EQ(S_FALSE,
            ia_root->accNavigate(NAVDIR_NEXT, ScopedVariant(2), end.AsInput()));
  EXPECT_EQ(VT_EMPTY, end.type());
}

TEST_F(AXPlatformNodeWinTest, AnnotatedImageName) {
  std::vector<const wchar_t*> expected_names;

  AXTreeUpdate tree;
  tree.root_id = 1;
  tree.nodes.resize(11);
  tree.nodes[0].id = 1;
  tree.nodes[0].child_ids = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  // If the status is EligibleForAnnotation and there's no existing label,
  // the name should be the discoverability string.
  tree.nodes[1].id = 2;
  tree.nodes[1].role = ax::mojom::Role::kImage;
  tree.nodes[1].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[1].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kEligibleForAnnotation);
  expected_names.push_back(
      L"To get missing image descriptions, open the context menu.");

  // If the status is EligibleForAnnotation, the discoverability string
  // should be appended to the existing name.
  tree.nodes[2].id = 3;
  tree.nodes[2].role = ax::mojom::Role::kImage;
  tree.nodes[2].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[2].SetName("ExistingLabel");
  tree.nodes[2].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kEligibleForAnnotation);
  expected_names.push_back(
      L"ExistingLabel. To get missing image descriptions, open the context "
      L"menu.");

  // If the status is SilentlyEligibleForAnnotation, the discoverability string
  // should not be appended to the existing name.
  tree.nodes[3].id = 4;
  tree.nodes[3].role = ax::mojom::Role::kImage;
  tree.nodes[3].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[3].SetName("ExistingLabel");
  tree.nodes[3].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kSilentlyEligibleForAnnotation);
  expected_names.push_back(L"ExistingLabel");

  // If the status is IneligibleForAnnotation, nothing should be appended.
  tree.nodes[4].id = 5;
  tree.nodes[4].role = ax::mojom::Role::kImage;
  tree.nodes[4].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[4].SetName("ExistingLabel");
  tree.nodes[4].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kIneligibleForAnnotation);
  expected_names.push_back(L"ExistingLabel");

  // If the status is AnnotationPending, pending text should be appended
  // to the name.
  tree.nodes[5].id = 6;
  tree.nodes[5].role = ax::mojom::Role::kImage;
  tree.nodes[5].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[5].SetName("ExistingLabel");
  tree.nodes[5].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kAnnotationPending);
  expected_names.push_back(L"ExistingLabel. Getting description...");

  // If the status is AnnotationSucceeded, and there's no annotation,
  // nothing should be appended. (Ideally this shouldn't happen.)
  tree.nodes[6].id = 7;
  tree.nodes[6].role = ax::mojom::Role::kImage;
  tree.nodes[6].SetName("ExistingLabel");
  tree.nodes[6].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kAnnotationSucceeded);
  expected_names.push_back(L"ExistingLabel");

  // If the status is AnnotationSucceeded, the annotation should be appended
  // to the existing label.
  tree.nodes[7].id = 8;
  tree.nodes[7].role = ax::mojom::Role::kImage;
  tree.nodes[7].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[7].SetName("ExistingLabel");
  tree.nodes[7].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kAnnotationSucceeded);
  expected_names.push_back(L"ExistingLabel. Annotation");

  // If the status is AnnotationEmpty, failure text should be added to the
  // name.
  tree.nodes[8].id = 9;
  tree.nodes[8].role = ax::mojom::Role::kImage;
  tree.nodes[8].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[8].SetName("ExistingLabel");
  tree.nodes[8].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kAnnotationEmpty);
  expected_names.push_back(L"ExistingLabel. No description available.");

  // If the status is AnnotationAdult, appropriate text should be appended
  // to the name.
  tree.nodes[9].id = 10;
  tree.nodes[9].role = ax::mojom::Role::kImage;
  tree.nodes[9].AddStringAttribute(ax::mojom::StringAttribute::kImageAnnotation,
                                   "Annotation");
  tree.nodes[9].SetName("ExistingLabel");
  tree.nodes[9].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kAnnotationAdult);
  expected_names.push_back(
      L"ExistingLabel. Appears to contain adult content. No description "
      L"available.");

  // If the status is AnnotationProcessFailed, failure text should be added
  // to the name.
  tree.nodes[10].id = 11;
  tree.nodes[10].role = ax::mojom::Role::kImage;
  tree.nodes[10].AddStringAttribute(
      ax::mojom::StringAttribute::kImageAnnotation, "Annotation");
  tree.nodes[10].SetName("ExistingLabel");
  tree.nodes[10].SetImageAnnotationStatus(
      ax::mojom::ImageAnnotationStatus::kAnnotationProcessFailed);
  expected_names.push_back(L"ExistingLabel. No description available.");

  // We should have one expected name per child of the root.
  ASSERT_EQ(expected_names.size(), tree.nodes[0].child_ids.size());
  int child_count = static_cast<int>(expected_names.size());

  Init(tree);

  ComPtr<IAccessible> root_obj(GetRootIAccessible());

  for (int child_index = 0; child_index < child_count; child_index++) {
    ComPtr<IDispatch> child_dispatch;
    ASSERT_HRESULT_SUCCEEDED(root_obj->get_accChild(
        ScopedVariant(child_index + 1), &child_dispatch));
    ComPtr<IAccessible> child;
    ASSERT_HRESULT_SUCCEEDED(child_dispatch.As(&child));

    ScopedBstr name;
    EXPECT_EQ(S_OK, child->get_accName(SELF, name.Receive()));
    EXPECT_STREQ(expected_names[child_index], name.Get());
  }
}

TEST_F(AXPlatformNodeWinTest, IGridProviderGetRowCount) {
  Init(BuildAriaColumnAndRowCountGrids());

  // Empty Grid
  ComPtr<IGridProvider> grid1_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[0]);

  // Grid with a cell that defines aria-rowindex (4) and aria-colindex (5)
  ComPtr<IGridProvider> grid2_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[1]);

  // Grid that specifies aria-rowcount (2) and aria-colcount (3)
  ComPtr<IGridProvider> grid3_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[2]);

  // Grid that specifies aria-rowcount and aria-colcount are both (-1)
  ComPtr<IGridProvider> grid4_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[3]);

  int row_count;

  EXPECT_HRESULT_SUCCEEDED(grid1_provider->get_RowCount(&row_count));
  EXPECT_EQ(row_count, 0);

  EXPECT_HRESULT_SUCCEEDED(grid2_provider->get_RowCount(&row_count));
  EXPECT_EQ(row_count, 4);

  EXPECT_HRESULT_SUCCEEDED(grid3_provider->get_RowCount(&row_count));
  EXPECT_EQ(row_count, 2);

  EXPECT_EQ(E_UNEXPECTED, grid4_provider->get_RowCount(&row_count));
}

TEST_F(AXPlatformNodeWinTest, IGridProviderGetColumnCount) {
  Init(BuildAriaColumnAndRowCountGrids());

  // Empty Grid
  ComPtr<IGridProvider> grid1_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[0]);

  // Grid with a cell that defines aria-rowindex (4) and aria-colindex (5)
  ComPtr<IGridProvider> grid2_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[1]);

  // Grid that specifies aria-rowcount (2) and aria-colcount (3)
  ComPtr<IGridProvider> grid3_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[2]);

  // Grid that specifies aria-rowcount and aria-colcount are both (-1)
  ComPtr<IGridProvider> grid4_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()->children()[3]);

  int column_count;

  EXPECT_HRESULT_SUCCEEDED(grid1_provider->get_ColumnCount(&column_count));
  EXPECT_EQ(column_count, 0);

  EXPECT_HRESULT_SUCCEEDED(grid2_provider->get_ColumnCount(&column_count));
  EXPECT_EQ(column_count, 5);

  EXPECT_HRESULT_SUCCEEDED(grid3_provider->get_ColumnCount(&column_count));
  EXPECT_EQ(column_count, 3);

  EXPECT_EQ(E_UNEXPECTED, grid4_provider->get_ColumnCount(&column_count));
}

TEST_F(AXPlatformNodeWinTest, IGridProviderGetItem) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kGrid;
  root.AddIntAttribute(ax::mojom::IntAttribute::kAriaRowCount, 1);
  root.AddIntAttribute(ax::mojom::IntAttribute::kAriaColumnCount, 1);

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData cell1;
  cell1.id = 3;
  cell1.role = ax::mojom::Role::kCell;
  row1.child_ids.push_back(cell1.id);

  Init(root, row1, cell1);

  ComPtr<IGridProvider> root_igridprovider(
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode()));

  ComPtr<IRawElementProviderSimple> cell1_irawelementprovidersimple(
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[0]->children()[0]));

  IRawElementProviderSimple* grid_item = nullptr;
  EXPECT_HRESULT_SUCCEEDED(root_igridprovider->GetItem(0, 0, &grid_item));
  EXPECT_NE(nullptr, grid_item);
  EXPECT_EQ(cell1_irawelementprovidersimple.Get(), grid_item);
}

TEST_F(AXPlatformNodeWinTest, ITableProviderGetColumnHeaders) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData column_header;
  column_header.id = 3;
  column_header.role = ax::mojom::Role::kColumnHeader;
  column_header.SetName(u"column_header");
  row1.child_ids.push_back(column_header.id);

  AXNodeData row_header;
  row_header.id = 4;
  row_header.role = ax::mojom::Role::kRowHeader;
  row_header.SetName(u"row_header");
  row1.child_ids.push_back(row_header.id);

  Init(root, row1, column_header, row_header);

  ComPtr<ITableProvider> root_itableprovider(
      QueryInterfaceFromNode<ITableProvider>(GetRootAsAXNode()));

  base::win::ScopedSafearray safearray;
  EXPECT_HRESULT_SUCCEEDED(
      root_itableprovider->GetColumnHeaders(safearray.Receive()));
  EXPECT_NE(nullptr, safearray.Get());

  std::vector<std::wstring> expected_names = {L"column_header"};
  EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(safearray.Get(), UIA_NamePropertyId,
                                   expected_names);

  // Remove column_header's native event target and verify it's no longer
  // returned.
  TestAXNodeWrapper* column_header_wrapper = TestAXNodeWrapper::GetOrCreate(
      GetTree(), GetRootAsAXNode()->children()[0]->children()[0]);
  column_header_wrapper->ResetNativeEventTarget();

  safearray.Release();
  EXPECT_HRESULT_SUCCEEDED(
      root_itableprovider->GetColumnHeaders(safearray.Receive()));
  EXPECT_EQ(nullptr, safearray.Get());
}

TEST_F(AXPlatformNodeWinTest, ITableProviderGetColumnHeadersMultipleHeaders) {
  // Build a table like this:
  //   header_r1c1  | header_r1c2 | header_r1c3
  //    cell_r2c1   | cell_r2c2   | cell_r2c3
  //    cell_r3c1   | header_r3c2 |

  // <table>
  //   <tr aria-label="row1">
  //     <th>header_r1c1</th>
  //     <th>header_r1c2</th>
  //     <th>header_r1c3</th>
  //   </tr>
  //   <tr aria-label="row2">
  //     <td>cell_r2c1</td>
  //     <td>cell_r2c2</td>
  //     <td>cell_r2c3</td>
  //   </tr>
  //   <tr aria-label="row3">
  //     <td>cell_r3c1</td>
  //     <th>header_r3c2</th>
  //   </tr>
  // </table>

  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData row2;
  row2.id = 3;
  row2.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row2.id);

  AXNodeData row3;
  row3.id = 4;
  row3.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row3.id);

  // <tr aria-label="row1">
  //   <th>header_r1c1</th> <th>header_r1c2</th> <th>header_r1c3</th>
  // </tr>
  AXNodeData header_r1c1;
  header_r1c1.id = 5;
  header_r1c1.role = ax::mojom::Role::kColumnHeader;
  header_r1c1.SetName(u"header_r1c1");
  row1.child_ids.push_back(header_r1c1.id);

  AXNodeData header_r1c2;
  header_r1c2.id = 6;
  header_r1c2.role = ax::mojom::Role::kColumnHeader;
  header_r1c2.SetName(u"header_r1c2");
  row1.child_ids.push_back(header_r1c2.id);

  AXNodeData header_r1c3;
  header_r1c3.id = 7;
  header_r1c3.role = ax::mojom::Role::kColumnHeader;
  header_r1c3.SetName(u"header_r1c3");
  row1.child_ids.push_back(header_r1c3.id);

  // <tr aria-label="row2">
  //   <td>cell_r2c1</td> <td>cell_r2c2</td> <td>cell_r2c3</td>
  // </tr>
  AXNodeData cell_r2c1;
  cell_r2c1.id = 8;
  cell_r2c1.role = ax::mojom::Role::kCell;
  cell_r2c1.SetName(u"cell_r2c1");
  row2.child_ids.push_back(cell_r2c1.id);

  AXNodeData cell_r2c2;
  cell_r2c2.id = 9;
  cell_r2c2.role = ax::mojom::Role::kCell;
  cell_r2c2.SetName(u"cell_r2c2");
  row2.child_ids.push_back(cell_r2c2.id);

  AXNodeData cell_r2c3;
  cell_r2c3.id = 10;
  cell_r2c3.role = ax::mojom::Role::kCell;
  cell_r2c3.SetName(u"cell_r2c3");
  row2.child_ids.push_back(cell_r2c3.id);

  // <tr aria-label="row3">
  //   <td>cell_r3c1</td> <th>header_r3c2</th>
  // </tr>
  AXNodeData cell_r3c1;
  cell_r3c1.id = 11;
  cell_r3c1.role = ax::mojom::Role::kCell;
  cell_r3c1.SetName(u"cell_r3c1");
  row3.child_ids.push_back(cell_r3c1.id);

  AXNodeData header_r3c2;
  header_r3c2.id = 12;
  header_r3c2.role = ax::mojom::Role::kColumnHeader;
  header_r3c2.SetName(u"header_r3c2");
  row3.child_ids.push_back(header_r3c2.id);

  Init(root, row1, row2, row3, header_r1c1, header_r1c2, header_r1c3, cell_r2c1,
       cell_r2c2, cell_r2c3, cell_r3c1, header_r3c2);

  ComPtr<ITableProvider> root_itableprovider(
      QueryInterfaceFromNode<ITableProvider>(GetRootAsAXNode()));

  base::win::ScopedSafearray safearray;
  EXPECT_HRESULT_SUCCEEDED(
      root_itableprovider->GetColumnHeaders(safearray.Receive()));
  EXPECT_NE(nullptr, safearray.Get());

  // Validate that we retrieve all column headers of the table and in the order
  // below.
  std::vector<std::wstring> expected_names = {L"header_r1c1", L"header_r1c2",
                                              L"header_r3c2", L"header_r1c3"};
  EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(safearray.Get(), UIA_NamePropertyId,
                                   expected_names);
}

TEST_F(AXPlatformNodeWinTest, ITableProviderGetRowHeaders) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData column_header;
  column_header.id = 3;
  column_header.role = ax::mojom::Role::kColumnHeader;
  column_header.SetName(u"column_header");
  row1.child_ids.push_back(column_header.id);

  AXNodeData row_header;
  row_header.id = 4;
  row_header.role = ax::mojom::Role::kRowHeader;
  row_header.SetName(u"row_header");
  row1.child_ids.push_back(row_header.id);

  Init(root, row1, column_header, row_header);

  ComPtr<ITableProvider> root_itableprovider(
      QueryInterfaceFromNode<ITableProvider>(GetRootAsAXNode()));

  base::win::ScopedSafearray safearray;
  EXPECT_HRESULT_SUCCEEDED(
      root_itableprovider->GetRowHeaders(safearray.Receive()));
  EXPECT_NE(nullptr, safearray.Get());
  std::vector<std::wstring> expected_names = {L"row_header"};
  EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(safearray.Get(), UIA_NamePropertyId,
                                   expected_names);

  // Remove row_header's native event target and verify it's no longer returned.
  TestAXNodeWrapper* row_header_wrapper = TestAXNodeWrapper::GetOrCreate(
      GetTree(), GetRootAsAXNode()->children()[0]->children()[1]);
  row_header_wrapper->ResetNativeEventTarget();

  safearray.Release();
  EXPECT_HRESULT_SUCCEEDED(
      root_itableprovider->GetRowHeaders(safearray.Receive()));
  EXPECT_EQ(nullptr, safearray.Get());
}

TEST_F(AXPlatformNodeWinTest, ITableProviderGetRowOrColumnMajor) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  Init(root);

  ComPtr<ITableProvider> root_itableprovider(
      QueryInterfaceFromNode<ITableProvider>(GetRootAsAXNode()));

  RowOrColumnMajor row_or_column_major;
  EXPECT_HRESULT_SUCCEEDED(
      root_itableprovider->get_RowOrColumnMajor(&row_or_column_major));
  EXPECT_EQ(row_or_column_major, RowOrColumnMajor_RowMajor);
}

TEST_F(AXPlatformNodeWinTest, ITableItemProviderGetColumnHeaderItems) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData column_header_1;
  column_header_1.id = 3;
  column_header_1.role = ax::mojom::Role::kColumnHeader;
  column_header_1.SetName(u"column_header_1");
  row1.child_ids.push_back(column_header_1.id);

  AXNodeData column_header_2;
  column_header_2.id = 4;
  column_header_2.role = ax::mojom::Role::kColumnHeader;
  column_header_2.SetName(u"column_header_2");
  row1.child_ids.push_back(column_header_2.id);

  AXNodeData row2;
  row2.id = 5;
  row2.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row2.id);

  AXNodeData cell;
  cell.id = 6;
  cell.role = ax::mojom::Role::kCell;
  row2.child_ids.push_back(cell.id);

  Init(root, row1, column_header_1, column_header_2, row2, cell);

  TestAXNodeWrapper* root_wrapper =
      TestAXNodeWrapper::GetOrCreate(GetTree(), GetRootAsAXNode());
  root_wrapper->BuildAllWrappers(GetTree(), GetRootAsAXNode());

  ComPtr<ITableItemProvider> cell_itableitemprovider(
      QueryInterfaceFromNode<ITableItemProvider>(
          GetRootAsAXNode()->children()[1]->children()[0]));

  base::win::ScopedSafearray safearray;
  EXPECT_HRESULT_SUCCEEDED(
      cell_itableitemprovider->GetColumnHeaderItems(safearray.Receive()));
  EXPECT_NE(nullptr, safearray.Get());

  std::vector<std::wstring> expected_names = {L"column_header_1"};
  EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(safearray.Get(), UIA_NamePropertyId,
                                   expected_names);

  // Remove column_header_1's native event target and verify it's no longer
  // returned.
  TestAXNodeWrapper* column_header_wrapper = TestAXNodeWrapper::GetOrCreate(
      GetTree(), GetRootAsAXNode()->children()[0]->children()[0]);
  column_header_wrapper->ResetNativeEventTarget();

  safearray.Release();
  EXPECT_HRESULT_SUCCEEDED(
      cell_itableitemprovider->GetColumnHeaderItems(safearray.Receive()));
  EXPECT_EQ(nullptr, safearray.Get());
}

TEST_F(AXPlatformNodeWinTest, ITableItemProviderGetRowHeaderItems) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData row_header_1;
  row_header_1.id = 3;
  row_header_1.role = ax::mojom::Role::kRowHeader;
  row_header_1.SetName(u"row_header_1");
  row1.child_ids.push_back(row_header_1.id);

  AXNodeData cell;
  cell.id = 4;
  cell.role = ax::mojom::Role::kCell;
  row1.child_ids.push_back(cell.id);

  AXNodeData row2;
  row2.id = 5;
  row2.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row2.id);

  AXNodeData row_header_2;
  row_header_2.id = 6;
  row_header_2.role = ax::mojom::Role::kRowHeader;
  row_header_2.SetName(u"row_header_2");
  row2.child_ids.push_back(row_header_2.id);

  Init(root, row1, row_header_1, cell, row2, row_header_2);

  TestAXNodeWrapper* root_wrapper =
      TestAXNodeWrapper::GetOrCreate(GetTree(), GetRootAsAXNode());
  root_wrapper->BuildAllWrappers(GetTree(), GetRootAsAXNode());

  ComPtr<ITableItemProvider> cell_itableitemprovider(
      QueryInterfaceFromNode<ITableItemProvider>(
          GetRootAsAXNode()->children()[0]->children()[1]));

  base::win::ScopedSafearray safearray;
  EXPECT_HRESULT_SUCCEEDED(
      cell_itableitemprovider->GetRowHeaderItems(safearray.Receive()));
  EXPECT_NE(nullptr, safearray.Get());
  std::vector<std::wstring> expected_names = {L"row_header_1"};
  EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(safearray.Get(), UIA_NamePropertyId,
                                   expected_names);

  // Remove row_header_1's native event target and verify it's no longer
  // returned.
  TestAXNodeWrapper* row_header_wrapper = TestAXNodeWrapper::GetOrCreate(
      GetTree(), GetRootAsAXNode()->children()[0]->children()[0]);
  row_header_wrapper->ResetNativeEventTarget();

  safearray.Release();
  EXPECT_HRESULT_SUCCEEDED(
      cell_itableitemprovider->GetRowHeaderItems(safearray.Receive()));
  EXPECT_EQ(nullptr, safearray.Get());
}

TEST_F(AXPlatformNodeWinTest, UIAGetPropertySimple) {
  AXNodeData root;
  root.role = ax::mojom::Role::kList;
  root.SetName("fake name");
  root.AddStringAttribute(ax::mojom::StringAttribute::kAccessKey, "Ctrl+Q");
  root.AddStringAttribute(ax::mojom::StringAttribute::kLanguage, "en-us");
  root.AddStringAttribute(ax::mojom::StringAttribute::kKeyShortcuts, "Alt+F4");
  root.AddStringAttribute(ax::mojom::StringAttribute::kDescription,
                          "fake description");
  root.AddIntAttribute(ax::mojom::IntAttribute::kSetSize, 2);
  root.AddIntAttribute(ax::mojom::IntAttribute::kInvalidState, 1);
  root.id = 1;

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kListItem;
  child1.AddIntAttribute(ax::mojom::IntAttribute::kPosInSet, 1);
  child1.SetName("child1");
  root.child_ids.push_back(child1.id);

  Init(root, child1);

  ComPtr<IRawElementProviderSimple> root_node =
      GetRootIRawElementProviderSimple();
  ScopedVariant uia_id;
  EXPECT_UIA_BSTR_EQ(root_node, UIA_AccessKeyPropertyId, L"Ctrl+Q");
  EXPECT_UIA_BSTR_EQ(root_node, UIA_AcceleratorKeyPropertyId, L"Alt+F4");
  ASSERT_HRESULT_SUCCEEDED(root_node->GetPropertyValue(
      UIA_AutomationIdPropertyId, uia_id.Receive()));
  EXPECT_UIA_BSTR_EQ(root_node, UIA_AutomationIdPropertyId,
                     uia_id.ptr()->bstrVal);
  EXPECT_UIA_BSTR_EQ(root_node, UIA_FullDescriptionPropertyId,
                     L"fake description");
  EXPECT_UIA_BSTR_EQ(root_node, UIA_AriaRolePropertyId, L"list");
  EXPECT_UIA_BSTR_EQ(root_node, UIA_AriaPropertiesPropertyId,
                     L"readonly=true;expanded=false;multiline=false;"
                     L"multiselectable=false;required=false;setsize=2");
  constexpr int en_us_lcid = 1033;
  EXPECT_UIA_INT_EQ(root_node, UIA_CulturePropertyId, en_us_lcid);
  EXPECT_UIA_BSTR_EQ(root_node, UIA_NamePropertyId, L"fake name");
  EXPECT_UIA_INT_EQ(root_node, UIA_ControlTypePropertyId,
                    int{UIA_ListControlTypeId});
  EXPECT_UIA_INT_EQ(root_node, UIA_OrientationPropertyId,
                    int{OrientationType_None});
  EXPECT_UIA_INT_EQ(root_node, UIA_SizeOfSetPropertyId, 2);
  EXPECT_UIA_INT_EQ(root_node, UIA_ToggleToggleStatePropertyId,
                    int{ToggleState_Off});
  EXPECT_UIA_BOOL_EQ(root_node, UIA_IsPasswordPropertyId, false);
  EXPECT_UIA_BOOL_EQ(root_node, UIA_IsEnabledPropertyId, true);
  EXPECT_UIA_BOOL_EQ(root_node, UIA_HasKeyboardFocusPropertyId, false);
  EXPECT_UIA_BOOL_EQ(root_node, UIA_IsRequiredForFormPropertyId, false);
  EXPECT_UIA_BOOL_EQ(root_node, UIA_IsDataValidForFormPropertyId, true);
  EXPECT_UIA_BOOL_EQ(root_node, UIA_IsKeyboardFocusablePropertyId, false);
  EXPECT_UIA_BOOL_EQ(root_node, UIA_IsOffscreenPropertyId, false);
  ComPtr<IRawElementProviderSimple> child_node1 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[0]);
  EXPECT_UIA_INT_EQ(child_node1, UIA_PositionInSetPropertyId, 1);
}

TEST_F(AXPlatformNodeWinTest, UIAGetPropertyValueClickablePoint) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kButton;
  root.relative_bounds.bounds = gfx::RectF(20, 30, 100, 200);
  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();

  // The clickable point of a rectangle {20, 30, 100, 200} is the rectangle's
  // center, with coordinates {x: 70, y: 130}.
  std::vector<double> expected_values = {70, 130};
  EXPECT_UIA_DOUBLE_ARRAY_EQ(raw_element_provider_simple,
                             UIA_ClickablePointPropertyId, expected_values);
}

TEST_F(AXPlatformNodeWinTest, UIAGetPropertyValueIsDialog) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.child_ids = {2, 3};

  AXNodeData alert_dialog;
  alert_dialog.id = 2;
  alert_dialog.role = ax::mojom::Role::kAlertDialog;

  AXNodeData dialog;
  dialog.id = 3;
  dialog.role = ax::mojom::Role::kDialog;

  Init(root, alert_dialog, dialog);

  EXPECT_UIA_BOOL_EQ(GetRootIRawElementProviderSimple(), UIA_IsDialogPropertyId,
                     false);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(0),
                     UIA_IsDialogPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(1),
                     UIA_IsDialogPropertyId, true);
}

TEST_F(AXPlatformNodeWinTest,
       UIAGetPropertyValueIsControlElementIgnoredInvisible) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.child_ids = {2, 3, 4, 5, 6, 7, 8};

  AXNodeData normal_button;
  normal_button.id = 2;
  normal_button.role = ax::mojom::Role::kButton;

  AXNodeData ignored_button;
  ignored_button.id = 3;
  ignored_button.role = ax::mojom::Role::kButton;
  ignored_button.AddState(ax::mojom::State::kIgnored);

  AXNodeData invisible_button;
  invisible_button.id = 4;
  invisible_button.role = ax::mojom::Role::kButton;
  invisible_button.AddState(ax::mojom::State::kInvisible);

  AXNodeData invisible_focusable_button;
  invisible_focusable_button.id = 5;
  invisible_focusable_button.role = ax::mojom::Role::kButton;
  invisible_focusable_button.AddState(ax::mojom::State::kInvisible);
  invisible_focusable_button.AddState(ax::mojom::State::kFocusable);

  AXNodeData focusable_generic_container;
  focusable_generic_container.id = 6;
  focusable_generic_container.role = ax::mojom::Role::kGenericContainer;
  focusable_generic_container.AddState(ax::mojom::State::kFocusable);

  AXNodeData ignored_focusable_generic_container;
  ignored_focusable_generic_container.id = 7;
  ignored_focusable_generic_container.role = ax::mojom::Role::kGenericContainer;
  ignored_focusable_generic_container.AddState(ax::mojom::State::kIgnored);
  focusable_generic_container.AddState(ax::mojom::State::kFocusable);

  AXNodeData invisible_focusable_generic_container;
  invisible_focusable_generic_container.id = 8;
  invisible_focusable_generic_container.role =
      ax::mojom::Role::kGenericContainer;
  invisible_focusable_generic_container.AddState(ax::mojom::State::kInvisible);
  invisible_focusable_generic_container.AddState(ax::mojom::State::kFocusable);

  Init(root, normal_button, ignored_button, invisible_button,
       invisible_focusable_button, focusable_generic_container,
       ignored_focusable_generic_container,
       invisible_focusable_generic_container);

  // Turn on web content mode for the AXTree.
  TestAXNodeWrapper::SetGlobalIsWebContent(true);

  // Normal button (id=2), no invisible or ignored state set. Should be a
  // control element.
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(0),
                     UIA_IsControlElementPropertyId, true);

  // Button with ignored state (id=3). Should not be a control element.
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(1),
                     UIA_IsControlElementPropertyId, false);

  // Button with invisible state (id=4). Should not be a control element.
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(2),
                     UIA_IsControlElementPropertyId, false);

  // Button with invisible state, but focusable (id=5). Should not be a control
  // element.
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(3),
                     UIA_IsControlElementPropertyId, false);

  // Generic container, focusable (id=6). Should be a control
  // element.
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(4),
                     UIA_IsControlElementPropertyId, true);

  // Generic container, ignored but focusable (id=7). Should not be a control
  // element.
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(5),
                     UIA_IsControlElementPropertyId, false);

  // Generic container, invisible and ignored, but focusable (id=8). Should not
  // be a control element.
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromChildIndex(6),
                     UIA_IsControlElementPropertyId, false);
}

TEST_F(AXPlatformNodeWinTest, UIAGetControllerForPropertyId) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.child_ids = {2, 3, 4};

  AXNodeData tab;
  tab.id = 2;
  tab.role = ax::mojom::Role::kTab;
  tab.SetName("tab");
  std::vector<AXNode::AXID> controller_ids = {3, 4};
  tab.AddIntListAttribute(ax::mojom::IntListAttribute::kControlsIds,
                          controller_ids);

  AXNodeData panel1;
  panel1.id = 3;
  panel1.role = ax::mojom::Role::kTabPanel;
  panel1.SetName("panel1");

  AXNodeData panel2;
  panel2.id = 4;
  panel2.role = ax::mojom::Role::kTabPanel;
  panel2.SetName("panel2");

  Init(root, tab, panel1, panel2);
  TestAXNodeWrapper* root_wrapper =
      TestAXNodeWrapper::GetOrCreate(GetTree(), GetRootAsAXNode());
  root_wrapper->BuildAllWrappers(GetTree(), GetRootAsAXNode());

  ComPtr<IRawElementProviderSimple> tab_node =
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[0]);

  std::vector<std::wstring> expected_names_1 = {L"panel1", L"panel2"};
  EXPECT_UIA_PROPERTY_ELEMENT_ARRAY_BSTR_EQ(
      tab_node, UIA_ControllerForPropertyId, UIA_NamePropertyId,
      expected_names_1);

  // Remove panel1's native event target and verify it's no longer returned.
  TestAXNodeWrapper* panel1_wrapper = TestAXNodeWrapper::GetOrCreate(
      GetTree(), GetRootAsAXNode()->children()[1]);
  panel1_wrapper->ResetNativeEventTarget();
  std::vector<std::wstring> expected_names_2 = {L"panel2"};
  EXPECT_UIA_PROPERTY_ELEMENT_ARRAY_BSTR_EQ(
      tab_node, UIA_ControllerForPropertyId, UIA_NamePropertyId,
      expected_names_2);
}

TEST_F(AXPlatformNodeWinTest, UIAGetDescribedByPropertyId) {
  AXNodeData root;
  std::vector<AXNode::AXID> describedby_ids = {2, 3, 4};
  root.AddIntListAttribute(ax::mojom::IntListAttribute::kDescribedbyIds,
                           describedby_ids);
  root.id = 1;
  root.role = ax::mojom::Role::kMarquee;
  root.SetName("root");

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kStaticText;
  child1.SetName("child1");

  root.child_ids.push_back(child1.id);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kStaticText;
  child2.SetName("child2");

  root.child_ids.push_back(child2.id);

  Init(root, child1, child2);

  ComPtr<IRawElementProviderSimple> root_node =
      GetRootIRawElementProviderSimple();

  std::vector<std::wstring> expected_names = {L"child1", L"child2"};
  EXPECT_UIA_PROPERTY_ELEMENT_ARRAY_BSTR_EQ(
      root_node, UIA_DescribedByPropertyId, UIA_NamePropertyId, expected_names);
}

TEST_F(AXPlatformNodeWinTest, UIAItemStatusPropertyId) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  row1.AddIntAttribute(ax::mojom::IntAttribute::kSortDirection,
                       static_cast<int>(ax::mojom::SortDirection::kAscending));
  root.child_ids.push_back(row1.id);

  AXNodeData header1;
  header1.id = 3;
  header1.role = ax::mojom::Role::kRowHeader;
  header1.AddIntAttribute(
      ax::mojom::IntAttribute::kSortDirection,
      static_cast<int>(ax::mojom::SortDirection::kAscending));
  row1.child_ids.push_back(header1.id);

  AXNodeData header2;
  header2.id = 4;
  header2.role = ax::mojom::Role::kColumnHeader;
  header2.AddIntAttribute(
      ax::mojom::IntAttribute::kSortDirection,
      static_cast<int>(ax::mojom::SortDirection::kDescending));
  row1.child_ids.push_back(header2.id);

  AXNodeData header3;
  header3.id = 5;
  header3.role = ax::mojom::Role::kColumnHeader;
  header3.AddIntAttribute(ax::mojom::IntAttribute::kSortDirection,
                          static_cast<int>(ax::mojom::SortDirection::kOther));
  row1.child_ids.push_back(header3.id);

  AXNodeData header4;
  header4.id = 6;
  header4.role = ax::mojom::Role::kColumnHeader;
  header4.AddIntAttribute(
      ax::mojom::IntAttribute::kSortDirection,
      static_cast<int>(ax::mojom::SortDirection::kUnsorted));
  row1.child_ids.push_back(header4.id);

  Init(root, row1, header1, header2, header3, header4);

  auto* row_node = GetRootAsAXNode()->children()[0];

  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         row_node->children()[0]),
                     UIA_ItemStatusPropertyId, L"ascending");

  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         row_node->children()[1]),
                     UIA_ItemStatusPropertyId, L"descending");

  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         row_node->children()[2]),
                     UIA_ItemStatusPropertyId, L"other");

  EXPECT_UIA_VALUE_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                          row_node->children()[3]),
                      UIA_ItemStatusPropertyId, ScopedVariant::kEmptyVariant);

  EXPECT_UIA_VALUE_EQ(
      QueryInterfaceFromNode<IRawElementProviderSimple>(row_node),
      UIA_ItemStatusPropertyId, ScopedVariant::kEmptyVariant);
}

TEST_F(AXPlatformNodeWinTest, UIAGetFlowsToPropertyId) {
  AXNodeData root;
  std::vector<AXNode::AXID> flowto_ids = {2, 3, 4};
  root.AddIntListAttribute(ax::mojom::IntListAttribute::kFlowtoIds, flowto_ids);
  root.id = 1;
  root.role = ax::mojom::Role::kMarquee;
  root.SetName("root");

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kStaticText;
  child1.SetName("child1");

  root.child_ids.push_back(child1.id);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kStaticText;
  child2.SetName("child2");

  root.child_ids.push_back(child2.id);

  Init(root, child1, child2);

  ComPtr<IRawElementProviderSimple> root_node =
      GetRootIRawElementProviderSimple();
  std::vector<std::wstring> expected_names = {L"child1", L"child2"};
  EXPECT_UIA_PROPERTY_ELEMENT_ARRAY_BSTR_EQ(root_node, UIA_FlowsToPropertyId,
                                            UIA_NamePropertyId, expected_names);
}

TEST_F(AXPlatformNodeWinTest, UIAGetPropertyValueFlowsFromNone) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("root");

  Init(root);

  ComPtr<IRawElementProviderSimple> root_node =
      GetRootIRawElementProviderSimple();

  ScopedVariant property_value;
  EXPECT_HRESULT_SUCCEEDED(root_node->GetPropertyValue(
      UIA_FlowsFromPropertyId, property_value.Receive()));
  EXPECT_EQ(VT_ARRAY | VT_UNKNOWN, property_value.type());
  EXPECT_EQ(nullptr, V_ARRAY(property_value.ptr()));
}

TEST_F(AXPlatformNodeWinTest, UIAGetPropertyValueFlowsFromSingle) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("root");
  root.AddIntListAttribute(ax::mojom::IntListAttribute::kFlowtoIds, {2});

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kGenericContainer;
  child1.SetName("child1");
  root.child_ids.push_back(child1.id);

  Init(root, child1);
  ASSERT_NE(nullptr,
            TestAXNodeWrapper::GetOrCreate(GetTree(), GetRootAsAXNode()));

  ComPtr<IRawElementProviderSimple> child_node1 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[0]);
  std::vector<std::wstring> expected_names = {L"root"};
  EXPECT_UIA_PROPERTY_ELEMENT_ARRAY_BSTR_EQ(
      child_node1, UIA_FlowsFromPropertyId, UIA_NamePropertyId, expected_names);
}

TEST_F(AXPlatformNodeWinTest, UIAGetPropertyValueFlowsFromMultiple) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("root");
  root.AddIntListAttribute(ax::mojom::IntListAttribute::kFlowtoIds, {2, 3});

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kGenericContainer;
  child1.SetName("child1");
  child1.AddIntListAttribute(ax::mojom::IntListAttribute::kFlowtoIds, {3});
  root.child_ids.push_back(child1.id);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kGenericContainer;
  child2.SetName("child2");
  root.child_ids.push_back(child2.id);

  Init(root, child1, child2);
  ASSERT_NE(nullptr,
            TestAXNodeWrapper::GetOrCreate(GetTree(), GetRootAsAXNode()));
  ASSERT_NE(nullptr, TestAXNodeWrapper::GetOrCreate(
                         GetTree(), GetRootAsAXNode()->children()[0]));

  ComPtr<IRawElementProviderSimple> child_node2 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[1]);
  std::vector<std::wstring> expected_names_1 = {L"root", L"child1"};
  EXPECT_UIA_PROPERTY_UNORDERED_ELEMENT_ARRAY_BSTR_EQ(
      child_node2, UIA_FlowsFromPropertyId, UIA_NamePropertyId,
      expected_names_1);

  // Remove child1's native event target and verify it's no longer returned.
  TestAXNodeWrapper* child1_wrapper = TestAXNodeWrapper::GetOrCreate(
      GetTree(), GetRootAsAXNode()->children()[0]);
  child1_wrapper->ResetNativeEventTarget();
  std::vector<std::wstring> expected_names_2 = {L"root"};
  EXPECT_UIA_PROPERTY_UNORDERED_ELEMENT_ARRAY_BSTR_EQ(
      child_node2, UIA_FlowsFromPropertyId, UIA_NamePropertyId,
      expected_names_2);
}

TEST_F(AXPlatformNodeWinTest, UIAGetPropertyValueFrameworkId) {
  AXNodeData root_ax_node_data;
  root_ax_node_data.id = 1;
  root_ax_node_data.role = ax::mojom::Role::kRootWebArea;
  Init(root_ax_node_data);

  ComPtr<IRawElementProviderSimple> root_raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  EXPECT_UIA_BSTR_EQ(root_raw_element_provider_simple,
                     UIA_FrameworkIdPropertyId, L"Chrome");
}

TEST_F(AXPlatformNodeWinTest, GetPropertyValue_LabeledByTest) {
  // ++1 root
  // ++++2 kGenericContainer LabeledBy 3
  // ++++++3 kStaticText "Hello"
  // ++++4 kGenericContainer LabeledBy 5
  // ++++++5 kGenericContainer
  // ++++++++6 kStaticText "3.14"
  // ++++7 kAlert LabeledBy 6
  AXNodeData root_1;
  AXNodeData gc_2;
  AXNodeData static_text_3;
  AXNodeData gc_4;
  AXNodeData gc_5;
  AXNodeData static_text_6;
  AXNodeData alert_7;

  root_1.id = 1;
  gc_2.id = 2;
  static_text_3.id = 3;
  gc_4.id = 4;
  gc_5.id = 5;
  static_text_6.id = 6;
  alert_7.id = 7;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {gc_2.id, gc_4.id, alert_7.id};

  gc_2.role = ax::mojom::Role::kGenericContainer;
  gc_2.AddIntListAttribute(ax::mojom::IntListAttribute::kLabelledbyIds,
                           {static_text_3.id});
  gc_2.child_ids = {static_text_3.id};

  static_text_3.role = ax::mojom::Role::kStaticText;
  static_text_3.SetName("Hello");

  gc_4.role = ax::mojom::Role::kGenericContainer;
  gc_4.AddIntListAttribute(ax::mojom::IntListAttribute::kLabelledbyIds,
                           {gc_5.id});
  gc_4.child_ids = {gc_5.id};

  gc_5.role = ax::mojom::Role::kGenericContainer;
  gc_5.child_ids = {static_text_6.id};

  static_text_6.role = ax::mojom::Role::kStaticText;
  static_text_6.SetName("3.14");

  alert_7.role = ax::mojom::Role::kAlert;
  alert_7.AddIntListAttribute(ax::mojom::IntListAttribute::kLabelledbyIds,
                              {static_text_6.id});

  Init(root_1, gc_2, static_text_3, gc_4, gc_5, static_text_6, alert_7);

  AXNode* root_node = GetRootAsAXNode();
  AXNode* gc_2_node = root_node->children()[0];
  AXNode* static_text_3_node = gc_2_node->children()[0];
  AXNode* gc_4_node = root_node->children()[1];
  AXNode* static_text_6_node = gc_4_node->children()[0]->children()[0];
  AXNode* alert_7_node = root_node->children()[2];

  // Case 1: |gc_2| is labeled by |static_text_3|.

  ComPtr<IRawElementProviderSimple> gc_2_provider =
      GetIRawElementProviderSimpleFromTree(gc_2_node->tree()->GetAXTreeID(),
                                           gc_2_node->id());
  ScopedVariant property_value;
  EXPECT_EQ(S_OK, gc_2_provider->GetPropertyValue(UIA_LabeledByPropertyId,
                                                  property_value.Receive()));
  ASSERT_EQ(property_value.type(), VT_UNKNOWN);
  ComPtr<IRawElementProviderSimple> static_text_3_provider;
  EXPECT_EQ(S_OK, property_value.ptr()->punkVal->QueryInterface(
                      IID_PPV_ARGS(&static_text_3_provider)));
  EXPECT_UIA_BSTR_EQ(static_text_3_provider, UIA_NamePropertyId, L"Hello");

  // Case 2: |gc_4| is labeled by |gc_5| and should return the first static text
  // child of that node, which is |static_text_6|.

  ComPtr<IRawElementProviderSimple> gc_4_provider =
      GetIRawElementProviderSimpleFromTree(gc_4_node->tree()->GetAXTreeID(),
                                           gc_4_node->id());
  property_value.Reset();
  EXPECT_EQ(S_OK, gc_4_provider->GetPropertyValue(UIA_LabeledByPropertyId,
                                                  property_value.Receive()));
  ASSERT_EQ(property_value.type(), VT_UNKNOWN);
  ComPtr<IRawElementProviderSimple> static_text_6_provider;
  EXPECT_EQ(S_OK, property_value.ptr()->punkVal->QueryInterface(
                      IID_PPV_ARGS(&static_text_6_provider)));
  EXPECT_UIA_BSTR_EQ(static_text_6_provider, UIA_NamePropertyId, L"3.14");

  // Case 3: Some UIA control types always expect an empty value for this
  // property. The role kAlert corresponds to UIA_TextControlTypeId, which
  // always expects an empty value. |alert_7| is marked as labeled by
  // |static_text_6|, but shouldn't expose it to the UIA_LabeledByPropertyId.

  ComPtr<IRawElementProviderSimple> alert_7_provider =
      GetIRawElementProviderSimpleFromTree(alert_7_node->tree()->GetAXTreeID(),
                                           alert_7_node->id());
  property_value.Reset();
  EXPECT_EQ(S_OK, alert_7_provider->GetPropertyValue(UIA_LabeledByPropertyId,
                                                     property_value.Receive()));
  ASSERT_EQ(property_value.type(), VT_EMPTY);

  // Remove the referenced nodes' native event targets and verify it's no longer
  // returned.

  // Case 1.
  TestAXNodeWrapper* static_text_3_node_wrapper =
      TestAXNodeWrapper::GetOrCreate(GetTree(), static_text_3_node);
  static_text_3_node_wrapper->ResetNativeEventTarget();

  property_value.Reset();
  EXPECT_EQ(S_OK, gc_2_provider->GetPropertyValue(UIA_LabeledByPropertyId,
                                                  property_value.Receive()));
  EXPECT_EQ(property_value.type(), VT_EMPTY);

  // Case 2.
  TestAXNodeWrapper* static_text_6_node_wrapper =
      TestAXNodeWrapper::GetOrCreate(GetTree(), static_text_6_node);
  static_text_6_node_wrapper->ResetNativeEventTarget();

  property_value.Reset();
  EXPECT_EQ(S_OK, gc_4_provider->GetPropertyValue(UIA_LabeledByPropertyId,
                                                  property_value.Receive()));
  EXPECT_EQ(property_value.type(), VT_EMPTY);
}

TEST_F(AXPlatformNodeWinTest, GetPropertyValue_HelpText) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;

  // Test Placeholder StringAttribute is exposed
  AXNodeData input1;
  input1.id = 2;
  input1.role = ax::mojom::Role::kTextField;
  input1.SetName("name-from-title");
  input1.AddIntAttribute(ax::mojom::IntAttribute::kNameFrom,
                         static_cast<int>(ax::mojom::NameFrom::kTitle));
  input1.AddStringAttribute(ax::mojom::StringAttribute::kPlaceholder,
                            "placeholder");
  root.child_ids.push_back(input1.id);

  // Test NameFrom Title is exposed
  AXNodeData input2;
  input2.id = 3;
  input2.role = ax::mojom::Role::kTextField;
  input2.SetName("name-from-title");
  input2.AddIntAttribute(ax::mojom::IntAttribute::kNameFrom,
                         static_cast<int>(ax::mojom::NameFrom::kTitle));
  root.child_ids.push_back(input2.id);

  // Test NameFrom Placeholder is exposed
  AXNodeData input3;
  input3.id = 4;
  input3.role = ax::mojom::Role::kTextField;
  input3.SetName("name-from-placeholder");
  input3.AddIntAttribute(ax::mojom::IntAttribute::kNameFrom,
                         static_cast<int>(ax::mojom::NameFrom::kPlaceholder));
  root.child_ids.push_back(input3.id);

  // Test Title StringAttribute is exposed
  AXNodeData input4;
  input4.id = 5;
  input4.role = ax::mojom::Role::kTextField;
  input4.SetName("name-from-attribute");
  input4.AddIntAttribute(ax::mojom::IntAttribute::kNameFrom,
                         static_cast<int>(ax::mojom::NameFrom::kAttribute));
  input4.AddStringAttribute(ax::mojom::StringAttribute::kTooltip, "tooltip");
  root.child_ids.push_back(input4.id);

  // Test NameFrom (other), without explicit
  // Title / Placeholder StringAttribute is not exposed
  AXNodeData input5;
  input5.id = 6;
  input5.role = ax::mojom::Role::kTextField;
  input5.SetName("name-from-attribute");
  input5.AddIntAttribute(ax::mojom::IntAttribute::kNameFrom,
                         static_cast<int>(ax::mojom::NameFrom::kAttribute));
  root.child_ids.push_back(input5.id);

  Init(root, input1, input2, input3, input4, input5);

  auto* root_node = GetRootAsAXNode();
  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         root_node->children()[0]),
                     UIA_HelpTextPropertyId, L"placeholder");
  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         root_node->children()[1]),
                     UIA_HelpTextPropertyId, L"name-from-title");
  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         root_node->children()[2]),
                     UIA_HelpTextPropertyId, L"name-from-placeholder");
  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         root_node->children()[3]),
                     UIA_HelpTextPropertyId, L"tooltip");
  EXPECT_UIA_VALUE_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                          root_node->children()[4]),
                      UIA_HelpTextPropertyId, ScopedVariant::kEmptyVariant);
}

TEST_F(AXPlatformNodeWinTest, GetPropertyValue_LocalizedControlType) {
  AXNodeData root;
  root.role = ax::mojom::Role::kUnknown;
  root.id = 1;
  root.AddStringAttribute(ax::mojom::StringAttribute::kRoleDescription,
                          "root role description");

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kSearchBox;
  child1.AddStringAttribute(ax::mojom::StringAttribute::kRoleDescription,
                            "child1 role description");
  root.child_ids.push_back(2);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kSearchBox;
  root.child_ids.push_back(3);

  Init(root, child1, child2);

  ComPtr<IRawElementProviderSimple> root_node =
      GetRootIRawElementProviderSimple();
  EXPECT_UIA_BSTR_EQ(root_node, UIA_LocalizedControlTypePropertyId,
                     L"root role description");
  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         GetRootAsAXNode()->children()[0]),
                     UIA_LocalizedControlTypePropertyId,
                     L"child1 role description");
  EXPECT_UIA_BSTR_EQ(QueryInterfaceFromNode<IRawElementProviderSimple>(
                         GetRootAsAXNode()->children()[1]),
                     UIA_LocalizedControlTypePropertyId, L"search box");
}

TEST_F(AXPlatformNodeWinTest, GetPropertyValue_IsControlElement) {
  AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.root_id = 1;
  update.nodes.resize(17);
  update.nodes[0].id = 1;
  update.nodes[0].role = ax::mojom::Role::kRootWebArea;
  update.nodes[0].child_ids = {2,  4,  6,  7,  8,  9,  10,
                               11, 12, 13, 14, 15, 16, 17};
  update.nodes[1].id = 2;
  update.nodes[1].role = ax::mojom::Role::kButton;
  update.nodes[1].child_ids = {3};
  update.nodes[2].id = 3;
  update.nodes[2].role = ax::mojom::Role::kStaticText;
  update.nodes[2].SetName("some text");
  update.nodes[3].id = 4;
  update.nodes[3].role = ax::mojom::Role::kGenericContainer;
  update.nodes[3].child_ids = {5};
  update.nodes[4].id = 5;
  update.nodes[4].role = ax::mojom::Role::kStaticText;
  update.nodes[4].SetName("more text");
  update.nodes[5].id = 6;
  update.nodes[5].role = ax::mojom::Role::kTable;
  update.nodes[6].id = 7;
  update.nodes[6].role = ax::mojom::Role::kList;
  update.nodes[7].id = 8;
  update.nodes[7].role = ax::mojom::Role::kForm;
  update.nodes[8].id = 9;
  update.nodes[8].role = ax::mojom::Role::kImage;
  update.nodes[9].id = 10;
  update.nodes[9].role = ax::mojom::Role::kImage;
  update.nodes[9].SetNameExplicitlyEmpty();
  update.nodes[10].id = 11;
  update.nodes[10].role = ax::mojom::Role::kArticle;
  update.nodes[11].id = 12;
  update.nodes[11].role = ax::mojom::Role::kGenericContainer;
  update.nodes[11].AddBoolAttribute(ax::mojom::BoolAttribute::kHasAriaAttribute,
                                    true);
  update.nodes[12].id = 13;
  update.nodes[12].role = ax::mojom::Role::kGenericContainer;
  update.nodes[12].AddBoolAttribute(ax::mojom::BoolAttribute::kEditableRoot,
                                    true);
  update.nodes[13].id = 14;
  update.nodes[13].role = ax::mojom::Role::kGenericContainer;
  update.nodes[13].SetName("name");
  update.nodes[14].id = 15;
  update.nodes[14].role = ax::mojom::Role::kGenericContainer;
  update.nodes[14].SetDescription("description");
  update.nodes[15].id = 16;
  update.nodes[15].role = ax::mojom::Role::kGenericContainer;
  update.nodes[15].AddState(ax::mojom::State::kFocusable);
  update.nodes[16].id = 17;
  update.nodes[16].role = ax::mojom::Role::kForm;
  update.nodes[16].SetName("name");

  Init(update);
  TestAXNodeWrapper::SetGlobalIsWebContent(true);

  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 2),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 3),
                     UIA_IsControlElementPropertyId, false);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 4),
                     UIA_IsControlElementPropertyId, false);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 5),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 6),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 7),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 8),
                     UIA_IsControlElementPropertyId, false);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 9),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 10),
                     UIA_IsControlElementPropertyId, false);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 11),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 12),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 13),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 14),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 15),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 16),
                     UIA_IsControlElementPropertyId, true);
  EXPECT_UIA_BOOL_EQ(GetIRawElementProviderSimpleFromTree(tree_id, 17),
                     UIA_IsControlElementPropertyId, true);
}

TEST_F(AXPlatformNodeWinTest, UIAGetProviderOptions) {
  AXNodeData root_data;
  root_data.id = 1;
  Init(root_data);

  ComPtr<IRawElementProviderSimple> root_node =
      GetRootIRawElementProviderSimple();

  ProviderOptions provider_options = static_cast<ProviderOptions>(0);
  EXPECT_HRESULT_SUCCEEDED(root_node->get_ProviderOptions(&provider_options));
  EXPECT_EQ(ProviderOptions_ServerSideProvider |
                ProviderOptions_UseComThreading |
                ProviderOptions_RefuseNonClientSupport |
                ProviderOptions_HasNativeIAccessible,
            provider_options);
}

TEST_F(AXPlatformNodeWinTest, UIAGetHostRawElementProvider) {
  AXNodeData root_data;
  root_data.id = 1;
  Init(root_data);

  ComPtr<IRawElementProviderSimple> root_node =
      GetRootIRawElementProviderSimple();

  ComPtr<IRawElementProviderSimple> host_provider;
  EXPECT_HRESULT_SUCCEEDED(
      root_node->get_HostRawElementProvider(&host_provider));
  EXPECT_EQ(nullptr, host_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, UIAGetBoundingRectangle) {
  AXNodeData root_data;
  root_data.id = 1;
  root_data.relative_bounds.bounds = gfx::RectF(10, 20, 30, 50);
  Init(root_data);

  ComPtr<IRawElementProviderFragment> root_node =
      GetRootIRawElementProviderFragment();

  UiaRect bounding_rectangle;
  EXPECT_HRESULT_SUCCEEDED(
      root_node->get_BoundingRectangle(&bounding_rectangle));
  EXPECT_EQ(10, bounding_rectangle.left);
  EXPECT_EQ(20, bounding_rectangle.top);
  EXPECT_EQ(30, bounding_rectangle.width);
  EXPECT_EQ(50, bounding_rectangle.height);
}

TEST_F(AXPlatformNodeWinTest, UIAGetFragmentRoot) {
  // This test needs to be run on a child node since AXPlatformRootNodeWin
  // overrides the method.
  AXNodeData root_data;
  root_data.id = 1;

  AXNodeData element1_data;
  element1_data.id = 2;
  root_data.child_ids.push_back(element1_data.id);

  Init(root_data, element1_data);
  InitFragmentRoot();

  AXNode* root_node = GetRootAsAXNode();
  AXNode* element1_node = root_node->children()[0];

  ComPtr<IRawElementProviderFragment> element1_provider =
      QueryInterfaceFromNode<IRawElementProviderFragment>(element1_node);
  ComPtr<IRawElementProviderFragmentRoot> expected_fragment_root =
      GetFragmentRoot();

  ComPtr<IRawElementProviderFragmentRoot> actual_fragment_root;
  EXPECT_HRESULT_SUCCEEDED(
      element1_provider->get_FragmentRoot(&actual_fragment_root));
  EXPECT_EQ(expected_fragment_root.Get(), actual_fragment_root.Get());

  // Test the case where the fragment root has gone away.
  ax_fragment_root_.reset();
  actual_fragment_root.Reset();
  EXPECT_UIA_ELEMENTNOTAVAILABLE(
      element1_provider->get_FragmentRoot(&actual_fragment_root));

  // Test the case where the widget has gone away.
  TestAXNodeWrapper* element1_wrapper =
      TestAXNodeWrapper::GetOrCreate(GetTree(), element1_node);
  element1_wrapper->ResetNativeEventTarget();
  EXPECT_UIA_ELEMENTNOTAVAILABLE(
      element1_provider->get_FragmentRoot(&actual_fragment_root));
}

TEST_F(AXPlatformNodeWinTest, UIAGetEmbeddedFragmentRoots) {
  AXNodeData root_data;
  root_data.id = 1;
  Init(root_data);

  ComPtr<IRawElementProviderFragment> root_provider =
      GetRootIRawElementProviderFragment();

  base::win::ScopedSafearray embedded_fragment_roots;
  EXPECT_HRESULT_SUCCEEDED(root_provider->GetEmbeddedFragmentRoots(
      embedded_fragment_roots.Receive()));
  EXPECT_EQ(nullptr, embedded_fragment_roots.Get());
}

TEST_F(AXPlatformNodeWinTest, UIAGetRuntimeId) {
  AXNodeData root_data;
  root_data.id = 1;
  Init(root_data);

  ComPtr<IRawElementProviderFragment> root_provider =
      GetRootIRawElementProviderFragment();

  base::win::ScopedSafearray runtime_id;
  EXPECT_HRESULT_SUCCEEDED(root_provider->GetRuntimeId(runtime_id.Receive()));

  LONG array_lower_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetLBound(runtime_id.Get(), 1, &array_lower_bound));
  EXPECT_EQ(0, array_lower_bound);

  LONG array_upper_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetUBound(runtime_id.Get(), 1, &array_upper_bound));
  EXPECT_EQ(1, array_upper_bound);

  int* array_data;
  EXPECT_HRESULT_SUCCEEDED(::SafeArrayAccessData(
      runtime_id.Get(), reinterpret_cast<void**>(&array_data)));
  EXPECT_EQ(UiaAppendRuntimeId, array_data[0]);
  EXPECT_NE(-1, array_data[1]);

  EXPECT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(runtime_id.Get()));
}

TEST_F(AXPlatformNodeWinTest, UIAIWindowProviderGetIsModalUnset) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<IWindowProvider> window_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_WindowPatternId, &window_provider));
  ASSERT_EQ(nullptr, window_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, UIAIWindowProviderGetIsModalFalse) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.AddBoolAttribute(ax::mojom::BoolAttribute::kModal, false);
  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<IWindowProvider> window_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_WindowPatternId, &window_provider));
  ASSERT_NE(nullptr, window_provider.Get());

  BOOL is_modal;
  EXPECT_HRESULT_SUCCEEDED(window_provider->get_IsModal(&is_modal));
  ASSERT_FALSE(is_modal);
}

TEST_F(AXPlatformNodeWinTest, UIAIWindowProviderGetIsModalTrue) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.AddBoolAttribute(ax::mojom::BoolAttribute::kModal, true);
  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<IWindowProvider> window_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_WindowPatternId, &window_provider));
  ASSERT_NE(nullptr, window_provider.Get());

  BOOL is_modal;
  EXPECT_HRESULT_SUCCEEDED(window_provider->get_IsModal(&is_modal));
  ASSERT_TRUE(is_modal);
}

TEST_F(AXPlatformNodeWinTest, UIAIWindowProviderInvalidArgument) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.AddBoolAttribute(ax::mojom::BoolAttribute::kModal, true);
  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<IWindowProvider> window_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_WindowPatternId, &window_provider));
  ASSERT_NE(nullptr, window_provider.Get());

  ASSERT_EQ(E_INVALIDARG, window_provider->WaitForInputIdle(0, nullptr));
  ASSERT_EQ(E_INVALIDARG, window_provider->get_CanMaximize(nullptr));
  ASSERT_EQ(E_INVALIDARG, window_provider->get_CanMinimize(nullptr));
  ASSERT_EQ(E_INVALIDARG, window_provider->get_IsModal(nullptr));
  ASSERT_EQ(E_INVALIDARG, window_provider->get_WindowVisualState(nullptr));
  ASSERT_EQ(E_INVALIDARG, window_provider->get_WindowInteractionState(nullptr));
  ASSERT_EQ(E_INVALIDARG, window_provider->get_IsTopmost(nullptr));
}

TEST_F(AXPlatformNodeWinTest, UIAIWindowProviderNotSupported) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.AddBoolAttribute(ax::mojom::BoolAttribute::kModal, true);
  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<IWindowProvider> window_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_WindowPatternId, &window_provider));
  ASSERT_NE(nullptr, window_provider.Get());

  BOOL bool_result;
  WindowVisualState window_visual_state_result;
  WindowInteractionState window_interaction_state_result;

  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED),
            window_provider->SetVisualState(
                WindowVisualState::WindowVisualState_Normal));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED), window_provider->Close());
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED),
            window_provider->WaitForInputIdle(0, &bool_result));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED),
            window_provider->get_CanMaximize(&bool_result));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED),
            window_provider->get_CanMinimize(&bool_result));
  ASSERT_EQ(
      static_cast<HRESULT>(UIA_E_NOTSUPPORTED),
      window_provider->get_WindowVisualState(&window_visual_state_result));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED),
            window_provider->get_WindowInteractionState(
                &window_interaction_state_result));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_NOTSUPPORTED),
            window_provider->get_IsTopmost(&bool_result));
}

TEST_F(AXPlatformNodeWinTest, UIANavigate) {
  AXNodeData root_data;
  root_data.id = 1;

  AXNodeData element1_data;
  element1_data.id = 2;
  root_data.child_ids.push_back(element1_data.id);

  AXNodeData element2_data;
  element2_data.id = 3;
  root_data.child_ids.push_back(element2_data.id);

  AXNodeData element3_data;
  element3_data.id = 4;
  element1_data.child_ids.push_back(element3_data.id);

  Init(root_data, element1_data, element2_data, element3_data);

  AXNode* root_node = GetRootAsAXNode();
  AXNode* element1_node = root_node->children()[0];
  AXNode* element2_node = root_node->children()[1];
  AXNode* element3_node = element1_node->children()[0];

  auto TestNavigate = [this](AXNode* element_node, AXNode* parent,
                             AXNode* next_sibling, AXNode* prev_sibling,
                             AXNode* first_child, AXNode* last_child) {
    ComPtr<IRawElementProviderFragment> element_provider =
        QueryInterfaceFromNode<IRawElementProviderFragment>(element_node);

    auto TestNavigateSingle = [&](NavigateDirection direction,
                                  AXNode* expected_node) {
      ComPtr<IRawElementProviderFragment> expected_provider =
          QueryInterfaceFromNode<IRawElementProviderFragment>(expected_node);

      ComPtr<IRawElementProviderFragment> navigated_to_fragment;
      EXPECT_HRESULT_SUCCEEDED(
          element_provider->Navigate(direction, &navigated_to_fragment));
      EXPECT_EQ(expected_provider.Get(), navigated_to_fragment.Get());
    };

    TestNavigateSingle(NavigateDirection_Parent, parent);
    TestNavigateSingle(NavigateDirection_NextSibling, next_sibling);
    TestNavigateSingle(NavigateDirection_PreviousSibling, prev_sibling);
    TestNavigateSingle(NavigateDirection_FirstChild, first_child);
    TestNavigateSingle(NavigateDirection_LastChild, last_child);
  };

  TestNavigate(root_node,
               nullptr,         // Parent
               nullptr,         // NextSibling
               nullptr,         // PreviousSibling
               element1_node,   // FirstChild
               element2_node);  // LastChild

  TestNavigate(element1_node, root_node, element2_node, nullptr, element3_node,
               element3_node);

  TestNavigate(element2_node, root_node, nullptr, element1_node, nullptr,
               nullptr);

  TestNavigate(element3_node, element1_node, nullptr, nullptr, nullptr,
               nullptr);
}

TEST_F(AXPlatformNodeWinTest, ISelectionProviderCanSelectMultipleDefault) {
  Init(BuildListBox(/*option_1_is_selected*/ false,
                    /*option_2_is_selected*/ false,
                    /*option_3_is_selected*/ false, {}));

  ComPtr<ISelectionProvider> selection_provider(
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode()));

  BOOL multiple = TRUE;
  EXPECT_HRESULT_SUCCEEDED(
      selection_provider->get_CanSelectMultiple(&multiple));
  EXPECT_FALSE(multiple);
}

TEST_F(AXPlatformNodeWinTest, ISelectionProviderCanSelectMultipleTrue) {
  const std::vector<ax::mojom::State> state = {
      ax::mojom::State::kMultiselectable, ax::mojom::State::kFocusable};
  Init(BuildListBox(/*option_1_is_selected*/ false,
                    /*option_2_is_selected*/ false,
                    /*option_3_is_selected*/ false,
                    /*additional_state*/ state));

  ComPtr<ISelectionProvider> selection_provider(
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode()));

  BOOL multiple = FALSE;
  EXPECT_HRESULT_SUCCEEDED(
      selection_provider->get_CanSelectMultiple(&multiple));
  EXPECT_TRUE(multiple);
}

TEST_F(AXPlatformNodeWinTest, ISelectionProviderIsSelectionRequiredDefault) {
  Init(BuildListBox(/*option_1_is_selected*/ false,
                    /*option_2_is_selected*/ false,
                    /*option_3_is_selected*/ false,
                    /*additional_state*/ {}));

  ComPtr<ISelectionProvider> selection_provider(
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode()));

  BOOL selection_required = TRUE;
  EXPECT_HRESULT_SUCCEEDED(
      selection_provider->get_IsSelectionRequired(&selection_required));
  EXPECT_FALSE(selection_required);
}

TEST_F(AXPlatformNodeWinTest, ISelectionProviderIsSelectionRequiredTrue) {
  Init(BuildListBox(/*option_1_is_selected*/ false,
                    /*option_2_is_selected*/ false,
                    /*option_3_is_selected*/ false,
                    /*additional_state*/ {ax::mojom::State::kRequired}));

  ComPtr<ISelectionProvider> selection_provider(
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode()));

  BOOL selection_required = FALSE;
  EXPECT_HRESULT_SUCCEEDED(
      selection_provider->get_IsSelectionRequired(&selection_required));
  EXPECT_TRUE(selection_required);
}

TEST_F(AXPlatformNodeWinTest, ISelectionProviderGetSelectionNoneSelected) {
  Init(BuildListBox(/*option_1_is_selected*/ false,
                    /*option_2_is_selected*/ false,
                    /*option_3_is_selected*/ false,
                    /*additional_state*/ {ax::mojom::State::kFocusable}));

  ComPtr<ISelectionProvider> selection_provider(
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode()));

  base::win::ScopedSafearray selected_items;
  EXPECT_HRESULT_SUCCEEDED(
      selection_provider->GetSelection(selected_items.Receive()));
  EXPECT_NE(nullptr, selected_items.Get());

  LONG array_lower_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetLBound(selected_items.Get(), 1, &array_lower_bound));
  EXPECT_EQ(0, array_lower_bound);

  LONG array_upper_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetUBound(selected_items.Get(), 1, &array_upper_bound));
  EXPECT_EQ(-1, array_upper_bound);
}

TEST_F(AXPlatformNodeWinTest,
       ISelectionProviderGetSelectionSingleItemSelected) {
  Init(BuildListBox(/*option_1_is_selected*/ false,
                    /*option_2_is_selected*/ true,
                    /*option_3_is_selected*/ false,
                    /*additional_state*/ {ax::mojom::State::kFocusable}));

  ComPtr<ISelectionProvider> selection_provider(
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode()));
  ComPtr<IRawElementProviderSimple> option2_provider(
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[1]));

  base::win::ScopedSafearray selected_items;
  EXPECT_HRESULT_SUCCEEDED(
      selection_provider->GetSelection(selected_items.Receive()));
  EXPECT_NE(nullptr, selected_items.Get());

  LONG array_lower_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetLBound(selected_items.Get(), 1, &array_lower_bound));
  EXPECT_EQ(0, array_lower_bound);

  LONG array_upper_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetUBound(selected_items.Get(), 1, &array_upper_bound));
  EXPECT_EQ(0, array_upper_bound);

  IRawElementProviderSimple** array_data;
  EXPECT_HRESULT_SUCCEEDED(::SafeArrayAccessData(
      selected_items.Get(), reinterpret_cast<void**>(&array_data)));
  EXPECT_EQ(option2_provider.Get(), array_data[0]);
  EXPECT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(selected_items.Get()));
}

TEST_F(AXPlatformNodeWinTest,
       ISelectionProviderGetSelectionMultipleItemsSelected) {
  const std::vector<ax::mojom::State> state = {
      ax::mojom::State::kMultiselectable, ax::mojom::State::kFocusable};
  Init(BuildListBox(/*option_1_is_selected*/ true,
                    /*option_2_is_selected*/ true,
                    /*option_3_is_selected*/ true,
                    /*additional_state*/ state));

  ComPtr<ISelectionProvider> selection_provider(
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode()));
  ComPtr<IRawElementProviderSimple> option1_provider(
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[0]));
  ComPtr<IRawElementProviderSimple> option2_provider(
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[1]));
  ComPtr<IRawElementProviderSimple> option3_provider(
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          GetRootAsAXNode()->children()[2]));

  base::win::ScopedSafearray selected_items;
  EXPECT_HRESULT_SUCCEEDED(
      selection_provider->GetSelection(selected_items.Receive()));
  EXPECT_NE(nullptr, selected_items.Get());

  LONG array_lower_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetLBound(selected_items.Get(), 1, &array_lower_bound));
  EXPECT_EQ(0, array_lower_bound);

  LONG array_upper_bound;
  EXPECT_HRESULT_SUCCEEDED(
      ::SafeArrayGetUBound(selected_items.Get(), 1, &array_upper_bound));
  EXPECT_EQ(2, array_upper_bound);

  IRawElementProviderSimple** array_data;
  EXPECT_HRESULT_SUCCEEDED(::SafeArrayAccessData(
      selected_items.Get(), reinterpret_cast<void**>(&array_data)));
  EXPECT_EQ(option1_provider.Get(), array_data[0]);
  EXPECT_EQ(option2_provider.Get(), array_data[1]);
  EXPECT_EQ(option3_provider.Get(), array_data[2]);

  EXPECT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(selected_items.Get()));
}

TEST_F(AXPlatformNodeWinTest, ComputeUIAControlType) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;

  AXNodeData child1;
  AXNode::AXID child1_id = 2;
  child1.id = child1_id;
  child1.role = ax::mojom::Role::kTable;
  root.child_ids.push_back(child1_id);

  AXNodeData child2;
  AXNode::AXID child2_id = 3;
  child2.id = child2_id;
  child2.role = ax::mojom::Role::kLayoutTable;
  root.child_ids.push_back(child2_id);

  AXNodeData child3;
  AXNode::AXID child3_id = 4;
  child3.id = child3_id;
  child3.role = ax::mojom::Role::kTextField;
  root.child_ids.push_back(child3_id);

  AXNodeData child4;
  AXNode::AXID child4_id = 5;
  child4.id = child4_id;
  child4.role = ax::mojom::Role::kSearchBox;
  root.child_ids.push_back(child4_id);

  Init(root, child1, child2, child3, child4);

  EXPECT_UIA_INT_EQ(
      QueryInterfaceFromNodeId<IRawElementProviderSimple>(child1_id),
      UIA_ControlTypePropertyId, int{UIA_TableControlTypeId});
  EXPECT_UIA_INT_EQ(
      QueryInterfaceFromNodeId<IRawElementProviderSimple>(child2_id),
      UIA_ControlTypePropertyId, int{UIA_TableControlTypeId});
  EXPECT_UIA_INT_EQ(
      QueryInterfaceFromNodeId<IRawElementProviderSimple>(child3_id),
      UIA_ControlTypePropertyId, int{UIA_EditControlTypeId});
  EXPECT_UIA_INT_EQ(
      QueryInterfaceFromNodeId<IRawElementProviderSimple>(child4_id),
      UIA_ControlTypePropertyId, int{UIA_EditControlTypeId});
}

TEST_F(AXPlatformNodeWinTest, UIALandmarkType) {
  auto TestLandmarkType = [this](ax::mojom::Role node_role,
                                 std::optional<LONG> expected_landmark_type,
                                 const std::string& node_name = {}) {
    AXNodeData root_data;
    root_data.id = 1;
    root_data.role = node_role;
    if (!node_name.empty())
      root_data.SetName(node_name);
    Init(root_data);

    ComPtr<IRawElementProviderSimple> root_provider =
        GetRootIRawElementProviderSimple();

    if (expected_landmark_type) {
      EXPECT_UIA_INT_EQ(root_provider, UIA_LandmarkTypePropertyId,
                        expected_landmark_type.value());
    } else {
      EXPECT_UIA_EMPTY(root_provider, UIA_LandmarkTypePropertyId);
    }
  };

  TestLandmarkType(ax::mojom::Role::kBanner, UIA_CustomLandmarkTypeId);
  TestLandmarkType(ax::mojom::Role::kComplementary, UIA_CustomLandmarkTypeId);
  TestLandmarkType(ax::mojom::Role::kContentInfo, UIA_CustomLandmarkTypeId);
  TestLandmarkType(ax::mojom::Role::kFooter, UIA_CustomLandmarkTypeId);
  TestLandmarkType(ax::mojom::Role::kMain, UIA_MainLandmarkTypeId);
  TestLandmarkType(ax::mojom::Role::kNavigation, UIA_NavigationLandmarkTypeId);
  TestLandmarkType(ax::mojom::Role::kSearch, UIA_SearchLandmarkTypeId);

  // Only named forms should be exposed as landmarks.
  TestLandmarkType(ax::mojom::Role::kForm, {});
  TestLandmarkType(ax::mojom::Role::kForm, UIA_FormLandmarkTypeId, "name");

  // Only named regions should be exposed as landmarks.
  TestLandmarkType(ax::mojom::Role::kRegion, {});
  TestLandmarkType(ax::mojom::Role::kRegion, UIA_CustomLandmarkTypeId, "name");

  TestLandmarkType(ax::mojom::Role::kGroup, {});
  TestLandmarkType(ax::mojom::Role::kHeading, {});
  TestLandmarkType(ax::mojom::Role::kList, {});
  TestLandmarkType(ax::mojom::Role::kTable, {});
}

TEST_F(AXPlatformNodeWinTest, UIALocalizedLandmarkType) {
  auto TestLocalizedLandmarkType =
      [this](ax::mojom::Role node_role,
             const std::wstring& expected_localized_landmark,
             const std::string& node_name = {}) {
        AXNodeData root_data;
        root_data.id = 1;
        root_data.role = node_role;
        if (!node_name.empty())
          root_data.SetName(node_name);
        Init(root_data);

        ComPtr<IRawElementProviderSimple> root_provider =
            GetRootIRawElementProviderSimple();

        if (expected_localized_landmark.empty()) {
          EXPECT_UIA_EMPTY(root_provider, UIA_LocalizedLandmarkTypePropertyId);
        } else {
          EXPECT_UIA_BSTR_EQ(root_provider, UIA_LocalizedLandmarkTypePropertyId,
                             expected_localized_landmark.c_str());
        }
      };

  TestLocalizedLandmarkType(ax::mojom::Role::kBanner, L"banner");
  TestLocalizedLandmarkType(ax::mojom::Role::kComplementary, L"complementary");
  TestLocalizedLandmarkType(ax::mojom::Role::kContentInfo,
                            L"content information");
  TestLocalizedLandmarkType(ax::mojom::Role::kFooter, L"content information");

  // Only named regions should be exposed as landmarks.
  TestLocalizedLandmarkType(ax::mojom::Role::kRegion, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kRegion, L"region", "name");

  TestLocalizedLandmarkType(ax::mojom::Role::kForm, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kGroup, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kHeading, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kList, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kMain, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kNavigation, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kSearch, {});
  TestLocalizedLandmarkType(ax::mojom::Role::kTable, {});
}

TEST_F(AXPlatformNodeWinTest, IRawElementProviderSimple2ShowContextMenu) {
  AXNodeData root_data;
  root_data.id = 1;

  AXNodeData element1_data;
  element1_data.id = 2;
  root_data.child_ids.push_back(element1_data.id);

  AXNodeData element2_data;
  element2_data.id = 3;
  root_data.child_ids.push_back(element2_data.id);

  Init(root_data, element1_data, element2_data);

  AXNode* root_node = GetRootAsAXNode();
  AXNode* element1_node = root_node->children()[0];
  AXNode* element2_node = root_node->children()[1];

  ComPtr<IRawElementProviderSimple2> root_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple2>(root_node);
  ComPtr<IRawElementProviderSimple2> element1_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple2>(element1_node);
  ComPtr<IRawElementProviderSimple2> element2_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple2>(element2_node);

  EXPECT_HRESULT_SUCCEEDED(element1_provider->ShowContextMenu());
  EXPECT_EQ(element1_node, TestAXNodeWrapper::GetNodeFromLastShowContextMenu());
  EXPECT_HRESULT_SUCCEEDED(element2_provider->ShowContextMenu());
  EXPECT_EQ(element2_node, TestAXNodeWrapper::GetNodeFromLastShowContextMenu());
  EXPECT_HRESULT_SUCCEEDED(root_provider->ShowContextMenu());
  EXPECT_EQ(root_node, TestAXNodeWrapper::GetNodeFromLastShowContextMenu());
}

TEST_F(AXPlatformNodeWinTest, UIAErrorHandling) {
  AXNodeData root;
  root.id = 1;
  Init(root);

  ComPtr<IRawElementProviderSimple> simple_provider =
      GetRootIRawElementProviderSimple();
  ComPtr<IRawElementProviderSimple2> simple2_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple2>(GetRootAsAXNode());
  ComPtr<IRawElementProviderFragment> fragment_provider =
      GetRootIRawElementProviderFragment();
  ComPtr<IGridItemProvider> grid_item_provider =
      QueryInterfaceFromNode<IGridItemProvider>(GetRootAsAXNode());
  ComPtr<IGridProvider> grid_provider =
      QueryInterfaceFromNode<IGridProvider>(GetRootAsAXNode());
  ComPtr<IScrollItemProvider> scroll_item_provider =
      QueryInterfaceFromNode<IScrollItemProvider>(GetRootAsAXNode());
  ComPtr<IScrollProvider> scroll_provider =
      QueryInterfaceFromNode<IScrollProvider>(GetRootAsAXNode());
  ComPtr<ISelectionItemProvider> selection_item_provider =
      QueryInterfaceFromNode<ISelectionItemProvider>(GetRootAsAXNode());
  ComPtr<ISelectionProvider> selection_provider =
      QueryInterfaceFromNode<ISelectionProvider>(GetRootAsAXNode());
  ComPtr<ITableItemProvider> table_item_provider =
      QueryInterfaceFromNode<ITableItemProvider>(GetRootAsAXNode());
  ComPtr<ITableProvider> table_provider =
      QueryInterfaceFromNode<ITableProvider>(GetRootAsAXNode());
  ComPtr<IExpandCollapseProvider> expand_collapse_provider =
      QueryInterfaceFromNode<IExpandCollapseProvider>(GetRootAsAXNode());
  ComPtr<IToggleProvider> toggle_provider =
      QueryInterfaceFromNode<IToggleProvider>(GetRootAsAXNode());
  ComPtr<IValueProvider> value_provider =
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode());
  ComPtr<IRangeValueProvider> range_value_provider =
      QueryInterfaceFromNode<IRangeValueProvider>(GetRootAsAXNode());
  ComPtr<IWindowProvider> window_provider =
      QueryInterfaceFromNode<IWindowProvider>(GetRootAsAXNode());

  // Create an empty tree.
  SetTree(std::make_unique<AXTree>());

  // IGridItemProvider
  int int_result = 0;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            grid_item_provider->get_Column(&int_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            grid_item_provider->get_ColumnSpan(&int_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            grid_item_provider->get_Row(&int_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            grid_item_provider->get_RowSpan(&int_result));

  // IExpandCollapseProvider
  ExpandCollapseState expand_collapse_state;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            expand_collapse_provider->Collapse());
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            expand_collapse_provider->Expand());
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            expand_collapse_provider->get_ExpandCollapseState(
                &expand_collapse_state));

  // IGridProvider
  ComPtr<IRawElementProviderSimple> temp_simple_provider;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            grid_provider->GetItem(0, 0, &temp_simple_provider));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            grid_provider->get_RowCount(&int_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            grid_provider->get_ColumnCount(&int_result));

  // IScrollItemProvider
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_item_provider->ScrollIntoView());

  // IScrollProvider
  BOOL bool_result = TRUE;
  double double_result = 3.14;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_provider->SetScrollPercent(0, 0));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_provider->get_HorizontallyScrollable(&bool_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_provider->get_HorizontalScrollPercent(&double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_provider->get_HorizontalViewSize(&double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_provider->get_VerticallyScrollable(&bool_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_provider->get_VerticalScrollPercent(&double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            scroll_provider->get_VerticalViewSize(&double_result));

  // ISelectionItemProvider
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            selection_item_provider->AddToSelection());
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            selection_item_provider->RemoveFromSelection());
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            selection_item_provider->Select());
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            selection_item_provider->get_IsSelected(&bool_result));
  EXPECT_EQ(
      static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
      selection_item_provider->get_SelectionContainer(&temp_simple_provider));

  // ISelectionProvider
  base::win::ScopedSafearray array_result;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            selection_provider->GetSelection(array_result.Receive()));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            selection_provider->get_CanSelectMultiple(&bool_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            selection_provider->get_IsSelectionRequired(&bool_result));

  // ITableItemProvider
  RowOrColumnMajor row_or_column_major_result;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            table_item_provider->GetColumnHeaderItems(array_result.Receive()));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            table_item_provider->GetRowHeaderItems(array_result.Receive()));

  // ITableProvider
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            table_provider->GetColumnHeaders(array_result.Receive()));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            table_provider->GetRowHeaders(array_result.Receive()));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            table_provider->get_RowOrColumnMajor(&row_or_column_major_result));

  // IRawElementProviderSimple
  ScopedVariant variant;
  ComPtr<IUnknown> unknown;
  ComPtr<IRawElementProviderSimple> host_provider;
  ProviderOptions options;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            simple_provider->GetPatternProvider(UIA_WindowPatternId, &unknown));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            simple_provider->GetPropertyValue(UIA_FrameworkIdPropertyId,
                                              variant.Receive()));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            simple_provider->get_ProviderOptions(&options));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            simple_provider->get_HostRawElementProvider(&host_provider));

  // IRawElementProviderSimple2
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            simple2_provider->ShowContextMenu());

  // IRawElementProviderFragment
  ComPtr<IRawElementProviderFragment> navigated_to_fragment;
  base::win::ScopedSafearray safearray;
  UiaRect bounding_rectangle;
  ComPtr<IRawElementProviderFragmentRoot> fragment_root;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_provider->Navigate(NavigateDirection_Parent,
                                        &navigated_to_fragment));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_provider->GetRuntimeId(safearray.Receive()));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_provider->get_BoundingRectangle(&bounding_rectangle));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_provider->GetEmbeddedFragmentRoots(safearray.Receive()));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_provider->get_FragmentRoot(&fragment_root));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_provider->SetFocus());

  // IValueProvider
  ScopedBstr bstr_value;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            value_provider->SetValue(L"3.14"));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            value_provider->get_Value(bstr_value.Receive()));

  // IRangeValueProvider
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            range_value_provider->SetValue(double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            range_value_provider->get_LargeChange(&double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            range_value_provider->get_Maximum(&double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            range_value_provider->get_Minimum(&double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            range_value_provider->get_SmallChange(&double_result));
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            range_value_provider->get_Value(&double_result));

  // IToggleProvider
  ToggleState toggle_state;
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            toggle_provider->Toggle());
  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            toggle_provider->get_ToggleState(&toggle_state));

  // IWindowProvider
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->SetVisualState(
                WindowVisualState::WindowVisualState_Normal));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->Close());
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->WaitForInputIdle(0, nullptr));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->get_CanMaximize(nullptr));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->get_CanMinimize(nullptr));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->get_IsModal(nullptr));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->get_WindowVisualState(nullptr));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->get_WindowInteractionState(nullptr));
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            window_provider->get_IsTopmost(nullptr));
}

TEST_F(AXPlatformNodeWinTest, GetPatternProviderSupportedPatterns) {
  constexpr AXNode::AXID root_id = 1;
  constexpr AXNode::AXID text_field_with_combo_box_id = 2;
  constexpr AXNode::AXID meter_id = 3;
  constexpr AXNode::AXID group_with_scroll_id = 4;
  constexpr AXNode::AXID checkbox_id = 5;
  constexpr AXNode::AXID link_id = 6;
  constexpr AXNode::AXID table_without_header_id = 7;
  constexpr AXNode::AXID table_without_header_cell_id = 8;
  constexpr AXNode::AXID table_with_header_id = 9;
  constexpr AXNode::AXID table_with_header_row_1_id = 10;
  constexpr AXNode::AXID table_with_header_column_header_id = 11;
  constexpr AXNode::AXID table_with_header_row_2_id = 12;
  constexpr AXNode::AXID table_with_header_cell_id = 13;
  constexpr AXNode::AXID grid_without_header_id = 14;
  constexpr AXNode::AXID grid_without_header_cell_id = 15;
  constexpr AXNode::AXID grid_with_header_id = 16;
  constexpr AXNode::AXID grid_with_header_row_1_id = 17;
  constexpr AXNode::AXID grid_with_header_column_header_id = 18;
  constexpr AXNode::AXID grid_with_header_row_2_id = 19;
  constexpr AXNode::AXID grid_with_header_cell_id = 20;

  AXTreeUpdate update;
  update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.has_tree_data = true;
  update.root_id = root_id;
  update.nodes.resize(20);
  update.nodes[0].id = root_id;
  update.nodes[0].role = ax::mojom::Role::kRootWebArea;
  update.nodes[0].child_ids = {text_field_with_combo_box_id,
                               meter_id,
                               group_with_scroll_id,
                               checkbox_id,
                               link_id,
                               table_without_header_id,
                               table_with_header_id,
                               grid_without_header_id,
                               grid_with_header_id};
  update.nodes[1].id = text_field_with_combo_box_id;
  update.nodes[1].role = ax::mojom::Role::kTextFieldWithComboBox;
  update.nodes[2].id = meter_id;
  update.nodes[2].role = ax::mojom::Role::kMeter;
  update.nodes[3].id = group_with_scroll_id;
  update.nodes[3].role = ax::mojom::Role::kGroup;
  update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kScrollXMin, 10);
  update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kScrollXMax, 10);
  update.nodes[3].AddIntAttribute(ax::mojom::IntAttribute::kScrollX, 10);
  update.nodes[4].id = checkbox_id;
  update.nodes[4].role = ax::mojom::Role::kCheckBox;
  update.nodes[5].id = link_id;
  update.nodes[5].role = ax::mojom::Role::kLink;
  update.nodes[6].id = table_without_header_id;
  update.nodes[6].role = ax::mojom::Role::kTable;
  update.nodes[6].child_ids = {table_without_header_cell_id};
  update.nodes[7].id = table_without_header_cell_id;
  update.nodes[7].role = ax::mojom::Role::kCell;
  update.nodes[8].id = table_with_header_id;
  update.nodes[8].role = ax::mojom::Role::kTable;
  update.nodes[8].child_ids = {table_with_header_row_1_id,
                               table_with_header_row_2_id};
  update.nodes[9].id = table_with_header_row_1_id;
  update.nodes[9].role = ax::mojom::Role::kRow;
  update.nodes[9].child_ids = {table_with_header_column_header_id};
  update.nodes[10].id = table_with_header_column_header_id;
  update.nodes[10].role = ax::mojom::Role::kColumnHeader;
  update.nodes[11].id = table_with_header_row_2_id;
  update.nodes[11].role = ax::mojom::Role::kRow;
  update.nodes[11].child_ids = {table_with_header_cell_id};
  update.nodes[12].id = table_with_header_cell_id;
  update.nodes[12].role = ax::mojom::Role::kCell;
  update.nodes[13].id = grid_without_header_id;
  update.nodes[13].role = ax::mojom::Role::kGrid;
  update.nodes[13].child_ids = {grid_without_header_cell_id};
  update.nodes[14].id = grid_without_header_cell_id;
  update.nodes[14].role = ax::mojom::Role::kCell;
  update.nodes[14].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);
  update.nodes[15].id = grid_with_header_id;
  update.nodes[15].role = ax::mojom::Role::kGrid;
  update.nodes[15].child_ids = {grid_with_header_row_1_id,
                                grid_with_header_row_2_id};
  update.nodes[16].id = grid_with_header_row_1_id;
  update.nodes[16].role = ax::mojom::Role::kRow;
  update.nodes[16].child_ids = {grid_with_header_column_header_id};
  update.nodes[17].id = grid_with_header_column_header_id;
  update.nodes[17].role = ax::mojom::Role::kColumnHeader;
  update.nodes[17].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);
  update.nodes[18].id = grid_with_header_row_2_id;
  update.nodes[18].role = ax::mojom::Role::kRow;
  update.nodes[18].child_ids = {grid_with_header_cell_id};
  update.nodes[19].id = grid_with_header_cell_id;
  update.nodes[19].role = ax::mojom::Role::kCell;
  update.nodes[19].AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);

  Init(update);

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_TextPatternId,
                        UIA_TextEditPatternId}),
            GetSupportedPatternsFromNodeId(root_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_ExpandCollapsePatternId, UIA_TextPatternId,
                        UIA_TextEditPatternId}),
            GetSupportedPatternsFromNodeId(text_field_with_combo_box_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_RangeValuePatternId}),
            GetSupportedPatternsFromNodeId(meter_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ScrollPatternId}),
            GetSupportedPatternsFromNodeId(group_with_scroll_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_TogglePatternId}),
            GetSupportedPatternsFromNodeId(checkbox_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_InvokePatternId}),
            GetSupportedPatternsFromNodeId(link_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_GridPatternId}),
            GetSupportedPatternsFromNodeId(table_without_header_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_GridItemPatternId}),
            GetSupportedPatternsFromNodeId(table_without_header_cell_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_GridPatternId,
                        UIA_TablePatternId}),
            GetSupportedPatternsFromNodeId(table_with_header_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_GridItemPatternId,
                        UIA_TableItemPatternId}),
            GetSupportedPatternsFromNodeId(table_with_header_column_header_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_GridItemPatternId,
                        UIA_TableItemPatternId}),
            GetSupportedPatternsFromNodeId(table_with_header_cell_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_SelectionPatternId, UIA_GridPatternId}),
            GetSupportedPatternsFromNodeId(grid_without_header_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_SelectionItemPatternId, UIA_GridItemPatternId}),
            GetSupportedPatternsFromNodeId(grid_without_header_cell_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_SelectionPatternId, UIA_GridPatternId,
                        UIA_TablePatternId}),
            GetSupportedPatternsFromNodeId(grid_with_header_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_GridItemPatternId, UIA_TableItemPatternId,
                        UIA_SelectionItemPatternId}),
            GetSupportedPatternsFromNodeId(grid_with_header_column_header_id));

  EXPECT_EQ(PatternSet({UIA_ScrollItemPatternId, UIA_ValuePatternId,
                        UIA_GridItemPatternId, UIA_TableItemPatternId,
                        UIA_SelectionItemPatternId}),
            GetSupportedPatternsFromNodeId(grid_with_header_cell_id));
}

TEST_F(AXPlatformNodeWinTest, GetPatternProviderExpandCollapsePattern) {
  ui::AXNodeData root;
  root.id = 1;

  ui::AXNodeData list_box;
  ui::AXNodeData list_item;
  ui::AXNodeData menu_item;
  ui::AXNodeData menu_list_option;
  ui::AXNodeData tree_item;
  ui::AXNodeData combo_box_grouping;
  ui::AXNodeData combo_box_menu_button;
  ui::AXNodeData disclosure_triangle;
  ui::AXNodeData text_field_with_combo_box;

  list_box.id = 2;
  list_item.id = 3;
  menu_item.id = 4;
  menu_list_option.id = 5;
  tree_item.id = 6;
  combo_box_grouping.id = 7;
  combo_box_menu_button.id = 8;
  disclosure_triangle.id = 9;
  text_field_with_combo_box.id = 10;

  root.child_ids.push_back(list_box.id);
  root.child_ids.push_back(list_item.id);
  root.child_ids.push_back(menu_item.id);
  root.child_ids.push_back(menu_list_option.id);
  root.child_ids.push_back(tree_item.id);
  root.child_ids.push_back(combo_box_grouping.id);
  root.child_ids.push_back(combo_box_menu_button.id);
  root.child_ids.push_back(disclosure_triangle.id);
  root.child_ids.push_back(text_field_with_combo_box.id);

  // list_box HasPopup set to false, does not support expand collapse.
  list_box.role = ax::mojom::Role::kListBoxOption;
  list_box.SetHasPopup(ax::mojom::HasPopup::kFalse);

  // list_item HasPopup set to true, supports expand collapse.
  list_item.role = ax::mojom::Role::kListItem;
  list_item.SetHasPopup(ax::mojom::HasPopup::kTrue);

  // menu_item has expanded state and supports expand collapse.
  menu_item.role = ax::mojom::Role::kMenuItem;
  menu_item.AddState(ax::mojom::State::kExpanded);

  // menu_list_option has collapsed state and supports expand collapse.
  menu_list_option.role = ax::mojom::Role::kMenuListOption;
  menu_list_option.AddState(ax::mojom::State::kCollapsed);

  // These roles by default supports expand collapse.
  tree_item.role = ax::mojom::Role::kTreeItem;
  combo_box_grouping.role = ax::mojom::Role::kComboBoxGrouping;
  combo_box_menu_button.role = ax::mojom::Role::kComboBoxMenuButton;
  disclosure_triangle.role = ax::mojom::Role::kDisclosureTriangle;
  text_field_with_combo_box.role = ax::mojom::Role::kTextFieldWithComboBox;

  Init(root, list_box, list_item, menu_item, menu_list_option, tree_item,
       combo_box_grouping, combo_box_menu_button, disclosure_triangle,
       text_field_with_combo_box);

  // list_box HasPopup set to false, does not support expand collapse.
  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetIRawElementProviderSimpleFromChildIndex(0);
  ComPtr<IExpandCollapseProvider> expandcollapse_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_EQ(nullptr, expandcollapse_provider.Get());

  // list_item HasPopup set to true, supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(1);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());

  // menu_item has expanded state and supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(2);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());

  // menu_list_option has collapsed state and supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(3);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());

  // tree_item by default supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(4);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());

  // combo_box_grouping by default supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(5);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());

  // combo_box_menu_button by default supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(6);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());

  // disclosure_triangle by default supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(7);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());

  // text_field_with_combo_box by default supports expand collapse.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(8);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  EXPECT_NE(nullptr, expandcollapse_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, GetPatternProviderInvokePattern) {
  ui::AXNodeData root;
  root.id = 1;

  ui::AXNodeData link;
  ui::AXNodeData generic_container;
  ui::AXNodeData combo_box_grouping;
  ui::AXNodeData check_box;

  link.id = 2;
  generic_container.id = 3;
  combo_box_grouping.id = 4;
  check_box.id = 5;

  root.child_ids.push_back(link.id);
  root.child_ids.push_back(generic_container.id);
  root.child_ids.push_back(combo_box_grouping.id);
  root.child_ids.push_back(check_box.id);

  // Role link is clickable and neither supports expand collapse nor supports
  // toggle. It should support invoke pattern.
  link.role = ax::mojom::Role::kLink;

  // Role generic container is not clickable. It should not support invoke
  // pattern.
  generic_container.role = ax::mojom::Role::kGenericContainer;

  // Role combo box grouping supports expand collapse. It should not support
  // invoke pattern.
  combo_box_grouping.role = ax::mojom::Role::kComboBoxGrouping;

  // Role check box supports toggle. It should not support invoke pattern.
  check_box.role = ax::mojom::Role::kCheckBox;

  Init(root, link, generic_container, combo_box_grouping, check_box);

  // Role link is clickable and neither supports expand collapse nor supports
  // toggle. It should support invoke pattern.
  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetIRawElementProviderSimpleFromChildIndex(0);
  ComPtr<IInvokeProvider> invoke_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_InvokePatternId, &invoke_provider));
  EXPECT_NE(nullptr, invoke_provider.Get());

  // Role generic container is not clickable. It should not support invoke
  // pattern.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(1);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_InvokePatternId, &invoke_provider));
  EXPECT_EQ(nullptr, invoke_provider.Get());

  // Role combo box grouping supports expand collapse. It should not support
  // invoke pattern.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(2);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_InvokePatternId, &invoke_provider));
  EXPECT_EQ(nullptr, invoke_provider.Get());

  // Role check box supports toggle. It should not support invoke pattern.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(3);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_InvokePatternId, &invoke_provider));
  EXPECT_EQ(nullptr, invoke_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, IExpandCollapsePatternProviderAction) {
  ui::AXNodeData root;
  root.id = 1;

  ui::AXNodeData combo_box_grouping_has_popup;
  ui::AXNodeData combo_box_grouping_expanded;
  ui::AXNodeData combo_box_grouping_collapsed;
  ui::AXNodeData combo_box_grouping_disabled;
  ui::AXNodeData button_has_popup_menu;
  ui::AXNodeData button_has_popup_menu_pressed;
  ui::AXNodeData button_has_popup_true;
  ui::AXNodeData generic_container_has_popup_menu;

  combo_box_grouping_has_popup.id = 2;
  combo_box_grouping_expanded.id = 3;
  combo_box_grouping_collapsed.id = 4;
  combo_box_grouping_disabled.id = 5;
  button_has_popup_menu.id = 6;
  button_has_popup_menu_pressed.id = 7;
  button_has_popup_true.id = 8;
  generic_container_has_popup_menu.id = 9;

  root.child_ids = {
      combo_box_grouping_has_popup.id, combo_box_grouping_expanded.id,
      combo_box_grouping_collapsed.id, combo_box_grouping_disabled.id,
      button_has_popup_menu.id,        button_has_popup_menu_pressed.id,
      button_has_popup_true.id,        generic_container_has_popup_menu.id};

  // combo_box_grouping HasPopup set to true, can collapse, can expand.
  // state is ExpandCollapseState_LeafNode.
  combo_box_grouping_has_popup.role = ax::mojom::Role::kComboBoxGrouping;
  combo_box_grouping_has_popup.SetHasPopup(ax::mojom::HasPopup::kTrue);

  // combo_box_grouping Expanded set, can collapse, cannot expand.
  // state is ExpandCollapseState_Expanded.
  combo_box_grouping_expanded.role = ax::mojom::Role::kComboBoxGrouping;
  combo_box_grouping_expanded.AddState(ax::mojom::State::kExpanded);

  // combo_box_grouping Collapsed set, can expand, cannot collapse.
  // state is ExpandCollapseState_Collapsed.
  combo_box_grouping_collapsed.role = ax::mojom::Role::kComboBoxGrouping;
  combo_box_grouping_collapsed.AddState(ax::mojom::State::kCollapsed);

  // combo_box_grouping is disabled, can neither expand nor collapse.
  // state is ExpandCollapseState_LeafNode.
  combo_box_grouping_disabled.role = ax::mojom::Role::kComboBoxGrouping;
  combo_box_grouping_disabled.SetRestriction(ax::mojom::Restriction::kDisabled);

  // button_has_popup_menu HasPopup set to kMenu and is not STATE_PRESSED.
  // state is ExpandCollapseState_Collapsed.
  button_has_popup_menu.role = ax::mojom::Role::kButton;
  button_has_popup_menu.SetHasPopup(ax::mojom::HasPopup::kMenu);

  // button_has_popup_menu_pressed HasPopup set to kMenu and is STATE_PRESSED.
  // state is ExpandCollapseState_Expanded.
  button_has_popup_menu_pressed.role = ax::mojom::Role::kButton;
  button_has_popup_menu_pressed.SetHasPopup(ax::mojom::HasPopup::kMenu);
  button_has_popup_menu_pressed.SetCheckedState(ax::mojom::CheckedState::kTrue);

  // button_has_popup_true HasPopup set to true but is not a menu.
  // state is ExpandCollapseState_LeafNode.
  button_has_popup_true.role = ax::mojom::Role::kButton;
  button_has_popup_true.SetHasPopup(ax::mojom::HasPopup::kTrue);

  // generic_container_has_popup_menu HasPopup set to menu but with no expand
  // state set.
  // state is ExpandCollapseState_LeafNode.
  generic_container_has_popup_menu.role = ax::mojom::Role::kGenericContainer;
  generic_container_has_popup_menu.SetHasPopup(ax::mojom::HasPopup::kMenu);

  Init(root, combo_box_grouping_has_popup, combo_box_grouping_disabled,
       combo_box_grouping_expanded, combo_box_grouping_collapsed,
       combo_box_grouping_disabled, button_has_popup_menu,
       button_has_popup_menu_pressed, button_has_popup_true,
       generic_container_has_popup_menu);

  // combo_box_grouping HasPopup set to true, can collapse, can expand.
  // state is ExpandCollapseState_LeafNode.
  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetIRawElementProviderSimpleFromChildIndex(0);
  ComPtr<IExpandCollapseProvider> expandcollapse_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(expandcollapse_provider->Collapse());
  EXPECT_HRESULT_SUCCEEDED(expandcollapse_provider->Expand());
  ExpandCollapseState state;
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_LeafNode, state);

  // combo_box_grouping Expanded set, can collapse, cannot expand.
  // state is ExpandCollapseState_Expanded.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(1);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(expandcollapse_provider->Collapse());
  EXPECT_HRESULT_FAILED(expandcollapse_provider->Expand());
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_Expanded, state);

  // combo_box_grouping Collapsed set, can expand, cannot collapse.
  // state is ExpandCollapseState_Collapsed.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(2);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_FAILED(expandcollapse_provider->Collapse());
  EXPECT_HRESULT_SUCCEEDED(expandcollapse_provider->Expand());
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_Collapsed, state);

  // combo_box_grouping is disabled, can neither expand nor collapse.
  // state is ExpandCollapseState_LeafNode.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(3);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_FAILED(expandcollapse_provider->Collapse());
  EXPECT_HRESULT_FAILED(expandcollapse_provider->Expand());
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_LeafNode, state);

  // button_has_popup_menu HasPopup set to kMenu and is not STATE_PRESSED.
  // state is ExpandCollapseState_Collapsed.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(4);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_Collapsed, state);

  // button_has_popup_menu_pressed HasPopup set to kMenu and is STATE_PRESSED.
  // state is ExpandCollapseState_Expanded.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(5);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_Expanded, state);

  // button_has_popup_true HasPopup set to true but is not a menu.
  // state is ExpandCollapseState_LeafNode.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(6);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_LeafNode, state);

  // generic_container_has_popup_menu HasPopup set to menu but with no expand
  // state set.
  // state is ExpandCollapseState_LeafNode.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(7);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_ExpandCollapsePatternId, &expandcollapse_provider));
  ASSERT_NE(nullptr, expandcollapse_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      expandcollapse_provider->get_ExpandCollapseState(&state));
  EXPECT_EQ(ExpandCollapseState_LeafNode, state);
}

TEST_F(AXPlatformNodeWinTest, IInvokeProviderInvoke) {
  ui::AXNodeData root;
  root.id = 1;

  ui::AXNodeData button;
  ui::AXNodeData button_disabled;

  button.id = 2;
  button_disabled.id = 3;

  root.child_ids.push_back(button.id);
  root.child_ids.push_back(button_disabled.id);

  // generic button can be invoked.
  button.role = ax::mojom::Role::kButton;

  // disabled button, cannot be invoked.
  button_disabled.role = ax::mojom::Role::kButton;
  button_disabled.SetRestriction(ax::mojom::Restriction::kDisabled);

  Init(root, button, button_disabled);
  AXNode* button_node = GetRootAsAXNode()->children()[0];

  // generic button can be invoked.
  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetIRawElementProviderSimpleFromChildIndex(0);
  ComPtr<IInvokeProvider> invoke_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_InvokePatternId, &invoke_provider));
  EXPECT_NE(nullptr, invoke_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(invoke_provider->Invoke());
  EXPECT_EQ(button_node, TestAXNodeWrapper::GetNodeFromLastDefaultAction());

  // disabled button, cannot be invoked.
  raw_element_provider_simple = GetIRawElementProviderSimpleFromChildIndex(1);
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_InvokePatternId, &invoke_provider));
  EXPECT_NE(nullptr, invoke_provider.Get());
  EXPECT_UIA_ELEMENTNOTENABLED(invoke_provider->Invoke());
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderNotSupported) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kNone;

  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<ISelectionItemProvider> selection_item_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_SelectionItemPatternId, &selection_item_provider));
  ASSERT_EQ(nullptr, selection_item_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderDisabled) {
  AXNodeData root;
  root.id = 1;
  root.AddIntAttribute(ax::mojom::IntAttribute::kRestriction,
                       static_cast<int>(ax::mojom::Restriction::kDisabled));
  root.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  root.role = ax::mojom::Role::kTab;

  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<ISelectionItemProvider> selection_item_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_SelectionItemPatternId, &selection_item_provider));
  ASSERT_NE(nullptr, selection_item_provider.Get());

  BOOL selected;

  EXPECT_UIA_ELEMENTNOTENABLED(selection_item_provider->AddToSelection());
  EXPECT_UIA_ELEMENTNOTENABLED(selection_item_provider->RemoveFromSelection());
  EXPECT_UIA_ELEMENTNOTENABLED(selection_item_provider->Select());
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderNotSelectable) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTab;

  Init(root);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetRootIRawElementProviderSimple();
  ComPtr<ISelectionItemProvider> selection_item_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_SelectionItemPatternId, &selection_item_provider));
  ASSERT_EQ(nullptr, selection_item_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderSimple) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kListBox;

  AXNodeData option1;
  option1.id = 2;
  option1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);
  option1.role = ax::mojom::Role::kListBoxOption;
  root.child_ids.push_back(option1.id);

  Init(root, option1);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      GetIRawElementProviderSimpleFromChildIndex(0);
  ComPtr<ISelectionItemProvider> option1_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_SelectionItemPatternId, &option1_provider));
  ASSERT_NE(nullptr, option1_provider.Get());

  BOOL selected;

  // Note: TestAXNodeWrapper::AccessibilityPerformAction will
  // flip kSelected for kListBoxOption when the kDoDefault action is fired.

  // Initial State
  EXPECT_HRESULT_SUCCEEDED(option1_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  // AddToSelection should fire event when not selected
  EXPECT_HRESULT_SUCCEEDED(option1_provider->AddToSelection());
  EXPECT_HRESULT_SUCCEEDED(option1_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // AddToSelection should not fire event when selected
  EXPECT_HRESULT_SUCCEEDED(option1_provider->AddToSelection());
  EXPECT_HRESULT_SUCCEEDED(option1_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // RemoveFromSelection should fire event when selected
  EXPECT_HRESULT_SUCCEEDED(option1_provider->RemoveFromSelection());
  EXPECT_HRESULT_SUCCEEDED(option1_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  // RemoveFromSelection should not fire event when not selected
  EXPECT_HRESULT_SUCCEEDED(option1_provider->RemoveFromSelection());
  EXPECT_HRESULT_SUCCEEDED(option1_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  // Select should fire event when not selected
  EXPECT_HRESULT_SUCCEEDED(option1_provider->Select());
  EXPECT_HRESULT_SUCCEEDED(option1_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // Select should not fire event when selected
  EXPECT_HRESULT_SUCCEEDED(option1_provider->Select());
  EXPECT_HRESULT_SUCCEEDED(option1_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderRadioButton) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRadioGroup;

  // CheckedState::kNone
  AXNodeData option1;
  option1.id = 2;
  option1.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kNone));
  option1.role = ax::mojom::Role::kRadioButton;
  root.child_ids.push_back(option1.id);

  // CheckedState::kFalse
  AXNodeData option2;
  option2.id = 3;
  option2.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kFalse));
  option2.role = ax::mojom::Role::kRadioButton;
  root.child_ids.push_back(option2.id);

  // CheckedState::kTrue
  AXNodeData option3;
  option3.id = 4;
  option3.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kTrue));
  option3.role = ax::mojom::Role::kRadioButton;
  root.child_ids.push_back(option3.id);

  // CheckedState::kMixed
  AXNodeData option4;
  option4.id = 5;
  option4.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kMixed));
  option4.role = ax::mojom::Role::kRadioButton;
  root.child_ids.push_back(option4.id);

  Init(root, option1, option2, option3, option4);

  BOOL selected;

  // Option 1, CheckedState::kNone, ISelectionItemProvider is not supported.
  ComPtr<ISelectionItemProvider> option1_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(0)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option1_provider));
  ASSERT_EQ(nullptr, option1_provider.Get());

  // Option 2, CheckedState::kFalse.
  ComPtr<ISelectionItemProvider> option2_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(1)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option2_provider));
  ASSERT_NE(nullptr, option2_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(option2_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  EXPECT_HRESULT_SUCCEEDED(option2_provider->Select());
  EXPECT_HRESULT_SUCCEEDED(option2_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // Option 3, CheckedState::kTrue.
  ComPtr<ISelectionItemProvider> option3_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(2)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option3_provider));
  ASSERT_NE(nullptr, option3_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(option3_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  EXPECT_HRESULT_SUCCEEDED(option3_provider->RemoveFromSelection());
  EXPECT_HRESULT_SUCCEEDED(option3_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  EXPECT_HRESULT_SUCCEEDED(option3_provider->AddToSelection());
  EXPECT_HRESULT_SUCCEEDED(option3_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // Option 4, CheckedState::kMixed, ISelectionItemProvider is not supported.
  ComPtr<ISelectionItemProvider> option4_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(3)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option4_provider));
  ASSERT_EQ(nullptr, option4_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderMenuItemRadio) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kMenu;

  // CheckedState::kNone
  AXNodeData option1;
  option1.id = 2;
  option1.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kNone));
  option1.role = ax::mojom::Role::kMenuItemRadio;
  root.child_ids.push_back(option1.id);

  // CheckedState::kFalse
  AXNodeData option2;
  option2.id = 3;
  option2.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kFalse));
  option2.role = ax::mojom::Role::kMenuItemRadio;
  root.child_ids.push_back(option2.id);

  // CheckedState::kTrue
  AXNodeData option3;
  option3.id = 4;
  option3.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kTrue));
  option3.role = ax::mojom::Role::kMenuItemRadio;
  root.child_ids.push_back(option3.id);

  // CheckedState::kMixed
  AXNodeData option4;
  option4.id = 5;
  option4.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                          static_cast<int>(ax::mojom::CheckedState::kMixed));
  option4.role = ax::mojom::Role::kMenuItemRadio;
  root.child_ids.push_back(option4.id);

  Init(root, option1, option2, option3, option4);

  BOOL selected;

  // Option 1, CheckedState::kNone, ISelectionItemProvider is not supported.
  ComPtr<ISelectionItemProvider> option1_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(0)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option1_provider));
  ASSERT_EQ(nullptr, option1_provider.Get());

  // Option 2, CheckedState::kFalse.
  ComPtr<ISelectionItemProvider> option2_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(1)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option2_provider));
  ASSERT_NE(nullptr, option2_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(option2_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  EXPECT_HRESULT_SUCCEEDED(option2_provider->Select());
  EXPECT_HRESULT_SUCCEEDED(option2_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // Option 3, CheckedState::kTrue.
  ComPtr<ISelectionItemProvider> option3_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(2)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option3_provider));
  ASSERT_NE(nullptr, option3_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(option3_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  EXPECT_HRESULT_SUCCEEDED(option3_provider->RemoveFromSelection());
  EXPECT_HRESULT_SUCCEEDED(option3_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  EXPECT_HRESULT_SUCCEEDED(option3_provider->AddToSelection());
  EXPECT_HRESULT_SUCCEEDED(option3_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // Option 4, CheckedState::kMixed, ISelectionItemProvider is not supported.
  ComPtr<ISelectionItemProvider> option4_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(3)->GetPatternProvider(
          UIA_SelectionItemPatternId, &option4_provider));
  ASSERT_EQ(nullptr, option4_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderTable) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTable;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData cell1;
  cell1.id = 3;
  cell1.role = ax::mojom::Role::kCell;
  cell1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);
  row1.child_ids.push_back(cell1.id);

  Init(root, row1, cell1);

  ComPtr<ISelectionItemProvider> selection_item_provider;
  EXPECT_HRESULT_SUCCEEDED(
      GetIRawElementProviderSimpleFromChildIndex(0)->GetPatternProvider(
          UIA_SelectionItemPatternId, &selection_item_provider));
  ASSERT_EQ(nullptr, selection_item_provider.Get());
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderGrid) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kGrid;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData cell1;
  cell1.id = 3;
  cell1.role = ax::mojom::Role::kCell;
  cell1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);
  row1.child_ids.push_back(cell1.id);

  Init(root, row1, cell1);

  const auto* row = GetRootAsAXNode()->children()[0];
  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      QueryInterfaceFromNode<IRawElementProviderSimple>(row->children()[0]);

  ComPtr<ISelectionItemProvider> selection_item_provider;
  EXPECT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_SelectionItemPatternId, &selection_item_provider));
  ASSERT_NE(nullptr, selection_item_provider.Get());

  BOOL selected;

  // Note: TestAXNodeWrapper::AccessibilityPerformAction will
  // flip kSelected for kCell when the kDoDefault action is fired.

  // Initial State
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  // AddToSelection should fire event when not selected
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->AddToSelection());
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // AddToSelection should not fire event when selected
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->AddToSelection());
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // RemoveFromSelection should fire event when selected
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->RemoveFromSelection());
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  // RemoveFromSelection should not fire event when not selected
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->RemoveFromSelection());
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_FALSE(selected);

  // Select should fire event when not selected
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->Select());
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);

  // Select should not fire event when selected
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->Select());
  EXPECT_HRESULT_SUCCEEDED(selection_item_provider->get_IsSelected(&selected));
  EXPECT_TRUE(selected);
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderGetSelectionContainer) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kGrid;

  AXNodeData row1;
  row1.id = 2;
  row1.role = ax::mojom::Role::kRow;
  root.child_ids.push_back(row1.id);

  AXNodeData cell1;
  cell1.id = 3;
  cell1.role = ax::mojom::Role::kCell;
  cell1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);
  row1.child_ids.push_back(cell1.id);

  Init(root, row1, cell1);

  ComPtr<IRawElementProviderSimple> container_provider =
      GetRootIRawElementProviderSimple();

  const auto* row = GetRootAsAXNode()->children()[0];
  ComPtr<ISelectionItemProvider> item_provider =
      QueryInterfaceFromNode<ISelectionItemProvider>(row->children()[0]);

  ComPtr<IRawElementProviderSimple> container;
  EXPECT_HRESULT_SUCCEEDED(item_provider->get_SelectionContainer(&container));
  EXPECT_NE(nullptr, container);
  EXPECT_EQ(container, container_provider);
}

TEST_F(AXPlatformNodeWinTest, ISelectionItemProviderSelectFollowFocus) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kTabList;

  AXNodeData tab1;
  tab1.id = 2;
  tab1.role = ax::mojom::Role::kTab;
  tab1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, false);
  tab1.SetDefaultActionVerb(ax::mojom::DefaultActionVerb::kClick);
  root.child_ids.push_back(tab1.id);

  Init(root, tab1);

  auto* tab1_node = GetRootAsAXNode()->children()[0];
  ComPtr<IRawElementProviderSimple> tab1_raw_element_provider_simple =
      QueryInterfaceFromNode<IRawElementProviderSimple>(tab1_node);
  ASSERT_NE(nullptr, tab1_raw_element_provider_simple.Get());

  ComPtr<IRawElementProviderFragment> tab1_raw_element_provider_fragment =
      IRawElementProviderFragmentFromNode(tab1_node);
  ASSERT_NE(nullptr, tab1_raw_element_provider_fragment.Get());

  ComPtr<ISelectionItemProvider> tab1_selection_item_provider;
  EXPECT_HRESULT_SUCCEEDED(tab1_raw_element_provider_simple->GetPatternProvider(
      UIA_SelectionItemPatternId, &tab1_selection_item_provider));
  ASSERT_NE(nullptr, tab1_selection_item_provider.Get());

  BOOL is_selected;
  // Before setting focus to "tab1", validate that "tab1" has selected=false.
  tab1_selection_item_provider->get_IsSelected(&is_selected);
  EXPECT_FALSE(is_selected);

  // Setting focus on "tab1" will result in selected=true.
  tab1_raw_element_provider_fragment->SetFocus();
  tab1_selection_item_provider->get_IsSelected(&is_selected);
  EXPECT_TRUE(is_selected);

  // Verify that we can still trigger action::kDoDefault through Select().
  EXPECT_HRESULT_SUCCEEDED(tab1_selection_item_provider->Select());
  tab1_selection_item_provider->get_IsSelected(&is_selected);
  EXPECT_TRUE(is_selected);
  EXPECT_EQ(tab1_node, TestAXNodeWrapper::GetNodeFromLastDefaultAction());
  // Verify that after Select(), "tab1" is still selected.
  tab1_selection_item_provider->get_IsSelected(&is_selected);
  EXPECT_TRUE(is_selected);

  // Since last Select() performed |action::kDoDefault|, which set
  // |kSelectedFromFocus| to false. Calling Select() again will not perform
  // |action::kDoDefault| again.
  TestAXNodeWrapper::SetNodeFromLastDefaultAction(nullptr);
  EXPECT_HRESULT_SUCCEEDED(tab1_selection_item_provider->Select());
  tab1_selection_item_provider->get_IsSelected(&is_selected);
  EXPECT_TRUE(is_selected);
  // Verify that after Select(),|action::kDoDefault| was not triggered on
  // "tab1".
  EXPECT_EQ(nullptr, TestAXNodeWrapper::GetNodeFromLastDefaultAction());
}

TEST_F(AXPlatformNodeWinTest, IValueProvider_GetValue) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kProgressIndicator;
  child1.AddFloatAttribute(ax::mojom::FloatAttribute::kValueForRange, 3.0f);
  root.child_ids.push_back(child1.id);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kTextField;
  child2.AddState(ax::mojom::State::kEditable);
  child2.AddStringAttribute(ax::mojom::StringAttribute::kValue, "test");
  root.child_ids.push_back(child2.id);

  AXNodeData child3;
  child3.id = 4;
  child3.role = ax::mojom::Role::kTextField;
  child3.AddStringAttribute(ax::mojom::StringAttribute::kValue, "test");
  child3.AddIntAttribute(ax::mojom::IntAttribute::kRestriction,
                         static_cast<int>(ax::mojom::Restriction::kReadOnly));
  root.child_ids.push_back(child3.id);

  Init(root, child1, child2, child3);

  ScopedBstr bstr_value;

  EXPECT_HRESULT_SUCCEEDED(
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[0])
          ->get_Value(bstr_value.Receive()));
  EXPECT_STREQ(L"3", bstr_value.Get());
  bstr_value.Reset();

  EXPECT_HRESULT_SUCCEEDED(
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[1])
          ->get_Value(bstr_value.Receive()));
  EXPECT_STREQ(L"test", bstr_value.Get());
  bstr_value.Reset();

  EXPECT_HRESULT_SUCCEEDED(
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[2])
          ->get_Value(bstr_value.Receive()));
  EXPECT_STREQ(L"test", bstr_value.Get());
  bstr_value.Reset();
}

TEST_F(AXPlatformNodeWinTest, IValueProvider_SetValue) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kProgressIndicator;
  child1.AddFloatAttribute(ax::mojom::FloatAttribute::kValueForRange, 3.0f);
  root.child_ids.push_back(child1.id);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kTextField;
  child2.AddStringAttribute(ax::mojom::StringAttribute::kValue, "test");
  root.child_ids.push_back(child2.id);

  AXNodeData child3;
  child3.id = 4;
  child3.role = ax::mojom::Role::kTextField;
  child3.AddStringAttribute(ax::mojom::StringAttribute::kValue, "test");
  child3.AddIntAttribute(ax::mojom::IntAttribute::kRestriction,
                         static_cast<int>(ax::mojom::Restriction::kReadOnly));
  root.child_ids.push_back(child3.id);

  Init(root, child1, child2, child3);

  ComPtr<IValueProvider> root_provider =
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode());
  ComPtr<IValueProvider> provider1 =
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[0]);
  ComPtr<IValueProvider> provider2 =
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[1]);
  ComPtr<IValueProvider> provider3 =
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[2]);

  ScopedBstr bstr_value;

  // Note: TestAXNodeWrapper::AccessibilityPerformAction will
  // modify the value when the kSetValue action is fired.

  EXPECT_UIA_ELEMENTNOTENABLED(provider1->SetValue(L"2"));
  EXPECT_HRESULT_SUCCEEDED(provider1->get_Value(bstr_value.Receive()));
  EXPECT_STREQ(L"3", bstr_value.Get());
  bstr_value.Reset();

  EXPECT_HRESULT_SUCCEEDED(provider2->SetValue(L"changed"));
  EXPECT_HRESULT_SUCCEEDED(provider2->get_Value(bstr_value.Receive()));
  EXPECT_STREQ(L"changed", bstr_value.Get());
  bstr_value.Reset();

  EXPECT_UIA_ELEMENTNOTENABLED(provider3->SetValue(L"changed"));
  EXPECT_HRESULT_SUCCEEDED(provider3->get_Value(bstr_value.Receive()));
  EXPECT_STREQ(L"test", bstr_value.Get());
  bstr_value.Reset();
}

TEST_F(AXPlatformNodeWinTest, IValueProvider_IsReadOnly) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;

  AXNodeData child1;
  child1.id = 2;
  child1.role = ax::mojom::Role::kTextField;
  child1.AddState(ax::mojom::State::kEditable);
  root.child_ids.push_back(child1.id);

  AXNodeData child2;
  child2.id = 3;
  child2.role = ax::mojom::Role::kTextField;
  child2.AddIntAttribute(ax::mojom::IntAttribute::kRestriction,
                         static_cast<int>(ax::mojom::Restriction::kReadOnly));
  root.child_ids.push_back(child2.id);

  AXNodeData child3;
  child3.id = 4;
  child3.role = ax::mojom::Role::kTextField;
  child3.AddIntAttribute(ax::mojom::IntAttribute::kRestriction,
                         static_cast<int>(ax::mojom::Restriction::kDisabled));
  root.child_ids.push_back(child3.id);

  AXNodeData child4;
  child4.id = 5;
  child4.role = ax::mojom::Role::kLink;
  root.child_ids.push_back(child4.id);

  Init(root, child1, child2, child3, child4);

  BOOL is_readonly = false;

  EXPECT_HRESULT_SUCCEEDED(
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[0])
          ->get_IsReadOnly(&is_readonly));
  EXPECT_FALSE(is_readonly);

  EXPECT_HRESULT_SUCCEEDED(
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[1])
          ->get_IsReadOnly(&is_readonly));
  EXPECT_TRUE(is_readonly);

  EXPECT_HRESULT_SUCCEEDED(
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[2])
          ->get_IsReadOnly(&is_readonly));
  EXPECT_TRUE(is_readonly);

  EXPECT_HRESULT_SUCCEEDED(
      QueryInterfaceFromNode<IValueProvider>(GetRootAsAXNode()->children()[3])
          ->get_IsReadOnly(&is_readonly));
  EXPECT_TRUE(is_readonly);
}

TEST_F(AXPlatformNodeWinTest, IScrollProviderSetScrollPercent) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kGenericContainer;
  root.AddIntAttribute(ax::mojom::IntAttribute::kScrollX, 0);
  root.AddIntAttribute(ax::mojom::IntAttribute::kScrollXMin, 0);
  root.AddIntAttribute(ax::mojom::IntAttribute::kScrollXMax, 100);

  root.AddIntAttribute(ax::mojom::IntAttribute::kScrollY, 60);
  root.AddIntAttribute(ax::mojom::IntAttribute::kScrollYMin, 10);
  root.AddIntAttribute(ax::mojom::IntAttribute::kScrollYMax, 60);

  Init(root);

  ComPtr<IScrollProvider> scroll_provider =
      QueryInterfaceFromNode<IScrollProvider>(GetRootAsAXNode());
  double x_scroll_percent;
  double y_scroll_percent;

  // Set x scroll percent: 0%, y scroll percent: 0%.
  // Expected x scroll percent: 0%, y scroll percent: 0%.
  EXPECT_HRESULT_SUCCEEDED(scroll_provider->SetScrollPercent(0, 0));
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_HorizontalScrollPercent(&x_scroll_percent));
  EXPECT_EQ(x_scroll_percent, 0);
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_VerticalScrollPercent(&y_scroll_percent));
  EXPECT_EQ(y_scroll_percent, 0);

  // Set x scroll percent: 100%, y scroll percent: 100%.
  // Expected x scroll percent: 100%, y scroll percent: 100%.
  EXPECT_HRESULT_SUCCEEDED(scroll_provider->SetScrollPercent(100, 100));
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_HorizontalScrollPercent(&x_scroll_percent));
  EXPECT_EQ(x_scroll_percent, 100);
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_VerticalScrollPercent(&y_scroll_percent));
  EXPECT_EQ(y_scroll_percent, 100);

  // Set x scroll percent: 500%, y scroll percent: 600%.
  // Expected x scroll percent: 100%, y scroll percent: 100%.
  EXPECT_HRESULT_SUCCEEDED(scroll_provider->SetScrollPercent(500, 600));
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_HorizontalScrollPercent(&x_scroll_percent));
  EXPECT_EQ(x_scroll_percent, 100);
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_VerticalScrollPercent(&y_scroll_percent));
  EXPECT_EQ(y_scroll_percent, 100);

  // Set x scroll percent: -100%, y scroll percent: -200%.
  // Expected x scroll percent: 0%, y scroll percent: 0%.
  EXPECT_HRESULT_SUCCEEDED(scroll_provider->SetScrollPercent(-100, -200));
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_HorizontalScrollPercent(&x_scroll_percent));
  EXPECT_EQ(x_scroll_percent, 0);
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_VerticalScrollPercent(&y_scroll_percent));
  EXPECT_EQ(y_scroll_percent, 0);

  // Set x scroll percent: 12%, y scroll percent: 34%.
  // Expected x scroll percent: 12%, y scroll percent: 34%.
  EXPECT_HRESULT_SUCCEEDED(scroll_provider->SetScrollPercent(12, 34));
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_HorizontalScrollPercent(&x_scroll_percent));
  EXPECT_EQ(x_scroll_percent, 12);
  EXPECT_HRESULT_SUCCEEDED(
      scroll_provider->get_VerticalScrollPercent(&y_scroll_percent));
  EXPECT_EQ(y_scroll_percent, 34);
}

TEST_F(AXPlatformNodeWinTest, MojoEventToUIAPropertyTest) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kCheckBox;
  root.AddIntAttribute(ax::mojom::IntAttribute::kCheckedState,
                       static_cast<int>(ax::mojom::CheckedState::kMixed));

  Init(root);

  ComPtr<AXPlatformNodeWin> platform_node =
      QueryInterfaceFromNode<AXPlatformNodeWin>(GetRootAsAXNode());
  EXPECT_EQ(
      platform_node->MojoEventToUIAProperty(ax::mojom::Event::kValueChanged),
      UIA_ToggleToggleStatePropertyId);
}

}  // namespace ui
