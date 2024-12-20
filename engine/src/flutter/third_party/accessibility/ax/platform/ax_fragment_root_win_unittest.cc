// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_fragment_root_win.h"
#include "ax_platform_node_win.h"
#include "ax_platform_node_win_unittest.h"
#include "test_ax_node_wrapper.h"

#include <UIAutomationClient.h>
#include <UIAutomationCoreApi.h>

#include "base/auto_reset.h"
#include "base/win/scoped_safearray.h"
#include "base/win/scoped_variant.h"
#include "gtest/gtest.h"
#include "uia_registrar_win.h"

using base::win::ScopedVariant;
using Microsoft::WRL::ComPtr;

namespace ui {

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

class AXFragmentRootTest : public AXPlatformNodeWinTest {
 public:
  AXFragmentRootTest() = default;
  ~AXFragmentRootTest() override = default;
  AXFragmentRootTest(const AXFragmentRootTest&) = delete;
  AXFragmentRootTest& operator=(const AXFragmentRootTest&) = delete;
};

TEST_F(AXFragmentRootTest, UIAFindItemByPropertyUniqueId) {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("root");
  root.child_ids = {2, 3};

  AXNodeData text1;
  text1.id = 2;
  text1.role = ax::mojom::Role::kStaticText;
  text1.SetName("text1");

  AXNodeData button;
  button.id = 3;
  button.role = ax::mojom::Role::kButton;
  button.SetName("button");
  button.child_ids = {4};

  AXNodeData text2;
  text2.id = 4;
  text2.role = ax::mojom::Role::kStaticText;
  text2.SetName("text2");

  Init(root, text1, button, text2);
  InitFragmentRoot();

  ComPtr<IRawElementProviderSimple> root_raw_element_provider_simple;
  ax_fragment_root_->GetNativeViewAccessible()->QueryInterface(
      IID_PPV_ARGS(&root_raw_element_provider_simple));
  ComPtr<IRawElementProviderSimple> text1_raw_element_provider_simple =
      GetIRawElementProviderSimpleFromChildIndex(0);
  ComPtr<IRawElementProviderSimple> button_raw_element_provider_simple =
      GetIRawElementProviderSimpleFromChildIndex(1);

  AXNode* text1_node = GetRootAsAXNode()->children()[0];
  AXNode* button_node = GetRootAsAXNode()->children()[1];

  ComPtr<IItemContainerProvider> item_container_provider;
  EXPECT_HRESULT_SUCCEEDED(root_raw_element_provider_simple->GetPatternProvider(
      UIA_ItemContainerPatternId, &item_container_provider));
  ASSERT_NE(nullptr, item_container_provider.Get());

  ScopedVariant unique_id_variant;
  int32_t unique_id;
  ComPtr<IRawElementProviderSimple> result;

  // When |start_after_element| is an invalid element, we should fail at finding
  // the item.
  {
    unique_id = AXPlatformNodeFromNode(GetRootAsAXNode())->GetUniqueId();
    unique_id_variant.Set(SysAllocString(reinterpret_cast<const wchar_t*>(
        base::NumberToString16(-unique_id).c_str())));

    ComPtr<IRawElementProviderSimple> invalid_element_provider_simple;
    EXPECT_HRESULT_SUCCEEDED(
        MockIRawElementProviderSimple::CreateMockIRawElementProviderSimple(
            &invalid_element_provider_simple));

    EXPECT_HRESULT_FAILED(item_container_provider->FindItemByProperty(
        invalid_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    result.Reset();
    unique_id_variant.Release();
  }

  // Fetch the AxUniqueId of "root", and verify we can retrieve its
  // corresponding IRawElementProviderSimple through FindItemByProperty().
  {
    unique_id = AXPlatformNodeFromNode(GetRootAsAXNode())->GetUniqueId();
    unique_id_variant.Set(SysAllocString(reinterpret_cast<const wchar_t*>(
        base::NumberToString16(-unique_id).c_str())));

    // When |start_after_element| of FindItemByProperty() is nullptr, we should
    // be able to find "text1".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        nullptr, UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"root");
    result.Reset();

    // When |start_after_element| of FindItemByProperty() is "text1", there
    // should be no element found, since "text1" comes after the element we are
    // looking for.
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        text1_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_EQ(nullptr, result.Get());
    result.Reset();

    // When |start_after_element| of FindItemByProperty() is "button", there
    // should be no element found, since "button" comes after the element we are
    // looking for.
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        button_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_EQ(nullptr, result.Get());

    result.Reset();
    unique_id_variant.Release();
  }

  // Fetch the AxUniqueId of "text1", and verify if we can retrieve its
  // corresponding IRawElementProviderSimple through FindItemByProperty().
  {
    unique_id = AXPlatformNodeFromNode(text1_node)->GetUniqueId();
    unique_id_variant.Set(SysAllocString(reinterpret_cast<const wchar_t*>(
        base::NumberToString16(-unique_id).c_str())));

    // When |start_after_element| of FindItemByProperty() is nullptr, we should
    // be able to find "text1".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        nullptr, UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"text1");
    result.Reset();

    // When |start_after_element| of FindItemByProperty() is "text1", there
    // should be no element found, since "text1" equals the element we are
    // looking for.
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        text1_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_EQ(nullptr, result.Get());
    result.Reset();

    // When |start_after_element| of FindItemByProperty() is "button", there
    // should be no element found, since "button" comes after the element we are
    // looking for.
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        button_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_EQ(nullptr, result.Get());
    result.Reset();
    unique_id_variant.Release();
  }

  // Fetch the AxUniqueId of "button", and verify we can retrieve its
  // corresponding IRawElementProviderSimple through FindItemByProperty().
  {
    unique_id = AXPlatformNodeFromNode(button_node)->GetUniqueId();
    unique_id_variant.Set(SysAllocString(reinterpret_cast<const wchar_t*>(
        base::NumberToString16(-unique_id).c_str())));

    // When |start_after_element| of FindItemByProperty() is nullptr, we should
    // be able to find "button".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        nullptr, UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"button");
    result.Reset();

    // When |start_after_element| of FindItemByProperty() is "text1", we should
    // be able to find "button".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        text1_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"button");
    result.Reset();

    // When |start_after_element| of FindItemByProperty() is "button", there
    // should be no element found, since "button" equals the element we are
    // looking for.
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        button_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_EQ(nullptr, result.Get());
    result.Reset();
    unique_id_variant.Release();
  }

  // Fetch the AxUniqueId of "text2", and verify we can retrieve its
  // corresponding IRawElementProviderSimple through FindItemByProperty().
  {
    unique_id =
        AXPlatformNodeFromNode(button_node->children()[0])->GetUniqueId();
    unique_id_variant.Set(SysAllocString(reinterpret_cast<const wchar_t*>(
        base::NumberToString16(-unique_id).c_str())));

    // When |start_after_element| of FindItemByProperty() is nullptr, we should
    // be able to find "text2".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        nullptr, UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"text2");

    // When |start_after_element| of FindItemByProperty() is root, we should
    // be able to find "text2".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        root_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"text2");

    // When |start_after_element| of FindItemByProperty() is "text1", we should
    // be able to find "text2".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        text1_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"text2");

    // When |start_after_element| of FindItemByProperty() is "button", we should
    // be able to find "text2".
    EXPECT_HRESULT_SUCCEEDED(item_container_provider->FindItemByProperty(
        button_raw_element_provider_simple.Get(),
        UiaRegistrarWin::GetInstance().GetUiaUniqueIdPropertyId(),
        unique_id_variant, &result));
    EXPECT_UIA_BSTR_EQ(result, UIA_NamePropertyId, L"text2");
  }
}

TEST_F(AXFragmentRootTest, TestUIAGetFragmentRoot) {
  AXNodeData root;
  root.id = 1;
  Init(root);
  InitFragmentRoot();

  ComPtr<IRawElementProviderFragmentRoot> expected_fragment_root =
      GetFragmentRoot();
  ComPtr<IRawElementProviderFragment> fragment_provider;
  expected_fragment_root.As(&fragment_provider);

  ComPtr<IRawElementProviderFragmentRoot> actual_fragment_root;
  EXPECT_HRESULT_SUCCEEDED(
      fragment_provider->get_FragmentRoot(&actual_fragment_root));
  EXPECT_EQ(expected_fragment_root.Get(), actual_fragment_root.Get());
}

TEST_F(AXFragmentRootTest, DISABLED_TestUIAElementProviderFromPoint) {
  AXNodeData root_data;
  root_data.id = 1;
  root_data.relative_bounds.bounds = gfx::RectF(0, 0, 80, 80);

  AXNodeData element1_data;
  element1_data.id = 2;
  element1_data.relative_bounds.bounds = gfx::RectF(0, 0, 50, 50);
  root_data.child_ids.push_back(element1_data.id);

  AXNodeData element2_data;
  element2_data.id = 3;
  element2_data.relative_bounds.bounds = gfx::RectF(0, 50, 30, 30);
  root_data.child_ids.push_back(element2_data.id);

  Init(root_data, element1_data, element2_data);
  InitFragmentRoot();

  AXNode* root_node = GetRootAsAXNode();
  AXNode* element1_node = root_node->children()[0];
  AXNode* element2_node = root_node->children()[1];

  ComPtr<IRawElementProviderFragmentRoot> fragment_root_prov(GetFragmentRoot());
  ComPtr<IRawElementProviderFragment> root_provider(
      GetRootIRawElementProviderFragment());
  ComPtr<IRawElementProviderFragment> element1_provider =
      QueryInterfaceFromNode<IRawElementProviderFragment>(element1_node);
  ComPtr<IRawElementProviderFragment> element2_provider =
      QueryInterfaceFromNode<IRawElementProviderFragment>(element2_node);

  ComPtr<IRawElementProviderFragment> provider_from_point;
  EXPECT_HRESULT_SUCCEEDED(fragment_root_prov->ElementProviderFromPoint(
      23, 31, &provider_from_point));
  EXPECT_EQ(element1_provider.Get(), provider_from_point.Get());

  EXPECT_HRESULT_SUCCEEDED(fragment_root_prov->ElementProviderFromPoint(
      23, 67, &provider_from_point));
  EXPECT_EQ(element2_provider.Get(), provider_from_point.Get());

  EXPECT_HRESULT_SUCCEEDED(fragment_root_prov->ElementProviderFromPoint(
      47, 67, &provider_from_point));
  EXPECT_EQ(root_provider.Get(), provider_from_point.Get());

  // This is on node 1 with scale factor of 1.5.
  std::unique_ptr<base::AutoReset<float>> scale_factor_reset =
      TestAXNodeWrapper::SetScaleFactor(1.5);
  EXPECT_HRESULT_SUCCEEDED(fragment_root_prov->ElementProviderFromPoint(
      60, 60, &provider_from_point));
  EXPECT_EQ(element1_provider.Get(), provider_from_point.Get());
}

TEST_F(AXFragmentRootTest, TestUIAGetFocus) {
  AXNodeData root_data;
  root_data.id = 1;

  AXNodeData element1_data;
  element1_data.id = 2;
  root_data.child_ids.push_back(element1_data.id);

  AXNodeData element2_data;
  element2_data.id = 3;
  root_data.child_ids.push_back(element2_data.id);

  Init(root_data, element1_data, element2_data);
  InitFragmentRoot();

  AXNode* root_node = GetRootAsAXNode();
  AXNode* element1_node = root_node->children()[0];
  AXNode* element2_node = root_node->children()[1];

  ComPtr<IRawElementProviderFragmentRoot> fragment_root_prov(GetFragmentRoot());
  ComPtr<IRawElementProviderFragment> root_provider(
      GetRootIRawElementProviderFragment());
  ComPtr<IRawElementProviderFragment> element1_provider =
      QueryInterfaceFromNode<IRawElementProviderFragment>(element1_node);
  ComPtr<IRawElementProviderFragment> element2_provider =
      QueryInterfaceFromNode<IRawElementProviderFragment>(element2_node);

  ComPtr<IRawElementProviderFragment> focused_fragment;
  EXPECT_HRESULT_SUCCEEDED(root_provider->SetFocus());
  EXPECT_HRESULT_SUCCEEDED(fragment_root_prov->GetFocus(&focused_fragment));
  EXPECT_EQ(root_provider.Get(), focused_fragment.Get());

  EXPECT_HRESULT_SUCCEEDED(element1_provider->SetFocus());
  EXPECT_HRESULT_SUCCEEDED(fragment_root_prov->GetFocus(&focused_fragment));
  EXPECT_EQ(element1_provider.Get(), focused_fragment.Get());

  EXPECT_HRESULT_SUCCEEDED(element2_provider->SetFocus());
  EXPECT_HRESULT_SUCCEEDED(fragment_root_prov->GetFocus(&focused_fragment));
  EXPECT_EQ(element2_provider.Get(), focused_fragment.Get());
}

TEST_F(AXFragmentRootTest, TestUIAErrorHandling) {
  AXNodeData root;
  root.id = 1;
  Init(root);
  InitFragmentRoot();

  ComPtr<IRawElementProviderSimple> simple_provider =
      GetRootIRawElementProviderSimple();
  ComPtr<IRawElementProviderFragment> fragment_provider =
      GetRootIRawElementProviderFragment();
  ComPtr<IRawElementProviderFragmentRoot> fragment_root_provider =
      GetFragmentRoot();

  SetTree(std::make_unique<AXTree>());
  ax_fragment_root_.reset(nullptr);

  ComPtr<IRawElementProviderSimple> returned_simple_provider;
  ComPtr<IRawElementProviderFragment> returned_fragment_provider;
  ComPtr<IRawElementProviderFragmentRoot> returned_fragment_root_provider;
  base::win::ScopedSafearray returned_runtime_id;

  EXPECT_EQ(
      static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
      simple_provider->get_HostRawElementProvider(&returned_simple_provider));

  EXPECT_EQ(
      static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
      fragment_provider->get_FragmentRoot(&returned_fragment_root_provider));

  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_provider->GetRuntimeId(returned_runtime_id.Receive()));

  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_root_provider->ElementProviderFromPoint(
                67, 23, &returned_fragment_provider));

  EXPECT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            fragment_root_provider->GetFocus(&returned_fragment_provider));
}

TEST_F(AXFragmentRootTest, TestGetChildCount) {
  AXNodeData root;
  root.id = 1;
  Init(root);
  InitFragmentRoot();

  AXPlatformNodeDelegate* fragment_root = ax_fragment_root_.get();
  EXPECT_EQ(1, fragment_root->GetChildCount());

  test_fragment_root_delegate_->child_ = nullptr;
  EXPECT_EQ(0, fragment_root->GetChildCount());
}

TEST_F(AXFragmentRootTest, TestChildAtIndex) {
  AXNodeData root;
  root.id = 1;
  Init(root);
  InitFragmentRoot();

  gfx::NativeViewAccessible native_view_accessible =
      AXPlatformNodeFromNode(GetRootAsAXNode())->GetNativeViewAccessible();
  AXPlatformNodeDelegate* fragment_root = ax_fragment_root_.get();
  EXPECT_EQ(native_view_accessible, fragment_root->ChildAtIndex(0));
  EXPECT_EQ(nullptr, fragment_root->ChildAtIndex(1));

  test_fragment_root_delegate_->child_ = nullptr;
  EXPECT_EQ(nullptr, fragment_root->ChildAtIndex(0));
}

TEST_F(AXFragmentRootTest, TestGetParent) {
  AXNodeData root;
  root.id = 1;
  Init(root);
  InitFragmentRoot();

  AXPlatformNodeDelegate* fragment_root = ax_fragment_root_.get();
  EXPECT_EQ(nullptr, fragment_root->GetParent());

  gfx::NativeViewAccessible native_view_accessible =
      AXPlatformNodeFromNode(GetRootAsAXNode())->GetNativeViewAccessible();
  test_fragment_root_delegate_->parent_ = native_view_accessible;
  EXPECT_EQ(native_view_accessible, fragment_root->GetParent());
}

TEST_F(AXFragmentRootTest, TestGetPropertyValue) {
  AXNodeData root;
  root.id = 1;
  Init(root);
  InitFragmentRoot();

  ComPtr<IRawElementProviderSimple> root_provider;
  ax_fragment_root_->GetNativeViewAccessible()->QueryInterface(
      IID_PPV_ARGS(&root_provider));

  // IsControlElement and IsContentElement should follow the setting on the
  // fragment root delegate.
  test_fragment_root_delegate_->is_control_element_ = true;
  ScopedVariant result;
  EXPECT_HRESULT_SUCCEEDED(root_provider->GetPropertyValue(
      UIA_IsControlElementPropertyId, result.Receive()));
  EXPECT_EQ(result.type(), VT_BOOL);
  EXPECT_EQ(result.ptr()->boolVal, VARIANT_TRUE);
  EXPECT_HRESULT_SUCCEEDED(root_provider->GetPropertyValue(
      UIA_IsContentElementPropertyId, result.Receive()));
  EXPECT_EQ(result.type(), VT_BOOL);
  EXPECT_EQ(result.ptr()->boolVal, VARIANT_TRUE);

  test_fragment_root_delegate_->is_control_element_ = false;
  EXPECT_HRESULT_SUCCEEDED(root_provider->GetPropertyValue(
      UIA_IsControlElementPropertyId, result.Receive()));
  EXPECT_EQ(result.type(), VT_BOOL);
  EXPECT_EQ(result.ptr()->boolVal, VARIANT_FALSE);
  EXPECT_HRESULT_SUCCEEDED(root_provider->GetPropertyValue(
      UIA_IsContentElementPropertyId, result.Receive()));
  EXPECT_EQ(result.type(), VT_BOOL);
  EXPECT_EQ(result.ptr()->boolVal, VARIANT_FALSE);

  // Other properties should return VT_EMPTY.
  EXPECT_HRESULT_SUCCEEDED(root_provider->GetPropertyValue(
      UIA_ControlTypePropertyId, result.Receive()));
  EXPECT_EQ(result.type(), VT_EMPTY);
}

TEST_F(AXFragmentRootTest, TestUIAMultipleFragmentRoots) {
  // Consider the following platform-neutral tree:
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

  ui::AXNodeData top_fragment_root_n1;
  top_fragment_root_n1.id = 1;

  ui::AXNodeData sibling_n2;
  sibling_n2.id = 2;

  ui::AXNodeData child_fragment_root_n3;
  child_fragment_root_n3.id = 3;

  ui::AXNodeData sibling_n6;
  sibling_n6.id = 6;
  ui::AXNodeData sibling_n7;
  sibling_n7.id = 7;

  child_fragment_root_n3.child_ids = {6, 7};

  ui::AXNodeData sibling_n4;
  sibling_n4.id = 4;

  ui::AXNodeData child_fragment_root_n5;
  child_fragment_root_n5.id = 5;

  ui::AXNodeData sibling_n8;
  sibling_n8.id = 8;
  ui::AXNodeData sibling_n9;
  sibling_n9.id = 9;

  child_fragment_root_n5.child_ids = {8, 9};
  top_fragment_root_n1.child_ids = {2, 3, 4, 5};

  ui::AXTreeUpdate update;
  update.has_tree_data = true;
  update.root_id = top_fragment_root_n1.id;
  update.nodes = {top_fragment_root_n1,
                  sibling_n2,
                  child_fragment_root_n3,
                  sibling_n6,
                  sibling_n7,
                  sibling_n4,
                  child_fragment_root_n5,
                  sibling_n8,
                  sibling_n9};
  update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();

  Init(update);
  InitFragmentRoot();

  AXNode* root_node = GetRootAsAXNode();

  // Set up other fragment roots
  AXNode* child_fragment_root_n3_node = root_node->children()[1];
  std::unique_ptr<TestFragmentRootDelegate> n3_fragment_root_delegate =
      std::make_unique<TestFragmentRootDelegate>();
  std::unique_ptr<AXFragmentRootWin> n3_fragment_root(InitNodeAsFragmentRoot(
      child_fragment_root_n3_node, n3_fragment_root_delegate.get()));

  AXNode* child_fragment_root_n5_node = root_node->children()[3];
  std::unique_ptr<TestFragmentRootDelegate> n5_fragment_root_delegate =
      std::make_unique<TestFragmentRootDelegate>();
  std::unique_ptr<AXFragmentRootWin> n5_fragment_root(InitNodeAsFragmentRoot(
      child_fragment_root_n5_node, n5_fragment_root_delegate.get()));

  // Test navigation from root fragment
  ComPtr<IRawElementProviderFragmentRoot> root_fragment_root =
      GetFragmentRoot();
  ComPtr<IRawElementProviderFragment> root_fragment;
  root_fragment_root.As(&root_fragment);

  ComPtr<IRawElementProviderFragment> test_fragment;
  EXPECT_HRESULT_SUCCEEDED(
      root_fragment->Navigate(NavigateDirection_Parent, &test_fragment));
  EXPECT_EQ(nullptr, test_fragment.Get());

  EXPECT_HRESULT_SUCCEEDED(
      root_fragment->Navigate(NavigateDirection_NextSibling, &test_fragment));
  EXPECT_EQ(nullptr, test_fragment.Get());

  EXPECT_HRESULT_SUCCEEDED(root_fragment->Navigate(
      NavigateDirection_PreviousSibling, &test_fragment));
  EXPECT_EQ(nullptr, test_fragment.Get());

  EXPECT_HRESULT_SUCCEEDED(
      root_fragment->Navigate(NavigateDirection_FirstChild, &test_fragment));
  ComPtr<IUnknown> root_child_unknown = test_fragment_root_delegate_->child_;
  ComPtr<IRawElementProviderFragment> root_child_fragment;
  root_child_unknown.As(&root_child_fragment);
  EXPECT_EQ(root_child_fragment.Get(), test_fragment.Get());

  EXPECT_HRESULT_SUCCEEDED(
      root_fragment->Navigate(NavigateDirection_LastChild, &test_fragment));
  EXPECT_EQ(root_child_fragment.Get(), test_fragment.Get());

  // Test navigation from first child root (R3)
  ComPtr<IRawElementProviderFragmentRoot> n3_fragment_root_provider;
  n3_fragment_root->GetNativeViewAccessible()->QueryInterface(
      IID_PPV_ARGS(&n3_fragment_root_provider));

  ComPtr<IRawElementProviderFragment> n3_fragment;
  n3_fragment_root_provider.As(&n3_fragment);
  EXPECT_HRESULT_SUCCEEDED(
      n3_fragment->Navigate(NavigateDirection_Parent, &test_fragment));
  EXPECT_EQ(root_child_fragment.Get(), test_fragment.Get());

  AXNode* sibling_n2_node = root_node->children()[0];
  EXPECT_HRESULT_SUCCEEDED(
      n3_fragment->Navigate(NavigateDirection_PreviousSibling, &test_fragment));
  EXPECT_EQ(IRawElementProviderFragmentFromNode(sibling_n2_node).Get(),
            test_fragment.Get());

  AXNode* sibling_n4_node = root_node->children()[2];
  EXPECT_HRESULT_SUCCEEDED(
      n3_fragment->Navigate(NavigateDirection_NextSibling, &test_fragment));
  EXPECT_EQ(IRawElementProviderFragmentFromNode(sibling_n4_node).Get(),
            test_fragment.Get());

  EXPECT_HRESULT_SUCCEEDED(
      n3_fragment->Navigate(NavigateDirection_FirstChild, &test_fragment));
  EXPECT_EQ(
      IRawElementProviderFragmentFromNode(child_fragment_root_n3_node).Get(),
      test_fragment.Get());
  EXPECT_HRESULT_SUCCEEDED(
      n3_fragment->Navigate(NavigateDirection_LastChild, &test_fragment));
  EXPECT_EQ(
      IRawElementProviderFragmentFromNode(child_fragment_root_n3_node).Get(),
      test_fragment.Get());

  // Test navigation from second child root (R5)
  ComPtr<IRawElementProviderFragmentRoot> n5_fragment_root_provider;
  n5_fragment_root->GetNativeViewAccessible()->QueryInterface(
      IID_PPV_ARGS(&n5_fragment_root_provider));

  ComPtr<IRawElementProviderFragment> n5_fragment;
  n5_fragment_root_provider.As(&n5_fragment);
  EXPECT_HRESULT_SUCCEEDED(
      n5_fragment->Navigate(NavigateDirection_Parent, &test_fragment));
  EXPECT_EQ(root_child_fragment.Get(), test_fragment.Get());
  EXPECT_HRESULT_SUCCEEDED(
      n5_fragment->Navigate(NavigateDirection_NextSibling, &test_fragment));
  EXPECT_EQ(nullptr, test_fragment.Get());
  EXPECT_HRESULT_SUCCEEDED(
      n5_fragment->Navigate(NavigateDirection_PreviousSibling, &test_fragment));
  EXPECT_EQ(IRawElementProviderFragmentFromNode(sibling_n4_node).Get(),
            test_fragment.Get());
  EXPECT_HRESULT_SUCCEEDED(
      n5_fragment->Navigate(NavigateDirection_FirstChild, &test_fragment));
  EXPECT_EQ(
      IRawElementProviderFragmentFromNode(child_fragment_root_n5_node).Get(),
      test_fragment.Get());
  EXPECT_HRESULT_SUCCEEDED(
      n5_fragment->Navigate(NavigateDirection_LastChild, &test_fragment));
  EXPECT_EQ(
      IRawElementProviderFragmentFromNode(child_fragment_root_n5_node).Get(),
      test_fragment.Get());
}

TEST_F(AXFragmentRootTest, TestFragmentRootMap) {
  AXNodeData root;
  root.id = 1;
  Init(root);

  // There should be nothing in the map before we create a fragment root.
  // Call GetForAcceleratedWidget() first to ensure that querying for a
  // fragment root doesn't inadvertently create an empty entry in the map
  // (https://crbug.com/1071185).
  EXPECT_EQ(nullptr, AXFragmentRootWin::GetForAcceleratedWidget(
                         gfx::kMockAcceleratedWidget));
  EXPECT_EQ(nullptr, AXFragmentRootWin::GetFragmentRootParentOf(
                         GetRootIAccessible().Get()));

  // After initializing a fragment root, we should be able to retrieve it using
  // its accelerated widget, or as the parent of its child.
  InitFragmentRoot();
  EXPECT_EQ(ax_fragment_root_.get(), AXFragmentRootWin::GetForAcceleratedWidget(
                                         gfx::kMockAcceleratedWidget));
  EXPECT_EQ(ax_fragment_root_.get(), AXFragmentRootWin::GetFragmentRootParentOf(
                                         GetRootIAccessible().Get()));

  // After deleting a fragment root, it should no longer be reachable from the
  // map.
  ax_fragment_root_.reset();
  EXPECT_EQ(nullptr, AXFragmentRootWin::GetForAcceleratedWidget(
                         gfx::kMockAcceleratedWidget));
  EXPECT_EQ(nullptr, AXFragmentRootWin::GetFragmentRootParentOf(
                         GetRootIAccessible().Get()));
}

}  // namespace ui
