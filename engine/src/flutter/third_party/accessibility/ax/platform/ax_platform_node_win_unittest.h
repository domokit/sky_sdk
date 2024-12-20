// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_WIN_UNITTEST_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_WIN_UNITTEST_H_

#include "ax_platform_node_unittest.h"
// clang-format off
#include "third_party/accessibility/base/win/atl.h"  // Must be before UIAutomationCore.h
// clang-format on

#include <UIAutomationCore.h>
#include <wrl.h>

#include <memory>
#include <string>
#include <unordered_set>

#include "third_party/accessibility/ax/platform/ax_fragment_root_delegate_win.h"

struct IAccessible;
struct IAccessibleTableCell;
struct IRawElementProviderFragment;
struct IRawElementProviderFragmentRoot;
struct IRawElementProviderSimple;
struct IUnknown;

namespace base {
namespace win {
class ScopedVariant;
}  // namespace win
}  // namespace base

namespace ui {

class AXFragmentRootWin;
class AXPlatformNode;

class TestFragmentRootDelegate : public AXFragmentRootDelegateWin {
 public:
  TestFragmentRootDelegate();
  virtual ~TestFragmentRootDelegate();
  gfx::NativeViewAccessible GetChildOfAXFragmentRoot() override;
  gfx::NativeViewAccessible GetParentOfAXFragmentRoot() override;
  bool IsAXFragmentRootAControlElement() override;
  gfx::NativeViewAccessible child_ = nullptr;
  gfx::NativeViewAccessible parent_ = nullptr;
  bool is_control_element_ = true;
};

class MockIRawElementProviderSimple
    : public CComObjectRootEx<CComMultiThreadModel>,
      public IRawElementProviderSimple {
 public:
  BEGIN_COM_MAP(MockIRawElementProviderSimple)
  COM_INTERFACE_ENTRY(IRawElementProviderSimple)
  END_COM_MAP()

  MockIRawElementProviderSimple();
  ~MockIRawElementProviderSimple();

  static HRESULT CreateMockIRawElementProviderSimple(
      IRawElementProviderSimple** provider);

  //
  // IRawElementProviderSimple methods.
  //
  IFACEMETHODIMP GetPatternProvider(PATTERNID pattern_id,
                                    IUnknown** result) override;

  IFACEMETHODIMP GetPropertyValue(PROPERTYID property_id,
                                  VARIANT* result) override;

  IFACEMETHODIMP
  get_ProviderOptions(enum ProviderOptions* ret) override;

  IFACEMETHODIMP
  get_HostRawElementProvider(IRawElementProviderSimple** provider) override;
};

class AXPlatformNodeWinTest : public AXPlatformNodeTest {
 public:
  AXPlatformNodeWinTest();
  ~AXPlatformNodeWinTest() override;

  void SetUp() override;

  void TearDown() override;

 protected:
  static const std::u16string kEmbeddedCharacterAsString;

  AXPlatformNode* AXPlatformNodeFromNode(AXNode* node);
  template <typename T>
  Microsoft::WRL::ComPtr<T> QueryInterfaceFromNodeId(AXNode::AXID id);
  template <typename T>
  Microsoft::WRL::ComPtr<T> QueryInterfaceFromNode(AXNode* node);
  Microsoft::WRL::ComPtr<IRawElementProviderSimple>
  GetRootIRawElementProviderSimple();
  Microsoft::WRL::ComPtr<IRawElementProviderSimple>
  GetIRawElementProviderSimpleFromChildIndex(int child_index);
  Microsoft::WRL::ComPtr<IRawElementProviderSimple>
  GetIRawElementProviderSimpleFromTree(const ui::AXTreeID tree_id,
                                       const AXNode::AXID node_id);
  Microsoft::WRL::ComPtr<IRawElementProviderFragment>
  GetRootIRawElementProviderFragment();
  Microsoft::WRL::ComPtr<IRawElementProviderFragment>
  IRawElementProviderFragmentFromNode(AXNode* node);
  Microsoft::WRL::ComPtr<IAccessible> IAccessibleFromNode(AXNode* node);
  Microsoft::WRL::ComPtr<IAccessible> GetRootIAccessible();
  void CheckVariantHasName(const base::win::ScopedVariant& variant,
                           const wchar_t* expected_name);
  void CheckIUnknownHasName(Microsoft::WRL::ComPtr<IUnknown> unknown,
                            const wchar_t* expected_name);
  Microsoft::WRL::ComPtr<IAccessibleTableCell> GetCellInTable();

  void InitFragmentRoot();
  AXFragmentRootWin* InitNodeAsFragmentRoot(AXNode* node,
                                            TestFragmentRootDelegate* delegate);
  Microsoft::WRL::ComPtr<IRawElementProviderFragmentRoot> GetFragmentRoot();

  using PatternSet = std::unordered_set<LONG>;
  PatternSet GetSupportedPatternsFromNodeId(AXNode::AXID id);

  std::unique_ptr<AXFragmentRootWin> ax_fragment_root_;

  std::unique_ptr<TestFragmentRootDelegate> test_fragment_root_delegate_;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_WIN_UNITTEST_H_
