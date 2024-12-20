// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax/platform/ax_platform_node_win_unittest.h"

#include <UIAutomationClient.h>
#include <UIAutomationCoreApi.h>

#include <filesystem>
#include <memory>
#include <utility>

#include "ax/ax_tree.h"
#include "ax/platform/ax_fragment_root_win.h"
#include "ax/platform/ax_platform_node_textrangeprovider_win.h"
#include "base/win/atl.h"
#include "base/win/scoped_bstr.h"
#include "base/win/scoped_safearray.h"
#include "base/win/scoped_variant.h"
#include "flutter/fml/icu_util.h"
#include "third_party/icu/source/common/unicode/putil.h"

using Microsoft::WRL::ComPtr;

namespace ui {

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

#define EXPECT_UIA_GETPROPERTYVALUE_EQ(node, property_id, expected)      \
  {                                                                      \
    base::win::ScopedVariant expectedVariant(expected);                  \
    ASSERT_EQ(VT_BSTR, expectedVariant.type());                          \
    ASSERT_NE(nullptr, expectedVariant.ptr()->bstrVal);                  \
    base::win::ScopedVariant actual;                                     \
    ASSERT_HRESULT_SUCCEEDED(                                            \
        node->GetPropertyValue(property_id, actual.Receive()));          \
    ASSERT_EQ(VT_BSTR, actual.type());                                   \
    ASSERT_NE(nullptr, actual.ptr()->bstrVal);                           \
    EXPECT_STREQ(expectedVariant.ptr()->bstrVal, actual.ptr()->bstrVal); \
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
      EXPECT_UIA_GETPROPERTYVALUE_EQ(element, element_test_property_id,       \
                                     expected_property_values[i].c_str());    \
    }                                                                         \
    ASSERT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(array));                 \
  }

#define EXPECT_UIA_SAFEARRAY_EQ(safearray, expected_property_values)   \
  {                                                                    \
    using T = typename decltype(expected_property_values)::value_type; \
    EXPECT_EQ(sizeof(T), ::SafeArrayGetElemsize(safearray));           \
    EXPECT_EQ(1u, SafeArrayGetDim(safearray));                         \
    LONG array_lower_bound;                                            \
    EXPECT_HRESULT_SUCCEEDED(                                          \
        SafeArrayGetLBound(safearray, 1, &array_lower_bound));         \
    LONG array_upper_bound;                                            \
    EXPECT_HRESULT_SUCCEEDED(                                          \
        SafeArrayGetUBound(safearray, 1, &array_upper_bound));         \
    const size_t count = array_upper_bound - array_lower_bound + 1;    \
    EXPECT_EQ(expected_property_values.size(), count);                 \
    if (sizeof(T) == ::SafeArrayGetElemsize(safearray) &&              \
        count == expected_property_values.size()) {                    \
      T* array_data;                                                   \
      EXPECT_HRESULT_SUCCEEDED(::SafeArrayAccessData(                  \
          safearray, reinterpret_cast<void**>(&array_data)));          \
      for (size_t i = 0; i < count; ++i) {                             \
        EXPECT_EQ(array_data[i], expected_property_values[i]);         \
      }                                                                \
      EXPECT_HRESULT_SUCCEEDED(::SafeArrayUnaccessData(safearray));    \
    }                                                                  \
  }

#define EXPECT_UIA_TEXTATTRIBUTE_EQ(provider, attribute, variant)          \
  {                                                                        \
    base::win::ScopedVariant scoped_variant;                               \
    EXPECT_HRESULT_SUCCEEDED(                                              \
        provider->GetAttributeValue(attribute, scoped_variant.Receive())); \
    EXPECT_EQ(0, scoped_variant.Compare(variant));                         \
  }

#define EXPECT_UIA_TEXTATTRIBUTE_MIXED(provider, attribute)                \
  {                                                                        \
    ComPtr<IUnknown> expected_mixed;                                       \
    EXPECT_HRESULT_SUCCEEDED(                                              \
        ::UiaGetReservedMixedAttributeValue(&expected_mixed));             \
    base::win::ScopedVariant scoped_variant;                               \
    EXPECT_HRESULT_SUCCEEDED(                                              \
        provider->GetAttributeValue(attribute, scoped_variant.Receive())); \
    EXPECT_EQ(VT_UNKNOWN, scoped_variant.type());                          \
    EXPECT_EQ(expected_mixed.Get(), V_UNKNOWN(scoped_variant.ptr()));      \
  }

#define EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(provider, attribute)           \
  {                                                                          \
    ComPtr<IUnknown> expected_notsupported;                                  \
    EXPECT_HRESULT_SUCCEEDED(                                                \
        ::UiaGetReservedNotSupportedValue(&expected_notsupported));          \
    base::win::ScopedVariant scoped_variant;                                 \
    EXPECT_HRESULT_SUCCEEDED(                                                \
        provider->GetAttributeValue(attribute, scoped_variant.Receive()));   \
    EXPECT_EQ(VT_UNKNOWN, scoped_variant.type());                            \
    EXPECT_EQ(expected_notsupported.Get(), V_UNKNOWN(scoped_variant.ptr())); \
  }

#define EXPECT_UIA_TEXTRANGE_EQ(provider, expected_content) \
  {                                                         \
    base::win::ScopedBstr provider_content;                 \
    EXPECT_HRESULT_SUCCEEDED(                               \
        provider->GetText(-1, provider_content.Receive())); \
    EXPECT_STREQ(expected_content, provider_content.Get()); \
  }

#define EXPECT_UIA_FIND_TEXT(text_range_provider, search_term, ignore_case,    \
                             owner)                                            \
  {                                                                            \
    base::win::ScopedBstr find_string(search_term);                            \
    ComPtr<ITextRangeProvider> text_range_provider_found;                      \
    EXPECT_HRESULT_SUCCEEDED(text_range_provider->FindText(                    \
        find_string.Get(), false, ignore_case, &text_range_provider_found));   \
    if (text_range_provider_found == nullptr) {                                \
      EXPECT_TRUE(false);                                                      \
    } else {                                                                   \
      SetOwner(owner, text_range_provider_found.Get());                        \
      base::win::ScopedBstr found_content;                                     \
      EXPECT_HRESULT_SUCCEEDED(                                                \
          text_range_provider_found->GetText(-1, found_content.Receive()));    \
      if (ignore_case)                                                         \
        EXPECT_TRUE(StringCompareICU(found_content.Get(), find_string.Get())); \
      else                                                                     \
        EXPECT_EQ(0, wcscmp(found_content.Get(), find_string.Get()));          \
    }                                                                          \
  }

#define EXPECT_UIA_FIND_TEXT_NO_MATCH(text_range_provider, search_term,      \
                                      ignore_case, owner)                    \
  {                                                                          \
    base::win::ScopedBstr find_string(search_term);                          \
    ComPtr<ITextRangeProvider> text_range_provider_found;                    \
    EXPECT_HRESULT_SUCCEEDED(text_range_provider->FindText(                  \
        find_string.Get(), false, ignore_case, &text_range_provider_found)); \
    EXPECT_EQ(nullptr, text_range_provider_found);                           \
  }

#define EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider, endpoint, unit,  \
                                         count, expected_text, expected_count) \
  {                                                                            \
    int result_count;                                                          \
    EXPECT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(          \
        endpoint, unit, count, &result_count));                                \
    EXPECT_EQ(expected_count, result_count);                                   \
    EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, expected_text);               \
  }

#define EXPECT_UIA_MOVE(text_range_provider, unit, count, expected_text, \
                        expected_count)                                  \
  {                                                                      \
    int result_count;                                                    \
    EXPECT_HRESULT_SUCCEEDED(                                            \
        text_range_provider->Move(unit, count, &result_count));          \
    EXPECT_EQ(expected_count, result_count);                             \
    EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, expected_text);         \
  }

#define EXPECT_ENCLOSING_ELEMENT(ax_node_given, ax_node_expected)            \
  {                                                                          \
    ComPtr<ITextRangeProvider> text_range_provider;                          \
    GetTextRangeProviderFromTextNode(text_range_provider, ax_node_given);    \
    ComPtr<IRawElementProviderSimple> enclosing_element;                     \
    ASSERT_HRESULT_SUCCEEDED(                                                \
        text_range_provider->GetEnclosingElement(&enclosing_element));       \
    ComPtr<IRawElementProviderSimple> expected_text_provider =               \
        QueryInterfaceFromNode<IRawElementProviderSimple>(ax_node_expected); \
    EXPECT_EQ(expected_text_provider.Get(), enclosing_element.Get());        \
  }

#define DCHECK_EQ(a, b) BASE_DCHECK((a) == (b))

static bool StringCompareICU(BSTR left, BSTR right) {
  size_t start, length;
  if (!StringSearch(reinterpret_cast<char16_t*>(left),
                    reinterpret_cast<char16_t*>(right), &start, &length, true,
                    false)) {
    return false;
  }
  return start == 0 && length == wcslen(left);
}

static AXNodePosition::AXPositionInstance CreateTextPosition(
    const AXNode& anchor,
    int text_offset,
    ax::mojom::TextAffinity affinity) {
  return AXNodePosition::CreateTextPosition(anchor.tree()->GetAXTreeID(),
                                            anchor.id(), text_offset, affinity);
}

class AXPlatformNodeTextRangeProviderTest : public ui::AXPlatformNodeWinTest {
 public:
  const AXNodePosition::AXPositionInstance& GetStart(
      const AXPlatformNodeTextRangeProviderWin* text_range) {
    return text_range->start();
  }

  const AXNodePosition::AXPositionInstance& GetEnd(
      const AXPlatformNodeTextRangeProviderWin* text_range) {
    return text_range->end();
  }

  ui::AXPlatformNodeWin* GetOwner(
      const AXPlatformNodeTextRangeProviderWin* text_range) {
    return text_range->GetOwner();
  }

  void CopyOwnerToClone(ITextRangeProvider* source_range,
                        ITextRangeProvider* destination_range) {
    ComPtr<ITextRangeProvider> source_provider = source_range;
    ComPtr<ITextRangeProvider> destination_provider = destination_range;

    ComPtr<AXPlatformNodeTextRangeProviderWin> source_provider_internal;
    ComPtr<AXPlatformNodeTextRangeProviderWin> destination_provider_internal;

    source_provider->QueryInterface(IID_PPV_ARGS(&source_provider_internal));
    destination_provider->QueryInterface(
        IID_PPV_ARGS(&destination_provider_internal));
    destination_provider_internal->SetOwnerForTesting(
        source_provider_internal->GetOwner());
  }

  void SetOwner(AXPlatformNodeWin* owner,
                ITextRangeProvider* destination_range) {
    ComPtr<AXPlatformNodeTextRangeProviderWin> destination_provider_internal;
    auto as =
        static_cast<AXPlatformNodeTextRangeProviderWin*>(destination_range);
    destination_range->QueryInterface(
        IID_PPV_ARGS(&destination_provider_internal));
    destination_provider_internal->SetOwnerForTesting(owner);
  }

  void NormalizeTextRange(AXPlatformNodeTextRangeProviderWin* text_range,
                          AXNodePosition::AXPositionInstance& start,
                          AXNodePosition::AXPositionInstance& end) {
    DCHECK_EQ(*GetStart(text_range), *start);
    DCHECK_EQ(*GetEnd(text_range), *end);
    text_range->NormalizeTextRange(start, end);
  }

  void GetTextRangeProviderFromTextNode(
      ComPtr<ITextRangeProvider>& text_range_provider,
      ui::AXNode* text_node) {
    ComPtr<IRawElementProviderSimple> provider_simple =
        QueryInterfaceFromNode<IRawElementProviderSimple>(text_node);
    ASSERT_NE(nullptr, provider_simple.Get());

    ComPtr<ITextProvider> text_provider;
    EXPECT_HRESULT_SUCCEEDED(
        provider_simple->GetPatternProvider(UIA_TextPatternId, &text_provider));
    ASSERT_NE(nullptr, text_provider.Get());

    EXPECT_HRESULT_SUCCEEDED(
        text_provider->get_DocumentRange(&text_range_provider));
    ASSERT_NE(nullptr, text_range_provider.Get());

    ComPtr<AXPlatformNodeTextRangeProviderWin> text_range_provider_interal;
    EXPECT_HRESULT_SUCCEEDED(text_range_provider->QueryInterface(
        IID_PPV_ARGS(&text_range_provider_interal)));
    AXPlatformNode* ax_platform_node = AXPlatformNodeFromNode(text_node);
    ASSERT_NE(ax_platform_node, nullptr);
    text_range_provider_interal->SetOwnerForTesting(
        static_cast<AXPlatformNodeWin*>(ax_platform_node));
  }

  void CreateTextRangeProviderWin(
      ComPtr<AXPlatformNodeTextRangeProviderWin>& text_range_provider_win,
      AXPlatformNodeWin* owner,
      const AXNode* start_anchor,
      int start_offset,
      ax::mojom::TextAffinity start_affinity,
      const AXNode* end_anchor,
      int end_offset,
      ax::mojom::TextAffinity end_affinity) {
    AXNodePosition::AXPositionInstance range_start =
        CreateTextPosition(*start_anchor, start_offset, start_affinity);
    AXNodePosition::AXPositionInstance range_end =
        CreateTextPosition(*end_anchor, end_offset, end_affinity);

    ComPtr<ITextRangeProvider> text_range_provider =
        AXPlatformNodeTextRangeProviderWin::CreateTextRangeProviderForTesting(
            owner, std::move(range_start), std::move(range_end));

    text_range_provider->QueryInterface(IID_PPV_ARGS(&text_range_provider_win));
  }

  void ComputeWordBoundariesOffsets(const std::string& text,
                                    std::vector<int>& word_start_offsets,
                                    std::vector<int>& word_end_offsets) {
    char previous_char = ' ';
    word_start_offsets = std::vector<int>();
    for (size_t i = 0; i < text.size(); ++i) {
      if (previous_char == ' ' && text[i] != ' ')
        word_start_offsets.push_back(i);
      previous_char = text[i];
    }

    previous_char = ' ';
    word_end_offsets = std::vector<int>();
    for (size_t i = text.size(); i > 0; --i) {
      if (previous_char == ' ' && text[i - 1] != ' ')
        word_end_offsets.push_back(i);
      previous_char = text[i - 1];
    }
    std::reverse(word_end_offsets.begin(), word_end_offsets.end());
  }

  AXTreeUpdate BuildTextDocument(
      const std::vector<std::string>& text_nodes_content,
      bool build_word_boundaries_offsets = false,
      bool place_text_on_one_line = false) {
    int current_id = 0;
    AXNodeData root_data;
    root_data.id = ++current_id;
    root_data.role = ax::mojom::Role::kRootWebArea;

    AXTreeUpdate update;
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    update.has_tree_data = true;

    for (const std::string& text_content : text_nodes_content) {
      AXNodeData static_text_data;
      static_text_data.id = ++current_id;
      static_text_data.role = ax::mojom::Role::kStaticText;
      static_text_data.SetName(text_content);
      root_data.child_ids.push_back(static_text_data.id);

      AXNodeData inline_box_data;
      inline_box_data.id = ++current_id;
      inline_box_data.role = ax::mojom::Role::kInlineTextBox;
      inline_box_data.SetName(text_content);
      static_text_data.child_ids = {inline_box_data.id};

      if (build_word_boundaries_offsets) {
        std::vector<int> word_end_offsets;
        std::vector<int> word_start_offsets;
        ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                                     word_end_offsets);
        inline_box_data.AddIntListAttribute(
            ax::mojom::IntListAttribute::kWordStarts, word_start_offsets);
        inline_box_data.AddIntListAttribute(
            ax::mojom::IntListAttribute::kWordEnds, word_end_offsets);
      }

      if (place_text_on_one_line && !update.nodes.empty()) {
        AXNodeData* previous_inline_box_data = &update.nodes.back();
        static_text_data.AddIntAttribute(
            ax::mojom::IntAttribute::kPreviousOnLineId,
            previous_inline_box_data->id);
        inline_box_data.AddIntAttribute(
            ax::mojom::IntAttribute::kPreviousOnLineId,
            previous_inline_box_data->id);
        previous_inline_box_data->AddIntAttribute(
            ax::mojom::IntAttribute::kNextOnLineId, inline_box_data.id);
      }

      update.nodes.push_back(static_text_data);
      update.nodes.push_back(inline_box_data);
    }

    update.nodes.insert(update.nodes.begin(), root_data);
    update.root_id = root_data.id;
    return update;
  }

  ui::AXTreeUpdate BuildAXTreeForBoundingRectangles() {
    // AXTree content:
    // <button>Button</button><input type="checkbox">Line 1<br>Line 2
    ui::AXNodeData root;
    ui::AXNodeData button;
    ui::AXNodeData check_box;
    ui::AXNodeData text_field;
    ui::AXNodeData static_text1;
    ui::AXNodeData line_break;
    ui::AXNodeData static_text2;
    ui::AXNodeData inline_box1;
    ui::AXNodeData inline_box2;
    ui::AXNodeData inline_box_line_break;

    const int ROOT_ID = 1;
    const int BUTTON_ID = 2;
    const int CHECK_BOX_ID = 3;
    const int TEXT_FIELD_ID = 4;
    const int STATIC_TEXT1_ID = 5;
    const int INLINE_BOX1_ID = 6;
    const int LINE_BREAK_ID = 7;
    const int INLINE_BOX_LINE_BREAK_ID = 8;
    const int STATIC_TEXT2_ID = 9;
    const int INLINE_BOX2_ID = 10;

    root.id = ROOT_ID;
    button.id = BUTTON_ID;
    check_box.id = CHECK_BOX_ID;
    text_field.id = TEXT_FIELD_ID;
    static_text1.id = STATIC_TEXT1_ID;
    inline_box1.id = INLINE_BOX1_ID;
    line_break.id = LINE_BREAK_ID;
    inline_box_line_break.id = INLINE_BOX_LINE_BREAK_ID;
    static_text2.id = STATIC_TEXT2_ID;
    inline_box2.id = INLINE_BOX2_ID;

    std::string LINE_1_TEXT = "Line 1";
    std::string LINE_2_TEXT = "Line 2";
    std::string LINE_BREAK_TEXT = "\n";
    std::string ALL_TEXT = LINE_1_TEXT + LINE_BREAK_TEXT + LINE_2_TEXT;
    std::string BUTTON_TEXT = "Button";
    std::string CHECKBOX_TEXT = "Check box";

    root.role = ax::mojom::Role::kRootWebArea;

    button.role = ax::mojom::Role::kButton;
    button.SetHasPopup(ax::mojom::HasPopup::kMenu);
    button.SetName(BUTTON_TEXT);
    button.SetValue(BUTTON_TEXT);
    button.relative_bounds.bounds = gfx::RectF(20, 20, 200, 30);
    button.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                           check_box.id);
    root.child_ids.push_back(button.id);

    check_box.role = ax::mojom::Role::kCheckBox;
    check_box.SetCheckedState(ax::mojom::CheckedState::kTrue);
    check_box.SetName(CHECKBOX_TEXT);
    check_box.relative_bounds.bounds = gfx::RectF(20, 50, 200, 30);
    check_box.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              button.id);
    root.child_ids.push_back(check_box.id);

    text_field.role = ax::mojom::Role::kTextField;
    text_field.AddState(ax::mojom::State::kEditable);
    text_field.AddStringAttribute(ax::mojom::StringAttribute::kHtmlTag,
                                  "input");
    text_field.AddStringAttribute(ax::mojom::StringAttribute::kInputType,
                                  "text");
    text_field.SetValue(ALL_TEXT);
    text_field.AddIntListAttribute(
        ax::mojom::IntListAttribute::kCachedLineStarts,
        std::vector<int32_t>{0, 7});
    text_field.child_ids.push_back(static_text1.id);
    text_field.child_ids.push_back(line_break.id);
    text_field.child_ids.push_back(static_text2.id);
    root.child_ids.push_back(text_field.id);

    static_text1.role = ax::mojom::Role::kStaticText;
    static_text1.AddState(ax::mojom::State::kEditable);
    static_text1.SetName(LINE_1_TEXT);
    static_text1.child_ids.push_back(inline_box1.id);

    inline_box1.role = ax::mojom::Role::kInlineTextBox;
    inline_box1.AddState(ax::mojom::State::kEditable);
    inline_box1.SetName(LINE_1_TEXT);
    inline_box1.relative_bounds.bounds = gfx::RectF(220, 20, 100, 30);
    std::vector<int32_t> character_offsets1;
    // The width of each character is 5px.
    character_offsets1.push_back(225);  // "L" {220, 20, 5x30}
    character_offsets1.push_back(230);  // "i" {225, 20, 5x30}
    character_offsets1.push_back(235);  // "n" {230, 20, 5x30}
    character_offsets1.push_back(240);  // "e" {235, 20, 5x30}
    character_offsets1.push_back(245);  // " " {240, 20, 5x30}
    character_offsets1.push_back(250);  // "1" {245, 20, 5x30}
    inline_box1.AddIntListAttribute(
        ax::mojom::IntListAttribute::kCharacterOffsets, character_offsets1);
    inline_box1.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                    std::vector<int32_t>{0, 5});
    inline_box1.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                    std::vector<int32_t>{4, 6});
    inline_box1.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                line_break.id);

    line_break.role = ax::mojom::Role::kLineBreak;
    line_break.AddState(ax::mojom::State::kEditable);
    line_break.SetName(LINE_BREAK_TEXT);
    line_break.relative_bounds.bounds = gfx::RectF(250, 20, 0, 30);
    line_break.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                               inline_box1.id);
    line_break.child_ids.push_back(inline_box_line_break.id);

    inline_box_line_break.role = ax::mojom::Role::kInlineTextBox;
    inline_box_line_break.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
    inline_box_line_break.SetName(LINE_BREAK_TEXT);
    inline_box_line_break.relative_bounds.bounds = gfx::RectF(250, 20, 0, 30);
    inline_box_line_break.AddIntListAttribute(
        ax::mojom::IntListAttribute::kCharacterOffsets, {0});
    inline_box_line_break.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordStarts, std::vector<int32_t>{0});
    inline_box_line_break.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordEnds, std::vector<int32_t>{0});

    static_text2.role = ax::mojom::Role::kStaticText;
    static_text2.AddState(ax::mojom::State::kEditable);
    static_text2.SetName(LINE_2_TEXT);
    static_text2.child_ids.push_back(inline_box2.id);

    inline_box2.role = ax::mojom::Role::kInlineTextBox;
    inline_box2.AddState(ax::mojom::State::kEditable);
    inline_box2.SetName(LINE_2_TEXT);
    inline_box2.relative_bounds.bounds = gfx::RectF(220, 50, 100, 30);
    std::vector<int32_t> character_offsets2;
    // The width of each character is 7 px.
    character_offsets2.push_back(227);  // "L" {220, 50, 7x30}
    character_offsets2.push_back(234);  // "i" {227, 50, 7x30}
    character_offsets2.push_back(241);  // "n" {234, 50, 7x30}
    character_offsets2.push_back(248);  // "e" {241, 50, 7x30}
    character_offsets2.push_back(255);  // " " {248, 50, 7x30}
    character_offsets2.push_back(262);  // "2" {255, 50, 7x30}
    inline_box2.AddIntListAttribute(
        ax::mojom::IntListAttribute::kCharacterOffsets, character_offsets2);
    inline_box2.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                    std::vector<int32_t>{0, 5});
    inline_box2.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                    std::vector<int32_t>{4, 6});

    AXTreeUpdate update;
    update.has_tree_data = true;
    update.root_id = ROOT_ID;
    update.nodes = {
        root,         button,      check_box,  text_field,
        static_text1, inline_box1, line_break, inline_box_line_break,
        static_text2, inline_box2};
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    return update;
  }

  const std::wstring tree_for_move_full_text =
      L"First line of text\nStandalone line\n"
      L"bold text\nParagraph 1\nParagraph 2";

  ui::AXTreeUpdate BuildAXTreeForMove() {
    ui::AXNodeData group1_data;
    group1_data.id = 2;
    group1_data.role = ax::mojom::Role::kGenericContainer;
    group1_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    ui::AXNodeData text_data;
    text_data.id = 3;
    text_data.role = ax::mojom::Role::kStaticText;
    std::string text_content = "First line of text";
    text_data.SetName(text_content);
    std::vector<int> word_end_offsets;
    std::vector<int> word_start_offsets;
    ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                                 word_end_offsets);
    text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  word_start_offsets);
    text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  word_end_offsets);
    group1_data.child_ids = {text_data.id};

    ui::AXNodeData group2_data;
    group2_data.id = 4;
    group2_data.role = ax::mojom::Role::kGenericContainer;

    ui::AXNodeData line_break1_data;
    line_break1_data.id = 5;
    line_break1_data.role = ax::mojom::Role::kLineBreak;
    line_break1_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
    line_break1_data.SetName("\n");

    ui::AXNodeData standalone_text_data;
    standalone_text_data.id = 6;
    standalone_text_data.role = ax::mojom::Role::kStaticText;
    text_content = "Standalone line";
    standalone_text_data.SetName(text_content);
    ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                                 word_end_offsets);
    standalone_text_data.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordStarts, word_start_offsets);
    standalone_text_data.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordEnds, word_end_offsets);

    ui::AXNodeData line_break2_data;
    line_break2_data.id = 7;
    line_break2_data.role = ax::mojom::Role::kLineBreak;
    line_break2_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
    line_break2_data.SetName("\n");

    group2_data.child_ids = {line_break1_data.id, standalone_text_data.id,
                             line_break2_data.id};
    standalone_text_data.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                         line_break2_data.id);
    line_break2_data.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                     standalone_text_data.id);

    ui::AXNodeData bold_text_data;
    bold_text_data.id = 8;
    bold_text_data.role = ax::mojom::Role::kStaticText;
    bold_text_data.AddIntAttribute(
        ax::mojom::IntAttribute::kTextStyle,
        static_cast<int32_t>(ax::mojom::TextStyle::kBold));
    text_content = "bold text";
    bold_text_data.SetName(text_content);
    ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                                 word_end_offsets);
    bold_text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                       word_start_offsets);
    bold_text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                       word_end_offsets);

    ui::AXNodeData paragraph1_data;
    paragraph1_data.id = 9;
    paragraph1_data.role = ax::mojom::Role::kParagraph;
    paragraph1_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    ui::AXNodeData paragraph1_text_data;
    paragraph1_text_data.id = 10;
    paragraph1_text_data.role = ax::mojom::Role::kStaticText;
    text_content = "Paragraph 1";
    paragraph1_text_data.SetName(text_content);
    ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                                 word_end_offsets);
    paragraph1_text_data.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordStarts, word_start_offsets);
    paragraph1_text_data.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordEnds, word_end_offsets);
    paragraph1_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    ui::AXNodeData ignored_text_data;
    ignored_text_data.id = 11;
    ignored_text_data.role = ax::mojom::Role::kStaticText;
    ignored_text_data.AddState(ax::mojom::State::kIgnored);
    text_content = "ignored text";
    ignored_text_data.SetName(text_content);

    paragraph1_data.child_ids = {paragraph1_text_data.id, ignored_text_data.id};

    ui::AXNodeData paragraph2_data;
    paragraph2_data.id = 12;
    paragraph2_data.role = ax::mojom::Role::kParagraph;
    paragraph2_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    ui::AXNodeData paragraph2_text_data;
    paragraph2_text_data.id = 13;
    paragraph2_text_data.role = ax::mojom::Role::kStaticText;
    text_content = "Paragraph 2";
    paragraph2_text_data.SetName(text_content);
    ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                                 word_end_offsets);
    paragraph2_text_data.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordStarts, word_start_offsets);
    paragraph2_text_data.AddIntListAttribute(
        ax::mojom::IntListAttribute::kWordEnds, word_end_offsets);
    paragraph1_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
    paragraph2_data.child_ids = {paragraph2_text_data.id};

    ui::AXNodeData root_data;
    root_data.id = 1;
    root_data.role = ax::mojom::Role::kRootWebArea;
    root_data.child_ids = {group1_data.id, group2_data.id, bold_text_data.id,
                           paragraph1_data.id, paragraph2_data.id};

    ui::AXTreeUpdate update;
    update.has_tree_data = true;
    update.root_id = root_data.id;
    update.nodes = {root_data,           group1_data,
                    text_data,           group2_data,
                    line_break1_data,    standalone_text_data,
                    line_break2_data,    bold_text_data,
                    paragraph1_data,     paragraph1_text_data,
                    ignored_text_data,   paragraph2_data,
                    paragraph2_text_data};
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    return update;
  }

  AXTreeUpdate BuildAXTreeForMoveByFormat() {
    //                    1
    //                    |
    //    -------------------------------------
    //    |       |       |    |    |    |    |
    //    2       4       8   10   12   14   16
    //    |       |       |    |    |    |    |
    //    |   ---------   |    |    |    |    |
    //    |   |   |   |   |    |    |    |    |
    //    3   5   6   7   9   11   13   15   17

    AXNodeData group1_data;
    group1_data.id = 2;
    group1_data.role = ax::mojom::Role::kGenericContainer;
    group1_data.AddStringAttribute(ax::mojom::StringAttribute::kFontFamily,
                                   "test font");
    group1_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    AXNodeData text_data;
    text_data.id = 3;
    text_data.role = ax::mojom::Role::kStaticText;
    text_data.SetName("Text with formatting");
    group1_data.child_ids = {text_data.id};

    AXNodeData group2_data;
    group2_data.id = 4;
    group2_data.role = ax::mojom::Role::kGenericContainer;
    group2_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    AXNodeData line_break1_data;
    line_break1_data.id = 5;
    line_break1_data.role = ax::mojom::Role::kLineBreak;
    line_break1_data.SetName("\n");

    AXNodeData standalone_text_data;
    standalone_text_data.id = 6;
    standalone_text_data.role = ax::mojom::Role::kStaticText;
    standalone_text_data.SetName("Standalone line with no formatting");

    AXNodeData line_break2_data;
    line_break2_data.id = 7;
    line_break2_data.role = ax::mojom::Role::kLineBreak;
    line_break2_data.SetName("\n");

    group2_data.child_ids = {line_break1_data.id, standalone_text_data.id,
                             line_break2_data.id};

    AXNodeData group3_data;
    group3_data.id = 8;
    group3_data.role = ax::mojom::Role::kGenericContainer;
    group3_data.AddIntAttribute(
        ax::mojom::IntAttribute::kTextStyle,
        static_cast<int32_t>(ax::mojom::TextStyle::kBold));
    group3_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    AXNodeData bold_text_data;
    bold_text_data.id = 9;
    bold_text_data.role = ax::mojom::Role::kStaticText;
    bold_text_data.SetName("bold text");
    group3_data.child_ids = {bold_text_data.id};

    AXNodeData paragraph1_data;
    paragraph1_data.id = 10;
    paragraph1_data.role = ax::mojom::Role::kParagraph;
    paragraph1_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 100);
    paragraph1_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    AXNodeData paragraph1_text_data;
    paragraph1_text_data.id = 11;
    paragraph1_text_data.role = ax::mojom::Role::kStaticText;
    paragraph1_text_data.SetName("Paragraph 1");
    paragraph1_data.child_ids = {paragraph1_text_data.id};

    AXNodeData paragraph2_data;
    paragraph2_data.id = 12;
    paragraph2_data.role = ax::mojom::Role::kParagraph;
    paragraph2_data.AddFloatAttribute(ax::mojom::FloatAttribute::kFontSize,
                                      1.0f);
    paragraph2_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    AXNodeData paragraph2_text_data;
    paragraph2_text_data.id = 13;
    paragraph2_text_data.role = ax::mojom::Role::kStaticText;
    paragraph2_text_data.SetName("Paragraph 2");
    paragraph2_data.child_ids = {paragraph2_text_data.id};

    AXNodeData paragraph3_data;
    paragraph3_data.id = 14;
    paragraph3_data.role = ax::mojom::Role::kParagraph;
    paragraph3_data.AddFloatAttribute(ax::mojom::FloatAttribute::kFontSize,
                                      1.0f);
    paragraph3_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    AXNodeData paragraph3_text_data;
    paragraph3_text_data.id = 15;
    paragraph3_text_data.role = ax::mojom::Role::kStaticText;
    paragraph3_text_data.SetName("Paragraph 3");
    paragraph3_data.child_ids = {paragraph3_text_data.id};

    AXNodeData paragraph4_data;
    paragraph4_data.id = 16;
    paragraph4_data.role = ax::mojom::Role::kParagraph;
    paragraph4_data.AddFloatAttribute(ax::mojom::FloatAttribute::kFontSize,
                                      2.0f);
    paragraph4_data.AddBoolAttribute(
        ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

    AXNodeData paragraph4_text_data;
    paragraph4_text_data.id = 17;
    paragraph4_text_data.role = ax::mojom::Role::kStaticText;
    paragraph4_text_data.SetName("Paragraph 4");
    paragraph4_data.child_ids = {paragraph4_text_data.id};

    AXNodeData root_data;
    root_data.id = 1;
    root_data.role = ax::mojom::Role::kRootWebArea;
    root_data.child_ids = {group1_data.id,     group2_data.id,
                           group3_data.id,     paragraph1_data.id,
                           paragraph2_data.id, paragraph3_data.id,
                           paragraph4_data.id};

    AXTreeUpdate update;
    update.has_tree_data = true;
    update.root_id = root_data.id;
    update.nodes = {root_data,
                    group1_data,
                    text_data,
                    group2_data,
                    line_break1_data,
                    standalone_text_data,
                    line_break2_data,
                    group3_data,
                    bold_text_data,
                    paragraph1_data,
                    paragraph1_text_data,
                    paragraph2_data,
                    paragraph2_text_data,
                    paragraph3_data,
                    paragraph3_text_data,
                    paragraph4_data,
                    paragraph4_text_data};
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    return update;
  }

  void ExpectPositionsEqual(const AXNodePosition::AXPositionInstance& a,
                            const AXNodePosition::AXPositionInstance& b) {
    EXPECT_EQ(*a, *b);
    EXPECT_EQ(a->anchor_id(), b->anchor_id());
    EXPECT_EQ(a->text_offset(), b->text_offset());
  }
};

class MockAXPlatformNodeTextRangeProviderWin
    : public CComObjectRootEx<CComMultiThreadModel>,
      public ITextRangeProvider {
 public:
  BEGIN_COM_MAP(MockAXPlatformNodeTextRangeProviderWin)
  COM_INTERFACE_ENTRY(ITextRangeProvider)
  END_COM_MAP()

  MockAXPlatformNodeTextRangeProviderWin() {}
  ~MockAXPlatformNodeTextRangeProviderWin() {}

  static HRESULT CreateMockTextRangeProvider(ITextRangeProvider** provider) {
    CComObject<MockAXPlatformNodeTextRangeProviderWin>* text_range_provider =
        nullptr;
    HRESULT hr =
        CComObject<MockAXPlatformNodeTextRangeProviderWin>::CreateInstance(
            &text_range_provider);
    if (SUCCEEDED(hr)) {
      *provider = text_range_provider;
    }

    return hr;
  }

  //
  // ITextRangeProvider methods.
  //
  IFACEMETHODIMP Clone(ITextRangeProvider** clone) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP Compare(ITextRangeProvider* other, BOOL* result) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP CompareEndpoints(TextPatternRangeEndpoint this_endpoint,
                                  ITextRangeProvider* other,
                                  TextPatternRangeEndpoint other_endpoint,
                                  int* result) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP ExpandToEnclosingUnit(TextUnit unit) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP FindAttribute(TEXTATTRIBUTEID attribute_id,
                               VARIANT val,
                               BOOL backward,
                               ITextRangeProvider** result) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP FindText(BSTR string,
                          BOOL backwards,
                          BOOL ignore_case,
                          ITextRangeProvider** result) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP GetAttributeValue(TEXTATTRIBUTEID attribute_id,
                                   VARIANT* value) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP GetBoundingRectangles(SAFEARRAY** rectangles) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP GetEnclosingElement(
      IRawElementProviderSimple** element) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP GetText(int max_count, BSTR* text) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP Move(TextUnit unit, int count, int* units_moved) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP MoveEndpointByUnit(TextPatternRangeEndpoint endpoint,
                                    TextUnit unit,
                                    int count,
                                    int* units_moved) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP MoveEndpointByRange(
      TextPatternRangeEndpoint this_endpoint,
      ITextRangeProvider* other,
      TextPatternRangeEndpoint other_endpoint) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP Select() override { return E_NOTIMPL; }

  IFACEMETHODIMP AddToSelection() override { return E_NOTIMPL; }

  IFACEMETHODIMP RemoveFromSelection() override { return E_NOTIMPL; }

  IFACEMETHODIMP ScrollIntoView(BOOL align_to_top) override {
    return E_NOTIMPL;
  }

  IFACEMETHODIMP GetChildren(SAFEARRAY** children) override {
    return E_NOTIMPL;
  }
};

TEST_F(AXPlatformNodeTextRangeProviderTest, TestITextRangeProviderClone) {
  Init(BuildTextDocument({"some text"}));

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetRootAsAXNode()->children()[0]);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text");

  ComPtr<ITextRangeProvider> text_range_provider_clone;
  text_range_provider->Clone(&text_range_provider_clone);
  CopyOwnerToClone(text_range_provider.Get(), text_range_provider_clone.Get());
  ComPtr<AXPlatformNodeTextRangeProviderWin> original_range;
  ComPtr<AXPlatformNodeTextRangeProviderWin> clone_range;

  text_range_provider->QueryInterface(IID_PPV_ARGS(&original_range));
  text_range_provider_clone->QueryInterface(IID_PPV_ARGS(&clone_range));

  EXPECT_EQ(*GetStart(original_range.Get()), *GetStart(clone_range.Get()));
  EXPECT_EQ(*GetEnd(original_range.Get()), *GetEnd(clone_range.Get()));
  EXPECT_EQ(GetOwner(original_range.Get()), GetOwner(clone_range.Get()));

  // Clear original text range provider.
  text_range_provider.Reset();
  EXPECT_EQ(nullptr, text_range_provider.Get());

  // Ensure the clone still works correctly.
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider_clone, L"some text");
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderCompareEndpoints) {
  Init(BuildTextDocument({"some text", "more text"},
                         false /* build_word_boundaries_offsets */,
                         true /* place_text_on_one_line */));

  AXNode* root_node = GetRootAsAXNode();

  // Get the textRangeProvider for the document,
  // which contains text "some textmore text".
  ComPtr<ITextRangeProvider> document_text_range_provider;
  GetTextRangeProviderFromTextNode(document_text_range_provider, root_node);

  // Get the textRangeProvider for "some text".
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   root_node->children()[0]);

  // Get the textRangeProvider for "more text".
  ComPtr<ITextRangeProvider> more_text_range_provider;
  GetTextRangeProviderFromTextNode(more_text_range_provider,
                                   root_node->children()[1]);

  // Compare the endpoints of the document which contains "some textmore text".
  int result;
  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_Start, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_Start, &result));
  EXPECT_EQ(0, result);

  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_End, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_End, &result));
  EXPECT_EQ(0, result);

  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_Start, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_End, &result));
  EXPECT_EQ(-1, result);

  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_End, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_Start, &result));
  EXPECT_EQ(1, result);

  // Compare the endpoints of "some text" and "more text". The position at the
  // end of "some text" is logically equivalent to the position at the start of
  // "more text".
  EXPECT_HRESULT_SUCCEEDED(text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_Start, more_text_range_provider.Get(),
      TextPatternRangeEndpoint_Start, &result));
  EXPECT_EQ(-1, result);

  EXPECT_HRESULT_SUCCEEDED(text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_End, more_text_range_provider.Get(),
      TextPatternRangeEndpoint_Start, &result));
  EXPECT_EQ(0, result);

  // Compare the endpoints of "some text" with those of the entire document. The
  // position at the start of "some text" is logically equivalent to the
  // position at the start of the document.
  EXPECT_HRESULT_SUCCEEDED(text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_Start, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_Start, &result));
  EXPECT_EQ(0, result);

  EXPECT_HRESULT_SUCCEEDED(text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_End, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_End, &result));
  EXPECT_EQ(-1, result);

  // Compare the endpoints of "more text" with those of the entire document.
  EXPECT_HRESULT_SUCCEEDED(more_text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_Start, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_Start, &result));
  EXPECT_EQ(1, result);

  EXPECT_HRESULT_SUCCEEDED(more_text_range_provider->CompareEndpoints(
      TextPatternRangeEndpoint_End, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_End, &result));
  EXPECT_EQ(0, result);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderExpandToEnclosingCharacter) {
  ui::AXTreeUpdate update = BuildTextDocument({"some text", "more text"});
  Init(update);
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);
  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Character));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"s");

  int count;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 2, &count));
  ASSERT_EQ(2, count);
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"om");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Character));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"o");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 9, &count));
  ASSERT_EQ(9, count);
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 8, &count));
  ASSERT_EQ(8, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"mo");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Character));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"m");

  // Move the start and end to the end of the document.
  // Expand to enclosing unit should never return a null position.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 9, &count));
  ASSERT_EQ(8, count);
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 9, &count));
  ASSERT_EQ(9, count);

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Character));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"t");

  // Move both endpoints to the position before the start of the "more text"
  // anchor. Then, force the start to be on the position after the end of
  // "some text" by moving one character backward and one forward.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -9, &count));
  ASSERT_EQ(-9, count);
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ -1,
      &count));
  ASSERT_EQ(-1, count);
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Character));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"m");

  // Check that the enclosing element of the range matches ATs expectations.
  ComPtr<IRawElementProviderSimple> more_text_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple>(
          root_node->children()[1]->children()[0]);
  ComPtr<IRawElementProviderSimple> enclosing_element;
  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(more_text_provider.Get(), enclosing_element.Get());
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderExpandToEnclosingWord) {
  Init(BuildTextDocument({"some text", "definitely not text"},
                         /*build_word_boundaries_offsets*/ true));

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetRootAsAXNode()->children()[1]);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"definitely not text");

  // Start endpoint is already on a word's start boundary.
  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Word));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"definitely ");

  // Start endpoint is between a word's start and end boundaries.
  int count;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ -2,
      &count));
  ASSERT_EQ(-2, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"xtdefinitely ");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 4, &count));
  ASSERT_EQ(4, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"xtdefinitely not ");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Word));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"text");

  // Start endpoint is on a word's end boundary.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 18,
      &count));
  ASSERT_EQ(18, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L" ");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Word));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"not ");
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderExpandToEnclosingLine) {
  Init(BuildTextDocument({"line #1", "maybe line #1?", "not line #1"}));

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetRootAsAXNode()->children()[0]);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"line #1");

  // Start endpoint is already on a line's start boundary.
  int count;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -11, &count));
  ASSERT_EQ(-7, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Line));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"line #1");

  // Start endpoint is between a line's start and end boundaries.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 13,
      &count));
  ASSERT_EQ(13, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 4, &count));
  ASSERT_EQ(4, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"line");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Line));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"maybe line #1?");

  // Start endpoint is on a line's end boundary.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 29,
      &count));
  ASSERT_EQ(25, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Line));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"not line #1");
}

// TOOD(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderExpandToEnclosingParagraph) {
  Init(BuildAXTreeForMove());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider,
                          /*expected_text*/ tree_for_move_full_text.data());

  // Start endpoint is already on a paragraph's start boundary.
  //
  // Note that there are 5 paragraphs, not 6, because the line break element
  // between the first and second paragraph is merged in the text of the first
  // paragraph. This is standard UIA behavior which merges any trailing
  // whitespace with the previous paragraph.
  int count;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Paragraph, /*count*/ -5, &count));
  EXPECT_EQ(-5, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Paragraph));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"First line of text\n");

  // Moving the start by two lines will create a degenerate range positioned
  // at the next paragraph (skipping the newline).
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Line, /*count*/ 2, &count));
  EXPECT_EQ(2, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Paragraph));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"Standalone line\n");

  // Move to the next paragraph via MoveEndpointByUnit (line), then move to
  // the middle of the paragraph via Move (word), then expand by paragraph.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Line, /*count*/ 1, &count));
  EXPECT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 1,
                  /*expected_text*/
                  L"",
                  /*expected_count*/ 1);
  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Paragraph));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"bold text\n");

  // Create a degenerate range at the end of the document, then expand by
  // paragraph.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Document, /*count*/ 1, &count));
  EXPECT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");
  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Paragraph));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"Paragraph 2");
}

// TOOD(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderExpandToEnclosingFormat) {
  Init(BuildAXTreeForMoveByFormat());
  AXNode* root_node = GetRootAsAXNode();
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);
  ComPtr<AXPlatformNodeTextRangeProviderWin> text_range_provider_internal;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->QueryInterface(
      IID_PPV_ARGS(&text_range_provider_internal)));

  EXPECT_UIA_TEXTRANGE_EQ(
      text_range_provider,
      L"Text with formatting\nStandalone line with no formatting\nbold "
      L"text\nParagraph 1\nParagraph 2\nParagraph 3\nParagraph 4");

  // https://docs.microsoft.com/en-us/windows/win32/api/uiautomationclient/nf-uiautomationclient-iuiautomationtextrange-expandtoenclosingunit
  // Consider two consecutive text units A and B.
  // The documentation illustrates 9 cases, but cases 1 and 9 are equivalent.
  // In each case, the expected output is a range from start of A to end of A.

  // Create a range encompassing nodes 11-15 which will serve as text units A
  // and B for this test.
  ComPtr<ITextRangeProvider> units_a_b_provider;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->Clone(&units_a_b_provider));
  CopyOwnerToClone(text_range_provider.Get(), units_a_b_provider.Get());

  int count;
  ASSERT_HRESULT_SUCCEEDED(units_a_b_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Line, /*count*/ 5, &count));
  ASSERT_EQ(5, count);
  ASSERT_HRESULT_SUCCEEDED(units_a_b_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Line, /*count*/ -1, &count));
  ASSERT_EQ(-1, count);
  EXPECT_UIA_TEXTRANGE_EQ(units_a_b_provider,
                          L"Paragraph 1\nParagraph 2\nParagraph 3");

  // Create a range encompassing node 11 which will serve as our expected
  // value of a range from start of A to end of A.
  ComPtr<ITextRangeProvider> unit_a_provider;
  ASSERT_HRESULT_SUCCEEDED(units_a_b_provider->Clone(&unit_a_provider));
  CopyOwnerToClone(units_a_b_provider.Get(), unit_a_provider.Get());
  ASSERT_HRESULT_SUCCEEDED(unit_a_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Line, /*count*/ -2, &count));
  ASSERT_EQ(-2, count);
  EXPECT_UIA_TEXTRANGE_EQ(unit_a_provider, L"Paragraph 1");

  // Case 1: Degenerate range at start of A.
  {
    SCOPED_TRACE("Case 1: Degenerate range at start of A.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByRange(
        TextPatternRangeEndpoint_End, test_case_provider.Get(),
        TextPatternRangeEndpoint_Start));
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }

  // Case 2: Range from start of A to middle of A.
  {
    SCOPED_TRACE("Case 2: Range from start of A to middle of A.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -7,
        &count));
    ASSERT_EQ(-7, count);
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"Para");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }

  // Case 3: Range from start of A to end of A.
  {
    SCOPED_TRACE("Case 3: Range from start of A to end of A.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"Paragraph 1");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }

  // Case 4: Range from start of A to middle of B.
  {
    SCOPED_TRACE("Case 4: Range from start of A to middle of B.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 5, &count));
    ASSERT_EQ(5, count);
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"Paragraph 1\nPara");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }

  // Case 5: Degenerate range in middle of A.
  {
    SCOPED_TRACE("Case 5: Degenerate range in middle of A.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 4,
        &count));
    ASSERT_EQ(4, count);
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByRange(
        TextPatternRangeEndpoint_End, test_case_provider.Get(),
        TextPatternRangeEndpoint_Start));
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }

  // Case 6: Range from middle of A to middle of A.
  {
    SCOPED_TRACE("Case 6: Range from middle of A to middle of A.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 4,
        &count));
    ASSERT_EQ(4, count);
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -2,
        &count));
    ASSERT_EQ(-2, count);
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"graph");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }

  // Case 7: Range from middle of A to end of A.
  {
    SCOPED_TRACE("Case 7: Range from middle of A to end of A.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 4,
        &count));
    ASSERT_EQ(4, count);
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"graph 1");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }

  // Case 8: Range from middle of A to middle of B.
  {
    SCOPED_TRACE("Case 8: Range from middle of A to middle of B.");
    ComPtr<ITextRangeProvider> test_case_provider;
    ASSERT_HRESULT_SUCCEEDED(unit_a_provider->Clone(&test_case_provider));
    CopyOwnerToClone(unit_a_provider.Get(), test_case_provider.Get());
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 5,
        &count));
    ASSERT_EQ(5, count);
    ASSERT_HRESULT_SUCCEEDED(test_case_provider->MoveEndpointByUnit(
        TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 5, &count));
    ASSERT_EQ(5, count);
    EXPECT_UIA_TEXTRANGE_EQ(test_case_provider, L"raph 1\nPara");

    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->ExpandToEnclosingUnit(TextUnit_Format));
    BOOL are_same;
    ASSERT_HRESULT_SUCCEEDED(
        test_case_provider->Compare(unit_a_provider.Get(), &are_same));
    EXPECT_TRUE(are_same);
  }
}

// TOOD(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderExpandToEnclosingFormatWithEmptyObjects) {
  // This test updates the tree structure to test a specific edge case.
  //
  // When using heading navigation, the empty objects (see
  // AXPosition::IsEmptyObjectReplacedByCharacter for information about empty
  // objects) sometimes cause a problem with
  // AXPlatformNodeTextRangeProviderWin::ExpandToEnclosingUnit.
  // With some specific AXTree (like the one used below), the empty object
  // causes ExpandToEnclosingUnit to move the range back on the heading that it
  // previously was instead of moving it forward/backward to the next heading.
  // To avoid this, empty objects are always marked as format boundaries.
  //
  // The issue normally occurs when a heading is directly followed by an ignored
  // empty object, itself followed by an unignored empty object.
  //
  // ++1 kRootWebArea
  // ++++2 kHeading
  // ++++++3 kStaticText
  // ++++++++4 kInlineTextBox
  // ++++5 kGenericContainer ignored
  // ++++6 kButton
  ui::AXNodeData root_1;
  ui::AXNodeData heading_2;
  ui::AXNodeData static_text_3;
  ui::AXNodeData inline_box_4;
  ui::AXNodeData generic_container_5;
  ui::AXNodeData button_6;

  root_1.id = 1;
  heading_2.id = 2;
  static_text_3.id = 3;
  inline_box_4.id = 4;
  generic_container_5.id = 5;
  button_6.id = 6;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {heading_2.id, generic_container_5.id, button_6.id};

  heading_2.role = ax::mojom::Role::kHeading;
  heading_2.child_ids = {static_text_3.id};

  static_text_3.role = ax::mojom::Role::kStaticText;
  static_text_3.child_ids = {inline_box_4.id};
  static_text_3.SetName("3.14");

  inline_box_4.role = ax::mojom::Role::kInlineTextBox;
  inline_box_4.SetName("3.14");

  generic_container_5.role = ax::mojom::Role::kGenericContainer;
  generic_container_5.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  generic_container_5.AddState(ax::mojom::State::kIgnored);

  button_6.role = ax::mojom::Role::kButton;

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_1.id;
  update.nodes.push_back(root_1);
  update.nodes.push_back(heading_2);
  update.nodes.push_back(static_text_3);
  update.nodes.push_back(inline_box_4);
  update.nodes.push_back(generic_container_5);
  update.nodes.push_back(button_6);

  Init(update);

  AXNode* root_node = GetRootAsAXNode();
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"3.14\xFFFC");

  // Create a degenerate range positioned at the boundary between nodes 4 and 6,
  // e.g., "3.14<>" and "<\xFFFC>" (because node 5 is ignored).
  int count;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Character, /*count*/ 5, &count));
  ASSERT_EQ(5, count);
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -1, &count));
  ASSERT_EQ(-1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  // ExpandToEnclosingUnit should move the range to the next non-ignored empty
  // object (i.e, node 6), and not at the beginning of node 4.
  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Format));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"\xFFFC");
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderExpandToEnclosingDocument) {
  Init(BuildTextDocument({"some text", "more text", "even more text"}));

  AXNode* root_node = GetRootAsAXNode();
  AXNode* text_node = root_node->children()[0];
  AXNode* more_text_node = root_node->children()[1];
  AXNode* even_more_text_node = root_node->children()[2];

  // Run the test twice, one for TextUnit_Document and once for TextUnit_Page,
  // since they should have identical behavior.
  const TextUnit textunit_types[] = {TextUnit_Document, TextUnit_Page};
  ComPtr<ITextRangeProvider> text_range_provider;

  for (auto& textunit : textunit_types) {
    GetTextRangeProviderFromTextNode(text_range_provider, text_node);
    ASSERT_HRESULT_SUCCEEDED(
        text_range_provider->ExpandToEnclosingUnit(textunit));
    EXPECT_UIA_TEXTRANGE_EQ(text_range_provider,
                            L"some textmore texteven more text");

    GetTextRangeProviderFromTextNode(text_range_provider, more_text_node);
    ASSERT_HRESULT_SUCCEEDED(
        text_range_provider->ExpandToEnclosingUnit(textunit));
    EXPECT_UIA_TEXTRANGE_EQ(text_range_provider,
                            L"some textmore texteven more text");

    GetTextRangeProviderFromTextNode(text_range_provider, even_more_text_node);
    ASSERT_HRESULT_SUCCEEDED(
        text_range_provider->ExpandToEnclosingUnit(textunit));
    EXPECT_UIA_TEXTRANGE_EQ(text_range_provider,
                            L"some textmore texteven more text");
  }
}

// TOOD(schectman) Why should this be ignored?
// https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderIgnoredForTextNavigation) {
  // ++1 kRootWebArea
  // ++++2 kStaticText
  // ++++++3 kInlineTextBox foo
  // ++++4 kSplitter
  // ++++5 kStaticText
  // ++++++6 kInlineTextBox bar
  // ++++7 genericContainer
  // ++++8 kStaticText
  // ++++++9 kInlineTextBox baz
  ui::AXNodeData root_1;
  ui::AXNodeData static_text_2;
  ui::AXNodeData inline_box_3;
  ui::AXNodeData splitter_4;
  ui::AXNodeData static_text_5;
  ui::AXNodeData inline_box_6;
  ui::AXNodeData generic_container_7;
  ui::AXNodeData static_text_8;
  ui::AXNodeData inline_box_9;

  root_1.id = 1;
  static_text_2.id = 2;
  inline_box_3.id = 3;
  splitter_4.id = 4;
  static_text_5.id = 5;
  inline_box_6.id = 6;
  generic_container_7.id = 7;
  static_text_8.id = 8;
  inline_box_9.id = 9;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {static_text_2.id, splitter_4.id, static_text_5.id,
                      generic_container_7.id, static_text_8.id};

  static_text_2.role = ax::mojom::Role::kStaticText;
  static_text_2.child_ids = {inline_box_3.id};
  static_text_2.SetName("foo");

  inline_box_3.role = ax::mojom::Role::kInlineTextBox;
  inline_box_3.SetName("foo");

  splitter_4.role = ax::mojom::Role::kSplitter;
  splitter_4.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  static_text_5.role = ax::mojom::Role::kStaticText;
  static_text_5.child_ids = {inline_box_6.id};
  static_text_5.SetName("bar");

  inline_box_6.role = ax::mojom::Role::kInlineTextBox;
  inline_box_6.SetName("bar");

  generic_container_7.role = ax::mojom::Role::kGenericContainer;
  generic_container_7.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  static_text_8.role = ax::mojom::Role::kStaticText;
  static_text_8.child_ids = {inline_box_9.id};
  static_text_8.SetName("bar");

  inline_box_9.role = ax::mojom::Role::kInlineTextBox;
  inline_box_9.SetName("baz");

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_1.id;
  update.nodes = {
      root_1,        static_text_2, inline_box_3,        splitter_4,
      static_text_5, inline_box_6,  generic_container_7, static_text_8,
      inline_box_9};

  Init(update);

  AXNode* root_node = GetRootAsAXNode();
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider,
                          L"foo\n\xFFFC\nbar\n\xFFFC\nbaz");

  int count;
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Paragraph, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"bar\n\xFFFC\nbaz");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_Start, TextUnit_Paragraph, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"baz");
}

// TODO(schectman) Segfault after test completes.
// Why? https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderInvalidCalls) {
  // Test for when a text range provider is invalid. Because no ax tree is
  // available, the anchor is invalid, so the text range provider fails the
  // validate call.
  {
    Init(BuildTextDocument({}));

    ComPtr<ITextRangeProvider> text_range_provider;
    GetTextRangeProviderFromTextNode(text_range_provider, GetRootAsAXNode());

    DestroyTree();
    ComPtr<ITextRangeProvider> text_range_provider_clone;
    EXPECT_UIA_ELEMENTNOTAVAILABLE(
        text_range_provider->Clone(&text_range_provider_clone));

    BOOL compare_result;
    EXPECT_UIA_ELEMENTNOTAVAILABLE(text_range_provider->Compare(
        text_range_provider.Get(), &compare_result));

    int compare_endpoints_result;
    EXPECT_UIA_ELEMENTNOTAVAILABLE(text_range_provider->CompareEndpoints(
        TextPatternRangeEndpoint_Start, text_range_provider.Get(),
        TextPatternRangeEndpoint_Start, &compare_endpoints_result));

    VARIANT attr_val;
    V_VT(&attr_val) = VT_BOOL;
    V_BOOL(&attr_val) = VARIANT_TRUE;
    ComPtr<ITextRangeProvider> matched_range_provider;
    EXPECT_UIA_ELEMENTNOTAVAILABLE(text_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, attr_val, true, &matched_range_provider));

    EXPECT_UIA_ELEMENTNOTAVAILABLE(text_range_provider->MoveEndpointByRange(
        TextPatternRangeEndpoint_Start, text_range_provider.Get(),
        TextPatternRangeEndpoint_Start));

    EXPECT_UIA_ELEMENTNOTAVAILABLE(text_range_provider->Select());
  }

  // Test for when this provider is valid, but the other provider is not an
  // instance of AXPlatformNodeTextRangeProviderWin, so no operation can be
  // performed on the other provider.
  {
    Init(BuildTextDocument({}));

    ComPtr<ITextRangeProvider> this_provider;
    GetTextRangeProviderFromTextNode(this_provider, GetRootAsAXNode());

    ComPtr<ITextRangeProvider> other_provider_different_type;
    MockAXPlatformNodeTextRangeProviderWin::CreateMockTextRangeProvider(
        &other_provider_different_type);

    BOOL compare_result;
    EXPECT_UIA_INVALIDOPERATION(this_provider->Compare(
        other_provider_different_type.Get(), &compare_result));

    int compare_endpoints_result;
    EXPECT_UIA_INVALIDOPERATION(this_provider->CompareEndpoints(
        TextPatternRangeEndpoint_Start, other_provider_different_type.Get(),
        TextPatternRangeEndpoint_Start, &compare_endpoints_result));

    EXPECT_UIA_INVALIDOPERATION(this_provider->MoveEndpointByRange(
        TextPatternRangeEndpoint_Start, other_provider_different_type.Get(),
        TextPatternRangeEndpoint_Start));
  }
}

TEST_F(AXPlatformNodeTextRangeProviderTest, TestITextRangeProviderGetText) {
  Init(BuildTextDocument({"some text", "more text"}));

  AXNode* root_node = GetRootAsAXNode();
  AXNode* text_node = root_node->children()[0];

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, text_node);

  base::win::ScopedBstr text_content;
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetText(-1, text_content.Receive()));
  EXPECT_STREQ(text_content.Get(), L"some text");
  text_content.Reset();

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetText(4, text_content.Receive()));
  EXPECT_STREQ(text_content.Get(), L"some");
  text_content.Reset();

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetText(0, text_content.Receive()));
  EXPECT_STREQ(text_content.Get(), L"");
  text_content.Reset();

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetText(9, text_content.Receive()));
  EXPECT_STREQ(text_content.Get(), L"some text");
  text_content.Reset();

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetText(10, text_content.Receive()));
  EXPECT_STREQ(text_content.Get(), L"some text");
  text_content.Reset();

  EXPECT_HRESULT_FAILED(text_range_provider->GetText(-1, nullptr));

  EXPECT_HRESULT_FAILED(
      text_range_provider->GetText(-2, text_content.Receive()));
  text_content.Reset();

  ComPtr<ITextRangeProvider> document_textrange;
  GetTextRangeProviderFromTextNode(document_textrange, root_node);

  EXPECT_HRESULT_SUCCEEDED(
      document_textrange->GetText(-1, text_content.Receive()));
  EXPECT_STREQ(text_content.Get(), L"some textmore text");
  text_content.Reset();
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderMoveCharacter) {
  Init(BuildAXTreeForMove());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  // Moving by 0 should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character, /*count*/ 0,
                  /*expected_text*/
                  L"First line of text\nStandalone line\n"
                  L"bold textParagraph 1Paragraph 2",
                  /*expected_count*/ 0);

  // Move forward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 1,
                  /*expected_text*/ L"i",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 18,
                  /*expected_text*/ L"S",
                  /*expected_count*/ 18);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 16,
                  /*expected_text*/ L"b",
                  /*expected_count*/ 16);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 60,
                  /*expected_text*/ L"2",
                  /*expected_count*/ 30);

  // Trying to move past the last character should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 1,
                  /*expected_text*/ L"2",
                  /*expected_count*/ 0);

  // Move backward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ -2,
                  /*expected_text*/ L"h",
                  /*expected_count*/ -2);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ -9,
                  /*expected_text*/ L"1",
                  /*expected_count*/ -9);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ -60,
                  /*expected_text*/ L"F",
                  /*expected_count*/ -54);

  // Moving backward by any number of characters at the start of document
  // should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ -1,
                  /*expected_text*/
                  L"F",
                  /*expected_count*/ 0);

  // Degenerate range moves.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ -1,
      /*expected_text*/ L"",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 4,
                  /*expected_text*/ L"",
                  /*expected_count*/ 4);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 62);

  // Trying to move past the last character should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Character,
                  /*count*/ -2,
                  /*expected_text*/ L"",
                  /*expected_count*/ -2);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveFormat) {
  Init(BuildAXTreeForMoveByFormat());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  // Moving by 0 should have no effect.
  EXPECT_UIA_MOVE(
      text_range_provider, TextUnit_Format,
      /*count*/ 0,
      /*expected_text*/
      L"Text with formatting\nStandalone line with no formatting\nbold "
      L"text\nParagraph 1\nParagraph 2\nParagraph 3\nParagraph 4",
      /*expected_count*/ 0);

  // Move forward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 1,
                  /*expected_text*/ L"\nStandalone line with no formatting\n",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 2,
                  /*expected_text*/ L"Paragraph 1",
                  /*expected_count*/ 2);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 1,
                  /*expected_text*/ L"Paragraph 2\nParagraph 3",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 1,
                  /*expected_text*/ L"Paragraph 4",
                  /*expected_count*/ 1);

  // Trying to move past the last format should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 1,
                  /*expected_text*/ L"Paragraph 4",
                  /*expected_count*/ 0);

  // Move backward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ -3,
                  /*expected_text*/ L"bold text",
                  /*expected_count*/ -3);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ -1,
                  /*expected_text*/ L"\nStandalone line with no formatting\n",
                  /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ -1,
                  /*expected_text*/ L"Text with formatting",
                  /*expected_count*/ -1);

  // Moving backward by any number of formats at the start of document
  // should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ -1,
                  /*expected_text*/
                  L"Text with formatting",
                  /*expected_count*/ 0);

  // Test degenerate range creation at the beginning of the document.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/ L"",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ 1,
      /*expected_text*/ L"Text with formatting",
      /*expected_count*/ 1);

  // Test degenerate range creation at the end of the document.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 5,
                  /*expected_text*/ L"Paragraph 4",
                  /*expected_count*/ 5);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Format,
      /*count*/ 1,
      /*expected_text*/ L"",
      /*expected_count*/ 1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/ L"Paragraph 4",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Format,
      /*count*/ 1,
      /*expected_text*/ L"",
      /*expected_count*/ 1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/ L"Paragraph 4",
      /*expected_count*/ -1);

  // Degenerate range moves.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ -5,
                  /*expected_text*/ L"Text with formatting",
                  /*expected_count*/ -5);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/ L"",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 3,
                  /*expected_text*/ L"",
                  /*expected_count*/ 3);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 3);

  // Trying to move past the last format should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Format,
                  /*count*/ -2,
                  /*expected_text*/ L"",
                  /*expected_count*/ -2);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveWord) {
  Init(BuildAXTreeForMove());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  // Moving by 0 should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word, /*count*/ 0,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);

  // Move forward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 1,
                  /*expected_text*/ L"line ",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 2,
                  /*expected_text*/ L"text",
                  /*expected_count*/ 2);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 2,
                  /*expected_text*/ L"line",
                  /*expected_count*/ 2);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 3,
                  /*expected_text*/ L"Paragraph ",
                  /*expected_count*/ 3);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 6,
                  /*expected_text*/ L"2",
                  /*expected_count*/ 3);

  // Trying to move past the last word should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 1,
                  /*expected_text*/ L"2",
                  /*expected_count*/ 0);

  // Move backward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ -3,
                  /*expected_text*/ L"Paragraph ",
                  /*expected_count*/ -3);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ -3,
                  /*expected_text*/ L"line",
                  /*expected_count*/ -3);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ -2,
                  /*expected_text*/ L"text",
                  /*expected_count*/ -2);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ -6,
                  /*expected_text*/ L"First ",
                  /*expected_count*/ -3);

  // Moving backward by any number of words at the start of document
  // should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ -20,
                  /*expected_text*/ L"First ",
                  /*expected_count*/ 0);

  // Degenerate range moves.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ -1,
                                   /*expected_text*/ L"",
                                   /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 4,
                  /*expected_text*/ L"",
                  /*expected_count*/ 4);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 8);

  // Trying to move past the last word should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Word,
                  /*count*/ -2,
                  /*expected_text*/ L"",
                  /*expected_count*/ -2);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveLine) {
  Init(BuildAXTreeForMove());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  // Moving by 0 should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line, /*count*/ 0,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);

  // Move forward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ 2,
                  /*expected_text*/ L"Standalone line",
                  /*expected_count*/ 2);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ 1,
                  /*expected_text*/ L"bold text",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ 10,
                  /*expected_text*/ L"Paragraph 2",
                  /*expected_count*/ 2);

  // Trying to move past the last line should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ 1,
                  /*expected_text*/ L"Paragraph 2",
                  /*expected_count*/ 0);

  // Move backward.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ -1,
                  /*expected_text*/ L"Paragraph 1",
                  /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ -5,
                  /*expected_text*/ L"First line of text",
                  /*expected_count*/ -4);

  // Moving backward by any number of lines at the start of document
  // should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ -20,
                  /*expected_text*/ L"First line of text",
                  /*expected_count*/ 0);

  // Degenerate range moves.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Line,
                                   /*count*/ -1,
                                   /*expected_text*/ L"",
                                   /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ 4,
                  /*expected_text*/ L"",
                  /*expected_count*/ 4);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 2);

  // Trying to move past the last line should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Line,
                  /*count*/ -2,
                  /*expected_text*/ L"",
                  /*expected_count*/ -2);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveParagraph) {
  Init(BuildAXTreeForMove());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  // Moving by 0 should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph, /*count*/ 0,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Paragraph,
      /*count*/ -4,
      /*expected_text*/ L"First line of text\n",
      /*expected_count*/ -4);

  // The first line break does not create an empty paragraph because even though
  // it is in a block element (i.e. a kGenericContainer) of its own which is a
  // line breaking object, it merges with the previous paragraph. This is
  // standard UIA behavior which merges any trailing whitespace with the
  // previous paragraph.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Paragraph,
      /*count*/ -1,
      /*expected_text*/ L"",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Paragraph,
      /*count*/ 1,
      /*expected_text*/ L"First line of text\n",
      /*expected_count*/ 1);

  //
  // Move forward.
  //

  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 1,
                  /*expected_text*/ L"Standalone line\n",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 1,
                  /*expected_text*/ L"bold text\n",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 1,
                  /*expected_text*/ L"Paragraph 1\n",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 1,
                  /*expected_text*/ L"Paragraph 2",
                  /*expected_count*/ 1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 2,
                  /*expected_text*/ L"Paragraph 2",
                  /*expected_count*/ 0);

  // Trying to move past the last paragraph should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 1,
                  /*expected_text*/ L"Paragraph 2",
                  /*expected_count*/ 0);

  //
  // Move backward.
  //

  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -1,
                  /*expected_text*/ L"Paragraph 1\n",
                  /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -1,
                  /*expected_text*/ L"bold text\n",
                  /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -1,
                  /*expected_text*/ L"Standalone line\n",
                  /*expected_count*/ -1);
  // The first line break creates an empty paragraph because it is in a block
  // element (i.e. a kGenericContainer) of its own which is a line breaking
  // object. It's like having a <br> element wrapped inside a <div>.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -1,
                  /*expected_text*/ L"First line of text\n",
                  /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -1,
                  /*expected_text*/ L"First line of text\n",
                  /*expected_count*/ 0);

  // Moving backward by any number of paragraphs at the start of document
  // should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -1,
                  /*expected_text*/ L"First line of text\n",
                  /*expected_count*/ 0);

  // Test degenerate range creation at the beginning of the document.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Paragraph,
      /*count*/ -1,
      /*expected_text*/ L"",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Paragraph,
      /*count*/ 1,
      /*expected_text*/ L"First line of text\n",
      /*expected_count*/ 1);

  // Test degenerate range creation at the end of the document.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 5,
                  /*expected_text*/ L"Paragraph 2",
                  /*expected_count*/ 4);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Paragraph,
      /*count*/ 1,
      /*expected_text*/ L"",
      /*expected_count*/ 1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Paragraph,
      /*count*/ -1,
      /*expected_text*/ L"Paragraph 2",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Paragraph,
      /*count*/ 1,
      /*expected_text*/ L"",
      /*expected_count*/ 1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Paragraph,
      /*count*/ -1,
      /*expected_text*/ L"Paragraph 2",
      /*expected_count*/ -1);

  //
  // Degenerate range moves.
  //

  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -6,
                  /*expected_text*/ L"First line of text\n",
                  /*expected_count*/ -4);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Paragraph,
      /*count*/ -1,
      /*expected_text*/ L"",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 3,
                  /*expected_text*/ L"",
                  /*expected_count*/ 3);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 2);

  // Trying to move past the last paragraph should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ 70,
                  /*expected_text*/ L"",
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Paragraph,
                  /*count*/ -2,
                  /*expected_text*/ L"",
                  /*expected_count*/ -2);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveDocument) {
  Init(BuildAXTreeForMove());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  // Moving by 0 should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Document, /*count*/ 0,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);

  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Document, /*count*/ -1,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Document, /*count*/ 2,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Page, /*count*/ 1,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Page, /*count*/ -1,
                  /*expected_text*/ tree_for_move_full_text.data(),
                  /*expected_count*/ 0);

  // Degenerate range moves.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Document,
      /*count*/ -2,
      /*expected_text*/ L"",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Page,
                  /*count*/ 4,
                  /*expected_text*/ L"",
                  /*expected_count*/ 1);

  // Trying to move past the last character should have no effect.
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Document,
                  /*count*/ 1,
                  /*expected_text*/ L"",
                  /*expected_count*/ 0);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Page,
                  /*count*/ -2,
                  /*expected_text*/ L"",
                  /*expected_count*/ -1);
  EXPECT_UIA_MOVE(text_range_provider, TextUnit_Document,
                  /*count*/ -1,
                  /*expected_text*/ L"",
                  /*expected_count*/ 0);
}

TEST_F(AXPlatformNodeTextRangeProviderTest, TestITextRangeProviderMove) {
  Init(BuildAXTreeForMove());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  // TODO(https://crbug.com/928948): test intermixed unit types
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderMoveEndpointByDocument) {
  Init(BuildTextDocument({"some text", "more text", "even more text"}));
  AXNode* text_node = GetRootAsAXNode()->children()[1];

  // Run the test twice, one for TextUnit_Document and once for TextUnit_Page,
  // since they should have identical behavior.
  const TextUnit textunit_types[] = {TextUnit_Document, TextUnit_Page};
  ComPtr<ITextRangeProvider> text_range_provider;

  for (auto& textunit : textunit_types) {
    GetTextRangeProviderFromTextNode(text_range_provider, text_node);

    // Verify MoveEndpointByUnit with zero count has no effect
    EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                     TextPatternRangeEndpoint_End, textunit,
                                     /*count*/ 0,
                                     /*expected_text*/ L"more text",
                                     /*expected_count*/ 0);

    // Move the endpoint to the end of the document. Verify all text content.
    EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
        text_range_provider, TextPatternRangeEndpoint_End, textunit,
        /*count*/ 1,
        /*expected_text*/ L"more texteven more text",
        /*expected_count*/ 1);

    // Verify no moves occur since the end is already at the end of the document
    EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
        text_range_provider, TextPatternRangeEndpoint_End, textunit,
        /*count*/ 5,
        /*expected_text*/ L"more texteven more text",
        /*expected_count*/ 0);

    // Move the end before the start
    EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                     TextPatternRangeEndpoint_End, textunit,
                                     /*count*/ -4,
                                     /*expected_text*/ L"",
                                     /*expected_count*/ -1);

    // Move the end back to the end of the document. The text content
    // should now include the entire document since end was previously
    // moved before start.
    EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
        text_range_provider, TextPatternRangeEndpoint_End, textunit,
        /*count*/ 1,
        /*expected_text*/ L"some textmore texteven more text",
        /*expected_count*/ 1);

    // Move the start point to the end
    EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                     TextPatternRangeEndpoint_Start, textunit,
                                     /*count*/ 3,
                                     /*expected_text*/ L"",
                                     /*expected_count*/ 1);

    // Move the start point back to the beginning
    EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
        text_range_provider, TextPatternRangeEndpoint_Start, textunit,
        /*count*/ -3,
        /*expected_text*/ L"some textmore texteven more text",
        /*expected_count*/ -1);
  }
}

// TODO(schectman) We are probably not accounting for multibyte characters
// properly yet. https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveEndpointByCharacterMultilingual) {
  // The English string has three characters, each 8 bits in length.
  const std::string english = "hey";

  // The Hindi string has two characters, the first one 32 bits and the second
  // 64 bits in length. It is formatted in UTF16.
  const std::string hindi =
      base::UTF16ToUTF8(u"\x0939\x093F\x0928\x094D\x0926\x0940");

  // The Thai string has three characters, the first one 48, the second 32 and
  // the last one 16 bits in length. It is formatted in UTF16.
  const std::string thai =
      base::UTF16ToUTF8(u"\x0E23\x0E39\x0E49\x0E2A\x0E36\x0E01");

  Init(BuildTextDocument({english, hindi, thai}));

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetRootAsAXNode()->children()[0]);

  // Verify MoveEndpointByUnit with zero count has no effect
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"hey");
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 0,
      /*expected_text*/ L"hey",
      /*expected_count*/ 0);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"ey",
      /*expected_count*/ 1);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ -1,
      /*expected_text*/ L"e",
      /*expected_count*/ -1);

  // Move end into the adjacent node.
  //
  // The first character of the second node is 32 bits in length.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 2,
      /*expected_text*/ L"ey\x0939\x093F",
      /*expected_count*/ 2);

  // The second character of the second node is 64 bits in length.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"ey\x939\x93F\x928\x94D\x926\x940",
      /*expected_count*/ 1);

  // Move start into the adjacent node as well.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 2,
      /*expected_text*/ L"\x939\x93F\x928\x94D\x926\x940",
      /*expected_count*/ 2);

  // Move end into the last node.
  //
  // The first character of the last node is 48 bits in length.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"\x939\x93F\x928\x94D\x926\x940\xE23\xE39\xE49",
      /*expected_count*/ 1);

  // Move end back into the second node and then into the last node again.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ -2,
      /*expected_text*/ L"\x939\x93F",
      /*expected_count*/ -2);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 3,
      /*expected_text*/
      L"\x939\x93F\x928\x94D\x926\x940\xE23\xE39\xE49\xE2A\xE36",
      /*expected_count*/ 3);

  // The last character of the last node is only 16 bits in length.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/
      L"\x939\x93F\x928\x94D\x926\x940\xE23\xE39\xE49\xE2A\xE36\xE01",
      /*expected_count*/ 1);

  // Move start into the last node.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 3,
      /*expected_text*/ L"\x0E2A\x0E36\x0E01",
      /*expected_count*/ 3);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ -1,
      /*expected_text*/ L"\x0E23\x0E39\x0E49\x0E2A\x0E36\x0E01",
      /*expected_count*/ -1);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveEndpointByWord) {
  Init(BuildTextDocument({"some text", "more text", "even more text"},
                         /*build_word_boundaries_offsets*/ true));

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetRootAsAXNode()->children()[1]);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"more text");

  // Moving with zero count does not alter the range.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ 0,
                                   /*expected_text*/ L"more text",
                                   /*expected_count*/ 0);

  // Moving the start forward and backward.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Word,
      /*count*/ 1,
      /*expected_text*/ L"text",
      /*expected_count*/ 1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Word,
      /*count*/ -1,
      /*expected_text*/ L"more text",
      /*expected_count*/ -1);

  // Moving the end backward and forward.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ -1,
                                   /*expected_text*/ L"more ",
                                   /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ 1,
                                   /*expected_text*/ L"more text",
                                   /*expected_count*/ 1);

  // Moving the start past the end, then reverting.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Word,
      /*count*/ 3,
      /*expected_text*/ L"",
      /*expected_count*/ 3);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Word,
      /*count*/ -3,
      /*expected_text*/ L"more texteven ",
      /*expected_count*/ -3);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ -1,
                                   /*expected_text*/ L"more text",
                                   /*expected_count*/ -1);

  // Moving the end past the start, then reverting.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ -3,
                                   /*expected_text*/ L"",
                                   /*expected_count*/ -3);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ 3,
                                   /*expected_text*/ L"textmore text",
                                   /*expected_count*/ 3);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Word,
      /*count*/ 1,
      /*expected_text*/ L"more text",
      /*expected_count*/ 1);

  // Moving the endpoints further than both ends of the document.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ 5,
                                   /*expected_text*/ L"more texteven more text",
                                   /*expected_count*/ 3);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Word,
      /*count*/ 6,
      /*expected_text*/ L"",
      /*expected_count*/ 5);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Word,
      /*count*/ -8,
      /*expected_text*/ L"some textmore texteven more text",
      /*expected_count*/ -7);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Word,
                                   /*count*/ -8,
                                   /*expected_text*/ L"",
                                   /*expected_count*/ -7);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveEndpointByLine) {
  Init(BuildTextDocument({"0", "1", "2", "3", "4", "5", "6"}));

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetRootAsAXNode()->children()[3]);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"3");

  // Moving with zero count does not alter the range.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Line,
                                   /*count*/ 0,
                                   /*expected_text*/ L"3",
                                   /*expected_count*/ 0);

  // Moving the start backward and forward.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Line,
      /*count*/ -2,
      /*expected_text*/ L"123",
      /*expected_count*/ -2);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Line,
      /*count*/ 1,
      /*expected_text*/ L"23",
      /*expected_count*/ 1);

  // Moving the end forward and backward.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Line,
                                   /*count*/ 3,
                                   /*expected_text*/ L"23456",
                                   /*expected_count*/ 3);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Line,
                                   /*count*/ -2,
                                   /*expected_text*/ L"234",
                                   /*expected_count*/ -2);

  // Moving the end past the start and vice versa.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Line,
                                   /*count*/ -4,
                                   /*expected_text*/ L"",
                                   /*expected_count*/ -4);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Line,
      /*count*/ -1,
      /*expected_text*/ L"0",
      /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Line,
      /*count*/ 6,
      /*expected_text*/ L"",
      /*expected_count*/ 6);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Line,
      /*count*/ -6,
      /*expected_text*/ L"012345",
      /*expected_count*/ -6);

  // Moving the endpoints further than both ends of the document.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Line,
                                   /*count*/ -13,
                                   /*expected_text*/ L"",
                                   /*expected_count*/ -6);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(text_range_provider,
                                   TextPatternRangeEndpoint_End, TextUnit_Line,
                                   /*count*/ 11,
                                   /*expected_text*/ L"0123456",
                                   /*expected_count*/ 7);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Line,
      /*count*/ 9,
      /*expected_text*/ L"",
      /*expected_count*/ 7);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Line,
      /*count*/ -7,
      /*expected_text*/ L"0123456",
      /*expected_count*/ -7);
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
// Verify that the endpoint can move past an empty text field.
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveEndpointByUnitTextField) {
  // An empty text field should also be a character, word, and line boundary.
  ui::AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  ui::AXNodeData group1_data;
  group1_data.id = 2;
  group1_data.role = ax::mojom::Role::kGenericContainer;

  ui::AXNodeData text_data;
  text_data.id = 3;
  text_data.role = ax::mojom::Role::kStaticText;
  std::string text_content = "some text";
  text_data.SetName(text_content);
  std::vector<int> word_start_offsets, word_end_offsets;
  ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                               word_end_offsets);
  text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                word_start_offsets);
  text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                word_end_offsets);

  ui::AXNodeData text_input_data;
  text_input_data.id = 4;
  text_input_data.role = ax::mojom::Role::kTextField;
  text_input_data.AddState(ax::mojom::State::kEditable);
  text_input_data.AddStringAttribute(ax::mojom::StringAttribute::kHtmlTag,
                                     "input");
  text_input_data.AddStringAttribute(ax::mojom::StringAttribute::kInputType,
                                     "text");

  ui::AXNodeData group2_data;
  group2_data.id = 5;
  group2_data.role = ax::mojom::Role::kGenericContainer;

  ui::AXNodeData more_text_data;
  more_text_data.id = 6;
  more_text_data.role = ax::mojom::Role::kStaticText;
  text_content = "more text";
  more_text_data.SetName(text_content);
  ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                               word_end_offsets);
  more_text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                     word_start_offsets);
  more_text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                     word_end_offsets);

  ui::AXNodeData empty_text_data;
  empty_text_data.id = 7;
  empty_text_data.role = ax::mojom::Role::kStaticText;
  empty_text_data.AddState(ax::mojom::State::kEditable);
  text_content = "";
  empty_text_data.SetNameExplicitlyEmpty();
  ComputeWordBoundariesOffsets(text_content, word_start_offsets,
                               word_end_offsets);
  empty_text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                      word_start_offsets);
  empty_text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                      word_end_offsets);

  root_data.child_ids = {group1_data.id, text_input_data.id, group2_data.id};
  group1_data.child_ids = {text_data.id};
  text_input_data.child_ids = {empty_text_data.id};
  group2_data.child_ids = {more_text_data.id};

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_data.id;
  update.nodes = {root_data,   group1_data,    text_data,      text_input_data,
                  group2_data, more_text_data, empty_text_data};

  Init(update);

  // Set up variables from the tree for testing.
  AXNode* root_node = GetRootAsAXNode();
  AXNode* text_node = root_node->children()[0]->children()[0];

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, text_node);

  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text");

  int count;
  // Tests for TextUnit_Character.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 2, &count));
  ASSERT_EQ(2, count);
  // Note that by design, empty objects such as empty text fields, are placed in
  // their own paragraph for easier screen reader navigation.
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFc");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ 2, &count));
  ASSERT_EQ(2, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFc\nm");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -1, &count));
  ASSERT_EQ(-1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFC\n");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -2, &count));
  ASSERT_EQ(-2, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n");

  // Tests for TextUnit_Word.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Word, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFC\n");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Word, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFC\nmore ");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Word, /*count*/ -1, &count));
  ASSERT_EQ(-1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFC\n");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Word, /*count*/ -1, &count));
  ASSERT_EQ(-1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n");

  // Tests for TextUnit_Line.
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Line, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFC");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Line, /*count*/ 1, &count));
  ASSERT_EQ(1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFC\nmore text");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Line, /*count*/ -1, &count));
  ASSERT_EQ(-1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text\n\xFFFC");

  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Line, /*count*/ -1, &count));
  ASSERT_EQ(-1, count);
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"some text");
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderMoveEndpointByFormat) {
  Init(BuildAXTreeForMoveByFormat());
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  EXPECT_UIA_TEXTRANGE_EQ(
      text_range_provider,
      L"Text with formatting\nStandalone line with no formatting\nbold "
      L"text\nParagraph 1\nParagraph 2\nParagraph 3\nParagraph 4");
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -2,
      /*expected_text*/
      L"Text with formatting\nStandalone line with no formatting\nbold "
      L"text\nParagraph 1",
      /*expected_count*/ -2);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/
      L"Text with formatting\nStandalone line with no formatting\nbold text",
      /*expected_count*/ -1);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/
      L"Text with formatting\nStandalone line with no formatting\n",
      /*expected_count*/ -1);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/ L"Text with formatting",
      /*expected_count*/ -1);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -1,
      /*expected_text*/ L"",
      /*expected_count*/ -1);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ 7,
      /*expected_text*/
      L"Text with formatting\nStandalone line with no formatting\nbold "
      L"text\nParagraph 1\nParagraph 2\nParagraph 3\nParagraph 4",
      /*expected_count*/ 6);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Format,
      /*count*/ -8,
      /*expected_text*/ L"",
      /*expected_count*/ -6);
}

TEST_F(AXPlatformNodeTextRangeProviderTest, TestITextRangeProviderCompare) {
  Init(BuildTextDocument({"some text", "some text"}));
  AXNode* root_node = GetRootAsAXNode();

  // Get the textRangeProvider for the document,
  // which contains text "some textsome text".
  ComPtr<ITextRangeProvider> document_text_range_provider;
  GetTextRangeProviderFromTextNode(document_text_range_provider, root_node);

  // Get the textRangeProvider for the first text node.
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   root_node->children()[0]);

  // Get the textRangeProvider for the second text node.
  ComPtr<ITextRangeProvider> more_text_range_provider;
  GetTextRangeProviderFromTextNode(more_text_range_provider,
                                   root_node->children()[1]);

  // Compare text range of the entire document with itself, which should return
  // that they are equal.
  BOOL result;
  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->Compare(
      document_text_range_provider.Get(), &result));
  EXPECT_TRUE(result);

  // Compare the text range of the entire document with one of its child, which
  // should return that they are not equal.
  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->Compare(
      text_range_provider.Get(), &result));
  EXPECT_FALSE(result);

  // Compare the text range of text_node which contains "some text" with
  // text range of more_text_node which also contains "some text". Those two
  // text ranges should not equal, because their endpoints are different, even
  // though their contents are the same.
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->Compare(more_text_range_provider.Get(), &result));
  EXPECT_FALSE(result);
}

TEST_F(AXPlatformNodeTextRangeProviderTest, TestITextRangeProviderSelection) {
  Init(BuildTextDocument({"some text"}));

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, GetRootAsAXNode());

  ASSERT_UIA_INVALIDOPERATION(text_range_provider->AddToSelection());
  ASSERT_UIA_INVALIDOPERATION(text_range_provider->RemoveFromSelection());
}

// TODO(schectman) Rectangles not implemented as in Chromium.
// https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderGetBoundingRectangles) {
  ui::AXTreeUpdate update = BuildAXTreeForBoundingRectangles();
  Init(update);
  ComPtr<ITextRangeProvider> text_range_provider;
  base::win::ScopedSafearray rectangles;
  int units_moved;

  // Expected bounding rects:
  // <button>Button</button><input type="checkbox">Line 1<br>Line 2
  // |---------------------||---------------------||----|   |------|
  GetTextRangeProviderFromTextNode(text_range_provider, GetRootAsAXNode());
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetBoundingRectangles(rectangles.Receive()));
  std::vector<double> expected_values = {20,  20, 200, 30, /* button */
                                         20,  50, 200, 30, /* check box */
                                         220, 20, 30,  30, /* line 1 */
                                         220, 50, 42,  30 /* line 2 */};
  EXPECT_UIA_SAFEARRAY_EQ(rectangles.Get(), expected_values);
  rectangles.Reset();

  // Move the text range end back by one character.
  // Expected bounding rects:
  // <button>Button</button><input type="checkbox">Line 1<br>Line 2
  // |---------------------||---------------------||----|   |----|
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Character, /*count*/ -1,
      &units_moved));
  ASSERT_EQ(-1, units_moved);
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetBoundingRectangles(rectangles.Receive()));
  expected_values = {20,  20, 200, 30, /* button */
                     20,  50, 200, 30, /* check box */
                     220, 20, 30,  30, /* line 1 */
                     220, 50, 35,  30 /* line 2 */};
  EXPECT_UIA_SAFEARRAY_EQ(rectangles.Get(), expected_values);
  rectangles.Reset();

  // Move the text range end back by one line.
  // Expected bounding rects:
  // <button>Button</button><input type="checkbox">Line 1<br>Line 2
  // |---------------------||---------------------||--------|
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Line, /*count*/ -1, &units_moved));
  ASSERT_EQ(-1, units_moved);
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetBoundingRectangles(rectangles.Receive()));
  expected_values = {20,  20, 200, 30, /* button */
                     20,  50, 200, 30, /* check box */
                     220, 20, 30,  30 /* line 1 */};
  EXPECT_UIA_SAFEARRAY_EQ(rectangles.Get(), expected_values);
  rectangles.Reset();

  // Move the text range end back by one line.
  // Expected bounding rects:
  // <button>Button</button><input type="checkbox">Line 1<br>Line 2
  // |---------------------||---------------------|
  ASSERT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByUnit(
      TextPatternRangeEndpoint_End, TextUnit_Word, /*count*/ -3, &units_moved));
  ASSERT_EQ(-3, units_moved);
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetBoundingRectangles(rectangles.Receive()));
  expected_values = {20, 20, 200, 30, /* button */
                     20, 50, 200, 30 /* check box */};
  EXPECT_UIA_SAFEARRAY_EQ(rectangles.Get(), expected_values);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderGetEnclosingElement) {
  // Set up ax tree with the following structure:
  //
  // root
  // |
  // paragraph______________________________________________
  // |               |            |       |                |
  // static_text     link         link    search input     pdf_highlight
  // |               |            |       |                |
  // text_node       static_text  ul      text_node        static_text
  //                 |            |                        |
  //                 text_node    li                       text_node
  //                              |
  //                              static_text
  //                              |
  //                              text_node

  ui::AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  ui::AXNodeData paragraph_data;
  paragraph_data.id = 2;
  paragraph_data.role = ax::mojom::Role::kParagraph;
  root_data.child_ids.push_back(paragraph_data.id);

  ui::AXNodeData static_text_data1;
  static_text_data1.id = 3;
  static_text_data1.role = ax::mojom::Role::kStaticText;
  paragraph_data.child_ids.push_back(static_text_data1.id);

  ui::AXNodeData inline_text_data1;
  inline_text_data1.id = 4;
  inline_text_data1.role = ax::mojom::Role::kInlineTextBox;
  static_text_data1.child_ids.push_back(inline_text_data1.id);

  ui::AXNodeData link_data;
  link_data.id = 5;
  link_data.role = ax::mojom::Role::kLink;
  paragraph_data.child_ids.push_back(link_data.id);

  ui::AXNodeData static_text_data2;
  static_text_data2.id = 6;
  static_text_data2.role = ax::mojom::Role::kStaticText;
  link_data.child_ids.push_back(static_text_data2.id);

  ui::AXNodeData inline_text_data2;
  inline_text_data2.id = 7;
  inline_text_data2.role = ax::mojom::Role::kInlineTextBox;
  static_text_data2.child_ids.push_back(inline_text_data2.id);

  ui::AXNodeData link_data2;
  link_data2.id = 8;
  link_data2.role = ax::mojom::Role::kLink;
  paragraph_data.child_ids.push_back(link_data2.id);

  ui::AXNodeData list_data;
  list_data.id = 9;
  list_data.role = ax::mojom::Role::kList;
  link_data2.child_ids.push_back(list_data.id);

  ui::AXNodeData list_item_data;
  list_item_data.id = 10;
  list_item_data.role = ax::mojom::Role::kListItem;
  list_data.child_ids.push_back(list_item_data.id);

  ui::AXNodeData static_text_data3;
  static_text_data3.id = 11;
  static_text_data3.role = ax::mojom::Role::kStaticText;
  list_item_data.child_ids.push_back(static_text_data3.id);

  ui::AXNodeData inline_text_data3;
  inline_text_data3.id = 12;
  inline_text_data3.role = ax::mojom::Role::kInlineTextBox;
  static_text_data3.child_ids.push_back(inline_text_data3.id);

  ui::AXNodeData search_box;
  search_box.id = 13;
  search_box.role = ax::mojom::Role::kSearchBox;
  search_box.AddState(ax::mojom::State::kEditable);
  search_box.AddStringAttribute(ax::mojom::StringAttribute::kHtmlTag, "input");
  search_box.AddStringAttribute(ax::mojom::StringAttribute::kInputType,
                                "search");
  paragraph_data.child_ids.push_back(search_box.id);

  ui::AXNodeData search_text;
  search_text.id = 14;
  search_text.role = ax::mojom::Role::kStaticText;
  search_text.AddState(ax::mojom::State::kEditable);
  search_text.SetName("placeholder");
  search_box.child_ids.push_back(search_text.id);

  ui::AXNodeData pdf_highlight_data;
  pdf_highlight_data.id = 15;
  pdf_highlight_data.role = ax::mojom::Role::kPdfActionableHighlight;
  paragraph_data.child_ids.push_back(pdf_highlight_data.id);

  ui::AXNodeData static_text_data4;
  static_text_data4.id = 16;
  static_text_data4.role = ax::mojom::Role::kStaticText;
  pdf_highlight_data.child_ids.push_back(static_text_data4.id);

  ui::AXNodeData inline_text_data4;
  inline_text_data4.id = 17;
  inline_text_data4.role = ax::mojom::Role::kInlineTextBox;
  static_text_data4.child_ids.push_back(inline_text_data4.id);

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_data.id;
  update.nodes = {root_data,         paragraph_data,    static_text_data1,
                  inline_text_data1, link_data,         static_text_data2,
                  inline_text_data2, link_data2,        list_data,
                  list_item_data,    static_text_data3, inline_text_data3,
                  search_box,        search_text,       pdf_highlight_data,
                  static_text_data4, inline_text_data4};
  Init(update);

  // Set up variables from the tree for testing.
  AXNode* paragraph_node = GetRootAsAXNode()->children()[0];
  AXNode* static_text_node1 = paragraph_node->children()[0];
  AXNode* link_node = paragraph_node->children()[1];
  AXNode* inline_text_node1 = static_text_node1->children()[0];
  AXNode* static_text_node2 = link_node->children()[0];
  AXNode* inline_text_node2 = static_text_node2->children()[0];
  AXNode* link_node2 = paragraph_node->children()[2];
  AXNode* list_node = link_node2->children()[0];
  AXNode* list_item_node = list_node->children()[0];
  AXNode* static_text_node3 = list_item_node->children()[0];
  AXNode* inline_text_node3 = static_text_node3->children()[0];
  AXNode* search_box_node = paragraph_node->children()[3];
  AXNode* search_text_node = search_box_node->children()[0];
  AXNode* pdf_highlight_node = paragraph_node->children()[4];
  AXNode* static_text_node4 = pdf_highlight_node->children()[0];
  AXNode* inline_text_node4 = static_text_node4->children()[0];
  AXPlatformNodeWin* owner =
      static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(paragraph_node));
  ASSERT_NE(owner, nullptr);

  ComPtr<IRawElementProviderSimple> link_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(link_node);
  ComPtr<IRawElementProviderSimple> static_text_node_raw1 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(static_text_node1);
  ComPtr<IRawElementProviderSimple> static_text_node_raw2 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(static_text_node2);
  ComPtr<IRawElementProviderSimple> static_text_node_raw3 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(static_text_node3);
  ComPtr<IRawElementProviderSimple> inline_text_node_raw1 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(inline_text_node1);
  ComPtr<IRawElementProviderSimple> inline_text_node_raw2 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(inline_text_node2);
  ComPtr<IRawElementProviderSimple> inline_text_node_raw3 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(inline_text_node3);
  ComPtr<IRawElementProviderSimple> search_box_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(search_box_node);
  ComPtr<IRawElementProviderSimple> search_text_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(search_text_node);
  ComPtr<IRawElementProviderSimple> pdf_highlight_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(pdf_highlight_node);
  ComPtr<IRawElementProviderSimple> inline_text_node_raw4 =
      QueryInterfaceFromNode<IRawElementProviderSimple>(inline_text_node4);

  // Test GetEnclosingElement for the two leaves text nodes. The enclosing
  // element of the first one should be its static text parent (because inline
  // text boxes shouldn't be exposed) and the enclosing element for the text
  // node that is grandchild of the link node should return the link node.
  // The text node in the link node with a complex subtree should behave
  // normally and return the static text parent.
  ComPtr<ITextProvider> text_provider;
  EXPECT_HRESULT_SUCCEEDED(inline_text_node_raw1->GetPatternProvider(
      UIA_TextPatternId, &text_provider));

  ComPtr<ITextRangeProvider> text_range_provider;
  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  ComPtr<IRawElementProviderSimple> enclosing_element;
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(inline_text_node_raw1.Get(), enclosing_element.Get());

  EXPECT_HRESULT_SUCCEEDED(inline_text_node_raw2->GetPatternProvider(
      UIA_TextPatternId, &text_provider));

  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(link_node_raw.Get(), enclosing_element.Get());

  EXPECT_HRESULT_SUCCEEDED(inline_text_node_raw3->GetPatternProvider(
      UIA_TextPatternId, &text_provider));

  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(inline_text_node_raw3.Get(), enclosing_element.Get());

  // The enclosing element of a text range in the search text should give the
  // search box
  EXPECT_HRESULT_SUCCEEDED(search_text_node_raw->GetPatternProvider(
      UIA_TextPatternId, &text_provider));

  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Character));

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(search_box_node_raw.Get(), enclosing_element.Get());

  // The enclosing element for the text node that is grandchild of the
  // pdf_highlight node should return the pdf_highlight node.
  EXPECT_HRESULT_SUCCEEDED(inline_text_node_raw4->GetPatternProvider(
      UIA_TextPatternId, &text_provider));

  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(pdf_highlight_node_raw.Get(), enclosing_element.Get());
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderGetEnclosingElementRichButton) {
  // Set up ax tree with the following structure:
  //
  // root
  // ++button_1
  // ++++static_text_1
  // ++++++inline_text_1
  // ++button_2
  // ++++heading
  // ++++++statix_text_2
  // ++++++++inline_text_2

  ui::AXNodeData root;
  ui::AXNodeData button_1;
  ui::AXNodeData static_text_1;
  ui::AXNodeData inline_text_1;
  ui::AXNodeData button_2;
  ui::AXNodeData heading;
  ui::AXNodeData static_text_2;
  ui::AXNodeData inline_text_2;

  root.id = 1;
  button_1.id = 2;
  static_text_1.id = 3;
  inline_text_1.id = 4;
  button_2.id = 5;
  heading.id = 6;
  static_text_2.id = 7;
  inline_text_2.id = 8;

  root.role = ax::mojom::Role::kRootWebArea;
  root.child_ids = {button_1.id, button_2.id};

  button_1.role = ax::mojom::Role::kButton;
  button_1.child_ids.push_back(static_text_1.id);

  static_text_1.role = ax::mojom::Role::kStaticText;
  static_text_1.child_ids.push_back(inline_text_1.id);

  inline_text_1.role = ax::mojom::Role::kInlineTextBox;

  button_2.role = ax::mojom::Role::kButton;
  button_2.child_ids.push_back(heading.id);

  heading.role = ax::mojom::Role::kHeading;
  heading.child_ids.push_back(static_text_2.id);

  static_text_2.role = ax::mojom::Role::kStaticText;
  static_text_2.child_ids.push_back(inline_text_2.id);

  inline_text_2.role = ax::mojom::Role::kInlineTextBox;

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root.id;
  update.nodes = {root,     button_1, static_text_1, inline_text_1,
                  button_2, heading,  static_text_2, inline_text_2};
  Init(update);

  // Set up variables from the tree for testing.
  AXNode* button_1_node = GetRootAsAXNode()->children()[0];
  AXNode* static_text_1_node = button_1_node->children()[0];
  AXNode* inline_text_1_node = static_text_1_node->children()[0];
  AXNode* button_2_node = GetRootAsAXNode()->children()[1];
  AXNode* heading_node = button_2_node->children()[0];
  AXNode* static_text_2_node = heading_node->children()[0];
  AXNode* inline_text_2_node = static_text_2_node->children()[0];
  AXPlatformNodeWin* owner =
      static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(button_1_node));
  ASSERT_NE(owner, nullptr);

  ComPtr<IRawElementProviderSimple> button_1_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(button_1_node);
  ComPtr<IRawElementProviderSimple> inline_text_1_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(inline_text_1_node);

  ComPtr<IRawElementProviderSimple> button_2_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(button_2_node);
  ComPtr<IRawElementProviderSimple> static_text_2_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(static_text_2_node);
  ComPtr<IRawElementProviderSimple> inline_text_2_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(inline_text_2_node);

  // 1. The first button should hide its children since it contains a single
  // text node. Thus, calling GetEnclosingElement on a descendant inline text
  // box should return the button itself.
  ComPtr<ITextProvider> text_provider;
  EXPECT_HRESULT_SUCCEEDED(inline_text_1_node_raw->GetPatternProvider(
      UIA_TextPatternId, &text_provider));

  ComPtr<ITextRangeProvider> text_range_provider;
  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  ComPtr<IRawElementProviderSimple> enclosing_element;
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(button_1_node_raw.Get(), enclosing_element.Get());

  // 2. The second button shouldn't hide its children since it doesn't contain a
  // single text node (it contains a heading node). Thus, calling
  // GetEnclosingElement on a descendant inline text box should return the
  // parent node.
  EXPECT_HRESULT_SUCCEEDED(inline_text_2_node_raw->GetPatternProvider(
      UIA_TextPatternId, &text_provider));

  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->GetEnclosingElement(&enclosing_element));
  EXPECT_EQ(button_2_node_raw.Get(), enclosing_element.Get());
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderMoveEndpointByRange) {
  Init(BuildTextDocument({"some text", "more text"}));

  AXNode* root_node = GetRootAsAXNode();
  AXNode* text_node = root_node->children()[0];
  AXNode* more_text_node = root_node->children()[1];
  AXPlatformNodeWin* owner =
      static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(root_node));
  ASSERT_NE(owner, nullptr);

  // Text range for the document, which contains text "some textmore text".
  ComPtr<IRawElementProviderSimple> root_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(root_node);
  ComPtr<ITextProvider> document_provider;
  EXPECT_HRESULT_SUCCEEDED(
      root_node_raw->GetPatternProvider(UIA_TextPatternId, &document_provider));
  ComPtr<ITextRangeProvider> document_text_range_provider;
  ComPtr<AXPlatformNodeTextRangeProviderWin> document_text_range;

  // Text range related to "some text".
  ComPtr<IRawElementProviderSimple> text_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(text_node);
  ComPtr<ITextProvider> text_provider;
  EXPECT_HRESULT_SUCCEEDED(
      text_node_raw->GetPatternProvider(UIA_TextPatternId, &text_provider));
  ComPtr<ITextRangeProvider> text_range_provider;
  ComPtr<AXPlatformNodeTextRangeProviderWin> text_range;

  // Text range related to "more text".
  ComPtr<IRawElementProviderSimple> more_text_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(more_text_node);
  ComPtr<ITextProvider> more_text_provider;
  EXPECT_HRESULT_SUCCEEDED(more_text_node_raw->GetPatternProvider(
      UIA_TextPatternId, &more_text_provider));
  ComPtr<ITextRangeProvider> more_text_range_provider;
  ComPtr<AXPlatformNodeTextRangeProviderWin> more_text_range;

  // Move the start of document text range "some textmore text" to the end of
  // itself.
  // The start of document text range "some textmore text" is at the end of
  // itself.
  //
  // Before:
  // |s                e|
  // "some textmore text"
  // After:
  //                  |s
  //                   e|
  // "some textmore text"

  // Get the textRangeProvider for the document, which contains text
  // "some textmore text".
  EXPECT_HRESULT_SUCCEEDED(
      document_provider->get_DocumentRange(&document_text_range_provider));
  SetOwner(owner, document_text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->MoveEndpointByRange(
      TextPatternRangeEndpoint_Start, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_End));

  document_text_range_provider->QueryInterface(
      IID_PPV_ARGS(&document_text_range));
  EXPECT_EQ(*GetStart(document_text_range.Get()),
            *GetEnd(document_text_range.Get()));

  // Move the end of document text range "some textmore text" to the start of
  // itself.
  // The end of document text range "some textmore text" is at the start of
  // itself.
  //
  // Before:
  // |s                e|
  // "some textmore text"
  // After:
  // |s
  //  e|
  // "some textmore text"

  // Get the textRangeProvider for the document, which contains text
  // "some textmore text".
  EXPECT_HRESULT_SUCCEEDED(
      document_provider->get_DocumentRange(&document_text_range_provider));
  SetOwner(owner, document_text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->MoveEndpointByRange(
      TextPatternRangeEndpoint_Start, document_text_range_provider.Get(),
      TextPatternRangeEndpoint_End));

  document_text_range_provider->QueryInterface(
      IID_PPV_ARGS(&document_text_range));
  EXPECT_EQ(*GetStart(document_text_range.Get()),
            *GetEnd(document_text_range.Get()));

  // Move the start of document text range "some textmore text" to the start
  // of text range "more text". The start of document text range "some
  // textmore text" is at the start of text range "more text". The end of
  // document range does not change.
  //
  // Before:
  // |s                e|
  // "some textmore text"
  // After:
  //          |s       e|
  // "some textmore text"

  // Get the textRangeProvider for the document, which contains text
  // "some textmore text".
  EXPECT_HRESULT_SUCCEEDED(
      document_provider->get_DocumentRange(&document_text_range_provider));
  SetOwner(owner, document_text_range_provider.Get());
  // Get the textRangeProvider for more_text_node which contains "more text".
  EXPECT_HRESULT_SUCCEEDED(
      more_text_provider->get_DocumentRange(&more_text_range_provider));
  SetOwner(owner, more_text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->MoveEndpointByRange(
      TextPatternRangeEndpoint_Start, more_text_range_provider.Get(),
      TextPatternRangeEndpoint_Start));

  document_text_range_provider->QueryInterface(
      IID_PPV_ARGS(&document_text_range));
  more_text_range_provider->QueryInterface(IID_PPV_ARGS(&more_text_range));
  EXPECT_EQ(*GetStart(document_text_range.Get()),
            *GetStart(more_text_range.Get()));

  // Move the end of document text range "some textmore text" to the end of
  // text range "some text".
  // The end of document text range "some textmore text" is at the end of text
  // range "some text". The start of document range does not change.
  //
  // Before:
  // |s                e|
  // "some textmore text"
  // After:
  // |s       e|
  // "some textmore text"

  // Get the textRangeProvider for the document, which contains text
  // "some textmore text".
  EXPECT_HRESULT_SUCCEEDED(
      document_provider->get_DocumentRange(&document_text_range_provider));
  SetOwner(owner, document_text_range_provider.Get());
  // Get the textRangeProvider for text_node which contains "some text".
  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(document_text_range_provider->MoveEndpointByRange(
      TextPatternRangeEndpoint_End, text_range_provider.Get(),
      TextPatternRangeEndpoint_End));

  document_text_range_provider->QueryInterface(
      IID_PPV_ARGS(&document_text_range));
  text_range_provider->QueryInterface(IID_PPV_ARGS(&text_range));
  EXPECT_EQ(*GetEnd(document_text_range.Get()), *GetEnd(text_range.Get()));

  // Move the end of text range "more text" to the start of
  // text range "some text". Since the order of the endpoints being moved
  // (those of "more text") have to be ensured, both endpoints of "more text"
  // is at the start of "some text".
  //
  // Before:
  //          |s       e|
  // "some textmore text"
  // After:
  //  e|
  // |s
  // "some textmore text"

  // Get the textRangeProvider for text_node which contains "some text".
  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, document_text_range_provider.Get());
  // Get the textRangeProvider for more_text_node which contains "more text".
  EXPECT_HRESULT_SUCCEEDED(
      more_text_provider->get_DocumentRange(&more_text_range_provider));
  SetOwner(owner, more_text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(more_text_range_provider->MoveEndpointByRange(
      TextPatternRangeEndpoint_End, text_range_provider.Get(),
      TextPatternRangeEndpoint_Start));

  text_range_provider->QueryInterface(IID_PPV_ARGS(&text_range));
  more_text_range_provider->QueryInterface(IID_PPV_ARGS(&more_text_range));
  EXPECT_EQ(*GetEnd(more_text_range.Get()), *GetStart(text_range.Get()));
  EXPECT_EQ(*GetStart(more_text_range.Get()), *GetStart(text_range.Get()));

  // Move the start of text range "some text" to the end of text range
  // "more text". Since the order of the endpoints being moved (those
  // of "some text") have to be ensured, both endpoints of "some text" is at
  // the end of "more text".
  //
  // Before:
  // |s       e|
  // "some textmore text"
  // After:
  //                  |s
  //                   e|
  // "some textmore text"

  // Get the textRangeProvider for text_node which contains "some text".
  EXPECT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  SetOwner(owner, text_range_provider.Get());
  // Get the textRangeProvider for more_text_node which contains "more text".
  EXPECT_HRESULT_SUCCEEDED(
      more_text_provider->get_DocumentRange(&more_text_range_provider));
  SetOwner(owner, more_text_range_provider.Get());

  EXPECT_HRESULT_SUCCEEDED(text_range_provider->MoveEndpointByRange(
      TextPatternRangeEndpoint_Start, more_text_range_provider.Get(),
      TextPatternRangeEndpoint_End));

  text_range_provider->QueryInterface(IID_PPV_ARGS(&text_range));
  more_text_range_provider->QueryInterface(IID_PPV_ARGS(&more_text_range));
  EXPECT_EQ(*GetStart(text_range.Get()), *GetEnd(more_text_range.Get()));
  EXPECT_EQ(*GetEnd(text_range.Get()), *GetEnd(more_text_range.Get()));
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderGetAttributeValue) {
  ui::AXNodeData text_data;
  text_data.id = 2;
  text_data.role = ax::mojom::Role::kStaticText;
  text_data.AddStringAttribute(ax::mojom::StringAttribute::kFontFamily, "sans");
  text_data.AddFloatAttribute(ax::mojom::FloatAttribute::kFontSize, 16);
  text_data.AddFloatAttribute(ax::mojom::FloatAttribute::kFontWeight, 300);
  text_data.AddIntAttribute(ax::mojom::IntAttribute::kTextOverlineStyle, 1);
  text_data.AddIntAttribute(ax::mojom::IntAttribute::kTextStrikethroughStyle,
                            2);
  text_data.AddIntAttribute(ax::mojom::IntAttribute::kTextUnderlineStyle, 3);
  text_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                            0xFFADBEEFU);
  text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);
  text_data.AddStringAttribute(ax::mojom::StringAttribute::kLanguage, "fr-CA");
  text_data.SetTextDirection(ax::mojom::WritingDirection::kRtl);
  text_data.AddTextStyle(ax::mojom::TextStyle::kItalic);
  text_data.SetTextPosition(ax::mojom::TextPosition::kSubscript);
  text_data.SetRestriction(ax::mojom::Restriction::kReadOnly);
  text_data.SetTextAlign(ax::mojom::TextAlign::kCenter);
  text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kMarkerTypes,
                                {(int)ax::mojom::MarkerType::kGrammar,
                                 (int)ax::mojom::MarkerType::kSpelling});
  text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kMarkerStarts,
                                {0, 5, 0, 14, 19});
  text_data.AddIntListAttribute(ax::mojom::IntListAttribute::kMarkerEnds,
                                {9, 9, 4, 18, 24});
  text_data.SetName("some text and some other text");

  ui::AXNodeData heading_data;
  heading_data.id = 3;
  heading_data.role = ax::mojom::Role::kHeading;
  heading_data.AddIntAttribute(ax::mojom::IntAttribute::kHierarchicalLevel, 6);
  heading_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                               0xFFADBEEFU);
  heading_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);
  heading_data.SetTextDirection(ax::mojom::WritingDirection::kRtl);
  heading_data.SetTextPosition(ax::mojom::TextPosition::kSuperscript);
  heading_data.AddState(ax::mojom::State::kEditable);
  heading_data.child_ids = {4};

  ui::AXNodeData heading_text_data;
  heading_text_data.id = 4;
  heading_text_data.role = ax::mojom::Role::kStaticText;
  heading_text_data.AddState(ax::mojom::State::kInvisible);
  heading_text_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                    0xFFADBEEFU);
  heading_text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor,
                                    0xFFADC0DEU);
  heading_text_data.SetTextDirection(ax::mojom::WritingDirection::kRtl);
  heading_text_data.SetTextPosition(ax::mojom::TextPosition::kSuperscript);
  heading_text_data.AddState(ax::mojom::State::kEditable);
  heading_text_data.SetTextAlign(ax::mojom::TextAlign::kJustify);
  heading_text_data.AddIntListAttribute(
      ax::mojom::IntListAttribute::kMarkerTypes,
      {(int)ax::mojom::MarkerType::kSpelling});
  heading_text_data.AddIntListAttribute(
      ax::mojom::IntListAttribute::kMarkerStarts, {5});
  heading_text_data.AddIntListAttribute(
      ax::mojom::IntListAttribute::kMarkerEnds, {9});
  heading_text_data.SetName("more text");

  ui::AXNodeData mark_data;
  mark_data.id = 5;
  mark_data.role = ax::mojom::Role::kMark;
  mark_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                            0xFFADBEEFU);
  mark_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);
  mark_data.SetTextDirection(ax::mojom::WritingDirection::kRtl);
  mark_data.child_ids = {6};

  ui::AXNodeData mark_text_data;
  mark_text_data.id = 6;
  mark_text_data.role = ax::mojom::Role::kStaticText;
  mark_text_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                 0xFFADBEEFU);
  mark_text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);
  mark_text_data.SetTextDirection(ax::mojom::WritingDirection::kRtl);
  mark_text_data.SetTextAlign(ax::mojom::TextAlign::kNone);
  mark_text_data.SetName("marked text");

  ui::AXNodeData list_data;
  list_data.id = 7;
  list_data.role = ax::mojom::Role::kList;
  list_data.child_ids = {8, 10};
  list_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                            0xFFADBEEFU);
  list_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);

  ui::AXNodeData list_item_data;
  list_item_data.id = 8;
  list_item_data.role = ax::mojom::Role::kListItem;
  list_item_data.child_ids = {9};
  list_item_data.AddIntAttribute(
      ax::mojom::IntAttribute::kListStyle,
      static_cast<int>(ax::mojom::ListStyle::kOther));
  list_item_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                 0xFFADBEEFU);
  list_item_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);

  ui::AXNodeData list_item_text_data;
  list_item_text_data.id = 9;
  list_item_text_data.role = ax::mojom::Role::kStaticText;
  list_item_text_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                      0xFFADBEEFU);
  list_item_text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor,
                                      0xFFADC0DEU);
  list_item_text_data.SetName("list item");

  ui::AXNodeData list_item2_data;
  list_item2_data.id = 10;
  list_item2_data.role = ax::mojom::Role::kListItem;
  list_item2_data.child_ids = {11};
  list_item2_data.AddIntAttribute(
      ax::mojom::IntAttribute::kListStyle,
      static_cast<int>(ax::mojom::ListStyle::kDisc));
  list_item2_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                  0xFFADBEEFU);
  list_item2_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);

  ui::AXNodeData list_item2_text_data;
  list_item2_text_data.id = 11;
  list_item2_text_data.role = ax::mojom::Role::kStaticText;
  list_item2_text_data.AddIntAttribute(
      ax::mojom::IntAttribute::kBackgroundColor, 0xFFADBEEFU);
  list_item2_text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor,
                                       0xFFADC0DEU);
  list_item2_text_data.SetName("list item 2");

  ui::AXNodeData input_text_data;
  input_text_data.id = 12;
  input_text_data.role = ax::mojom::Role::kTextField;
  input_text_data.AddState(ax::mojom::State::kEditable);
  input_text_data.AddIntAttribute(
      ax::mojom::IntAttribute::kNameFrom,
      static_cast<int>(ax::mojom::NameFrom::kPlaceholder));
  input_text_data.AddStringAttribute(ax::mojom::StringAttribute::kPlaceholder,
                                     "placeholder2");
  input_text_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                  0xFFADBEEFU);
  input_text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);
  input_text_data.AddStringAttribute(ax::mojom::StringAttribute::kHtmlTag,
                                     "input");
  input_text_data.AddStringAttribute(ax::mojom::StringAttribute::kInputType,
                                     "text");
  input_text_data.SetName("placeholder");
  input_text_data.child_ids = {13};

  ui::AXNodeData placeholder_text_data;
  placeholder_text_data.id = 13;
  placeholder_text_data.role = ax::mojom::Role::kStaticText;
  placeholder_text_data.AddIntAttribute(
      ax::mojom::IntAttribute::kBackgroundColor, 0xFFADBEEFU);
  placeholder_text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor,
                                        0xFFADC0DEU);
  placeholder_text_data.SetName("placeholder");

  ui::AXNodeData input_text_data2;
  input_text_data2.id = 14;
  input_text_data2.role = ax::mojom::Role::kTextField;
  input_text_data2.AddState(ax::mojom::State::kEditable);
  input_text_data2.SetRestriction(ax::mojom::Restriction::kDisabled);
  input_text_data2.AddStringAttribute(ax::mojom::StringAttribute::kPlaceholder,
                                      "placeholder2");
  input_text_data2.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                   0xFFADBEEFU);
  input_text_data2.AddIntAttribute(ax::mojom::IntAttribute::kColor,
                                   0xFFADC0DEU);
  input_text_data2.AddStringAttribute(ax::mojom::StringAttribute::kHtmlTag,
                                      "input");
  input_text_data2.AddStringAttribute(ax::mojom::StringAttribute::kInputType,
                                      "text");
  input_text_data2.SetName("foo");
  input_text_data2.child_ids = {15};

  ui::AXNodeData placeholder_text_data2;
  placeholder_text_data2.id = 15;
  placeholder_text_data2.role = ax::mojom::Role::kStaticText;
  placeholder_text_data2.AddIntAttribute(
      ax::mojom::IntAttribute::kBackgroundColor, 0xFFADBEEFU);
  placeholder_text_data2.AddIntAttribute(ax::mojom::IntAttribute::kColor,
                                         0xFFADC0DEU);
  placeholder_text_data2.SetName("placeholder2");

  ui::AXNodeData link_data;
  link_data.id = 16;
  link_data.role = ax::mojom::Role::kLink;
  link_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                            0xFFADBEEFU);
  link_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);

  ui::AXNodeData link_text_data;
  link_text_data.id = 17;
  link_text_data.role = ax::mojom::Role::kStaticText;
  link_text_data.AddIntAttribute(ax::mojom::IntAttribute::kBackgroundColor,
                                 0xFFADBEEFU);
  link_text_data.AddIntAttribute(ax::mojom::IntAttribute::kColor, 0xFFADC0DEU);
  link_data.child_ids = {17};

  ui::AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;
  root_data.child_ids = {2, 3, 5, 7, 12, 14, 16};

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_data.id;
  update.nodes.push_back(root_data);
  update.nodes.push_back(text_data);
  update.nodes.push_back(heading_data);
  update.nodes.push_back(heading_text_data);
  update.nodes.push_back(mark_data);
  update.nodes.push_back(mark_text_data);
  update.nodes.push_back(list_data);
  update.nodes.push_back(list_item_data);
  update.nodes.push_back(list_item_text_data);
  update.nodes.push_back(list_item2_data);
  update.nodes.push_back(list_item2_text_data);
  update.nodes.push_back(input_text_data);
  update.nodes.push_back(placeholder_text_data);
  update.nodes.push_back(input_text_data2);
  update.nodes.push_back(placeholder_text_data2);
  update.nodes.push_back(link_data);
  update.nodes.push_back(link_text_data);

  Init(update);

  AXNode* root_node = GetRootAsAXNode();
  AXNode* text_node = root_node->children()[0];
  AXNode* heading_node = root_node->children()[1];
  AXNode* heading_text_node = heading_node->children()[0];
  AXNode* mark_node = root_node->children()[2];
  AXNode* mark_text_node = mark_node->children()[0];
  AXNode* list_node = root_node->children()[3];
  AXNode* list_item_node = list_node->children()[0];
  AXNode* list_item_text_node = list_item_node->children()[0];
  AXNode* list_item2_node = list_node->children()[1];
  AXNode* list_item2_text_node = list_item2_node->children()[0];
  AXNode* input_text_node = root_node->children()[4];
  AXNode* placeholder_text_node = input_text_node->children()[0];
  AXNode* input_text_node2 = root_node->children()[5];
  AXNode* placeholder_text_node2 = input_text_node2->children()[0];
  AXNode* link_node = root_node->children()[6];
  AXNode* link_text_node = link_node->children()[0];

  ComPtr<ITextRangeProvider> document_range_provider;
  GetTextRangeProviderFromTextNode(document_range_provider, root_node);
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, text_node);
  ComPtr<ITextRangeProvider> heading_text_range_provider;
  GetTextRangeProviderFromTextNode(heading_text_range_provider,
                                   heading_text_node);
  ComPtr<ITextRangeProvider> mark_text_range_provider;
  GetTextRangeProviderFromTextNode(mark_text_range_provider, mark_text_node);
  ComPtr<ITextRangeProvider> list_item_text_range_provider;
  GetTextRangeProviderFromTextNode(list_item_text_range_provider,
                                   list_item_text_node);
  ComPtr<ITextRangeProvider> list_item2_text_range_provider;
  GetTextRangeProviderFromTextNode(list_item2_text_range_provider,
                                   list_item2_text_node);

  ComPtr<ITextRangeProvider> placeholder_text_range_provider;
  GetTextRangeProviderFromTextNode(placeholder_text_range_provider,
                                   placeholder_text_node);

  ComPtr<ITextRangeProvider> placeholder_text_range_provider2;
  GetTextRangeProviderFromTextNode(placeholder_text_range_provider2,
                                   placeholder_text_node2);

  ComPtr<ITextRangeProvider> link_text_range_provider;
  GetTextRangeProviderFromTextNode(link_text_range_provider, link_text_node);

  base::win::ScopedVariant expected_variant;

  // SkColor is ARGB, COLORREF is 0BGR
  expected_variant.Set(static_cast<int32_t>(0x00EFBEADU));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider,
                              UIA_BackgroundColorAttributeId, expected_variant);
  // Important: all nodes need to have the kColor and kBackgroundColor attribute
  // set for this test, otherwise the following assert will fail.
  EXPECT_UIA_TEXTATTRIBUTE_EQ(document_range_provider,
                              UIA_BackgroundColorAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(static_cast<int32_t>(BulletStyle::BulletStyle_None));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(list_item_text_range_provider,
                              UIA_BulletStyleAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(
      static_cast<int32_t>(BulletStyle::BulletStyle_FilledRoundBullet));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(list_item2_text_range_provider,
                              UIA_BulletStyleAttributeId, expected_variant);
  expected_variant.Reset();

  {
    base::win::ScopedVariant lang_variant;
    EXPECT_HRESULT_SUCCEEDED(text_range_provider->GetAttributeValue(
        UIA_CultureAttributeId, lang_variant.Receive()));

    EXPECT_EQ(lang_variant.type(), VT_I4);
    const LCID lcid = V_I4(lang_variant.ptr());
    EXPECT_EQ(LANG_FRENCH, PRIMARYLANGID(lcid));
    EXPECT_EQ(SUBLANG_FRENCH_CANADIAN, SUBLANGID(lcid));
    EXPECT_EQ(SORT_DEFAULT, SORTIDFROMLCID(lcid));
  }

  std::wstring font_name = L"sans";
  expected_variant.Set(SysAllocString(font_name.c_str()));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_FontNameAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(12.0);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_FontSizeAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(300);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_FontWeightAttributeId,
                              expected_variant);
  expected_variant.Reset();

  // SkColor is ARGB, COLORREF is 0BGR
  expected_variant.Set(static_cast<int32_t>(0x00DEC0ADU));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider,
                              UIA_ForegroundColorAttributeId, expected_variant);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(document_range_provider,
                              UIA_ForegroundColorAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_IsHiddenAttributeId,
                              expected_variant);
  expected_variant.Reset();

  EXPECT_UIA_TEXTATTRIBUTE_MIXED(document_range_provider,
                                 UIA_IsHiddenAttributeId);

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_IsItalicAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(heading_text_range_provider,
                              UIA_IsItalicAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_IsReadOnlyAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(heading_text_range_provider,
                              UIA_IsReadOnlyAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(placeholder_text_range_provider,
                              UIA_IsReadOnlyAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(placeholder_text_range_provider2,
                              UIA_IsReadOnlyAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(link_text_range_provider,
                              UIA_IsReadOnlyAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(HorizontalTextAlignment_Centered);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider,
                              UIA_HorizontalTextAlignmentAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(HorizontalTextAlignment_Justified);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(heading_text_range_provider,
                              UIA_HorizontalTextAlignmentAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_IsSubscriptAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(heading_text_range_provider,
                              UIA_IsSubscriptAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_IsSuperscriptAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(heading_text_range_provider,
                              UIA_IsSuperscriptAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(TextDecorationLineStyle::TextDecorationLineStyle_Dot);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_OverlineStyleAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(TextDecorationLineStyle::TextDecorationLineStyle_Dash);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(
      text_range_provider, UIA_StrikethroughStyleAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(TextDecorationLineStyle::TextDecorationLineStyle_Single);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider,
                              UIA_UnderlineStyleAttributeId, expected_variant);
  expected_variant.Reset();

  std::wstring style_name;
  expected_variant.Set(SysAllocString(style_name.c_str()));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider, UIA_StyleNameAttributeId,
                              expected_variant);
  expected_variant.Reset();

  expected_variant.Set(static_cast<int32_t>(StyleId_Heading6));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(heading_text_range_provider,
                              UIA_StyleIdAttributeId, expected_variant);
  expected_variant.Reset();

  style_name = L"mark";
  expected_variant.Set(SysAllocString(style_name.c_str()));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(mark_text_range_provider,
                              UIA_StyleNameAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(static_cast<int32_t>(StyleId_NumberedList));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(list_item_text_range_provider,
                              UIA_StyleIdAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(static_cast<int32_t>(StyleId_BulletedList));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(list_item2_text_range_provider,
                              UIA_StyleIdAttributeId, expected_variant);
  expected_variant.Reset();

  expected_variant.Set(
      static_cast<int32_t>(FlowDirections::FlowDirections_RightToLeft));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(
      text_range_provider, UIA_TextFlowDirectionsAttributeId, expected_variant);
  EXPECT_UIA_TEXTATTRIBUTE_MIXED(document_range_provider,
                                 UIA_TextFlowDirectionsAttributeId);
  expected_variant.Reset();

  // Move the start endpoint back and forth one character to force such endpoint
  // to be located at the end of the previous anchor, this shouldn't cause
  // GetAttributeValue to include the previous anchor's attributes.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(mark_text_range_provider,
                                   TextPatternRangeEndpoint_Start,
                                   TextUnit_Character,
                                   /*count*/ -1,
                                   /*expected_text*/ L"tmarked text",
                                   /*expected_count*/ -1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(mark_text_range_provider,
                                   TextPatternRangeEndpoint_Start,
                                   TextUnit_Character,
                                   /*count*/ 1,
                                   /*expected_text*/ L"marked text",
                                   /*expected_count*/ 1);
  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(mark_text_range_provider,
                              UIA_IsSuperscriptAttributeId, expected_variant);
  expected_variant.Reset();

  // Same idea as above, but moving forth and back the end endpoint to force it
  // to be located at the start of the next anchor.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(mark_text_range_provider,
                                   TextPatternRangeEndpoint_End,
                                   TextUnit_Character,
                                   /*count*/ 1,
                                   /*expected_text*/ L"marked textl",
                                   /*expected_count*/ 1);
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(mark_text_range_provider,
                                   TextPatternRangeEndpoint_End,
                                   TextUnit_Character,
                                   /*count*/ -1,
                                   /*expected_text*/ L"marked text",
                                   /*expected_count*/ -1);
  expected_variant.Set(
      static_cast<int32_t>(FlowDirections::FlowDirections_RightToLeft));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(mark_text_range_provider,
                              UIA_TextFlowDirectionsAttributeId,
                              expected_variant);
  expected_variant.Reset();

  {
    // |text_node| has a grammar error on "some text", a highlight for the
    // first word, a spelling error for the second word, a "spelling-error"
    // highlight for the fourth word, and a "grammar-error" highlight for the
    // fifth word. So the range has mixed annotations.
    EXPECT_UIA_TEXTATTRIBUTE_MIXED(text_range_provider,
                                   UIA_AnnotationTypesAttributeId);

    // Testing annotations in range [5,9)
    // start: TextPosition, anchor_id=2, text_offset=5,
    //        annotated_text=some <t>ext and some other text
    // end  : TextPosition, anchor_id=2, text_offset=9,
    //        annotated_text=some text<> and some other text
    AXPlatformNodeWin* owner =
        static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(text_node));
    ComPtr<AXPlatformNodeTextRangeProviderWin> range_with_annotations;
    CreateTextRangeProviderWin(
        range_with_annotations, owner,
        /*start_anchor=*/text_node, /*start_offset=*/5,
        /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
        /*end_anchor=*/text_node, /*end_offset=*/9,
        /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

    base::win::ScopedVariant annotation_types_variant;
    EXPECT_HRESULT_SUCCEEDED(range_with_annotations->GetAttributeValue(
        UIA_AnnotationTypesAttributeId, annotation_types_variant.Receive()));

    EXPECT_EQ(annotation_types_variant.type(), VT_ARRAY | VT_I4);
    std::vector<int> expected_annotations = {AnnotationType_SpellingError,
                                             AnnotationType_GrammarError};
    EXPECT_UIA_SAFEARRAY_EQ(V_ARRAY(annotation_types_variant.ptr()),
                            expected_annotations);
  }

  {
    // Testing annotations in range [0,4)
    // start: TextPosition, anchor_id=2, text_offset=0,
    //        annotated_text=<s>ome text and some other text
    // end  : TextPosition, anchor_id=2, text_offset=4,
    //        annotated_text=some<> text and some other text
    AXPlatformNodeWin* owner =
        static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(text_node));
    ComPtr<AXPlatformNodeTextRangeProviderWin> range_with_annotations;
    CreateTextRangeProviderWin(
        range_with_annotations, owner,
        /*start_anchor=*/text_node, /*start_offset=*/0,
        /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
        /*end_anchor=*/text_node, /*end_offset=*/4,
        /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

    base::win::ScopedVariant annotation_types_variant;
    EXPECT_HRESULT_SUCCEEDED(range_with_annotations->GetAttributeValue(
        UIA_AnnotationTypesAttributeId, annotation_types_variant.Receive()));

    EXPECT_EQ(annotation_types_variant.type(), VT_ARRAY | VT_I4);
    std::vector<int> expected_annotations = {AnnotationType_GrammarError,
                                             AnnotationType_Highlighted};
    EXPECT_UIA_SAFEARRAY_EQ(V_ARRAY(annotation_types_variant.ptr()),
                            expected_annotations);
  }

  {
    // Testing annotations in range [14,18)
    // start: TextPosition, anchor_id=2, text_offset=14,
    //        annotated_text=some text and <s>ome other text
    // end  : TextPosition, anchor_id=2, text_offset=18,
    //        annotated_text=some text and some<> other text
    AXPlatformNodeWin* owner =
        static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(text_node));
    ComPtr<AXPlatformNodeTextRangeProviderWin> range_with_annotations;
    CreateTextRangeProviderWin(
        range_with_annotations, owner,
        /*start_anchor=*/text_node, /*start_offset=*/14,
        /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
        /*end_anchor=*/text_node, /*end_offset=*/18,
        /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

    base::win::ScopedVariant annotation_types_variant;
    EXPECT_HRESULT_SUCCEEDED(range_with_annotations->GetAttributeValue(
        UIA_AnnotationTypesAttributeId, annotation_types_variant.Receive()));

    EXPECT_EQ(annotation_types_variant.type(), VT_ARRAY | VT_I4);
    std::vector<int> expected_annotations = {AnnotationType_SpellingError};
    EXPECT_UIA_SAFEARRAY_EQ(V_ARRAY(annotation_types_variant.ptr()),
                            expected_annotations);
  }

  {
    // Testing annotations in range [19,24)
    // start: TextPosition, anchor_id=2, text_offset=19,
    //        annotated_text=some text and some <o>ther text
    // end  : TextPosition, anchor_id=2, text_offset=24,
    //        annotated_text=some text and some other<> text
    AXPlatformNodeWin* owner =
        static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(text_node));
    ComPtr<AXPlatformNodeTextRangeProviderWin> range_with_annotations;
    CreateTextRangeProviderWin(
        range_with_annotations, owner,
        /*start_anchor=*/text_node, /*start_offset=*/19,
        /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
        /*end_anchor=*/text_node, /*end_offset=*/24,
        /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

    base::win::ScopedVariant annotation_types_variant;
    EXPECT_HRESULT_SUCCEEDED(range_with_annotations->GetAttributeValue(
        UIA_AnnotationTypesAttributeId, annotation_types_variant.Receive()));

    EXPECT_EQ(annotation_types_variant.type(), VT_ARRAY | VT_I4);
    std::vector<int> expected_annotations = {AnnotationType_GrammarError};
    EXPECT_UIA_SAFEARRAY_EQ(V_ARRAY(annotation_types_variant.ptr()),
                            expected_annotations);
  }

  {
    // |heading_text_node| has a spelling error for one word, and no
    // annotations for the remaining text, so the range has mixed annotations.
    EXPECT_UIA_TEXTATTRIBUTE_MIXED(heading_text_range_provider,
                                   UIA_AnnotationTypesAttributeId);

    // start: TextPosition, anchor_id=4, text_offset=5,
    //        annotated_text=more <t>ext
    // end  : TextPosition, anchor_id=4, text_offset=9,
    //        annotated_text=more text<>
    AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
        AXPlatformNodeFromNode(heading_text_node));
    ComPtr<AXPlatformNodeTextRangeProviderWin> range_with_annotations;
    CreateTextRangeProviderWin(
        range_with_annotations, owner,
        /*start_anchor=*/heading_text_node, /*start_offset=*/5,
        /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
        /*end_anchor=*/heading_text_node, /*end_offset=*/9,
        /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

    base::win::ScopedVariant annotation_types_variant;
    EXPECT_HRESULT_SUCCEEDED(range_with_annotations->GetAttributeValue(
        UIA_AnnotationTypesAttributeId, annotation_types_variant.Receive()));

    std::vector<int> expected_annotations = {AnnotationType_SpellingError};
    EXPECT_UIA_SAFEARRAY_EQ(V_ARRAY(annotation_types_variant.ptr()),
                            expected_annotations);
  }

  {
    base::win::ScopedVariant empty_variant;
    EXPECT_UIA_TEXTATTRIBUTE_EQ(mark_text_range_provider,
                                UIA_AnnotationTypesAttributeId, empty_variant);
  }
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderGetAttributeValueAnnotationObjects) {
  // rootWebArea id=1
  // ++mark id=2 detailsIds=comment1 comment2 highlighted
  // ++++staticText id=3 name="some text"
  // ++comment id=4 name="comment 1"
  // ++++staticText id=5 name="comment 1"
  // ++comment id=6 name="comment 2"
  // ++++staticText id=7 name="comment 2"
  // ++mark id=8 name="highlighted"
  // ++++staticText id=9 name="highlighted"

  AXNodeData root;
  AXNodeData annotation_target;
  AXNodeData some_text;
  AXNodeData comment1;
  AXNodeData comment1_text;
  AXNodeData comment2;
  AXNodeData comment2_text;
  AXNodeData highlighted;
  AXNodeData highlighted_text;

  root.id = 1;
  annotation_target.id = 2;
  some_text.id = 3;
  comment1.id = 4;
  comment1_text.id = 5;
  comment2.id = 6;
  comment2_text.id = 7;
  highlighted.id = 8;
  highlighted_text.id = 9;

  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("root");
  root.child_ids = {annotation_target.id, comment1.id, comment2.id,
                    highlighted.id};

  annotation_target.role = ax::mojom::Role::kMark;
  annotation_target.child_ids = {some_text.id};
  annotation_target.AddIntListAttribute(
      ax::mojom::IntListAttribute::kDetailsIds,
      {comment1.id, comment2.id, highlighted.id});

  some_text.role = ax::mojom::Role::kStaticText;
  some_text.SetName("some text");

  comment1.role = ax::mojom::Role::kComment;
  comment1.SetName("comment 1");
  comment1.child_ids = {comment1_text.id};

  comment1_text.role = ax::mojom::Role::kStaticText;
  comment1_text.SetName("comment 1");

  comment2.role = ax::mojom::Role::kComment;
  comment2.SetName("comment 2");
  comment2.child_ids = {comment2_text.id};

  comment2_text.role = ax::mojom::Role::kStaticText;
  comment2_text.SetName("comment 2");

  highlighted.role = ax::mojom::Role::kMark;
  highlighted.SetName("highlighted");
  highlighted.child_ids = {highlighted_text.id};

  highlighted_text.role = ax::mojom::Role::kStaticText;
  highlighted_text.SetName("highlighted");

  ui::AXTreeUpdate update;
  update.has_tree_data = true;
  update.root_id = root.id;
  update.nodes = {root,          annotation_target, some_text,
                  comment1,      comment1_text,     comment2,
                  comment2_text, highlighted,       highlighted_text};
  update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();

  Init(update);

  AXNode* root_node = GetRootAsAXNode();
  AXNode* annotation_target_node = root_node->children()[0];
  AXNode* comment1_node = root_node->children()[1];
  AXNode* comment2_node = root_node->children()[2];
  AXNode* highlighted_node = root_node->children()[3];

  ComPtr<AXPlatformNodeTextRangeProviderWin> some_text_range_provider;

  // Create a text range encapsulates |annotation_target_node| with content
  // "some text".
  // start: TextPosition, anchor_id=2, text_offset=0, annotated_text=<s>ome text
  // end  : TextPosition, anchor_id=2, text_offset=9, annotated_text=some text<>
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(annotation_target_node));
  CreateTextRangeProviderWin(
      some_text_range_provider, owner,
      /*start_anchor=*/annotation_target_node, /*start_offset=*/0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/annotation_target_node, /*end_offset=*/9,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, some_text_range_provider.Get());
  EXPECT_UIA_TEXTRANGE_EQ(some_text_range_provider, L"some text");

  ComPtr<IRawElementProviderSimple> comment1_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple>(comment1_node);
  ASSERT_NE(nullptr, comment1_provider.Get());
  ComPtr<IRawElementProviderSimple> comment2_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple>(comment2_node);
  ASSERT_NE(nullptr, comment2_provider.Get());
  ComPtr<IRawElementProviderSimple> highlighted_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple>(highlighted_node);
  ASSERT_NE(nullptr, highlighted_provider.Get());

  ComPtr<IAnnotationProvider> annotation_provider;
  int annotation_type;

  // Validate |comment1_node| with Role::kComment supports IAnnotationProvider.
  EXPECT_HRESULT_SUCCEEDED(comment1_provider->GetPatternProvider(
      UIA_AnnotationPatternId, &annotation_provider));
  ASSERT_NE(nullptr, annotation_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      annotation_provider->get_AnnotationTypeId(&annotation_type));
  EXPECT_EQ(AnnotationType_Comment, annotation_type);
  annotation_provider.Reset();

  // Validate |comment2_node| with Role::kComment supports IAnnotationProvider.
  EXPECT_HRESULT_SUCCEEDED(comment2_provider->GetPatternProvider(
      UIA_AnnotationPatternId, &annotation_provider));
  ASSERT_NE(nullptr, annotation_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      annotation_provider->get_AnnotationTypeId(&annotation_type));
  EXPECT_EQ(AnnotationType_Comment, annotation_type);
  annotation_provider.Reset();

  // Validate |highlighted_node| with Role::kMark supports
  // IAnnotationProvider.
  EXPECT_HRESULT_SUCCEEDED(highlighted_provider->GetPatternProvider(
      UIA_AnnotationPatternId, &annotation_provider));
  ASSERT_NE(nullptr, annotation_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      annotation_provider->get_AnnotationTypeId(&annotation_type));
  EXPECT_EQ(AnnotationType_Highlighted, annotation_type);
  annotation_provider.Reset();

  base::win::ScopedVariant annotation_objects_variant;
  EXPECT_HRESULT_SUCCEEDED(some_text_range_provider->GetAttributeValue(
      UIA_AnnotationObjectsAttributeId, annotation_objects_variant.Receive()));
  EXPECT_EQ(VT_UNKNOWN | VT_ARRAY, annotation_objects_variant.type());

  std::vector<std::wstring> expected_names = {L"comment 1", L"comment 2",
                                              L"highlighted"};
  EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(V_ARRAY(annotation_objects_variant.ptr()),
                                   UIA_NamePropertyId, expected_names);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderGetAttributeValueAnnotationObjectsMixed) {
  // rootWebArea id=1
  // ++mark id=2 detailsIds=comment
  // ++++staticText id=3 name="some text"
  // ++staticText id=4 name="read only" restriction=readOnly
  // ++comment id=5 name="comment 1"
  // ++++staticText id=6 name="comment 1"

  AXNodeData root;
  AXNodeData highlighted;
  AXNodeData some_text;
  AXNodeData readonly_text;
  AXNodeData comment1;
  AXNodeData comment1_text;

  root.id = 1;
  highlighted.id = 2;
  some_text.id = 3;
  readonly_text.id = 4;
  comment1.id = 5;
  comment1_text.id = 6;

  root.role = ax::mojom::Role::kRootWebArea;
  root.SetName("root");
  root.child_ids = {highlighted.id, readonly_text.id, comment1.id};

  highlighted.role = ax::mojom::Role::kMark;
  highlighted.child_ids = {some_text.id};
  highlighted.AddIntListAttribute(ax::mojom::IntListAttribute::kDetailsIds,
                                  {comment1.id});

  some_text.role = ax::mojom::Role::kStaticText;
  some_text.SetName("some text");

  readonly_text.role = ax::mojom::Role::kStaticText;
  readonly_text.SetRestriction(ax::mojom::Restriction::kReadOnly);
  readonly_text.SetName("read only");

  comment1.role = ax::mojom::Role::kComment;
  comment1.SetName("comment 1");
  comment1.child_ids = {comment1_text.id};

  comment1_text.role = ax::mojom::Role::kStaticText;
  comment1_text.SetName("comment 1");

  ui::AXTreeUpdate update;
  update.has_tree_data = true;
  update.root_id = root.id;
  update.nodes = {root,          highlighted, some_text,
                  readonly_text, comment1,    comment1_text};
  update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();

  Init(update);

  AXNode* root_node = GetRootAsAXNode();
  AXNode* highlighted_node = root_node->children()[0];
  AXNode* some_text_node = highlighted_node->children()[0];
  AXNode* readonly_text_node = root_node->children()[1];
  AXNode* comment1_node = root_node->children()[2];

  // Create a text range encapsulates |highlighted_node| with content
  // "some text".
  // start: TextPosition, anchor_id=2, text_offset=0, annotated_text=<s>ome text
  // end  : TextPosition, anchor_id=2, text_offset=9, annotated_text=some text<>
  ComPtr<AXPlatformNodeTextRangeProviderWin> some_text_range_provider;
  AXPlatformNodeWin* owner =
      static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(highlighted_node));
  CreateTextRangeProviderWin(
      some_text_range_provider, owner,
      /*start_anchor=*/highlighted_node, /*start_offset=*/0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/highlighted_node, /*end_offset=*/9,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, some_text_range_provider.Get());
  EXPECT_UIA_TEXTRANGE_EQ(some_text_range_provider, L"some text");

  ComPtr<ITextRangeProvider> readonly_text_range_provider;
  GetTextRangeProviderFromTextNode(readonly_text_range_provider,
                                   readonly_text_node);
  ASSERT_NE(nullptr, readonly_text_range_provider.Get());

  ComPtr<IRawElementProviderSimple> comment1_provider =
      QueryInterfaceFromNode<IRawElementProviderSimple>(comment1_node);
  ASSERT_NE(nullptr, comment1_provider.Get());

  ComPtr<IAnnotationProvider> annotation_provider;
  int annotation_type;
  base::win::ScopedVariant expected_variant;

  // Validate |comment1_node| with Role::kComment supports IAnnotationProvider.
  EXPECT_HRESULT_SUCCEEDED(comment1_provider->GetPatternProvider(
      UIA_AnnotationPatternId, &annotation_provider));
  ASSERT_NE(nullptr, annotation_provider.Get());
  EXPECT_HRESULT_SUCCEEDED(
      annotation_provider->get_AnnotationTypeId(&annotation_type));
  EXPECT_EQ(AnnotationType_Comment, annotation_type);
  annotation_provider.Reset();

  // Validate text range "some text" supports AnnotationObjectsAttribute.
  EXPECT_HRESULT_SUCCEEDED(some_text_range_provider->GetAttributeValue(
      UIA_AnnotationObjectsAttributeId, expected_variant.Receive()));
  EXPECT_EQ(VT_UNKNOWN | VT_ARRAY, expected_variant.type());

  std::vector<std::wstring> expected_names = {L"comment 1"};
  EXPECT_UIA_ELEMENT_ARRAY_BSTR_EQ(V_ARRAY(expected_variant.ptr()),
                                   UIA_NamePropertyId, expected_names);
  expected_variant.Reset();

  // Validate text range "read only" supports IsReadOnlyAttribute.
  // Use IsReadOnly on text range "read only" as a second property in order to
  // test the "mixed" property in the following section.
  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(readonly_text_range_provider,
                              UIA_IsReadOnlyAttributeId, expected_variant);

  // Validate text range "some textread only" returns mixed attribute.
  // start: TextPosition, anchor_id=2, text_offset=0, annotated_text=<s>ome text
  // end  : TextPosition, anchor_id=3, text_offset=9, annotated_text=read only<>
  ComPtr<AXPlatformNodeTextRangeProviderWin> mixed_text_range_provider;
  CreateTextRangeProviderWin(
      mixed_text_range_provider, owner,
      /*start_anchor=*/some_text_node, /*start_offset=*/0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/readonly_text_node, /*end_offset=*/9,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(mixed_text_range_provider, L"some textread only");
  EXPECT_UIA_TEXTATTRIBUTE_MIXED(mixed_text_range_provider,
                                 UIA_AnnotationObjectsAttributeId);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderGetAttributeValueNotSupported) {
  ui::AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  ui::AXNodeData text_data_first;
  text_data_first.id = 2;
  text_data_first.role = ax::mojom::Role::kStaticText;
  text_data_first.SetName("first");
  root_data.child_ids.push_back(text_data_first.id);

  ui::AXNodeData text_data_second;
  text_data_second.id = 3;
  text_data_second.role = ax::mojom::Role::kStaticText;
  text_data_second.SetName("second");
  root_data.child_ids.push_back(text_data_second.id);

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_data.id;
  update.nodes.push_back(root_data);
  update.nodes.push_back(text_data_first);
  update.nodes.push_back(text_data_second);

  Init(update);

  ComPtr<ITextRangeProvider> document_range_provider;
  GetTextRangeProviderFromTextNode(document_range_provider, GetRootAsAXNode());

  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_AfterParagraphSpacingAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_AnimationStyleAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_BeforeParagraphSpacingAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_CapStyleAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_CaretBidiModeAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_CaretPositionAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_IndentationFirstLineAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_IndentationLeadingAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_IndentationTrailingAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_IsActiveAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_LineSpacingAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_LinkAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_MarginBottomAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_MarginLeadingAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_MarginTopAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_MarginTrailingAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_OutlineStylesAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_OverlineColorAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_SelectionActiveEndAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_StrikethroughColorAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_TabsAttributeId);
  EXPECT_UIA_TEXTATTRIBUTE_NOTSUPPORTED(document_range_provider,
                                        UIA_UnderlineColorAttributeId);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderGetAttributeValueWithAncestorTextPosition) {
  ui::AXTreeUpdate initial_state;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  initial_state.tree_data.tree_id = tree_id;
  initial_state.has_tree_data = true;
  initial_state.root_id = 1;
  initial_state.nodes.resize(5);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2};
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids = {3};
  initial_state.nodes[1].role = ax::mojom::Role::kGenericContainer;
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].child_ids = {4, 5};
  initial_state.nodes[2].role = ax::mojom::Role::kGenericContainer;
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[3].SetName("some text");
  initial_state.nodes[3].AddIntAttribute(
      ax::mojom::IntAttribute::kBackgroundColor, 0xFFADBEEFU);
  initial_state.nodes[4].id = 5;
  initial_state.nodes[4].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[4].SetName("more text");
  initial_state.nodes[4].AddIntAttribute(
      ax::mojom::IntAttribute::kBackgroundColor, 0xFFADBEEFU);

  Init(initial_state);
  const AXTree* tree = GetTree();
  const AXNode* some_text_node = tree->GetFromId(4);
  const AXNode* more_text_node = tree->GetFromId(5);

  // Making |owner| AXID:2 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire subtree, and not only AXID:3 for example.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 2)));

  // start: TextPosition, anchor_id=4, text_offset=0, annotated_text=<s>ome text
  // end  : TextPosition, anchor_id=5, text_offset=8,
  //        annotated_text=more tex<t>
  ComPtr<AXPlatformNodeTextRangeProviderWin> text_range_provider_win;
  CreateTextRangeProviderWin(
      text_range_provider_win, owner,
      /*start_anchor=*/some_text_node, /*start_offset=*/0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/more_text_node, /*end_offset=*/8,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  ASSERT_TRUE(GetStart(text_range_provider_win.Get())->IsTextPosition());
  ASSERT_EQ(4, GetStart(text_range_provider_win.Get())->anchor_id());
  ASSERT_EQ(0, GetStart(text_range_provider_win.Get())->text_offset());
  ASSERT_TRUE(GetEnd(text_range_provider_win.Get())->IsTextPosition());
  ASSERT_EQ(5, GetEnd(text_range_provider_win.Get())->anchor_id());
  ASSERT_EQ(8, GetEnd(text_range_provider_win.Get())->text_offset());

  base::win::ScopedVariant expected_variant;
  // SkColor is ARGB, COLORREF is 0BGR
  expected_variant.Set(static_cast<int32_t>(0x00EFBEADU));
  EXPECT_UIA_TEXTATTRIBUTE_EQ(text_range_provider_win,
                              UIA_BackgroundColorAttributeId, expected_variant);
}

TEST_F(AXPlatformNodeTextRangeProviderTest, TestITextRangeProviderSelect) {
  Init(BuildTextDocument({"some text", "more text2"}));
  AXNode* root_node = GetRootAsAXNode();

  // Text range for the document, which contains text "some textmore text2".
  ComPtr<IRawElementProviderSimple> root_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(root_node);
  ComPtr<ITextProvider> document_provider;
  ComPtr<ITextRangeProvider> document_text_range_provider;
  ComPtr<AXPlatformNodeTextRangeProviderWin> document_text_range;
  EXPECT_HRESULT_SUCCEEDED(
      root_node_raw->GetPatternProvider(UIA_TextPatternId, &document_provider));
  EXPECT_HRESULT_SUCCEEDED(
      document_provider->get_DocumentRange(&document_text_range_provider));
  document_text_range_provider->QueryInterface(
      IID_PPV_ARGS(&document_text_range));
  AXPlatformNodeWin* owner_platform =
      static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(root_node));
  ASSERT_NE(owner_platform, nullptr);
  SetOwner(owner_platform, document_text_range_provider.Get());

  // Text range related to "some text".
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   root_node->children()[0]);
  ComPtr<AXPlatformNodeTextRangeProviderWin> text_range;
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider->QueryInterface(IID_PPV_ARGS(&text_range)));

  // Text range related to "more text2".
  ComPtr<ITextRangeProvider> more_text_range_provider;
  GetTextRangeProviderFromTextNode(more_text_range_provider,
                                   root_node->children()[1]);
  SetOwner(owner_platform, more_text_range_provider.Get());
  ComPtr<AXPlatformNodeTextRangeProviderWin> more_text_range;
  more_text_range_provider->QueryInterface(IID_PPV_ARGS(&more_text_range));

  AXPlatformNodeDelegate* delegate =
      GetOwner(document_text_range.Get())->GetDelegate();

  ComPtr<ITextRangeProvider> selected_text_range_provider;
  base::win::ScopedSafearray selection;
  LONG index = 0;
  LONG ubound;
  LONG lbound;

  // Text range "some text" performs select.
  {
    text_range_provider->Select();

    // Verify selection.
    AXTree::Selection unignored_selection = delegate->GetUnignoredSelection();
    EXPECT_EQ(3, unignored_selection.anchor_object_id);
    EXPECT_EQ(3, unignored_selection.focus_object_id);
    EXPECT_EQ(0, unignored_selection.anchor_offset);
    EXPECT_EQ(9, unignored_selection.focus_offset);

    // Verify the content of the selection.
    document_provider->GetSelection(selection.Receive());
    ASSERT_NE(nullptr, selection.Get());

    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetUBound(selection.Get(), 1, &ubound));
    EXPECT_EQ(0, ubound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetLBound(selection.Get(), 1, &lbound));
    EXPECT_EQ(0, lbound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetElement(
        selection.Get(), &index,
        static_cast<void**>(&selected_text_range_provider)));
    SetOwner(owner_platform, selected_text_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(selected_text_range_provider, L"some text");

    selected_text_range_provider.Reset();
    selection.Reset();
  }

  // Text range "more text2" performs select.
  {
    more_text_range_provider->Select();

    // Verify selection
    AXTree::Selection unignored_selection = delegate->GetUnignoredSelection();
    EXPECT_EQ(5, unignored_selection.anchor_object_id);
    EXPECT_EQ(5, unignored_selection.focus_object_id);
    EXPECT_EQ(0, unignored_selection.anchor_offset);
    EXPECT_EQ(10, unignored_selection.focus_offset);

    // Verify the content of the selection.
    document_provider->GetSelection(selection.Receive());
    ASSERT_NE(nullptr, selection.Get());

    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetUBound(selection.Get(), 1, &ubound));
    EXPECT_EQ(0, ubound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetLBound(selection.Get(), 1, &lbound));
    EXPECT_EQ(0, lbound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetElement(
        selection.Get(), &index,
        static_cast<void**>(&selected_text_range_provider)));
    SetOwner(owner_platform, selected_text_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(selected_text_range_provider, L"more text2");

    selected_text_range_provider.Reset();
    selection.Reset();
  }

  // Document text range "some textmore text2" performs select.
  {
    document_text_range_provider->Select();

    // Verify selection.
    AXTree::Selection unignored_selection = delegate->GetUnignoredSelection();
    EXPECT_EQ(3, unignored_selection.anchor_object_id);
    EXPECT_EQ(5, unignored_selection.focus_object_id);
    EXPECT_EQ(0, unignored_selection.anchor_offset);
    EXPECT_EQ(10, unignored_selection.focus_offset);

    // Verify the content of the selection.
    document_provider->GetSelection(selection.Receive());
    ASSERT_NE(nullptr, selection.Get());

    document_provider->GetSelection(selection.Receive());
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetUBound(selection.Get(), 1, &ubound));
    EXPECT_EQ(0, ubound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetLBound(selection.Get(), 1, &lbound));
    EXPECT_EQ(0, lbound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetElement(
        selection.Get(), &index,
        static_cast<void**>(&selected_text_range_provider)));
    SetOwner(owner_platform, selected_text_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(selected_text_range_provider,
                            L"some textmore text2");
  }

  // A degenerate text range performs select.
  {
    // Move the endpoint of text range so it becomes degenerate, then select.
    text_range_provider->MoveEndpointByRange(TextPatternRangeEndpoint_Start,
                                             text_range_provider.Get(),
                                             TextPatternRangeEndpoint_End);
    text_range_provider->Select();

    // Verify selection.
    AXTree::Selection unignored_selection = delegate->GetUnignoredSelection();
    EXPECT_EQ(3, unignored_selection.anchor_object_id);
    EXPECT_EQ(3, unignored_selection.focus_object_id);
    EXPECT_EQ(9, unignored_selection.anchor_offset);
    EXPECT_EQ(9, unignored_selection.focus_offset);

    // Verify selection on degenerate range.
    document_provider->GetSelection(selection.Receive());
    ASSERT_NE(nullptr, selection.Get());

    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetUBound(selection.Get(), 1, &ubound));
    EXPECT_EQ(0, ubound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetLBound(selection.Get(), 1, &lbound));
    EXPECT_EQ(0, lbound);
    EXPECT_HRESULT_SUCCEEDED(SafeArrayGetElement(
        selection.Get(), &index,
        static_cast<void**>(&selected_text_range_provider)));
    SetOwner(owner_platform, selected_text_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(selected_text_range_provider, L"");

    selected_text_range_provider.Reset();
    selection.Reset();
  }
}

// TODO(crbug.com/1124051): Remove this test once this crbug is fixed.
TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderSelectListMarker) {
  ui::AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  ui::AXNodeData list_data;
  list_data.id = 2;
  list_data.role = ax::mojom::Role::kList;
  root_data.child_ids.push_back(list_data.id);

  ui::AXNodeData list_item_data;
  list_item_data.id = 3;
  list_item_data.role = ax::mojom::Role::kListItem;
  list_data.child_ids.push_back(list_item_data.id);

  ui::AXNodeData list_marker;
  list_marker.id = 4;
  list_marker.role = ax::mojom::Role::kListMarker;
  list_item_data.child_ids.push_back(list_marker.id);

  ui::AXNodeData static_text_data;
  static_text_data.id = 5;
  static_text_data.role = ax::mojom::Role::kStaticText;
  static_text_data.SetName("1. ");
  list_marker.child_ids.push_back(static_text_data.id);

  ui::AXNodeData list_item_text_data;
  list_item_text_data.id = 6;
  list_item_text_data.role = ax::mojom::Role::kStaticText;
  list_item_text_data.SetName("First Item");
  list_item_data.child_ids.push_back(list_item_text_data.id);

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_data.id;
  update.nodes = {root_data,   list_data,        list_item_data,
                  list_marker, static_text_data, list_item_text_data};
  Init(update);
  AXNode* root_node = GetRootAsAXNode();

  // Text range related to "1. ".
  AXNode* list_node = root_node->children()[0];
  AXNode* list_item_node = list_node->children()[0];
  AXNode* list_marker_node = list_item_node->children()[0];
  ComPtr<ITextRangeProvider> list_marker_text_range_provider;
  GetTextRangeProviderFromTextNode(list_marker_text_range_provider,
                                   list_marker_node->children()[0]);

  // A list marker text range performs select.
  EXPECT_HRESULT_SUCCEEDED(list_marker_text_range_provider->Select());

  // Verify selection was not performed on list marker range.
  base::win::ScopedSafearray selection;
  ComPtr<IRawElementProviderSimple> root_node_raw =
      QueryInterfaceFromNode<IRawElementProviderSimple>(root_node);
  ComPtr<ITextProvider> document_provider;
  EXPECT_HRESULT_SUCCEEDED(
      root_node_raw->GetPatternProvider(UIA_TextPatternId, &document_provider));
  EXPECT_HRESULT_SUCCEEDED(
      document_provider->GetSelection(selection.Receive()));
  ASSERT_EQ(nullptr, selection.Get());
  selection.Reset();
}

TEST_F(AXPlatformNodeTextRangeProviderTest, TestITextRangeProviderFindText) {
  // Initialize the ICU data from the icudtl.dat file, if it exists.
  wchar_t buffer[MAX_PATH];
  GetModuleFileName(nullptr, buffer, MAX_PATH);
  std::filesystem::path exec_path(buffer);
  exec_path.remove_filename();
  exec_path.append("icudtl.dat");
  const std::string icudtl_path = exec_path.string();
  if (std::filesystem::exists(icudtl_path)) {
    fml::icu::InitializeICU(icudtl_path);
  }

  // \xC3\xA9 are the UTF8 bytes for codepoint 0xE9 - accented lowercase e.
  Init(BuildTextDocument({"some text", "more text", "resum\xC3\xA9"},
                         false /* build_word_boundaries_offsets */,
                         true /* place_text_on_one_line */));

  AXNode* root_node = GetRootAsAXNode();
  AXPlatformNodeWin* owner =
      static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(root_node));
  ASSERT_NE(owner, nullptr);
  ComPtr<ITextRangeProvider> range;

  // Test Leaf kStaticText search.
  GetTextRangeProviderFromTextNode(range, root_node->children()[0]);
  EXPECT_UIA_FIND_TEXT(range, L"some text", false, owner);
  EXPECT_UIA_FIND_TEXT(range, L"SoMe TeXt", true, owner);
  GetTextRangeProviderFromTextNode(range, root_node->children()[1]);
  EXPECT_UIA_FIND_TEXT(range, L"more", false, owner);
  EXPECT_UIA_FIND_TEXT(range, L"MoRe", true, owner);

  // Test searching for leaf content from ancestor.
  GetTextRangeProviderFromTextNode(range, root_node);
  EXPECT_UIA_FIND_TEXT(range, L"some text", false, owner);
  EXPECT_UIA_FIND_TEXT(range, L"SoMe TeXt", true, owner);
  EXPECT_UIA_FIND_TEXT(range, L"more text", false, owner);
  EXPECT_UIA_FIND_TEXT(range, L"MoRe TeXt", true, owner);
  EXPECT_UIA_FIND_TEXT(range, L"more", false, owner);
  // Accented lowercase e.
  EXPECT_UIA_FIND_TEXT(range, L"resum\xE9", false, owner);
  // Accented uppercase +e.
  EXPECT_UIA_FIND_TEXT(range, L"resum\xC9", true, owner);
  EXPECT_UIA_FIND_TEXT(range, L"resume", true, owner);
  EXPECT_UIA_FIND_TEXT(range, L"resumE", true, owner);
  // Test finding text that crosses a node boundary.
  EXPECT_UIA_FIND_TEXT(range, L"textmore", false, owner);
  // Test no match.
  EXPECT_UIA_FIND_TEXT_NO_MATCH(range, L"no match", false, owner);
  EXPECT_UIA_FIND_TEXT_NO_MATCH(range, L"resume", false, owner);

  // Test if range returned is in expected anchor node.
  GetTextRangeProviderFromTextNode(range, root_node->children()[1]);
  base::win::ScopedBstr find_string(L"more text");
  Microsoft::WRL::ComPtr<ITextRangeProvider> text_range_provider_found;
  EXPECT_HRESULT_SUCCEEDED(range->FindText(find_string.Get(), false, false,
                                           &text_range_provider_found));
  Microsoft::WRL::ComPtr<AXPlatformNodeTextRangeProviderWin>
      text_range_provider_win;
  text_range_provider_found->QueryInterface(
      IID_PPV_ARGS(&text_range_provider_win));
  ASSERT_TRUE(GetStart(text_range_provider_win.Get())->IsTextPosition());
  EXPECT_EQ(5, GetStart(text_range_provider_win.Get())->anchor_id());
  EXPECT_EQ(0, GetStart(text_range_provider_win.Get())->text_offset());
  ASSERT_TRUE(GetEnd(text_range_provider_win.Get())->IsTextPosition());
  EXPECT_EQ(5, GetEnd(text_range_provider_win.Get())->anchor_id());
  EXPECT_EQ(9, GetEnd(text_range_provider_win.Get())->text_offset());
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       FindTextWithEmbeddedObjectCharacter) {
  // ++1 kRootWebArea
  // ++++2 kList
  // ++++++3 kListItem
  // ++++++++4 kStaticText
  // ++++++++++5 kInlineTextBox
  // ++++++6 kListItem
  // ++++++++7 kStaticText
  // ++++++++++8 kInlineTextBox
  ui::AXNodeData root_1;
  ui::AXNodeData list_2;
  ui::AXNodeData list_item_3;
  ui::AXNodeData static_text_4;
  ui::AXNodeData inline_box_5;
  ui::AXNodeData list_item_6;
  ui::AXNodeData static_text_7;
  ui::AXNodeData inline_box_8;

  root_1.id = 1;
  list_2.id = 2;
  list_item_3.id = 3;
  static_text_4.id = 4;
  inline_box_5.id = 5;
  list_item_6.id = 6;
  static_text_7.id = 7;
  inline_box_8.id = 8;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {list_2.id};

  list_2.role = ax::mojom::Role::kList;
  list_2.child_ids = {list_item_3.id, list_item_6.id};

  list_item_3.role = ax::mojom::Role::kListItem;
  list_item_3.child_ids = {static_text_4.id};

  static_text_4.role = ax::mojom::Role::kStaticText;
  static_text_4.SetName("foo");
  static_text_4.child_ids = {inline_box_5.id};

  inline_box_5.role = ax::mojom::Role::kInlineTextBox;
  inline_box_5.SetName("foo");

  list_item_6.role = ax::mojom::Role::kListItem;
  list_item_6.child_ids = {static_text_7.id};

  static_text_7.role = ax::mojom::Role::kStaticText;
  static_text_7.child_ids = {inline_box_8.id};
  static_text_7.SetName("bar");

  inline_box_8.role = ax::mojom::Role::kInlineTextBox;
  inline_box_8.SetName("bar");

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_1.id;
  update.nodes = {root_1,       list_2,      list_item_3,   static_text_4,
                  inline_box_5, list_item_6, static_text_7, inline_box_8};

  Init(update);

  AXNode* root_node = GetRootAsAXNode();
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider, root_node);

  base::win::ScopedBstr find_string(L"oobar");
  Microsoft::WRL::ComPtr<ITextRangeProvider> text_range_provider_found;
  EXPECT_HRESULT_SUCCEEDED(text_range_provider->FindText(
      find_string.Get(), false, false, &text_range_provider_found));
  ASSERT_TRUE(text_range_provider_found.Get());
  Microsoft::WRL::ComPtr<AXPlatformNodeTextRangeProviderWin>
      text_range_provider_win;
  text_range_provider_found->QueryInterface(
      IID_PPV_ARGS(&text_range_provider_win));
  ASSERT_TRUE(GetStart(text_range_provider_win.Get())->IsTextPosition());
  EXPECT_EQ(5, GetStart(text_range_provider_win.Get())->anchor_id());
  EXPECT_EQ(1, GetStart(text_range_provider_win.Get())->text_offset());
  ASSERT_TRUE(GetEnd(text_range_provider_win.Get())->IsTextPosition());
  EXPECT_EQ(8, GetEnd(text_range_provider_win.Get())->anchor_id());
  EXPECT_EQ(3, GetEnd(text_range_provider_win.Get())->text_offset());
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderFindTextBackwards) {
  Init(BuildTextDocument({"text", "some", "text"},
                         false /* build_word_boundaries_offsets */,
                         true /* place_text_on_one_line */));
  AXNode* root_node = GetRootAsAXNode();

  ComPtr<ITextRangeProvider> root_range_provider;
  GetTextRangeProviderFromTextNode(root_range_provider, root_node);
  ComPtr<ITextRangeProvider> text_node1_range;
  GetTextRangeProviderFromTextNode(text_node1_range, root_node->children()[0]);
  ComPtr<ITextRangeProvider> text_node3_range;
  GetTextRangeProviderFromTextNode(text_node3_range, root_node->children()[2]);

  ComPtr<ITextRangeProvider> text_range_provider_found;
  base::win::ScopedBstr find_string(L"text");
  BOOL range_equal;

  // Forward search finds the text_node1.
  EXPECT_HRESULT_SUCCEEDED(root_range_provider->FindText(
      find_string.Get(), false, false, &text_range_provider_found));
  CopyOwnerToClone(root_range_provider.Get(), text_range_provider_found.Get());
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider_found, find_string.Get());

  range_equal = false;
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider_found->Compare(text_node1_range.Get(), &range_equal));
  EXPECT_TRUE(range_equal);

  // Backwards search finds the text_node3.
  EXPECT_HRESULT_SUCCEEDED(root_range_provider->FindText(
      find_string.Get(), true, false, &text_range_provider_found));
  CopyOwnerToClone(root_range_provider.Get(), text_range_provider_found.Get());
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider_found, find_string.Get());

  range_equal = false;
  EXPECT_HRESULT_SUCCEEDED(
      text_range_provider_found->Compare(text_node3_range.Get(), &range_equal));
  EXPECT_TRUE(range_equal);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestITextRangeProviderFindAttribute) {
  // document - visible
  //  [empty]
  //
  // Search forward, look for IsHidden=true.
  // Expected: nullptr
  // Search forward, look for IsHidden=false.
  // Expected: ""
  // Note: returns "" rather than nullptr here because document root web area by
  //       default set to visible. So the text range represents document matches
  //       our searching criteria. And we return a degenerate range.
  //
  // Search backward, look for IsHidden=true.
  // Expected: nullptr
  // Search backward, look for IsHidden=false.
  // Expected: ""
  // Note: returns "" rather than nullptr here because document root web area by
  //       default set to visible. So the text range represents document matches
  //       our searching criteria. And we return a degenerate range.
  {
    ui::AXNodeData root_data;
    root_data.id = 1;
    root_data.role = ax::mojom::Role::kRootWebArea;

    ui::AXTreeUpdate update;
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    update.has_tree_data = true;
    update.root_id = root_data.id;
    update.nodes = {root_data};

    Init(update);

    bool is_search_backward;
    VARIANT is_hidden_attr_val;
    V_VT(&is_hidden_attr_val) = VT_BOOL;
    ComPtr<ITextRangeProvider> matched_range_provider;
    ComPtr<ITextRangeProvider> document_range_provider;
    GetTextRangeProviderFromTextNode(document_range_provider,
                                     GetRootAsAXNode());

    // Search forward, look for IsHidden=true.
    // Expected: nullptr
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_EQ(nullptr, matched_range_provider.Get());

    // Search forward, look for IsHidden=false.
    // Expected: ""
    // Note: returns "" rather than nullptr here because document root web area
    //       by default set to visible. So the text range represents document
    //       matches our searching criteria. And we return a degenerate range.
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"");
    matched_range_provider.Reset();

    // Search backward, look for IsHidden=true.
    // Expected: nullptr
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_EQ(nullptr, matched_range_provider.Get());

    // Search backward, look for IsHidden=false.
    // Expected: ""
    // Note: returns "" rather than nullptr here because document root web area
    //       by default set to visible. So the text range represents document
    //       matches our searching criteria. And we return a degenerate range.
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"");
  }

  // document - visible
  //  text1 - invisible
  //
  // Search forward, look for IsHidden=true.
  // Expected: "text1"
  // Search forward, look for IsHidden=false.
  // Expected: nullptr
  // Search backward, look for IsHidden=true.
  // Expected: "text1"
  // Search backward, look for IsHidden=false.
  // Expected: nullptr
  {
    ui::AXNodeData text_data1;
    text_data1.id = 2;
    text_data1.role = ax::mojom::Role::kStaticText;
    text_data1.AddState(ax::mojom::State::kInvisible);
    text_data1.SetName("text1");

    ui::AXNodeData root_data;
    root_data.id = 1;
    root_data.role = ax::mojom::Role::kRootWebArea;
    root_data.child_ids = {2};

    ui::AXTreeUpdate update;
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    update.has_tree_data = true;
    update.root_id = root_data.id;
    update.nodes = {root_data, text_data1};

    Init(update);

    bool is_search_backward;
    VARIANT is_hidden_attr_val;
    V_VT(&is_hidden_attr_val) = VT_BOOL;
    ComPtr<ITextRangeProvider> matched_range_provider;
    ComPtr<ITextRangeProvider> document_range_provider;
    GetTextRangeProviderFromTextNode(document_range_provider,
                                     GetRootAsAXNode());

    // Search forward, look for IsHidden=true.
    // Expected: "text1"
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text1");
    matched_range_provider.Reset();

    // Search forward, look for IsHidden=false.
    // Expected: nullptr
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_EQ(nullptr, matched_range_provider.Get());

    // Search backward, look for IsHidden=true.
    // Expected: "text1"
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text1");
    matched_range_provider.Reset();

    // Search backward, look for IsHidden=false.
    // Expected: nullptr
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_EQ(nullptr, matched_range_provider.Get());
  }

  // document - visible
  //  text1 - visible
  //  text2 - visible
  //
  // Search forward, look for IsHidden=true.
  // Expected: nullptr
  // Search forward, look for IsHidden=false.
  // Expected: "text1text2"
  // Search backward, look for IsHidden=true.
  // Expected: nullptr
  // Search backward, look for IsHidden=false.
  // Expected: "text1text2"
  {
    ui::AXNodeData text_data1;
    text_data1.id = 2;
    text_data1.role = ax::mojom::Role::kStaticText;
    text_data1.SetName("text1");

    ui::AXNodeData text_data2;
    text_data2.id = 3;
    text_data2.role = ax::mojom::Role::kStaticText;
    text_data2.SetName("text2");

    ui::AXNodeData root_data;
    root_data.id = 1;
    root_data.role = ax::mojom::Role::kRootWebArea;
    root_data.child_ids = {2, 3};

    ui::AXTreeUpdate update;
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    update.has_tree_data = true;
    update.root_id = root_data.id;
    update.nodes = {root_data, text_data1, text_data2};

    Init(update);

    bool is_search_backward;
    VARIANT is_hidden_attr_val;
    V_VT(&is_hidden_attr_val) = VT_BOOL;
    ComPtr<ITextRangeProvider> matched_range_provider;
    ComPtr<ITextRangeProvider> document_range_provider;
    GetTextRangeProviderFromTextNode(document_range_provider,
                                     GetRootAsAXNode());

    // Search forward, look for IsHidden=true.
    // Expected: nullptr
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_EQ(nullptr, matched_range_provider.Get());

    // Search forward, look for IsHidden=false.
    // Expected: "text1text2"
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text1text2");
    matched_range_provider.Reset();

    // Search backward, look for IsHidden=true.
    // Expected: nullptr
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_EQ(nullptr, matched_range_provider.Get());

    // Search backward, look for IsHidden=false.
    // Expected: "text1text2"
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text1text2");
  }

  // document - visible
  //  text1 - visible
  //  text2 - invisible
  //  text3 - invisible
  //  text4 - visible
  //  text5 - invisible
  //
  // Search forward, look for IsHidden=true.
  // Expected: "text2text3"
  // Search forward, look for IsHidden=false.
  // Expected: "text1"
  // Search backward, look for IsHidden=true.
  // Expected: "text5"
  // Search backward, look for IsHidden=false.
  // Expected: "text4"
  {
    ui::AXNodeData text_data1;
    text_data1.id = 2;
    text_data1.role = ax::mojom::Role::kStaticText;
    text_data1.SetName("text1");

    ui::AXNodeData text_data2;
    text_data2.id = 3;
    text_data2.role = ax::mojom::Role::kStaticText;
    text_data2.AddState(ax::mojom::State::kInvisible);
    text_data2.SetName("text2");

    ui::AXNodeData text_data3;
    text_data3.id = 4;
    text_data3.role = ax::mojom::Role::kStaticText;
    text_data3.AddState(ax::mojom::State::kInvisible);
    text_data3.SetName("text3");

    ui::AXNodeData text_data4;
    text_data4.id = 5;
    text_data4.role = ax::mojom::Role::kStaticText;
    text_data4.SetName("text4");

    ui::AXNodeData text_data5;
    text_data5.id = 6;
    text_data5.role = ax::mojom::Role::kStaticText;
    text_data5.AddState(ax::mojom::State::kInvisible);
    text_data5.SetName("text5");

    ui::AXNodeData root_data;
    root_data.id = 1;
    root_data.role = ax::mojom::Role::kRootWebArea;
    root_data.child_ids = {2, 3, 4, 5, 6};

    ui::AXTreeUpdate update;
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    update.has_tree_data = true;
    update.root_id = root_data.id;
    update.nodes = {root_data,  text_data1, text_data2,
                    text_data3, text_data4, text_data5};

    Init(update);

    bool is_search_backward;
    VARIANT is_hidden_attr_val;
    V_VT(&is_hidden_attr_val) = VT_BOOL;
    ComPtr<ITextRangeProvider> matched_range_provider;
    ComPtr<ITextRangeProvider> document_range_provider;
    GetTextRangeProviderFromTextNode(document_range_provider,
                                     GetRootAsAXNode());

    // Search forward, look for IsHidden=true.
    // Expected: "text2text3"
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text2text3");
    matched_range_provider.Reset();

    // Search forward, look for IsHidden=false.
    // Expected: "text1"
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text1");
    matched_range_provider.Reset();

    // Search backward, look for IsHidden=true.
    // Expected: "text5"
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text5");
    matched_range_provider.Reset();

    // Search backward, look for IsHidden=false.
    // Expected: "text4"
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text4");
  }

  // document - visible
  //  text1 - visible
  //  text2 - invisible
  //  text3 - invisible
  //  text4 - invisible
  //  text5 - visible
  //
  // Search forward, look for IsHidden=true.
  // Expected: "text2text3text4"
  // Search forward, look for IsHidden=false.
  // Expected: "text1"
  // Search backward, look for IsHidden=true.
  // Expected: "text2text3text4"
  // Search backward, look for IsHidden=false.
  // Expected: "text5"
  {
    ui::AXNodeData text_data1;
    text_data1.id = 2;
    text_data1.role = ax::mojom::Role::kStaticText;
    text_data1.SetName("text1");

    ui::AXNodeData text_data2;
    text_data2.id = 3;
    text_data2.role = ax::mojom::Role::kStaticText;
    text_data2.AddState(ax::mojom::State::kInvisible);
    text_data2.SetName("text2");

    ui::AXNodeData text_data3;
    text_data3.id = 4;
    text_data3.role = ax::mojom::Role::kStaticText;
    text_data3.AddState(ax::mojom::State::kInvisible);
    text_data3.SetName("text3");

    ui::AXNodeData text_data4;
    text_data4.id = 5;
    text_data4.role = ax::mojom::Role::kStaticText;
    text_data4.AddState(ax::mojom::State::kInvisible);
    text_data4.SetName("text4");

    ui::AXNodeData text_data5;
    text_data5.id = 6;
    text_data5.role = ax::mojom::Role::kStaticText;
    text_data5.SetName("text5");

    ui::AXNodeData root_data;
    root_data.id = 1;
    root_data.role = ax::mojom::Role::kRootWebArea;
    root_data.child_ids = {2, 3, 4, 5, 6};

    ui::AXTreeUpdate update;
    update.tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
    update.has_tree_data = true;
    update.root_id = root_data.id;
    update.nodes = {root_data,  text_data1, text_data2,
                    text_data3, text_data4, text_data5};

    Init(update);

    bool is_search_backward;
    VARIANT is_hidden_attr_val;
    V_VT(&is_hidden_attr_val) = VT_BOOL;
    ComPtr<ITextRangeProvider> matched_range_provider;
    ComPtr<ITextRangeProvider> document_range_provider;
    GetTextRangeProviderFromTextNode(document_range_provider,
                                     GetRootAsAXNode());

    // Search forward, look for IsHidden=true.
    // Expected: "text2text3text4"
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text2text3text4");
    matched_range_provider.Reset();

    // Search forward, look for IsHidden=false.
    // Expected: "text1"
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = false;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text1");
    matched_range_provider.Reset();

    // Search backward, look for IsHidden=true.
    // Expected: "text2text3text4"
    V_BOOL(&is_hidden_attr_val) = VARIANT_TRUE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text2text3text4");
    matched_range_provider.Reset();

    // Search backward, look for IsHidden=false.
    // Expected: "text5"
    V_BOOL(&is_hidden_attr_val) = VARIANT_FALSE;
    is_search_backward = true;
    document_range_provider->FindAttribute(
        UIA_IsHiddenAttributeId, is_hidden_attr_val, is_search_backward,
        &matched_range_provider);
    ASSERT_NE(nullptr, matched_range_provider.Get());
    CopyOwnerToClone(document_range_provider.Get(),
                     matched_range_provider.Get());
    EXPECT_UIA_TEXTRANGE_EQ(matched_range_provider, L"text5");
  }
}

TEST_F(AXPlatformNodeTextRangeProviderTest, ElementNotAvailable) {
  AXNodeData root_ax_node_data;
  root_ax_node_data.id = 1;
  root_ax_node_data.role = ax::mojom::Role::kRootWebArea;

  Init(root_ax_node_data);

  ComPtr<IRawElementProviderSimple> raw_element_provider_simple =
      QueryInterfaceFromNode<IRawElementProviderSimple>(GetRootAsAXNode());
  ASSERT_NE(nullptr, raw_element_provider_simple.Get());

  ComPtr<ITextProvider> text_provider;
  ASSERT_HRESULT_SUCCEEDED(raw_element_provider_simple->GetPatternProvider(
      UIA_TextPatternId, &text_provider));
  ASSERT_NE(nullptr, text_provider.Get());

  ComPtr<ITextRangeProvider> text_range_provider;
  ASSERT_HRESULT_SUCCEEDED(
      text_provider->get_DocumentRange(&text_range_provider));
  ASSERT_NE(nullptr, text_range_provider.Get());

  // An empty tree.
  SetTree(std::make_unique<AXTree>());

  BOOL bool_arg = FALSE;
  ASSERT_EQ(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE),
            text_range_provider->ScrollIntoView(bool_arg));
}

// TODO(schectman) Non-empty ignored nodes are not used by Flutter.
// https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestITextRangeProviderIgnoredNodes) {
  // Parent Tree
  // 1
  // |
  // 2(i)
  // |________________________________
  // |   |   |    |      |           |
  // 3   4   5    6      7(i)        8(i)
  //              |      |________
  //              |      |       |
  //              9(i)   10(i)   11
  //              |      |____
  //              |      |   |
  //              12    13   14

  ui::AXTreeUpdate tree_update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  tree_update.tree_data.tree_id = tree_id;
  tree_update.has_tree_data = true;
  tree_update.root_id = 1;
  tree_update.nodes.resize(14);
  tree_update.nodes[0].id = 1;
  tree_update.nodes[0].child_ids = {2};
  tree_update.nodes[0].role = ax::mojom::Role::kRootWebArea;

  tree_update.nodes[1].id = 2;
  tree_update.nodes[1].child_ids = {3, 4, 5, 6, 7, 8};
  // According to the existing Blink code, editable roots are never ignored.
  // However, we can still create this tree structure only for test purposes.
  tree_update.nodes[1].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[1].AddState(ax::mojom::State::kEditable);
  tree_update.nodes[1].AddState(ax::mojom::State::kRichlyEditable);
  // tree_update.nodes[1].AddBoolAttribute(
  //     ax::mojom::BoolAttribute::kNonAtomicTextFieldRoot, true);
  tree_update.nodes[1].role = ax::mojom::Role::kGenericContainer;

  tree_update.nodes[2].id = 3;
  tree_update.nodes[2].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[2].SetName(".3.");

  tree_update.nodes[3].id = 4;
  tree_update.nodes[3].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[3].SetName(".4.");

  tree_update.nodes[4].id = 5;
  tree_update.nodes[4].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[4].SetName(".5.");

  tree_update.nodes[5].id = 6;
  tree_update.nodes[5].role = ax::mojom::Role::kButton;
  tree_update.nodes[5].child_ids = {9};

  tree_update.nodes[6].id = 7;
  tree_update.nodes[6].child_ids = {10, 11};
  tree_update.nodes[6].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[6].role = ax::mojom::Role::kGenericContainer;

  tree_update.nodes[7].id = 8;
  tree_update.nodes[7].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[7].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[7].SetName(".8.");

  tree_update.nodes[8].id = 9;
  tree_update.nodes[8].child_ids = {12};
  tree_update.nodes[8].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[8].role = ax::mojom::Role::kGenericContainer;

  tree_update.nodes[9].id = 10;
  tree_update.nodes[9].child_ids = {13, 14};
  tree_update.nodes[9].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[8].role = ax::mojom::Role::kGenericContainer;

  tree_update.nodes[10].id = 11;
  tree_update.nodes[10].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[10].SetName(".11.");

  tree_update.nodes[11].id = 12;
  tree_update.nodes[11].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[11].AddState(ax::mojom::State::kIgnored);
  tree_update.nodes[11].SetName(".12.");

  tree_update.nodes[12].id = 13;
  tree_update.nodes[12].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[12].SetName(".13.");

  tree_update.nodes[13].id = 14;
  tree_update.nodes[13].role = ax::mojom::Role::kStaticText;
  tree_update.nodes[13].SetName(".14.");

  Init(tree_update);
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 1),
                           GetNodeFromTree(tree_id, 1));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 2),
                           GetNodeFromTree(tree_id, 1));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 3),
                           GetNodeFromTree(tree_id, 3));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 4),
                           GetNodeFromTree(tree_id, 4));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 5),
                           GetNodeFromTree(tree_id, 5));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 8),
                           GetNodeFromTree(tree_id, 1));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 11),
                           GetNodeFromTree(tree_id, 11));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 13),
                           GetNodeFromTree(tree_id, 13));
  EXPECT_ENCLOSING_ELEMENT(GetNodeFromTree(tree_id, 14),
                           GetNodeFromTree(tree_id, 14));

  // Test movement and GetText()
  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetNodeFromTree(tree_id, 1));

  ASSERT_HRESULT_SUCCEEDED(
      text_range_provider->ExpandToEnclosingUnit(TextUnit_Character));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L".");

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 2,
      /*expected_text*/ L".3.",
      /*expected_count*/ 2);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 6,
      /*expected_text*/ L".3..4..5.",
      /*expected_count*/ 6);

  // By design, empty objects, such as the unlabelled button in this case, are
  // placed in their own paragraph for easier screen reader navigation.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 15,
      /*expected_text*/ L".3..4..5.\n\xFFFC\n.13..14..11.",
      /*expected_count*/ 15);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestNormalizeTextRangePastEndOfDocument) {
  ui::AXTreeUpdate initial_state;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  initial_state.tree_data.tree_id = tree_id;
  initial_state.has_tree_data = true;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2};
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids = {3};
  initial_state.nodes[1].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[1].SetName("aaa");
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kInlineTextBox;
  initial_state.nodes[2].SetName("aaa");

  Init(initial_state);

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetNodeFromTree(tree_id, 3));

  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"aaa");
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 2,
      /*expected_text*/ L"a",
      /*expected_count*/ 2);

  ComPtr<AXPlatformNodeTextRangeProviderWin> text_range_provider_win;
  text_range_provider->QueryInterface(IID_PPV_ARGS(&text_range_provider_win));

  const AXNodePosition::AXPositionInstance start_after_move =
      GetStart(text_range_provider_win.Get())->Clone();
  const AXNodePosition::AXPositionInstance end_after_move =
      GetEnd(text_range_provider_win.Get())->Clone();
  EXPECT_LT(*start_after_move, *end_after_move);

  AXTreeUpdate update;
  update.nodes.resize(2);
  update.nodes[0] = initial_state.nodes[1];
  update.nodes[0].SetName("aa");
  update.nodes[1] = initial_state.nodes[2];
  update.nodes[1].SetName("aa");
  ASSERT_TRUE(GetTree()->Unserialize(update));

  auto* text_range = text_range_provider_win.Get();

  auto original_start = GetStart(text_range)->Clone();
  auto original_end = GetEnd(text_range)->Clone();

  auto normalized_start = GetStart(text_range)->Clone();
  auto normalized_end = GetEnd(text_range)->Clone();
  NormalizeTextRange(text_range, normalized_start, normalized_end);
  // Verify that the original range was not changed by normalization.
  ExpectPositionsEqual(original_start, GetStart(text_range));
  ExpectPositionsEqual(original_end, GetEnd(text_range));

  EXPECT_EQ(*start_after_move, *normalized_start);
  EXPECT_EQ(*end_after_move, *normalized_end);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestNormalizeTextRangePastEndOfDocumentWithIgnoredNodes) {
  ui::AXTreeUpdate initial_state;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  initial_state.tree_data.tree_id = tree_id;
  initial_state.has_tree_data = true;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2};
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].child_ids = {3, 4};
  initial_state.nodes[1].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[1].SetName("aaa");
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kInlineTextBox;
  initial_state.nodes[2].SetName("aaa");
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kInlineTextBox;
  initial_state.nodes[3].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[3].SetName("ignored");

  Init(initial_state);

  ComPtr<ITextRangeProvider> text_range_provider;
  GetTextRangeProviderFromTextNode(text_range_provider,
                                   GetNodeFromTree(tree_id, 3));

  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"aaa");
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 2,
      /*expected_text*/ L"a",
      /*expected_count*/ 2);

  ComPtr<AXPlatformNodeTextRangeProviderWin> text_range_provider_win;
  text_range_provider->QueryInterface(IID_PPV_ARGS(&text_range_provider_win));

  const AXNodePosition::AXPositionInstance start_after_move =
      GetStart(text_range_provider_win.Get())->Clone();
  const AXNodePosition::AXPositionInstance end_after_move =
      GetEnd(text_range_provider_win.Get())->Clone();
  EXPECT_LT(*start_after_move, *end_after_move);

  AXTreeUpdate update;
  update.nodes.resize(2);
  update.nodes[0] = initial_state.nodes[1];
  update.nodes[0].SetName("aa");
  update.nodes[1] = initial_state.nodes[2];
  update.nodes[1].SetName("aa");
  ASSERT_TRUE(GetTree()->Unserialize(update));

  auto* text_range = text_range_provider_win.Get();

  auto original_start = GetStart(text_range)->Clone();
  auto original_end = GetEnd(text_range)->Clone();

  auto normalized_start = GetStart(text_range)->Clone();
  auto normalized_end = GetEnd(text_range)->Clone();
  NormalizeTextRange(text_range, normalized_start, normalized_end);
  // Verify that the original range was not changed by normalization.
  ExpectPositionsEqual(original_start, GetStart(text_range));
  ExpectPositionsEqual(original_end, GetEnd(text_range));

  EXPECT_EQ(*start_after_move, *normalized_start);
  EXPECT_EQ(*end_after_move, *normalized_end);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestNormalizeTextRangeInsideIgnoredNodes) {
  ui::AXTreeUpdate initial_state;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  initial_state.tree_data.tree_id = tree_id;
  initial_state.has_tree_data = true;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].child_ids = {2, 3, 4};
  initial_state.nodes[0].role = ax::mojom::Role::kRootWebArea;
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[1].SetName("before");
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[2].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[2].SetName("ignored");
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kStaticText;
  initial_state.nodes[3].SetName("after");

  Init(initial_state);
  const AXTree* tree = GetTree();
  const AXNode* ignored_node = tree->GetFromId(3);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  // start: TextPosition, anchor_id=3, text_offset=1, annotated_text=i<g>nored
  // end  : TextPosition, anchor_id=3, text_offset=6, annotated_text=ignore<d>
  ComPtr<AXPlatformNodeTextRangeProviderWin> ignored_range_win;
  CreateTextRangeProviderWin(
      ignored_range_win, owner,
      /*start_anchor=*/ignored_node, /*start_offset=*/0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/ignored_node, /*end_offset=*/0,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_TRUE(GetStart(ignored_range_win.Get())->IsIgnored());
  EXPECT_TRUE(GetEnd(ignored_range_win.Get())->IsIgnored());

  auto original_start = GetStart(ignored_range_win.Get())->Clone();
  auto original_end = GetEnd(ignored_range_win.Get())->Clone();

  auto normalized_start = GetStart(ignored_range_win.Get())->Clone();
  auto normalized_end = GetEnd(ignored_range_win.Get())->Clone();
  NormalizeTextRange(ignored_range_win.Get(), normalized_start, normalized_end);
  // Verify that the original range was not changed by normalization.
  ExpectPositionsEqual(original_start, GetStart(ignored_range_win.Get()));
  ExpectPositionsEqual(original_end, GetEnd(ignored_range_win.Get()));

  EXPECT_FALSE(normalized_start->IsIgnored());
  EXPECT_FALSE(normalized_end->IsIgnored());
  EXPECT_LE(*GetStart(ignored_range_win.Get()), *normalized_start);
  EXPECT_LE(*GetEnd(ignored_range_win.Get()), *normalized_end);
  EXPECT_LE(*normalized_start, *normalized_end);

  // Remove the last node, forcing |NormalizeTextRange| to normalize
  // using the opposite AdjustmentBehavior.
  AXTreeUpdate update;
  update.nodes.resize(1);
  update.nodes[0] = initial_state.nodes[0];
  update.nodes[0].child_ids = {2, 3};
  ASSERT_TRUE(GetTree()->Unserialize(update));

  original_start = GetStart(ignored_range_win.Get())->Clone();
  original_end = GetEnd(ignored_range_win.Get())->Clone();

  normalized_start = GetStart(ignored_range_win.Get())->Clone();
  normalized_end = GetEnd(ignored_range_win.Get())->Clone();
  NormalizeTextRange(ignored_range_win.Get(), normalized_start, normalized_end);
  // Verify that the original range was not changed by normalization.
  ExpectPositionsEqual(original_start, GetStart(ignored_range_win.Get()));
  ExpectPositionsEqual(original_end, GetEnd(ignored_range_win.Get()));

  EXPECT_FALSE(normalized_start->IsIgnored());
  EXPECT_FALSE(normalized_end->IsIgnored());
  EXPECT_GE(*GetStart(ignored_range_win.Get()), *normalized_start);
  EXPECT_GE(*GetEnd(ignored_range_win.Get()), *normalized_end);
  EXPECT_LE(*normalized_start, *normalized_end);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestNormalizeTextRangeSpanIgnoredNodes) {
  ui::AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  ui::AXNodeData before_text;
  before_text.id = 2;
  before_text.role = ax::mojom::Role::kStaticText;
  before_text.SetName("before");
  root_data.child_ids.push_back(before_text.id);

  ui::AXNodeData ignored_text1;
  ignored_text1.id = 3;
  ignored_text1.role = ax::mojom::Role::kStaticText;
  ignored_text1.AddState(ax::mojom::State::kIgnored);
  ignored_text1.SetName("ignored1");
  root_data.child_ids.push_back(ignored_text1.id);

  ui::AXNodeData ignored_text2;
  ignored_text2.id = 4;
  ignored_text2.role = ax::mojom::Role::kStaticText;
  ignored_text2.AddState(ax::mojom::State::kIgnored);
  ignored_text2.SetName("ignored2");
  root_data.child_ids.push_back(ignored_text2.id);

  ui::AXNodeData after_text;
  after_text.id = 5;
  after_text.role = ax::mojom::Role::kStaticText;
  after_text.SetName("after");
  root_data.child_ids.push_back(after_text.id);

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_data.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_data, before_text, ignored_text1, ignored_text2,
                  after_text};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* before_text_node = tree->GetFromId(before_text.id);
  const AXNode* after_text_node = tree->GetFromId(after_text.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  // Original range before NormalizeTextRange()
  // |before<>||ignored1||ignored2||<a>fter|
  //         |-----------------------|
  // start: TextPosition, anchor_id=2, text_offset=6, annotated_text=before<>
  // end  : TextPosition, anchor_id=5, text_offset=0, annotated_text=<a>fter
  ComPtr<AXPlatformNodeTextRangeProviderWin> range_span_ignored_nodes;
  CreateTextRangeProviderWin(
      range_span_ignored_nodes, owner,
      /*start_anchor=*/before_text_node, /*start_offset=*/6,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/after_text_node, /*end_offset=*/0,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  auto original_start = GetStart(range_span_ignored_nodes.Get())->Clone();
  auto original_end = GetEnd(range_span_ignored_nodes.Get())->Clone();

  // Normalized range after NormalizeTextRange()
  // |before||ignored1||ignored2||<a>fter|
  //                              |-|
  AXNodePosition::AXPositionInstance normalized_start =
      GetStart(range_span_ignored_nodes.Get())->Clone();
  AXNodePosition::AXPositionInstance normalized_end =
      GetEnd(range_span_ignored_nodes.Get())->Clone();
  NormalizeTextRange(range_span_ignored_nodes.Get(), normalized_start,
                     normalized_end);
  // Verify that the original range was not changed by normalization.
  ExpectPositionsEqual(original_start,
                       GetStart(range_span_ignored_nodes.Get()));
  ExpectPositionsEqual(original_end, GetEnd(range_span_ignored_nodes.Get()));

  EXPECT_EQ(*normalized_start, *normalized_end);

  EXPECT_TRUE(normalized_start->IsTextPosition());
  EXPECT_TRUE(normalized_start->AtStartOfAnchor());
  EXPECT_EQ(5, normalized_start->anchor_id());
  EXPECT_EQ(0, normalized_start->text_offset());

  EXPECT_TRUE(normalized_end->IsTextPosition());
  EXPECT_TRUE(normalized_end->AtStartOfAnchor());
  EXPECT_EQ(5, normalized_end->anchor_id());
  EXPECT_EQ(0, normalized_end->text_offset());
}

// TODO(schectman) Non-zero text offset in position into an empty node.
// Why? https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestNormalizeTextRangeForceSameAnchorOnDegenerateRange) {
  // ++1 kRootWebArea
  // ++++2 kGenericContainer
  // ++++++3 kImage
  // ++++4 kTextField
  // ++++++5 kGenericContainer
  // ++++++++6 kStaticText
  // ++++++++++7 kInlineTextBox
  ui::AXNodeData root_1;
  ui::AXNodeData generic_container_2;
  ui::AXNodeData line_break_3;
  ui::AXNodeData text_field_4;
  ui::AXNodeData generic_container_5;
  ui::AXNodeData static_text_6;
  ui::AXNodeData inline_box_7;

  root_1.id = 1;
  generic_container_2.id = 2;
  line_break_3.id = 3;
  text_field_4.id = 4;
  generic_container_5.id = 5;
  static_text_6.id = 6;
  inline_box_7.id = 7;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {generic_container_2.id, text_field_4.id};

  generic_container_2.role = ax::mojom::Role::kGenericContainer;
  generic_container_2.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  generic_container_2.child_ids = {line_break_3.id};

  line_break_3.role = ax::mojom::Role::kLineBreak;

  text_field_4.role = ax::mojom::Role::kTextField;
  text_field_4.AddState(ax::mojom::State::kEditable);
  text_field_4.child_ids = {generic_container_5.id};
  text_field_4.SetValue("3.14");

  generic_container_5.role = ax::mojom::Role::kGenericContainer;
  generic_container_5.child_ids = {static_text_6.id};

  static_text_6.role = ax::mojom::Role::kStaticText;
  static_text_6.child_ids = {inline_box_7.id};
  static_text_6.SetName("3.14");

  inline_box_7.role = ax::mojom::Role::kInlineTextBox;
  inline_box_7.SetName("3.14");

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_1.id;
  update.nodes.push_back(root_1);
  update.nodes.push_back(generic_container_2);
  update.nodes.push_back(line_break_3);
  update.nodes.push_back(text_field_4);
  update.nodes.push_back(generic_container_5);
  update.nodes.push_back(static_text_6);
  update.nodes.push_back(inline_box_7);

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* line_break_3_node = tree->GetFromId(line_break_3.id);
  const AXNode* inline_box_7_node = tree->GetFromId(inline_box_7.id);

  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_data.tree_id, 1)));

  // start: TextPosition, anchor_id=3, text_offset=1, annotated_text=/xFFFC<>
  // end  : TextPosition, anchor_id=7, text_offset=0, annotated_text=<p>i
  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor=*/line_break_3_node, /*start_offset=*/1,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/inline_box_7_node, /*end_offset=*/0,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  auto original_start = GetStart(range.Get())->Clone();
  auto original_end = GetEnd(range.Get())->Clone();

  AXNodePosition::AXPositionInstance normalized_start =
      GetStart(range.Get())->Clone();
  AXNodePosition::AXPositionInstance normalized_end =
      GetEnd(range.Get())->Clone();
  NormalizeTextRange(range.Get(), normalized_start, normalized_end);
  // Verify that the original range was not changed by normalization.
  ExpectPositionsEqual(original_start, GetStart(range.Get()));
  ExpectPositionsEqual(original_end, GetEnd(range.Get()));

  EXPECT_EQ(*normalized_start, *normalized_start);

  EXPECT_TRUE(normalized_start->AtStartOfAnchor());
  EXPECT_TRUE(normalized_end->AtStartOfAnchor());
  EXPECT_EQ(7, normalized_start->anchor_id());
  EXPECT_EQ(7, normalized_end->anchor_id());
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest, DISABLED_TestValidateStartAndEnd) {
  // This test updates the tree structure to test a specific edge case -
  // CreatePositionAtFormatBoundary when text lies at the beginning and end
  // of the AX tree.
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData text_data;
  text_data.id = 2;
  text_data.role = ax::mojom::Role::kStaticText;
  text_data.SetName("some text");

  AXNodeData more_text_data;
  more_text_data.id = 3;
  more_text_data.role = ax::mojom::Role::kStaticText;
  more_text_data.SetName("more text");

  root_data.child_ids = {text_data.id, more_text_data.id};

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_data.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_data, text_data, more_text_data};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* root_node = tree->GetFromId(root_data.id);
  const AXNode* more_text_node = tree->GetFromId(more_text_data.id);

  // Create a position at MaxTextOffset
  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  // start: TextPosition, anchor_id=1, text_offset=0, annotated_text=<s>ome text
  // end  : TextPosition, anchor_id=3, text_offset=9, annotated_text=more text<>
  ComPtr<AXPlatformNodeTextRangeProviderWin> text_range_provider;
  CreateTextRangeProviderWin(
      text_range_provider, owner,
      /*start_anchor=*/root_node, /*start_offset=*/0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor=*/more_text_node, /*end_offset=*/9,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  // Since the end of the range is at MaxTextOffset, moving it by 1 character
  // should have an expected_count of 0.
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"some textmore text",
      /*expected_count*/ 0);

  // Now make a change to shorten MaxTextOffset. Ensure that this position is
  // invalid, then call SnapToMaxTextOffsetIfBeyond and ensure that it is now
  // valid.
  more_text_data.SetName("ore tex");
  AXTreeUpdate test_update;
  test_update.nodes = {more_text_data};
  ASSERT_TRUE(GetTree()->Unserialize(test_update));

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"some textore tex",
      /*expected_count*/ 0);

  // Now modify the tree so that start_ is pointing to a node that has been
  // removed from the tree.
  text_data.SetNameExplicitlyEmpty();
  AXTreeUpdate test_update2;
  test_update2.nodes = {text_data};
  ASSERT_TRUE(GetTree()->Unserialize(test_update2));

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"re tex",
      /*expected_count*/ 1);

  // Now adjust a node that's not the final node in the tree to point past
  // MaxTextOffset. First move the range endpoints so that they're pointing to
  // MaxTextOffset on the first node.
  text_data.SetName("some text");
  AXTreeUpdate test_update3;
  test_update3.nodes = {text_data};
  ASSERT_TRUE(GetTree()->Unserialize(test_update3));

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ -10,
      /*expected_text*/ L"some textore tex",
      /*expected_count*/ -10);

  // Ensure that we're at MaxTextOffset on the first node by first
  // overshooting a negative move...
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ -8,
      /*expected_text*/ L"some tex",
      /*expected_count*/ -8);

  // ...followed by a positive move
  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_End, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"some text",
      /*expected_count*/ 1);

  // Now our range's start_ is pointing to offset 0 on the first node and end_
  // is pointing to MaxTextOffset on the first node. Now modify the tree so
  // that MaxTextOffset is invalid on the first node and ensure that we can
  // still move
  text_data.SetName("some tex");
  AXTreeUpdate test_update4;
  test_update4.nodes = {text_data};
  ASSERT_TRUE(GetTree()->Unserialize(test_update4));

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(
      text_range_provider, TextPatternRangeEndpoint_Start, TextUnit_Character,
      /*count*/ 1,
      /*expected_text*/ L"ome tex",
      /*expected_count*/ 1);
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestReplaceStartAndEndEndpointNode) {
  // This test updates the tree structure to ensure that the text range is still
  // valid after a text node gets replaced by another one. This case occurs
  // every time an AT's focus moves to a node whose style is affected by focus,
  // thus generating a tree update.
  //
  // ++1 kRootWebArea
  // ++++2 kGroup (ignored)
  // ++++++3 kStaticText/++++4 kStaticText (replacement node)
  // ++++5 kStaticText/++++6 kStaticText (replacement node)
  AXNodeData root_1;
  AXNodeData group_2;
  AXNodeData text_3;
  AXNodeData text_4;
  AXNodeData text_5;
  AXNodeData text_6;

  root_1.id = 1;
  group_2.id = 2;
  text_3.id = 3;
  text_4.id = 4;
  text_5.id = 5;
  text_6.id = 6;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {text_3.id, text_5.id};

  group_2.role = ax::mojom::Role::kGroup;
  group_2.AddState(ax::mojom::State::kIgnored);
  group_2.child_ids = {text_3.id};

  text_3.role = ax::mojom::Role::kStaticText;
  text_3.SetName("some text");

  // Replacement node of |text_3|.
  text_4.role = ax::mojom::Role::kStaticText;
  text_4.SetName("some text");

  text_5.role = ax::mojom::Role::kStaticText;
  text_5.SetName("more text");

  // Replacement node of |text_5|.
  text_6.role = ax::mojom::Role::kStaticText;
  text_6.SetName("more text");

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1, text_3, text_5};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* text_3_node = tree->GetFromId(text_3.id);
  const AXNode* text_5_node = tree->GetFromId(text_5.id);

  // Create a position at MaxTextOffset.
  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  // start: TextPosition, anchor_id=3, text_offset=0, annotated_text=<s>ome text
  // end  : TextPosition, anchor_id=5, text_offset=9, annotated_text=more text<>
  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor*/ text_3_node, /*start_offset*/ 0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_5_node, /*end_offset*/ 9,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"some textmore text");

  // 1. Replace the node on which |start_| is.
  {
    // Replace node |text_3| with |text_4|.
    root_1.child_ids = {text_4.id, text_5.id};
    AXTreeUpdate test_update;
    test_update.nodes = {root_1, text_4};
    ASSERT_TRUE(GetTree()->Unserialize(test_update));

    // Replacing that node shouldn't impact the range.
    base::win::ScopedSafearray children;
    range->GetChildren(children.Receive());
    EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"some textmore text");

    // The |start_| endpoint should have moved to the root, skipping its ignored
    // parent.
    EXPECT_EQ(root_1.id, GetStart(range.Get())->anchor_id());
    EXPECT_EQ(0, GetStart(range.Get())->text_offset());

    // The |end_| endpoint should not have moved.
    EXPECT_EQ(text_5.id, GetEnd(range.Get())->anchor_id());
    EXPECT_EQ(9, GetEnd(range.Get())->text_offset());
  }

  // 2. Replace the node on which |end_| is.
  {
    // Replace node |text_4| with |text_5|.
    root_1.child_ids = {text_4.id, text_6.id};
    AXTreeUpdate test_update;
    test_update.nodes = {root_1, text_6};
    ASSERT_TRUE(GetTree()->Unserialize(test_update));

    // Replacing that node shouldn't impact the range.
    base::win::ScopedSafearray children;
    range->GetChildren(children.Receive());
    EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"some textmore text");

    // The |start_| endpoint should still be on its parent.
    EXPECT_EQ(root_1.id, GetStart(range.Get())->anchor_id());
    EXPECT_EQ(0, GetStart(range.Get())->text_offset());

    // The |end_| endpoint should have moved to its parent.
    EXPECT_EQ(root_1.id, GetEnd(range.Get())->anchor_id());
    EXPECT_EQ(18, GetEnd(range.Get())->text_offset());
  }

  // 3. Replace the node on which |start_| and |end_| is.
  {
    // start: TextPosition, anchor_id=4, text_offset=0, annotated_text=<s>ome
    // end  : TextPosition, anchor_id=4, text_offset=4, annotated_text=some<>
    const AXNode* text_4_node = tree->GetFromId(text_4.id);
    ComPtr<AXPlatformNodeTextRangeProviderWin> range_2;
    CreateTextRangeProviderWin(
        range_2, owner,
        /*start_anchor*/ text_4_node, /*start_offset*/ 0,
        /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
        /*end_anchor*/ text_4_node, /*end_offset*/ 4,
        /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

    EXPECT_UIA_TEXTRANGE_EQ(range_2, /*expected_text*/ L"some");

    // Replace node |text_4| with |text_3|.
    root_1.child_ids = {text_3.id, text_6.id};
    AXTreeUpdate test_update;
    test_update.nodes = {root_1, text_3};
    ASSERT_TRUE(GetTree()->Unserialize(test_update));

    // Replacing that node shouldn't impact the range.
    base::win::ScopedSafearray children;
    range_2->GetChildren(children.Receive());
    EXPECT_UIA_TEXTRANGE_EQ(range_2, /*expected_text*/ L"some");

    // The |start_| endpoint should have moved to its parent.
    EXPECT_EQ(root_1.id, GetStart(range_2.Get())->anchor_id());
    EXPECT_EQ(0, GetStart(range_2.Get())->text_offset());

    // The |end_| endpoint should have moved to its parent.
    EXPECT_EQ(root_1.id, GetEnd(range_2.Get())->anchor_id());
    EXPECT_EQ(4, GetEnd(range_2.Get())->text_offset());
  }
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestDeleteSubtreeThatIncludesEndpoints) {
  // This test updates the tree structure to ensure that the text range is still
  // valid after a subtree that includes the text range is deleted, resulting in
  // a change to the range.
  //
  // ++1 kRootWebArea
  // ++++2 kStaticText "one"
  // ++++3 kGenericContainer
  // ++++++4 kGenericContainer
  // ++++++++5 kStaticText " two"
  // ++++++6 kGenericContainer
  // ++++++++7 kStaticText " three"
  AXNodeData root_1;
  AXNodeData text_2;
  AXNodeData gc_3;
  AXNodeData gc_4;
  AXNodeData text_5;
  AXNodeData gc_6;
  AXNodeData text_7;

  root_1.id = 1;
  text_2.id = 2;
  gc_3.id = 3;
  gc_4.id = 4;
  text_5.id = 5;
  gc_6.id = 6;
  text_7.id = 7;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {text_2.id, gc_3.id};

  text_2.role = ax::mojom::Role::kStaticText;
  text_2.SetName("one");

  gc_3.role = ax::mojom::Role::kGenericContainer;
  gc_3.child_ids = {gc_4.id, gc_6.id};

  gc_4.role = ax::mojom::Role::kGenericContainer;
  gc_4.child_ids = {text_5.id};

  text_5.role = ax::mojom::Role::kStaticText;
  text_5.SetName(" two");

  gc_6.role = ax::mojom::Role::kGenericContainer;
  gc_6.child_ids = {text_7.id};

  text_7.role = ax::mojom::Role::kStaticText;
  text_7.SetName(" three");

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1, text_2, gc_3, gc_4, text_5, gc_6, text_7};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* text_5_node = tree->GetFromId(text_5.id);
  const AXNode* text_7_node = tree->GetFromId(text_7.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  // Create a range that spans " two three" located on the leaf nodes.

  // start: TextPosition, anchor_id=5, text_offset=0
  // end  : TextPosition, anchor_id=7, text_offset=6
  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor*/ text_5_node, /*start_offset*/ 0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_7_node, /*end_offset*/ 6,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L" two three");

  // Delete |gc_3|, which will delete the entire subtree where both of our
  // endpoints are.
  AXTreeUpdate test_update;
  root_1.child_ids = {text_2.id};
  test_update.nodes = {root_1};
  ASSERT_TRUE(GetTree()->Unserialize(test_update));

  // The text range should now be a degenerate range positioned at the end of
  // root, the parent of |gc_3|, since |gc_3| has been deleted.
  EXPECT_EQ(root_1.id, GetStart(range.Get())->anchor_id());
  EXPECT_EQ(3, GetStart(range.Get())->text_offset());

  EXPECT_EQ(root_1.id, GetEnd(range.Get())->anchor_id());
  EXPECT_EQ(3, GetEnd(range.Get())->text_offset());
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestDeleteSubtreeWithIgnoredAncestors) {
  // This test updates the tree structure to ensure that the text range doesn't
  // crash and points to null positions after a subtree that includes the text
  // range is deleted and all ancestors are ignored.
  //
  // ++1 kRootWebArea ignored
  // ++++2 kStaticText "one"
  // ++++3 kGenericContainer ignored
  // ++++++4 kGenericContainer
  // ++++++++5 kGenericContainer
  // ++++++++++6 kStaticText " two"
  // ++++++++7 kGenericContainer ignored
  // ++++++++++8 kStaticText " ignored" ignored
  // ++++++++9 kGenericContainer
  // ++++++++++10 kStaticText " three"
  // ++++11 kGenericContainer
  // ++++++12 kStaticText "four"
  AXNodeData root_1;
  AXNodeData text_2;
  AXNodeData gc_3;
  AXNodeData gc_4;
  AXNodeData gc_5;
  AXNodeData text_6;
  AXNodeData gc_7;
  AXNodeData text_8;
  AXNodeData gc_9;
  AXNodeData text_10;
  AXNodeData gc_11;
  AXNodeData text_12;

  root_1.id = 1;
  text_2.id = 2;
  gc_3.id = 3;
  gc_4.id = 4;
  gc_5.id = 5;
  text_6.id = 6;
  gc_7.id = 7;
  text_8.id = 8;
  gc_9.id = 9;
  text_10.id = 10;
  gc_11.id = 11;
  text_12.id = 12;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {text_2.id, gc_3.id, gc_11.id};
  root_1.AddState(ax::mojom::State::kIgnored);

  text_2.role = ax::mojom::Role::kStaticText;
  text_2.SetName("one");

  gc_3.role = ax::mojom::Role::kGenericContainer;
  gc_3.AddState(ax::mojom::State::kIgnored);
  gc_3.child_ids = {gc_4.id};

  gc_4.role = ax::mojom::Role::kGenericContainer;
  gc_4.child_ids = {gc_5.id, gc_7.id, gc_9.id};

  gc_5.role = ax::mojom::Role::kGenericContainer;
  gc_5.child_ids = {text_6.id};

  text_6.role = ax::mojom::Role::kStaticText;
  text_6.SetName(" two");

  gc_7.role = ax::mojom::Role::kGenericContainer;
  gc_7.AddState(ax::mojom::State::kIgnored);
  gc_7.child_ids = {text_8.id};

  text_8.role = ax::mojom::Role::kStaticText;
  text_8.AddState(ax::mojom::State::kIgnored);
  text_8.SetName(" ignored");

  gc_9.role = ax::mojom::Role::kGenericContainer;
  gc_9.child_ids = {text_10.id};

  text_10.role = ax::mojom::Role::kStaticText;
  text_10.SetName(" three");

  gc_11.role = ax::mojom::Role::kGenericContainer;
  gc_11.child_ids = {text_12.id};

  text_12.role = ax::mojom::Role::kStaticText;
  text_12.SetName("four");

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1, text_2, gc_3, gc_4,    gc_5,  text_6,
                  gc_7,   text_8, gc_9, text_10, gc_11, text_12};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* text_6_node = tree->GetFromId(text_6.id);
  const AXNode* text_10_node = tree->GetFromId(text_10.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  // Create a range that spans " two three" located on the leaf nodes.

  // start: TextPosition, anchor_id=5, text_offset=0
  // end  : TextPosition, anchor_id=7, text_offset=6
  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor*/ text_6_node, /*start_offset*/ 2,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_10_node, /*end_offset*/ 6,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"wo three");

  // Delete |gc_3|, which will delete the entire subtree where both of our
  // endpoints are.
  AXTreeUpdate test_update;
  gc_3.child_ids = {};
  test_update.nodes = {gc_3};
  ASSERT_TRUE(GetTree()->Unserialize(test_update));

  // There was no unignored position in which to place the start and end - they
  // should now be null positions.
  EXPECT_TRUE(GetStart(range.Get())->IsNullPosition());
  EXPECT_TRUE(GetEnd(range.Get())->IsNullPosition());
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_TestDeleteSubtreeThatIncludesEndpointsNormalizeMoves) {
  // This test updates the tree structure to ensure that the text range is still
  // valid after a subtree that includes the text range is deleted, resulting in
  // a change to the range that is adjusted forwards due to an ignored node.
  //
  // ++1 kRootWebArea
  // ++++2 kStaticText "one"
  // ++++3 kGenericContainer ignored
  // ++++++4 kGenericContainer
  // ++++++++5 kGenericContainer
  // ++++++++++6 kStaticText " two"
  // ++++++++7 kGenericContainer
  // ++++++++++8 kStaticText " three"
  // ++++++++9 kGenericContainer ignored
  // ++++++++++10 kStaticText " ignored" ignored
  // ++++11 kGenericContainer
  // ++++++12 kStaticText "four"
  AXNodeData root_1;
  AXNodeData text_2;
  AXNodeData gc_3;
  AXNodeData gc_4;
  AXNodeData gc_5;
  AXNodeData text_6;
  AXNodeData gc_7;
  AXNodeData text_8;
  AXNodeData gc_9;
  AXNodeData text_10;
  AXNodeData gc_11;
  AXNodeData text_12;

  root_1.id = 1;
  text_2.id = 2;
  gc_3.id = 3;
  gc_4.id = 4;
  gc_5.id = 5;
  text_6.id = 6;
  gc_7.id = 7;
  text_8.id = 8;
  gc_9.id = 9;
  text_10.id = 10;
  gc_11.id = 11;
  text_12.id = 12;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {text_2.id, gc_3.id, gc_11.id};

  text_2.role = ax::mojom::Role::kStaticText;
  text_2.SetName("one");

  gc_3.role = ax::mojom::Role::kGenericContainer;
  gc_3.AddState(ax::mojom::State::kIgnored);
  gc_3.child_ids = {gc_4.id};

  gc_4.role = ax::mojom::Role::kGenericContainer;
  gc_4.child_ids = {gc_5.id, gc_7.id, gc_9.id};

  gc_5.role = ax::mojom::Role::kGenericContainer;
  gc_5.child_ids = {text_6.id};

  text_6.role = ax::mojom::Role::kStaticText;
  text_6.SetName(" two");

  gc_7.role = ax::mojom::Role::kGenericContainer;
  gc_7.child_ids = {text_8.id};

  text_8.role = ax::mojom::Role::kStaticText;
  text_8.SetName(" three");

  gc_9.role = ax::mojom::Role::kGenericContainer;
  gc_9.AddState(ax::mojom::State::kIgnored);
  gc_9.child_ids = {text_10.id};

  text_10.role = ax::mojom::Role::kStaticText;
  text_10.AddState(ax::mojom::State::kIgnored);
  text_10.SetName(" ignored");

  gc_11.role = ax::mojom::Role::kGenericContainer;
  gc_11.child_ids = {text_12.id};

  text_12.role = ax::mojom::Role::kStaticText;
  text_12.SetName("four");

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1, text_2, gc_3, gc_4,    gc_5,  text_6,
                  gc_7,   text_8, gc_9, text_10, gc_11, text_12};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* text_6_node = tree->GetFromId(text_6.id);
  const AXNode* text_8_node = tree->GetFromId(text_8.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  // Create a range that spans " two three" located on the leaf nodes.

  // start: TextPosition, anchor_id=5, text_offset=0
  // end  : TextPosition, anchor_id=7, text_offset=6
  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor*/ text_6_node, /*start_offset*/ 2,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_8_node, /*end_offset*/ 6,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"wo three");

  // Delete |gc_3|, which will delete the entire subtree where both of our
  // endpoints are.
  AXTreeUpdate test_update;
  gc_3.child_ids = {};
  test_update.nodes = {gc_3};
  ASSERT_TRUE(GetTree()->Unserialize(test_update));

  // The text range should now be a degenerate range positioned at the end of
  // root, the parent of |gc_3|, since |gc_3| has been deleted.
  EXPECT_EQ(text_12.id, GetStart(range.Get())->anchor_id());
  EXPECT_EQ(0, GetStart(range.Get())->text_offset());

  EXPECT_EQ(text_12.id, GetEnd(range.Get())->anchor_id());
  EXPECT_EQ(0, GetEnd(range.Get())->text_offset());
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestDeleteTreePositionPreviousSibling) {
  // This test creates a degenerate range with endpoints pointing after the last
  // child of the 2 generic container. It then deletes a previous sibling and
  // ensures that we don't crash with an out of bounds index that causes null
  // child positions to be created.
  //
  // ++1 kRootWebArea
  // ++++2 kGenericContainer
  // ++++++3 kHeading
  // ++++++++4 kStaticText
  // ++++++++++5 kInlineTextBox
  // ++++++6 kGenericContainer
  // ++++++7 kButton
  ui::AXNodeData root_1;
  ui::AXNodeData generic_container_2;
  ui::AXNodeData heading_3;
  ui::AXNodeData static_text_4;
  ui::AXNodeData inline_box_5;
  ui::AXNodeData generic_container_6;
  ui::AXNodeData button_7;

  root_1.id = 1;
  generic_container_2.id = 2;
  heading_3.id = 3;
  static_text_4.id = 4;
  inline_box_5.id = 5;
  generic_container_6.id = 6;
  button_7.id = 7;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {generic_container_2.id};

  generic_container_2.role = ax::mojom::Role::kGenericContainer;
  generic_container_2.child_ids = {heading_3.id, generic_container_6.id,
                                   button_7.id};

  heading_3.role = ax::mojom::Role::kHeading;
  heading_3.child_ids = {static_text_4.id};

  static_text_4.role = ax::mojom::Role::kStaticText;
  static_text_4.child_ids = {inline_box_5.id};
  static_text_4.SetName("3.14");

  inline_box_5.role = ax::mojom::Role::kInlineTextBox;
  inline_box_5.SetName("3.14");

  generic_container_6.role = ax::mojom::Role::kGenericContainer;
  generic_container_6.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  button_7.role = ax::mojom::Role::kButton;

  ui::AXTreeUpdate update;
  ui::AXTreeData tree_data;
  tree_data.tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.tree_data = tree_data;
  update.has_tree_data = true;
  update.root_id = root_1.id;
  update.nodes = {root_1,       generic_container_2, heading_3, static_text_4,
                  inline_box_5, generic_container_6, button_7};

  Init(update);
  AXTree* tree = GetTree();

  AXNode* root_node = GetRootAsAXNode();
  AXNodePosition::AXPositionInstance range_start =
      AXNodePosition::CreateTreePosition(tree->GetAXTreeID(),
                                         generic_container_2.id,
                                         /*child_index*/ 3);
  AXNodePosition::AXPositionInstance range_end = range_start->Clone();

  AXPlatformNodeWin* owner =
      static_cast<AXPlatformNodeWin*>(AXPlatformNodeFromNode(root_node));
  ComPtr<ITextRangeProvider> text_range_provider =
      AXPlatformNodeTextRangeProviderWin::CreateTextRangeProviderForTesting(
          owner, std::move(range_start), std::move(range_end));
  EXPECT_UIA_TEXTRANGE_EQ(text_range_provider, L"");

  generic_container_2.child_ids = {heading_3.id, button_7.id};
  AXTreeUpdate test_update;
  test_update.nodes = {generic_container_2};
  ASSERT_TRUE(tree->Unserialize(test_update));

  root_1.child_ids = {};
  test_update.nodes = {root_1};
  ASSERT_TRUE(tree->Unserialize(test_update));
}

TEST_F(AXPlatformNodeTextRangeProviderTest,
       TestReplaceStartAndEndEndpointRepeatRemoval) {
  // This test updates the tree structure to ensure that the text range is still
  // valid after text nodes get removed repeatedly.
  //
  // ++1 kRootWebArea
  // ++++2 kStaticText
  // ++++3 kGroup (ignored)
  // ++++++4 kStaticText
  // ++++5 kStaticText
  AXNodeData root_1;
  AXNodeData text_2;
  AXNodeData group_3;
  AXNodeData text_4;
  AXNodeData text_5;

  root_1.id = 1;
  text_2.id = 2;
  group_3.id = 3;
  text_4.id = 4;
  text_5.id = 5;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {text_2.id, group_3.id, text_5.id};

  text_2.role = ax::mojom::Role::kStaticText;
  text_2.SetName("text 2");

  group_3.role = ax::mojom::Role::kGroup;
  group_3.AddState(ax::mojom::State::kIgnored);
  group_3.child_ids = {text_4.id};

  text_4.role = ax::mojom::Role::kStaticText;
  text_4.SetName("text 4");

  text_5.role = ax::mojom::Role::kStaticText;
  text_5.SetName("text 5");

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1, text_2, group_3, text_4, text_5};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* text_2_node = tree->GetFromId(text_2.id);
  const AXNode* text_4_node = tree->GetFromId(text_4.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor*/ text_2_node, /*start_offset*/ 0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_4_node, /*end_offset*/ 0,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"text 2");

  // start: TextPosition, anchor_id=2, text_offset=0, annotated_text=<t>ext2
  // end  : TextPosition, anchor_id=4, text_offset=0, annotated_text=<>text4
  // 1. Remove |text_4| which |end_| is anchored on.
  {
    // Remove node |text_4|.
    group_3.child_ids = {};
    AXTreeUpdate test_update;
    test_update.nodes = {root_1, group_3};
    ASSERT_TRUE(GetTree()->Unserialize(test_update));

    // Replacing that node should not impact the range.
    EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"text 2");
  }

  // start: TextPosition, anchor_id=2, text_offset=0, annotated_text=<>text2
  // end  : TextPosition, anchor_id=2, text_offset=5, annotated_text=text2<>
  // 2. Remove |text_2|, which both |start_| and |end_| are anchored to and
  //  replace with |text_5|.
  {
    root_1.child_ids = {group_3.id, text_5.id};
    AXTreeUpdate test_update;
    test_update.nodes = {root_1, group_3};
    ASSERT_TRUE(GetTree()->Unserialize(test_update));

    // Removing that node should adjust the range to the |text_5|, as it took
    // |text_2|'s position.
    EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"text 5");
  }

  // start: TextPosition, anchor_id=5, text_offset=0, annotated_text=<>text5
  // end  : TextPosition, anchor_id=5, text_offset=5, annotated_text=text5<>
  // 3. Remove |text_5|, which both |start_| and |end_| are pointing to.
  {
    root_1.child_ids = {group_3.id};
    AXTreeUpdate test_update;
    test_update.nodes = {root_1, group_3};
    ASSERT_TRUE(GetTree()->Unserialize(test_update));

    // Removing the last text node should leave a degenerate range.
    EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"");
  }
}

TEST_F(AXPlatformNodeTextRangeProviderTest, CaretAtEndOfTextFieldReadOnly) {
  // This test places a degenerate range at end of text field, and it should not
  // normalize to other positions, so we should expect the
  // 'UIA_IsReadOnlyAttributeId' attribute queried at this position to return
  // false.
  // ++1 kRootWebArea
  // ++++2 kTextField editable value="hello"
  // ++++++3 kGenericContainer editable isLineBreakingObject=true
  // ++++++++4 kStaticText editable name="hello"
  // ++++++++++5 kInlineTextBox editable name="hello"
  // ++++6 kStaticText name="abc"
  // ++++++7 kInlineTextBox name="abc"
  AXNodeData root_1;
  AXNodeData text_field_2;
  AXNodeData generic_container_3;
  AXNodeData static_text_4;
  AXNodeData inline_text_5;
  AXNodeData static_text_6;
  AXNodeData inline_text_7;

  root_1.id = 1;
  text_field_2.id = 2;
  generic_container_3.id = 3;
  static_text_4.id = 4;
  inline_text_5.id = 5;
  static_text_6.id = 6;
  inline_text_7.id = 7;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {text_field_2.id, static_text_6.id};

  text_field_2.role = ax::mojom::Role::kTextField;
  text_field_2.AddState(ax::mojom::State::kEditable);
  text_field_2.SetValue("hello");
  text_field_2.child_ids = {generic_container_3.id};

  generic_container_3.role = ax::mojom::Role::kGenericContainer;
  generic_container_3.AddState(ax::mojom::State::kEditable);
  generic_container_3.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  generic_container_3.child_ids = {static_text_4.id};

  static_text_4.role = ax::mojom::Role::kStaticText;
  static_text_4.SetName("hello");
  static_text_4.AddState(ax::mojom::State::kEditable);
  static_text_4.child_ids = {inline_text_5.id};

  inline_text_5.role = ax::mojom::Role::kInlineTextBox;
  inline_text_5.SetName("hello");
  inline_text_5.AddState(ax::mojom::State::kEditable);

  static_text_6.role = ax::mojom::Role::kStaticText;
  static_text_6.SetName("abc");
  static_text_6.child_ids = {inline_text_7.id};

  inline_text_7.role = ax::mojom::Role::kInlineTextBox;
  inline_text_7.SetName("abc");

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1,        text_field_2,  generic_container_3,
                  static_text_4, inline_text_5, static_text_6,
                  inline_text_7};

  Init(update);
  const AXTree* tree = GetTree();
  const AXNode* inline_text_5_node = tree->GetFromId(inline_text_5.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  base::win::ScopedVariant expected_variant;

  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor*/ inline_text_5_node, /*start_offset*/ 3,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ inline_text_5_node, /*end_offset*/ 4,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"l");

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(range, UIA_IsReadOnlyAttributeId,
                              expected_variant);

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(range, TextPatternRangeEndpoint_Start,
                                   TextUnit_Character,
                                   /*count*/ 1,
                                   /*expected_text*/ L"",
                                   /*expected_count*/ 1);
  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(range, UIA_IsReadOnlyAttributeId,
                              expected_variant);

  EXPECT_UIA_MOVE(range, TextUnit_Character,
                  /*count*/ 1,
                  /*expected_text*/
                  L"",
                  /*expected_count*/ 1);
  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(range, UIA_IsReadOnlyAttributeId,
                              expected_variant);
  const AXNodePosition::AXPositionInstance& start = GetStart(range.Get());
  const AXNodePosition::AXPositionInstance& end = GetEnd(range.Get());
  EXPECT_TRUE(start->AtEndOfAnchor());
  EXPECT_EQ(5, start->anchor_id());
  EXPECT_EQ(5, start->text_offset());

  EXPECT_TRUE(end->AtEndOfAnchor());
  EXPECT_EQ(5, end->anchor_id());
  EXPECT_EQ(5, end->text_offset());
}

// TODO(schectman) Not all attributes treated as in Chromium.
// https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_GeneratedNewlineReturnsCommonAnchorReadonly) {
  // This test places a range that starts at the end of a paragraph and
  // ends at the beginning of the next paragraph. The range only contains the
  // generated newline character. The readonly attribute value returned should
  // be the one of the common anchor of the start and end endpoint.

  // ++1 kRootWebArea
  // ++++2 kGenericContainer
  // ++++++3 kImage
  // ++++++4 kTextField editable
  // ++++5 kGenericContainer editable
  // ++++++6 kImage
  // ++++++7 kTextField editable
  // ++++8 kGenericContainer
  // ++++++9 kTextField editable
  // ++++++10 kTextField editable
  AXNodeData root_1;
  AXNodeData generic_container_2;
  AXNodeData image_3;
  AXNodeData text_field_4;
  AXNodeData generic_container_5;
  AXNodeData image_6;
  AXNodeData text_field_7;
  AXNodeData generic_container_8;
  AXNodeData text_field_9;
  AXNodeData text_field_10;

  root_1.id = 1;
  generic_container_2.id = 2;
  image_3.id = 3;
  text_field_4.id = 4;
  generic_container_5.id = 5;
  image_6.id = 6;
  text_field_7.id = 7;
  generic_container_8.id = 8;
  text_field_9.id = 9;
  text_field_10.id = 10;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {generic_container_2.id, generic_container_5.id,
                      generic_container_8.id};

  generic_container_2.role = ax::mojom::Role::kGenericContainer;
  generic_container_2.child_ids = {image_3.id, text_field_4.id};

  image_3.role = ax::mojom::Role::kImage;
  image_3.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                           true);

  text_field_4.role = ax::mojom::Role::kTextField;
  text_field_4.AddState(ax::mojom::State::kEditable);

  generic_container_5.role = ax::mojom::Role::kGenericContainer;
  generic_container_5.AddState(ax::mojom::State::kEditable);
  generic_container_5.child_ids = {image_6.id, text_field_7.id};

  image_6.role = ax::mojom::Role::kImage;
  image_6.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                           true);

  text_field_7.role = ax::mojom::Role::kTextField;
  text_field_7.AddState(ax::mojom::State::kEditable);

  generic_container_8.role = ax::mojom::Role::kGenericContainer;
  generic_container_8.child_ids = {text_field_9.id, text_field_10.id};

  text_field_9.role = ax::mojom::Role::kTextField;
  text_field_9.AddState(ax::mojom::State::kEditable);
  text_field_9.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                                true);

  text_field_10.role = ax::mojom::Role::kTextField;
  text_field_10.AddState(ax::mojom::State::kEditable);

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1,       generic_container_2, image_3,
                  text_field_4, generic_container_5, image_6,
                  text_field_7, generic_container_8, text_field_9,
                  text_field_10};

  Init(update);
  const AXTree* tree = GetTree();

  const AXNode* image_3_node = tree->GetFromId(image_3.id);
  const AXNode* image_6_node = tree->GetFromId(image_6.id);
  const AXNode* text_field_4_node = tree->GetFromId(text_field_4.id);
  const AXNode* text_field_7_node = tree->GetFromId(text_field_7.id);
  const AXNode* text_field_9_node = tree->GetFromId(text_field_9.id);
  const AXNode* text_field_10_node = tree->GetFromId(text_field_10.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  base::win::ScopedVariant expected_variant;

  ComPtr<AXPlatformNodeTextRangeProviderWin> range_1;
  CreateTextRangeProviderWin(
      range_1, owner,
      /*start_anchor*/ image_3_node, /*start_offset*/ 1,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_field_4_node, /*end_offset*/ 0,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range_1, /*expected_text*/ L"");

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(range_1, UIA_IsReadOnlyAttributeId,
                              expected_variant);
  expected_variant.Reset();

  ComPtr<AXPlatformNodeTextRangeProviderWin> range_2;
  CreateTextRangeProviderWin(
      range_2, owner,
      /*start_anchor*/ image_6_node, /*start_offset*/ 1,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_field_7_node, /*end_offset*/ 0,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range_2, /*expected_text*/ L"");

  expected_variant.Set(false);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(range_2, UIA_IsReadOnlyAttributeId,
                              expected_variant);
  expected_variant.Reset();

  // This is testing a corner case when the range spans two text fields
  // separated by a paragraph boundary. This case used to not work because we
  // were relying on NormalizeTextRange to handle generated newlines and
  // normalization doesn't work when the range spans text fields.
  ComPtr<AXPlatformNodeTextRangeProviderWin> range_3;
  CreateTextRangeProviderWin(
      range_3, owner,
      /*start_anchor*/ text_field_9_node, /*start_offset*/ 1,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ text_field_10_node, /*end_offset*/ 0,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range_3, /*expected_text*/ L"");

  expected_variant.Set(true);
  EXPECT_UIA_TEXTATTRIBUTE_EQ(range_3, UIA_IsReadOnlyAttributeId,
                              expected_variant);
  expected_variant.Reset();
}

// TODO(schectman) https://github.com/flutter/flutter/issues/117012
TEST_F(AXPlatformNodeTextRangeProviderTest,
       DISABLED_MoveEndpointToLastIgnoredForTextNavigationNode) {
  // This test moves the end endpoint of a range by one paragraph unit forward
  // to the last node of the tree. That last node happens to be a node that is
  // ignored for text navigation, but since it's the last node in the tree, it
  // should successfully move the endpoint to that node and keep the units_moved
  // value in sync.
  // ++1 kRootWebArea
  // ++++2 kStaticText name="abc"
  // ++++++3 kInlineTextBox name="abc"
  // ++++4 kGenericContainer
  AXNodeData root_1;
  AXNodeData static_text_2;
  AXNodeData inline_text_3;
  AXNodeData generic_container_4;

  root_1.id = 1;
  static_text_2.id = 2;
  inline_text_3.id = 3;
  generic_container_4.id = 4;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {static_text_2.id, generic_container_4.id};

  static_text_2.role = ax::mojom::Role::kStaticText;
  static_text_2.SetName("abc");
  static_text_2.child_ids = {inline_text_3.id};

  inline_text_3.role = ax::mojom::Role::kInlineTextBox;
  inline_text_3.SetName("abc");

  generic_container_4.role = ax::mojom::Role::kGenericContainer;

  ui::AXTreeUpdate update;
  ui::AXTreeID tree_id = ui::AXTreeID::CreateNewAXTreeID();
  update.root_id = root_1.id;
  update.tree_data.tree_id = tree_id;
  update.has_tree_data = true;
  update.nodes = {root_1, static_text_2, inline_text_3, generic_container_4};

  Init(update);
  const AXTree* tree = GetTree();
  const AXNode* inline_text_3_node = tree->GetFromId(inline_text_3.id);

  // Making |owner| AXID:1 so that |TestAXNodeWrapper::BuildAllWrappers|
  // will build the entire tree.
  AXPlatformNodeWin* owner = static_cast<AXPlatformNodeWin*>(
      AXPlatformNodeFromNode(GetNodeFromTree(tree_id, 1)));

  ComPtr<AXPlatformNodeTextRangeProviderWin> range;
  base::win::ScopedVariant expected_variant;

  CreateTextRangeProviderWin(
      range, owner,
      /*start_anchor*/ inline_text_3_node, /*start_offset*/ 0,
      /*start_affinity*/ ax::mojom::TextAffinity::kDownstream,
      /*end_anchor*/ inline_text_3_node, /*end_offset*/ 3,
      /*end_affinity*/ ax::mojom::TextAffinity::kDownstream);

  EXPECT_UIA_TEXTRANGE_EQ(range, /*expected_text*/ L"abc");

  EXPECT_UIA_MOVE_ENDPOINT_BY_UNIT(range, TextPatternRangeEndpoint_End,
                                   TextUnit_Paragraph,
                                   /*count*/ 1,
                                   /*expected_text*/ L"abc\xFFFC",
                                   /*expected_count*/ 1);
}

}  // namespace ui
