//
// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_WIN_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_WIN_H_

#include <atlbase.h>
#include <atlcom.h>
#include <objbase.h>
#include <oleacc.h>
#include <oleauto.h>
#include <uiautomation.h>
#include <wrl/client.h>

#include <array>
#include <map>
#include <string>
#include <vector>

#include "ax/ax_export.h"
#include "ax/platform/ax_platform_node_base.h"
#include "base/compiler_specific.h"
#include "gfx/range/range.h"

//
// Macros to use at the top of any AXPlatformNodeWin (or derived class) method
// that implements a UIA COM interface. The error code UIA_E_ELEMENTNOTAVAILABLE
// signals to the OS that the object is no longer valid and no further methods
// should be called on it.
//
#define UIA_VALIDATE_CALL()               \
  if (!AXPlatformNodeBase::GetDelegate()) \
    return UIA_E_ELEMENTNOTAVAILABLE;
#define UIA_VALIDATE_CALL_1_ARG(arg)      \
  if (!AXPlatformNodeBase::GetDelegate()) \
    return UIA_E_ELEMENTNOTAVAILABLE;     \
  if (!arg)                               \
    return E_INVALIDARG;                  \
  *arg = {};

namespace base {
namespace win {
class VariantVector;
}  // namespace win
}  // namespace base

namespace ui {

class AXPlatformNodeWin;
// TODO(nektar): Remove multithread superclass since we don't support it.
class AX_EXPORT __declspec(uuid("26f5641a-246d-457b-a96d-07f3fae6acf2"))
    AXPlatformNodeWin : public CComObjectRootEx<CComMultiThreadModel>,
                        public IDispatchImpl<IAccessible>,
                        public IExpandCollapseProvider,
                        public IGridItemProvider,
                        public IGridProvider,
                        public IInvokeProvider,
                        public IRangeValueProvider,
                        public IRawElementProviderFragment,
                        public IRawElementProviderSimple2,
                        public IScrollItemProvider,
                        public IScrollProvider,
                        public ISelectionItemProvider,
                        public ISelectionProvider,
                        public IServiceProvider,
                        public ITableItemProvider,
                        public ITableProvider,
                        public IToggleProvider,
                        public IValueProvider,
                        public IWindowProvider,
                        public AXPlatformNodeBase {
  using IDispatchImpl::Invoke;

 public:
  BEGIN_COM_MAP(AXPlatformNodeWin)
  // TODO(nektar): Find a way to remove the following entry because it's not
  // an interface.
  COM_INTERFACE_ENTRY(AXPlatformNodeWin)
  COM_INTERFACE_ENTRY(IAccessible)
  COM_INTERFACE_ENTRY(IDispatch)
  COM_INTERFACE_ENTRY(IExpandCollapseProvider)
  COM_INTERFACE_ENTRY(IGridItemProvider)
  COM_INTERFACE_ENTRY(IGridProvider)
  COM_INTERFACE_ENTRY(IInvokeProvider)
  COM_INTERFACE_ENTRY(IRangeValueProvider)
  COM_INTERFACE_ENTRY(IRawElementProviderFragment)
  COM_INTERFACE_ENTRY(IRawElementProviderSimple)
  COM_INTERFACE_ENTRY(IRawElementProviderSimple2)
  COM_INTERFACE_ENTRY(IScrollItemProvider)
  COM_INTERFACE_ENTRY(IScrollProvider)
  COM_INTERFACE_ENTRY(ISelectionItemProvider)
  COM_INTERFACE_ENTRY(ISelectionProvider)
  COM_INTERFACE_ENTRY(ITableItemProvider)
  COM_INTERFACE_ENTRY(ITableProvider)
  COM_INTERFACE_ENTRY(IToggleProvider)
  COM_INTERFACE_ENTRY(IValueProvider)
  COM_INTERFACE_ENTRY(IWindowProvider)
  COM_INTERFACE_ENTRY(IServiceProvider)
  END_COM_MAP()

  ~AXPlatformNodeWin() override;

  void Init(AXPlatformNodeDelegate* delegate) override;

  // AXPlatformNode overrides.
  gfx::NativeViewAccessible GetNativeViewAccessible() override;
  void NotifyAccessibilityEvent(ax::mojom::Event event_type) override;

  // AXPlatformNodeBase overrides.
  void Destroy() override;
  std::u16string GetValue() const override;
  bool IsPlatformCheckable() const override;

  //
  // IAccessible methods.
  //

  // Retrieves the child element or child object at a given point on the screen.
  IFACEMETHODIMP accHitTest(LONG screen_physical_pixel_x,
                            LONG screen_physical_pixel_y,
                            VARIANT* child) override;

  // Performs the object's default action.
  IFACEMETHODIMP accDoDefaultAction(VARIANT var_id) override;

  // Retrieves the specified object's current screen location.
  IFACEMETHODIMP accLocation(LONG* physical_pixel_left,
                             LONG* physical_pixel_top,
                             LONG* width,
                             LONG* height,
                             VARIANT var_id) override;

  // Traverses to another UI element and retrieves the object.
  IFACEMETHODIMP accNavigate(LONG nav_dir,
                             VARIANT start,
                             VARIANT* end) override;

  // Retrieves an IDispatch interface pointer for the specified child.
  IFACEMETHODIMP get_accChild(VARIANT var_child,
                              IDispatch** disp_child) override;

  // Retrieves the number of accessible children.
  IFACEMETHODIMP get_accChildCount(LONG* child_count) override;

  // Retrieves a string that describes the object's default action.
  IFACEMETHODIMP get_accDefaultAction(VARIANT var_id,
                                      BSTR* default_action) override;

  // Retrieves the tooltip description.
  IFACEMETHODIMP get_accDescription(VARIANT var_id, BSTR* desc) override;

  // Retrieves the object that has the keyboard focus.
  IFACEMETHODIMP get_accFocus(VARIANT* focus_child) override;

  // Retrieves the specified object's shortcut.
  IFACEMETHODIMP get_accKeyboardShortcut(VARIANT var_id,
                                         BSTR* access_key) override;

  // Retrieves the name of the specified object.
  IFACEMETHODIMP get_accName(VARIANT var_id, BSTR* name) override;

  // Retrieves the IDispatch interface of the object's parent.
  IFACEMETHODIMP get_accParent(IDispatch** disp_parent) override;

  // Retrieves information describing the role of the specified object.
  IFACEMETHODIMP get_accRole(VARIANT var_id, VARIANT* role) override;

  // Retrieves the current state of the specified object.
  IFACEMETHODIMP get_accState(VARIANT var_id, VARIANT* state) override;

  // Gets the help string for the specified object.
  IFACEMETHODIMP get_accHelp(VARIANT var_id, BSTR* help) override;

  // Retrieve or set the string value associated with the specified object.
  // Setting the value is not typically used by screen readers, but it's
  // used frequently by automation software.
  IFACEMETHODIMP get_accValue(VARIANT var_id, BSTR* value) override;
  IFACEMETHODIMP put_accValue(VARIANT var_id, BSTR new_value) override;

  // IAccessible methods not implemented.
  IFACEMETHODIMP get_accSelection(VARIANT* selected) override;
  IFACEMETHODIMP accSelect(LONG flags_sel, VARIANT var_id) override;
  IFACEMETHODIMP get_accHelpTopic(BSTR* help_file,
                                  VARIANT var_id,
                                  LONG* topic_id) override;
  IFACEMETHODIMP put_accName(VARIANT var_id, BSTR put_name) override;

  //
  // IExpandCollapseProvider methods.
  //

  IFACEMETHODIMP Collapse() override;

  IFACEMETHODIMP Expand() override;

  IFACEMETHODIMP get_ExpandCollapseState(ExpandCollapseState* result) override;

  //
  // IGridItemProvider methods.
  //

  IFACEMETHODIMP get_Column(int* result) override;

  IFACEMETHODIMP get_ColumnSpan(int* result) override;

  IFACEMETHODIMP get_ContainingGrid(
      IRawElementProviderSimple** result) override;

  IFACEMETHODIMP get_Row(int* result) override;

  IFACEMETHODIMP get_RowSpan(int* result) override;

  //
  // IGridProvider methods.
  //

  IFACEMETHODIMP GetItem(int row,
                         int column,
                         IRawElementProviderSimple** result) override;

  IFACEMETHODIMP get_RowCount(int* result) override;

  IFACEMETHODIMP get_ColumnCount(int* result) override;

  //
  // IInvokeProvider methods.
  //

  IFACEMETHODIMP Invoke() override;

  //
  // IScrollItemProvider methods.
  //

  IFACEMETHODIMP ScrollIntoView() override;

  //
  // IScrollProvider methods.
  //

  IFACEMETHODIMP Scroll(ScrollAmount horizontal_amount,
                        ScrollAmount vertical_amount) override;

  IFACEMETHODIMP SetScrollPercent(double horizontal_percent,
                                  double vertical_percent) override;

  IFACEMETHODIMP get_HorizontallyScrollable(BOOL* result) override;

  IFACEMETHODIMP get_HorizontalScrollPercent(double* result) override;

  // Horizontal size of the viewable region as a percentage of the total content
  // area.
  IFACEMETHODIMP get_HorizontalViewSize(double* result) override;

  IFACEMETHODIMP get_VerticallyScrollable(BOOL* result) override;

  IFACEMETHODIMP get_VerticalScrollPercent(double* result) override;

  // Vertical size of the viewable region as a percentage of the total content
  // area.
  IFACEMETHODIMP get_VerticalViewSize(double* result) override;

  //
  // ISelectionItemProvider methods.
  //

  IFACEMETHODIMP AddToSelection() override;

  IFACEMETHODIMP RemoveFromSelection() override;

  IFACEMETHODIMP Select() override;

  IFACEMETHODIMP get_IsSelected(BOOL* result) override;

  IFACEMETHODIMP get_SelectionContainer(
      IRawElementProviderSimple** result) override;

  //
  // ISelectionProvider methods.
  //

  IFACEMETHODIMP GetSelection(SAFEARRAY** result) override;

  IFACEMETHODIMP get_CanSelectMultiple(BOOL* result) override;

  IFACEMETHODIMP get_IsSelectionRequired(BOOL* result) override;

  //
  // ITableItemProvider methods.
  //

  IFACEMETHODIMP GetColumnHeaderItems(SAFEARRAY** result) override;

  IFACEMETHODIMP GetRowHeaderItems(SAFEARRAY** result) override;

  //
  // ITableProvider methods.
  //

  IFACEMETHODIMP GetColumnHeaders(SAFEARRAY** result) override;

  IFACEMETHODIMP GetRowHeaders(SAFEARRAY** result) override;

  IFACEMETHODIMP get_RowOrColumnMajor(RowOrColumnMajor* result) override;

  //
  // IToggleProvider methods.
  //

  IFACEMETHODIMP Toggle() override;

  IFACEMETHODIMP get_ToggleState(ToggleState* result) override;

  //
  // IValueProvider methods.
  //

  IFACEMETHODIMP SetValue(LPCWSTR val) override;

  IFACEMETHODIMP get_IsReadOnly(BOOL* result) override;

  IFACEMETHODIMP get_Value(BSTR* result) override;

  //
  // IWindowProvider methods.
  //

  IFACEMETHODIMP SetVisualState(WindowVisualState window_visual_state) override;

  IFACEMETHODIMP Close() override;

  IFACEMETHODIMP WaitForInputIdle(int milliseconds, BOOL* result) override;

  IFACEMETHODIMP get_CanMaximize(BOOL* result) override;

  IFACEMETHODIMP get_CanMinimize(BOOL* result) override;

  IFACEMETHODIMP get_IsModal(BOOL* result) override;

  IFACEMETHODIMP get_WindowVisualState(WindowVisualState* result) override;

  IFACEMETHODIMP get_WindowInteractionState(
      WindowInteractionState* result) override;

  IFACEMETHODIMP get_IsTopmost(BOOL* result) override;

  //
  // IRangeValueProvider methods.
  //

  IFACEMETHODIMP SetValue(double val) override;

  IFACEMETHODIMP get_LargeChange(double* result) override;

  IFACEMETHODIMP get_Maximum(double* result) override;

  IFACEMETHODIMP get_Minimum(double* result) override;

  IFACEMETHODIMP get_SmallChange(double* result) override;

  IFACEMETHODIMP get_Value(double* result) override;

  //
  // IRawElementProviderFragment methods.
  //

  IFACEMETHODIMP Navigate(
      NavigateDirection direction,
      IRawElementProviderFragment** element_provider) override;
  IFACEMETHODIMP GetRuntimeId(SAFEARRAY** runtime_id) override;
  IFACEMETHODIMP get_BoundingRectangle(
      UiaRect* screen_physical_pixel_bounds) override;
  IFACEMETHODIMP GetEmbeddedFragmentRoots(
      SAFEARRAY** embedded_fragment_roots) override;
  IFACEMETHODIMP SetFocus() override;
  IFACEMETHODIMP get_FragmentRoot(
      IRawElementProviderFragmentRoot** fragment_root) override;

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

  //
  // IRawElementProviderSimple2 methods.
  //

  IFACEMETHODIMP ShowContextMenu() override;

  //
  // IServiceProvider methods.
  //

  IFACEMETHODIMP QueryService(REFGUID guidService,
                              REFIID riid,
                              void** object) override;

  //
  // Methods used by the ATL COM map.
  //

  // Called by BEGIN_COM_MAP() / END_COM_MAP().
  static STDMETHODIMP InternalQueryInterface(void* this_ptr,
                                             const _ATL_INTMAP_ENTRY* entries,
                                             REFIID riid,
                                             void** object);

  // Support method for ITextRangeProvider::GetAttributeValue.
  // If either |start_offset| or |end_offset| are not provided then the
  // endpoint is treated as the start or end of the node respectively.
  HRESULT GetTextAttributeValue(TEXTATTRIBUTEID attribute_id,
                                const std::optional<int>& start_offset,
                                const std::optional<int>& end_offset,
                                base::win::VariantVector* result);

  // IRawElementProviderSimple support method.
  bool IsPatternProviderSupported(PATTERNID pattern_id);

  // Prefer GetPatternProviderImpl when calling internally. We should avoid
  // calling external APIs internally as it will cause the histograms to become
  // innaccurate.
  HRESULT GetPatternProviderImpl(PATTERNID pattern_id, IUnknown** result);

  // Prefer GetPropertyValueImpl when calling internally. We should avoid
  // calling external APIs internally as it will cause the histograms to become
  // innaccurate.
  HRESULT GetPropertyValueImpl(PROPERTYID property_id, VARIANT* result);

  // Helper to return the runtime id (without going through a SAFEARRAY)
  using RuntimeIdArray = std::array<int, 2>;
  void GetRuntimeIdArray(RuntimeIdArray& runtime_id);

  // Updates the active composition range and fires UIA text edit event about
  // composition (active or committed)
  void OnActiveComposition(const gfx::Range& range,
                           const std::u16string& active_composition_text,
                           bool is_composition_committed);
  // Returns true if there is an active composition
  bool HasActiveComposition() const;
  // Returns the start/end offsets of the active composition
  gfx::Range GetActiveCompositionOffsets() const;

  // Helper to recursively find live-regions and fire a change event on them
  void FireLiveRegionChangeRecursive();

  // Returns the parent node that makes this node inaccessible.
  AXPlatformNodeWin* GetLowestAccessibleElement();

  // Returns the first |IsTextOnlyObject| descendant using
  // depth-first pre-order traversal.
  AXPlatformNodeWin* GetFirstTextOnlyDescendant();

  // Convert a mojo event to an MSAA event. Exposed for testing.
  static std::optional<DWORD> MojoEventToMSAAEvent(ax::mojom::Event event);

  // Convert a mojo event to a UIA event. Exposed for testing.
  static std::optional<EVENTID> MojoEventToUIAEvent(ax::mojom::Event event);

  // Convert a mojo event to a UIA property id. Exposed for testing.
  std::optional<PROPERTYID> MojoEventToUIAProperty(ax::mojom::Event event);

  // |AXPlatformNodeBase|
  bool IsDescendantOf(AXPlatformNode* ancestor) const override;

 protected:
  // This is hard-coded; all products based on the Chromium engine will have the
  // same framework name, so that assistive technology can detect any
  // Chromium-based product.
  static constexpr const wchar_t* FRAMEWORK_ID = L"Chrome";

  AXPlatformNodeWin();

  int MSAAState() const;

  int MSAARole();

  std::u16string UIAAriaRole();

  std::u16string ComputeUIAProperties();

  LONG ComputeUIAControlType();

  AXPlatformNodeWin* ComputeUIALabeledBy();

  bool CanHaveUIALabeledBy();

  bool IsNameExposed() const;

  bool IsUIAControl() const;

  std::optional<LONG> ComputeUIALandmarkType() const;

  bool IsInaccessibleDueToAncestor() const;

  bool ShouldHideChildrenForUIA() const;

  ExpandCollapseState ComputeExpandCollapseState() const;

  // AXPlatformNodeBase overrides.
  void Dispose() override;

  AXHypertext old_hypertext_;

  // These protected methods are still used by BrowserAccessibilityComWin. At
  // some point post conversion, we can probably move these to be private
  // methods.

  // A helper to add the given string value to |attributes|.
  void AddAttributeToList(const char* name,
                          const char* value,
                          PlatformAttributeList* attributes) override;

 private:
  bool IsWebAreaForPresentationalIframe();
  bool ShouldNodeHaveFocusableState(const AXNodeData& data) const;

  // Get the value attribute as a Bstr, this means something different depending
  // on the type of element being queried. (e.g. kColorWell uses kColorValue).
  static BSTR GetValueAttributeAsBstr(AXPlatformNodeWin* target);

  HRESULT GetStringAttributeAsBstr(ax::mojom::StringAttribute attribute,
                                   BSTR* value_bstr) const;

  HRESULT GetNameAsBstr(BSTR* value_bstr) const;

  // Escapes characters in string attributes as required by the UIA Aria
  // Property Spec. It's okay for input to be the same as output.
  static void SanitizeStringAttributeForUIAAriaProperty(
      const std::u16string& input,
      std::u16string* output);

  // If the string attribute |attribute| is present, add its value as a
  // UIA AriaProperties Property with the name |uia_aria_property|.
  void StringAttributeToUIAAriaProperty(std::vector<std::u16string>& properties,
                                        ax::mojom::StringAttribute attribute,
                                        const char* uia_aria_property);

  // If the bool attribute |attribute| is present, add its value as a
  // UIA AriaProperties Property with the name |uia_aria_property|.
  void BoolAttributeToUIAAriaProperty(std::vector<std::u16string>& properties,
                                      ax::mojom::BoolAttribute attribute,
                                      const char* uia_aria_property);

  // If the int attribute |attribute| is present, add its value as a
  // UIA AriaProperties Property with the name |uia_aria_property|.
  void IntAttributeToUIAAriaProperty(std::vector<std::u16string>& properties,
                                     ax::mojom::IntAttribute attribute,
                                     const char* uia_aria_property);

  // If the float attribute |attribute| is present, add its value as a
  // UIA AriaProperties Property with the name |uia_aria_property|.
  void FloatAttributeToUIAAriaProperty(std::vector<std::u16string>& properties,
                                       ax::mojom::FloatAttribute attribute,
                                       const char* uia_aria_property);

  // If the state |state| exists, set the
  // UIA AriaProperties Property with the name |uia_aria_property| to "true".
  // Otherwise set the AriaProperties Property to "false".
  void StateToUIAAriaProperty(std::vector<std::u16string>& properties,
                              ax::mojom::State state,
                              const char* uia_aria_property);

  // If the Html attribute |html_attribute_name| is present, add its value as a
  // UIA AriaProperties Property with the name |uia_aria_property|.
  void HtmlAttributeToUIAAriaProperty(std::vector<std::u16string>& properties,
                                      const char* html_attribute_name,
                                      const char* uia_aria_property);

  // If the IntList attribute |attribute| is present, return an array
  // of automation elements referenced by the ids in the
  // IntList attribute. Otherwise return an empty array.
  // The function will skip over any ids that cannot be resolved.
  SAFEARRAY* CreateUIAElementsArrayForRelation(
      const ax::mojom::IntListAttribute& attribute);

  // Return an array of automation elements based on the attribute
  // IntList::kControlsIds for web content and IntAttribute::kViewPopupId. These
  // two attributes denote the controllees, web content elements and view popup
  // element respectively.
  // The function will skip over any ids that cannot be resolved.
  SAFEARRAY* CreateUIAControllerForArray();

  // Return an unordered array of automation elements which reference this node
  // for the given attribute.
  SAFEARRAY* CreateUIAElementsArrayForReverseRelation(
      const ax::mojom::IntListAttribute& attribute);

  // Return a vector of AXPlatformNodeWin referenced by the ids in function
  // argument. The function will skip over any ids that cannot be resolved as
  // valid relation target.
  std::vector<AXPlatformNodeWin*> CreatePlatformNodeVectorFromRelationIdVector(
      std::vector<int32_t>& relation_id_list);

  // Create a safearray of automation elements from a vector of
  // AXPlatformNodeWin.
  // The caller should validate that all of the given ax platform nodes are
  // valid relation targets.
  SAFEARRAY* CreateUIAElementsSafeArray(
      std::vector<AXPlatformNodeWin*>& platform_node_list);

  // Return an array that contains the center x, y coordinates of the
  // clickable point.
  SAFEARRAY* CreateClickablePointArray();

  // Returns the scroll offsets to which UI Automation should scroll an
  // accessible object, given the horizontal and vertical scroll amounts.
  gfx::Vector2d CalculateUIAScrollPoint(
      const ScrollAmount horizontal_amount,
      const ScrollAmount vertical_amount) const;

  void AddAlertTarget();
  void RemoveAlertTarget();

  // Enum used to specify whether IAccessibleText is requesting text
  // At, Before, or After a specified offset.
  enum class TextOffsetType { kAtOffset, kBeforeOffset, kAfterOffset };

  // Many MSAA methods take a var_id parameter indicating that the operation
  // should be performed on a particular child ID, rather than this object.
  // This method tries to figure out the target object from |var_id| and
  // returns a pointer to the target object if it exists, otherwise nullptr.
  // Does not return a new reference.
  AXPlatformNodeWin* GetTargetFromChildID(const VARIANT& var_id);

  // Returns true if this node is in a treegrid.
  bool IsInTreeGrid();

  // Helper method for returning selected indicies. It is expected that the
  // caller ensures that the input has been validated.
  HRESULT AllocateComArrayFromVector(std::vector<LONG>& results,
                                     LONG max,
                                     LONG** selected,
                                     LONG* n_selected);

  // Helper method for mutating the ISelectionItemProvider selected state
  HRESULT ISelectionItemProviderSetSelected(bool selected) const;

  // Helper method getting the selected status.
  bool ISelectionItemProviderIsSelected() const;

  //
  // Getters for UIA GetTextAttributeValue
  //

  // Computes the AnnotationTypes Attribute for the current node.
  HRESULT GetAnnotationTypesAttribute(const std::optional<int>& start_offset,
                                      const std::optional<int>& end_offset,
                                      base::win::VariantVector* result);
  // Lookup the LCID for the language this node is using.
  // Returns base::nullopt if there was an error.
  std::optional<LCID> GetCultureAttributeAsLCID() const;
  // Converts an int attribute to a COLORREF
  COLORREF GetIntAttributeAsCOLORREF(ax::mojom::IntAttribute attribute) const;
  // Converts the ListStyle to UIA BulletStyle
  BulletStyle ComputeUIABulletStyle() const;
  // Helper to get the UIA StyleId enumeration for this node
  LONG ComputeUIAStyleId() const;
  // Convert mojom TextAlign to UIA HorizontalTextAlignment enumeration
  static std::optional<HorizontalTextAlignment>
  AXTextAlignToUIAHorizontalTextAlignment(ax::mojom::TextAlign text_align);
  // Converts IntAttribute::kHierarchicalLevel to UIA StyleId enumeration
  static LONG AXHierarchicalLevelToUIAStyleId(int32_t hierarchical_level);
  // Converts a ListStyle to UIA StyleId enumeration
  static LONG AXListStyleToUIAStyleId(ax::mojom::ListStyle list_style);
  // Convert mojom TextDirection to UIA FlowDirections enumeration
  static FlowDirections TextDirectionToFlowDirections(
      ax::mojom::WritingDirection);

  // Helper method for |GetMarkerTypeFromRange| which aggregates all
  // of the ranges for |marker_type| attached to |node|.
  static void AggregateRangesForMarkerType(
      AXPlatformNodeBase* node,
      ax::mojom::MarkerType marker_type,
      int offset_ranges_amount,
      std::vector<std::pair<int, int>>* ranges);

  enum class MarkerTypeRangeResult {
    // The MarkerType does not overlap the range.
    kNone,
    // The MarkerType overlaps the entire range.
    kMatch,
    // The MarkerType partially overlaps the range.
    kMixed,
  };

  // Determine if a text range overlaps a |marker_type|, and whether
  // the overlap is a partial or complete match.
  MarkerTypeRangeResult GetMarkerTypeFromRange(
      const std::optional<int>& start_offset,
      const std::optional<int>& end_offset,
      ax::mojom::MarkerType marker_type);

  bool IsAncestorComboBox();

  bool IsPlaceholderText() const;

  // Helper method for getting the horizontal scroll percent.
  double GetHorizontalScrollPercent();

  // Helper method for getting the vertical scroll percent.
  double GetVerticalScrollPercent();

  // Helper to get the UIA FontName for this node as a BSTR.
  BSTR GetFontNameAttributeAsBSTR() const;

  // Helper to get the UIA StyleName for this node as a BSTR.
  BSTR GetStyleNameAttributeAsBSTR() const;

  // Gets the TextDecorationLineStyle based on the provided int attribute.
  TextDecorationLineStyle GetUIATextDecorationStyle(
      const ax::mojom::IntAttribute int_attribute) const;

  // IRawElementProviderSimple support methods.

  using PatternProviderFactoryMethod = void (*)(AXPlatformNodeWin*, IUnknown**);

  PatternProviderFactoryMethod GetPatternProviderFactoryMethod(
      PATTERNID pattern_id);

  // Fires UIA text edit event about composition (active or committed)
  void FireUiaTextEditTextChangedEvent(
      const gfx::Range& range,
      const std::u16string& active_composition_text,
      bool is_composition_committed);

  // Return true if the given element is valid enough to be returned as a value
  // for a UIA relation property (e.g. ControllerFor).
  static bool IsValidUiaRelationTarget(AXPlatformNode* ax_platform_node);

  // Start and end offsets of an active composition
  gfx::Range active_composition_range_;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_WIN_H_
