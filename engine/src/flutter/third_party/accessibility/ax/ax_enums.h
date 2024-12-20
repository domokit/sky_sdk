// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_ENUMS_H_
#define UI_ACCESSIBILITY_AX_ENUMS_H_

// For new entries to the following four enums, also add to
// extensions/common/api/automation.idl. This is enforced
// by a PRESUBMIT check.
//
// Explanation of in-lined comments next to some enum values/attributes:
//
// Web: this attribute is only used in web content.
//
// Native: this attribute is only used in native UI.
//
// Implicit: for events, it would be cleaner if we just updated the AX node and
//     each platform fired the appropriate events to indicate which
//     platform-specific attributes changed.
//
//  if Native / [Platform1, ...] is specified, the attribute is only used
//  on those platforms.
//
// If unspecified, the attribute is used across web and native on multiple
// platforms.
namespace ax {

namespace mojom {

enum class Event {
  kNone,
  kActiveDescendantChanged,
  kAlert,
  kAriaAttributeChanged,   // Implicit
  kAutocorrectionOccured,  // Unknown: http://crbug.com/392498
  kBlur,                   // Remove: http://crbug.com/392502
  kCheckedStateChanged,    // Implicit
  kChildrenChanged,
  kClicked,
  kControlsChanged,
  kDocumentSelectionChanged,
  kDocumentTitleChanged,
  kEndOfTest,        // Sentinel value indicating the end of a test
  kExpandedChanged,  // Web
  kFocus,
  kFocusAfterMenuClose,
  kFocusContext,  // Contextual focus event that must delay the next focus event
  kHide,          // Remove: http://crbug.com/392502
  kHitTestResult,
  kHover,
  kImageFrameUpdated,     // Web
  kInvalidStatusChanged,  // Implicit
  kLayoutComplete,        // Web
  kLiveRegionCreated,     // Implicit
  kLiveRegionChanged,     // Web
  kLoadComplete,          // Web
  kLoadStart,             // Web / AuraLinux
  kLocationChanged,       // Web
  kMediaStartedPlaying,   // Native / Automation
  kMediaStoppedPlaying,   // Native / Automation
  kMenuEnd,               // Native / web: menu interaction has ended.
  kMenuListItemSelected,  // Web
  kMenuListValueChanged,  // Web
  kMenuPopupEnd,          // Native / web: a menu/submenu is hidden/closed.
  kMenuPopupStart,        // Native / web: a menu/submenu is shown/opened.
  kMenuStart,             // Native / web: menu interaction has begun.
  kMouseCanceled,
  kMouseDragged,
  kMouseMoved,
  kMousePressed,
  kMouseReleased,
  kRowCollapsed,
  kRowCountChanged,
  kRowExpanded,
  kScrollPositionChanged,    // Web
  kScrolledToAnchor,         // Web
  kSelectedChildrenChanged,  // Web
  kSelection,                // Native
  kSelectionAdd,             // Native
  kSelectionRemove,          // Native
  kShow,                     // Native / Automation
  kStateChanged,             // Native / Automation
  kTextChanged,
  kWindowActivated,          // Native
  kWindowDeactivated,        // Native
  kWindowVisibilityChanged,  // Native
  kTextSelectionChanged,
  kTooltipClosed,
  kTooltipOpened,
  kTreeChanged,  // Accessibility tree changed. Don't
                 // explicitly fire an accessibility event,
                 // only implicitly due to the change.
  kValueChanged,
  kMinValue = kNone,
  kMaxValue = kValueChanged,
};

// Accessibility object roles.
// The majority of these roles come from the ARIA specification. Reference
// the latest draft for proper usage.
//
// Roles not included by the ARIA specification should be avoided, especially
// internal roles used by the accessibility infrastructure.
//
// Explanation of in-lined comments next to some enum values.
//
// Web: this attribute is only used in web content.
//
// Native: this attribute is only used in native UI.
enum class Role {
  kNone,
  kAbbr,
  kAlert,
  kAlertDialog,
  kAnchor,
  kApplication,
  kArticle,
  kAudio,
  kBanner,
  kBlockquote,
  kButton,
  kCanvas,
  kCaption,
  kCaret,
  kCell,
  kCheckBox,
  kClient,
  kCode,
  kColorWell,
  kColumn,
  kColumnHeader,
  kComboBoxGrouping,
  kComboBoxMenuButton,
  kComplementary,
  kComment,
  kContentDeletion,
  kContentInsertion,
  kContentInfo,
  kDate,
  kDateTime,
  kDefinition,
  kDescriptionList,
  kDescriptionListDetail,
  kDescriptionListTerm,
  kDesktop,  // internal
  kDetails,
  kDialog,
  kDirectory,
  kDisclosureTriangle,
  // --------------------------------------------------------------
  // DPub Roles:
  // https://www.w3.org/TR/dpub-aam-1.0/#mapping_role_table
  kDocAbstract,
  kDocAcknowledgments,
  kDocAfterword,
  kDocAppendix,
  kDocBackLink,
  kDocBiblioEntry,
  kDocBibliography,
  kDocBiblioRef,
  kDocChapter,
  kDocColophon,
  kDocConclusion,
  kDocCover,
  kDocCredit,
  kDocCredits,
  kDocDedication,
  kDocEndnote,
  kDocEndnotes,
  kDocEpigraph,
  kDocEpilogue,
  kDocErrata,
  kDocExample,
  kDocFootnote,
  kDocForeword,
  kDocGlossary,
  kDocGlossRef,
  kDocIndex,
  kDocIntroduction,
  kDocNoteRef,
  kDocNotice,
  kDocPageBreak,
  kDocPageList,
  kDocPart,
  kDocPreface,
  kDocPrologue,
  kDocPullquote,
  kDocQna,
  kDocSubtitle,
  kDocTip,
  kDocToc,
  // End DPub roles.
  // --------------------------------------------------------------
  kDocument,
  kEmbeddedObject,
  kEmphasis,
  kFeed,
  kFigcaption,
  kFigure,
  kFooter,
  kFooterAsNonLandmark,
  kForm,
  kGenericContainer,
  // --------------------------------------------------------------
  // ARIA Graphics module roles:
  // https://rawgit.com/w3c/graphics-aam/master/#mapping_role_table
  kGraphicsDocument,
  kGraphicsObject,
  kGraphicsSymbol,
  // End ARIA Graphics module roles.
  // --------------------------------------------------------------
  kGrid,
  kGroup,
  kHeader,
  kHeaderAsNonLandmark,
  kHeading,
  kIframe,
  kIframePresentational,
  kIgnored,
  kImage,
  kImageMap,
  kImeCandidate,
  kInlineTextBox,
  kInputTime,
  kKeyboard,
  kLabelText,
  kLayoutTable,
  kLayoutTableCell,
  kLayoutTableRow,
  kLegend,
  kLineBreak,
  kLink,
  kList,
  kListBox,
  kListBoxOption,
  // kListGrid behaves similar to an ARIA grid but is primarily used by
  // TableView and its subclasses, so that they could be exposed correctly on
  // certain platforms.
  kListGrid,  // Native
  kListItem,
  kListMarker,
  kLog,
  kMain,
  kMark,
  kMarquee,
  kMath,
  kMenu,
  kMenuBar,
  kMenuItem,
  kMenuItemCheckBox,
  kMenuItemRadio,
  kMenuListOption,
  kMenuListPopup,
  kMeter,
  kNavigation,
  kNote,
  kPane,
  kParagraph,
  kPdfActionableHighlight,  // PDF specific highlight role.
  kPluginObject,
  kPopUpButton,
  kPortal,
  kPre,
  kPresentational,
  kProgressIndicator,
  kRadioButton,
  kRadioGroup,
  kRegion,
  kRootWebArea,
  kRow,
  kRowGroup,
  kRowHeader,
  kRuby,
  kRubyAnnotation,
  kScrollBar,
  kScrollView,
  kSearch,
  kSearchBox,
  kSection,
  kSlider,
  kSliderThumb,
  kSpinButton,
  kSplitter,
  kStaticText,
  kStatus,
  kStrong,
  kSuggestion,
  kSvgRoot,
  kSwitch,
  kTab,
  kTabList,
  kTabPanel,
  kTable,
  kTableHeaderContainer,
  kTerm,
  kTextField,
  kTextFieldWithComboBox,
  kTime,
  kTimer,
  kTitleBar,
  kToggleButton,
  kToolbar,
  kTooltip,
  kTree,
  kTreeGrid,
  kTreeItem,
  kUnknown,
  kVideo,
  kWebArea,
  kWebView,
  kWindow,
  kMinValue = kNone,
  kMaxValue = kWindow,
};

enum class State {
  kNone,
  kAutofillAvailable,
  kCollapsed,
  kDefault,
  kEditable,
  kExpanded,
  kFocusable,
  // Grows horizontally, e.g. most toolbars and separators.
  kHorizontal,
  kHovered,
  // Skip over this node in the accessibility tree, but keep its subtree.
  kIgnored,
  kInvisible,
  kLinked,
  kMultiline,
  kMultiselectable,
  kProtected,
  kRequired,
  kRichlyEditable,
  // Grows vertically, e.g. menu or combo box.
  kVertical,
  kVisited,
  kMinValue = kNone,
  kMaxValue = kVisited,
};

// An action to be taken on an accessibility node.
// In contrast to |AXDefaultActionVerb|, these describe what happens to the
// object, e.g. "FOCUS".
enum class Action {
  kNone,

  // Request image annotations for all the eligible images on a page.
  kAnnotatePageImages,

  kBlur,

  // Notifies a node that it no longer has accessibility focus.
  // Currently used only on Android and only internally, it's not
  // exposed to the open web. See kSetAccessibilityFocus, below.
  kClearAccessibilityFocus,

  // Collapse the collapsible node.
  kCollapse,

  kCustomAction,

  // Decrement a slider or range control by one step value.
  kDecrement,

  // Do the default action for an object, typically this means "click".
  kDoDefault,

  // Expand the expandable node.
  kExpand,

  kFocus,

  // Return the content of this image object in the image_data attribute.
  kGetImageData,

  // Gets the bounding rect for a range of text.
  kGetTextLocation,

  kHideTooltip,

  // Given a point, find the object it corresponds to and fire a
  // |AXActionData.hit_test_event_to_fire| event on it in response.
  kHitTest,

  // Increment a slider or range control by one step value.
  kIncrement,

  // For internal use only; signals to tree sources to invalidate an entire
  // tree.
  kInternalInvalidateTree,

  // Load inline text boxes for this subtree, providing information
  // about word boundaries, line layout, and individual character
  // bounding boxes.
  kLoadInlineTextBoxes,

  // Delete any selected text in the control's text value and
  // insert |AXActionData::value| in its place, like when typing or pasting.
  kReplaceSelectedText,

  // Scrolls by approximately one screen in a specific direction. Should be
  // called on a node that has scrollable boolean set to true.
  kScrollBackward,
  kScrollDown,
  kScrollForward,
  kScrollLeft,
  kScrollRight,
  kScrollUp,

  // Scroll any scrollable containers to make the target object visible
  // on the screen.  Optionally pass a subfocus rect in
  // AXActionData.target_rect, in node-local coordinates.
  kScrollToMakeVisible,

  // Scroll the given object to a specified point on the screen in
  // global screen coordinates. Pass a point in AXActionData.target_point.
  kScrollToPoint,

  // Notifies a node that it has accessibility focus.
  // Currently used only on Android and only internally, it's not
  // exposed to the open web. See kClearAccessibilityFocus, above.
  kSetAccessibilityFocus,

  kSetScrollOffset,
  kSetSelection,

  // Don't focus this node, but set it as the sequential focus navigation
  // starting point, so that pressing Tab moves to the next element
  // following this one, for example.
  kSetSequentialFocusNavigationStartingPoint,

  // Replace the value of the control with AXActionData::value and
  // reset the selection, if applicable.
  kSetValue,
  kShowContextMenu,

  // Send an event signaling the end of a test.
  kSignalEndOfTest,
  kShowTooltip,
  // Used for looping through the enum, This must be the last value of this
  // enum.
  kMinValue = kNone,
  kMaxValue = kShowTooltip,
};

enum class ActionFlags {
  kNone,
  kRequestImages,
  kRequestInlineTextBoxes,
  kMinValue = kNone,
  kMaxValue = kRequestInlineTextBoxes,
};

// A list of valid values for the horizontal and vertical scroll alignment
// arguments in |AXActionData|. These values control where a node is scrolled
// in the viewport.
enum class ScrollAlignment {
  kNone,
  kScrollAlignmentCenter,
  kScrollAlignmentTop,
  kScrollAlignmentBottom,
  kScrollAlignmentLeft,
  kScrollAlignmentRight,
  kScrollAlignmentClosestEdge,
  kMinValue = kNone,
  kMaxValue = kScrollAlignmentClosestEdge,
};

// A list of valid values for the scroll behavior argument to argument in
// |AXActionData|. These values control whether a node is scrolled in the
// viewport if it is already visible.
enum class ScrollBehavior {
  kNone,
  kDoNotScrollIfVisible,
  kScrollIfVisible,
  kMinValue = kNone,
  kMaxValue = kScrollIfVisible,
};

// A list of valid values for the |AXIntAttribute| |default_action_verb|.
// These will describe the action that will be performed on a given node when
// executing the default action, which is a click.
// In contrast to |AXAction|, these describe what the user can do on the
// object, e.g. "PRESS", not what happens to the object as a result.
// Only one verb can be used at a time to describe the default action.
enum class DefaultActionVerb {
  kNone,
  kActivate,
  kCheck,
  kClick,

  // A click will be performed on one of the node's ancestors.
  // This happens when the node itself is not clickable, but one of its
  // ancestors has click handlers attached which are able to capture the click
  // as it bubbles up.
  kClickAncestor,

  kJump,
  kOpen,
  kPress,
  kSelect,
  kUncheck,
  kMinValue = kNone,
  kMaxValue = kUncheck,
};

// A change to the accessibility tree.
enum class Mutation {
  kNone,
  kNodeCreated,
  kSubtreeCreated,
  kNodeChanged,
  kNodeRemoved,
  kMinValue = kNone,
  kMaxValue = kNodeRemoved,
};

enum class StringAttribute {
  kNone,
  kAccessKey,
  // Only used when invalid_state == invalid_state_other.
  kAriaInvalidValue,
  kAutoComplete,
  kChildTreeId,
  kClassName,
  kContainerLiveRelevant,
  kContainerLiveStatus,
  kDescription,
  kDisplay,
  // Only present when different from parent.
  kFontFamily,
  kHtmlTag,
  // Stores an automatic image annotation if one is available. Only valid on
  // ax::mojom::Role::kImage. See kImageAnnotationStatus, too.
  kImageAnnotation,
  kImageDataUrl,
  kInnerHtml,
  kInputType,
  kKeyShortcuts,
  // Only present when different from parent.
  kLanguage,
  kName,
  kLiveRelevant,
  kLiveStatus,
  // Only if not already exposed in kName (NameFrom::kPlaceholder)
  kPlaceholder,
  kRole,
  kRoleDescription,
  // Only if not already exposed in kName (NameFrom::kTitle)
  kTooltip,
  kUrl,
  kValue,
  kMinValue = kNone,
  kMaxValue = kValue,
};

enum class IntAttribute {
  kNone,
  kDefaultActionVerb,
  // Scrollable container attributes.
  kScrollX,
  kScrollXMin,
  kScrollXMax,
  kScrollY,
  kScrollYMin,
  kScrollYMax,

  // Attributes for retrieving the endpoints of a selection.
  kTextSelStart,
  kTextSelEnd,

  // aria_col* and aria_row* attributes
  kAriaColumnCount,
  kAriaCellColumnIndex,
  kAriaCellColumnSpan,
  kAriaRowCount,
  kAriaCellRowIndex,
  kAriaCellRowSpan,

  // Table attributes.
  kTableRowCount,
  kTableColumnCount,
  kTableHeaderId,

  // Table row attributes.
  kTableRowIndex,
  kTableRowHeaderId,

  // Table column attributes.
  kTableColumnIndex,
  kTableColumnHeaderId,

  // Table cell attributes.
  kTableCellColumnIndex,
  kTableCellColumnSpan,
  kTableCellRowIndex,
  kTableCellRowSpan,
  kSortDirection,

  // Tree control attributes.
  kHierarchicalLevel,

  // What information was used to compute the object's name
  // (of type AXNameFrom).
  kNameFrom,

  // What information was used to compute the object's description
  // (of type AXDescriptionFrom).
  kDescriptionFrom,

  // Relationships between this element and other elements.
  kActivedescendantId,
  kErrormessageId,
  kInPageLinkTargetId,
  kMemberOfId,
  kNextOnLineId,
  kPopupForId,
  kPreviousOnLineId,

  // Input restriction, if any, such as readonly or disabled.
  // Of type AXRestriction, see below.
  // No value or enabled control or other object that is not disabled.
  kRestriction,

  // Position or Number of items in current set of listitems or treeitems
  kSetSize,
  kPosInSet,

  // In the case of Role::kColorWell, specifies the selected color.
  kColorValue,

  // Indicates the element that represents the current item within a container
  // or set of related elements.
  kAriaCurrentState,

  // Text attributes.

  // Foreground and background color in RGBA.
  kBackgroundColor,
  kColor,

  kHasPopup,

  // Image annotation status, of type ImageAnnotationStatus.
  kImageAnnotationStatus,

  // Indicates if a form control has invalid input or
  // if an element has an aria-invalid attribute.
  kInvalidState,

  // Of type AXCheckedState
  kCheckedState,

  // The list style type. Only available on list items.
  kListStyle,

  // Specifies the alignment of the text, e.g. left, center, right, justify
  kTextAlign,

  // Specifies the direction of the text, e.g., right-to-left.
  kTextDirection,

  // Specifies the position of the text, e.g., subscript.
  kTextPosition,

  // Bold, italic, underline, etc.
  kTextStyle,

  // The overline text decoration style.
  kTextOverlineStyle,

  // The strikethrough text decoration style.
  kTextStrikethroughStyle,

  // The underline text decoration style.
  kTextUnderlineStyle,

  // Focus traversal in views and Android.
  kPreviousFocusId,
  kNextFocusId,

  // For indicating what functions can be performed when a dragged object
  // is released on the drop target.
  // Note: aria-dropeffect is deprecated in WAI-ARIA 1.1.
  kDropeffect,

  // The DOMNodeID from Blink. Currently only populated when using
  // the accessibility tree for PDF exporting. Warning, this is totally
  // unrelated to the accessibility node ID, or the ID attribute for an
  // HTML element - it's an ID used to uniquely identify nodes in Blink.
  kDOMNodeId,
  kMinValue = kNone,
  kMaxValue = kDOMNodeId,
};

enum class FloatAttribute {
  kNone,
  // Range attributes.
  kValueForRange,
  kMinValueForRange,
  kMaxValueForRange,
  kStepValueForRange,

  // Text attributes.
  // Font size is in pixels.
  kFontSize,

  // Font weight can take on any arbitrary numeric value. Increments of 100 in
  // range [0, 900] represent keywords such as light, normal, bold, etc. 0 is
  // the default.
  kFontWeight,

  // The text indent of the text, in mm.
  kTextIndent,
  kMinValue = kNone,
  kMaxValue = kTextIndent,
};

// These attributes can take three states:
// true, false, or undefined/unset.
//
// Some attributes are only ever true or unset. In these cases, undefined is
// equivalent to false. In other attributes, all three states have meaning.
//
// Finally, note that different tree sources can use all three states for a
// given attribute, while another tree source only uses two.
enum class BoolAttribute {
  kNone,

  // Generic busy state, does not have to be on a live region.
  kBusy,

  // The object is at the root of an editable field, such as a content
  // editable.
  kEditableRoot,

  // Live region attributes.
  kContainerLiveAtomic,
  kContainerLiveBusy,
  kLiveAtomic,

  // If a dialog box is marked as explicitly modal
  kModal,

  // If this is set, all of the other fields in this struct should
  // be ignored and only the locations should change.
  kUpdateLocationOnly,

  // Set on a canvas element if it has fallback content.
  kCanvasHasFallback,

  // Indicates this node is user-scrollable, e.g. overflow:scroll|auto, as
  // opposed to only programmatically scrollable, like overflow:hidden, or
  // not scrollable at all, e.g. overflow:visible.
  kScrollable,

  // A hint to clients that the node is clickable.
  kClickable,

  // Indicates that this node clips its children, i.e. may have
  // overflow: hidden or clip children by default.
  kClipsChildren,

  // Indicates that this node is not selectable because the style has
  // user-select: none. Note that there may be other reasons why a node is
  // not selectable - for example, bullets in a list. However, this attribute
  // is only set on user-select: none.
  kNotUserSelectableStyle,

  // Indicates whether this node is selected or unselected.
  kSelected,

  // Indicates whether this node is selected due to selection follows focus.
  kSelectedFromFocus,

  // Indicates whether this node supports text location.
  kSupportsTextLocation,

  // Indicates whether this node can be grabbed for drag-and-drop operation.
  // Note: aria-grabbed is deprecated in WAI-ARIA 1.1.
  kGrabbed,

  // Indicates whether this node causes a hard line-break
  // (e.g. block level elements, or <br>)
  kIsLineBreakingObject,

  // Indicates whether this node causes a page break
  kIsPageBreakingObject,

  // True if the node has any ARIA attributes set.
  kHasAriaAttribute,
  kMinValue = kNone,
  kMaxValue = kHasAriaAttribute,
};

enum class IntListAttribute {
  kNone,
  // Ids of nodes that are children of this node logically, but are
  // not children of this node in the tree structure. As an example,
  // a table cell is a child of a row, and an 'indirect' child of a
  // column.
  kIndirectChildIds,

  // Relationships between this element and other elements.
  kControlsIds,
  kDetailsIds,
  kDescribedbyIds,
  kFlowtoIds,
  kLabelledbyIds,
  kRadioGroupIds,

  // For static text. These int lists must be the same size; they represent
  // the start and end character offset of each marker. Examples of markers
  // include spelling and grammar errors, and find-in-page matches.
  kMarkerTypes,
  kMarkerStarts,
  kMarkerEnds,

  // For inline text. This is the pixel position of the end of this
  // character within the bounding rectangle of this object, in the
  // direction given by StringAttribute::kTextDirection. For example,
  // for left-to-right text, the first offset is the right coordinate of
  // the first character within the object's bounds, the second offset
  // is the right coordinate of the second character, and so on.
  kCharacterOffsets,

  // Used for caching. Do not read directly. Use
  // |AXNode::GetOrComputeLineStartOffsets|
  // For all text fields and content editable roots: A list of the start
  // offsets of each line inside this object.
  kCachedLineStarts,

  // For inline text. These int lists must be the same size; they represent
  // the start and end character offset of each word within this text.
  kWordStarts,
  kWordEnds,

  // Used for an UI element to define custom actions for it. For example, a
  // list UI will allow a user to reorder items in the list by dragging the
  // items. Developer can expose those actions as custom actions. Currently
  // custom actions are used only in Android window.
  kCustomActionIds,
  kMinValue = kNone,
  kMaxValue = kCustomActionIds,
};

enum class StringListAttribute {
  kNone,
  // Descriptions for custom actions. This must be aligned with
  // custom_action_ids.
  kCustomActionDescriptions,
  kMinValue = kNone,
  kMaxValue = kCustomActionDescriptions,
};

enum class ListStyle {
  kNone,
  kCircle,
  kDisc,
  kImage,
  kNumeric,
  kSquare,
  kOther,  // Language specific ordering (alpha, roman, cjk-ideographic, etc...)
  kMinValue = kNone,
  kMaxValue = kOther,
};

enum class MarkerType {
  kNone = 0,
  kSpelling = 1,
  kGrammar = 2,
  kTextMatch = 4,
  // DocumentMarker::MarkerType::Composition = 8 is ignored for accessibility
  // purposes
  kActiveSuggestion = 16,
  kSuggestion = 32,
  kMinValue = kNone,
  kMaxValue = kSuggestion,
};

// Describes a move direction in the accessibility tree that is independent of
// the left-to-right or right-to-left direction of the text. For example, a
// forward movement will always move to the next node in depth-first pre-order
// traversal.
enum class MoveDirection {
  kForward,
  kBackward,
  kNone = kForward,
  kMinValue = kForward,
  kMaxValue = kBackward,
};

// Describes the edit or selection command that resulted in a selection or a
// text changed event.
enum class Command {
  kClearSelection,
  kCut,
  kDelete,
  kDictate,
  kExtendSelection,  // The existing selection has been extended or shrunk.
  kFormat,           // The text attributes, such as font size, have changed.
  kInsert,
  kMarker,         // A document marker has been added or removed.
  kMoveSelection,  // The selection has been moved by a specific granularity.
  kPaste,
  kReplace,
  kSetSelection,  // A completely new selection has been set.
  kType,
  kNone = kType,
  kMinValue = kClearSelection,
  kMaxValue = kType,
};

// Defines a set of text boundaries in the accessibility tree.
//
// Most boundaries come in three flavors: A "WordStartOrEnd" boundary for
// example differs from a "WordStart" or a "WordEnd" boundary in that the first
// would consider both the start and the end of the word to be boundaries, while
// the other two would consider only the start or the end respectively.
//
// An "Object" boundary is found at the start or end of a node's entire text,
// e.g. at the start or end of a text field.
//
// TODO(nektar): Split TextBoundary into TextUnit and TextBoundary.
enum class TextBoundary {
  kCharacter,
  kFormat,
  kLineEnd,
  kLineStart,
  kLineStartOrEnd,
  kObject,
  kPageEnd,
  kPageStart,
  kPageStartOrEnd,
  kParagraphEnd,
  kParagraphStart,
  kParagraphStartOrEnd,
  kSentenceEnd,
  kSentenceStart,
  kSentenceStartOrEnd,
  kWebPage,
  kWordEnd,
  kWordStart,
  kWordStartOrEnd,
  kNone = kObject,
  kMinValue = kCharacter,
  kMaxValue = kWordStartOrEnd,
};

// Types of text alignment according to the IAccessible2 Object Attributes spec.
enum class TextAlign {
  kNone,
  kLeft,
  kRight,
  kCenter,
  kJustify,
  kMinValue = kNone,
  kMaxValue = kJustify,
};

enum class WritingDirection {
  kNone,
  kLtr,
  kRtl,
  kTtb,
  kBtt,
  kMinValue = kNone,
  kMaxValue = kBtt,
};

enum class TextPosition {
  kNone,
  kSubscript,
  kSuperscript,
  kMinValue = kNone,
  kMaxValue = kSuperscript,
};

// A Java counterpart will be generated for this enum.
// GENERATED_JAVA_ENUM_PACKAGE: org.chromium.ui.accessibility
enum class TextStyle {
  kBold,
  kItalic,
  kUnderline,
  kLineThrough,
  kOverline,
  kNone,
  kMinValue = kBold,
  kMaxValue = kNone,
};

enum class TextDecorationStyle {
  kNone,
  kDotted,
  kDashed,
  kSolid,
  kDouble,
  kWavy,
  kMinValue = kNone,
  kMaxValue = kWavy,
};

enum class AriaCurrentState {
  kNone,
  kFalse,
  kTrue,
  kPage,
  kStep,
  kLocation,
  kUnclippedLocation,
  kDate,
  kTime,
  kMinValue = kNone,
  kMaxValue = kTime,
};

enum class HasPopup {
  kFalse = 0,
  kTrue,
  kMenu,
  kListbox,
  kTree,
  kGrid,
  kDialog,
  kNone = kFalse,
  kMinValue = kNone,
  kMaxValue = kDialog,
};

enum class InvalidState {
  kNone,
  kFalse,
  kTrue,
  kOther,
  kMinValue = kNone,
  kMaxValue = kOther,
};

// Input restriction associated with an object.
// No value for a control means it is enabled.
// Use read_only for a textbox that allows focus/selection but not input.
// Use disabled for a control or group of controls that disallows input.
enum class Restriction {
  kNone,
  kReadOnly,
  kDisabled,
  kMinValue = kNone,
  kMaxValue = kDisabled,
};

enum class CheckedState {
  kNone,
  kFalse,
  kTrue,
  kMixed,
  kMinValue = kNone,
  kMaxValue = kMixed,
};

enum class SortDirection {
  kNone,
  kUnsorted,
  kAscending,
  kDescending,
  kOther,
  kMinValue = kNone,
  kMaxValue = kOther,
};

enum class NameFrom {
  kNone,
  kUninitialized,
  kAttribute,  // E.g. aria-label.
  kAttributeExplicitlyEmpty,
  kCaption,  // E.g. in the case of a table, from a caption element.
  kContents,
  kPlaceholder,     // E.g. from an HTML placeholder attribute on a text field.
  kRelatedElement,  // E.g. from a figcaption Element in a figure.
  kTitle,           // E.g. <input type="text" title="title">.
  kValue,           // E.g. <input type="button" value="Button's name">.
  kMinValue = kNone,
  kMaxValue = kValue,
};

enum class DescriptionFrom {
  kNone,
  kUninitialized,
  kAttribute,
  kContents,
  kRelatedElement,
  kTitle,
  kMinValue = kNone,
  kMaxValue = kTitle,
};

enum class EventFrom {
  kNone,
  kUser,
  kPage,
  kAction,
  kMinValue = kNone,
  kMaxValue = kAction,
};

// Touch gestures on Chrome OS.
enum class Gesture {
  kNone,
  kClick,
  kSwipeLeft1,
  kSwipeUp1,
  kSwipeRight1,
  kSwipeDown1,
  kSwipeLeft2,
  kSwipeUp2,
  kSwipeRight2,
  kSwipeDown2,
  kSwipeLeft3,
  kSwipeUp3,
  kSwipeRight3,
  kSwipeDown3,
  kSwipeLeft4,
  kSwipeUp4,
  kSwipeRight4,
  kSwipeDown4,
  kTap2,
  kTap3,
  kTap4,
  kTouchExplore,
  kMinValue = kNone,
  kMaxValue = kTouchExplore,
};

enum class TextAffinity {
  kNone,
  kDownstream,
  kUpstream,
  kMinValue = kNone,
  kMaxValue = kUpstream,
};

// Compares two nodes in an accessibility tree in pre-order traversal.
enum class TreeOrder {
  kNone,
  // Not in the same tree, or other error.
  kUndefined,

  // First node is before the second one.
  kBefore,

  // Nodes are the same.
  kEqual,

  // First node is after the second one.
  kAfter,
  kMinValue = kNone,
  kMaxValue = kAfter,
};

// For internal use by ui::AXTreeID / ui::AXTreeID.
enum class AXTreeIDType {
  kUnknown,  // The Tree ID is unknown.
  kToken,    // Every other tree ID must have a valid unguessable token.
  kMinValue = kUnknown,
  kMaxValue = kToken,
};

enum class ImageAnnotationStatus {
  // Not an image, or image annotation feature not enabled.
  kNone,

  // Not eligible due to the scheme of the page. Image annotations are only
  // generated for images on http, https, file and data URLs.
  kWillNotAnnotateDueToScheme,

  // Not loaded yet, already labeled by the author, or not eligible
  // due to size, type, etc.
  kIneligibleForAnnotation,

  // Eligible to be automatically annotated if the user requests it.
  // This is communicated to the user via a tutor message.
  kEligibleForAnnotation,

  // Eligible to be automatically annotated but this is not communicated to the
  // user.
  kSilentlyEligibleForAnnotation,

  // An annotation has been requested but has not been received yet.
  kAnnotationPending,

  // An annotation has been provided and kImageAnnotation contains the
  // annotation text.
  kAnnotationSucceeded,

  // The annotation request was processed successfully, but it was not
  // possible to come up with an annotation for this image.
  kAnnotationEmpty,

  // The image is classified as adult content and no annotation will
  // be generated.
  kAnnotationAdult,

  // The annotation process failed, e.g. unable to contact the server,
  // request timed out, etc.
  kAnnotationProcessFailed,
  kMinValue = kNone,
  kMaxValue = kAnnotationProcessFailed,
};

enum class Dropeffect {
  kNone,
  kCopy,
  kExecute,
  kLink,
  kMove,
  kPopup,
  kMinValue = kNone,
  kMaxValue = kPopup,
};

}  // namespace mojom

}  // namespace ax

#endif  // UI_ACCESSIBILITY_AX_ENUMS_H_
