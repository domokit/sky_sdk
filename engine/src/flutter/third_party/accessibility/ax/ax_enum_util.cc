// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_enum_util.h"

namespace ui {

const char* ToString(ax::mojom::Event event) {
  switch (event) {
    case ax::mojom::Event::kNone:
      return "none";
    case ax::mojom::Event::kActiveDescendantChanged:
      return "activedescendantchanged";
    case ax::mojom::Event::kAlert:
      return "alert";
    case ax::mojom::Event::kAriaAttributeChanged:
      return "ariaAttributeChanged";
    case ax::mojom::Event::kAutocorrectionOccured:
      return "autocorrectionOccured";
    case ax::mojom::Event::kBlur:
      return "blur";
    case ax::mojom::Event::kCheckedStateChanged:
      return "checkedStateChanged";
    case ax::mojom::Event::kChildrenChanged:
      return "childrenChanged";
    case ax::mojom::Event::kClicked:
      return "clicked";
    case ax::mojom::Event::kControlsChanged:
      return "controlsChanged";
    case ax::mojom::Event::kDocumentSelectionChanged:
      return "documentSelectionChanged";
    case ax::mojom::Event::kDocumentTitleChanged:
      return "documentTitleChanged";
    case ax::mojom::Event::kEndOfTest:
      return "endOfTest";
    case ax::mojom::Event::kExpandedChanged:
      return "expandedChanged";
    case ax::mojom::Event::kFocus:
      return "focus";
    case ax::mojom::Event::kFocusAfterMenuClose:
      return "focusAfterMenuClose";
    case ax::mojom::Event::kFocusContext:
      return "focusContext";
    case ax::mojom::Event::kHide:
      return "hide";
    case ax::mojom::Event::kHitTestResult:
      return "hitTestResult";
    case ax::mojom::Event::kHover:
      return "hover";
    case ax::mojom::Event::kImageFrameUpdated:
      return "imageFrameUpdated";
    case ax::mojom::Event::kInvalidStatusChanged:
      return "invalidStatusChanged";
    case ax::mojom::Event::kLayoutComplete:
      return "layoutComplete";
    case ax::mojom::Event::kLiveRegionCreated:
      return "liveRegionCreated";
    case ax::mojom::Event::kLiveRegionChanged:
      return "liveRegionChanged";
    case ax::mojom::Event::kLoadComplete:
      return "loadComplete";
    case ax::mojom::Event::kLoadStart:
      return "loadStart";
    case ax::mojom::Event::kLocationChanged:
      return "locationChanged";
    case ax::mojom::Event::kMediaStartedPlaying:
      return "mediaStartedPlaying";
    case ax::mojom::Event::kMediaStoppedPlaying:
      return "mediaStoppedPlaying";
    case ax::mojom::Event::kMenuEnd:
      return "menuEnd";
    case ax::mojom::Event::kMenuListItemSelected:
      return "menuListItemSelected";
    case ax::mojom::Event::kMenuListValueChanged:
      return "menuListValueChanged";
    case ax::mojom::Event::kMenuPopupEnd:
      return "menuPopupEnd";
    case ax::mojom::Event::kMenuPopupStart:
      return "menuPopupStart";
    case ax::mojom::Event::kMenuStart:
      return "menuStart";
    case ax::mojom::Event::kMouseCanceled:
      return "mouseCanceled";
    case ax::mojom::Event::kMouseDragged:
      return "mouseDragged";
    case ax::mojom::Event::kMouseMoved:
      return "mouseMoved";
    case ax::mojom::Event::kMousePressed:
      return "mousePressed";
    case ax::mojom::Event::kMouseReleased:
      return "mouseReleased";
    case ax::mojom::Event::kRowCollapsed:
      return "rowCollapsed";
    case ax::mojom::Event::kRowCountChanged:
      return "rowCountChanged";
    case ax::mojom::Event::kRowExpanded:
      return "rowExpanded";
    case ax::mojom::Event::kScrollPositionChanged:
      return "scrollPositionChanged";
    case ax::mojom::Event::kScrolledToAnchor:
      return "scrolledToAnchor";
    case ax::mojom::Event::kSelectedChildrenChanged:
      return "selectedChildrenChanged";
    case ax::mojom::Event::kSelection:
      return "selection";
    case ax::mojom::Event::kSelectionAdd:
      return "selectionAdd";
    case ax::mojom::Event::kSelectionRemove:
      return "selectionRemove";
    case ax::mojom::Event::kShow:
      return "show";
    case ax::mojom::Event::kStateChanged:
      return "stateChanged";
    case ax::mojom::Event::kTextChanged:
      return "textChanged";
    case ax::mojom::Event::kTextSelectionChanged:
      return "textSelectionChanged";
    case ax::mojom::Event::kTooltipClosed:
      return "tooltipClosed";
    case ax::mojom::Event::kTooltipOpened:
      return "tooltipOpened";
    case ax::mojom::Event::kWindowActivated:
      return "windowActivated";
    case ax::mojom::Event::kWindowDeactivated:
      return "windowDeactivated";
    case ax::mojom::Event::kWindowVisibilityChanged:
      return "windowVisibilityChanged";
    case ax::mojom::Event::kTreeChanged:
      return "treeChanged";
    case ax::mojom::Event::kValueChanged:
      return "valueChanged";
  }

  return "";
}

ax::mojom::Event ParseEvent(const char* event) {
  if (0 == strcmp(event, "none"))
    return ax::mojom::Event::kNone;
  if (0 == strcmp(event, "activedescendantchanged"))
    return ax::mojom::Event::kActiveDescendantChanged;
  if (0 == strcmp(event, "alert"))
    return ax::mojom::Event::kAlert;
  if (0 == strcmp(event, "ariaAttributeChanged"))
    return ax::mojom::Event::kAriaAttributeChanged;
  if (0 == strcmp(event, "autocorrectionOccured"))
    return ax::mojom::Event::kAutocorrectionOccured;
  if (0 == strcmp(event, "blur"))
    return ax::mojom::Event::kBlur;
  if (0 == strcmp(event, "checkedStateChanged"))
    return ax::mojom::Event::kCheckedStateChanged;
  if (0 == strcmp(event, "childrenChanged"))
    return ax::mojom::Event::kChildrenChanged;
  if (0 == strcmp(event, "clicked"))
    return ax::mojom::Event::kClicked;
  if (0 == strcmp(event, "controlsChanged"))
    return ax::mojom::Event::kControlsChanged;
  if (0 == strcmp(event, "documentSelectionChanged"))
    return ax::mojom::Event::kDocumentSelectionChanged;
  if (0 == strcmp(event, "documentTitleChanged"))
    return ax::mojom::Event::kDocumentTitleChanged;
  if (0 == strcmp(event, "endOfTest"))
    return ax::mojom::Event::kEndOfTest;
  if (0 == strcmp(event, "expandedChanged"))
    return ax::mojom::Event::kExpandedChanged;
  if (0 == strcmp(event, "focus"))
    return ax::mojom::Event::kFocus;
  if (0 == strcmp(event, "focusAfterMenuClose"))
    return ax::mojom::Event::kFocusAfterMenuClose;
  if (0 == strcmp(event, "focusContext"))
    return ax::mojom::Event::kFocusContext;
  if (0 == strcmp(event, "hide"))
    return ax::mojom::Event::kHide;
  if (0 == strcmp(event, "hitTestResult"))
    return ax::mojom::Event::kHitTestResult;
  if (0 == strcmp(event, "hover"))
    return ax::mojom::Event::kHover;
  if (0 == strcmp(event, "imageFrameUpdated"))
    return ax::mojom::Event::kImageFrameUpdated;
  if (0 == strcmp(event, "invalidStatusChanged"))
    return ax::mojom::Event::kInvalidStatusChanged;
  if (0 == strcmp(event, "layoutComplete"))
    return ax::mojom::Event::kLayoutComplete;
  if (0 == strcmp(event, "liveRegionCreated"))
    return ax::mojom::Event::kLiveRegionCreated;
  if (0 == strcmp(event, "liveRegionChanged"))
    return ax::mojom::Event::kLiveRegionChanged;
  if (0 == strcmp(event, "loadComplete"))
    return ax::mojom::Event::kLoadComplete;
  if (0 == strcmp(event, "loadStart"))
    return ax::mojom::Event::kLoadStart;
  if (0 == strcmp(event, "locationChanged"))
    return ax::mojom::Event::kLocationChanged;
  if (0 == strcmp(event, "mediaStartedPlaying"))
    return ax::mojom::Event::kMediaStartedPlaying;
  if (0 == strcmp(event, "mediaStoppedPlaying"))
    return ax::mojom::Event::kMediaStoppedPlaying;
  if (0 == strcmp(event, "menuEnd"))
    return ax::mojom::Event::kMenuEnd;
  if (0 == strcmp(event, "menuListItemSelected"))
    return ax::mojom::Event::kMenuListItemSelected;
  if (0 == strcmp(event, "menuListValueChanged"))
    return ax::mojom::Event::kMenuListValueChanged;
  if (0 == strcmp(event, "menuPopupEnd"))
    return ax::mojom::Event::kMenuPopupEnd;
  if (0 == strcmp(event, "menuPopupStart"))
    return ax::mojom::Event::kMenuPopupStart;
  if (0 == strcmp(event, "menuStart"))
    return ax::mojom::Event::kMenuStart;
  if (0 == strcmp(event, "mouseCanceled"))
    return ax::mojom::Event::kMouseCanceled;
  if (0 == strcmp(event, "mouseDragged"))
    return ax::mojom::Event::kMouseDragged;
  if (0 == strcmp(event, "mouseMoved"))
    return ax::mojom::Event::kMouseMoved;
  if (0 == strcmp(event, "mousePressed"))
    return ax::mojom::Event::kMousePressed;
  if (0 == strcmp(event, "mouseReleased"))
    return ax::mojom::Event::kMouseReleased;
  if (0 == strcmp(event, "rowCollapsed"))
    return ax::mojom::Event::kRowCollapsed;
  if (0 == strcmp(event, "rowCountChanged"))
    return ax::mojom::Event::kRowCountChanged;
  if (0 == strcmp(event, "rowExpanded"))
    return ax::mojom::Event::kRowExpanded;
  if (0 == strcmp(event, "scrollPositionChanged"))
    return ax::mojom::Event::kScrollPositionChanged;
  if (0 == strcmp(event, "scrolledToAnchor"))
    return ax::mojom::Event::kScrolledToAnchor;
  if (0 == strcmp(event, "selectedChildrenChanged"))
    return ax::mojom::Event::kSelectedChildrenChanged;
  if (0 == strcmp(event, "selection"))
    return ax::mojom::Event::kSelection;
  if (0 == strcmp(event, "selectionAdd"))
    return ax::mojom::Event::kSelectionAdd;
  if (0 == strcmp(event, "selectionRemove"))
    return ax::mojom::Event::kSelectionRemove;
  if (0 == strcmp(event, "show"))
    return ax::mojom::Event::kShow;
  if (0 == strcmp(event, "stateChanged"))
    return ax::mojom::Event::kStateChanged;
  if (0 == strcmp(event, "textChanged"))
    return ax::mojom::Event::kTextChanged;
  if (0 == strcmp(event, "textSelectionChanged"))
    return ax::mojom::Event::kTextSelectionChanged;
  if (0 == strcmp(event, "tooltipClosed"))
    return ax::mojom::Event::kTooltipClosed;
  if (0 == strcmp(event, "tooltipOpened"))
    return ax::mojom::Event::kTooltipOpened;
  if (0 == strcmp(event, "windowActivated"))
    return ax::mojom::Event::kWindowActivated;
  if (0 == strcmp(event, "windowDeactivated"))
    return ax::mojom::Event::kWindowDeactivated;
  if (0 == strcmp(event, "windowVisibilityChanged"))
    return ax::mojom::Event::kWindowVisibilityChanged;
  if (0 == strcmp(event, "treeChanged"))
    return ax::mojom::Event::kTreeChanged;
  if (0 == strcmp(event, "valueChanged"))
    return ax::mojom::Event::kValueChanged;
  return ax::mojom::Event::kNone;
}

const char* ToString(ax::mojom::Role role) {
  switch (role) {
    case ax::mojom::Role::kNone:
      return "none";
    case ax::mojom::Role::kAbbr:
      return "abbr";
    case ax::mojom::Role::kAlertDialog:
      return "alertDialog";
    case ax::mojom::Role::kAlert:
      return "alert";
    case ax::mojom::Role::kAnchor:
      return "anchor";
    case ax::mojom::Role::kApplication:
      return "application";
    case ax::mojom::Role::kArticle:
      return "article";
    case ax::mojom::Role::kAudio:
      return "audio";
    case ax::mojom::Role::kBanner:
      return "banner";
    case ax::mojom::Role::kBlockquote:
      return "blockquote";
    case ax::mojom::Role::kButton:
      return "button";
    case ax::mojom::Role::kCanvas:
      return "canvas";
    case ax::mojom::Role::kCaption:
      return "caption";
    case ax::mojom::Role::kCaret:
      return "caret";
    case ax::mojom::Role::kCell:
      return "cell";
    case ax::mojom::Role::kCheckBox:
      return "checkBox";
    case ax::mojom::Role::kClient:
      return "client";
    case ax::mojom::Role::kCode:
      return "code";
    case ax::mojom::Role::kColorWell:
      return "colorWell";
    case ax::mojom::Role::kColumnHeader:
      return "columnHeader";
    case ax::mojom::Role::kColumn:
      return "column";
    case ax::mojom::Role::kComboBoxGrouping:
      return "comboBoxGrouping";
    case ax::mojom::Role::kComboBoxMenuButton:
      return "comboBoxMenuButton";
    case ax::mojom::Role::kComment:
      return "comment";
    case ax::mojom::Role::kComplementary:
      return "complementary";
    case ax::mojom::Role::kContentDeletion:
      return "contentDeletion";
    case ax::mojom::Role::kContentInsertion:
      return "contentInsertion";
    case ax::mojom::Role::kContentInfo:
      return "contentInfo";
    case ax::mojom::Role::kDate:
      return "date";
    case ax::mojom::Role::kDateTime:
      return "dateTime";
    case ax::mojom::Role::kDefinition:
      return "definition";
    case ax::mojom::Role::kDescriptionListDetail:
      return "descriptionListDetail";
    case ax::mojom::Role::kDescriptionList:
      return "descriptionList";
    case ax::mojom::Role::kDescriptionListTerm:
      return "descriptionListTerm";
    case ax::mojom::Role::kDesktop:
      return "desktop";
    case ax::mojom::Role::kDetails:
      return "details";
    case ax::mojom::Role::kDialog:
      return "dialog";
    case ax::mojom::Role::kDirectory:
      return "directory";
    case ax::mojom::Role::kDisclosureTriangle:
      return "disclosureTriangle";
    case ax::mojom::Role::kDocAbstract:
      return "docAbstract";
    case ax::mojom::Role::kDocAcknowledgments:
      return "docAcknowledgments";
    case ax::mojom::Role::kDocAfterword:
      return "docAfterword";
    case ax::mojom::Role::kDocAppendix:
      return "docAppendix";
    case ax::mojom::Role::kDocBackLink:
      return "docBackLink";
    case ax::mojom::Role::kDocBiblioEntry:
      return "docBiblioEntry";
    case ax::mojom::Role::kDocBibliography:
      return "docBibliography";
    case ax::mojom::Role::kDocBiblioRef:
      return "docBiblioRef";
    case ax::mojom::Role::kDocChapter:
      return "docChapter";
    case ax::mojom::Role::kDocColophon:
      return "docColophon";
    case ax::mojom::Role::kDocConclusion:
      return "docConclusion";
    case ax::mojom::Role::kDocCover:
      return "docCover";
    case ax::mojom::Role::kDocCredit:
      return "docCredit";
    case ax::mojom::Role::kDocCredits:
      return "docCredits";
    case ax::mojom::Role::kDocDedication:
      return "docDedication";
    case ax::mojom::Role::kDocEndnote:
      return "docEndnote";
    case ax::mojom::Role::kDocEndnotes:
      return "docEndnotes";
    case ax::mojom::Role::kDocEpigraph:
      return "docEpigraph";
    case ax::mojom::Role::kDocEpilogue:
      return "docEpilogue";
    case ax::mojom::Role::kDocErrata:
      return "docErrata";
    case ax::mojom::Role::kDocExample:
      return "docExample";
    case ax::mojom::Role::kDocFootnote:
      return "docFootnote";
    case ax::mojom::Role::kDocForeword:
      return "docForeword";
    case ax::mojom::Role::kDocGlossary:
      return "docGlossary";
    case ax::mojom::Role::kDocGlossRef:
      return "docGlossref";
    case ax::mojom::Role::kDocIndex:
      return "docIndex";
    case ax::mojom::Role::kDocIntroduction:
      return "docIntroduction";
    case ax::mojom::Role::kDocNoteRef:
      return "docNoteRef";
    case ax::mojom::Role::kDocNotice:
      return "docNotice";
    case ax::mojom::Role::kDocPageBreak:
      return "docPageBreak";
    case ax::mojom::Role::kDocPageList:
      return "docPageList";
    case ax::mojom::Role::kDocPart:
      return "docPart";
    case ax::mojom::Role::kDocPreface:
      return "docPreface";
    case ax::mojom::Role::kDocPrologue:
      return "docPrologue";
    case ax::mojom::Role::kDocPullquote:
      return "docPullquote";
    case ax::mojom::Role::kDocQna:
      return "docQna";
    case ax::mojom::Role::kDocSubtitle:
      return "docSubtitle";
    case ax::mojom::Role::kDocTip:
      return "docTip";
    case ax::mojom::Role::kDocToc:
      return "docToc";
    case ax::mojom::Role::kDocument:
      return "document";
    case ax::mojom::Role::kEmbeddedObject:
      return "embeddedObject";
    case ax::mojom::Role::kEmphasis:
      return "emphasis";
    case ax::mojom::Role::kFeed:
      return "feed";
    case ax::mojom::Role::kFigcaption:
      return "figcaption";
    case ax::mojom::Role::kFigure:
      return "figure";
    case ax::mojom::Role::kFooter:
      return "footer";
    case ax::mojom::Role::kFooterAsNonLandmark:
      return "footerAsNonLandmark";
    case ax::mojom::Role::kForm:
      return "form";
    case ax::mojom::Role::kGenericContainer:
      return "genericContainer";
    case ax::mojom::Role::kGraphicsDocument:
      return "graphicsDocument";
    case ax::mojom::Role::kGraphicsObject:
      return "graphicsObject";
    case ax::mojom::Role::kGraphicsSymbol:
      return "graphicsSymbol";
    case ax::mojom::Role::kGrid:
      return "grid";
    case ax::mojom::Role::kGroup:
      return "group";
    case ax::mojom::Role::kHeader:
      return "header";
    case ax::mojom::Role::kHeaderAsNonLandmark:
      return "headerAsNonLandmark";
    case ax::mojom::Role::kHeading:
      return "heading";
    case ax::mojom::Role::kIframe:
      return "iframe";
    case ax::mojom::Role::kIframePresentational:
      return "iframePresentational";
    case ax::mojom::Role::kIgnored:
      return "ignored";
    case ax::mojom::Role::kImageMap:
      return "imageMap";
    case ax::mojom::Role::kImage:
      return "image";
    case ax::mojom::Role::kImeCandidate:
      return "imeCandidate";
    case ax::mojom::Role::kInlineTextBox:
      return "inlineTextBox";
    case ax::mojom::Role::kInputTime:
      return "inputTime";
    case ax::mojom::Role::kKeyboard:
      return "keyboard";
    case ax::mojom::Role::kLabelText:
      return "labelText";
    case ax::mojom::Role::kLayoutTable:
      return "layoutTable";
    case ax::mojom::Role::kLayoutTableCell:
      return "layoutTableCell";
    case ax::mojom::Role::kLayoutTableRow:
      return "layoutTableRow";
    case ax::mojom::Role::kLegend:
      return "legend";
    case ax::mojom::Role::kLineBreak:
      return "lineBreak";
    case ax::mojom::Role::kLink:
      return "link";
    case ax::mojom::Role::kList:
      return "list";
    case ax::mojom::Role::kListBoxOption:
      return "listBoxOption";
    case ax::mojom::Role::kListBox:
      return "listBox";
    case ax::mojom::Role::kListGrid:
      return "listGrid";
    case ax::mojom::Role::kListItem:
      return "listItem";
    case ax::mojom::Role::kListMarker:
      return "listMarker";
    case ax::mojom::Role::kLog:
      return "log";
    case ax::mojom::Role::kMain:
      return "main";
    case ax::mojom::Role::kMark:
      return "mark";
    case ax::mojom::Role::kMarquee:
      return "marquee";
    case ax::mojom::Role::kMath:
      return "math";
    case ax::mojom::Role::kMenu:
      return "menu";
    case ax::mojom::Role::kMenuBar:
      return "menuBar";
    case ax::mojom::Role::kMenuItem:
      return "menuItem";
    case ax::mojom::Role::kMenuItemCheckBox:
      return "menuItemCheckBox";
    case ax::mojom::Role::kMenuItemRadio:
      return "menuItemRadio";
    case ax::mojom::Role::kMenuListOption:
      return "menuListOption";
    case ax::mojom::Role::kMenuListPopup:
      return "menuListPopup";
    case ax::mojom::Role::kMeter:
      return "meter";
    case ax::mojom::Role::kNavigation:
      return "navigation";
    case ax::mojom::Role::kNote:
      return "note";
    case ax::mojom::Role::kPane:
      return "pane";
    case ax::mojom::Role::kParagraph:
      return "paragraph";
    case ax::mojom::Role::kPdfActionableHighlight:
      return "pdfActionableHighlight";
    case ax::mojom::Role::kPluginObject:
      return "pluginObject";
    case ax::mojom::Role::kPopUpButton:
      return "popUpButton";
    case ax::mojom::Role::kPortal:
      return "portal";
    case ax::mojom::Role::kPre:
      return "pre";
    case ax::mojom::Role::kPresentational:
      return "presentational";
    case ax::mojom::Role::kProgressIndicator:
      return "progressIndicator";
    case ax::mojom::Role::kRadioButton:
      return "radioButton";
    case ax::mojom::Role::kRadioGroup:
      return "radioGroup";
    case ax::mojom::Role::kRegion:
      return "region";
    case ax::mojom::Role::kRootWebArea:
      return "rootWebArea";
    case ax::mojom::Role::kRow:
      return "row";
    case ax::mojom::Role::kRowGroup:
      return "rowGroup";
    case ax::mojom::Role::kRowHeader:
      return "rowHeader";
    case ax::mojom::Role::kRuby:
      return "ruby";
    case ax::mojom::Role::kRubyAnnotation:
      return "rubyAnnotation";
    case ax::mojom::Role::kSection:
      return "section";
    case ax::mojom::Role::kStrong:
      return "strong";
    case ax::mojom::Role::kSuggestion:
      return "suggestion";
    case ax::mojom::Role::kSvgRoot:
      return "svgRoot";
    case ax::mojom::Role::kScrollBar:
      return "scrollBar";
    case ax::mojom::Role::kScrollView:
      return "scrollView";
    case ax::mojom::Role::kSearch:
      return "search";
    case ax::mojom::Role::kSearchBox:
      return "searchBox";
    case ax::mojom::Role::kSlider:
      return "slider";
    case ax::mojom::Role::kSliderThumb:
      return "sliderThumb";
    case ax::mojom::Role::kSpinButton:
      return "spinButton";
    case ax::mojom::Role::kSplitter:
      return "splitter";
    case ax::mojom::Role::kStaticText:
      return "staticText";
    case ax::mojom::Role::kStatus:
      return "status";
    case ax::mojom::Role::kSwitch:
      return "switch";
    case ax::mojom::Role::kTabList:
      return "tabList";
    case ax::mojom::Role::kTabPanel:
      return "tabPanel";
    case ax::mojom::Role::kTab:
      return "tab";
    case ax::mojom::Role::kTable:
      return "table";
    case ax::mojom::Role::kTableHeaderContainer:
      return "tableHeaderContainer";
    case ax::mojom::Role::kTerm:
      return "term";
    case ax::mojom::Role::kTextField:
      return "textField";
    case ax::mojom::Role::kTextFieldWithComboBox:
      return "textFieldWithComboBox";
    case ax::mojom::Role::kTime:
      return "time";
    case ax::mojom::Role::kTimer:
      return "timer";
    case ax::mojom::Role::kTitleBar:
      return "titleBar";
    case ax::mojom::Role::kToggleButton:
      return "toggleButton";
    case ax::mojom::Role::kToolbar:
      return "toolbar";
    case ax::mojom::Role::kTreeGrid:
      return "treeGrid";
    case ax::mojom::Role::kTreeItem:
      return "treeItem";
    case ax::mojom::Role::kTree:
      return "tree";
    case ax::mojom::Role::kUnknown:
      return "unknown";
    case ax::mojom::Role::kTooltip:
      return "tooltip";
    case ax::mojom::Role::kVideo:
      return "video";
    case ax::mojom::Role::kWebArea:
      return "webArea";
    case ax::mojom::Role::kWebView:
      return "webView";
    case ax::mojom::Role::kWindow:
      return "window";
  }

  return "";
}

ax::mojom::Role ParseRole(const char* role) {
  if (0 == strcmp(role, "none"))
    return ax::mojom::Role::kNone;
  if (0 == strcmp(role, "abbr"))
    return ax::mojom::Role::kAbbr;
  if (0 == strcmp(role, "alertDialog"))
    return ax::mojom::Role::kAlertDialog;
  if (0 == strcmp(role, "alert"))
    return ax::mojom::Role::kAlert;
  if (0 == strcmp(role, "anchor"))
    return ax::mojom::Role::kAnchor;
  if (0 == strcmp(role, "application"))
    return ax::mojom::Role::kApplication;
  if (0 == strcmp(role, "article"))
    return ax::mojom::Role::kArticle;
  if (0 == strcmp(role, "audio"))
    return ax::mojom::Role::kAudio;
  if (0 == strcmp(role, "banner"))
    return ax::mojom::Role::kBanner;
  if (0 == strcmp(role, "blockquote"))
    return ax::mojom::Role::kBlockquote;
  if (0 == strcmp(role, "button"))
    return ax::mojom::Role::kButton;
  if (0 == strcmp(role, "canvas"))
    return ax::mojom::Role::kCanvas;
  if (0 == strcmp(role, "caption"))
    return ax::mojom::Role::kCaption;
  if (0 == strcmp(role, "caret"))
    return ax::mojom::Role::kCaret;
  if (0 == strcmp(role, "cell"))
    return ax::mojom::Role::kCell;
  if (0 == strcmp(role, "checkBox"))
    return ax::mojom::Role::kCheckBox;
  if (0 == strcmp(role, "client"))
    return ax::mojom::Role::kClient;
  if (0 == strcmp(role, "code"))
    return ax::mojom::Role::kCode;
  if (0 == strcmp(role, "colorWell"))
    return ax::mojom::Role::kColorWell;
  if (0 == strcmp(role, "columnHeader"))
    return ax::mojom::Role::kColumnHeader;
  if (0 == strcmp(role, "column"))
    return ax::mojom::Role::kColumn;
  if (0 == strcmp(role, "comboBoxGrouping"))
    return ax::mojom::Role::kComboBoxGrouping;
  if (0 == strcmp(role, "comboBoxMenuButton"))
    return ax::mojom::Role::kComboBoxMenuButton;
  if (0 == strcmp(role, "comment"))
    return ax::mojom::Role::kComment;
  if (0 == strcmp(role, "complementary"))
    return ax::mojom::Role::kComplementary;
  if (0 == strcmp(role, "contentDeletion"))
    return ax::mojom::Role::kContentDeletion;
  if (0 == strcmp(role, "contentInsertion"))
    return ax::mojom::Role::kContentInsertion;
  if (0 == strcmp(role, "contentInfo"))
    return ax::mojom::Role::kContentInfo;
  if (0 == strcmp(role, "date"))
    return ax::mojom::Role::kDate;
  if (0 == strcmp(role, "dateTime"))
    return ax::mojom::Role::kDateTime;
  if (0 == strcmp(role, "definition"))
    return ax::mojom::Role::kDefinition;
  if (0 == strcmp(role, "descriptionListDetail"))
    return ax::mojom::Role::kDescriptionListDetail;
  if (0 == strcmp(role, "descriptionList"))
    return ax::mojom::Role::kDescriptionList;
  if (0 == strcmp(role, "descriptionListTerm"))
    return ax::mojom::Role::kDescriptionListTerm;
  if (0 == strcmp(role, "desktop"))
    return ax::mojom::Role::kDesktop;
  if (0 == strcmp(role, "details"))
    return ax::mojom::Role::kDetails;
  if (0 == strcmp(role, "dialog"))
    return ax::mojom::Role::kDialog;
  if (0 == strcmp(role, "directory"))
    return ax::mojom::Role::kDirectory;
  if (0 == strcmp(role, "disclosureTriangle"))
    return ax::mojom::Role::kDisclosureTriangle;
  if (0 == strcmp(role, "docAbstract"))
    return ax::mojom::Role::kDocAbstract;
  if (0 == strcmp(role, "docAcknowledgments"))
    return ax::mojom::Role::kDocAcknowledgments;
  if (0 == strcmp(role, "docAfterword"))
    return ax::mojom::Role::kDocAfterword;
  if (0 == strcmp(role, "docAppendix"))
    return ax::mojom::Role::kDocAppendix;
  if (0 == strcmp(role, "docBackLink"))
    return ax::mojom::Role::kDocBackLink;
  if (0 == strcmp(role, "docBiblioEntry"))
    return ax::mojom::Role::kDocBiblioEntry;
  if (0 == strcmp(role, "docBibliography"))
    return ax::mojom::Role::kDocBibliography;
  if (0 == strcmp(role, "docBiblioRef"))
    return ax::mojom::Role::kDocBiblioRef;
  if (0 == strcmp(role, "docChapter"))
    return ax::mojom::Role::kDocChapter;
  if (0 == strcmp(role, "docColophon"))
    return ax::mojom::Role::kDocColophon;
  if (0 == strcmp(role, "docConclusion"))
    return ax::mojom::Role::kDocConclusion;
  if (0 == strcmp(role, "docCover"))
    return ax::mojom::Role::kDocCover;
  if (0 == strcmp(role, "docCredit"))
    return ax::mojom::Role::kDocCredit;
  if (0 == strcmp(role, "docCredits"))
    return ax::mojom::Role::kDocCredits;
  if (0 == strcmp(role, "docDedication"))
    return ax::mojom::Role::kDocDedication;
  if (0 == strcmp(role, "docEndnote"))
    return ax::mojom::Role::kDocEndnote;
  if (0 == strcmp(role, "docEndnotes"))
    return ax::mojom::Role::kDocEndnotes;
  if (0 == strcmp(role, "docEpigraph"))
    return ax::mojom::Role::kDocEpigraph;
  if (0 == strcmp(role, "docEpilogue"))
    return ax::mojom::Role::kDocEpilogue;
  if (0 == strcmp(role, "docErrata"))
    return ax::mojom::Role::kDocErrata;
  if (0 == strcmp(role, "docExample"))
    return ax::mojom::Role::kDocExample;
  if (0 == strcmp(role, "docFootnote"))
    return ax::mojom::Role::kDocFootnote;
  if (0 == strcmp(role, "docForeword"))
    return ax::mojom::Role::kDocForeword;
  if (0 == strcmp(role, "docGlossary"))
    return ax::mojom::Role::kDocGlossary;
  if (0 == strcmp(role, "docGlossref"))
    return ax::mojom::Role::kDocGlossRef;
  if (0 == strcmp(role, "docIndex"))
    return ax::mojom::Role::kDocIndex;
  if (0 == strcmp(role, "docIntroduction"))
    return ax::mojom::Role::kDocIntroduction;
  if (0 == strcmp(role, "docNoteRef"))
    return ax::mojom::Role::kDocNoteRef;
  if (0 == strcmp(role, "docNotice"))
    return ax::mojom::Role::kDocNotice;
  if (0 == strcmp(role, "docPageBreak"))
    return ax::mojom::Role::kDocPageBreak;
  if (0 == strcmp(role, "docPageList"))
    return ax::mojom::Role::kDocPageList;
  if (0 == strcmp(role, "docPart"))
    return ax::mojom::Role::kDocPart;
  if (0 == strcmp(role, "docPreface"))
    return ax::mojom::Role::kDocPreface;
  if (0 == strcmp(role, "docPrologue"))
    return ax::mojom::Role::kDocPrologue;
  if (0 == strcmp(role, "docPullquote"))
    return ax::mojom::Role::kDocPullquote;
  if (0 == strcmp(role, "docQna"))
    return ax::mojom::Role::kDocQna;
  if (0 == strcmp(role, "docSubtitle"))
    return ax::mojom::Role::kDocSubtitle;
  if (0 == strcmp(role, "docTip"))
    return ax::mojom::Role::kDocTip;
  if (0 == strcmp(role, "docToc"))
    return ax::mojom::Role::kDocToc;
  if (0 == strcmp(role, "document"))
    return ax::mojom::Role::kDocument;
  if (0 == strcmp(role, "embeddedObject"))
    return ax::mojom::Role::kEmbeddedObject;
  if (0 == strcmp(role, "emphasis"))
    return ax::mojom::Role::kEmphasis;
  if (0 == strcmp(role, "feed"))
    return ax::mojom::Role::kFeed;
  if (0 == strcmp(role, "figcaption"))
    return ax::mojom::Role::kFigcaption;
  if (0 == strcmp(role, "figure"))
    return ax::mojom::Role::kFigure;
  if (0 == strcmp(role, "footer"))
    return ax::mojom::Role::kFooter;
  if (0 == strcmp(role, "footerAsNonLandmark"))
    return ax::mojom::Role::kFooterAsNonLandmark;
  if (0 == strcmp(role, "form"))
    return ax::mojom::Role::kForm;
  if (0 == strcmp(role, "genericContainer"))
    return ax::mojom::Role::kGenericContainer;
  if (0 == strcmp(role, "graphicsDocument"))
    return ax::mojom::Role::kGraphicsDocument;
  if (0 == strcmp(role, "graphicsObject"))
    return ax::mojom::Role::kGraphicsObject;
  if (0 == strcmp(role, "graphicsSymbol"))
    return ax::mojom::Role::kGraphicsSymbol;
  if (0 == strcmp(role, "grid"))
    return ax::mojom::Role::kGrid;
  if (0 == strcmp(role, "group"))
    return ax::mojom::Role::kGroup;
  if (0 == strcmp(role, "heading"))
    return ax::mojom::Role::kHeading;
  if (0 == strcmp(role, "header"))
    return ax::mojom::Role::kHeader;
  if (0 == strcmp(role, "headerAsNonLandmark"))
    return ax::mojom::Role::kHeaderAsNonLandmark;
  if (0 == strcmp(role, "pdfActionableHighlight"))
    return ax::mojom::Role::kPdfActionableHighlight;
  if (0 == strcmp(role, "iframe"))
    return ax::mojom::Role::kIframe;
  if (0 == strcmp(role, "iframePresentational"))
    return ax::mojom::Role::kIframePresentational;
  if (0 == strcmp(role, "ignored"))
    return ax::mojom::Role::kIgnored;
  if (0 == strcmp(role, "imageMap"))
    return ax::mojom::Role::kImageMap;
  if (0 == strcmp(role, "image"))
    return ax::mojom::Role::kImage;
  if (0 == strcmp(role, "imeCandidate"))
    return ax::mojom::Role::kImeCandidate;
  if (0 == strcmp(role, "inlineTextBox"))
    return ax::mojom::Role::kInlineTextBox;
  if (0 == strcmp(role, "inputTime"))
    return ax::mojom::Role::kInputTime;
  if (0 == strcmp(role, "keyboard"))
    return ax::mojom::Role::kKeyboard;
  if (0 == strcmp(role, "labelText"))
    return ax::mojom::Role::kLabelText;
  if (0 == strcmp(role, "layoutTable"))
    return ax::mojom::Role::kLayoutTable;
  if (0 == strcmp(role, "layoutTableCell"))
    return ax::mojom::Role::kLayoutTableCell;
  if (0 == strcmp(role, "layoutTableRow"))
    return ax::mojom::Role::kLayoutTableRow;
  if (0 == strcmp(role, "legend"))
    return ax::mojom::Role::kLegend;
  if (0 == strcmp(role, "lineBreak"))
    return ax::mojom::Role::kLineBreak;
  if (0 == strcmp(role, "link"))
    return ax::mojom::Role::kLink;
  if (0 == strcmp(role, "listBoxOption"))
    return ax::mojom::Role::kListBoxOption;
  if (0 == strcmp(role, "listBox"))
    return ax::mojom::Role::kListBox;
  if (0 == strcmp(role, "listGrid"))
    return ax::mojom::Role::kListGrid;
  if (0 == strcmp(role, "listItem"))
    return ax::mojom::Role::kListItem;
  if (0 == strcmp(role, "listMarker"))
    return ax::mojom::Role::kListMarker;
  if (0 == strcmp(role, "list"))
    return ax::mojom::Role::kList;
  if (0 == strcmp(role, "log"))
    return ax::mojom::Role::kLog;
  if (0 == strcmp(role, "main"))
    return ax::mojom::Role::kMain;
  if (0 == strcmp(role, "mark"))
    return ax::mojom::Role::kMark;
  if (0 == strcmp(role, "marquee"))
    return ax::mojom::Role::kMarquee;
  if (0 == strcmp(role, "math"))
    return ax::mojom::Role::kMath;
  if (0 == strcmp(role, "menu"))
    return ax::mojom::Role::kMenu;
  if (0 == strcmp(role, "menuBar"))
    return ax::mojom::Role::kMenuBar;
  if (0 == strcmp(role, "menuItem"))
    return ax::mojom::Role::kMenuItem;
  if (0 == strcmp(role, "menuItemCheckBox"))
    return ax::mojom::Role::kMenuItemCheckBox;
  if (0 == strcmp(role, "menuItemRadio"))
    return ax::mojom::Role::kMenuItemRadio;
  if (0 == strcmp(role, "menuListOption"))
    return ax::mojom::Role::kMenuListOption;
  if (0 == strcmp(role, "menuListPopup"))
    return ax::mojom::Role::kMenuListPopup;
  if (0 == strcmp(role, "meter"))
    return ax::mojom::Role::kMeter;
  if (0 == strcmp(role, "navigation"))
    return ax::mojom::Role::kNavigation;
  if (0 == strcmp(role, "note"))
    return ax::mojom::Role::kNote;
  if (0 == strcmp(role, "pane"))
    return ax::mojom::Role::kPane;
  if (0 == strcmp(role, "paragraph"))
    return ax::mojom::Role::kParagraph;
  if (0 == strcmp(role, "pluginObject"))
    return ax::mojom::Role::kPluginObject;
  if (0 == strcmp(role, "popUpButton"))
    return ax::mojom::Role::kPopUpButton;
  if (0 == strcmp(role, "portal"))
    return ax::mojom::Role::kPortal;
  if (0 == strcmp(role, "pre"))
    return ax::mojom::Role::kPre;
  if (0 == strcmp(role, "presentational"))
    return ax::mojom::Role::kPresentational;
  if (0 == strcmp(role, "progressIndicator"))
    return ax::mojom::Role::kProgressIndicator;
  if (0 == strcmp(role, "radioButton"))
    return ax::mojom::Role::kRadioButton;
  if (0 == strcmp(role, "radioGroup"))
    return ax::mojom::Role::kRadioGroup;
  if (0 == strcmp(role, "region"))
    return ax::mojom::Role::kRegion;
  if (0 == strcmp(role, "rootWebArea"))
    return ax::mojom::Role::kRootWebArea;
  if (0 == strcmp(role, "row"))
    return ax::mojom::Role::kRow;
  if (0 == strcmp(role, "rowGroup"))
    return ax::mojom::Role::kRowGroup;
  if (0 == strcmp(role, "rowHeader"))
    return ax::mojom::Role::kRowHeader;
  if (0 == strcmp(role, "ruby"))
    return ax::mojom::Role::kRuby;
  if (0 == strcmp(role, "rubyAnnotation"))
    return ax::mojom::Role::kRubyAnnotation;
  if (0 == strcmp(role, "section"))
    return ax::mojom::Role::kSection;
  if (0 == strcmp(role, "scrollBar"))
    return ax::mojom::Role::kScrollBar;
  if (0 == strcmp(role, "scrollView"))
    return ax::mojom::Role::kScrollView;
  if (0 == strcmp(role, "search"))
    return ax::mojom::Role::kSearch;
  if (0 == strcmp(role, "searchBox"))
    return ax::mojom::Role::kSearchBox;
  if (0 == strcmp(role, "slider"))
    return ax::mojom::Role::kSlider;
  if (0 == strcmp(role, "sliderThumb"))
    return ax::mojom::Role::kSliderThumb;
  if (0 == strcmp(role, "spinButton"))
    return ax::mojom::Role::kSpinButton;
  if (0 == strcmp(role, "splitter"))
    return ax::mojom::Role::kSplitter;
  if (0 == strcmp(role, "staticText"))
    return ax::mojom::Role::kStaticText;
  if (0 == strcmp(role, "status"))
    return ax::mojom::Role::kStatus;
  if (0 == strcmp(role, "suggestion"))
    return ax::mojom::Role::kSuggestion;
  if (0 == strcmp(role, "svgRoot"))
    return ax::mojom::Role::kSvgRoot;
  if (0 == strcmp(role, "switch"))
    return ax::mojom::Role::kSwitch;
  if (0 == strcmp(role, "strong"))
    return ax::mojom::Role::kStrong;
  if (0 == strcmp(role, "tabList"))
    return ax::mojom::Role::kTabList;
  if (0 == strcmp(role, "tabPanel"))
    return ax::mojom::Role::kTabPanel;
  if (0 == strcmp(role, "tab"))
    return ax::mojom::Role::kTab;
  if (0 == strcmp(role, "tableHeaderContainer"))
    return ax::mojom::Role::kTableHeaderContainer;
  if (0 == strcmp(role, "table"))
    return ax::mojom::Role::kTable;
  if (0 == strcmp(role, "term"))
    return ax::mojom::Role::kTerm;
  if (0 == strcmp(role, "textField"))
    return ax::mojom::Role::kTextField;
  if (0 == strcmp(role, "textFieldWithComboBox"))
    return ax::mojom::Role::kTextFieldWithComboBox;
  if (0 == strcmp(role, "time"))
    return ax::mojom::Role::kTime;
  if (0 == strcmp(role, "timer"))
    return ax::mojom::Role::kTimer;
  if (0 == strcmp(role, "titleBar"))
    return ax::mojom::Role::kTitleBar;
  if (0 == strcmp(role, "toggleButton"))
    return ax::mojom::Role::kToggleButton;
  if (0 == strcmp(role, "toolbar"))
    return ax::mojom::Role::kToolbar;
  if (0 == strcmp(role, "treeGrid"))
    return ax::mojom::Role::kTreeGrid;
  if (0 == strcmp(role, "treeItem"))
    return ax::mojom::Role::kTreeItem;
  if (0 == strcmp(role, "tree"))
    return ax::mojom::Role::kTree;
  if (0 == strcmp(role, "unknown"))
    return ax::mojom::Role::kUnknown;
  if (0 == strcmp(role, "tooltip"))
    return ax::mojom::Role::kTooltip;
  if (0 == strcmp(role, "video"))
    return ax::mojom::Role::kVideo;
  if (0 == strcmp(role, "webArea"))
    return ax::mojom::Role::kWebArea;
  if (0 == strcmp(role, "webView"))
    return ax::mojom::Role::kWebView;
  if (0 == strcmp(role, "window"))
    return ax::mojom::Role::kWindow;
  return ax::mojom::Role::kNone;
}

const char* ToString(ax::mojom::State state) {
  switch (state) {
    case ax::mojom::State::kNone:
      return "none";
    case ax::mojom::State::kAutofillAvailable:
      return "autofillAvailable";
    case ax::mojom::State::kCollapsed:
      return "collapsed";
    case ax::mojom::State::kDefault:
      return "default";
    case ax::mojom::State::kEditable:
      return "editable";
    case ax::mojom::State::kExpanded:
      return "expanded";
    case ax::mojom::State::kFocusable:
      return "focusable";
    case ax::mojom::State::kHorizontal:
      return "horizontal";
    case ax::mojom::State::kHovered:
      return "hovered";
    case ax::mojom::State::kIgnored:
      return "ignored";
    case ax::mojom::State::kInvisible:
      return "invisible";
    case ax::mojom::State::kLinked:
      return "linked";
    case ax::mojom::State::kMultiline:
      return "multiline";
    case ax::mojom::State::kMultiselectable:
      return "multiselectable";
    case ax::mojom::State::kProtected:
      return "protected";
    case ax::mojom::State::kRequired:
      return "required";
    case ax::mojom::State::kRichlyEditable:
      return "richlyEditable";
    case ax::mojom::State::kVertical:
      return "vertical";
    case ax::mojom::State::kVisited:
      return "visited";
  }

  return "";
}

ax::mojom::State ParseState(const char* state) {
  if (0 == strcmp(state, "none"))
    return ax::mojom::State::kNone;
  if (0 == strcmp(state, "autofillAvailable"))
    return ax::mojom::State::kAutofillAvailable;
  if (0 == strcmp(state, "collapsed"))
    return ax::mojom::State::kCollapsed;
  if (0 == strcmp(state, "default"))
    return ax::mojom::State::kDefault;
  if (0 == strcmp(state, "editable"))
    return ax::mojom::State::kEditable;
  if (0 == strcmp(state, "expanded"))
    return ax::mojom::State::kExpanded;
  if (0 == strcmp(state, "focusable"))
    return ax::mojom::State::kFocusable;
  if (0 == strcmp(state, "horizontal"))
    return ax::mojom::State::kHorizontal;
  if (0 == strcmp(state, "hovered"))
    return ax::mojom::State::kHovered;
  if (0 == strcmp(state, "ignored"))
    return ax::mojom::State::kIgnored;
  if (0 == strcmp(state, "invisible"))
    return ax::mojom::State::kInvisible;
  if (0 == strcmp(state, "linked"))
    return ax::mojom::State::kLinked;
  if (0 == strcmp(state, "multiline"))
    return ax::mojom::State::kMultiline;
  if (0 == strcmp(state, "multiselectable"))
    return ax::mojom::State::kMultiselectable;
  if (0 == strcmp(state, "protected"))
    return ax::mojom::State::kProtected;
  if (0 == strcmp(state, "required"))
    return ax::mojom::State::kRequired;
  if (0 == strcmp(state, "richlyEditable"))
    return ax::mojom::State::kRichlyEditable;
  if (0 == strcmp(state, "vertical"))
    return ax::mojom::State::kVertical;
  if (0 == strcmp(state, "visited"))
    return ax::mojom::State::kVisited;
  return ax::mojom::State::kNone;
}

const char* ToString(ax::mojom::Action action) {
  switch (action) {
    case ax::mojom::Action::kNone:
      return "none";
    case ax::mojom::Action::kBlur:
      return "blur";
    case ax::mojom::Action::kClearAccessibilityFocus:
      return "clearAccessibilityFocus";
    case ax::mojom::Action::kCollapse:
      return "collapse";
    case ax::mojom::Action::kCustomAction:
      return "customAction";
    case ax::mojom::Action::kDecrement:
      return "decrement";
    case ax::mojom::Action::kDoDefault:
      return "doDefault";
    case ax::mojom::Action::kExpand:
      return "expand";
    case ax::mojom::Action::kFocus:
      return "focus";
    case ax::mojom::Action::kGetImageData:
      return "getImageData";
    case ax::mojom::Action::kHitTest:
      return "hitTest";
    case ax::mojom::Action::kIncrement:
      return "increment";
    case ax::mojom::Action::kLoadInlineTextBoxes:
      return "loadInlineTextBoxes";
    case ax::mojom::Action::kReplaceSelectedText:
      return "replaceSelectedText";
    case ax::mojom::Action::kScrollBackward:
      return "scrollBackward";
    case ax::mojom::Action::kScrollForward:
      return "scrollForward";
    case ax::mojom::Action::kScrollUp:
      return "scrollUp";
    case ax::mojom::Action::kScrollDown:
      return "scrollDown";
    case ax::mojom::Action::kScrollLeft:
      return "scrollLeft";
    case ax::mojom::Action::kScrollRight:
      return "scrollRight";
    case ax::mojom::Action::kScrollToMakeVisible:
      return "scrollToMakeVisible";
    case ax::mojom::Action::kScrollToPoint:
      return "scrollToPoint";
    case ax::mojom::Action::kSetAccessibilityFocus:
      return "setAccessibilityFocus";
    case ax::mojom::Action::kSetScrollOffset:
      return "setScrollOffset";
    case ax::mojom::Action::kSetSelection:
      return "setSelection";
    case ax::mojom::Action::kSetSequentialFocusNavigationStartingPoint:
      return "setSequentialFocusNavigationStartingPoint";
    case ax::mojom::Action::kSetValue:
      return "setValue";
    case ax::mojom::Action::kShowContextMenu:
      return "showContextMenu";
    case ax::mojom::Action::kGetTextLocation:
      return "getTextLocation";
    case ax::mojom::Action::kAnnotatePageImages:
      return "annotatePageImages";
    case ax::mojom::Action::kSignalEndOfTest:
      return "signalEndOfTest";
    case ax::mojom::Action::kShowTooltip:
      return "showTooltip";
    case ax::mojom::Action::kHideTooltip:
      return "hideTooltip";
    case ax::mojom::Action::kInternalInvalidateTree:
      return "internalInvalidateTree";
  }

  return "";
}

ax::mojom::Action ParseAction(const char* action) {
  if (0 == strcmp(action, "none"))
    return ax::mojom::Action::kNone;
  if (0 == strcmp(action, "annotatePageImages"))
    return ax::mojom::Action::kAnnotatePageImages;
  if (0 == strcmp(action, "blur"))
    return ax::mojom::Action::kBlur;
  if (0 == strcmp(action, "clearAccessibilityFocus"))
    return ax::mojom::Action::kClearAccessibilityFocus;
  if (0 == strcmp(action, "collapse"))
    return ax::mojom::Action::kCollapse;
  if (0 == strcmp(action, "customAction"))
    return ax::mojom::Action::kCustomAction;
  if (0 == strcmp(action, "decrement"))
    return ax::mojom::Action::kDecrement;
  if (0 == strcmp(action, "doDefault"))
    return ax::mojom::Action::kDoDefault;
  if (0 == strcmp(action, "expand"))
    return ax::mojom::Action::kExpand;
  if (0 == strcmp(action, "focus"))
    return ax::mojom::Action::kFocus;
  if (0 == strcmp(action, "getImageData"))
    return ax::mojom::Action::kGetImageData;
  if (0 == strcmp(action, "getTextLocation"))
    return ax::mojom::Action::kGetTextLocation;
  if (0 == strcmp(action, "hitTest"))
    return ax::mojom::Action::kHitTest;
  if (0 == strcmp(action, "increment"))
    return ax::mojom::Action::kIncrement;
  if (0 == strcmp(action, "loadInlineTextBoxes"))
    return ax::mojom::Action::kLoadInlineTextBoxes;
  if (0 == strcmp(action, "replaceSelectedText"))
    return ax::mojom::Action::kReplaceSelectedText;
  if (0 == strcmp(action, "scrollBackward"))
    return ax::mojom::Action::kScrollBackward;
  if (0 == strcmp(action, "scrollForward"))
    return ax::mojom::Action::kScrollForward;
  if (0 == strcmp(action, "scrollUp"))
    return ax::mojom::Action::kScrollUp;
  if (0 == strcmp(action, "scrollDown"))
    return ax::mojom::Action::kScrollDown;
  if (0 == strcmp(action, "scrollLeft"))
    return ax::mojom::Action::kScrollLeft;
  if (0 == strcmp(action, "scrollRight"))
    return ax::mojom::Action::kScrollRight;
  if (0 == strcmp(action, "scrollToMakeVisible"))
    return ax::mojom::Action::kScrollToMakeVisible;
  if (0 == strcmp(action, "scrollToPoint"))
    return ax::mojom::Action::kScrollToPoint;
  if (0 == strcmp(action, "setAccessibilityFocus"))
    return ax::mojom::Action::kSetAccessibilityFocus;
  if (0 == strcmp(action, "setScrollOffset"))
    return ax::mojom::Action::kSetScrollOffset;
  if (0 == strcmp(action, "setSelection"))
    return ax::mojom::Action::kSetSelection;
  if (0 == strcmp(action, "setSequentialFocusNavigationStartingPoint"))
    return ax::mojom::Action::kSetSequentialFocusNavigationStartingPoint;
  if (0 == strcmp(action, "setValue"))
    return ax::mojom::Action::kSetValue;
  if (0 == strcmp(action, "showContextMenu"))
    return ax::mojom::Action::kShowContextMenu;
  if (0 == strcmp(action, "signalEndOfTest"))
    return ax::mojom::Action::kSignalEndOfTest;
  if (0 == strcmp(action, "showTooltip"))
    return ax::mojom::Action::kShowTooltip;
  if (0 == strcmp(action, "hideTooltip"))
    return ax::mojom::Action::kHideTooltip;
  if (0 == strcmp(action, "internalInvalidateTree"))
    return ax::mojom::Action::kInternalInvalidateTree;
  return ax::mojom::Action::kNone;
}

const char* ToString(ax::mojom::ActionFlags action_flags) {
  switch (action_flags) {
    case ax::mojom::ActionFlags::kNone:
      return "none";
    case ax::mojom::ActionFlags::kRequestImages:
      return "requestImages";
    case ax::mojom::ActionFlags::kRequestInlineTextBoxes:
      return "requestInlineTextBoxes";
  }

  return "";
}

ax::mojom::ActionFlags ParseActionFlags(const char* action_flags) {
  if (0 == strcmp(action_flags, "none"))
    return ax::mojom::ActionFlags::kNone;
  if (0 == strcmp(action_flags, "requestImages"))
    return ax::mojom::ActionFlags::kRequestImages;
  if (0 == strcmp(action_flags, "requestInlineTextBoxes"))
    return ax::mojom::ActionFlags::kRequestInlineTextBoxes;
  return ax::mojom::ActionFlags::kNone;
}

const char* ToString(ax::mojom::ScrollAlignment scroll_alignment) {
  switch (scroll_alignment) {
    case ax::mojom::ScrollAlignment::kNone:
      return "none";
    case ax::mojom::ScrollAlignment::kScrollAlignmentCenter:
      return "scrollAlignmentCenter";
    case ax::mojom::ScrollAlignment::kScrollAlignmentTop:
      return "scrollAlignmentTop";
    case ax::mojom::ScrollAlignment::kScrollAlignmentBottom:
      return "scrollAlignmentBottom";
    case ax::mojom::ScrollAlignment::kScrollAlignmentLeft:
      return "scrollAlignmentLeft";
    case ax::mojom::ScrollAlignment::kScrollAlignmentRight:
      return "scrollAlignmentRight";
    case ax::mojom::ScrollAlignment::kScrollAlignmentClosestEdge:
      return "scrollAlignmentClosestEdge";
  }
}

ax::mojom::ScrollAlignment ParseScrollAlignment(const char* scroll_alignment) {
  if (0 == strcmp(scroll_alignment, "none"))
    return ax::mojom::ScrollAlignment::kNone;
  if (0 == strcmp(scroll_alignment, "scrollAlignmentCenter"))
    return ax::mojom::ScrollAlignment::kScrollAlignmentCenter;
  if (0 == strcmp(scroll_alignment, "scrollAlignmentTop"))
    return ax::mojom::ScrollAlignment::kScrollAlignmentTop;
  if (0 == strcmp(scroll_alignment, "scrollAlignmentBottom"))
    return ax::mojom::ScrollAlignment::kScrollAlignmentBottom;
  if (0 == strcmp(scroll_alignment, "scrollAlignmentLeft"))
    return ax::mojom::ScrollAlignment::kScrollAlignmentLeft;
  if (0 == strcmp(scroll_alignment, "scrollAlignmentRight"))
    return ax::mojom::ScrollAlignment::kScrollAlignmentRight;
  if (0 == strcmp(scroll_alignment, "scrollAlignmentClosestEdge"))
    return ax::mojom::ScrollAlignment::kScrollAlignmentClosestEdge;
  return ax::mojom::ScrollAlignment::kNone;
}

const char* ToString(ax::mojom::DefaultActionVerb default_action_verb) {
  switch (default_action_verb) {
    case ax::mojom::DefaultActionVerb::kNone:
      return "none";
    case ax::mojom::DefaultActionVerb::kActivate:
      return "activate";
    case ax::mojom::DefaultActionVerb::kCheck:
      return "check";
    case ax::mojom::DefaultActionVerb::kClick:
      return "click";
    case ax::mojom::DefaultActionVerb::kClickAncestor:
      // Some screen readers, such as Jaws, expect the following spelling of
      // this verb.
      return "click-ancestor";
    case ax::mojom::DefaultActionVerb::kJump:
      return "jump";
    case ax::mojom::DefaultActionVerb::kOpen:
      return "open";
    case ax::mojom::DefaultActionVerb::kPress:
      return "press";
    case ax::mojom::DefaultActionVerb::kSelect:
      return "select";
    case ax::mojom::DefaultActionVerb::kUncheck:
      return "uncheck";
  }

  return "";
}

ax::mojom::DefaultActionVerb ParseDefaultActionVerb(
    const char* default_action_verb) {
  if (0 == strcmp(default_action_verb, "none"))
    return ax::mojom::DefaultActionVerb::kNone;
  if (0 == strcmp(default_action_verb, "activate"))
    return ax::mojom::DefaultActionVerb::kActivate;
  if (0 == strcmp(default_action_verb, "check"))
    return ax::mojom::DefaultActionVerb::kCheck;
  if (0 == strcmp(default_action_verb, "click"))
    return ax::mojom::DefaultActionVerb::kClick;
  // Some screen readers, such as Jaws, expect the following spelling of this
  // verb.
  if (0 == strcmp(default_action_verb, "click-ancestor"))
    return ax::mojom::DefaultActionVerb::kClickAncestor;
  if (0 == strcmp(default_action_verb, "jump"))
    return ax::mojom::DefaultActionVerb::kJump;
  if (0 == strcmp(default_action_verb, "open"))
    return ax::mojom::DefaultActionVerb::kOpen;
  if (0 == strcmp(default_action_verb, "press"))
    return ax::mojom::DefaultActionVerb::kPress;
  if (0 == strcmp(default_action_verb, "select"))
    return ax::mojom::DefaultActionVerb::kSelect;
  if (0 == strcmp(default_action_verb, "uncheck"))
    return ax::mojom::DefaultActionVerb::kUncheck;
  return ax::mojom::DefaultActionVerb::kNone;
}

const char* ToString(ax::mojom::Mutation mutation) {
  switch (mutation) {
    case ax::mojom::Mutation::kNone:
      return "none";
    case ax::mojom::Mutation::kNodeCreated:
      return "nodeCreated";
    case ax::mojom::Mutation::kSubtreeCreated:
      return "subtreeCreated";
    case ax::mojom::Mutation::kNodeChanged:
      return "nodeChanged";
    case ax::mojom::Mutation::kNodeRemoved:
      return "nodeRemoved";
  }

  return "";
}

ax::mojom::Mutation ParseMutation(const char* mutation) {
  if (0 == strcmp(mutation, "none"))
    return ax::mojom::Mutation::kNone;
  if (0 == strcmp(mutation, "nodeCreated"))
    return ax::mojom::Mutation::kNodeCreated;
  if (0 == strcmp(mutation, "subtreeCreated"))
    return ax::mojom::Mutation::kSubtreeCreated;
  if (0 == strcmp(mutation, "nodeChanged"))
    return ax::mojom::Mutation::kNodeChanged;
  if (0 == strcmp(mutation, "nodeRemoved"))
    return ax::mojom::Mutation::kNodeRemoved;
  return ax::mojom::Mutation::kNone;
}

const char* ToString(ax::mojom::StringAttribute string_attribute) {
  switch (string_attribute) {
    case ax::mojom::StringAttribute::kNone:
      return "none";
    case ax::mojom::StringAttribute::kAccessKey:
      return "accessKey";
    case ax::mojom::StringAttribute::kAriaInvalidValue:
      return "ariaInvalidValue";
    case ax::mojom::StringAttribute::kAutoComplete:
      return "autoComplete";
    case ax::mojom::StringAttribute::kChildTreeId:
      return "childTreeId";
    case ax::mojom::StringAttribute::kClassName:
      return "className";
    case ax::mojom::StringAttribute::kContainerLiveRelevant:
      return "containerLiveRelevant";
    case ax::mojom::StringAttribute::kContainerLiveStatus:
      return "containerLiveStatus";
    case ax::mojom::StringAttribute::kDescription:
      return "description";
    case ax::mojom::StringAttribute::kDisplay:
      return "display";
    case ax::mojom::StringAttribute::kFontFamily:
      return "fontFamily";
    case ax::mojom::StringAttribute::kHtmlTag:
      return "htmlTag";
    case ax::mojom::StringAttribute::kImageAnnotation:
      return "imageAnnotation";
    case ax::mojom::StringAttribute::kImageDataUrl:
      return "imageDataUrl";
    case ax::mojom::StringAttribute::kInnerHtml:
      return "innerHtml";
    case ax::mojom::StringAttribute::kInputType:
      return "inputType";
    case ax::mojom::StringAttribute::kKeyShortcuts:
      return "keyShortcuts";
    case ax::mojom::StringAttribute::kLanguage:
      return "language";
    case ax::mojom::StringAttribute::kName:
      return "name";
    case ax::mojom::StringAttribute::kLiveRelevant:
      return "liveRelevant";
    case ax::mojom::StringAttribute::kLiveStatus:
      return "liveStatus";
    case ax::mojom::StringAttribute::kPlaceholder:
      return "placeholder";
    case ax::mojom::StringAttribute::kRole:
      return "role";
    case ax::mojom::StringAttribute::kRoleDescription:
      return "roleDescription";
    case ax::mojom::StringAttribute::kTooltip:
      return "tooltip";
    case ax::mojom::StringAttribute::kUrl:
      return "url";
    case ax::mojom::StringAttribute::kValue:
      return "value";
  }

  return "";
}

ax::mojom::StringAttribute ParseStringAttribute(const char* string_attribute) {
  if (0 == strcmp(string_attribute, "none"))
    return ax::mojom::StringAttribute::kNone;
  if (0 == strcmp(string_attribute, "accessKey"))
    return ax::mojom::StringAttribute::kAccessKey;
  if (0 == strcmp(string_attribute, "ariaInvalidValue"))
    return ax::mojom::StringAttribute::kAriaInvalidValue;
  if (0 == strcmp(string_attribute, "autoComplete"))
    return ax::mojom::StringAttribute::kAutoComplete;
  if (0 == strcmp(string_attribute, "childTreeId"))
    return ax::mojom::StringAttribute::kChildTreeId;
  if (0 == strcmp(string_attribute, "className"))
    return ax::mojom::StringAttribute::kClassName;
  if (0 == strcmp(string_attribute, "containerLiveRelevant"))
    return ax::mojom::StringAttribute::kContainerLiveRelevant;
  if (0 == strcmp(string_attribute, "containerLiveStatus"))
    return ax::mojom::StringAttribute::kContainerLiveStatus;
  if (0 == strcmp(string_attribute, "description"))
    return ax::mojom::StringAttribute::kDescription;
  if (0 == strcmp(string_attribute, "display"))
    return ax::mojom::StringAttribute::kDisplay;
  if (0 == strcmp(string_attribute, "fontFamily"))
    return ax::mojom::StringAttribute::kFontFamily;
  if (0 == strcmp(string_attribute, "htmlTag"))
    return ax::mojom::StringAttribute::kHtmlTag;
  if (0 == strcmp(string_attribute, "imageAnnotation"))
    return ax::mojom::StringAttribute::kImageAnnotation;
  if (0 == strcmp(string_attribute, "imageDataUrl"))
    return ax::mojom::StringAttribute::kImageDataUrl;
  if (0 == strcmp(string_attribute, "innerHtml"))
    return ax::mojom::StringAttribute::kInnerHtml;
  if (0 == strcmp(string_attribute, "inputType"))
    return ax::mojom::StringAttribute::kInputType;
  if (0 == strcmp(string_attribute, "keyShortcuts"))
    return ax::mojom::StringAttribute::kKeyShortcuts;
  if (0 == strcmp(string_attribute, "language"))
    return ax::mojom::StringAttribute::kLanguage;
  if (0 == strcmp(string_attribute, "name"))
    return ax::mojom::StringAttribute::kName;
  if (0 == strcmp(string_attribute, "liveRelevant"))
    return ax::mojom::StringAttribute::kLiveRelevant;
  if (0 == strcmp(string_attribute, "liveStatus"))
    return ax::mojom::StringAttribute::kLiveStatus;
  if (0 == strcmp(string_attribute, "placeholder"))
    return ax::mojom::StringAttribute::kPlaceholder;
  if (0 == strcmp(string_attribute, "role"))
    return ax::mojom::StringAttribute::kRole;
  if (0 == strcmp(string_attribute, "roleDescription"))
    return ax::mojom::StringAttribute::kRoleDescription;
  if (0 == strcmp(string_attribute, "tooltip"))
    return ax::mojom::StringAttribute::kTooltip;
  if (0 == strcmp(string_attribute, "url"))
    return ax::mojom::StringAttribute::kUrl;
  if (0 == strcmp(string_attribute, "value"))
    return ax::mojom::StringAttribute::kValue;
  return ax::mojom::StringAttribute::kNone;
}

const char* ToString(ax::mojom::IntAttribute int_attribute) {
  switch (int_attribute) {
    case ax::mojom::IntAttribute::kNone:
      return "none";
    case ax::mojom::IntAttribute::kDefaultActionVerb:
      return "defaultActionVerb";
    case ax::mojom::IntAttribute::kDropeffect:
      return "dropeffect";
    case ax::mojom::IntAttribute::kScrollX:
      return "scrollX";
    case ax::mojom::IntAttribute::kScrollXMin:
      return "scrollXMin";
    case ax::mojom::IntAttribute::kScrollXMax:
      return "scrollXMax";
    case ax::mojom::IntAttribute::kScrollY:
      return "scrollY";
    case ax::mojom::IntAttribute::kScrollYMin:
      return "scrollYMin";
    case ax::mojom::IntAttribute::kScrollYMax:
      return "scrollYMax";
    case ax::mojom::IntAttribute::kTextSelStart:
      return "textSelStart";
    case ax::mojom::IntAttribute::kTextSelEnd:
      return "textSelEnd";
    case ax::mojom::IntAttribute::kAriaColumnCount:
      return "ariaColumnCount";
    case ax::mojom::IntAttribute::kAriaCellColumnIndex:
      return "ariaCellColumnIndex";
    case ax::mojom::IntAttribute::kAriaCellColumnSpan:
      return "ariaCellColumnSpan";
    case ax::mojom::IntAttribute::kAriaRowCount:
      return "ariaRowCount";
    case ax::mojom::IntAttribute::kAriaCellRowIndex:
      return "ariaCellRowIndex";
    case ax::mojom::IntAttribute::kAriaCellRowSpan:
      return "ariaCellRowSpan";
    case ax::mojom::IntAttribute::kTableRowCount:
      return "tableRowCount";
    case ax::mojom::IntAttribute::kTableColumnCount:
      return "tableColumnCount";
    case ax::mojom::IntAttribute::kTableHeaderId:
      return "tableHeaderId";
    case ax::mojom::IntAttribute::kTableRowIndex:
      return "tableRowIndex";
    case ax::mojom::IntAttribute::kTableRowHeaderId:
      return "tableRowHeaderId";
    case ax::mojom::IntAttribute::kTableColumnIndex:
      return "tableColumnIndex";
    case ax::mojom::IntAttribute::kTableColumnHeaderId:
      return "tableColumnHeaderId";
    case ax::mojom::IntAttribute::kTableCellColumnIndex:
      return "tableCellColumnIndex";
    case ax::mojom::IntAttribute::kTableCellColumnSpan:
      return "tableCellColumnSpan";
    case ax::mojom::IntAttribute::kTableCellRowIndex:
      return "tableCellRowIndex";
    case ax::mojom::IntAttribute::kTableCellRowSpan:
      return "tableCellRowSpan";
    case ax::mojom::IntAttribute::kSortDirection:
      return "sortDirection";
    case ax::mojom::IntAttribute::kHierarchicalLevel:
      return "hierarchicalLevel";
    case ax::mojom::IntAttribute::kNameFrom:
      return "nameFrom";
    case ax::mojom::IntAttribute::kDescriptionFrom:
      return "descriptionFrom";
    case ax::mojom::IntAttribute::kActivedescendantId:
      return "activedescendantId";
    case ax::mojom::IntAttribute::kErrormessageId:
      return "errormessageId";
    case ax::mojom::IntAttribute::kInPageLinkTargetId:
      return "inPageLinkTargetId";
    case ax::mojom::IntAttribute::kMemberOfId:
      return "memberOfId";
    case ax::mojom::IntAttribute::kNextOnLineId:
      return "nextOnLineId";
    case ax::mojom::IntAttribute::kPopupForId:
      return "popupForId";
    case ax::mojom::IntAttribute::kPreviousOnLineId:
      return "previousOnLineId";
    case ax::mojom::IntAttribute::kRestriction:
      return "restriction";
    case ax::mojom::IntAttribute::kSetSize:
      return "setSize";
    case ax::mojom::IntAttribute::kPosInSet:
      return "posInSet";
    case ax::mojom::IntAttribute::kColorValue:
      return "colorValue";
    case ax::mojom::IntAttribute::kAriaCurrentState:
      return "ariaCurrentState";
    case ax::mojom::IntAttribute::kBackgroundColor:
      return "backgroundColor";
    case ax::mojom::IntAttribute::kColor:
      return "color";
    case ax::mojom::IntAttribute::kHasPopup:
      return "haspopup";
    case ax::mojom::IntAttribute::kInvalidState:
      return "invalidState";
    case ax::mojom::IntAttribute::kCheckedState:
      return "checkedState";
    case ax::mojom::IntAttribute::kListStyle:
      return "listStyle";
    case ax::mojom::IntAttribute::kTextAlign:
      return "text-align";
    case ax::mojom::IntAttribute::kTextDirection:
      return "textDirection";
    case ax::mojom::IntAttribute::kTextPosition:
      return "textPosition";
    case ax::mojom::IntAttribute::kTextStyle:
      return "textStyle";
    case ax::mojom::IntAttribute::kTextOverlineStyle:
      return "textOverlineStyle";
    case ax::mojom::IntAttribute::kTextStrikethroughStyle:
      return "textStrikethroughStyle";
    case ax::mojom::IntAttribute::kTextUnderlineStyle:
      return "textUnderlineStyle";
    case ax::mojom::IntAttribute::kPreviousFocusId:
      return "previousFocusId";
    case ax::mojom::IntAttribute::kNextFocusId:
      return "nextFocusId";
    case ax::mojom::IntAttribute::kImageAnnotationStatus:
      return "imageAnnotationStatus";
    case ax::mojom::IntAttribute::kDOMNodeId:
      return "domNodeId";
  }

  return "";
}

ax::mojom::IntAttribute ParseIntAttribute(const char* int_attribute) {
  if (0 == strcmp(int_attribute, "none"))
    return ax::mojom::IntAttribute::kNone;
  if (0 == strcmp(int_attribute, "defaultActionVerb"))
    return ax::mojom::IntAttribute::kDefaultActionVerb;
  if (0 == strcmp(int_attribute, "dropeffect"))
    return ax::mojom::IntAttribute::kDropeffect;
  if (0 == strcmp(int_attribute, "scrollX"))
    return ax::mojom::IntAttribute::kScrollX;
  if (0 == strcmp(int_attribute, "scrollXMin"))
    return ax::mojom::IntAttribute::kScrollXMin;
  if (0 == strcmp(int_attribute, "scrollXMax"))
    return ax::mojom::IntAttribute::kScrollXMax;
  if (0 == strcmp(int_attribute, "scrollY"))
    return ax::mojom::IntAttribute::kScrollY;
  if (0 == strcmp(int_attribute, "scrollYMin"))
    return ax::mojom::IntAttribute::kScrollYMin;
  if (0 == strcmp(int_attribute, "scrollYMax"))
    return ax::mojom::IntAttribute::kScrollYMax;
  if (0 == strcmp(int_attribute, "textSelStart"))
    return ax::mojom::IntAttribute::kTextSelStart;
  if (0 == strcmp(int_attribute, "textSelEnd"))
    return ax::mojom::IntAttribute::kTextSelEnd;
  if (0 == strcmp(int_attribute, "ariaColumnCount"))
    return ax::mojom::IntAttribute::kAriaColumnCount;
  if (0 == strcmp(int_attribute, "ariaCellColumnIndex"))
    return ax::mojom::IntAttribute::kAriaCellColumnIndex;
  if (0 == strcmp(int_attribute, "ariaCellColumnSpan"))
    return ax::mojom::IntAttribute::kAriaCellColumnSpan;
  if (0 == strcmp(int_attribute, "ariaRowCount"))
    return ax::mojom::IntAttribute::kAriaRowCount;
  if (0 == strcmp(int_attribute, "ariaCellRowIndex"))
    return ax::mojom::IntAttribute::kAriaCellRowIndex;
  if (0 == strcmp(int_attribute, "ariaCellRowSpan"))
    return ax::mojom::IntAttribute::kAriaCellRowSpan;
  if (0 == strcmp(int_attribute, "tableRowCount"))
    return ax::mojom::IntAttribute::kTableRowCount;
  if (0 == strcmp(int_attribute, "tableColumnCount"))
    return ax::mojom::IntAttribute::kTableColumnCount;
  if (0 == strcmp(int_attribute, "tableHeaderId"))
    return ax::mojom::IntAttribute::kTableHeaderId;
  if (0 == strcmp(int_attribute, "tableRowIndex"))
    return ax::mojom::IntAttribute::kTableRowIndex;
  if (0 == strcmp(int_attribute, "tableRowHeaderId"))
    return ax::mojom::IntAttribute::kTableRowHeaderId;
  if (0 == strcmp(int_attribute, "tableColumnIndex"))
    return ax::mojom::IntAttribute::kTableColumnIndex;
  if (0 == strcmp(int_attribute, "tableColumnHeaderId"))
    return ax::mojom::IntAttribute::kTableColumnHeaderId;
  if (0 == strcmp(int_attribute, "tableCellColumnIndex"))
    return ax::mojom::IntAttribute::kTableCellColumnIndex;
  if (0 == strcmp(int_attribute, "tableCellColumnSpan"))
    return ax::mojom::IntAttribute::kTableCellColumnSpan;
  if (0 == strcmp(int_attribute, "tableCellRowIndex"))
    return ax::mojom::IntAttribute::kTableCellRowIndex;
  if (0 == strcmp(int_attribute, "tableCellRowSpan"))
    return ax::mojom::IntAttribute::kTableCellRowSpan;
  if (0 == strcmp(int_attribute, "sortDirection"))
    return ax::mojom::IntAttribute::kSortDirection;
  if (0 == strcmp(int_attribute, "hierarchicalLevel"))
    return ax::mojom::IntAttribute::kHierarchicalLevel;
  if (0 == strcmp(int_attribute, "nameFrom"))
    return ax::mojom::IntAttribute::kNameFrom;
  if (0 == strcmp(int_attribute, "descriptionFrom"))
    return ax::mojom::IntAttribute::kDescriptionFrom;
  if (0 == strcmp(int_attribute, "activedescendantId"))
    return ax::mojom::IntAttribute::kActivedescendantId;
  if (0 == strcmp(int_attribute, "errormessageId"))
    return ax::mojom::IntAttribute::kErrormessageId;
  if (0 == strcmp(int_attribute, "inPageLinkTargetId"))
    return ax::mojom::IntAttribute::kInPageLinkTargetId;
  if (0 == strcmp(int_attribute, "memberOfId"))
    return ax::mojom::IntAttribute::kMemberOfId;
  if (0 == strcmp(int_attribute, "nextOnLineId"))
    return ax::mojom::IntAttribute::kNextOnLineId;
  if (0 == strcmp(int_attribute, "popupForId"))
    return ax::mojom::IntAttribute::kPopupForId;
  if (0 == strcmp(int_attribute, "previousOnLineId"))
    return ax::mojom::IntAttribute::kPreviousOnLineId;
  if (0 == strcmp(int_attribute, "restriction"))
    return ax::mojom::IntAttribute::kRestriction;
  if (0 == strcmp(int_attribute, "setSize"))
    return ax::mojom::IntAttribute::kSetSize;
  if (0 == strcmp(int_attribute, "posInSet"))
    return ax::mojom::IntAttribute::kPosInSet;
  if (0 == strcmp(int_attribute, "colorValue"))
    return ax::mojom::IntAttribute::kColorValue;
  if (0 == strcmp(int_attribute, "ariaCurrentState"))
    return ax::mojom::IntAttribute::kAriaCurrentState;
  if (0 == strcmp(int_attribute, "backgroundColor"))
    return ax::mojom::IntAttribute::kBackgroundColor;
  if (0 == strcmp(int_attribute, "color"))
    return ax::mojom::IntAttribute::kColor;
  if (0 == strcmp(int_attribute, "haspopup"))
    return ax::mojom::IntAttribute::kHasPopup;
  if (0 == strcmp(int_attribute, "invalidState"))
    return ax::mojom::IntAttribute::kInvalidState;
  if (0 == strcmp(int_attribute, "checkedState"))
    return ax::mojom::IntAttribute::kCheckedState;
  if (0 == strcmp(int_attribute, "listStyle"))
    return ax::mojom::IntAttribute::kListStyle;
  if (0 == strcmp(int_attribute, "text-align"))
    return ax::mojom::IntAttribute::kTextAlign;
  if (0 == strcmp(int_attribute, "textDirection"))
    return ax::mojom::IntAttribute::kTextDirection;
  if (0 == strcmp(int_attribute, "textPosition"))
    return ax::mojom::IntAttribute::kTextPosition;
  if (0 == strcmp(int_attribute, "textStyle"))
    return ax::mojom::IntAttribute::kTextStyle;
  if (0 == strcmp(int_attribute, "textOverlineStyle"))
    return ax::mojom::IntAttribute::kTextOverlineStyle;
  if (0 == strcmp(int_attribute, "textStrikethroughStyle"))
    return ax::mojom::IntAttribute::kTextStrikethroughStyle;
  if (0 == strcmp(int_attribute, "textUnderlineStyle"))
    return ax::mojom::IntAttribute::kTextUnderlineStyle;
  if (0 == strcmp(int_attribute, "previousFocusId"))
    return ax::mojom::IntAttribute::kPreviousFocusId;
  if (0 == strcmp(int_attribute, "nextFocusId"))
    return ax::mojom::IntAttribute::kNextFocusId;
  if (0 == strcmp(int_attribute, "imageAnnotationStatus"))
    return ax::mojom::IntAttribute::kImageAnnotationStatus;
  if (0 == strcmp(int_attribute, "domNodeId"))
    return ax::mojom::IntAttribute::kDOMNodeId;
  return ax::mojom::IntAttribute::kNone;
}

const char* ToString(ax::mojom::FloatAttribute float_attribute) {
  switch (float_attribute) {
    case ax::mojom::FloatAttribute::kNone:
      return "none";
    case ax::mojom::FloatAttribute::kValueForRange:
      return "valueForRange";
    case ax::mojom::FloatAttribute::kMinValueForRange:
      return "minValueForRange";
    case ax::mojom::FloatAttribute::kMaxValueForRange:
      return "maxValueForRange";
    case ax::mojom::FloatAttribute::kStepValueForRange:
      return "stepValueForRange";
    case ax::mojom::FloatAttribute::kFontSize:
      return "fontSize";
    case ax::mojom::FloatAttribute::kFontWeight:
      return "fontWeight";
    case ax::mojom::FloatAttribute::kTextIndent:
      return "textIndent";
  }

  return "";
}

ax::mojom::FloatAttribute ParseFloatAttribute(const char* float_attribute) {
  if (0 == strcmp(float_attribute, "none"))
    return ax::mojom::FloatAttribute::kNone;
  if (0 == strcmp(float_attribute, "valueForRange"))
    return ax::mojom::FloatAttribute::kValueForRange;
  if (0 == strcmp(float_attribute, "minValueForRange"))
    return ax::mojom::FloatAttribute::kMinValueForRange;
  if (0 == strcmp(float_attribute, "maxValueForRange"))
    return ax::mojom::FloatAttribute::kMaxValueForRange;
  if (0 == strcmp(float_attribute, "stepValueForRange"))
    return ax::mojom::FloatAttribute::kStepValueForRange;
  if (0 == strcmp(float_attribute, "fontSize"))
    return ax::mojom::FloatAttribute::kFontSize;
  if (0 == strcmp(float_attribute, "fontWeight"))
    return ax::mojom::FloatAttribute::kFontWeight;
  if (0 == strcmp(float_attribute, "textIndent"))
    return ax::mojom::FloatAttribute::kTextIndent;
  return ax::mojom::FloatAttribute::kNone;
}

const char* ToString(ax::mojom::BoolAttribute bool_attribute) {
  switch (bool_attribute) {
    case ax::mojom::BoolAttribute::kNone:
      return "none";
    case ax::mojom::BoolAttribute::kBusy:
      return "busy";
    case ax::mojom::BoolAttribute::kEditableRoot:
      return "editableRoot";
    case ax::mojom::BoolAttribute::kContainerLiveAtomic:
      return "containerLiveAtomic";
    case ax::mojom::BoolAttribute::kContainerLiveBusy:
      return "containerLiveBusy";
    case ax::mojom::BoolAttribute::kGrabbed:
      return "grabbed";
    case ax::mojom::BoolAttribute::kLiveAtomic:
      return "liveAtomic";
    case ax::mojom::BoolAttribute::kModal:
      return "modal";
    case ax::mojom::BoolAttribute::kUpdateLocationOnly:
      return "updateLocationOnly";
    case ax::mojom::BoolAttribute::kCanvasHasFallback:
      return "canvasHasFallback";
    case ax::mojom::BoolAttribute::kScrollable:
      return "scrollable";
    case ax::mojom::BoolAttribute::kClickable:
      return "clickable";
    case ax::mojom::BoolAttribute::kClipsChildren:
      return "clipsChildren";
    case ax::mojom::BoolAttribute::kNotUserSelectableStyle:
      return "notUserSelectableStyle";
    case ax::mojom::BoolAttribute::kSelected:
      return "selected";
    case ax::mojom::BoolAttribute::kSelectedFromFocus:
      return "selectedFromFocus";
    case ax::mojom::BoolAttribute::kSupportsTextLocation:
      return "supportsTextLocation";
    case ax::mojom::BoolAttribute::kIsLineBreakingObject:
      return "isLineBreakingObject";
    case ax::mojom::BoolAttribute::kIsPageBreakingObject:
      return "isPageBreakingObject";
    case ax::mojom::BoolAttribute::kHasAriaAttribute:
      return "hasAriaAttribute";
  }

  return "";
}

ax::mojom::BoolAttribute ParseBoolAttribute(const char* bool_attribute) {
  if (0 == strcmp(bool_attribute, "none"))
    return ax::mojom::BoolAttribute::kNone;
  if (0 == strcmp(bool_attribute, "busy"))
    return ax::mojom::BoolAttribute::kBusy;
  if (0 == strcmp(bool_attribute, "editableRoot"))
    return ax::mojom::BoolAttribute::kEditableRoot;
  if (0 == strcmp(bool_attribute, "containerLiveAtomic"))
    return ax::mojom::BoolAttribute::kContainerLiveAtomic;
  if (0 == strcmp(bool_attribute, "containerLiveBusy"))
    return ax::mojom::BoolAttribute::kContainerLiveBusy;
  if (0 == strcmp(bool_attribute, "grabbed"))
    return ax::mojom::BoolAttribute::kGrabbed;
  if (0 == strcmp(bool_attribute, "liveAtomic"))
    return ax::mojom::BoolAttribute::kLiveAtomic;
  if (0 == strcmp(bool_attribute, "modal"))
    return ax::mojom::BoolAttribute::kModal;
  if (0 == strcmp(bool_attribute, "updateLocationOnly"))
    return ax::mojom::BoolAttribute::kUpdateLocationOnly;
  if (0 == strcmp(bool_attribute, "canvasHasFallback"))
    return ax::mojom::BoolAttribute::kCanvasHasFallback;
  if (0 == strcmp(bool_attribute, "scrollable"))
    return ax::mojom::BoolAttribute::kScrollable;
  if (0 == strcmp(bool_attribute, "clickable"))
    return ax::mojom::BoolAttribute::kClickable;
  if (0 == strcmp(bool_attribute, "clipsChildren"))
    return ax::mojom::BoolAttribute::kClipsChildren;
  if (0 == strcmp(bool_attribute, "notUserSelectableStyle"))
    return ax::mojom::BoolAttribute::kNotUserSelectableStyle;
  if (0 == strcmp(bool_attribute, "selected"))
    return ax::mojom::BoolAttribute::kSelected;
  if (0 == strcmp(bool_attribute, "selectedFromFocus"))
    return ax::mojom::BoolAttribute::kSelectedFromFocus;
  if (0 == strcmp(bool_attribute, "supportsTextLocation"))
    return ax::mojom::BoolAttribute::kSupportsTextLocation;
  if (0 == strcmp(bool_attribute, "isLineBreakingObject"))
    return ax::mojom::BoolAttribute::kIsLineBreakingObject;
  if (0 == strcmp(bool_attribute, "isPageBreakingObject"))
    return ax::mojom::BoolAttribute::kIsPageBreakingObject;
  if (0 == strcmp(bool_attribute, "hasAriaAttribute"))
    return ax::mojom::BoolAttribute::kHasAriaAttribute;
  return ax::mojom::BoolAttribute::kNone;
}

const char* ToString(ax::mojom::IntListAttribute int_list_attribute) {
  switch (int_list_attribute) {
    case ax::mojom::IntListAttribute::kNone:
      return "none";
    case ax::mojom::IntListAttribute::kIndirectChildIds:
      return "indirectChildIds";
    case ax::mojom::IntListAttribute::kControlsIds:
      return "controlsIds";
    case ax::mojom::IntListAttribute::kDetailsIds:
      return "detailsIds";
    case ax::mojom::IntListAttribute::kDescribedbyIds:
      return "describedbyIds";
    case ax::mojom::IntListAttribute::kFlowtoIds:
      return "flowtoIds";
    case ax::mojom::IntListAttribute::kLabelledbyIds:
      return "labelledbyIds";
    case ax::mojom::IntListAttribute::kRadioGroupIds:
      return "radioGroupIds";
    case ax::mojom::IntListAttribute::kMarkerTypes:
      return "markerTypes";
    case ax::mojom::IntListAttribute::kMarkerStarts:
      return "markerStarts";
    case ax::mojom::IntListAttribute::kMarkerEnds:
      return "markerEnds";
    case ax::mojom::IntListAttribute::kCharacterOffsets:
      return "characterOffsets";
    case ax::mojom::IntListAttribute::kCachedLineStarts:
      return "cachedLineStarts";
    case ax::mojom::IntListAttribute::kWordStarts:
      return "wordStarts";
    case ax::mojom::IntListAttribute::kWordEnds:
      return "wordEnds";
    case ax::mojom::IntListAttribute::kCustomActionIds:
      return "customActionIds";
  }

  return "";
}

ax::mojom::IntListAttribute ParseIntListAttribute(
    const char* int_list_attribute) {
  if (0 == strcmp(int_list_attribute, "none"))
    return ax::mojom::IntListAttribute::kNone;
  if (0 == strcmp(int_list_attribute, "indirectChildIds"))
    return ax::mojom::IntListAttribute::kIndirectChildIds;
  if (0 == strcmp(int_list_attribute, "controlsIds"))
    return ax::mojom::IntListAttribute::kControlsIds;
  if (0 == strcmp(int_list_attribute, "detailsIds"))
    return ax::mojom::IntListAttribute::kDetailsIds;
  if (0 == strcmp(int_list_attribute, "describedbyIds"))
    return ax::mojom::IntListAttribute::kDescribedbyIds;
  if (0 == strcmp(int_list_attribute, "flowtoIds"))
    return ax::mojom::IntListAttribute::kFlowtoIds;
  if (0 == strcmp(int_list_attribute, "labelledbyIds"))
    return ax::mojom::IntListAttribute::kLabelledbyIds;
  if (0 == strcmp(int_list_attribute, "radioGroupIds"))
    return ax::mojom::IntListAttribute::kRadioGroupIds;
  if (0 == strcmp(int_list_attribute, "markerTypes"))
    return ax::mojom::IntListAttribute::kMarkerTypes;
  if (0 == strcmp(int_list_attribute, "markerStarts"))
    return ax::mojom::IntListAttribute::kMarkerStarts;
  if (0 == strcmp(int_list_attribute, "markerEnds"))
    return ax::mojom::IntListAttribute::kMarkerEnds;
  if (0 == strcmp(int_list_attribute, "characterOffsets"))
    return ax::mojom::IntListAttribute::kCharacterOffsets;
  if (0 == strcmp(int_list_attribute, "cachedLineStarts"))
    return ax::mojom::IntListAttribute::kCachedLineStarts;
  if (0 == strcmp(int_list_attribute, "wordStarts"))
    return ax::mojom::IntListAttribute::kWordStarts;
  if (0 == strcmp(int_list_attribute, "wordEnds"))
    return ax::mojom::IntListAttribute::kWordEnds;
  if (0 == strcmp(int_list_attribute, "customActionIds"))
    return ax::mojom::IntListAttribute::kCustomActionIds;
  return ax::mojom::IntListAttribute::kNone;
}

const char* ToString(ax::mojom::StringListAttribute string_list_attribute) {
  switch (string_list_attribute) {
    case ax::mojom::StringListAttribute::kNone:
      return "none";
    case ax::mojom::StringListAttribute::kCustomActionDescriptions:
      return "customActionDescriptions";
  }

  return "";
}

ax::mojom::StringListAttribute ParseStringListAttribute(
    const char* string_list_attribute) {
  if (0 == strcmp(string_list_attribute, "none"))
    return ax::mojom::StringListAttribute::kNone;
  if (0 == strcmp(string_list_attribute, "customActionDescriptions"))
    return ax::mojom::StringListAttribute::kCustomActionDescriptions;
  return ax::mojom::StringListAttribute::kNone;
}

const char* ToString(ax::mojom::ListStyle list_style) {
  switch (list_style) {
    case ax::mojom::ListStyle::kNone:
      return "none";
    case ax::mojom::ListStyle::kCircle:
      return "circle";
    case ax::mojom::ListStyle::kDisc:
      return "disc";
    case ax::mojom::ListStyle::kImage:
      return "image";
    case ax::mojom::ListStyle::kNumeric:
      return "numeric";
    case ax::mojom::ListStyle::kOther:
      return "other";
    case ax::mojom::ListStyle::kSquare:
      return "square";
  }

  return "";
}

ax::mojom::ListStyle ParseListStyle(const char* list_style) {
  if (0 == strcmp(list_style, "none"))
    return ax::mojom::ListStyle::kNone;
  if (0 == strcmp(list_style, "circle"))
    return ax::mojom::ListStyle::kCircle;
  if (0 == strcmp(list_style, "disc"))
    return ax::mojom::ListStyle::kDisc;
  if (0 == strcmp(list_style, "image"))
    return ax::mojom::ListStyle::kImage;
  if (0 == strcmp(list_style, "numeric"))
    return ax::mojom::ListStyle::kNumeric;
  if (0 == strcmp(list_style, "other"))
    return ax::mojom::ListStyle::kOther;
  if (0 == strcmp(list_style, "square"))
    return ax::mojom::ListStyle::kSquare;
  return ax::mojom::ListStyle::kNone;
}

const char* ToString(ax::mojom::MarkerType marker_type) {
  switch (marker_type) {
    case ax::mojom::MarkerType::kNone:
      return "none";
    case ax::mojom::MarkerType::kSpelling:
      return "spelling";
    case ax::mojom::MarkerType::kGrammar:
      return "grammar";
    case ax::mojom::MarkerType::kTextMatch:
      return "textMatch";
    case ax::mojom::MarkerType::kActiveSuggestion:
      return "activeSuggestion";
    case ax::mojom::MarkerType::kSuggestion:
      return "suggestion";
  }

  return "";
}

ax::mojom::MarkerType ParseMarkerType(const char* marker_type) {
  if (0 == strcmp(marker_type, "none"))
    return ax::mojom::MarkerType::kNone;
  if (0 == strcmp(marker_type, "spelling"))
    return ax::mojom::MarkerType::kSpelling;
  if (0 == strcmp(marker_type, "grammar"))
    return ax::mojom::MarkerType::kGrammar;
  if (0 == strcmp(marker_type, "textMatch"))
    return ax::mojom::MarkerType::kTextMatch;
  if (0 == strcmp(marker_type, "activeSuggestion"))
    return ax::mojom::MarkerType::kActiveSuggestion;
  if (0 == strcmp(marker_type, "suggestion"))
    return ax::mojom::MarkerType::kSuggestion;
  return ax::mojom::MarkerType::kNone;
}

const char* ToString(ax::mojom::MoveDirection move_direction) {
  switch (move_direction) {
    case ax::mojom::MoveDirection::kForward:
      return "forward";
    case ax::mojom::MoveDirection::kBackward:
      return "backward";
  }

  return "";
}

ax::mojom::MoveDirection ParseMoveDirection(const char* move_direction) {
  if (0 == strcmp(move_direction, "forward"))
    return ax::mojom::MoveDirection::kForward;
  if (0 == strcmp(move_direction, "backward"))
    return ax::mojom::MoveDirection::kBackward;
  return ax::mojom::MoveDirection::kForward;
}

const char* ToString(ax::mojom::Command command) {
  switch (command) {
    case ax::mojom::Command::kClearSelection:
      return "clearSelection";
    case ax::mojom::Command::kCut:
      return "cut";
    case ax::mojom::Command::kDelete:
      return "delete";
    case ax::mojom::Command::kDictate:
      return "dictate";
    case ax::mojom::Command::kExtendSelection:
      return "extendSelection";
    case ax::mojom::Command::kFormat:
      return "format";
    case ax::mojom::Command::kInsert:
      return "insert";
    case ax::mojom::Command::kMarker:
      return "marker";
    case ax::mojom::Command::kMoveSelection:
      return "moveSelection";
    case ax::mojom::Command::kPaste:
      return "paste";
    case ax::mojom::Command::kReplace:
      return "replace";
    case ax::mojom::Command::kSetSelection:
      return "setSelection";
    case ax::mojom::Command::kType:
      return "type";
  }

  return "";
}

ax::mojom::Command ParseCommand(const char* command) {
  if (0 == strcmp(command, "clearSelection"))
    return ax::mojom::Command::kClearSelection;
  if (0 == strcmp(command, "cut"))
    return ax::mojom::Command::kCut;
  if (0 == strcmp(command, "delete"))
    return ax::mojom::Command::kDelete;
  if (0 == strcmp(command, "dictate"))
    return ax::mojom::Command::kDictate;
  if (0 == strcmp(command, "extendSelection"))
    return ax::mojom::Command::kExtendSelection;
  if (0 == strcmp(command, "format"))
    return ax::mojom::Command::kFormat;
  if (0 == strcmp(command, "insert"))
    return ax::mojom::Command::kInsert;
  if (0 == strcmp(command, "marker"))
    return ax::mojom::Command::kMarker;
  if (0 == strcmp(command, "moveSelection"))
    return ax::mojom::Command::kMoveSelection;
  if (0 == strcmp(command, "paste"))
    return ax::mojom::Command::kPaste;
  if (0 == strcmp(command, "replace"))
    return ax::mojom::Command::kReplace;
  if (0 == strcmp(command, "setSelection"))
    return ax::mojom::Command::kSetSelection;
  if (0 == strcmp(command, "type"))
    return ax::mojom::Command::kType;

  // Return the default command.
  return ax::mojom::Command::kType;
}

const char* ToString(ax::mojom::TextBoundary text_boundary) {
  switch (text_boundary) {
    case ax::mojom::TextBoundary::kCharacter:
      return "character";
    case ax::mojom::TextBoundary::kFormat:
      return "format";
    case ax::mojom::TextBoundary::kLineEnd:
      return "lineEnd";
    case ax::mojom::TextBoundary::kLineStart:
      return "lineStart";
    case ax::mojom::TextBoundary::kLineStartOrEnd:
      return "lineStartOrEnd";
    case ax::mojom::TextBoundary::kObject:
      return "object";
    case ax::mojom::TextBoundary::kPageEnd:
      return "pageEnd";
    case ax::mojom::TextBoundary::kPageStart:
      return "pageStart";
    case ax::mojom::TextBoundary::kPageStartOrEnd:
      return "pageStartOrEnd";
    case ax::mojom::TextBoundary::kParagraphEnd:
      return "paragraphEnd";
    case ax::mojom::TextBoundary::kParagraphStart:
      return "paragraphStart";
    case ax::mojom::TextBoundary::kParagraphStartOrEnd:
      return "paragraphStartOrEnd";
    case ax::mojom::TextBoundary::kSentenceEnd:
      return "sentenceEnd";
    case ax::mojom::TextBoundary::kSentenceStart:
      return "sentenceStart";
    case ax::mojom::TextBoundary::kSentenceStartOrEnd:
      return "sentenceStartOrEnd";
    case ax::mojom::TextBoundary::kWebPage:
      return "webPage";
    case ax::mojom::TextBoundary::kWordEnd:
      return "wordEnd";
    case ax::mojom::TextBoundary::kWordStart:
      return "wordStart";
    case ax::mojom::TextBoundary::kWordStartOrEnd:
      return "wordStartOrEnd";
  }

  return "";
}

ax::mojom::TextBoundary ParseTextBoundary(const char* text_boundary) {
  if (0 == strcmp(text_boundary, "object"))
    return ax::mojom::TextBoundary::kObject;
  if (0 == strcmp(text_boundary, "character"))
    return ax::mojom::TextBoundary::kCharacter;
  if (0 == strcmp(text_boundary, "format"))
    return ax::mojom::TextBoundary::kFormat;
  if (0 == strcmp(text_boundary, "lineEnd"))
    return ax::mojom::TextBoundary::kLineEnd;
  if (0 == strcmp(text_boundary, "lineStart"))
    return ax::mojom::TextBoundary::kLineStart;
  if (0 == strcmp(text_boundary, "lineStartOrEnd"))
    return ax::mojom::TextBoundary::kLineStartOrEnd;
  if (0 == strcmp(text_boundary, "pageEnd"))
    return ax::mojom::TextBoundary::kPageEnd;
  if (0 == strcmp(text_boundary, "pageStart"))
    return ax::mojom::TextBoundary::kPageStart;
  if (0 == strcmp(text_boundary, "pageStartOrEnd"))
    return ax::mojom::TextBoundary::kPageStartOrEnd;
  if (0 == strcmp(text_boundary, "paragraphEnd"))
    return ax::mojom::TextBoundary::kParagraphEnd;
  if (0 == strcmp(text_boundary, "paragraphStart"))
    return ax::mojom::TextBoundary::kParagraphStart;
  if (0 == strcmp(text_boundary, "paragraphStartOrEnd"))
    return ax::mojom::TextBoundary::kParagraphStartOrEnd;
  if (0 == strcmp(text_boundary, "sentenceEnd"))
    return ax::mojom::TextBoundary::kSentenceEnd;
  if (0 == strcmp(text_boundary, "sentenceStart"))
    return ax::mojom::TextBoundary::kSentenceStart;
  if (0 == strcmp(text_boundary, "sentenceStartOrEnd"))
    return ax::mojom::TextBoundary::kSentenceStartOrEnd;
  if (0 == strcmp(text_boundary, "webPage"))
    return ax::mojom::TextBoundary::kWebPage;
  if (0 == strcmp(text_boundary, "wordEnd"))
    return ax::mojom::TextBoundary::kWordEnd;
  if (0 == strcmp(text_boundary, "wordStart"))
    return ax::mojom::TextBoundary::kWordStart;
  if (0 == strcmp(text_boundary, "wordStartOrEnd"))
    return ax::mojom::TextBoundary::kWordStartOrEnd;
  return ax::mojom::TextBoundary::kObject;
}

const char* ToString(ax::mojom::TextDecorationStyle text_decoration_style) {
  switch (text_decoration_style) {
    case ax::mojom::TextDecorationStyle::kNone:
      return "none";
    case ax::mojom::TextDecorationStyle::kSolid:
      return "solid";
    case ax::mojom::TextDecorationStyle::kDashed:
      return "dashed";
    case ax::mojom::TextDecorationStyle::kDotted:
      return "dotted";
    case ax::mojom::TextDecorationStyle::kDouble:
      return "double";
    case ax::mojom::TextDecorationStyle::kWavy:
      return "wavy";
  }

  return "";
}

ax::mojom::TextDecorationStyle ParseTextDecorationStyle(
    const char* text_decoration_style) {
  if (0 == strcmp(text_decoration_style, "none"))
    return ax::mojom::TextDecorationStyle::kNone;
  if (0 == strcmp(text_decoration_style, "solid"))
    return ax::mojom::TextDecorationStyle::kSolid;
  if (0 == strcmp(text_decoration_style, "dashed"))
    return ax::mojom::TextDecorationStyle::kDashed;
  if (0 == strcmp(text_decoration_style, "dotted"))
    return ax::mojom::TextDecorationStyle::kDotted;
  if (0 == strcmp(text_decoration_style, "double"))
    return ax::mojom::TextDecorationStyle::kDouble;
  if (0 == strcmp(text_decoration_style, "wavy"))
    return ax::mojom::TextDecorationStyle::kWavy;
  return ax::mojom::TextDecorationStyle::kNone;
}

const char* ToString(ax::mojom::TextAlign text_align) {
  switch (text_align) {
    case ax::mojom::TextAlign::kNone:
      return "none";
    case ax::mojom::TextAlign::kLeft:
      return "left";
    case ax::mojom::TextAlign::kRight:
      return "right";
    case ax::mojom::TextAlign::kCenter:
      return "center";
    case ax::mojom::TextAlign::kJustify:
      return "justify";
  }

  return "";
}

ax::mojom::TextAlign ParseTextAlign(const char* text_align) {
  if (0 == strcmp(text_align, "none"))
    return ax::mojom::TextAlign::kNone;
  if (0 == strcmp(text_align, "left"))
    return ax::mojom::TextAlign::kLeft;
  if (0 == strcmp(text_align, "right"))
    return ax::mojom::TextAlign::kRight;
  if (0 == strcmp(text_align, "center"))
    return ax::mojom::TextAlign::kCenter;
  if (0 == strcmp(text_align, "justify"))
    return ax::mojom::TextAlign::kJustify;
  return ax::mojom::TextAlign::kNone;
}

const char* ToString(ax::mojom::WritingDirection text_direction) {
  switch (text_direction) {
    case ax::mojom::WritingDirection::kNone:
      return "none";
    case ax::mojom::WritingDirection::kLtr:
      return "ltr";
    case ax::mojom::WritingDirection::kRtl:
      return "rtl";
    case ax::mojom::WritingDirection::kTtb:
      return "ttb";
    case ax::mojom::WritingDirection::kBtt:
      return "btt";
  }

  return "";
}

ax::mojom::WritingDirection ParseTextDirection(const char* text_direction) {
  if (0 == strcmp(text_direction, "none"))
    return ax::mojom::WritingDirection::kNone;
  if (0 == strcmp(text_direction, "ltr"))
    return ax::mojom::WritingDirection::kLtr;
  if (0 == strcmp(text_direction, "rtl"))
    return ax::mojom::WritingDirection::kRtl;
  if (0 == strcmp(text_direction, "ttb"))
    return ax::mojom::WritingDirection::kTtb;
  if (0 == strcmp(text_direction, "btt"))
    return ax::mojom::WritingDirection::kBtt;
  return ax::mojom::WritingDirection::kNone;
}

const char* ToString(ax::mojom::TextPosition text_position) {
  switch (text_position) {
    case ax::mojom::TextPosition::kNone:
      return "none";
    case ax::mojom::TextPosition::kSubscript:
      return "subscript";
    case ax::mojom::TextPosition::kSuperscript:
      return "superscript";
  }

  return "";
}

ax::mojom::TextPosition ParseTextPosition(const char* text_position) {
  if (0 == strcmp(text_position, "none"))
    return ax::mojom::TextPosition::kNone;
  if (0 == strcmp(text_position, "subscript"))
    return ax::mojom::TextPosition::kSubscript;
  if (0 == strcmp(text_position, "superscript"))
    return ax::mojom::TextPosition::kSuperscript;
  return ax::mojom::TextPosition::kNone;
}

const char* ToString(ax::mojom::TextStyle text_style) {
  switch (text_style) {
    case ax::mojom::TextStyle::kNone:
      return "none";
    case ax::mojom::TextStyle::kBold:
      return "bold";
    case ax::mojom::TextStyle::kItalic:
      return "italic";
    case ax::mojom::TextStyle::kUnderline:
      return "underline";
    case ax::mojom::TextStyle::kLineThrough:
      return "lineThrough";
    case ax::mojom::TextStyle::kOverline:
      return "overline";
  }

  return "";
}

ax::mojom::TextStyle ParseTextStyle(const char* text_style) {
  if (0 == strcmp(text_style, "none"))
    return ax::mojom::TextStyle::kNone;
  if (0 == strcmp(text_style, "bold"))
    return ax::mojom::TextStyle::kBold;
  if (0 == strcmp(text_style, "italic"))
    return ax::mojom::TextStyle::kItalic;
  if (0 == strcmp(text_style, "underline"))
    return ax::mojom::TextStyle::kUnderline;
  if (0 == strcmp(text_style, "lineThrough"))
    return ax::mojom::TextStyle::kLineThrough;
  if (0 == strcmp(text_style, "overline"))
    return ax::mojom::TextStyle::kOverline;
  return ax::mojom::TextStyle::kNone;
}

const char* ToString(ax::mojom::AriaCurrentState aria_current_state) {
  switch (aria_current_state) {
    case ax::mojom::AriaCurrentState::kNone:
      return "none";
    case ax::mojom::AriaCurrentState::kFalse:
      return "false";
    case ax::mojom::AriaCurrentState::kTrue:
      return "true";
    case ax::mojom::AriaCurrentState::kPage:
      return "page";
    case ax::mojom::AriaCurrentState::kStep:
      return "step";
    case ax::mojom::AriaCurrentState::kLocation:
      return "location";
    case ax::mojom::AriaCurrentState::kUnclippedLocation:
      return "unclippedLocation";
    case ax::mojom::AriaCurrentState::kDate:
      return "date";
    case ax::mojom::AriaCurrentState::kTime:
      return "time";
  }

  return "";
}

ax::mojom::AriaCurrentState ParseAriaCurrentState(
    const char* aria_current_state) {
  if (0 == strcmp(aria_current_state, "none"))
    return ax::mojom::AriaCurrentState::kNone;
  if (0 == strcmp(aria_current_state, "false"))
    return ax::mojom::AriaCurrentState::kFalse;
  if (0 == strcmp(aria_current_state, "true"))
    return ax::mojom::AriaCurrentState::kTrue;
  if (0 == strcmp(aria_current_state, "page"))
    return ax::mojom::AriaCurrentState::kPage;
  if (0 == strcmp(aria_current_state, "step"))
    return ax::mojom::AriaCurrentState::kStep;
  if (0 == strcmp(aria_current_state, "location"))
    return ax::mojom::AriaCurrentState::kLocation;
  if (0 == strcmp(aria_current_state, "unclippedLocation"))
    return ax::mojom::AriaCurrentState::kUnclippedLocation;
  if (0 == strcmp(aria_current_state, "date"))
    return ax::mojom::AriaCurrentState::kDate;
  if (0 == strcmp(aria_current_state, "time"))
    return ax::mojom::AriaCurrentState::kTime;
  return ax::mojom::AriaCurrentState::kNone;
}

const char* ToString(ax::mojom::HasPopup has_popup) {
  switch (has_popup) {
    case ax::mojom::HasPopup::kFalse:
      return "";
    case ax::mojom::HasPopup::kTrue:
      return "true";
    case ax::mojom::HasPopup::kMenu:
      return "menu";
    case ax::mojom::HasPopup::kListbox:
      return "listbox";
    case ax::mojom::HasPopup::kTree:
      return "tree";
    case ax::mojom::HasPopup::kGrid:
      return "grid";
    case ax::mojom::HasPopup::kDialog:
      return "dialog";
  }

  return "";
}

ax::mojom::HasPopup ParseHasPopup(const char* has_popup) {
  if (0 == strcmp(has_popup, "true"))
    return ax::mojom::HasPopup::kTrue;
  if (0 == strcmp(has_popup, "menu"))
    return ax::mojom::HasPopup::kMenu;
  if (0 == strcmp(has_popup, "listbox"))
    return ax::mojom::HasPopup::kListbox;
  if (0 == strcmp(has_popup, "tree"))
    return ax::mojom::HasPopup::kTree;
  if (0 == strcmp(has_popup, "grid"))
    return ax::mojom::HasPopup::kGrid;
  if (0 == strcmp(has_popup, "dialog"))
    return ax::mojom::HasPopup::kDialog;

  return ax::mojom::HasPopup::kFalse;
}

const char* ToString(ax::mojom::InvalidState invalid_state) {
  switch (invalid_state) {
    case ax::mojom::InvalidState::kNone:
      return "none";
    case ax::mojom::InvalidState::kFalse:
      return "false";
    case ax::mojom::InvalidState::kTrue:
      return "true";
    case ax::mojom::InvalidState::kOther:
      return "other";
  }

  return "";
}

ax::mojom::InvalidState ParseInvalidState(const char* invalid_state) {
  if (0 == strcmp(invalid_state, "none"))
    return ax::mojom::InvalidState::kNone;
  if (0 == strcmp(invalid_state, "false"))
    return ax::mojom::InvalidState::kFalse;
  if (0 == strcmp(invalid_state, "true"))
    return ax::mojom::InvalidState::kTrue;
  if (0 == strcmp(invalid_state, "other"))
    return ax::mojom::InvalidState::kOther;
  return ax::mojom::InvalidState::kNone;
}

const char* ToString(ax::mojom::Restriction restriction) {
  switch (restriction) {
    case ax::mojom::Restriction::kNone:
      return "none";
    case ax::mojom::Restriction::kReadOnly:
      return "readOnly";
    case ax::mojom::Restriction::kDisabled:
      return "disabled";
  }

  return "";
}

ax::mojom::Restriction ParseRestriction(const char* restriction) {
  if (0 == strcmp(restriction, "none"))
    return ax::mojom::Restriction::kNone;
  if (0 == strcmp(restriction, "readOnly"))
    return ax::mojom::Restriction::kReadOnly;
  if (0 == strcmp(restriction, "disabled"))
    return ax::mojom::Restriction::kDisabled;
  return ax::mojom::Restriction::kNone;
}

const char* ToString(ax::mojom::CheckedState checked_state) {
  switch (checked_state) {
    case ax::mojom::CheckedState::kNone:
      return "none";
    case ax::mojom::CheckedState::kFalse:
      return "false";
    case ax::mojom::CheckedState::kTrue:
      return "true";
    case ax::mojom::CheckedState::kMixed:
      return "mixed";
  }

  return "";
}

ax::mojom::CheckedState ParseCheckedState(const char* checked_state) {
  if (0 == strcmp(checked_state, "none"))
    return ax::mojom::CheckedState::kNone;
  if (0 == strcmp(checked_state, "false"))
    return ax::mojom::CheckedState::kFalse;
  if (0 == strcmp(checked_state, "true"))
    return ax::mojom::CheckedState::kTrue;
  if (0 == strcmp(checked_state, "mixed"))
    return ax::mojom::CheckedState::kMixed;
  return ax::mojom::CheckedState::kNone;
}

const char* ToString(ax::mojom::SortDirection sort_direction) {
  switch (sort_direction) {
    case ax::mojom::SortDirection::kNone:
      return "none";
    case ax::mojom::SortDirection::kUnsorted:
      return "unsorted";
    case ax::mojom::SortDirection::kAscending:
      return "ascending";
    case ax::mojom::SortDirection::kDescending:
      return "descending";
    case ax::mojom::SortDirection::kOther:
      return "other";
  }

  return "";
}

ax::mojom::SortDirection ParseSortDirection(const char* sort_direction) {
  if (0 == strcmp(sort_direction, "none"))
    return ax::mojom::SortDirection::kNone;
  if (0 == strcmp(sort_direction, "unsorted"))
    return ax::mojom::SortDirection::kUnsorted;
  if (0 == strcmp(sort_direction, "ascending"))
    return ax::mojom::SortDirection::kAscending;
  if (0 == strcmp(sort_direction, "descending"))
    return ax::mojom::SortDirection::kDescending;
  if (0 == strcmp(sort_direction, "other"))
    return ax::mojom::SortDirection::kOther;
  return ax::mojom::SortDirection::kNone;
}

const char* ToString(ax::mojom::NameFrom name_from) {
  switch (name_from) {
    case ax::mojom::NameFrom::kNone:
      return "none";
    case ax::mojom::NameFrom::kUninitialized:
      return "uninitialized";
    case ax::mojom::NameFrom::kAttribute:
      return "attribute";
    case ax::mojom::NameFrom::kAttributeExplicitlyEmpty:
      return "attributeExplicitlyEmpty";
    case ax::mojom::NameFrom::kCaption:
      return "caption";
    case ax::mojom::NameFrom::kContents:
      return "contents";
    case ax::mojom::NameFrom::kPlaceholder:
      return "placeholder";
    case ax::mojom::NameFrom::kRelatedElement:
      return "relatedElement";
    case ax::mojom::NameFrom::kTitle:
      return "title";
    case ax::mojom::NameFrom::kValue:
      return "value";
  }

  return "";
}

ax::mojom::NameFrom ParseNameFrom(const char* name_from) {
  if (0 == strcmp(name_from, "none"))
    return ax::mojom::NameFrom::kNone;
  if (0 == strcmp(name_from, "uninitialized"))
    return ax::mojom::NameFrom::kUninitialized;
  if (0 == strcmp(name_from, "attribute"))
    return ax::mojom::NameFrom::kAttribute;
  if (0 == strcmp(name_from, "attributeExplicitlyEmpty"))
    return ax::mojom::NameFrom::kAttributeExplicitlyEmpty;
  if (0 == strcmp(name_from, "caption"))
    return ax::mojom::NameFrom::kCaption;
  if (0 == strcmp(name_from, "contents"))
    return ax::mojom::NameFrom::kContents;
  if (0 == strcmp(name_from, "placeholder"))
    return ax::mojom::NameFrom::kPlaceholder;
  if (0 == strcmp(name_from, "relatedElement"))
    return ax::mojom::NameFrom::kRelatedElement;
  if (0 == strcmp(name_from, "title"))
    return ax::mojom::NameFrom::kTitle;
  if (0 == strcmp(name_from, "value"))
    return ax::mojom::NameFrom::kValue;
  return ax::mojom::NameFrom::kNone;
}

const char* ToString(ax::mojom::DescriptionFrom description_from) {
  switch (description_from) {
    case ax::mojom::DescriptionFrom::kNone:
      return "none";
    case ax::mojom::DescriptionFrom::kUninitialized:
      return "uninitialized";
    case ax::mojom::DescriptionFrom::kAttribute:
      return "attribute";
    case ax::mojom::DescriptionFrom::kContents:
      return "contents";
    case ax::mojom::DescriptionFrom::kRelatedElement:
      return "relatedElement";
    case ax::mojom::DescriptionFrom::kTitle:
      return "title";
  }

  return "";
}

ax::mojom::DescriptionFrom ParseDescriptionFrom(const char* description_from) {
  if (0 == strcmp(description_from, "none"))
    return ax::mojom::DescriptionFrom::kNone;
  if (0 == strcmp(description_from, "uninitialized"))
    return ax::mojom::DescriptionFrom::kUninitialized;
  if (0 == strcmp(description_from, "attribute"))
    return ax::mojom::DescriptionFrom::kAttribute;
  if (0 == strcmp(description_from, "contents"))
    return ax::mojom::DescriptionFrom::kContents;
  if (0 == strcmp(description_from, "relatedElement"))
    return ax::mojom::DescriptionFrom::kRelatedElement;
  if (0 == strcmp(description_from, "title"))
    return ax::mojom::DescriptionFrom::kTitle;
  return ax::mojom::DescriptionFrom::kNone;
}

const char* ToString(ax::mojom::EventFrom event_from) {
  switch (event_from) {
    case ax::mojom::EventFrom::kNone:
      return "none";
    case ax::mojom::EventFrom::kUser:
      return "user";
    case ax::mojom::EventFrom::kPage:
      return "page";
    case ax::mojom::EventFrom::kAction:
      return "action";
  }

  return "";
}

ax::mojom::EventFrom ParseEventFrom(const char* event_from) {
  if (0 == strcmp(event_from, "none"))
    return ax::mojom::EventFrom::kNone;
  if (0 == strcmp(event_from, "user"))
    return ax::mojom::EventFrom::kUser;
  if (0 == strcmp(event_from, "page"))
    return ax::mojom::EventFrom::kPage;
  if (0 == strcmp(event_from, "action"))
    return ax::mojom::EventFrom::kAction;
  return ax::mojom::EventFrom::kNone;
}

const char* ToString(ax::mojom::Gesture gesture) {
  switch (gesture) {
    case ax::mojom::Gesture::kNone:
      return "none";
    case ax::mojom::Gesture::kClick:
      return "click";
    case ax::mojom::Gesture::kSwipeLeft1:
      return "swipeLeft1";
    case ax::mojom::Gesture::kSwipeUp1:
      return "swipeUp1";
    case ax::mojom::Gesture::kSwipeRight1:
      return "swipeRight1";
    case ax::mojom::Gesture::kSwipeDown1:
      return "swipeDown1";
    case ax::mojom::Gesture::kSwipeLeft2:
      return "swipeLeft2";
    case ax::mojom::Gesture::kSwipeUp2:
      return "swipeUp2";
    case ax::mojom::Gesture::kSwipeRight2:
      return "swipeRight2";
    case ax::mojom::Gesture::kSwipeDown2:
      return "swipeDown2";
    case ax::mojom::Gesture::kSwipeLeft3:
      return "swipeLeft3";
    case ax::mojom::Gesture::kSwipeUp3:
      return "swipeUp3";
    case ax::mojom::Gesture::kSwipeRight3:
      return "swipeRight3";
    case ax::mojom::Gesture::kSwipeDown3:
      return "swipeDown3";
    case ax::mojom::Gesture::kSwipeLeft4:
      return "swipeLeft4";
    case ax::mojom::Gesture::kSwipeUp4:
      return "swipeUp4";
    case ax::mojom::Gesture::kSwipeRight4:
      return "swipeRight4";
    case ax::mojom::Gesture::kSwipeDown4:
      return "swipeDown4";
    case ax::mojom::Gesture::kTap2:
      return "tap2";
    case ax::mojom::Gesture::kTap3:
      return "tap3";
    case ax::mojom::Gesture::kTap4:
      return "tap4";
    case ax::mojom::Gesture::kTouchExplore:
      return "touchExplore";
  }

  return "";
}

ax::mojom::Gesture ParseGesture(const char* gesture) {
  if (0 == strcmp(gesture, "none"))
    return ax::mojom::Gesture::kNone;
  if (0 == strcmp(gesture, "click"))
    return ax::mojom::Gesture::kClick;
  if (0 == strcmp(gesture, "swipeLeft1"))
    return ax::mojom::Gesture::kSwipeLeft1;
  if (0 == strcmp(gesture, "swipeUp1"))
    return ax::mojom::Gesture::kSwipeUp1;
  if (0 == strcmp(gesture, "swipeRight1"))
    return ax::mojom::Gesture::kSwipeRight1;
  if (0 == strcmp(gesture, "swipeDown1"))
    return ax::mojom::Gesture::kSwipeDown1;
  if (0 == strcmp(gesture, "swipeLeft2"))
    return ax::mojom::Gesture::kSwipeLeft2;
  if (0 == strcmp(gesture, "swipeUp2"))
    return ax::mojom::Gesture::kSwipeUp2;
  if (0 == strcmp(gesture, "swipeRight2"))
    return ax::mojom::Gesture::kSwipeRight2;
  if (0 == strcmp(gesture, "swipeDown2"))
    return ax::mojom::Gesture::kSwipeDown2;
  if (0 == strcmp(gesture, "swipeLeft3"))
    return ax::mojom::Gesture::kSwipeLeft3;
  if (0 == strcmp(gesture, "swipeUp3"))
    return ax::mojom::Gesture::kSwipeUp3;
  if (0 == strcmp(gesture, "swipeRight3"))
    return ax::mojom::Gesture::kSwipeRight3;
  if (0 == strcmp(gesture, "swipeDown3"))
    return ax::mojom::Gesture::kSwipeDown3;
  if (0 == strcmp(gesture, "swipeLeft4"))
    return ax::mojom::Gesture::kSwipeLeft4;
  if (0 == strcmp(gesture, "swipeUp4"))
    return ax::mojom::Gesture::kSwipeUp4;
  if (0 == strcmp(gesture, "swipeRight4"))
    return ax::mojom::Gesture::kSwipeRight4;
  if (0 == strcmp(gesture, "swipeDown4"))
    return ax::mojom::Gesture::kSwipeDown4;
  if (0 == strcmp(gesture, "tap2"))
    return ax::mojom::Gesture::kTap2;
  if (0 == strcmp(gesture, "tap3"))
    return ax::mojom::Gesture::kTap3;
  if (0 == strcmp(gesture, "tap4"))
    return ax::mojom::Gesture::kTap4;
  if (0 == strcmp(gesture, "touchExplore"))
    return ax::mojom::Gesture::kTouchExplore;
  return ax::mojom::Gesture::kNone;
}

const char* ToString(ax::mojom::TextAffinity text_affinity) {
  switch (text_affinity) {
    case ax::mojom::TextAffinity::kNone:
      return "none";
    case ax::mojom::TextAffinity::kDownstream:
      return "downstream";
    case ax::mojom::TextAffinity::kUpstream:
      return "upstream";
  }

  return "";
}

ax::mojom::TextAffinity ParseTextAffinity(const char* text_affinity) {
  if (0 == strcmp(text_affinity, "none"))
    return ax::mojom::TextAffinity::kNone;
  if (0 == strcmp(text_affinity, "downstream"))
    return ax::mojom::TextAffinity::kDownstream;
  if (0 == strcmp(text_affinity, "upstream"))
    return ax::mojom::TextAffinity::kUpstream;
  return ax::mojom::TextAffinity::kNone;
}

const char* ToString(ax::mojom::TreeOrder tree_order) {
  switch (tree_order) {
    case ax::mojom::TreeOrder::kNone:
      return "none";
    case ax::mojom::TreeOrder::kUndefined:
      return "undefined";
    case ax::mojom::TreeOrder::kBefore:
      return "before";
    case ax::mojom::TreeOrder::kEqual:
      return "equal";
    case ax::mojom::TreeOrder::kAfter:
      return "after";
  }

  return "";
}

ax::mojom::TreeOrder ParseTreeOrder(const char* tree_order) {
  if (0 == strcmp(tree_order, "none"))
    return ax::mojom::TreeOrder::kNone;
  if (0 == strcmp(tree_order, "undefined"))
    return ax::mojom::TreeOrder::kUndefined;
  if (0 == strcmp(tree_order, "before"))
    return ax::mojom::TreeOrder::kBefore;
  if (0 == strcmp(tree_order, "equal"))
    return ax::mojom::TreeOrder::kEqual;
  if (0 == strcmp(tree_order, "after"))
    return ax::mojom::TreeOrder::kAfter;
  return ax::mojom::TreeOrder::kNone;
}

const char* ToString(ax::mojom::ImageAnnotationStatus status) {
  switch (status) {
    case ax::mojom::ImageAnnotationStatus::kNone:
      return "none";
    case ax::mojom::ImageAnnotationStatus::kWillNotAnnotateDueToScheme:
      return "kWillNotAnnotateDueToScheme";
    case ax::mojom::ImageAnnotationStatus::kIneligibleForAnnotation:
      return "ineligibleForAnnotation";
    case ax::mojom::ImageAnnotationStatus::kEligibleForAnnotation:
      return "eligibleForAnnotation";
    case ax::mojom::ImageAnnotationStatus::kSilentlyEligibleForAnnotation:
      return "silentlyEligibleForAnnotation";
    case ax::mojom::ImageAnnotationStatus::kAnnotationPending:
      return "annotationPending";
    case ax::mojom::ImageAnnotationStatus::kAnnotationSucceeded:
      return "annotationSucceeded";
    case ax::mojom::ImageAnnotationStatus::kAnnotationEmpty:
      return "annotationEmpty";
    case ax::mojom::ImageAnnotationStatus::kAnnotationAdult:
      return "annotationAdult";
    case ax::mojom::ImageAnnotationStatus::kAnnotationProcessFailed:
      return "annotationProcessFailed";
  }

  return "";
}

ax::mojom::ImageAnnotationStatus ParseImageAnnotationStatus(
    const char* status) {
  if (0 == strcmp(status, "none"))
    return ax::mojom::ImageAnnotationStatus::kNone;
  if (0 == strcmp(status, "kWillNotAnnotateDueToScheme"))
    return ax::mojom::ImageAnnotationStatus::kWillNotAnnotateDueToScheme;
  if (0 == strcmp(status, "ineligibleForAnnotation"))
    return ax::mojom::ImageAnnotationStatus::kIneligibleForAnnotation;
  if (0 == strcmp(status, "eligibleForAnnotation"))
    return ax::mojom::ImageAnnotationStatus::kEligibleForAnnotation;
  if (0 == strcmp(status, "silentlyEligibleForAnnotation"))
    return ax::mojom::ImageAnnotationStatus::kSilentlyEligibleForAnnotation;
  if (0 == strcmp(status, "annotationPending"))
    return ax::mojom::ImageAnnotationStatus::kAnnotationPending;
  if (0 == strcmp(status, "annotationSucceeded"))
    return ax::mojom::ImageAnnotationStatus::kAnnotationSucceeded;
  if (0 == strcmp(status, "annotationEmpty"))
    return ax::mojom::ImageAnnotationStatus::kAnnotationEmpty;
  if (0 == strcmp(status, "annotationAdult"))
    return ax::mojom::ImageAnnotationStatus::kAnnotationAdult;
  if (0 == strcmp(status, "annotationProcessFailed"))
    return ax::mojom::ImageAnnotationStatus::kAnnotationProcessFailed;

  return ax::mojom::ImageAnnotationStatus::kNone;
}

const char* ToString(ax::mojom::Dropeffect dropeffect) {
  switch (dropeffect) {
    case ax::mojom::Dropeffect::kCopy:
      return "copy";
    case ax::mojom::Dropeffect::kExecute:
      return "execute";
    case ax::mojom::Dropeffect::kLink:
      return "link";
    case ax::mojom::Dropeffect::kMove:
      return "move";
    case ax::mojom::Dropeffect::kPopup:
      return "popup";
    case ax::mojom::Dropeffect::kNone:
      return "none";
  }

  return "";
}

ax::mojom::Dropeffect ParseDropeffect(const char* dropeffect) {
  if (0 == strcmp(dropeffect, "copy"))
    return ax::mojom::Dropeffect::kCopy;
  if (0 == strcmp(dropeffect, "execute"))
    return ax::mojom::Dropeffect::kExecute;
  if (0 == strcmp(dropeffect, "link"))
    return ax::mojom::Dropeffect::kLink;
  if (0 == strcmp(dropeffect, "move"))
    return ax::mojom::Dropeffect::kMove;
  if (0 == strcmp(dropeffect, "popup"))
    return ax::mojom::Dropeffect::kPopup;
  return ax::mojom::Dropeffect::kNone;
}

}  // namespace ui
