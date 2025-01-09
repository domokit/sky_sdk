// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Class name constants which correspond to the class names used by the
/// Android accessibility bridge.
class AndroidClassName {
  /// The class name used for checkboxes.
  static const String checkBox = 'android.widget.CheckBox';

  /// The default className if none is provided by flutter.
  static const String view = 'android.view.View';

  /// The class name used for radio buttons.
  static const String radio = 'android.widget.RadioButton';

  /// The class name used for editable text fields.
  static const String editText = 'android.widget.EditText';

  /// The class name used for read only text fields.
  static const String textView = 'android.widget.TextView';

  /// The class name used for toggle switches.
  static const String toggleSwitch = 'android.widget.Switch';

  /// The default className for buttons.
  static const String button = 'android.widget.Button';
}

/// Action constants which correspond to `AccessibilityAction` in Android.
enum AndroidSemanticsAction {
  /// Matches `AccessibilityAction.ACTION_FOCUS`.
  focus(_kFocusIndex),

  /// Matches `AccessibilityAction.ACTION_CLEAR_FOCUS`.
  clearFocus(_kClearFocusIndex),

  /// Matches `AccessibilityAction.ACTION_SELECT`.
  select(_kSelectIndex),

  /// Matches `AccessibilityAction.ACTION_CLEAR_SELECTION`.
  clearSelection(_kClearSelectionIndex),

  /// Matches `AccessibilityAction.ACTION_CLICK`.
  click(_kClickIndex),

  /// Matches `AccessibilityAction.ACTION_LONG_CLICK`.
  longClick(_kLongClickIndex),

  /// Matches `AccessibilityAction.ACTION_ACCESSIBILITY_FOCUS`.
  accessibilityFocus(_kAccessibilityFocusIndex),

  /// Matches `AccessibilityAction.ACTION_CLEAR_ACCESSIBILITY_FOCUS`.
  clearAccessibilityFocus(_kClearAccessibilityFocusIndex),

  /// Matches `AccessibilityAction.ACTION_NEXT_AT_MOVEMENT_GRANULARITY`.
  nextAtMovementGranularity(_kNextAtMovementGranularityIndex),

  /// Matches `AccessibilityAction.ACTION_PREVIOUS_AT_MOVEMENT_GRANULARITY`.
  previousAtMovementGranularity(_kPreviousAtMovementGranularityIndex),

  /// Matches `AccessibilityAction.ACTION_NEXT_HTML_ELEMENT`.
  nextHtmlElement(_kNextHtmlElementIndex),

  /// Matches `AccessibilityAction.ACTION_PREVIOUS_HTML_ELEMENT`.
  previousHtmlElement(_kPreviousHtmlElementIndex),

  /// Matches `AccessibilityAction.ACTION_SCROLL_FORWARD`.
  scrollForward(_kScrollForwardIndex),

  /// Matches `AccessibilityAction.ACTION_SCROLL_BACKWARD`.
  scrollBackward(_kScrollBackwardIndex),

  /// Matches `AccessibilityAction.ACTION_CUT`.
  cut(_kCutIndex),

  /// Matches `AccessibilityAction.ACTION_COPY`.
  copy(_kCopyIndex),

  /// Matches `AccessibilityAction.ACTION_PASTE`.
  paste(_kPasteIndex),

  /// Matches `AccessibilityAction.ACTION_SET_SELECTION`.
  setSelection(_kSetSelectionIndex),

  /// Matches `AccessibilityAction.ACTION_EXPAND`.
  expand(_kExpandIndex),

  /// Matches `AccessibilityAction.ACTION_COLLAPSE`.
  collapse(_kCollapseIndex),

  /// Matches `AccessibilityAction.SET_TEXT`.
  setText(_kSetText);

  const AndroidSemanticsAction(this.id);

  /// The Android id of the action.
  final int id;

  // These indices need to be in sync with android_semantics_testing/android/app/src/main/java/com/yourcompany/platforminteraction/MainActivity.java
  static const int _kFocusIndex = 1 << 0;
  static const int _kClearFocusIndex = 1 << 1;
  static const int _kSelectIndex = 1 << 2;
  static const int _kClearSelectionIndex = 1 << 3;
  static const int _kClickIndex = 1 << 4;
  static const int _kLongClickIndex = 1 << 5;
  static const int _kAccessibilityFocusIndex = 1 << 6;
  static const int _kClearAccessibilityFocusIndex = 1 << 7;
  static const int _kNextAtMovementGranularityIndex = 1 << 8;
  static const int _kPreviousAtMovementGranularityIndex = 1 << 9;
  static const int _kNextHtmlElementIndex = 1 << 10;
  static const int _kPreviousHtmlElementIndex = 1 << 11;
  static const int _kScrollForwardIndex = 1 << 12;
  static const int _kScrollBackwardIndex = 1 << 13;
  static const int _kCutIndex = 1 << 14;
  static const int _kCopyIndex = 1 << 15;
  static const int _kPasteIndex = 1 << 16;
  static const int _kSetSelectionIndex = 1 << 17;
  static const int _kExpandIndex = 1 << 18;
  static const int _kCollapseIndex = 1 << 19;
  static const int _kSetText = 1 << 21;

  static const Map<int, AndroidSemanticsAction> _kActionById = <int, AndroidSemanticsAction>{
    _kFocusIndex: focus,
    _kClearFocusIndex: clearFocus,
    _kSelectIndex: select,
    _kClearSelectionIndex: clearSelection,
    _kClickIndex: click,
    _kLongClickIndex: longClick,
    _kAccessibilityFocusIndex: accessibilityFocus,
    _kClearAccessibilityFocusIndex: clearAccessibilityFocus,
    _kNextAtMovementGranularityIndex: nextAtMovementGranularity,
    _kPreviousAtMovementGranularityIndex: previousAtMovementGranularity,
    _kNextHtmlElementIndex: nextHtmlElement,
    _kPreviousHtmlElementIndex: previousHtmlElement,
    _kScrollForwardIndex: scrollForward,
    _kScrollBackwardIndex: scrollBackward,
    _kCutIndex: cut,
    _kCopyIndex: copy,
    _kPasteIndex: paste,
    _kSetSelectionIndex: setSelection,
    _kExpandIndex: expand,
    _kCollapseIndex: collapse,
    _kSetText: setText,
  };

  /// Creates a new [AndroidSemanticsAction] from an integer `value`.
  ///
  /// Returns `null` if the id is not a known Android accessibility action.
  static AndroidSemanticsAction? deserialize(int value) {
    return _kActionById[value];
  }
}
