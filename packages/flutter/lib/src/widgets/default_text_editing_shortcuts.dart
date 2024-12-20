// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'focus_traversal.dart';
import 'framework.dart';
import 'scrollable_helpers.dart';
import 'shortcuts.dart';
import 'text_editing_intents.dart';

/// A widget with the shortcuts used for the default text editing behavior.
///
/// This default behavior can be overridden by placing a [Shortcuts] widget
/// lower in the widget tree than this. See the [Action] class for an example
/// of remapping an [Intent] to a custom [Action].
///
/// The [Shortcuts] widget usually takes precedence over system keybindings.
/// Proceed with caution if the shortcut you wish to override is also used by
/// the system. For example, overriding [LogicalKeyboardKey.backspace] could
/// cause CJK input methods to discard more text than they should when the
/// backspace key is pressed during text composition on iOS.
///
/// {@macro flutter.widgets.editableText.shortcutsAndTextInput}
///
/// {@tool snippet}
///
/// This example shows how to use an additional [Shortcuts] widget to override
/// some default text editing keyboard shortcuts to have new behavior. Instead
/// of moving the cursor, alt + up/down will change the focused widget.
///
/// ```dart
/// @override
/// Widget build(BuildContext context) {
///   // If using WidgetsApp or its descendants MaterialApp or CupertinoApp,
///   // then DefaultTextEditingShortcuts is already being inserted into the
///   // widget tree.
///   return const DefaultTextEditingShortcuts(
///     child: Center(
///       child: Shortcuts(
///         shortcuts: <ShortcutActivator, Intent>{
///           SingleActivator(LogicalKeyboardKey.arrowDown, alt: true): NextFocusIntent(),
///           SingleActivator(LogicalKeyboardKey.arrowUp, alt: true): PreviousFocusIntent(),
///         },
///         child: Column(
///           children: <Widget>[
///             TextField(
///               decoration: InputDecoration(
///                 hintText: 'alt + down moves to the next field.',
///               ),
///             ),
///             TextField(
///               decoration: InputDecoration(
///                 hintText: 'And alt + up moves to the previous.',
///               ),
///             ),
///           ],
///         ),
///       ),
///     ),
///   );
/// }
/// ```
/// {@end-tool}
///
/// {@tool snippet}
///
/// This example shows how to use an additional [Shortcuts] widget to override
/// default text editing shortcuts to have completely custom behavior defined by
/// a custom Intent and Action. Here, the up/down arrow keys increment/decrement
/// a counter instead of moving the cursor.
///
/// ```dart
/// class IncrementCounterIntent extends Intent {}
/// class DecrementCounterIntent extends Intent {}
///
/// class MyWidget extends StatefulWidget {
///   const MyWidget({ super.key });
///
///   @override
///   MyWidgetState createState() => MyWidgetState();
/// }
///
/// class MyWidgetState extends State<MyWidget> {
///
///   int _counter = 0;
///
///   @override
///   Widget build(BuildContext context) {
///     // If using WidgetsApp or its descendants MaterialApp or CupertinoApp,
///     // then DefaultTextEditingShortcuts is already being inserted into the
///     // widget tree.
///     return DefaultTextEditingShortcuts(
///       child: Center(
///         child: Column(
///           mainAxisAlignment: MainAxisAlignment.center,
///           children: <Widget>[
///             const Text(
///               'You have pushed the button this many times:',
///             ),
///             Text(
///               '$_counter',
///               style: Theme.of(context).textTheme.headlineMedium,
///             ),
///             Shortcuts(
///               shortcuts: <ShortcutActivator, Intent>{
///                 const SingleActivator(LogicalKeyboardKey.arrowUp): IncrementCounterIntent(),
///                 const SingleActivator(LogicalKeyboardKey.arrowDown): DecrementCounterIntent(),
///               },
///               child: Actions(
///                 actions: <Type, Action<Intent>>{
///                   IncrementCounterIntent: CallbackAction<IncrementCounterIntent>(
///                     onInvoke: (IncrementCounterIntent intent) {
///                       setState(() {
///                         _counter++;
///                       });
///                       return null;
///                     },
///                   ),
///                   DecrementCounterIntent: CallbackAction<DecrementCounterIntent>(
///                     onInvoke: (DecrementCounterIntent intent) {
///                       setState(() {
///                         _counter--;
///                       });
///                       return null;
///                     },
///                   ),
///                 },
///                 child: const TextField(
///                   maxLines: 2,
///                   decoration: InputDecoration(
///                     hintText: 'Up/down increment/decrement here.',
///                   ),
///                 ),
///               ),
///             ),
///             const TextField(
///               maxLines: 2,
///               decoration: InputDecoration(
///                 hintText: 'Up/down behave normally here.',
///               ),
///             ),
///           ],
///         ),
///       ),
///     );
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///   * [WidgetsApp], which creates a DefaultTextEditingShortcuts.
class DefaultTextEditingShortcuts extends StatelessWidget {
  /// Creates a [DefaultTextEditingShortcuts] widget that provides the default text editing
  /// shortcuts on the current platform.
  const DefaultTextEditingShortcuts({super.key, required this.child});

  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  // These shortcuts are shared between all platforms except Apple platforms,
  // because they use different modifier keys as the line/word modifier.
  static final Map<ShortcutActivator, Intent> _commonShortcuts = <ShortcutActivator, Intent>{
    // Delete Shortcuts.
    for (final bool pressShift in const <bool>[true, false]) ...<SingleActivator, Intent>{
      SingleActivator(LogicalKeyboardKey.backspace, shift: pressShift): const DeleteCharacterIntent(
        forward: false,
      ),
      SingleActivator(
        LogicalKeyboardKey.backspace,
        control: true,
        shift: pressShift,
      ): const DeleteToNextWordBoundaryIntent(forward: false),
      SingleActivator(
        LogicalKeyboardKey.backspace,
        alt: true,
        shift: pressShift,
      ): const DeleteToLineBreakIntent(forward: false),
      SingleActivator(LogicalKeyboardKey.delete, shift: pressShift): const DeleteCharacterIntent(
        forward: true,
      ),
      SingleActivator(
        LogicalKeyboardKey.delete,
        control: true,
        shift: pressShift,
      ): const DeleteToNextWordBoundaryIntent(forward: true),
      SingleActivator(
        LogicalKeyboardKey.delete,
        alt: true,
        shift: pressShift,
      ): const DeleteToLineBreakIntent(forward: true),
    },

    // Arrow: Move selection.
    const SingleActivator(LogicalKeyboardKey.arrowLeft): const ExtendSelectionByCharacterIntent(
      forward: false,
      collapseSelection: true,
    ),
    const SingleActivator(LogicalKeyboardKey.arrowRight): const ExtendSelectionByCharacterIntent(
      forward: true,
      collapseSelection: true,
    ),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: true),

    // Shift + Arrow: Extend selection.
    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      shift: true,
    ): const ExtendSelectionByCharacterIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      shift: true,
    ): const ExtendSelectionByCharacterIntent(forward: true, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: false,
      collapseSelection: false,
    ),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      alt: true,
    ): const ExtendSelectionToLineBreakIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      alt: true,
    ): const ExtendSelectionToLineBreakIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      alt: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      alt: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      shift: true,
      alt: true,
    ): const ExtendSelectionToLineBreakIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      shift: true,
      alt: true,
    ): const ExtendSelectionToLineBreakIntent(forward: true, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      shift: true,
      alt: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      shift: true,
      alt: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      control: true,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      control: true,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      shift: true,
      control: true,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      shift: true,
      control: true,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      shift: true,
      control: true,
    ): const ExtendSelectionToNextParagraphBoundaryIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      shift: true,
      control: true,
    ): const ExtendSelectionToNextParagraphBoundaryIntent(forward: true, collapseSelection: false),

    // Page Up / Down: Move selection by page.
    const SingleActivator(
      LogicalKeyboardKey.pageUp,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.pageDown,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: true, collapseSelection: true),

    // Shift + Page Up / Down: Extend selection by page.
    const SingleActivator(
      LogicalKeyboardKey.pageUp,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(
      forward: false,
      collapseSelection: false,
    ),
    const SingleActivator(
      LogicalKeyboardKey.pageDown,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.keyX,
      control: true,
    ): const CopySelectionTextIntent.cut(SelectionChangedCause.keyboard),
    const SingleActivator(LogicalKeyboardKey.keyC, control: true): CopySelectionTextIntent.copy,
    const SingleActivator(LogicalKeyboardKey.keyV, control: true): const PasteTextIntent(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(LogicalKeyboardKey.keyA, control: true): const SelectAllTextIntent(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(LogicalKeyboardKey.keyZ, control: true): const UndoTextIntent(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(
      LogicalKeyboardKey.keyZ,
      shift: true,
      control: true,
    ): const RedoTextIntent(SelectionChangedCause.keyboard),
    // These keys should go to the IME when a field is focused, not to other
    // Shortcuts.
    const SingleActivator(LogicalKeyboardKey.space): const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.enter): const DoNothingAndStopPropagationTextIntent(),
  };

  // The following key combinations have no effect on text editing on this
  // platform:
  //   * End
  //   * Home
  //   * Meta + X
  //   * Meta + C
  //   * Meta + V
  //   * Meta + A
  //   * Meta + shift? + Z
  //   * Meta + shift? + arrow down
  //   * Meta + shift? + arrow left
  //   * Meta + shift? + arrow right
  //   * Meta + shift? + arrow up
  //   * Shift + end
  //   * Shift + home
  //   * Meta + shift? + delete
  //   * Meta + shift? + backspace
  static final Map<ShortcutActivator, Intent> _androidShortcuts = _commonShortcuts;

  static final Map<ShortcutActivator, Intent> _fuchsiaShortcuts = _androidShortcuts;

  static final Map<ShortcutActivator, Intent> _linuxNumpadShortcuts = <ShortcutActivator, Intent>{
    // When numLock is on, numpad keys shortcuts require shift to be pressed too.
    const SingleActivator(
      LogicalKeyboardKey.numpad6,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionByCharacterIntent(forward: true, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.numpad4,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionByCharacterIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.numpad8,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: false,
      collapseSelection: false,
    ),
    const SingleActivator(
      LogicalKeyboardKey.numpad2,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.numpad6,
      shift: true,
      control: true,
      numLock: LockState.locked,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: true, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.numpad4,
      shift: true,
      control: true,
      numLock: LockState.locked,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.numpad8,
      shift: true,
      control: true,
      numLock: LockState.locked,
    ): const ExtendSelectionToNextParagraphBoundaryIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.numpad2,
      shift: true,
      control: true,
      numLock: LockState.locked,
    ): const ExtendSelectionToNextParagraphBoundaryIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.numpad9,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(
      forward: false,
      collapseSelection: false,
    ),
    const SingleActivator(
      LogicalKeyboardKey.numpad3,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.numpad7,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: false,
      collapseSelection: false,
    ),
    const SingleActivator(
      LogicalKeyboardKey.numpad1,
      shift: true,
      numLock: LockState.locked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.numpadDecimal,
      shift: true,
      numLock: LockState.locked,
    ): const DeleteCharacterIntent(forward: true),
    const SingleActivator(
      LogicalKeyboardKey.numpadDecimal,
      shift: true,
      control: true,
      numLock: LockState.locked,
    ): const DeleteToNextWordBoundaryIntent(forward: true),

    // When numLock is off, numpad keys shortcuts require shift not to be pressed.
    const SingleActivator(
      LogicalKeyboardKey.numpad6,
      numLock: LockState.unlocked,
    ): const ExtendSelectionByCharacterIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad4,
      numLock: LockState.unlocked,
    ): const ExtendSelectionByCharacterIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad8,
      numLock: LockState.unlocked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad2,
      numLock: LockState.unlocked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.numpad6,
      control: true,
      numLock: LockState.unlocked,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad4,
      control: true,
      numLock: LockState.unlocked,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad8,
      control: true,
      numLock: LockState.unlocked,
    ): const ExtendSelectionToNextParagraphBoundaryIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad2,
      control: true,
      numLock: LockState.unlocked,
    ): const ExtendSelectionToNextParagraphBoundaryIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.numpad9,
      numLock: LockState.unlocked,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad3,
      numLock: LockState.unlocked,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.numpad7,
      numLock: LockState.unlocked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.numpad1,
      numLock: LockState.unlocked,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.numpadDecimal,
      numLock: LockState.unlocked,
    ): const DeleteCharacterIntent(forward: true),
    const SingleActivator(
      LogicalKeyboardKey.numpadDecimal,
      control: true,
      numLock: LockState.unlocked,
    ): const DeleteToNextWordBoundaryIntent(forward: true),
  };

  static final Map<ShortcutActivator, Intent> _linuxShortcuts = <ShortcutActivator, Intent>{
    ..._commonShortcuts,
    ..._linuxNumpadShortcuts,
    const SingleActivator(LogicalKeyboardKey.home): const ExtendSelectionToLineBreakIntent(
      forward: false,
      collapseSelection: true,
    ),
    const SingleActivator(LogicalKeyboardKey.end): const ExtendSelectionToLineBreakIntent(
      forward: true,
      collapseSelection: true,
    ),
    const SingleActivator(
      LogicalKeyboardKey.home,
      shift: true,
    ): const ExtendSelectionToLineBreakIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.end,
      shift: true,
    ): const ExtendSelectionToLineBreakIntent(forward: true, collapseSelection: false),
    // The following key combinations have no effect on text editing on this
    // platform:
    //   * Control + shift? + end
    //   * Control + shift? + home
    //   * Meta + X
    //   * Meta + C
    //   * Meta + V
    //   * Meta + A
    //   * Meta + shift? + Z
    //   * Meta + shift? + arrow down
    //   * Meta + shift? + arrow left
    //   * Meta + shift? + arrow right
    //   * Meta + shift? + arrow up
    //   * Meta + shift? + delete
    //   * Meta + shift? + backspace
  };

  // macOS document shortcuts: https://support.apple.com/en-us/HT201236.
  // The macOS shortcuts uses different word/line modifiers than most other
  // platforms.
  static final Map<ShortcutActivator, Intent> _macShortcuts = <ShortcutActivator, Intent>{
    for (final bool pressShift in const <bool>[true, false]) ...<SingleActivator, Intent>{
      SingleActivator(LogicalKeyboardKey.backspace, shift: pressShift): const DeleteCharacterIntent(
        forward: false,
      ),
      SingleActivator(
        LogicalKeyboardKey.backspace,
        alt: true,
        shift: pressShift,
      ): const DeleteToNextWordBoundaryIntent(forward: false),
      SingleActivator(
        LogicalKeyboardKey.backspace,
        meta: true,
        shift: pressShift,
      ): const DeleteToLineBreakIntent(forward: false),
      SingleActivator(LogicalKeyboardKey.delete, shift: pressShift): const DeleteCharacterIntent(
        forward: true,
      ),
      SingleActivator(
        LogicalKeyboardKey.delete,
        alt: true,
        shift: pressShift,
      ): const DeleteToNextWordBoundaryIntent(forward: true),
      SingleActivator(
        LogicalKeyboardKey.delete,
        meta: true,
        shift: pressShift,
      ): const DeleteToLineBreakIntent(forward: true),
    },

    const SingleActivator(LogicalKeyboardKey.arrowLeft): const ExtendSelectionByCharacterIntent(
      forward: false,
      collapseSelection: true,
    ),
    const SingleActivator(LogicalKeyboardKey.arrowRight): const ExtendSelectionByCharacterIntent(
      forward: true,
      collapseSelection: true,
    ),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: true),

    // Shift + Arrow: Extend selection.
    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      shift: true,
    ): const ExtendSelectionByCharacterIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      shift: true,
    ): const ExtendSelectionByCharacterIntent(forward: true, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: false,
      collapseSelection: false,
    ),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: false),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      alt: true,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      alt: true,
    ): const ExtendSelectionToNextWordBoundaryIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      alt: true,
    ): const ExtendSelectionToLineBreakIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      alt: true,
    ): const ExtendSelectionToLineBreakIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      shift: true,
      alt: true,
    ): const ExtendSelectionToNextWordBoundaryOrCaretLocationIntent(forward: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      shift: true,
      alt: true,
    ): const ExtendSelectionToNextWordBoundaryOrCaretLocationIntent(forward: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      shift: true,
      alt: true,
    ): const ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent(forward: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      shift: true,
      alt: true,
    ): const ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent(forward: true),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      meta: true,
    ): const ExtendSelectionToLineBreakIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      meta: true,
    ): const ExtendSelectionToLineBreakIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      meta: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      meta: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: true, collapseSelection: true),

    const SingleActivator(
      LogicalKeyboardKey.arrowLeft,
      shift: true,
      meta: true,
    ): const ExpandSelectionToLineBreakIntent(forward: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowRight,
      shift: true,
      meta: true,
    ): const ExpandSelectionToLineBreakIntent(forward: true),
    const SingleActivator(
      LogicalKeyboardKey.arrowUp,
      shift: true,
      meta: true,
    ): const ExpandSelectionToDocumentBoundaryIntent(forward: false),
    const SingleActivator(
      LogicalKeyboardKey.arrowDown,
      shift: true,
      meta: true,
    ): const ExpandSelectionToDocumentBoundaryIntent(forward: true),

    const SingleActivator(LogicalKeyboardKey.keyT, control: true):
        const TransposeCharactersIntent(),

    const SingleActivator(LogicalKeyboardKey.home): const ScrollToDocumentBoundaryIntent(
      forward: false,
    ),
    const SingleActivator(LogicalKeyboardKey.end): const ScrollToDocumentBoundaryIntent(
      forward: true,
    ),
    const SingleActivator(
      LogicalKeyboardKey.home,
      shift: true,
    ): const ExpandSelectionToDocumentBoundaryIntent(forward: false),
    const SingleActivator(
      LogicalKeyboardKey.end,
      shift: true,
    ): const ExpandSelectionToDocumentBoundaryIntent(forward: true),

    const SingleActivator(LogicalKeyboardKey.pageUp): const ScrollIntent(
      direction: AxisDirection.up,
      type: ScrollIncrementType.page,
    ),
    const SingleActivator(LogicalKeyboardKey.pageDown): const ScrollIntent(
      direction: AxisDirection.down,
      type: ScrollIncrementType.page,
    ),
    const SingleActivator(
      LogicalKeyboardKey.pageUp,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(
      forward: false,
      collapseSelection: false,
    ),
    const SingleActivator(
      LogicalKeyboardKey.pageDown,
      shift: true,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: true, collapseSelection: false),

    const SingleActivator(LogicalKeyboardKey.keyX, meta: true): const CopySelectionTextIntent.cut(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(LogicalKeyboardKey.keyC, meta: true): CopySelectionTextIntent.copy,
    const SingleActivator(LogicalKeyboardKey.keyV, meta: true): const PasteTextIntent(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(LogicalKeyboardKey.keyA, meta: true): const SelectAllTextIntent(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(LogicalKeyboardKey.keyZ, meta: true): const UndoTextIntent(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(LogicalKeyboardKey.keyZ, shift: true, meta: true): const RedoTextIntent(
      SelectionChangedCause.keyboard,
    ),
    const SingleActivator(
      LogicalKeyboardKey.keyE,
      control: true,
    ): const ExtendSelectionToLineBreakIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.keyA,
      control: true,
    ): const ExtendSelectionToLineBreakIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.keyF,
      control: true,
    ): const ExtendSelectionByCharacterIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.keyB,
      control: true,
    ): const ExtendSelectionByCharacterIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.keyN,
      control: true,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.keyP,
      control: true,
    ): const ExtendSelectionVerticallyToAdjacentLineIntent(forward: false, collapseSelection: true),
    // These keys should go to the IME when a field is focused, not to other
    // Shortcuts.
    const SingleActivator(LogicalKeyboardKey.space): const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.enter): const DoNothingAndStopPropagationTextIntent(),
    // The following key combinations have no effect on text editing on this
    // platform:
    //   * End
    //   * Home
    //   * Control + shift? + end
    //   * Control + shift? + home
    //   * Control + shift? + Z
  };

  // There is no complete documentation of iOS shortcuts: use macOS ones.
  static final Map<ShortcutActivator, Intent> _iOSShortcuts = _macShortcuts;

  // The following key combinations have no effect on text editing on this
  // platform:
  //   * Meta + X
  //   * Meta + C
  //   * Meta + V
  //   * Meta + A
  //   * Meta + shift? + arrow down
  //   * Meta + shift? + arrow left
  //   * Meta + shift? + arrow right
  //   * Meta + shift? + arrow up
  //   * Meta + delete
  //   * Meta + backspace
  static final Map<ShortcutActivator, Intent> _windowsShortcuts = <ShortcutActivator, Intent>{
    ..._commonShortcuts,
    const SingleActivator(
      LogicalKeyboardKey.pageUp,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.pageDown,
    ): const ExtendSelectionVerticallyToAdjacentPageIntent(forward: true, collapseSelection: true),
    const SingleActivator(LogicalKeyboardKey.home): const ExtendSelectionToLineBreakIntent(
      forward: false,
      collapseSelection: true,
      continuesAtWrap: true,
    ),
    const SingleActivator(LogicalKeyboardKey.end): const ExtendSelectionToLineBreakIntent(
      forward: true,
      collapseSelection: true,
      continuesAtWrap: true,
    ),
    const SingleActivator(
      LogicalKeyboardKey.home,
      shift: true,
    ): const ExtendSelectionToLineBreakIntent(
      forward: false,
      collapseSelection: false,
      continuesAtWrap: true,
    ),
    const SingleActivator(
      LogicalKeyboardKey.end,
      shift: true,
    ): const ExtendSelectionToLineBreakIntent(
      forward: true,
      collapseSelection: false,
      continuesAtWrap: true,
    ),
    const SingleActivator(
      LogicalKeyboardKey.home,
      control: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: false, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.end,
      control: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: true, collapseSelection: true),
    const SingleActivator(
      LogicalKeyboardKey.home,
      shift: true,
      control: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: false, collapseSelection: false),
    const SingleActivator(
      LogicalKeyboardKey.end,
      shift: true,
      control: true,
    ): const ExtendSelectionToDocumentBoundaryIntent(forward: true, collapseSelection: false),
  };

  // Web handles its text selection natively and doesn't use any of these
  // shortcuts in Flutter.
  static final Map<ShortcutActivator, Intent> _webDisablingTextShortcuts =
      <ShortcutActivator, Intent>{
        for (final bool pressShift in const <bool>[true, false]) ...<SingleActivator, Intent>{
          SingleActivator(LogicalKeyboardKey.backspace, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
          SingleActivator(LogicalKeyboardKey.delete, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
          SingleActivator(LogicalKeyboardKey.backspace, alt: true, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
          SingleActivator(LogicalKeyboardKey.delete, alt: true, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
          SingleActivator(LogicalKeyboardKey.backspace, control: true, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
          SingleActivator(LogicalKeyboardKey.delete, control: true, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
          SingleActivator(LogicalKeyboardKey.backspace, meta: true, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
          SingleActivator(LogicalKeyboardKey.delete, meta: true, shift: pressShift):
              const DoNothingAndStopPropagationTextIntent(),
        },
        ..._commonDisablingTextShortcuts,
        const SingleActivator(LogicalKeyboardKey.keyX, control: true):
            const DoNothingAndStopPropagationTextIntent(),
        const SingleActivator(LogicalKeyboardKey.keyX, meta: true):
            const DoNothingAndStopPropagationTextIntent(),
        const SingleActivator(LogicalKeyboardKey.keyC, control: true):
            const DoNothingAndStopPropagationTextIntent(),
        const SingleActivator(LogicalKeyboardKey.keyC, meta: true):
            const DoNothingAndStopPropagationTextIntent(),
        const SingleActivator(LogicalKeyboardKey.keyV, control: true):
            const DoNothingAndStopPropagationTextIntent(),
        const SingleActivator(LogicalKeyboardKey.keyV, meta: true):
            const DoNothingAndStopPropagationTextIntent(),
        const SingleActivator(LogicalKeyboardKey.keyA, control: true):
            const DoNothingAndStopPropagationTextIntent(),
        const SingleActivator(LogicalKeyboardKey.keyA, meta: true):
            const DoNothingAndStopPropagationTextIntent(),
      };

  static const Map<ShortcutActivator, Intent> _commonDisablingTextShortcuts =
      <ShortcutActivator, Intent>{
        SingleActivator(LogicalKeyboardKey.arrowDown, alt: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowLeft, alt: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowRight, alt: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowUp, alt: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowDown, meta: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowLeft, meta: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowRight, meta: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowUp, meta: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowDown): DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowLeft): DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowRight): DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowUp): DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowLeft, control: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowRight, control: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true, control: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, control: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.space): DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.enter): DoNothingAndStopPropagationTextIntent(),
      };

  static final Map<ShortcutActivator, Intent>
  _macDisablingTextShortcuts = <ShortcutActivator, Intent>{
    ..._commonDisablingTextShortcuts,
    ..._iOSDisablingTextShortcuts,
    const SingleActivator(LogicalKeyboardKey.escape): const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.tab): const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.tab, shift: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowDown, shift: true, alt: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowUp, shift: true, alt: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true, alt: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, alt: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowLeft, shift: true, meta: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.arrowRight, shift: true, meta: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.pageUp): const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.pageDown):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.end): const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.home): const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.pageUp, shift: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.pageDown, shift: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.end, shift: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.home, shift: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.end, control: true):
        const DoNothingAndStopPropagationTextIntent(),
    const SingleActivator(LogicalKeyboardKey.home, control: true):
        const DoNothingAndStopPropagationTextIntent(),
  };

  // Hand backspace/delete events that do not depend on text layout (delete
  // character and delete to the next word) back to the IME to allow it to
  // update composing text properly.
  static const Map<ShortcutActivator, Intent> _iOSDisablingTextShortcuts =
      <ShortcutActivator, Intent>{
        SingleActivator(LogicalKeyboardKey.backspace): DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.backspace, shift: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.delete): DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.delete, shift: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.backspace, alt: true, shift: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.backspace, alt: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.delete, alt: true, shift: true):
            DoNothingAndStopPropagationTextIntent(),
        SingleActivator(LogicalKeyboardKey.delete, alt: true):
            DoNothingAndStopPropagationTextIntent(),
      };

  static Map<ShortcutActivator, Intent> get _shortcuts {
    return switch (defaultTargetPlatform) {
      TargetPlatform.android => _androidShortcuts,
      TargetPlatform.fuchsia => _fuchsiaShortcuts,
      TargetPlatform.iOS => _iOSShortcuts,
      TargetPlatform.linux => _linuxShortcuts,
      TargetPlatform.macOS => _macShortcuts,
      TargetPlatform.windows => _windowsShortcuts,
    };
  }

  Map<ShortcutActivator, Intent>? _getDisablingShortcut() {
    if (kIsWeb) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.linux:
          return <ShortcutActivator, Intent>{
            ..._webDisablingTextShortcuts,
            for (final ShortcutActivator activator in _linuxNumpadShortcuts.keys)
              activator as SingleActivator: const DoNothingAndStopPropagationTextIntent(),
          };
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.windows:
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          return _webDisablingTextShortcuts;
      }
    }
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return null;
      case TargetPlatform.iOS:
        return _iOSDisablingTextShortcuts;
      case TargetPlatform.macOS:
        return _macDisablingTextShortcuts;
    }
  }

  @override
  Widget build(BuildContext context) {
    Widget result = child;
    final Map<ShortcutActivator, Intent>? disablingShortcut = _getDisablingShortcut();
    if (disablingShortcut != null) {
      // These shortcuts make sure of the following:
      //
      // 1. Shortcuts fired when an EditableText is focused are ignored and
      //    forwarded to the platform by the EditableText's Actions, because it
      //    maps DoNothingAndStopPropagationTextIntent to DoNothingAction.
      // 2. Shortcuts fired when no EditableText is focused will still trigger
      //    _shortcuts assuming DoNothingAndStopPropagationTextIntent is
      //    unhandled elsewhere.
      result = Shortcuts(
        debugLabel: '<Web Disabling Text Editing Shortcuts>',
        shortcuts: disablingShortcut,
        child: result,
      );
    }
    return Shortcuts(
      debugLabel: '<Default Text Editing Shortcuts>',
      shortcuts: _shortcuts,
      child: result,
    );
  }
}

/// Maps the selector from NSStandardKeyBindingResponding to the Intent if the
/// selector is recognized.
Intent? intentForMacOSSelector(String selectorName) {
  const Map<String, Intent> selectorToIntent = <String, Intent>{
    'deleteBackward:': DeleteCharacterIntent(forward: false),
    'deleteWordBackward:': DeleteToNextWordBoundaryIntent(forward: false),
    'deleteToBeginningOfLine:': DeleteToLineBreakIntent(forward: false),
    'deleteForward:': DeleteCharacterIntent(forward: true),
    'deleteWordForward:': DeleteToNextWordBoundaryIntent(forward: true),
    'deleteToEndOfLine:': DeleteToLineBreakIntent(forward: true),

    'moveLeft:': ExtendSelectionByCharacterIntent(forward: false, collapseSelection: true),
    'moveRight:': ExtendSelectionByCharacterIntent(forward: true, collapseSelection: true),
    'moveForward:': ExtendSelectionByCharacterIntent(forward: true, collapseSelection: true),
    'moveBackward:': ExtendSelectionByCharacterIntent(forward: false, collapseSelection: true),

    'moveUp:': ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: false,
      collapseSelection: true,
    ),
    'moveDown:': ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: true,
      collapseSelection: true,
    ),

    'moveLeftAndModifySelection:': ExtendSelectionByCharacterIntent(
      forward: false,
      collapseSelection: false,
    ),
    'moveRightAndModifySelection:': ExtendSelectionByCharacterIntent(
      forward: true,
      collapseSelection: false,
    ),
    'moveUpAndModifySelection:': ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: false,
      collapseSelection: false,
    ),
    'moveDownAndModifySelection:': ExtendSelectionVerticallyToAdjacentLineIntent(
      forward: true,
      collapseSelection: false,
    ),

    'moveWordLeft:': ExtendSelectionToNextWordBoundaryIntent(
      forward: false,
      collapseSelection: true,
    ),
    'moveWordRight:': ExtendSelectionToNextWordBoundaryIntent(
      forward: true,
      collapseSelection: true,
    ),
    'moveToBeginningOfParagraph:': ExtendSelectionToLineBreakIntent(
      forward: false,
      collapseSelection: true,
    ),
    'moveToEndOfParagraph:': ExtendSelectionToLineBreakIntent(
      forward: true,
      collapseSelection: true,
    ),

    'moveWordLeftAndModifySelection:': ExtendSelectionToNextWordBoundaryOrCaretLocationIntent(
      forward: false,
    ),
    'moveWordRightAndModifySelection:': ExtendSelectionToNextWordBoundaryOrCaretLocationIntent(
      forward: true,
    ),
    'moveParagraphBackwardAndModifySelection:':
        ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent(forward: false),
    'moveParagraphForwardAndModifySelection:':
        ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent(forward: true),

    'moveToLeftEndOfLine:': ExtendSelectionToLineBreakIntent(
      forward: false,
      collapseSelection: true,
    ),
    'moveToRightEndOfLine:': ExtendSelectionToLineBreakIntent(
      forward: true,
      collapseSelection: true,
    ),
    'moveToBeginningOfDocument:': ExtendSelectionToDocumentBoundaryIntent(
      forward: false,
      collapseSelection: true,
    ),
    'moveToEndOfDocument:': ExtendSelectionToDocumentBoundaryIntent(
      forward: true,
      collapseSelection: true,
    ),

    'moveToLeftEndOfLineAndModifySelection:': ExpandSelectionToLineBreakIntent(forward: false),
    'moveToRightEndOfLineAndModifySelection:': ExpandSelectionToLineBreakIntent(forward: true),
    'moveToBeginningOfDocumentAndModifySelection:': ExpandSelectionToDocumentBoundaryIntent(
      forward: false,
    ),
    'moveToEndOfDocumentAndModifySelection:': ExpandSelectionToDocumentBoundaryIntent(
      forward: true,
    ),

    'transpose:': TransposeCharactersIntent(),

    'scrollToBeginningOfDocument:': ScrollToDocumentBoundaryIntent(forward: false),
    'scrollToEndOfDocument:': ScrollToDocumentBoundaryIntent(forward: true),

    'scrollPageUp:': ScrollIntent(direction: AxisDirection.up, type: ScrollIncrementType.page),
    'scrollPageDown:': ScrollIntent(direction: AxisDirection.down, type: ScrollIncrementType.page),
    'pageUpAndModifySelection:': ExtendSelectionVerticallyToAdjacentPageIntent(
      forward: false,
      collapseSelection: false,
    ),
    'pageDownAndModifySelection:': ExtendSelectionVerticallyToAdjacentPageIntent(
      forward: true,
      collapseSelection: false,
    ),

    // Escape key when there's no IME selection popup.
    'cancelOperation:': DismissIntent(),
    // Tab when there's no IME selection.
    'insertTab:': NextFocusIntent(),
    'insertBacktab:': PreviousFocusIntent(),
  };
  return selectorToIntent[selectorName];
}
