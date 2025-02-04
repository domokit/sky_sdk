// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'default_text_editing_shortcuts.dart';
/// @docImport 'editable_text.dart';
library;

import 'package:flutter/services.dart';

import 'actions.dart';
import 'basic.dart';
import 'focus_manager.dart';

/// An [Intent] to send the event straight to the engine.
///
/// See also:
///
///   * [DefaultTextEditingShortcuts], which triggers this [Intent].
class DoNothingAndStopPropagationTextIntent extends Intent {
  /// Creates an instance of [DoNothingAndStopPropagationTextIntent].
  const DoNothingAndStopPropagationTextIntent();
}

/// A text editing related [Intent] that performs an operation towards a given
/// direction of the current caret location.
abstract class DirectionalTextEditingIntent extends Intent {
  /// Creates a [DirectionalTextEditingIntent].
  const DirectionalTextEditingIntent(this.forward);

  /// Whether the input field, if applicable, should perform the text editing
  /// operation from the current caret location towards the end of the document.
  ///
  /// Unless otherwise specified by the recipient of this intent, this parameter
  /// uses the logical order of characters in the string to determine the
  /// direction, and is not affected by the writing direction of the text.
  final bool forward;
}

/// Deletes the character before or after the caret location, based on whether
/// `forward` is true.
///
/// {@template flutter.widgets.TextEditingIntents.logicalOrder}
/// {@endtemplate}
///
/// Typically a text field will not respond to this intent if it has no active
/// caret ([TextSelection.isValid] is false for the current selection).
class DeleteCharacterIntent extends DirectionalTextEditingIntent {
  /// Creates a [DeleteCharacterIntent].
  const DeleteCharacterIntent({required bool forward}) : super(forward);
}

/// Deletes from the current caret location to the previous or next word
/// boundary, based on whether `forward` is true.
class DeleteToNextWordBoundaryIntent extends DirectionalTextEditingIntent {
  /// Creates a [DeleteToNextWordBoundaryIntent].
  const DeleteToNextWordBoundaryIntent({required bool forward}) : super(forward);
}

/// Deletes from the current caret location to the previous or next soft or hard
/// line break, based on whether `forward` is true.
class DeleteToLineBreakIntent extends DirectionalTextEditingIntent {
  /// Creates a [DeleteToLineBreakIntent].
  const DeleteToLineBreakIntent({required bool forward}) : super(forward);
}

/// A [DirectionalTextEditingIntent] that moves the caret or the selection to a
/// new location.
abstract class DirectionalCaretMovementIntent extends DirectionalTextEditingIntent {
  /// Creates a [DirectionalCaretMovementIntent].
  const DirectionalCaretMovementIntent(
    super.forward,
    this.collapseSelection, [
    this.collapseAtReversal = false,
    this.continuesAtWrap = false,
  ]) : assert(!collapseSelection || !collapseAtReversal);

  /// Whether this [Intent] should make the selection collapsed (so it becomes a
  /// caret), after the movement.
  ///
  /// When [collapseSelection] is false, the input field typically only moves
  /// the current [TextSelection.extent] to the new location, while maintains
  /// the current [TextSelection.base] location.
  ///
  /// When [collapseSelection] is true, the input field typically should move
  /// both the [TextSelection.base] and the [TextSelection.extent] to the new
  /// location.
  final bool collapseSelection;

  /// Whether to collapse the selection when it would otherwise reverse order.
  ///
  /// For example, consider when forward is true and the extent is before the
  /// base. If collapseAtReversal is true, then this will cause the selection to
  /// collapse at the base. If it's false, then the extent will be placed at the
  /// linebreak, reversing the order of base and offset.
  ///
  /// Cannot be true when collapseSelection is true.
  final bool collapseAtReversal;

  /// Whether or not to continue to the next line at a wordwrap.
  ///
  /// If true, when an [Intent] to go to the beginning/end of a wordwrapped line
  /// is received and the selection is already at the beginning/end of the line,
  /// then the selection will be moved to the next/previous line. If false, the
  /// selection will remain at the wordwrap.
  final bool continuesAtWrap;
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the previous or the next character
/// boundary.
class ExtendSelectionByCharacterIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionByCharacterIntent].
  const ExtendSelectionByCharacterIntent({required bool forward, required bool collapseSelection})
    : super(forward, collapseSelection);
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the previous or the next word
/// boundary.
class ExtendSelectionToNextWordBoundaryIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionToNextWordBoundaryIntent].
  const ExtendSelectionToNextWordBoundaryIntent({
    required bool forward,
    required bool collapseSelection,
  }) : super(forward, collapseSelection);
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the previous or the next word
/// boundary, or the [TextSelection.base] position if it's closer in the move
/// direction.
///
/// This [Intent] typically has the same effect as an
/// [ExtendSelectionToNextWordBoundaryIntent], except it collapses the selection
/// when the order of [TextSelection.base] and [TextSelection.extent] would
/// reverse.
///
/// This is typically only used on MacOS.
class ExtendSelectionToNextWordBoundaryOrCaretLocationIntent
    extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionToNextWordBoundaryOrCaretLocationIntent].
  const ExtendSelectionToNextWordBoundaryOrCaretLocationIntent({required bool forward})
    : super(forward, false, true);
}

/// Expands the current selection to the document boundary in the direction
/// given by [forward].
///
/// Unlike [ExpandSelectionToLineBreakIntent], the extent will be moved, which
/// matches the behavior on MacOS.
///
/// See also:
///
///   [ExtendSelectionToDocumentBoundaryIntent], which is similar but always
///   moves the extent.
class ExpandSelectionToDocumentBoundaryIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExpandSelectionToDocumentBoundaryIntent].
  const ExpandSelectionToDocumentBoundaryIntent({required bool forward}) : super(forward, false);
}

/// Expands the current selection to the closest line break in the direction
/// given by [forward].
///
/// Either the base or extent can move, whichever is closer to the line break.
/// The selection will never shrink.
///
/// This behavior is common on MacOS.
///
/// See also:
///
///   [ExtendSelectionToLineBreakIntent], which is similar but always moves the
///   extent.
class ExpandSelectionToLineBreakIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExpandSelectionToLineBreakIntent].
  const ExpandSelectionToLineBreakIntent({required bool forward}) : super(forward, false);
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the closest line break in the direction
/// given by [forward].
///
/// See also:
///
///   [ExpandSelectionToLineBreakIntent], which is similar but always increases
///   the size of the selection.
class ExtendSelectionToLineBreakIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionToLineBreakIntent].
  const ExtendSelectionToLineBreakIntent({
    required bool forward,
    required bool collapseSelection,
    bool collapseAtReversal = false,
    bool continuesAtWrap = false,
  }) : assert(!collapseSelection || !collapseAtReversal),
       super(forward, collapseSelection, collapseAtReversal, continuesAtWrap);
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the closest position on the adjacent
/// line.
class ExtendSelectionVerticallyToAdjacentLineIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionVerticallyToAdjacentLineIntent].
  const ExtendSelectionVerticallyToAdjacentLineIntent({
    required bool forward,
    required bool collapseSelection,
  }) : super(forward, collapseSelection);
}

/// Expands, or moves the current selection from the current
/// [TextSelection.extent] position to the closest position on the adjacent
/// page.
class ExtendSelectionVerticallyToAdjacentPageIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionVerticallyToAdjacentPageIntent].
  const ExtendSelectionVerticallyToAdjacentPageIntent({
    required bool forward,
    required bool collapseSelection,
  }) : super(forward, collapseSelection);
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the previous or the next paragraph
/// boundary.
class ExtendSelectionToNextParagraphBoundaryIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionToNextParagraphBoundaryIntent].
  const ExtendSelectionToNextParagraphBoundaryIntent({
    required bool forward,
    required bool collapseSelection,
  }) : super(forward, collapseSelection);
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the previous or the next paragraph
/// boundary depending on the [forward] parameter.
///
/// This [Intent] typically has the same effect as an
/// [ExtendSelectionToNextParagraphBoundaryIntent], except it collapses the selection
/// when the order of [TextSelection.base] and [TextSelection.extent] would
/// reverse.
///
/// This is typically only used on MacOS.
class ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent
    extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent].
  const ExtendSelectionToNextParagraphBoundaryOrCaretLocationIntent({required bool forward})
    : super(forward, false, true);
}

/// Extends, or moves the current selection from the current
/// [TextSelection.extent] position to the start or the end of the document.
///
/// See also:
///
///   [ExtendSelectionToDocumentBoundaryIntent], which is similar but always
///   increases the size of the selection.
class ExtendSelectionToDocumentBoundaryIntent extends DirectionalCaretMovementIntent {
  /// Creates an [ExtendSelectionToDocumentBoundaryIntent].
  const ExtendSelectionToDocumentBoundaryIntent({
    required bool forward,
    required bool collapseSelection,
  }) : super(forward, collapseSelection);
}

/// Scrolls to the beginning or end of the document depending on the [forward]
/// parameter.
class ScrollToDocumentBoundaryIntent extends DirectionalTextEditingIntent {
  /// Creates a [ScrollToDocumentBoundaryIntent].
  const ScrollToDocumentBoundaryIntent({required bool forward}) : super(forward);
}

/// Scrolls up or down by page depending on the [forward] parameter.
/// Extends the selection up or down by page based on the [forward] parameter.
class ExtendSelectionByPageIntent extends DirectionalTextEditingIntent {
  /// Creates a [ExtendSelectionByPageIntent].
  const ExtendSelectionByPageIntent({required bool forward}) : super(forward);
}

/// An [Intent] to select everything in the field.
class SelectAllTextIntent extends Intent {
  /// Creates an instance of [SelectAllTextIntent].
  const SelectAllTextIntent(this.cause);

  /// {@template flutter.widgets.TextEditingIntents.cause}
  /// The [SelectionChangedCause] that triggered the intent.
  /// {@endtemplate}
  final SelectionChangedCause cause;
}

/// An [Intent] that represents a user interaction that attempts to copy or cut
/// the current selection in the field.
class CopySelectionTextIntent extends Intent {
  const CopySelectionTextIntent._(this.cause, this.collapseSelection);

  /// Creates an [Intent] that represents a user interaction that attempts to
  /// cut the current selection in the field.
  const CopySelectionTextIntent.cut(SelectionChangedCause cause) : this._(cause, true);

  /// An [Intent] that represents a user interaction that attempts to copy the
  /// current selection in the field.
  static const CopySelectionTextIntent copy = CopySelectionTextIntent._(
    SelectionChangedCause.keyboard,
    false,
  );

  /// {@macro flutter.widgets.TextEditingIntents.cause}
  final SelectionChangedCause cause;

  /// Whether the original text needs to be removed from the input field if the
  /// copy action was successful.
  final bool collapseSelection;
}

/// An [Intent] to paste text from [Clipboard] to the field.
class PasteTextIntent extends Intent {
  /// Creates an instance of [PasteTextIntent].
  const PasteTextIntent(this.cause);

  /// {@macro flutter.widgets.TextEditingIntents.cause}
  final SelectionChangedCause cause;
}

/// An [Intent] that represents a user interaction that attempts to go back to
/// the previous editing state.
class RedoTextIntent extends Intent {
  /// Creates a [RedoTextIntent].
  const RedoTextIntent(this.cause);

  /// {@macro flutter.widgets.TextEditingIntents.cause}
  final SelectionChangedCause cause;
}

/// An [Intent] that represents a user interaction that attempts to modify the
/// current [TextEditingValue] in an input field.
class ReplaceTextIntent extends Intent {
  /// Creates a [ReplaceTextIntent].
  const ReplaceTextIntent(
    this.currentTextEditingValue,
    this.replacementText,
    this.replacementRange,
    this.cause,
  );

  /// The [TextEditingValue] that this [Intent]'s action should perform on.
  final TextEditingValue currentTextEditingValue;

  /// The text to replace the original text within the [replacementRange] with.
  final String replacementText;

  /// The range of text in [currentTextEditingValue] that needs to be replaced.
  final TextRange replacementRange;

  /// {@macro flutter.widgets.TextEditingIntents.cause}
  final SelectionChangedCause cause;
}

/// An [Intent] that represents a user interaction that attempts to go back to
/// the previous editing state.
class UndoTextIntent extends Intent {
  /// Creates an [UndoTextIntent].
  const UndoTextIntent(this.cause);

  /// {@macro flutter.widgets.TextEditingIntents.cause}
  final SelectionChangedCause cause;
}

/// An [Intent] that represents a user interaction that attempts to change the
/// selection in an input field.
class UpdateSelectionIntent extends Intent {
  /// Creates an [UpdateSelectionIntent].
  const UpdateSelectionIntent(this.currentTextEditingValue, this.newSelection, this.cause);

  /// The [TextEditingValue] that this [Intent]'s action should perform on.
  final TextEditingValue currentTextEditingValue;

  /// The new [TextSelection] the input field should adopt.
  final TextSelection newSelection;

  /// {@macro flutter.widgets.TextEditingIntents.cause}
  final SelectionChangedCause cause;
}

/// An [Intent] that represents a user interaction that attempts to swap the
/// characters immediately around the cursor.
class TransposeCharactersIntent extends Intent {
  /// Creates a [TransposeCharactersIntent].
  const TransposeCharactersIntent();
}

/// An [Intent] that represents a tap outside the field.
///
/// Invoked when the user taps outside the focused [EditableText] if
/// [EditableText.onTapOutside] is null.
///
/// Override this [Intent] to modify the default behavior, which is to unfocus
/// on a touch event on web and do nothing on other platforms.
///
/// See also:
///
///  * [Action.overridable] for an example on how to make an [Action]
///    overridable.
class EditableTextTapOutsideIntent extends Intent {
  /// Creates an [EditableTextTapOutsideIntent].
  const EditableTextTapOutsideIntent({required this.focusNode, required this.pointerDownEvent});

  /// The [FocusNode] that this [Intent]'s action should be performed on.
  final FocusNode focusNode;

  /// The [PointerDownEvent] that initiated this [Intent].
  final PointerDownEvent pointerDownEvent;
}

/// An [Intent] that represents a tap outside the field.
///
/// Invoked when the user taps up outside the focused [EditableText] if
/// [EditableText.onTapUpOutside] is null.
///
/// Override this [Intent] to modify the default behavior, which is to unfocus
/// on a touch event on web and do nothing on other platforms.
///
/// {@tool dartpad}
/// A common requirement is to unfocus text fields when the user taps outside of
/// it. For UX reasons, it's often desirable to only unfocus when the user taps
/// outside of the text field, but not when they scroll.
///
/// To achieve this, you can override the default behavior of
/// [EditableTextTapOutsideIntent] and [EditableTextTapUpOutsideIntent] to check
/// the difference in distance between the pointer down and pointer up events
/// before potentially unfocusing.
///
/// ** See code in examples/api/lib/widgets/text_editing_intents/editable_text_tap_up_outside_intent.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Action.overridable] for an example on how to make an [Action]
///    overridable.
class EditableTextTapUpOutsideIntent extends Intent {
  /// Creates an [EditableTextTapUpOutsideIntent].
  const EditableTextTapUpOutsideIntent({required this.focusNode, required this.pointerUpEvent});

  /// The [FocusNode] that this [Intent]'s action should be performed on.
  final FocusNode focusNode;

  /// The [PointerUpEvent] that initiated this [Intent].
  final PointerUpEvent pointerUpEvent;
}
