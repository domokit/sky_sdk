// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'text_input.dart';
library;

import 'dart:ui' show TextRange;

import 'package:flutter/foundation.dart';

import 'text_editing.dart';
import 'text_input.dart' show TextEditingValue;

export 'dart:ui' show TextRange;

export 'text_editing.dart' show TextSelection;
export 'text_input.dart' show TextEditingValue;

TextAffinity? _toTextAffinity(String? affinity) {
  return switch (affinity) {
    'TextAffinity.downstream' => TextAffinity.downstream,
    'TextAffinity.upstream' => TextAffinity.upstream,
    _ => null,
  };
}

// Replaces a range of text in the original string with the text given in the
// replacement string.
String _replace(String originalText, String replacementText, TextRange replacementRange) {
  assert(replacementRange.isValid);
  return originalText.replaceRange(replacementRange.start, replacementRange.end, replacementText);
}

// Verify that the given range is within the text.
bool _debugTextRangeIsValid(TextRange range, String text) {
  if (!range.isValid) {
    return true;
  }

  return (range.start >= 0 && range.start <= text.length) &&
      (range.end >= 0 && range.end <= text.length);
}

/// A structure representing a granular change that has occurred to the editing
/// state as a result of text editing.
///
/// See also:
///
///  * [TextEditingDeltaInsertion], a delta representing an insertion.
///  * [TextEditingDeltaDeletion], a delta representing a deletion.
///  * [TextEditingDeltaReplacement], a delta representing a replacement.
///  * [TextEditingDeltaNonTextUpdate], a delta representing an update to the
///    selection and/or composing region.
///  * [TextInputConfiguration], to opt-in your [DeltaTextInputClient] to receive
///    [TextEditingDelta]'s you must set [TextInputConfiguration.enableDeltaModel]
///    to true.
abstract class TextEditingDelta with Diagnosticable {
  /// Creates a delta for a given change to the editing state.
  const TextEditingDelta({required this.oldText, required this.selection, required this.composing});

  /// Creates an instance of this class from a JSON object by inferring the
  /// type of delta based on values sent from the engine.
  factory TextEditingDelta.fromJSON(Map<String, dynamic> encoded) {
    // An insertion delta is one where replacement destination is collapsed.
    //
    // A deletion delta is one where the replacement source is empty.
    //
    // An insertion/deletion can still occur when the replacement destination is not
    // collapsed, or the replacement source is not empty.
    //
    // On native platforms when composing text, the entire composing region is
    // replaced on input, rather than reporting character by character
    // insertion/deletion. In these cases we can detect if there was an
    // insertion/deletion by checking if the text inside the original composing
    // region was modified by the replacement. If the text is the same then we have
    // an insertion/deletion. If the text is different then we can say we have
    // a replacement.
    //
    // For example say we are currently composing the word: 'world'.
    // Our current state is 'worl|' with the cursor at the end of 'l'. If we
    // input the character 'd', the platform will tell us 'worl' was replaced
    // with 'world' at range (0,4). Here we can check if the text found in the
    // composing region (0,4) has been modified. We see that it hasn't because
    // 'worl' == 'worl', so this means that the text in
    // 'world'{replacementDestinationEnd, replacementDestinationStart + replacementSourceEnd}
    // can be considered an insertion. In this case we inserted 'd'.
    //
    // Similarly for a deletion, say we are currently composing the word: 'worl'.
    // Our current state is 'world|' with the cursor at the end of 'd'. If we
    // press backspace to delete the character 'd', the platform will tell us 'world'
    // was replaced with 'worl' at range (0,5). Here we can check if the text found
    // in the new composing region, is the same as the replacement text. We can do this
    // by using oldText{replacementDestinationStart, replacementDestinationStart + replacementSourceEnd}
    // which in this case is 'worl'. We then compare 'worl' with 'worl' and
    // verify that they are the same. This means that the text in
    // 'world'{replacementDestinationEnd, replacementDestinationStart + replacementSourceEnd} was deleted.
    // In this case the character 'd' was deleted.
    //
    // A replacement delta occurs when the original composing region has been
    // modified.
    //
    // A non text update delta occurs when the selection and/or composing region
    // has been changed by the platform, and there have been no changes to the
    // text value.
    final String oldText = encoded['oldText'] as String;
    final int replacementDestinationStart = encoded['deltaStart'] as int;
    final int replacementDestinationEnd = encoded['deltaEnd'] as int;
    final String replacementSource = encoded['deltaText'] as String;
    const int replacementSourceStart = 0;
    final int replacementSourceEnd = replacementSource.length;

    // This delta is explicitly a non text update.
    final bool isNonTextUpdate =
        replacementDestinationStart == -1 &&
        replacementDestinationStart == replacementDestinationEnd;
    final TextRange newComposing = TextRange(
      start: encoded['composingBase'] as int? ?? -1,
      end: encoded['composingExtent'] as int? ?? -1,
    );
    final TextSelection newSelection = TextSelection(
      baseOffset: encoded['selectionBase'] as int? ?? -1,
      extentOffset: encoded['selectionExtent'] as int? ?? -1,
      affinity: _toTextAffinity(encoded['selectionAffinity'] as String?) ?? TextAffinity.downstream,
      isDirectional: encoded['selectionIsDirectional'] as bool? ?? false,
    );

    if (isNonTextUpdate) {
      assert(
        _debugTextRangeIsValid(newSelection, oldText),
        'The selection range: $newSelection is not within the bounds of text: $oldText of length: ${oldText.length}',
      );
      assert(
        _debugTextRangeIsValid(newComposing, oldText),
        'The composing range: $newComposing is not within the bounds of text: $oldText of length: ${oldText.length}',
      );

      return TextEditingDeltaNonTextUpdate(
        oldText: oldText,
        selection: newSelection,
        composing: newComposing,
      );
    }

    assert(
      _debugTextRangeIsValid(
        TextRange(start: replacementDestinationStart, end: replacementDestinationEnd),
        oldText,
      ),
      'The delta range: ${TextRange(start: replacementSourceStart, end: replacementSourceEnd)} is not within the bounds of text: $oldText of length: ${oldText.length}',
    );

    final String newText = _replace(
      oldText,
      replacementSource,
      TextRange(start: replacementDestinationStart, end: replacementDestinationEnd),
    );

    assert(
      _debugTextRangeIsValid(newSelection, newText),
      'The selection range: $newSelection is not within the bounds of text: $newText of length: ${newText.length}',
    );
    assert(
      _debugTextRangeIsValid(newComposing, newText),
      'The composing range: $newComposing is not within the bounds of text: $newText of length: ${newText.length}',
    );

    final bool isEqual = oldText == newText;

    final bool isDeletionGreaterThanOne =
        (replacementDestinationEnd - replacementDestinationStart) -
            (replacementSourceEnd - replacementSourceStart) >
        1;
    final bool isDeletingByReplacingWithEmpty =
        replacementSource.isEmpty &&
        replacementSourceStart == 0 &&
        replacementSourceStart == replacementSourceEnd;

    final bool isReplacedByShorter =
        isDeletionGreaterThanOne &&
        (replacementSourceEnd - replacementSourceStart <
            replacementDestinationEnd - replacementDestinationStart);
    final bool isReplacedByLonger =
        replacementSourceEnd - replacementSourceStart >
        replacementDestinationEnd - replacementDestinationStart;
    final bool isReplacedBySame =
        replacementSourceEnd - replacementSourceStart ==
        replacementDestinationEnd - replacementDestinationStart;

    final bool isInsertingInsideComposingRegion =
        replacementDestinationStart + replacementSourceEnd > replacementDestinationEnd;
    final bool isDeletingInsideComposingRegion =
        !isReplacedByShorter &&
        !isDeletingByReplacingWithEmpty &&
        replacementDestinationStart + replacementSourceEnd < replacementDestinationEnd;

    String newComposingText;
    String originalComposingText;

    if (isDeletingByReplacingWithEmpty || isDeletingInsideComposingRegion || isReplacedByShorter) {
      newComposingText = replacementSource.substring(replacementSourceStart, replacementSourceEnd);
      originalComposingText = oldText.substring(
        replacementDestinationStart,
        replacementDestinationStart + replacementSourceEnd,
      );
    } else {
      newComposingText = replacementSource.substring(
        replacementSourceStart,
        replacementSourceStart + (replacementDestinationEnd - replacementDestinationStart),
      );
      originalComposingText = oldText.substring(
        replacementDestinationStart,
        replacementDestinationEnd,
      );
    }

    final bool isOriginalComposingRegionTextChanged = !(originalComposingText == newComposingText);
    final bool isReplaced =
        isOriginalComposingRegionTextChanged ||
        (isReplacedByLonger || isReplacedByShorter || isReplacedBySame);

    if (isEqual) {
      return TextEditingDeltaNonTextUpdate(
        oldText: oldText,
        selection: newSelection,
        composing: newComposing,
      );
    } else if ((isDeletingByReplacingWithEmpty || isDeletingInsideComposingRegion) &&
        !isOriginalComposingRegionTextChanged) {
      // Deletion.
      int actualStart = replacementDestinationStart;

      if (!isDeletionGreaterThanOne) {
        actualStart = replacementDestinationEnd - 1;
      }

      return TextEditingDeltaDeletion(
        oldText: oldText,
        deletedRange: TextRange(start: actualStart, end: replacementDestinationEnd),
        selection: newSelection,
        composing: newComposing,
      );
    } else if ((replacementDestinationStart == replacementDestinationEnd ||
            isInsertingInsideComposingRegion) &&
        !isOriginalComposingRegionTextChanged) {
      // Insertion.
      return TextEditingDeltaInsertion(
        oldText: oldText,
        textInserted: replacementSource.substring(
          replacementDestinationEnd - replacementDestinationStart,
          (replacementDestinationEnd - replacementDestinationStart) +
              (replacementSource.length -
                  (replacementDestinationEnd - replacementDestinationStart)),
        ),
        insertionOffset: replacementDestinationEnd,
        selection: newSelection,
        composing: newComposing,
      );
    } else if (isReplaced) {
      // Replacement.
      return TextEditingDeltaReplacement(
        oldText: oldText,
        replacementText: replacementSource,
        replacedRange: TextRange(
          start: replacementDestinationStart,
          end: replacementDestinationEnd,
        ),
        selection: newSelection,
        composing: newComposing,
      );
    }
    assert(false);
    return TextEditingDeltaNonTextUpdate(
      oldText: oldText,
      selection: newSelection,
      composing: newComposing,
    );
  }

  /// The old text state before the delta has occurred.
  final String oldText;

  /// The range of text that is currently selected after the delta has been
  /// applied.
  final TextSelection selection;

  /// The range of text that is still being composed after the delta has been
  /// applied.
  final TextRange composing;

  /// This method will take the given [TextEditingValue] and return a new
  /// [TextEditingValue] with that instance of [TextEditingDelta] applied to it.
  TextEditingValue apply(TextEditingValue value);
}

/// A structure representing an insertion of a single/or contiguous sequence of
/// characters at some offset of an editing state.
@immutable
class TextEditingDeltaInsertion extends TextEditingDelta {
  /// Creates an insertion delta for a given change to the editing state.
  ///
  /// {@template flutter.services.TextEditingDelta.optIn}
  /// See also:
  ///
  ///  * [TextInputConfiguration], to opt-in your [DeltaTextInputClient] to receive
  ///    [TextEditingDelta]'s you must set [TextInputConfiguration.enableDeltaModel]
  ///    to true.
  /// {@endtemplate}
  const TextEditingDeltaInsertion({
    required super.oldText,
    required this.textInserted,
    required this.insertionOffset,
    required super.selection,
    required super.composing,
  });

  /// The text that is being inserted into [oldText].
  final String textInserted;

  /// The offset in the [oldText] where the insertion begins.
  final int insertionOffset;

  @override
  TextEditingValue apply(TextEditingValue value) {
    // To stay inline with the plain text model we should follow a last write wins
    // policy and apply the delta to the oldText. This is due to the asynchronous
    // nature of the connection between the framework and platform text input plugins.
    String newText = oldText;
    assert(
      _debugTextRangeIsValid(TextRange.collapsed(insertionOffset), newText),
      'Applying TextEditingDeltaInsertion failed, the insertionOffset: $insertionOffset is not within the bounds of $newText of length: ${newText.length}',
    );
    newText = _replace(newText, textInserted, TextRange.collapsed(insertionOffset));
    assert(
      _debugTextRangeIsValid(selection, newText),
      'Applying TextEditingDeltaInsertion failed, the selection range: $selection is not within the bounds of $newText of length: ${newText.length}',
    );
    assert(
      _debugTextRangeIsValid(composing, newText),
      'Applying TextEditingDeltaInsertion failed, the composing range: $composing is not within the bounds of $newText of length: ${newText.length}',
    );
    return value.copyWith(text: newText, selection: selection, composing: composing);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('oldText', oldText));
    properties.add(DiagnosticsProperty<String>('textInserted', textInserted));
    properties.add(DiagnosticsProperty<int>('insertionOffset', insertionOffset));
    properties.add(DiagnosticsProperty<TextSelection>('selection', selection));
    properties.add(DiagnosticsProperty<TextRange>('composing', composing));
  }
}

/// A structure representing the deletion of a single/or contiguous sequence of
/// characters in an editing state.
@immutable
class TextEditingDeltaDeletion extends TextEditingDelta {
  /// Creates a deletion delta for a given change to the editing state.
  ///
  /// {@macro flutter.services.TextEditingDelta.optIn}
  const TextEditingDeltaDeletion({
    required super.oldText,
    required this.deletedRange,
    required super.selection,
    required super.composing,
  });

  /// The range in [oldText] that is being deleted.
  final TextRange deletedRange;

  /// The text from [oldText] that is being deleted.
  String get textDeleted => oldText.substring(deletedRange.start, deletedRange.end);

  @override
  TextEditingValue apply(TextEditingValue value) {
    // To stay inline with the plain text model we should follow a last write wins
    // policy and apply the delta to the oldText. This is due to the asynchronous
    // nature of the connection between the framework and platform text input plugins.
    String newText = oldText;
    assert(
      _debugTextRangeIsValid(deletedRange, newText),
      'Applying TextEditingDeltaDeletion failed, the deletedRange: $deletedRange is not within the bounds of $newText of length: ${newText.length}',
    );
    newText = _replace(newText, '', deletedRange);
    assert(
      _debugTextRangeIsValid(selection, newText),
      'Applying TextEditingDeltaDeletion failed, the selection range: $selection is not within the bounds of $newText of length: ${newText.length}',
    );
    assert(
      _debugTextRangeIsValid(composing, newText),
      'Applying TextEditingDeltaDeletion failed, the composing range: $composing is not within the bounds of $newText of length: ${newText.length}',
    );
    return value.copyWith(text: newText, selection: selection, composing: composing);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('oldText', oldText));
    properties.add(DiagnosticsProperty<String>('textDeleted', textDeleted));
    properties.add(DiagnosticsProperty<TextRange>('deletedRange', deletedRange));
    properties.add(DiagnosticsProperty<TextSelection>('selection', selection));
    properties.add(DiagnosticsProperty<TextRange>('composing', composing));
  }
}

/// A structure representing a replacement of a range of characters with a
/// new sequence of text.
@immutable
class TextEditingDeltaReplacement extends TextEditingDelta {
  /// Creates a replacement delta for a given change to the editing state.
  ///
  /// The range that is being replaced can either grow or shrink based on the
  /// given replacement text.
  ///
  /// A replacement can occur in cases such as auto-correct, suggestions, and
  /// when a selection is replaced by a single character.
  ///
  /// {@macro flutter.services.TextEditingDelta.optIn}
  const TextEditingDeltaReplacement({
    required super.oldText,
    required this.replacementText,
    required this.replacedRange,
    required super.selection,
    required super.composing,
  });

  /// The new text that is replacing [replacedRange] in [oldText].
  final String replacementText;

  /// The range in [oldText] that is being replaced.
  final TextRange replacedRange;

  /// The original text that is being replaced in [oldText].
  String get textReplaced => oldText.substring(replacedRange.start, replacedRange.end);

  @override
  TextEditingValue apply(TextEditingValue value) {
    // To stay inline with the plain text model we should follow a last write wins
    // policy and apply the delta to the oldText. This is due to the asynchronous
    // nature of the connection between the framework and platform text input plugins.
    String newText = oldText;
    assert(
      _debugTextRangeIsValid(replacedRange, newText),
      'Applying TextEditingDeltaReplacement failed, the replacedRange: $replacedRange is not within the bounds of $newText of length: ${newText.length}',
    );
    newText = _replace(newText, replacementText, replacedRange);
    assert(
      _debugTextRangeIsValid(selection, newText),
      'Applying TextEditingDeltaReplacement failed, the selection range: $selection is not within the bounds of $newText of length: ${newText.length}',
    );
    assert(
      _debugTextRangeIsValid(composing, newText),
      'Applying TextEditingDeltaReplacement failed, the composing range: $composing is not within the bounds of $newText of length: ${newText.length}',
    );
    return value.copyWith(text: newText, selection: selection, composing: composing);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('oldText', oldText));
    properties.add(DiagnosticsProperty<String>('textReplaced', textReplaced));
    properties.add(DiagnosticsProperty<String>('replacementText', replacementText));
    properties.add(DiagnosticsProperty<TextRange>('replacedRange', replacedRange));
    properties.add(DiagnosticsProperty<TextSelection>('selection', selection));
    properties.add(DiagnosticsProperty<TextRange>('composing', composing));
  }
}

/// A structure representing changes to the selection and/or composing regions
/// of an editing state and no changes to the text value.
@immutable
class TextEditingDeltaNonTextUpdate extends TextEditingDelta {
  /// Creates a delta representing no updates to the text value of the current
  /// editing state. This delta includes updates to the selection and/or composing
  /// regions.
  ///
  /// A situation where this delta would be created is when dragging the selection
  /// handles. There are no changes to the text, but there are updates to the selection
  /// and potentially the composing region as well.
  ///
  /// {@macro flutter.services.TextEditingDelta.optIn}
  const TextEditingDeltaNonTextUpdate({
    required super.oldText,
    required super.selection,
    required super.composing,
  });

  @override
  TextEditingValue apply(TextEditingValue value) {
    // To stay inline with the plain text model we should follow a last write wins
    // policy and apply the delta to the oldText. This is due to the asynchronous
    // nature of the connection between the framework and platform text input plugins.
    assert(
      _debugTextRangeIsValid(selection, oldText),
      'Applying TextEditingDeltaNonTextUpdate failed, the selection range: $selection is not within the bounds of $oldText of length: ${oldText.length}',
    );
    assert(
      _debugTextRangeIsValid(composing, oldText),
      'Applying TextEditingDeltaNonTextUpdate failed, the composing region: $composing is not within the bounds of $oldText of length: ${oldText.length}',
    );
    return TextEditingValue(text: oldText, selection: selection, composing: composing);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<String>('oldText', oldText));
    properties.add(DiagnosticsProperty<TextSelection>('selection', selection));
    properties.add(DiagnosticsProperty<TextRange>('composing', composing));
  }
}
