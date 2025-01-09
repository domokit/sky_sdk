// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'editable_text.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/services.dart'
    show SpellCheckResults, SpellCheckService, SuggestionSpan, TextEditingValue;

import 'editable_text.dart' show EditableTextContextMenuBuilder;

/// Controls how spell check is performed for text input.
///
/// This configuration determines the [SpellCheckService] used to fetch the
/// [List<SuggestionSpan>] spell check results and the [TextStyle] used to
/// mark misspelled words within text input.
@immutable
class SpellCheckConfiguration {
  /// Creates a configuration that specifies the service and suggestions handler
  /// for spell check.
  const SpellCheckConfiguration({
    this.spellCheckService,
    this.misspelledSelectionColor,
    this.misspelledTextStyle,
    this.spellCheckSuggestionsToolbarBuilder,
  }) : _spellCheckEnabled = true;

  /// Creates a configuration that disables spell check.
  const SpellCheckConfiguration.disabled()
    : _spellCheckEnabled = false,
      spellCheckService = null,
      spellCheckSuggestionsToolbarBuilder = null,
      misspelledTextStyle = null,
      misspelledSelectionColor = null;

  /// The service used to fetch spell check results for text input.
  final SpellCheckService? spellCheckService;

  /// The color the paint the selection highlight when spell check is showing
  /// suggestions for a misspelled word.
  ///
  /// For example, on iOS, the selection appears red while the spell check menu
  /// is showing.
  final Color? misspelledSelectionColor;

  /// Style used to indicate misspelled words.
  ///
  /// This is nullable to allow style-specific wrappers of [EditableText]
  /// to infer this, but this must be specified if this configuration is
  /// provided directly to [EditableText] or its construction will fail with an
  /// assertion error.
  final TextStyle? misspelledTextStyle;

  /// Builds the toolbar used to display spell check suggestions for misspelled
  /// words.
  final EditableTextContextMenuBuilder? spellCheckSuggestionsToolbarBuilder;

  final bool _spellCheckEnabled;

  /// Whether or not the configuration should enable or disable spell check.
  bool get spellCheckEnabled => _spellCheckEnabled;

  /// Returns a copy of the current [SpellCheckConfiguration] instance with
  /// specified overrides.
  SpellCheckConfiguration copyWith({
    SpellCheckService? spellCheckService,
    Color? misspelledSelectionColor,
    TextStyle? misspelledTextStyle,
    EditableTextContextMenuBuilder? spellCheckSuggestionsToolbarBuilder,
  }) {
    if (!_spellCheckEnabled) {
      // A new configuration should be constructed to enable spell check.
      return const SpellCheckConfiguration.disabled();
    }

    return SpellCheckConfiguration(
      spellCheckService: spellCheckService ?? this.spellCheckService,
      misspelledSelectionColor: misspelledSelectionColor ?? this.misspelledSelectionColor,
      misspelledTextStyle: misspelledTextStyle ?? this.misspelledTextStyle,
      spellCheckSuggestionsToolbarBuilder:
          spellCheckSuggestionsToolbarBuilder ?? this.spellCheckSuggestionsToolbarBuilder,
    );
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'SpellCheckConfiguration')}('
        '${_spellCheckEnabled ? 'enabled' : 'disabled'}, '
        'service: $spellCheckService, '
        'text style: $misspelledTextStyle, '
        'toolbar builder: $spellCheckSuggestionsToolbarBuilder'
        ')';
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SpellCheckConfiguration &&
        other.spellCheckService == spellCheckService &&
        other.misspelledTextStyle == misspelledTextStyle &&
        other.spellCheckSuggestionsToolbarBuilder == spellCheckSuggestionsToolbarBuilder &&
        other._spellCheckEnabled == _spellCheckEnabled;
  }

  @override
  int get hashCode => Object.hash(
    spellCheckService,
    misspelledTextStyle,
    spellCheckSuggestionsToolbarBuilder,
    _spellCheckEnabled,
  );
}

// Methods for displaying spell check results:

/// Adjusts spell check results to correspond to [newText] if the only results
/// that the handler has access to are the [results] corresponding to
/// [resultsText].
///
/// Used in the case where the request for the spell check results of the
/// [newText] is lagging in order to avoid display of incorrect results.
List<SuggestionSpan> _correctSpellCheckResults(
  String newText,
  String resultsText,
  List<SuggestionSpan> results,
) {
  final List<SuggestionSpan> correctedSpellCheckResults = <SuggestionSpan>[];
  int spanPointer = 0;
  int offset = 0;

  // Assumes that the order of spans has not been jumbled for optimization
  // purposes, and will only search since the previously found span.
  int searchStart = 0;

  while (spanPointer < results.length) {
    final SuggestionSpan currentSpan = results[spanPointer];
    final String currentSpanText = resultsText.substring(
      currentSpan.range.start,
      currentSpan.range.end,
    );
    final int spanLength = currentSpan.range.end - currentSpan.range.start;

    // Try finding SuggestionSpan from resultsText in new text.
    final String escapedText = RegExp.escape(currentSpanText);
    final RegExp currentSpanTextRegexp = RegExp('\\b$escapedText\\b');
    final int foundIndex = newText.substring(searchStart).indexOf(currentSpanTextRegexp);

    // Check whether word was found exactly where expected or elsewhere in the newText.
    final bool currentSpanFoundExactly = currentSpan.range.start == foundIndex + searchStart;
    final bool currentSpanFoundExactlyWithOffset =
        currentSpan.range.start + offset == foundIndex + searchStart;
    final bool currentSpanFoundElsewhere = foundIndex >= 0;

    if (currentSpanFoundExactly || currentSpanFoundExactlyWithOffset) {
      // currentSpan was found at the same index in newText and resultsText
      // or at the same index with the previously calculated adjustment by
      // the offset value, so apply it to new text by adding it to the list of
      // corrected results.
      final SuggestionSpan adjustedSpan = SuggestionSpan(
        TextRange(start: currentSpan.range.start + offset, end: currentSpan.range.end + offset),
        currentSpan.suggestions,
      );

      // Start search for the next misspelled word at the end of currentSpan.
      searchStart = math.min(currentSpan.range.end + 1 + offset, newText.length);
      correctedSpellCheckResults.add(adjustedSpan);
    } else if (currentSpanFoundElsewhere) {
      // Word was pushed forward but not modified.
      final int adjustedSpanStart = searchStart + foundIndex;
      final int adjustedSpanEnd = adjustedSpanStart + spanLength;
      final SuggestionSpan adjustedSpan = SuggestionSpan(
        TextRange(start: adjustedSpanStart, end: adjustedSpanEnd),
        currentSpan.suggestions,
      );

      // Start search for the next misspelled word at the end of the
      // adjusted currentSpan.
      searchStart = math.min(adjustedSpanEnd + 1, newText.length);
      // Adjust offset to reflect the difference between where currentSpan
      // was positioned in resultsText versus in newText.
      offset = adjustedSpanStart - currentSpan.range.start;
      correctedSpellCheckResults.add(adjustedSpan);
    }
    spanPointer++;
  }
  return correctedSpellCheckResults;
}

/// Builds the [TextSpan] tree given the current state of the text input and
/// spell check results.
///
/// The [value] is the current [TextEditingValue] requested to be rendered
/// by a text input widget. The [composingWithinCurrentTextRange] value
/// represents whether or not there is a valid composing region in the
/// [value]. The [style] is the [TextStyle] to render the [value]'s text with,
/// and the [misspelledTextStyle] is the [TextStyle] to render misspelled
/// words within the [value]'s text with. The [spellCheckResults] are the
/// results of spell checking the [value]'s text.
TextSpan buildTextSpanWithSpellCheckSuggestions(
  TextEditingValue value,
  bool composingWithinCurrentTextRange,
  TextStyle? style,
  TextStyle misspelledTextStyle,
  SpellCheckResults spellCheckResults,
) {
  List<SuggestionSpan> spellCheckResultsSpans = spellCheckResults.suggestionSpans;
  final String spellCheckResultsText = spellCheckResults.spellCheckedText;

  if (spellCheckResultsText != value.text) {
    spellCheckResultsSpans = _correctSpellCheckResults(
      value.text,
      spellCheckResultsText,
      spellCheckResultsSpans,
    );
  }

  // We will draw the TextSpan tree based on the composing region, if it is
  // available.
  // TODO(camsim99): The two separate strategies for building TextSpan trees
  // based on the availability of a composing region should be merged:
  // https://github.com/flutter/flutter/issues/124142.
  final bool shouldConsiderComposingRegion = defaultTargetPlatform == TargetPlatform.android;
  if (shouldConsiderComposingRegion) {
    return TextSpan(
      style: style,
      children: _buildSubtreesWithComposingRegion(
        spellCheckResultsSpans,
        value,
        style,
        misspelledTextStyle,
        composingWithinCurrentTextRange,
      ),
    );
  }

  return TextSpan(
    style: style,
    children: _buildSubtreesWithoutComposingRegion(
      spellCheckResultsSpans,
      value,
      style,
      misspelledTextStyle,
      value.selection.baseOffset,
    ),
  );
}

/// Builds the [TextSpan] tree for spell check without considering the composing
/// region. Instead, uses the cursor to identify the word that's actively being
/// edited and shouldn't be spell checked. This is useful for platforms and IMEs
/// that don't use the composing region for the active word.
List<TextSpan> _buildSubtreesWithoutComposingRegion(
  List<SuggestionSpan>? spellCheckSuggestions,
  TextEditingValue value,
  TextStyle? style,
  TextStyle misspelledStyle,
  int cursorIndex,
) {
  final List<TextSpan> textSpanTreeChildren = <TextSpan>[];

  int textPointer = 0;
  int currentSpanPointer = 0;
  int endIndex;
  final String text = value.text;
  final TextStyle misspelledJointStyle = style?.merge(misspelledStyle) ?? misspelledStyle;
  bool cursorInCurrentSpan = false;

  // Add text interwoven with any misspelled words to the tree.
  if (spellCheckSuggestions != null) {
    while (textPointer < text.length && currentSpanPointer < spellCheckSuggestions.length) {
      final SuggestionSpan currentSpan = spellCheckSuggestions[currentSpanPointer];

      if (currentSpan.range.start > textPointer) {
        endIndex = currentSpan.range.start < text.length ? currentSpan.range.start : text.length;
        textSpanTreeChildren.add(
          TextSpan(style: style, text: text.substring(textPointer, endIndex)),
        );
        textPointer = endIndex;
      } else {
        endIndex = currentSpan.range.end < text.length ? currentSpan.range.end : text.length;
        cursorInCurrentSpan =
            currentSpan.range.start <= cursorIndex && currentSpan.range.end >= cursorIndex;
        textSpanTreeChildren.add(
          TextSpan(
            style: cursorInCurrentSpan ? style : misspelledJointStyle,
            text: text.substring(currentSpan.range.start, endIndex),
          ),
        );

        textPointer = endIndex;
        currentSpanPointer++;
      }
    }
  }

  // Add any remaining text to the tree if applicable.
  if (textPointer < text.length) {
    textSpanTreeChildren.add(
      TextSpan(style: style, text: text.substring(textPointer, text.length)),
    );
  }

  return textSpanTreeChildren;
}

/// Builds [TextSpan] subtree for text with misspelled words with logic based on
/// a valid composing region.
List<TextSpan> _buildSubtreesWithComposingRegion(
  List<SuggestionSpan>? spellCheckSuggestions,
  TextEditingValue value,
  TextStyle? style,
  TextStyle misspelledStyle,
  bool composingWithinCurrentTextRange,
) {
  final List<TextSpan> textSpanTreeChildren = <TextSpan>[];

  int textPointer = 0;
  int currentSpanPointer = 0;
  int endIndex;
  SuggestionSpan currentSpan;
  final String text = value.text;
  final TextRange composingRegion = value.composing;
  final TextStyle composingTextStyle =
      style?.merge(const TextStyle(decoration: TextDecoration.underline)) ??
      const TextStyle(decoration: TextDecoration.underline);
  final TextStyle misspelledJointStyle = style?.merge(misspelledStyle) ?? misspelledStyle;
  bool textPointerWithinComposingRegion = false;
  bool currentSpanIsComposingRegion = false;

  // Add text interwoven with any misspelled words to the tree.
  if (spellCheckSuggestions != null) {
    while (textPointer < text.length && currentSpanPointer < spellCheckSuggestions.length) {
      currentSpan = spellCheckSuggestions[currentSpanPointer];

      if (currentSpan.range.start > textPointer) {
        endIndex = currentSpan.range.start < text.length ? currentSpan.range.start : text.length;
        textPointerWithinComposingRegion =
            composingRegion.start >= textPointer &&
            composingRegion.end <= endIndex &&
            !composingWithinCurrentTextRange;

        if (textPointerWithinComposingRegion) {
          _addComposingRegionTextSpans(
            textSpanTreeChildren,
            text,
            textPointer,
            composingRegion,
            style,
            composingTextStyle,
          );
          textSpanTreeChildren.add(
            TextSpan(style: style, text: text.substring(composingRegion.end, endIndex)),
          );
        } else {
          textSpanTreeChildren.add(
            TextSpan(style: style, text: text.substring(textPointer, endIndex)),
          );
        }

        textPointer = endIndex;
      } else {
        endIndex = currentSpan.range.end < text.length ? currentSpan.range.end : text.length;
        currentSpanIsComposingRegion =
            textPointer >= composingRegion.start &&
            endIndex <= composingRegion.end &&
            !composingWithinCurrentTextRange;
        textSpanTreeChildren.add(
          TextSpan(
            style: currentSpanIsComposingRegion ? composingTextStyle : misspelledJointStyle,
            text: text.substring(currentSpan.range.start, endIndex),
          ),
        );

        textPointer = endIndex;
        currentSpanPointer++;
      }
    }
  }

  // Add any remaining text to the tree if applicable.
  if (textPointer < text.length) {
    if (textPointer < composingRegion.start && !composingWithinCurrentTextRange) {
      _addComposingRegionTextSpans(
        textSpanTreeChildren,
        text,
        textPointer,
        composingRegion,
        style,
        composingTextStyle,
      );

      if (composingRegion.end != text.length) {
        textSpanTreeChildren.add(
          TextSpan(style: style, text: text.substring(composingRegion.end, text.length)),
        );
      }
    } else {
      textSpanTreeChildren.add(
        TextSpan(style: style, text: text.substring(textPointer, text.length)),
      );
    }
  }

  return textSpanTreeChildren;
}

/// Helper method to create [TextSpan] tree children for specified range of
/// text up to and including the composing region.
void _addComposingRegionTextSpans(
  List<TextSpan> treeChildren,
  String text,
  int start,
  TextRange composingRegion,
  TextStyle? style,
  TextStyle composingTextStyle,
) {
  treeChildren.add(TextSpan(style: style, text: text.substring(start, composingRegion.start)));
  treeChildren.add(
    TextSpan(
      style: composingTextStyle,
      text: text.substring(composingRegion.start, composingRegion.end),
    ),
  );
}
