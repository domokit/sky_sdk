// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:math' as math;

import 'package:characters/characters.dart';
import 'package:flutter/foundation.dart';

import 'text_input.dart';

export 'package:flutter/foundation.dart' show TargetPlatform;

export 'text_input.dart' show TextEditingValue;

// Examples can assume:
// late RegExp _pattern;

/// Mechanisms for enforcing maximum length limits.
///
/// This is used by [TextField] to specify how the [TextField.maxLength] should
/// be applied.
///
/// {@template flutter.services.textFormatter.maxLengthEnforcement}
/// ### [MaxLengthEnforcement.enforced] versus
/// [MaxLengthEnforcement.truncateAfterCompositionEnds]
///
/// Both [MaxLengthEnforcement.enforced] and
/// [MaxLengthEnforcement.truncateAfterCompositionEnds] make sure the final
/// length of the text does not exceed the max length specified. The difference
/// is that [MaxLengthEnforcement.enforced] truncates all text while
/// [MaxLengthEnforcement.truncateAfterCompositionEnds] allows composing text to
/// exceed the limit. Allowing this "placeholder" composing text to exceed the
/// limit may provide a better user experience on some platforms for entering
/// ideographic characters (e.g. CJK characters) via composing on phonetic
/// keyboards.
///
/// Some input methods (Gboard on Android for example) initiate text composition
/// even for Latin characters, in which case the best experience may be to
/// truncate those composing characters with [MaxLengthEnforcement.enforced].
///
/// In fields that strictly support only a small subset of characters, such as
/// verification code fields, [MaxLengthEnforcement.enforced] may provide the
/// best experience.
/// {@endtemplate}
///
/// See also:
///
///  * [TextField.maxLengthEnforcement] which is used in conjunction with
///    [TextField.maxLength] to limit the length of user input. [TextField] also
///    provides a character counter to provide visual feedback.
enum MaxLengthEnforcement {
  /// No enforcement applied to the editing value. It's possible to exceed the
  /// max length.
  none,

  /// Keep the length of the text input from exceeding the max length even when
  /// the text has an unfinished composing region.
  enforced,

  /// Users can still input text if the current value is composing even after
  /// reaching the max length limit. After composing ends, the value will be
  /// truncated.
  truncateAfterCompositionEnds,
}

/// A [TextInputFormatter] can be optionally injected into an [EditableText]
/// to provide as-you-type validation and formatting of the text being edited.
///
/// Text modification should only be applied when text is being committed by the
/// IME and not on text under composition (i.e., only when
/// [TextEditingValue.composing] is collapsed).
///
/// See also the [FilteringTextInputFormatter], a subclass that
/// removes characters that the user tries to enter if they do, or do
/// not, match a given pattern (as applicable).
///
/// To create custom formatters, extend the [TextInputFormatter] class and
/// implement the [formatEditUpdate] method.
///
/// ## Handling emojis and other complex characters
/// {@macro flutter.widgets.EditableText.onChanged}
///
/// See also:
///
///  * [EditableText] on which the formatting apply.
///  * [FilteringTextInputFormatter], a provided formatter for filtering
///    characters.
abstract class TextInputFormatter {
  /// This constructor enables subclasses to provide const constructors so that they can be used in const expressions.
  const TextInputFormatter();

  /// A shorthand to creating a custom [TextInputFormatter] which formats
  /// incoming text input changes with the given function.
  const factory TextInputFormatter.withFunction(TextInputFormatFunction formatFunction) =
      _SimpleTextInputFormatter;

  /// Called when text is being typed or cut/copy/pasted in the [EditableText].
  ///
  /// You can override the resulting text based on the previous text value and
  /// the incoming new text value.
  ///
  /// When formatters are chained, `oldValue` reflects the initial value of
  /// [TextEditingValue] at the beginning of the chain.
  TextEditingValue formatEditUpdate(TextEditingValue oldValue, TextEditingValue newValue);
}

/// Function signature expected for creating custom [TextInputFormatter]
/// shorthands via [TextInputFormatter.withFunction].
typedef TextInputFormatFunction =
    TextEditingValue Function(TextEditingValue oldValue, TextEditingValue newValue);

/// Wiring for [TextInputFormatter.withFunction].
class _SimpleTextInputFormatter extends TextInputFormatter {
  const _SimpleTextInputFormatter(this.formatFunction);

  final TextInputFormatFunction formatFunction;

  @override
  TextEditingValue formatEditUpdate(TextEditingValue oldValue, TextEditingValue newValue) {
    return formatFunction(oldValue, newValue);
  }
}

// A mutable, half-open range [`base`, `extent`) within a string.
class _MutableTextRange {
  _MutableTextRange(this.base, this.extent);

  static _MutableTextRange? fromComposingRange(TextRange range) {
    return range.isValid && !range.isCollapsed ? _MutableTextRange(range.start, range.end) : null;
  }

  static _MutableTextRange? fromTextSelection(TextSelection selection) {
    return selection.isValid
        ? _MutableTextRange(selection.baseOffset, selection.extentOffset)
        : null;
  }

  /// The start index of the range, inclusive.
  ///
  /// The value of [base] should always be greater than or equal to 0, and can
  /// be larger than, smaller than, or equal to [extent].
  int base;

  /// The end index of the range, exclusive.
  ///
  /// The value of [extent] should always be greater than or equal to 0, and can
  /// be larger than, smaller than, or equal to [base].
  int extent;
}

// The intermediate state of a [FilteringTextInputFormatter] when it's
// formatting a new user input.
class _TextEditingValueAccumulator {
  _TextEditingValueAccumulator(this.inputValue)
    : selection = _MutableTextRange.fromTextSelection(inputValue.selection),
      composingRegion = _MutableTextRange.fromComposingRange(inputValue.composing);

  // The original string that was sent to the [FilteringTextInputFormatter] as
  // input.
  final TextEditingValue inputValue;

  /// The [StringBuffer] that contains the string which has already been
  /// formatted.
  ///
  /// In a [FilteringTextInputFormatter], typically the replacement string,
  /// instead of the original string within the given range, is written to this
  /// [StringBuffer].
  final StringBuffer stringBuffer = StringBuffer();

  /// The updated selection, as well as the original selection from the input
  /// [TextEditingValue] of the [FilteringTextInputFormatter].
  ///
  /// This parameter will be null if the input [TextEditingValue.selection] is
  /// invalid.
  final _MutableTextRange? selection;

  /// The updated composing region, as well as the original composing region
  /// from the input [TextEditingValue] of the [FilteringTextInputFormatter].
  ///
  /// This parameter will be null if the input [TextEditingValue.composing] is
  /// invalid or collapsed.
  final _MutableTextRange? composingRegion;

  // Whether this state object has reached its end-of-life.
  bool debugFinalized = false;

  TextEditingValue finalize() {
    debugFinalized = true;
    final _MutableTextRange? selection = this.selection;
    final _MutableTextRange? composingRegion = this.composingRegion;
    return TextEditingValue(
      text: stringBuffer.toString(),
      composing:
          composingRegion == null || composingRegion.base == composingRegion.extent
              ? TextRange.empty
              : TextRange(start: composingRegion.base, end: composingRegion.extent),
      selection:
          selection == null
              ? const TextSelection.collapsed(offset: -1)
              : TextSelection(
                baseOffset: selection.base,
                extentOffset: selection.extent,
                // Try to preserve the selection affinity and isDirectional. This
                // may not make sense if the selection has changed.
                affinity: inputValue.selection.affinity,
                isDirectional: inputValue.selection.isDirectional,
              ),
    );
  }
}

/// A [TextInputFormatter] that prevents the insertion of characters matching
/// (or not matching) a particular pattern, by replacing the characters with the
/// given [replacementString].
///
/// Instances of filtered characters found in the new [TextEditingValue]s
/// will be replaced by the [replacementString] which defaults to the empty
/// string, and the current [TextEditingValue.selection] and
/// [TextEditingValue.composing] region will be adjusted to account for the
/// replacement.
///
/// This formatter is typically used to match potentially recurring [Pattern]s
/// in the new [TextEditingValue]. It never completely rejects the new
/// [TextEditingValue] and falls back to the current [TextEditingValue] when the
/// given [filterPattern] fails to match. Consider using a different
/// [TextInputFormatter] such as:
///
/// ```dart
/// // _pattern is a RegExp or other Pattern object
/// TextInputFormatter.withFunction(
///   (TextEditingValue oldValue, TextEditingValue newValue) {
///     return _pattern.hasMatch(newValue.text) ? newValue : oldValue;
///   },
/// ),
/// ```
///
/// for accepting/rejecting new input based on a predicate on the full string.
/// As an example, [FilteringTextInputFormatter] typically shouldn't be used
/// with [RegExp]s that contain positional matchers (`^` or `$`) since these
/// patterns are usually meant for matching the whole string.
///
/// ### Quote characters on iOS
///
/// When filtering single (`'`) or double (`"`) quote characters, be aware that
/// the default iOS keyboard actually inserts special directional versions of
/// these characters (`‘` and `’` for single quote, and `“` and `”` for double
/// quote). Consider including all three variants in your regular expressions to
/// support iOS.
class FilteringTextInputFormatter extends TextInputFormatter {
  /// Creates a formatter that replaces banned patterns with the given
  /// [replacementString].
  ///
  /// If [allow] is true, then the filter pattern is an allow list,
  /// and characters must match the pattern to be accepted. See also
  /// the [FilteringTextInputFormatter.allow()] constructor.
  ///
  /// If [allow] is false, then the filter pattern is a deny list,
  /// and characters that match the pattern are rejected. See also
  /// the [FilteringTextInputFormatter.deny] constructor.
  FilteringTextInputFormatter(
    this.filterPattern, {
    required this.allow,
    this.replacementString = '',
  });

  /// Creates a formatter that only allows characters matching a pattern.
  FilteringTextInputFormatter.allow(Pattern filterPattern, {String replacementString = ''})
    : this(filterPattern, allow: true, replacementString: replacementString);

  /// Creates a formatter that blocks characters matching a pattern.
  FilteringTextInputFormatter.deny(Pattern filterPattern, {String replacementString = ''})
    : this(filterPattern, allow: false, replacementString: replacementString);

  /// A [Pattern] to match or replace in incoming [TextEditingValue]s.
  ///
  /// The behavior of the pattern depends on the [allow] property. If
  /// it is true, then this is an allow list, specifying a pattern that
  /// characters must match to be accepted. Otherwise, it is a deny list,
  /// specifying a pattern that characters must not match to be accepted.
  ///
  /// {@tool snippet}
  /// Typically the pattern is a regular expression, as in:
  ///
  /// ```dart
  /// FilteringTextInputFormatter onlyDigits = FilteringTextInputFormatter.allow(RegExp(r'[0-9]'));
  /// ```
  /// {@end-tool}
  ///
  /// {@tool snippet}
  /// If the pattern is a single character, a pattern consisting of a
  /// [String] can be used:
  ///
  /// ```dart
  /// FilteringTextInputFormatter noTabs = FilteringTextInputFormatter.deny('\t');
  /// ```
  /// {@end-tool}
  final Pattern filterPattern;

  /// Whether the pattern is an allow list or not.
  ///
  /// When true, [filterPattern] denotes an allow list: characters
  /// must match the filter to be allowed.
  ///
  /// When false, [filterPattern] denotes a deny list: characters
  /// that match the filter are disallowed.
  final bool allow;

  /// String used to replace banned patterns.
  ///
  /// For deny lists ([allow] is false), each match of the
  /// [filterPattern] is replaced with this string. If [filterPattern]
  /// can match more than one character at a time, then this can
  /// result in multiple characters being replaced by a single
  /// instance of this [replacementString].
  ///
  /// For allow lists ([allow] is true), sequences between matches of
  /// [filterPattern] are replaced as one, regardless of the number of
  /// characters.
  ///
  /// For example, consider a [filterPattern] consisting of just the
  /// letter "o", applied to text field whose initial value is the
  /// string "Into The Woods", with the [replacementString] set to
  /// `*`.
  ///
  /// If [allow] is true, then the result will be "*o*oo*". Each
  /// sequence of characters not matching the pattern is replaced by
  /// its own single copy of the replacement string, regardless of how
  /// many characters are in that sequence.
  ///
  /// If [allow] is false, then the result will be "Int* the W**ds".
  /// Every matching sequence is replaced, and each "o" matches the
  /// pattern separately.
  ///
  /// If the pattern was the [RegExp] `o+`, the result would be the
  /// same in the case where [allow] is true, but in the case where
  /// [allow] is false, the result would be "Int* the W*ds" (with the
  /// two "o"s replaced by a single occurrence of the replacement
  /// string) because both of the "o"s would be matched simultaneously
  /// by the pattern.
  ///
  /// The filter may adjust the selection and the composing region of the text
  /// after applying the text replacement, such that they still cover the same
  /// text. For instance, if the pattern was `o+` and the last character "s" was
  /// selected: "Into The Wood|s|", then the result will be "Into The W*d|s|",
  /// with the selection still around the same character "s" despite that it is
  /// now the 12th character.
  ///
  /// In the case where one end point of the selection (or the composing region)
  /// is strictly inside the banned pattern (for example, "Into The |Wo|ods"),
  /// that endpoint will be moved to the end of the replacement string (it will
  /// become "Into The |W*|ds" if the pattern was `o+` and the original text and
  /// selection were "Into The |Wo|ods").
  final String replacementString;

  @override
  TextEditingValue formatEditUpdate(
    TextEditingValue oldValue, // unused.
    TextEditingValue newValue,
  ) {
    final _TextEditingValueAccumulator formatState = _TextEditingValueAccumulator(newValue);
    assert(!formatState.debugFinalized);

    final Iterable<Match> matches = filterPattern.allMatches(newValue.text);
    Match? previousMatch;
    for (final Match match in matches) {
      assert(match.end >= match.start);
      // Compute the non-match region between this `Match` and the previous
      // `Match`. Depending on the value of `allow`, either the match region or
      // the non-match region is the banned pattern.
      //
      // The non-matching region.
      _processRegion(allow, previousMatch?.end ?? 0, match.start, formatState);
      assert(!formatState.debugFinalized);
      // The matched region.
      _processRegion(!allow, match.start, match.end, formatState);
      assert(!formatState.debugFinalized);

      previousMatch = match;
    }

    // Handle the last non-matching region between the last match region and the
    // end of the text.
    _processRegion(allow, previousMatch?.end ?? 0, newValue.text.length, formatState);
    assert(!formatState.debugFinalized);
    return formatState.finalize();
  }

  void _processRegion(
    bool isBannedRegion,
    int regionStart,
    int regionEnd,
    _TextEditingValueAccumulator state,
  ) {
    final String replacementString =
        isBannedRegion
            ? (regionStart == regionEnd ? '' : this.replacementString)
            : state.inputValue.text.substring(regionStart, regionEnd);

    state.stringBuffer.write(replacementString);

    if (replacementString.length == regionEnd - regionStart) {
      // We don't have to adjust the indices if the replaced string and the
      // replacement string have the same length.
      return;
    }

    int adjustIndex(int originalIndex) {
      // The length added by adding the replacementString.
      final int replacedLength =
          originalIndex <= regionStart && originalIndex < regionEnd ? 0 : replacementString.length;
      // The length removed by removing the replacementRange.
      final int removedLength = originalIndex.clamp(regionStart, regionEnd) - regionStart;
      return replacedLength - removedLength;
    }

    state.selection?.base += adjustIndex(state.inputValue.selection.baseOffset);
    state.selection?.extent += adjustIndex(state.inputValue.selection.extentOffset);
    state.composingRegion?.base += adjustIndex(state.inputValue.composing.start);
    state.composingRegion?.extent += adjustIndex(state.inputValue.composing.end);
  }

  /// A [TextInputFormatter] that forces input to be a single line.
  static final TextInputFormatter singleLineFormatter = FilteringTextInputFormatter.deny('\n');

  /// A [TextInputFormatter] that takes in digits `[0-9]` only.
  static final TextInputFormatter digitsOnly = FilteringTextInputFormatter.allow(RegExp(r'[0-9]'));
}

/// A [TextInputFormatter] that prevents the insertion of more characters
/// than allowed.
///
/// Since this formatter only prevents new characters from being added to the
/// text, it preserves the existing [TextEditingValue.selection].
///
/// Characters are counted as user-perceived characters using the
/// [characters](https://pub.dev/packages/characters) package, so even complex
/// characters like extended grapheme clusters and surrogate pairs are counted
/// as single characters.
///
/// See also:
///  * [maxLength], which discusses the precise meaning of "number of
///    characters".
class LengthLimitingTextInputFormatter extends TextInputFormatter {
  /// Creates a formatter that prevents the insertion of more characters than a
  /// limit.
  ///
  /// The [maxLength] must be null, -1 or greater than zero. If it is null or -1
  /// then no limit is enforced.
  LengthLimitingTextInputFormatter(this.maxLength, {this.maxLengthEnforcement})
    : assert(maxLength == null || maxLength == -1 || maxLength > 0);

  /// The limit on the number of user-perceived characters that this formatter
  /// will allow.
  ///
  /// The value must be null or greater than zero. If it is null or -1, then no
  /// limit is enforced.
  ///
  /// {@template flutter.services.lengthLimitingTextInputFormatter.maxLength}
  /// ## Characters
  ///
  /// For a specific definition of what is considered a character, see the
  /// [characters](https://pub.dev/packages/characters) package on Pub, which is
  /// what Flutter uses to delineate characters. In general, even complex
  /// characters like surrogate pairs and extended grapheme clusters are
  /// correctly interpreted by Flutter as each being a single user-perceived
  /// character.
  ///
  /// For instance, the character "ö" can be represented as '\u{006F}\u{0308}',
  /// which is the letter "o" followed by a composed diaeresis "¨", or it can
  /// be represented as '\u{00F6}', which is the Unicode scalar value "LATIN
  /// SMALL LETTER O WITH DIAERESIS". It will be counted as a single character
  /// in both cases.
  ///
  /// Similarly, some emoji are represented by multiple scalar values. The
  /// Unicode "THUMBS UP SIGN + MEDIUM SKIN TONE MODIFIER", "👍🏽"is counted as
  /// a single character, even though it is a combination of two Unicode scalar
  /// values, '\u{1F44D}\u{1F3FD}'.
  /// {@endtemplate}
  ///
  /// ### Composing text behaviors
  ///
  /// There is no guarantee for the final value before the composing ends.
  /// So while the value is composing, the constraint of [maxLength] will be
  /// temporary lifted until the composing ends.
  ///
  /// In addition, if the current value already reached the [maxLength],
  /// composing is not allowed.
  final int? maxLength;

  /// Determines how the [maxLength] limit should be enforced.
  ///
  /// Defaults to [MaxLengthEnforcement.enforced].
  ///
  /// {@macro flutter.services.textFormatter.maxLengthEnforcement}
  final MaxLengthEnforcement? maxLengthEnforcement;

  /// Returns a [MaxLengthEnforcement] that follows the specified [platform]'s
  /// convention.
  ///
  /// {@template flutter.services.textFormatter.effectiveMaxLengthEnforcement}
  /// ### Platform specific behaviors
  ///
  /// Different platforms follow different behaviors by default, according to
  /// their native behavior.
  ///  * Android, Windows: [MaxLengthEnforcement.enforced]. The native behavior
  ///    of these platforms is enforced. The composing will be handled by the
  ///    IME while users are entering CJK characters.
  ///  * iOS: [MaxLengthEnforcement.truncateAfterCompositionEnds]. iOS has no
  ///    default behavior and it requires users implement the behavior
  ///    themselves. Allow the composition to exceed to avoid breaking CJK input.
  ///  * Web, macOS, linux, fuchsia:
  ///    [MaxLengthEnforcement.truncateAfterCompositionEnds]. These platforms
  ///    allow the composition to exceed by default.
  /// {@endtemplate}
  static MaxLengthEnforcement getDefaultMaxLengthEnforcement([TargetPlatform? platform]) {
    if (kIsWeb) {
      return MaxLengthEnforcement.truncateAfterCompositionEnds;
    } else {
      switch (platform ?? defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.windows:
          return MaxLengthEnforcement.enforced;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
        case TargetPlatform.linux:
        case TargetPlatform.fuchsia:
          return MaxLengthEnforcement.truncateAfterCompositionEnds;
      }
    }
  }

  /// Truncate the given TextEditingValue to maxLength user-perceived
  /// characters.
  ///
  /// See also:
  ///  * [Dart's characters package](https://pub.dev/packages/characters).
  ///  * [Dart's documentation on runes and grapheme clusters](https://dart.dev/guides/language/language-tour#runes-and-grapheme-clusters).
  @visibleForTesting
  static TextEditingValue truncate(TextEditingValue value, int maxLength) {
    final CharacterRange iterator = CharacterRange(value.text);
    if (value.text.characters.length > maxLength) {
      iterator.expandNext(maxLength);
    }
    final String truncated = iterator.current;

    return TextEditingValue(
      text: truncated,
      selection: value.selection.copyWith(
        baseOffset: math.min(value.selection.start, truncated.length),
        extentOffset: math.min(value.selection.end, truncated.length),
      ),
      composing:
          !value.composing.isCollapsed && truncated.length > value.composing.start
              ? TextRange(
                start: value.composing.start,
                end: math.min(value.composing.end, truncated.length),
              )
              : TextRange.empty,
    );
  }

  @override
  TextEditingValue formatEditUpdate(TextEditingValue oldValue, TextEditingValue newValue) {
    final int? maxLength = this.maxLength;

    if (maxLength == null || maxLength == -1 || newValue.text.characters.length <= maxLength) {
      return newValue;
    }

    assert(maxLength > 0);

    switch (maxLengthEnforcement ?? getDefaultMaxLengthEnforcement()) {
      case MaxLengthEnforcement.none:
        return newValue;
      case MaxLengthEnforcement.enforced:
        // If already at the maximum and tried to enter even more, and has no
        // selection, keep the old value.
        if (oldValue.text.characters.length == maxLength && oldValue.selection.isCollapsed) {
          return oldValue;
        }

        // Enforced to return a truncated value.
        return truncate(newValue, maxLength);
      case MaxLengthEnforcement.truncateAfterCompositionEnds:
        // If already at the maximum and tried to enter even more, and the old
        // value is not composing, keep the old value.
        if (oldValue.text.characters.length == maxLength && !oldValue.composing.isValid) {
          return oldValue;
        }

        // Temporarily exempt `newValue` from the maxLength limit if it has a
        // composing text going and no enforcement to the composing value, until
        // the composing is finished.
        if (newValue.composing.isValid) {
          return newValue;
        }

        return truncate(newValue, maxLength);
    }
  }
}
