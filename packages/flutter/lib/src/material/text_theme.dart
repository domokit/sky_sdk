// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'elevated_button.dart';
/// @docImport 'material.dart';
/// @docImport 'outlined_button.dart';
/// @docImport 'text_button.dart';
/// @docImport 'theme.dart';
/// @docImport 'theme_data.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'theme.dart';
import 'typography.dart';

/// Material design text theme.
///
/// Definitions for the various typographical styles found in Material Design
/// (e.g., labelLarge, bodySmall). Rather than creating a [TextTheme] directly,
/// you can obtain an instance as [Typography.black] or [Typography.white].
///
/// To obtain the current text theme, call [TextTheme.of] with the current
/// [BuildContext]. This is equivalent to calling [Theme.of] and reading
/// the [ThemeData.textTheme] property.
///
/// The names of the TextTheme properties match this table from the
/// [Material Design spec](https://m3.material.io/styles/typography/tokens).
///
/// ![](https://lh3.googleusercontent.com/Yvngs5mQSjXa_9T4X3JDucO62c5hdZHPDa7qeRH6DsJQvGr_q7EBrTkhkPiQd9OeR1v_Uk38Cjd9nUpP3nevDyHpKWuXSfQ1Gq78bOnBN7sr=s0)
///
/// The Material Design typography scheme was significantly changed in the
/// current (2021) version of the specification
/// ([https://m3.material.io/styles/typography/tokens](https://m3.material.io/styles/typography/tokens)).
///
/// The **2021** spec has fifteen text styles:
///
/// | NAME           | SIZE |  HEIGHT |  WEIGHT |  SPACING |             |
/// |----------------|------|---------|---------|----------|-------------|
/// | displayLarge   | 57.0 |   64.0  | regular | -0.25    |             |
/// | displayMedium  | 45.0 |   52.0  | regular |  0.0     |             |
/// | displaySmall   | 36.0 |   44.0  | regular |  0.0     |             |
/// | headlineLarge  | 32.0 |   40.0  | regular |  0.0     |             |
/// | headlineMedium | 28.0 |   36.0  | regular |  0.0     |             |
/// | headlineSmall  | 24.0 |   32.0  | regular |  0.0     |             |
/// | titleLarge     | 22.0 |   28.0  | regular |  0.0     |             |
/// | titleMedium    | 16.0 |   24.0  | medium  |  0.15    |             |
/// | titleSmall     | 14.0 |   20.0  | medium  |  0.1     |             |
/// | bodyLarge      | 16.0 |   24.0  | regular |  0.5     |             |
/// | bodyMedium     | 14.0 |   20.0  | regular |  0.25    |             |
/// | bodySmall      | 12.0 |   16.0  | regular |  0.4     |             |
/// | labelLarge     | 14.0 |   20.0  | medium  |  0.1     |             |
/// | labelMedium    | 12.0 |   16.0  | medium  |  0.5     |             |
/// | labelSmall     | 11.0 |   16.0  | medium  |  0.5     |             |
///
/// ...where "regular" is `FontWeight.w400` and "medium" is `FontWeight.w500`.
///
/// The names of the 2018 TextTheme properties match this table from the
/// [Material Design spec](https://material.io/design/typography/the-type-system.html#type-scale)
/// with a few exceptions: the styles called H1-H6 in the spec are
/// displayLarge-titleLarge in the API chart, body1,body2 are called
/// bodyLarge and bodyMedium, caption is now bodySmall, button is labelLarge,
/// and overline is now labelSmall.
///
/// The **2018** spec has thirteen text styles:
///
/// | NAME           | SIZE |  WEIGHT |  SPACING |             |
/// |----------------|------|---------|----------|-------------|
/// | displayLarge   | 96.0 | light   | -1.5     |             |
/// | displayMedium  | 60.0 | light   | -0.5     |             |
/// | displaySmall   | 48.0 | regular |  0.0     |             |
/// | headlineMedium | 34.0 | regular |  0.25    |             |
/// | headlineSmall  | 24.0 | regular |  0.0     |             |
/// | titleLarge     | 20.0 | medium  |  0.15    |             |
/// | titleMedium    | 16.0 | regular |  0.15    |             |
/// | titleSmall     | 14.0 | medium  |  0.1     |             |
/// | bodyLarge      | 16.0 | regular |  0.5     |             |
/// | bodyMedium     | 14.0 | regular |  0.25    |             |
/// | bodySmall      | 12.0 | regular |  0.4     |             |
/// | labelLarge     | 14.0 | medium  |  1.25    |             |
/// | labelSmall     | 10.0 | regular |  1.5     |             |
///
/// ...where "light" is `FontWeight.w300`, "regular" is `FontWeight.w400` and
/// "medium" is `FontWeight.w500`.
///
/// By default, text styles are initialized to match the 2018 Material Design
/// specification as listed above. To provide backwards compatibility, the 2014
/// specification is also available.
///
/// To explicitly configure a [Theme] for the 2018 sizes, weights, and letter
/// spacings, you can initialize its [ThemeData.typography] value using
/// [Typography.material2018]. The [Typography] constructor defaults to this
/// configuration. To configure a [Theme] for the 2014 sizes, weights, and letter
/// spacings, initialize its [ThemeData.typography] value using
/// [Typography.material2014].
///
/// See also:
///
///  * [Typography], the class that generates [TextTheme]s appropriate for a platform.
///  * [Theme], for other aspects of a Material Design application that can be
///    globally adjusted, such as the color scheme.
///  * <https://material.io/design/typography/>
@immutable
class TextTheme with Diagnosticable {
  /// Creates a text theme that uses the given values.
  ///
  /// Rather than creating a new text theme, consider using [Typography.black]
  /// or [Typography.white], which implement the typography styles in the
  /// Material Design specification:
  ///
  /// <https://material.io/design/typography/#type-scale>
  ///
  /// If you do decide to create your own text theme, consider using one of
  /// those predefined themes as a starting point for [copyWith] or [apply].
  ///
  /// The 2018 styles cannot be mixed with the 2021 styles. Only one or the
  /// other is allowed in this constructor. The 2018 styles are deprecated and
  /// will eventually be removed.
  const TextTheme({
    this.displayLarge,
    this.displayMedium,
    this.displaySmall,
    this.headlineLarge,
    this.headlineMedium,
    this.headlineSmall,
    this.titleLarge,
    this.titleMedium,
    this.titleSmall,
    this.bodyLarge,
    this.bodyMedium,
    this.bodySmall,
    this.labelLarge,
    this.labelMedium,
    this.labelSmall,
  });

  /// Largest of the display styles.
  ///
  /// As the largest text on the screen, display styles are reserved for short,
  /// important text or numerals. They work best on large screens.
  final TextStyle? displayLarge;

  /// Middle size of the display styles.
  ///
  /// As the largest text on the screen, display styles are reserved for short,
  /// important text or numerals. They work best on large screens.
  final TextStyle? displayMedium;

  /// Smallest of the display styles.
  ///
  /// As the largest text on the screen, display styles are reserved for short,
  /// important text or numerals. They work best on large screens.
  final TextStyle? displaySmall;

  /// Largest of the headline styles.
  ///
  /// Headline styles are smaller than display styles. They're best-suited for
  /// short, high-emphasis text on smaller screens.
  final TextStyle? headlineLarge;

  /// Middle size of the headline styles.
  ///
  /// Headline styles are smaller than display styles. They're best-suited for
  /// short, high-emphasis text on smaller screens.
  final TextStyle? headlineMedium;

  /// Smallest of the headline styles.
  ///
  /// Headline styles are smaller than display styles. They're best-suited for
  /// short, high-emphasis text on smaller screens.
  final TextStyle? headlineSmall;

  /// Largest of the title styles.
  ///
  /// Titles are smaller than headline styles and should be used for shorter,
  /// medium-emphasis text.
  final TextStyle? titleLarge;

  /// Middle size of the title styles.
  ///
  /// Titles are smaller than headline styles and should be used for shorter,
  /// medium-emphasis text.
  final TextStyle? titleMedium;

  /// Smallest of the title styles.
  ///
  /// Titles are smaller than headline styles and should be used for shorter,
  /// medium-emphasis text.
  final TextStyle? titleSmall;

  /// Largest of the body styles.
  ///
  /// Body styles are used for longer passages of text.
  final TextStyle? bodyLarge;

  /// Middle size of the body styles.
  ///
  /// Body styles are used for longer passages of text.
  ///
  /// The default text style for [Material].
  final TextStyle? bodyMedium;

  /// Smallest of the body styles.
  ///
  /// Body styles are used for longer passages of text.
  final TextStyle? bodySmall;

  /// Largest of the label styles.
  ///
  /// Label styles are smaller, utilitarian styles, used for areas of the UI
  /// such as text inside of components or very small supporting text in the
  /// content body, like captions.
  ///
  /// Used for text on [ElevatedButton], [TextButton] and [OutlinedButton].
  final TextStyle? labelLarge;

  /// Middle size of the label styles.
  ///
  /// Label styles are smaller, utilitarian styles, used for areas of the UI
  /// such as text inside of components or very small supporting text in the
  /// content body, like captions.
  final TextStyle? labelMedium;

  /// Smallest of the label styles.
  ///
  /// Label styles are smaller, utilitarian styles, used for areas of the UI
  /// such as text inside of components or very small supporting text in the
  /// content body, like captions.
  final TextStyle? labelSmall;

  /// Creates a copy of this text theme but with the given fields replaced with
  /// the new values.
  ///
  /// Consider using [Typography.black] or [Typography.white], which implement
  /// the typography styles in the Material Design specification, as a starting
  /// point.
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// /// A Widget that sets the ambient theme's title text color for its
  /// /// descendants, while leaving other ambient theme attributes alone.
  /// class TitleColorThemeCopy extends StatelessWidget {
  ///   const TitleColorThemeCopy({super.key, required this.titleColor, required this.child});
  ///
  ///   final Color titleColor;
  ///   final Widget child;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     final ThemeData theme = Theme.of(context);
  ///     return Theme(
  ///       data: theme.copyWith(
  ///         textTheme: theme.textTheme.copyWith(
  ///           titleLarge: theme.textTheme.titleLarge!.copyWith(
  ///             color: titleColor,
  ///           ),
  ///         ),
  ///       ),
  ///       child: child,
  ///     );
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [merge] is used instead of [copyWith] when you want to merge all
  ///    of the fields of a TextTheme instead of individual fields.
  TextTheme copyWith({
    TextStyle? displayLarge,
    TextStyle? displayMedium,
    TextStyle? displaySmall,
    TextStyle? headlineLarge,
    TextStyle? headlineMedium,
    TextStyle? headlineSmall,
    TextStyle? titleLarge,
    TextStyle? titleMedium,
    TextStyle? titleSmall,
    TextStyle? bodyLarge,
    TextStyle? bodyMedium,
    TextStyle? bodySmall,
    TextStyle? labelLarge,
    TextStyle? labelMedium,
    TextStyle? labelSmall,
  }) {
    return TextTheme(
      displayLarge: displayLarge ?? this.displayLarge,
      displayMedium: displayMedium ?? this.displayMedium,
      displaySmall: displaySmall ?? this.displaySmall,
      headlineLarge: headlineLarge ?? this.headlineLarge,
      headlineMedium: headlineMedium ?? this.headlineMedium,
      headlineSmall: headlineSmall ?? this.headlineSmall,
      titleLarge: titleLarge ?? this.titleLarge,
      titleMedium: titleMedium ?? this.titleMedium,
      titleSmall: titleSmall ?? this.titleSmall,
      bodyLarge: bodyLarge ?? this.bodyLarge,
      bodyMedium: bodyMedium ?? this.bodyMedium,
      bodySmall: bodySmall ?? this.bodySmall,
      labelLarge: labelLarge ?? this.labelLarge,
      labelMedium: labelMedium ?? this.labelMedium,
      labelSmall: labelSmall ?? this.labelSmall,
    );
  }

  /// Creates a new [TextTheme] where each text style from this object has been
  /// merged with the matching text style from the `other` object.
  ///
  /// The merging is done by calling [TextStyle.merge] on each respective pair
  /// of text styles from this and the [other] text themes and is subject to
  /// the value of [TextStyle.inherit] flag. For more details, see the
  /// documentation on [TextStyle.merge] and [TextStyle.inherit].
  ///
  /// If this theme, or the `other` theme has members that are null, then the
  /// non-null one (if any) is used. If the `other` theme is itself null, then
  /// this [TextTheme] is returned unchanged. If values in both are set, then
  /// the values are merged using [TextStyle.merge].
  ///
  /// This is particularly useful if one [TextTheme] defines one set of
  /// properties and another defines a different set, e.g. having colors
  /// defined in one text theme and font sizes in another, or when one
  /// [TextTheme] has only some fields defined, and you want to define the rest
  /// by merging it with a default theme.
  ///
  /// {@tool snippet}
  ///
  /// ```dart
  /// /// A Widget that sets the ambient theme's title text color for its
  /// /// descendants, while leaving other ambient theme attributes alone.
  /// class TitleColorTheme extends StatelessWidget {
  ///   const TitleColorTheme({super.key, required this.child, required this.titleColor});
  ///
  ///   final Color titleColor;
  ///   final Widget child;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     ThemeData theme = Theme.of(context);
  ///     // This partialTheme is incomplete: it only has the title style
  ///     // defined. Just replacing theme.textTheme with partialTheme would
  ///     // set the title, but everything else would be null. This isn't very
  ///     // useful, so merge it with the existing theme to keep all of the
  ///     // preexisting definitions for the other styles.
  ///     final TextTheme partialTheme = TextTheme(titleLarge: TextStyle(color: titleColor));
  ///     theme = theme.copyWith(textTheme: theme.textTheme.merge(partialTheme));
  ///     return Theme(data: theme, child: child);
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [copyWith] is used instead of [merge] when you wish to override
  ///    individual fields in the [TextTheme] instead of merging all of the
  ///    fields of two [TextTheme]s.
  TextTheme merge(TextTheme? other) {
    if (other == null) {
      return this;
    }
    return copyWith(
      displayLarge: displayLarge?.merge(other.displayLarge) ?? other.displayLarge,
      displayMedium: displayMedium?.merge(other.displayMedium) ?? other.displayMedium,
      displaySmall: displaySmall?.merge(other.displaySmall) ?? other.displaySmall,
      headlineLarge: headlineLarge?.merge(other.headlineLarge) ?? other.headlineLarge,
      headlineMedium: headlineMedium?.merge(other.headlineMedium) ?? other.headlineMedium,
      headlineSmall: headlineSmall?.merge(other.headlineSmall) ?? other.headlineSmall,
      titleLarge: titleLarge?.merge(other.titleLarge) ?? other.titleLarge,
      titleMedium: titleMedium?.merge(other.titleMedium) ?? other.titleMedium,
      titleSmall: titleSmall?.merge(other.titleSmall) ?? other.titleSmall,
      bodyLarge: bodyLarge?.merge(other.bodyLarge) ?? other.bodyLarge,
      bodyMedium: bodyMedium?.merge(other.bodyMedium) ?? other.bodyMedium,
      bodySmall: bodySmall?.merge(other.bodySmall) ?? other.bodySmall,
      labelLarge: labelLarge?.merge(other.labelLarge) ?? other.labelLarge,
      labelMedium: labelMedium?.merge(other.labelMedium) ?? other.labelMedium,
      labelSmall: labelSmall?.merge(other.labelSmall) ?? other.labelSmall,
    );
  }

  /// Creates a copy of this text theme but with the given field replaced in
  /// each of the individual text styles.
  ///
  /// The `displayColor` is applied to [displayLarge], [displayMedium],
  /// [displaySmall], [headlineLarge], [headlineMedium], and [bodySmall]. The
  /// `bodyColor` is applied to the remaining text styles.
  ///
  /// Consider using [Typography.black] or [Typography.white], which implement
  /// the typography styles in the Material Design specification, as a starting
  /// point.
  TextTheme apply({
    String? fontFamily,
    List<String>? fontFamilyFallback,
    String? package,
    double fontSizeFactor = 1.0,
    double fontSizeDelta = 0.0,
    double letterSpacingFactor = 1.0,
    double letterSpacingDelta = 0.0,
    double wordSpacingFactor = 1.0,
    double wordSpacingDelta = 0.0,
    double heightFactor = 1.0,
    double heightDelta = 0.0,
    Color? displayColor,
    Color? bodyColor,
    TextDecoration? decoration,
    Color? decorationColor,
    TextDecorationStyle? decorationStyle,
  }) {
    return TextTheme(
      displayLarge: displayLarge?.apply(
        color: displayColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      displayMedium: displayMedium?.apply(
        color: displayColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      displaySmall: displaySmall?.apply(
        color: displayColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      headlineLarge: headlineLarge?.apply(
        color: displayColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      headlineMedium: headlineMedium?.apply(
        color: displayColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      headlineSmall: headlineSmall?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      titleLarge: titleLarge?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      titleMedium: titleMedium?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      titleSmall: titleSmall?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      bodyLarge: bodyLarge?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      bodyMedium: bodyMedium?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      bodySmall: bodySmall?.apply(
        color: displayColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      labelLarge: labelLarge?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      labelMedium: labelMedium?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
      labelSmall: labelSmall?.apply(
        color: bodyColor,
        decoration: decoration,
        decorationColor: decorationColor,
        decorationStyle: decorationStyle,
        fontFamily: fontFamily,
        fontFamilyFallback: fontFamilyFallback,
        fontSizeFactor: fontSizeFactor,
        fontSizeDelta: fontSizeDelta,
        letterSpacingDelta: letterSpacingDelta,
        letterSpacingFactor: letterSpacingFactor,
        wordSpacingDelta: wordSpacingDelta,
        wordSpacingFactor: wordSpacingFactor,
        heightFactor: heightFactor,
        heightDelta: heightDelta,
        package: package,
      ),
    );
  }

  /// Linearly interpolate between two text themes.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static TextTheme lerp(TextTheme? a, TextTheme? b, double t) {
    if (identical(a, b) && a != null) {
      return a;
    }
    return TextTheme(
      displayLarge: TextStyle.lerp(a?.displayLarge, b?.displayLarge, t),
      displayMedium: TextStyle.lerp(a?.displayMedium, b?.displayMedium, t),
      displaySmall: TextStyle.lerp(a?.displaySmall, b?.displaySmall, t),
      headlineLarge: TextStyle.lerp(a?.headlineLarge, b?.headlineLarge, t),
      headlineMedium: TextStyle.lerp(a?.headlineMedium, b?.headlineMedium, t),
      headlineSmall: TextStyle.lerp(a?.headlineSmall, b?.headlineSmall, t),
      titleLarge: TextStyle.lerp(a?.titleLarge, b?.titleLarge, t),
      titleMedium: TextStyle.lerp(a?.titleMedium, b?.titleMedium, t),
      titleSmall: TextStyle.lerp(a?.titleSmall, b?.titleSmall, t),
      bodyLarge: TextStyle.lerp(a?.bodyLarge, b?.bodyLarge, t),
      bodyMedium: TextStyle.lerp(a?.bodyMedium, b?.bodyMedium, t),
      bodySmall: TextStyle.lerp(a?.bodySmall, b?.bodySmall, t),
      labelLarge: TextStyle.lerp(a?.labelLarge, b?.labelLarge, t),
      labelMedium: TextStyle.lerp(a?.labelMedium, b?.labelMedium, t),
      labelSmall: TextStyle.lerp(a?.labelSmall, b?.labelSmall, t),
    );
  }

  /// The [ThemeData.textTheme] property of the ambient [Theme].
  ///
  /// Equivalent to `Theme.of(context).textTheme`.
  ///
  /// See also:
  /// * [TextTheme.primaryOf], which returns the [ThemeData.primaryTextTheme] property of
  ///   the ambient [Theme] instead.
  static TextTheme of(BuildContext context) => Theme.of(context).textTheme;

  /// The [ThemeData.primaryTextTheme] property of the ambient [Theme].
  ///
  ///
  /// Equivalent to `Theme.of(context).primaryTextTheme`.
  ///
  /// See also:
  /// * [TextTheme.of], which returns the [ThemeData.textTheme] property of the ambient
  ///   [Theme] instead.
  static TextTheme primaryOf(BuildContext context) => Theme.of(context).primaryTextTheme;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is TextTheme &&
        displayLarge == other.displayLarge &&
        displayMedium == other.displayMedium &&
        displaySmall == other.displaySmall &&
        headlineLarge == other.headlineLarge &&
        headlineMedium == other.headlineMedium &&
        headlineSmall == other.headlineSmall &&
        titleLarge == other.titleLarge &&
        titleMedium == other.titleMedium &&
        titleSmall == other.titleSmall &&
        bodyLarge == other.bodyLarge &&
        bodyMedium == other.bodyMedium &&
        bodySmall == other.bodySmall &&
        labelLarge == other.labelLarge &&
        labelMedium == other.labelMedium &&
        labelSmall == other.labelSmall;
  }

  @override
  int get hashCode => Object.hash(
    displayLarge,
    displayMedium,
    displaySmall,
    headlineLarge,
    headlineMedium,
    headlineSmall,
    titleLarge,
    titleMedium,
    titleSmall,
    bodyLarge,
    bodyMedium,
    bodySmall,
    labelLarge,
    labelMedium,
    labelSmall,
  );

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    final TextTheme defaultTheme = Typography.material2018(platform: defaultTargetPlatform).black;
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'displayLarge',
        displayLarge,
        defaultValue: defaultTheme.displayLarge,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'displayMedium',
        displayMedium,
        defaultValue: defaultTheme.displayMedium,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'displaySmall',
        displaySmall,
        defaultValue: defaultTheme.displaySmall,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'headlineLarge',
        headlineLarge,
        defaultValue: defaultTheme.headlineLarge,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'headlineMedium',
        headlineMedium,
        defaultValue: defaultTheme.headlineMedium,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'headlineSmall',
        headlineSmall,
        defaultValue: defaultTheme.headlineSmall,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'titleLarge',
        titleLarge,
        defaultValue: defaultTheme.titleLarge,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'titleMedium',
        titleMedium,
        defaultValue: defaultTheme.titleMedium,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'titleSmall',
        titleSmall,
        defaultValue: defaultTheme.titleSmall,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>('bodyLarge', bodyLarge, defaultValue: defaultTheme.bodyLarge),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'bodyMedium',
        bodyMedium,
        defaultValue: defaultTheme.bodyMedium,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>('bodySmall', bodySmall, defaultValue: defaultTheme.bodySmall),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'labelLarge',
        labelLarge,
        defaultValue: defaultTheme.labelLarge,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'labelMedium',
        labelMedium,
        defaultValue: defaultTheme.labelMedium,
      ),
    );
    properties.add(
      DiagnosticsProperty<TextStyle>(
        'labelSmall',
        labelSmall,
        defaultValue: defaultTheme.labelSmall,
      ),
    );
  }
}
