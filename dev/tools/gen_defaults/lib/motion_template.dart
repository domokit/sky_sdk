// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'template.dart';
import 'token_logger.dart';

class MotionTemplate extends TokenTemplate {
  /// Since we generate the tokens dynamically, we need to store them and log
  /// them manually, instead of using [getToken].
  MotionTemplate(String blockName, String fileName, this.tokens, this.tokensLogger)
    : super(blockName, fileName, tokens);
  Map<String, dynamic> tokens;
  TokenLogger tokensLogger;

  // List of duration tokens.
  late List<MapEntry<String, dynamic>> durationTokens =
      tokens.entries
          .where((MapEntry<String, dynamic> entry) => entry.key.contains('.duration.'))
          .toList()
        ..sort(
          (MapEntry<String, dynamic> a, MapEntry<String, dynamic> b) =>
              (a.value as double).compareTo(b.value as double),
        );

  // List of easing curve tokens.
  late List<MapEntry<String, dynamic>> easingCurveTokens =
      tokens.entries
          .where((MapEntry<String, dynamic> entry) => entry.key.contains('.easing.'))
          .toList()
        ..sort(
          // Sort the legacy curves at the end of the list.
          (MapEntry<String, dynamic> a, MapEntry<String, dynamic> b) =>
              a.key.contains('legacy') ? 1 : a.key.compareTo(b.key),
        );

  String durationTokenString(String token, dynamic tokenValue) {
    tokensLogger.log(token);
    final String tokenName = token.split('.').last.replaceAll('-', '').replaceFirst('Ms', '');
    final int milliseconds = (tokenValue as double).toInt();
    return '''
  /// The $tokenName duration (${milliseconds}ms) in the Material specification.
  ///
  /// See also:
  ///
  /// * [M3 guidelines: Duration tokens](https://m3.material.io/styles/motion/easing-and-duration/tokens-specs#c009dec6-f29b-4503-b9f0-482af14a8bbd)
  /// * [M3 guidelines: Applying easing and duration](https://m3.material.io/styles/motion/easing-and-duration/applying-easing-and-duration)
  static const Duration $tokenName = Duration(milliseconds: $milliseconds);
''';
  }

  String easingCurveTokenString(String token, dynamic tokenValue) {
    tokensLogger.log(token);
    final String tokenName = token.replaceFirst('md.sys.motion.easing.', '').replaceAllMapped(
      RegExp(r'[-\.](\w)'),
      (Match match) {
        return match.group(1)!.toUpperCase();
      },
    );
    return '''
  /// The $tokenName easing curve in the Material specification.
  ///
  /// See also:
  ///
  /// * [M3 guidelines: Easing tokens](https://m3.material.io/styles/motion/easing-and-duration/tokens-specs#433b1153-2ea3-4fe2-9748-803a47bc97ee)
  /// * [M3 guidelines: Applying easing and duration](https://m3.material.io/styles/motion/easing-and-duration/applying-easing-and-duration)
  static const Curve $tokenName = $tokenValue;
''';
  }

  @override
  String generate() => '''
/// The set of durations in the Material specification.
///
/// See also:
///
/// * [M3 guidelines: Duration tokens](https://m3.material.io/styles/motion/easing-and-duration/tokens-specs#c009dec6-f29b-4503-b9f0-482af14a8bbd)
/// * [M3 guidelines: Applying easing and duration](https://m3.material.io/styles/motion/easing-and-duration/applying-easing-and-duration)
abstract final class Durations {
${durationTokens.map((MapEntry<String, dynamic> entry) => durationTokenString(entry.key, entry.value)).join('\n')}}


// TODO(guidezpl): Improve with description and assets, b/289870605

/// The set of easing curves in the Material specification.
///
/// See also:
///
/// * [M3 guidelines: Easing tokens](https://m3.material.io/styles/motion/easing-and-duration/tokens-specs#433b1153-2ea3-4fe2-9748-803a47bc97ee)
/// * [M3 guidelines: Applying easing and duration](https://m3.material.io/styles/motion/easing-and-duration/applying-easing-and-duration)
/// * [Curves], for a collection of non-Material animation easing curves.
abstract final class Easing {
${easingCurveTokens.map((MapEntry<String, dynamic> entry) => easingCurveTokenString(entry.key, entry.value)).join('\n')}}
''';
}
