// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Parses raw test data into a list of [TestCase] objects.
List<TestCase> parseRawTestData(String rawTestData, {required bool isV8}) {
  return rawTestData
      .split('\n')
      .where(isValidTestCase)
      .map((String line) => _checkReplacement(line, isV8: isV8))
      .map(_parse)
      .toList();
}

bool isValidTestCase(String line) {
  return line.startsWith('×');
}

String _checkReplacement(String line, {required bool isV8}) {
  String replacement = line;

  // Special cases for rules LB8, LB11, LB13, LB14, LB15, LB16, LB17 to allow
  // line breaks after spaces.
  final RegExp spacesRegex = RegExp(r'SPACE \(SP\) × \[(8|11|13|14|15|16|17)\.');
  if (replacement.contains(spacesRegex)) {
    replacement = replacement
        .replaceAll('0020 ×', '0020 ÷') // SPACE (SP)
        .replaceAllMapped(spacesRegex, (Match m) => 'SPACE (SP) ÷ [${m.group(1)}.');
  }

  if (!isV8) {
    // Some test cases contradict rule LB25, so we are fixing them with the few
    // regexes below.

    final RegExp lb25Regex1 = RegExp(r'\((CP_CP30|CL)\)(.*?) ÷ \[999\.0\] (PERCENT|DOLLAR)');
    if (replacement.contains(lb25Regex1)) {
      replacement = replacement
          .replaceAll(' ÷ 0024', ' × 0024') // DOLLAR SIGN (PR)
          .replaceAll(' ÷ 0025', ' × 0025') // PERCENT SIGN (PO)
          .replaceAllMapped(
            lb25Regex1,
            (Match m) => '(${m.group(1)})${m.group(2)} × [999.0] ${m.group(3)}',
          );
    }
    final RegExp lb25Regex2 = RegExp(r'\((IS|SY)\)(.*?) ÷ \[999\.0\] (DIGIT)');
    if (replacement.contains(lb25Regex2)) {
      replacement = replacement
          .replaceAll(' ÷ 0030', ' × 0030') // DIGIT ZERO (NU)
          .replaceAllMapped(
            lb25Regex2,
            (Match m) => '(${m.group(1)})${m.group(2)} × [999.0] ${m.group(3)}',
          );
    }
    final RegExp lb25Regex3 = RegExp(r'\((PR|PO)\)(.*?) ÷ \[999\.0\] (LEFT)');
    if (replacement.contains(lb25Regex3)) {
      replacement = replacement
          .replaceAll(' ÷ 0028', ' × 0028') // LEFT PARENTHESIS (OP_OP30)
          .replaceAll(' ÷ 007B', ' × 007B') // LEFT CURLY BRACKET (OP_OP30)
          .replaceAll(' ÷ 2329', ' × 2329') // LEFT-POINTING ANGLE BRACKET (OP)
          .replaceAllMapped(
            lb25Regex3,
            (Match m) => '(${m.group(1)})${m.group(2)} × [999.0] ${m.group(3)}',
          );
    }
  }

  if (isV8) {
    // v8BreakIterator deviates from the spec around Hiragana and Katakana
    // letters.

    final RegExp hiragana21Regex = RegExp(
      r' × \[21\.03\] (HIRAGANA LETTER|KATAKANA LETTER|KATAKANA-HIRAGANA)',
    );
    if (replacement.contains(hiragana21Regex) &&
        !replacement.contains('(BB)') &&
        !replacement.contains('(PR)')) {
      replacement = replacement
          .replaceAll(' × 3041', ' ÷ 3041') // HIRAGANA LETTER (CJ)
          .replaceAll(' × 30E5', ' ÷ 30E5') // KATAKANA LETTER (CJ)
          .replaceAll(' × 30FC', ' ÷ 30FC') // KATAKANA-HIRAGANA PROLONGED SOUND MARK (CJ)
          .replaceAllMapped(hiragana21Regex, (Match m) => ' ÷ [21.03] ${m.group(1)}');
    }
    if (replacement.contains(' × [16.0] HIRAGANA LETTER')) {
      replacement = replacement
          .replaceAll(' × 3041', ' ÷ 3041') // HIRAGANA LETTER (CJ)
          .replaceAll(' × [16.0] HIRAGANA LETTER', ' ÷ [16.0] HIRAGANA LETTER');
    }
    final RegExp hiraganaPercentRegex = RegExp(r'HIRAGANA .*? ÷ \[999\.0\] PERCENT');
    if (replacement.contains(hiraganaPercentRegex)) {
      replacement = replacement
          .replaceAll(' ÷ 0025', ' × 0025') // PERCENT SIGN (PO)
          .replaceAll(' ÷ [999.0] PERCENT', ' × [999.0] PERCENT');
    }

    // v8BreakIterator also deviates from the spec around hyphens, commas and
    // full stops.

    final RegExp hyphenRegex = RegExp(
      r'\((HY|IS)\)(.*?) ÷ \[999\.0\] (DIGIT|NUMBER|SECTION|THAI|<reserved-50005>)',
    );
    if (replacement.contains(hyphenRegex)) {
      replacement = replacement
          .replaceAll(' ÷ 0030', ' × 0030') // DIGIT ZERO (NU)
          .replaceAll(' ÷ 0023', ' × 0023') // NUMBER SIGN (AL)
          .replaceAll(' ÷ 00A7', ' × 00A7') // SECTION SIGN (AI_AL)
          .replaceAll(' ÷ 0E01', ' × 0E01') // THAI CHARACTER KO KAI (SA_AL)
          .replaceAll(' ÷ 50005', ' × 50005') // <reserved-50005> (XX_AL)
          .replaceAllMapped(
            hyphenRegex,
            (Match m) => '(${m.group(1)})${m.group(2)} × [999.0] ${m.group(3)}',
          );
    }
  }

  return replacement;
}

final RegExp spaceRegex = RegExp(r'\s+');
final RegExp signRegex = RegExp(r'([×÷])\s+\[(\d+\.\d+)\]\s*');
final RegExp charRegex = RegExp(
  r'([A-Z0-9-]+(?:\s+[A-Z0-9-]+)*)\s+\(([A-Z0-9_]+)\)\s*',
  caseSensitive: false,
);
final RegExp charWithBracketsRegex = RegExp(
  r'(\<[A-Z0-9()-]+(?:\s+[A-Z0-9()-]+)*\>)\s+\(([A-Z0-9_]+)\)\s*',
  caseSensitive: false,
);

TestCase _parse(String line) {
  final int hashIndex = line.indexOf('#');
  final List<String> sequence = line.substring(0, hashIndex).trim().split(spaceRegex);
  final String explanation = line.substring(hashIndex + 1).trim();

  final List<Sign> signs = <Sign>[];
  final Match signMatch = signRegex.matchAsPrefix(explanation)!;
  signs.add(Sign._(code: signMatch.group(1)!, rule: signMatch.group(2)!));

  final List<Char> chars = <Char>[];

  int i = signMatch.group(0)!.length;
  while (i < explanation.length) {
    final Match charMatch =
        explanation[i] == '<'
            ? charWithBracketsRegex.matchAsPrefix(explanation, i)!
            : charRegex.matchAsPrefix(explanation, i)!;
    final int charCode = int.parse(sequence[2 * chars.length + 1], radix: 16);
    chars.add(Char._(code: charCode, name: charMatch.group(1)!, property: charMatch.group(2)!));
    i += charMatch.group(0)!.length;

    final Match signMatch = signRegex.matchAsPrefix(explanation, i)!;
    signs.add(Sign._(code: signMatch.group(1)!, rule: signMatch.group(2)!));
    i += signMatch.group(0)!.length;
  }
  return TestCase._(signs: signs, chars: chars, raw: line);
}

/// Represents a character in a test case.
///
/// The character has a code, name and a property that determines how it behaves
/// with regards to line breaks.
class Char {
  Char._({required this.code, required this.name, required this.property});

  final int code;
  final String name;
  final String property;

  /// Whether this character is a code point that gets encoded as a UTF-16
  /// surrogate pair.
  bool get isSurrogatePair => code > 0xFFFF;
}

/// Represents a sign between two characters in a test case.
///
/// The sign could either be "×" to indicate no line break, or "÷" to indicate
/// the existence of a line break opportunity.
class Sign {
  Sign._({required this.code, required this.rule});

  final String code;
  final String rule;

  bool get isBreakOpportunity => code == '÷';
}

/// Represents an entire test case.
///
/// A test case is a sequence of characters combined with signs between them.
/// The signs indicate where line break opportunities exist.
class TestCase {
  TestCase._({required this.signs, required this.chars, required this.raw});

  final List<Sign> signs;
  final List<Char> chars;
  final String raw;

  Iterable<int> get charCodes => chars.map((Char char) => char.code);

  /// Returns the text that this test case is covering.
  String toText() {
    return String.fromCharCodes(charCodes);
  }

  @override
  String toString() {
    return raw;
  }
}
