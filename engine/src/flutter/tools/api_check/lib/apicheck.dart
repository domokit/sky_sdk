// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show LineSplitter;
import 'dart:io';

import 'package:analyzer/dart/analysis/analysis_context.dart';
import 'package:analyzer/dart/analysis/analysis_context_collection.dart';
import 'package:analyzer/dart/analysis/features.dart';
import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/analysis/session.dart';
import 'package:analyzer/dart/analysis/utilities.dart';
import 'package:analyzer/dart/ast/ast.dart';

/// Returns all indexed fields in [className].
///
/// Field names are expected to be of the form `kFooBarIndex`; prefixed with a
/// `k` and terminated in `Index`.
List<String> getDartClassFields({required String sourcePath, required String className}) {
  final List<String> includedPaths = <String>[sourcePath];
  final AnalysisContextCollection collection = AnalysisContextCollection(
    includedPaths: includedPaths,
  );
  final AnalysisContext context = collection.contextFor(sourcePath);
  final AnalysisSession session = context.currentSession;

  final SomeParsedUnitResult result = session.getParsedUnit(sourcePath);
  if (result is! ParsedUnitResult) {
    return <String>[];
  }

  // Locate all fields matching the expression in the class.
  final RegExp fieldExp = RegExp(r'_k(\w*)Index');
  final List<String> fields = <String>[];
  for (final CompilationUnitMember unitMember in result.unit.declarations) {
    if (unitMember is ClassDeclaration && unitMember.name.lexeme == className) {
      for (final ClassMember classMember in unitMember.members) {
        if (classMember is FieldDeclaration) {
          for (final VariableDeclaration field in classMember.fields.variables) {
            final String fieldName = field.name.lexeme;
            final RegExpMatch? match = fieldExp.firstMatch(fieldName);
            if (match != null) {
              fields.add(match.group(1)!);
            }
          }
        }
      }
    }
  }
  return fields;
}

/// Returns all values in [enumName].
///
/// Enum values are expected to be of the form `kEnumNameFooBar`; prefixed with
/// `kEnumName`.
List<String> getCppEnumValues({required String sourcePath, required String enumName}) {
  final List<String> lines = File(sourcePath).readAsLinesSync();
  final int enumEnd = lines.indexOf('} $enumName;');
  if (enumEnd < 0) {
    return <String>[];
  }
  final int enumStart = lines.lastIndexOf('typedef enum {', enumEnd);
  if (enumStart < 0 || enumStart >= enumEnd) {
    return <String>[];
  }
  final RegExp valueExp = RegExp('^\\s*k$enumName(\\w*)');
  return _extractMatchingExpression(lines: lines.sublist(enumStart + 1, enumEnd), regexp: valueExp);
}

/// Returns all values in [enumName].
///
/// Enum values are expected to be of the form `kFooBar`; prefixed with `k`.
List<String> getCppEnumClassValues({required String sourcePath, required String enumName}) {
  final List<String> lines = _getBlockStartingWith(
    source: File(sourcePath).readAsStringSync(),
    startExp: RegExp('enum class $enumName .* {'),
  );
  final RegExp valueExp = RegExp(r'^\s*k(\w*)');
  return _extractMatchingExpression(lines: lines, regexp: valueExp);
}

/// Returns all values in [enumName].
///
/// Enum value declarations are expected to be of the form `FOO_BAR(1 << N)`;
/// in all caps.
List<String> getJavaEnumValues({required String sourcePath, required String enumName}) {
  final List<String> lines = _getBlockStartingWith(
    source: File(sourcePath).readAsStringSync(),
    startExp: RegExp('enum $enumName {'),
  );
  final RegExp valueExp = RegExp(r'^\s*([A-Z_]*)\(');
  return _extractMatchingExpression(lines: lines, regexp: valueExp);
}

/// Returns all values in [lines] whose line of code matches [regexp].
///
/// The contents of the first match group in [regexp] is returned; therefore
/// it must contain a match group.
List<String> _extractMatchingExpression({required Iterable<String> lines, required RegExp regexp}) {
  final List<String> values = <String>[];
  for (final String line in lines) {
    final RegExpMatch? match = regexp.firstMatch(line);
    if (match != null) {
      values.add(match.group(1)!);
    }
  }
  return values;
}

/// Returns all lines of the block starting with [startString].
///
/// [startString] MUST end with '{'.
List<String> _getBlockStartingWith({required String source, required RegExp startExp}) {
  assert(startExp.pattern.endsWith('{'));

  final int blockStart = source.indexOf(startExp);
  if (blockStart < 0) {
    return <String>[];
  }
  // Find start of block.
  int pos = blockStart;
  while (pos < source.length && source[pos] != '{') {
    pos++;
  }
  int braceCount = 1;

  // Count braces until end of block.
  pos++;
  while (pos < source.length && braceCount > 0) {
    if (source[pos] == '{') {
      braceCount++;
    } else if (source[pos] == '}') {
      braceCount--;
    }
    pos++;
  }
  final int blockEnd = pos;
  return LineSplitter.split(source, blockStart, blockEnd).toList();
}

/// Apply a visitor to all compilation units in the dart:ui library.
void visitUIUnits(String flutterRoot, AstVisitor<void> visitor) {
  final String uiRoot = '$flutterRoot/lib/ui';
  final FeatureSet analyzerFeatures = FeatureSet.latestLanguageVersion();
  final ParseStringResult uiResult = parseFile(
    path: '$uiRoot/ui.dart',
    featureSet: analyzerFeatures,
  );
  for (final PartDirective part in uiResult.unit.directives.whereType<PartDirective>()) {
    final String partPath = part.uri.stringValue!;
    final ParseStringResult partResult = parseFile(
      path: '$uiRoot/$partPath',
      featureSet: analyzerFeatures,
    );

    for (final CompilationUnitMember unitMember in partResult.unit.declarations) {
      unitMember.accept(visitor);
    }
  }
}
