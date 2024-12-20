// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:args/args.dart';
import 'package:path/path.dart' as path;

final ArgParser argParser =
    ArgParser()
      ..addOption('output-dir')
      ..addOption('input-dir')
      ..addFlag('ui')
      ..addFlag('public')
      ..addOption('library-name')
      ..addOption('api-file')
      ..addMultiOption('source-file')
      ..addOption('stamp')
      ..addOption('depfile')
      ..addOption('exclude-pattern')
      ..addOption('build-dir');

final List<Replacer> uiPatterns = <Replacer>[
  AllReplacer(RegExp(r'library\s+ui;'), 'library dart.ui;'),
  AllReplacer(RegExp(r'part\s+of\s+ui;'), 'part of dart.ui;'),

  // import 'src/engine.dart' as engine;
  AllReplacer(
    RegExp(r'''
import\s*'src/engine.dart'\s*as\s+engine;
'''),
    r'''
import 'dart:_engine' as engine;
''',
  ),

  // import 'ui_web/src/ui_web.dart' as ui_web;
  AllReplacer(
    RegExp(r'''
import\s*'ui_web/src/ui_web.dart'\s*as\s+ui_web;
'''),
    r'''
import 'dart:ui_web' as ui_web;
''',
  ),
];

List<Replacer> generateApiFilePatterns(
  String libraryName,
  bool isPublic,
  List<String> extraImports,
) {
  final String libraryPrefix = isPublic ? '' : '_';
  return <Replacer>[
    AllReplacer(RegExp('library\\s+$libraryName;'), '''
@JS()
library dart.$libraryPrefix$libraryName;

import 'dart:async';
import 'dart:collection';
import 'dart:convert' hide Codec;
import 'dart:developer' as developer;
import 'dart:js_util' as js_util;
import 'dart:_js_annotations';
import 'dart:js_interop' hide JS;
import 'dart:js_interop_unsafe';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;
${extraImports.join('\n')}
'''),
    // Replace exports of engine files with "part" directives.
    MappedReplacer(
      RegExp('''
export\\s*'$libraryName/(.*)';
'''),
      (Match match) => '''
part '$libraryName/${match.group(1)}';
''',
    ),
  ];
}

List<Replacer> generatePartsPatterns(String libraryName, bool isPublic) {
  final String libraryPrefix = isPublic ? '' : '_';
  return <Replacer>[
    AllReplacer(RegExp('part\\s+of\\s+$libraryName;'), 'part of dart.$libraryPrefix$libraryName;'),
    // Remove library-level JS annotations.
    AllReplacer(RegExp(r'\n@JS(.*)\nlibrary .+;'), ''),
    // Remove library directives.
    AllReplacer(RegExp(r'\nlibrary .+;'), ''),
    // Remove imports/exports from all part files.
    AllReplacer(RegExp(r"\nimport '.+'\n\s*if \(dart\.library\..*\) '.*';"), ''),
    AllReplacer(RegExp(r'\nimport\s*.*'), ''),
    AllReplacer(RegExp(r'\nexport\s*.*'), ''),
    AllReplacer(RegExp(r'\n@DefaultAsset(.*)'), ''),
  ];
}

final List<Replacer> stripMetaPatterns = <Replacer>[
  AllReplacer(RegExp(r"import\s*'package:meta/meta.dart';"), ''),
  AllReplacer('@required', ''),
  AllReplacer('@protected', ''),
  AllReplacer('@mustCallSuper', ''),
  AllReplacer('@immutable', ''),
  AllReplacer('@visibleForTesting', ''),
];

const Set<String> rootLibraryNames = <String>{'ui_web', 'engine', 'skwasm_stub', 'skwasm_impl'};

final Map<Pattern, String> extraImportsMap = <Pattern, String>{
  RegExp('skwasm_(stub|impl)'):
      "import 'dart:_skwasm_impl' if (dart.library.html) 'dart:_skwasm_stub';",
  'ui_web': "import 'dart:ui_web' as ui_web;",
  'engine': "import 'dart:_engine';",
  'web_unicode': "import 'dart:_web_unicode';",
  'web_test_fonts': "import 'dart:_web_test_fonts';",
  'web_locale_keymap': "import 'dart:_web_locale_keymap' as locale_keymap;",
};

// Rewrites the "package"-style web ui library into a dart:ui implementation.
// So far this only requires a replace of the library declarations.
void main(List<String> arguments) {
  final ArgResults results = argParser.parse(arguments);
  final Directory directory = Directory(results['output-dir'] as String);
  final String inputDirectoryPath = results['input-dir'] as String;
  final String? excludePattern = results['exclude-pattern'] as String?;
  final String stampfilePath = results['stamp'] as String;
  final String depfilePath = results['depfile'] as String;
  final String buildDirPath = results['build-dir'] as String;

  String Function(String source)? preprocessor;
  List<Replacer> replacementPatterns;
  String? libraryName;

  final bool isPublic = results['public'] as bool;

  if (results['ui'] as bool) {
    replacementPatterns = uiPatterns;
  } else {
    libraryName = results['library-name'] as String?;
    if (libraryName == null) {
      throw Exception('library-name must be specified if not rewriting ui');
    }
    preprocessor = (String source) => preprocessPartFile(source, libraryName!);
    replacementPatterns = generatePartsPatterns(libraryName, isPublic);
  }

  final List<String> inputFiles = <String>[];
  final List<FileSystemEntity> entries = Directory(
    inputDirectoryPath,
  ).listSync(recursive: true, followLinks: false);
  for (final File inputFile in entries.whereType<File>()) {
    if (excludePattern != null && inputFile.path.startsWith(excludePattern)) {
      continue;
    }
    if (!inputFile.path.endsWith('.dart') || inputFile.path.endsWith('_test.dart')) {
      continue;
    }
    inputFiles.add(path.relative(inputFile.path, from: buildDirPath));
    String pathSuffix = inputFile.path.substring(inputDirectoryPath.length);
    if (libraryName != null) {
      pathSuffix = path.join(libraryName, pathSuffix);
    }
    final String outputFilePath = path.join(directory.path, pathSuffix);
    processFile(inputFile.path, outputFilePath, preprocessor, replacementPatterns);
  }

  if (results['api-file'] != null) {
    if (libraryName == null) {
      throw Exception('library-name must be specified if api-file is specified');
    }

    final String inputFilePath = results['api-file'] as String;
    final String outputFilePath = path.join(directory.path, path.basename(inputFilePath));

    final List<String> extraImports = getExtraImportsForLibrary(libraryName);
    replacementPatterns = generateApiFilePatterns(libraryName, isPublic, extraImports);

    processFile(
      inputFilePath,
      outputFilePath,
      (String source) => validateApiFile(inputFilePath, source, libraryName!),
      replacementPatterns,
    );
  }

  File(stampfilePath).writeAsStringSync('stamp');
  writeDepfile(depfilePath, stampfilePath, inputFiles);
}

void writeDepfile(String depfilePath, String stampfilePath, List<String> inputFiles) {
  final StringBuffer outBuf = StringBuffer();
  outBuf.write('$stampfilePath: ');
  outBuf.write(inputFiles.join(' '));
  File(depfilePath).writeAsStringSync(outBuf.toString());
}

List<String> getExtraImportsForLibrary(String libraryName) {
  // Only our root libraries should have extra imports.
  if (!rootLibraryNames.contains(libraryName)) {
    return <String>[];
  }

  final List<String> extraImports = <String>[];
  for (final MapEntry<Pattern, String> entry in extraImportsMap.entries) {
    // A library shouldn't import itself.
    if (entry.key.matchAsPrefix(libraryName) == null) {
      extraImports.add(entry.value);
    }
  }
  if (libraryName == 'skwasm_impl') {
    extraImports.add("import 'dart:_wasm';");
  }
  return extraImports;
}

void processFile(
  String inputFilePath,
  String outputFilePath,
  String Function(String source)? preprocessor,
  List<Replacer> replacementPatterns,
) {
  final File inputFile = File(inputFilePath);
  final File outputFile = File(outputFilePath)..createSync(recursive: true);
  outputFile.writeAsStringSync(
    processSource(inputFile.readAsStringSync(), preprocessor, replacementPatterns),
  );
}

String processSource(
  String source,
  String Function(String source)? preprocessor,
  List<Replacer> replacementPatterns,
) {
  if (preprocessor != null) {
    source = preprocessor(source);
  }
  for (final Replacer replacer in stripMetaPatterns) {
    source = replacer.perform(source);
  }
  for (final Replacer replacer in replacementPatterns) {
    source = replacer.perform(source);
  }
  return source;
}

// Enforces a particular structure in top level api files for sublibraries.
//
// Code in api files must only be made of the library directive, exports,
// and code comments. Imports are disallowed. Instead, the required imports are
// added by this script during the rewrite.
String validateApiFile(String apiFilePath, String apiFileCode, String libraryName) {
  final List<String> expectedLines = <String>['library $libraryName;'];

  final List<String> lines = apiFileCode.split('\n');
  for (int i = 0; i < lines.length; i += 1) {
    final int lineNumber = i + 1;
    final String line = lines[i].trim();

    if (line.isEmpty) {
      // Emply lines are OK
      continue;
    }

    if (expectedLines.contains(line)) {
      // Expected; let it pass.
      continue;
    }

    if (line.startsWith('//')) {
      // Comments are OK
      continue;
    }

    if (line.startsWith('export')) {
      // Exports are OK
      continue;
    }

    if (line.startsWith('@DefaultAsset')) {
      // Default asset annotations are OK
      continue;
    }

    if (line.startsWith("import 'dart:ffi';")) {
      // dart:ffi import is an exception to the import rule, since the
      // @DefaultAsset annotation comes from dart:ffi.
      continue;
    }

    throw Exception(
      'on line $lineNumber: unexpected code in $apiFilePath. This file '
      'may only contain comments and exports. Found:\n'
      '$line',
    );
  }
  return apiFileCode;
}

String preprocessPartFile(String source, String libraryName) {
  return 'part of $libraryName;\n$source';
}

/// Responsible for performing string replacements.
abstract class Replacer {
  /// Performs the replacement in the provided [text].
  String perform(String text);
}

/// Replaces all occurrences of a pattern with a fixed string.
class AllReplacer implements Replacer {
  /// Creates a new tuple with the given [pattern] and [replacement] string.
  AllReplacer(this._pattern, this._replacement);

  /// The pattern to be replaced.
  final Pattern _pattern;

  /// The replacement string.
  final String _replacement;

  @override
  String perform(String text) {
    return text.replaceAll(_pattern, _replacement);
  }
}

/// Uses a callback to replace each occurrence of a pattern.
class MappedReplacer implements Replacer {
  MappedReplacer(this._pattern, this._replace);

  /// The pattern to be replaced.
  final RegExp _pattern;

  /// A callback to replace each occurrence of [_pattern].
  final String Function(Match match) _replace;

  @override
  String perform(String text) {
    return text.replaceAllMapped(_pattern, _replace);
  }
}
