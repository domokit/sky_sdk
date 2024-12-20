// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// To run this, from the root of the Flutter repository:
//   bin/cache/dart-sdk/bin/dart --enable-asserts dev/bots/analyze_snippet_code.dart

// In general, please prefer using full linked examples in API docs.
//
// For documentation on creating sample code, see ../../examples/api/README.md
// See also our style guide's discussion on documentation and sample code:
// https://github.com/flutter/flutter/blob/main/docs/contributing/Style-guide-for-Flutter-repo.md
//
// This tool is used to analyze smaller snippets of code in the API docs.
// Such snippets are wrapped in ```dart ... ``` blocks, which may themselves
// be wrapped in {@tool snippet} ... {@end-tool} blocks to set them apart
// in the rendered output.
//
// Such snippets:
//
//  * If they start with `import` are treated as full application samples; avoid
//    doing this in general, it's better to use samples as described above. (One
//    exception might be in dart:ui where the sample code would end up in a
//    different repository which would be awkward.)
//
//  * If they start with a comment that says `// continuing from previous example...`,
//    they automatically import the previous test's file.
//
//  * If they start with a comment that says `// (e.g. in a stateful widget)`,
//    are analyzed after being inserted into a class that inherits from State.
//
//  * If they start with what looks like a getter, function declaration, or
//    other top-level keyword (`class`, `typedef`, etc), or if they start with
//    the keyword `final`, they are analyzed directly.
//
//  * If they end with a trailing semicolon or have a line starting with a
//    statement keyword like `while` or `try`, are analyzed after being inserted
//    into a function body.
//
//  * If they start with the word `static`, are placed in a class body before
//    analysis.
//
//  * Otherwise, are used as an initializer for a global variable for the
//    purposes of analysis; in this case, any leading label (`foo:`)
//    and any trailing comma are removed.
//
// In particular, these rules imply that starting an example with `const` means
// it is an _expression_, not a top-level declaration. This is because mostly
// `const` indicates a Widget.
//
// A line that contains just a comment with an ellipsis (`// ...`) adds an ignore
// for the `non_abstract_class_inherits_abstract_member` error for the snippet.
// This is useful when you're writing an example that extends an abstract class
// with lots of members, but you only care to show one.
//
// At the top of a file you can say `// Examples can assume:` and then list some
// commented-out declarations that will be included in the analysis for snippets
// in that file. This section may also contain explicit import statements.
//
// For files without an `// Examples can assume:` section or if that section
// contains no explicit imports, the snippets will implicitly import all the
// main Flutter packages (including material and flutter_test), as well as most
// core Dart packages with the usual prefixes.
//
// When invoked without an additional path argument, the script will analyze
// the code snippets for all packages in the "packages" subdirectory that do
// not specify "nodoc: true" in their pubspec.yaml (i.e. all packages for which
// we publish docs will have their doc code snippets analyzed).

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:args/args.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;
import 'package:watcher/watcher.dart';

final String _flutterRoot = path.dirname(path.dirname(path.dirname(path.fromUri(Platform.script))));
final String _packageFlutter = path.join(_flutterRoot, 'packages', 'flutter', 'lib');
final String _defaultDartUiLocation = path.join(
  _flutterRoot,
  'bin',
  'cache',
  'pkg',
  'sky_engine',
  'lib',
  'ui',
);
final String _flutter = path.join(
  _flutterRoot,
  'bin',
  Platform.isWindows ? 'flutter.bat' : 'flutter',
);

Future<void> main(List<String> arguments) async {
  bool asserts = false;
  assert(() {
    asserts = true;
    return true;
  }());
  if (!asserts) {
    print('You must run this script with asserts enabled.');
    exit(1);
  }
  int width;
  try {
    width = stdout.terminalColumns;
  } on StdoutException {
    width = 80;
  }
  final ArgParser argParser = ArgParser(usageLineLength: width);
  argParser.addOption(
    'temp',
    valueHelp: 'path',
    help:
        'A location where temporary files may be written. Defaults to a '
        'directory in the system temp folder. If specified, will not be '
        'automatically removed at the end of execution.',
  );
  argParser.addFlag(
    'verbose',
    negatable: false,
    help: 'Print verbose output for the analysis process.',
  );
  argParser.addOption(
    'dart-ui-location',
    defaultsTo: _defaultDartUiLocation,
    valueHelp: 'path',
    help:
        'A location where the dart:ui dart files are to be found. Defaults to '
        'the sky_engine directory installed in this flutter repo. This '
        'is typically the engine/src/flutter/lib/ui directory in an engine dev setup. '
        'Implies --include-dart-ui.',
  );
  argParser.addFlag(
    'include-dart-ui',
    defaultsTo: true,
    help: 'Includes the dart:ui code supplied by the engine in the analysis.',
  );
  argParser.addFlag('help', negatable: false, help: 'Print help for this command.');
  argParser.addOption(
    'interactive',
    abbr: 'i',
    valueHelp: 'file',
    help: 'Analyzes the snippet code in a specified file interactively.',
  );

  final ArgResults parsedArguments;
  try {
    parsedArguments = argParser.parse(arguments);
  } on FormatException catch (e) {
    print(e.message);
    print('dart --enable-asserts analyze_snippet_code.dart [options]');
    print(argParser.usage);
    exit(1);
  }

  if (parsedArguments['help'] as bool) {
    print('dart --enable-asserts analyze_snippet_code.dart [options]');
    print(argParser.usage);
    exit(0);
  }

  final List<Directory> flutterPackages;
  if (parsedArguments.rest.length == 1) {
    // Used for testing.
    flutterPackages = <Directory>[Directory(parsedArguments.rest.single)];
  } else {
    // By default analyze snippets in all packages in the packages subdirectory
    // that do not specify "nodoc: true" in their pubspec.yaml.
    flutterPackages = <Directory>[];
    final String packagesRoot = path.join(_flutterRoot, 'packages');
    for (final FileSystemEntity entity in Directory(packagesRoot).listSync()) {
      if (entity is! Directory) {
        continue;
      }
      final File pubspec = File(path.join(entity.path, 'pubspec.yaml'));
      if (!pubspec.existsSync()) {
        throw StateError("Unexpected package '${entity.path}' found in packages directory");
      }
      if (!pubspec.readAsStringSync().contains('nodoc: true')) {
        flutterPackages.add(Directory(path.join(entity.path, 'lib')));
      }
    }
    assert(flutterPackages.length >= 4);
  }

  final bool includeDartUi =
      parsedArguments.wasParsed('dart-ui-location') || parsedArguments['include-dart-ui'] as bool;
  late Directory dartUiLocation;
  if (((parsedArguments['dart-ui-location'] ?? '') as String).isNotEmpty) {
    dartUiLocation = Directory(path.absolute(parsedArguments['dart-ui-location'] as String));
  } else {
    dartUiLocation = Directory(_defaultDartUiLocation);
  }
  if (!dartUiLocation.existsSync()) {
    stderr.writeln('Unable to find dart:ui directory ${dartUiLocation.path}');
    exit(1);
  }

  if (parsedArguments['interactive'] != null) {
    await _runInteractive(
      flutterPackages: flutterPackages,
      tempDirectory: parsedArguments['temp'] as String?,
      filePath: parsedArguments['interactive'] as String,
      dartUiLocation: includeDartUi ? dartUiLocation : null,
    );
  } else {
    if (await _SnippetChecker(
      flutterPackages,
      tempDirectory: parsedArguments['temp'] as String?,
      verbose: parsedArguments['verbose'] as bool,
      dartUiLocation: includeDartUi ? dartUiLocation : null,
    ).checkSnippets()) {
      stderr.writeln(
        'See the documentation at the top of dev/bots/analyze_snippet_code.dart for details.',
      );
      exit(1);
    }
  }
}

/// A class to represent a line of input code.
@immutable
class _Line {
  const _Line({this.code = '', this.line = -1, this.indent = 0}) : generated = false;
  const _Line.generated({this.code = ''}) : line = -1, indent = 0, generated = true;

  final int line;
  final int indent;
  final String code;
  final bool generated;

  String asLocation(String filename, int column) {
    return '$filename:$line:${column + indent}';
  }

  @override
  String toString() => code;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _Line &&
        other.line == line &&
        other.indent == indent &&
        other.code == code &&
        other.generated == generated;
  }

  @override
  int get hashCode => Object.hash(line, indent, code, generated);
}

@immutable
class _ErrorBase implements Comparable<Object> {
  const _ErrorBase({this.file, this.line, this.column});
  final String? file;
  final int? line;
  final int? column;

  @override
  int compareTo(Object other) {
    if (other is _ErrorBase) {
      if (other.file != file) {
        if (other.file == null) {
          return -1;
        }
        if (file == null) {
          return 1;
        }
        return file!.compareTo(other.file!);
      }
      if (other.line != line) {
        if (other.line == null) {
          return -1;
        }
        if (line == null) {
          return 1;
        }
        return line!.compareTo(other.line!);
      }
      if (other.column != column) {
        if (other.column == null) {
          return -1;
        }
        if (column == null) {
          return 1;
        }
        return column!.compareTo(other.column!);
      }
    }
    return toString().compareTo(other.toString());
  }
}

@immutable
class _SnippetCheckerException extends _ErrorBase implements Exception {
  const _SnippetCheckerException(this.message, {super.file, super.line});
  final String message;

  @override
  String toString() {
    if (file != null || line != null) {
      final String fileStr = file == null ? '' : '$file:';
      final String lineStr = line == null ? '' : '$line:';
      return '$fileStr$lineStr $message';
    } else {
      return message;
    }
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _SnippetCheckerException &&
        other.message == message &&
        other.file == file &&
        other.line == line;
  }

  @override
  int get hashCode => Object.hash(message, file, line);
}

/// A class representing an analysis error along with the context of the error.
///
/// Changes how it converts to a string based on the source of the error.
@immutable
class _AnalysisError extends _ErrorBase {
  const _AnalysisError(String file, int line, int column, this.message, this.errorCode, this.source)
    : super(file: file, line: line, column: column);

  final String message;
  final String errorCode;
  final _Line source;

  @override
  String toString() {
    return '${source.asLocation(file!, column!)}: $message ($errorCode)';
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _AnalysisError &&
        other.file == file &&
        other.line == line &&
        other.column == column &&
        other.message == message &&
        other.errorCode == errorCode &&
        other.source == source;
  }

  @override
  int get hashCode => Object.hash(file, line, column, message, errorCode, source);
}

/// Checks code snippets for analysis errors.
///
/// Extracts dartdoc content from flutter package source code, identifies code
/// sections, and writes them to a temporary directory, where 'flutter analyze'
/// is used to analyze the sources for problems. If problems are found, the
/// error output from the analyzer is parsed for details, and the problem
/// locations are translated back to the source location.
class _SnippetChecker {
  /// Creates a [_SnippetChecker].
  ///
  /// The positional argument is the path to the package directory for the
  /// flutter package within the Flutter root dir.
  ///
  /// The optional `tempDirectory` argument supplies the location for the
  /// temporary files to be written and analyzed. If not supplied, it defaults
  /// to a system generated temp directory.
  ///
  /// The optional `verbose` argument indicates whether or not status output
  /// should be emitted while doing the check.
  ///
  /// The optional `dartUiLocation` argument indicates the location of the
  /// `dart:ui` code to be analyzed along with the framework code. If not
  /// supplied, the default location of the `dart:ui` code in the Flutter
  /// repository is used (i.e. "<flutter repo>/bin/cache/pkg/sky_engine/lib/ui").
  _SnippetChecker(
    this._flutterPackages, {
    String? tempDirectory,
    this.verbose = false,
    Directory? dartUiLocation,
  }) : _tempDirectory = _createTempDirectory(tempDirectory),
       _keepTmp = tempDirectory != null,
       _dartUiLocation = dartUiLocation;

  /// The prefix of each comment line
  static const String _dartDocPrefix = '///';

  /// The prefix of each comment line with a space appended.
  static const String _dartDocPrefixWithSpace = '$_dartDocPrefix ';

  /// A RegExp that matches the beginning of a dartdoc snippet.
  static final RegExp _dartDocSnippetBeginRegex = RegExp(r'{@tool ([^ }]+)(?:| ([^}]*))}');

  /// A RegExp that matches the end of a dartdoc snippet.
  static final RegExp _dartDocSnippetEndRegex = RegExp(r'{@end-tool}');

  /// A RegExp that matches the start of a code block within dartdoc.
  static final RegExp _codeBlockStartRegex = RegExp(r'^ */// *```dart$');

  /// A RegExp that matches the start of a code block within a regular comment.
  /// Such blocks are not analyzed. They can be used to give sample code for
  /// internal (private) APIs where visibility would make analyzing the sample
  /// code problematic.
  static final RegExp _uncheckedCodeBlockStartRegex = RegExp(r'^ *// *```dart$');

  /// A RegExp that matches the end of a code block within dartdoc.
  static final RegExp _codeBlockEndRegex = RegExp(r'^ */// *``` *$');

  /// A RegExp that matches a line starting with a comment or annotation
  static final RegExp _nonCodeRegExp = RegExp(r'^ *(//|@)');

  /// A RegExp that matches things that look like a function declaration.
  static final RegExp _maybeFunctionDeclarationRegExp = RegExp(
    r'^([A-Z][A-Za-z0-9_<>, ?]*|int|double|num|bool|void)\?? (_?[a-z][A-Za-z0-9_<>]*)\(.*',
  );

  /// A RegExp that matches things that look like a getter.
  static final RegExp _maybeGetterDeclarationRegExp = RegExp(
    r'^([A-Z][A-Za-z0-9_<>?]*|int|double|num|bool)\?? get (_?[a-z][A-Za-z0-9_<>]*) (?:=>|{).*',
  );

  /// A RegExp that matches an identifier followed by a colon, potentially with two spaces of indent.
  static final RegExp _namedArgumentRegExp = RegExp(r'^(?:  )?([a-zA-Z0-9_]+): ');

  /// A RegExp that matches things that look unambiguously like top-level declarations.
  static final RegExp _topLevelDeclarationRegExp = RegExp(
    r'^(abstract|class|mixin|enum|typedef|final|extension) ',
  );

  /// A RegExp that matches things that look unambiguously like statements.
  static final RegExp _statementRegExp = RegExp(r'^(if|while|for|try) ');

  /// A RegExp that matches things that look unambiguously like declarations that must be in a class.
  static final RegExp _classDeclarationRegExp = RegExp(r'^(static) ');

  /// A RegExp that matches a line that ends with a comma (and maybe a comment)
  static final RegExp _trailingCommaRegExp = RegExp(r'^(.*),(| *//.*)$');

  /// A RegExp that matches a line that ends with a semicolon (and maybe a comment)
  static final RegExp _trailingSemicolonRegExp = RegExp(r'^(.*);(| *//.*)$');

  /// A RegExp that matches a line that ends with a closing brace (and maybe a comment)
  static final RegExp _trailingCloseBraceRegExp = RegExp(r'^(.*)}(| *//.*)$');

  /// A RegExp that matches a line that only contains a commented-out ellipsis
  /// (and maybe whitespace). Has three groups: before, ellipsis, after.
  static final RegExp _ellipsisRegExp = RegExp(r'^( *)(// \.\.\.)( *)$');

  /// Whether or not to print verbose output.
  final bool verbose;

  /// Whether or not to keep the temp directory around after running.
  ///
  /// Defaults to false.
  final bool _keepTmp;

  /// The temporary directory where all output is written. This will be deleted
  /// automatically if there are no errors unless _keepTmp is true.
  final Directory _tempDirectory;

  /// The package directories within the flutter root dir that will be checked.
  final List<Directory> _flutterPackages;

  /// The directory for the dart:ui code to be analyzed with the flutter code.
  ///
  /// If this is null, then no dart:ui code is included in the analysis. It
  /// defaults to the location inside of the flutter bin/cache directory that
  /// contains the dart:ui code supplied by the engine.
  final Directory? _dartUiLocation;

  static List<File> _listDartFiles(Directory directory, {bool recursive = false}) {
    return directory
        .listSync(recursive: recursive, followLinks: false)
        .whereType<File>()
        .where((File file) => path.extension(file.path) == '.dart')
        .toList();
  }

  static const List<String> ignoresDirectives = <String>[
    '// ignore_for_file: directives_ordering',
    '// ignore_for_file: duplicate_ignore',
    '// ignore_for_file: no_leading_underscores_for_local_identifiers',
    '// ignore_for_file: prefer_final_locals',
    '// ignore_for_file: unnecessary_import',
    '// ignore_for_file: unreachable_from_main',
    '// ignore_for_file: unused_element',
    '// ignore_for_file: unused_element_parameter',
    '// ignore_for_file: unused_local_variable',
  ];

  /// Computes the headers needed for each snippet file.
  List<_Line> get headersWithoutImports {
    return _headersWithoutImports ??=
        ignoresDirectives.map<_Line>((String code) => _Line.generated(code: code)).toList();
  }

  List<_Line>? _headersWithoutImports;

  /// Computes the headers needed for each snippet file.
  List<_Line> get headersWithImports {
    return _headersWithImports ??=
        <String>[
          ...ignoresDirectives,
          '// ignore_for_file: unused_import',
          "import 'dart:async';",
          "import 'dart:convert';",
          "import 'dart:io';",
          "import 'dart:math' as math;",
          "import 'dart:typed_data';",
          "import 'dart:ui' as ui;",
          "import 'package:flutter_test/flutter_test.dart';",
          for (final File file in _listDartFiles(Directory(_packageFlutter)))
            "import 'package:flutter/${path.basename(file.path)}';",
        ].map<_Line>((String code) => _Line.generated(code: code)).toList();
  }

  List<_Line>? _headersWithImports;

  /// Checks all the snippets in the Dart files in [_flutterPackage] for errors.
  /// Returns true if any errors are found, false otherwise.
  Future<bool> checkSnippets() async {
    final Map<String, _SnippetFile> snippets = <String, _SnippetFile>{};
    if (_dartUiLocation != null && !_dartUiLocation.existsSync()) {
      stderr.writeln('Unable to analyze engine dart snippets at ${_dartUiLocation.path}.');
    }
    final List<File> filesToAnalyze = <File>[
      for (final Directory flutterPackage in _flutterPackages)
        ..._listDartFiles(flutterPackage, recursive: true),
      if (_dartUiLocation != null && _dartUiLocation.existsSync())
        ..._listDartFiles(_dartUiLocation, recursive: true),
    ];
    final Set<Object> errors = <Object>{};
    errors.addAll(await _extractSnippets(filesToAnalyze, snippetMap: snippets));
    errors.addAll(_analyze(snippets));
    (errors.toList()..sort()).map(_stringify).forEach(stderr.writeln);
    stderr.writeln('Found ${errors.length} snippet code errors.');
    cleanupTempDirectory();
    return errors.isNotEmpty;
  }

  static Directory _createTempDirectory(String? tempArg) {
    if (tempArg != null) {
      final Directory tempDirectory = Directory(
        path.join(Directory.systemTemp.absolute.path, path.basename(tempArg)),
      );
      if (path.basename(tempArg) != tempArg) {
        stderr.writeln(
          'Supplied temporary directory name should be a name, not a path. Using ${tempDirectory.absolute.path} instead.',
        );
      }
      print('Leaving temporary output in ${tempDirectory.absolute.path}.');
      // Make sure that any directory left around from a previous run is cleared out.
      if (tempDirectory.existsSync()) {
        tempDirectory.deleteSync(recursive: true);
      }
      tempDirectory.createSync();
      return tempDirectory;
    }
    return Directory.systemTemp.createTempSync('flutter_analyze_snippet_code.');
  }

  void recreateTempDirectory() {
    _tempDirectory.deleteSync(recursive: true);
    _tempDirectory.createSync();
  }

  void cleanupTempDirectory() {
    if (_keepTmp) {
      print('Leaving temporary directory ${_tempDirectory.path} around for your perusal.');
    } else {
      try {
        _tempDirectory.deleteSync(recursive: true);
      } on FileSystemException catch (e) {
        stderr.writeln('Failed to delete ${_tempDirectory.path}: $e');
      }
    }
  }

  /// Creates a name for the snippets tool to use for the snippet ID from a
  /// filename and starting line number.
  String _createNameFromSource(String prefix, String filename, int start) {
    String snippetId = path.split(filename).join('.');
    snippetId = path.basenameWithoutExtension(snippetId);
    snippetId = '$prefix.$snippetId.$start';
    return snippetId;
  }

  /// Extracts the snippets from the Dart files in [files], writes them
  /// to disk, and adds them to the [snippetMap].
  Future<List<Object>> _extractSnippets(
    List<File> files, {
    required Map<String, _SnippetFile> snippetMap,
  }) async {
    final List<Object> errors = <Object>[];
    _SnippetFile? lastExample;
    for (final File file in files) {
      try {
        final String relativeFilePath = path.relative(file.path, from: _flutterRoot);
        final List<String> fileLines = file.readAsLinesSync();
        final List<_Line> ignorePreambleLinesOnly = <_Line>[];
        final List<_Line> preambleLines = <_Line>[];
        final List<_Line> customImports = <_Line>[];
        bool inExamplesCanAssumePreamble =
            false; // Whether or not we're in the file-wide preamble section ("Examples can assume").
        bool inToolSection = false; // Whether or not we're in a code snippet
        bool inDartSection = false; // Whether or not we're in a '```dart' segment.
        bool inOtherBlock = false; // Whether we're in some other '```' segment.
        int lineNumber = 0;
        final List<String> block = <String>[];
        late _Line startLine;
        for (final String line in fileLines) {
          lineNumber += 1;
          final String trimmedLine = line.trim();
          if (inExamplesCanAssumePreamble) {
            if (line.isEmpty) {
              // end of preamble
              inExamplesCanAssumePreamble = false;
            } else if (!line.startsWith('// ')) {
              throw _SnippetCheckerException(
                'Unexpected content in snippet code preamble.',
                file: relativeFilePath,
                line: lineNumber,
              );
            } else {
              final _Line newLine = _Line(line: lineNumber, indent: 3, code: line.substring(3));
              if (newLine.code.startsWith('import ')) {
                customImports.add(newLine);
              } else {
                preambleLines.add(newLine);
              }
              if (line.startsWith('// // ignore_for_file: ')) {
                ignorePreambleLinesOnly.add(newLine);
              }
            }
          } else if (trimmedLine.startsWith(_dartDocSnippetEndRegex)) {
            if (!inToolSection) {
              throw _SnippetCheckerException(
                '{@tool-end} marker detected without matching {@tool}.',
                file: relativeFilePath,
                line: lineNumber,
              );
            }
            if (inDartSection) {
              throw _SnippetCheckerException(
                "Dart section didn't terminate before end of snippet",
                file: relativeFilePath,
                line: lineNumber,
              );
            }
            inToolSection = false;
          } else if (inDartSection) {
            final RegExpMatch? snippetMatch = _dartDocSnippetBeginRegex.firstMatch(trimmedLine);
            if (snippetMatch != null) {
              throw _SnippetCheckerException(
                '{@tool} found inside Dart section',
                file: relativeFilePath,
                line: lineNumber,
              );
            }
            if (trimmedLine.startsWith(_codeBlockEndRegex)) {
              inDartSection = false;
              final _SnippetFile snippet = _processBlock(
                startLine,
                block,
                preambleLines,
                ignorePreambleLinesOnly,
                relativeFilePath,
                lastExample,
                customImports,
              );
              final String path = _writeSnippetFile(snippet).path;
              assert(!snippetMap.containsKey(path));
              snippetMap[path] = snippet;
              block.clear();
              lastExample = snippet;
            } else if (trimmedLine == _dartDocPrefix) {
              block.add('');
            } else {
              final int index = line.indexOf(_dartDocPrefixWithSpace);
              if (index < 0) {
                throw _SnippetCheckerException(
                  'Dart section inexplicably did not contain "$_dartDocPrefixWithSpace" prefix.',
                  file: relativeFilePath,
                  line: lineNumber,
                );
              }
              block.add(line.substring(index + 4));
            }
          } else if (trimmedLine.startsWith(_codeBlockStartRegex)) {
            if (inOtherBlock) {
              throw _SnippetCheckerException(
                'Found "```dart" section in another "```" section.',
                file: relativeFilePath,
                line: lineNumber,
              );
            }
            assert(block.isEmpty);
            startLine = _Line(
              line: lineNumber + 1,
              indent: line.indexOf(_dartDocPrefixWithSpace) + _dartDocPrefixWithSpace.length,
            );
            inDartSection = true;
          } else if (line.contains('```')) {
            if (inOtherBlock) {
              inOtherBlock = false;
            } else if (line.contains('```yaml') ||
                line.contains('```ascii') ||
                line.contains('```java') ||
                line.contains('```objectivec') ||
                line.contains('```kotlin') ||
                line.contains('```swift') ||
                line.contains('```glsl') ||
                line.contains('```json') ||
                line.contains('```csv') ||
                line.contains('```sh')) {
              inOtherBlock = true;
            } else if (line.startsWith(_uncheckedCodeBlockStartRegex)) {
              // this is an intentionally-unchecked block that doesn't appear in the API docs.
              inOtherBlock = true;
            } else {
              throw _SnippetCheckerException(
                'Found "```" in code but it did not match $_codeBlockStartRegex so something is wrong. Line was: "$line"',
                file: relativeFilePath,
                line: lineNumber,
              );
            }
          } else if (!inToolSection) {
            final RegExpMatch? snippetMatch = _dartDocSnippetBeginRegex.firstMatch(trimmedLine);
            if (snippetMatch != null) {
              inToolSection = true;
            } else if (line == '// Examples can assume:') {
              if (inToolSection || inDartSection) {
                throw _SnippetCheckerException(
                  '"// Examples can assume:" sections must come before all sample code.',
                  file: relativeFilePath,
                  line: lineNumber,
                );
              }
              inExamplesCanAssumePreamble = true;
            }
          }
        }
      } on _SnippetCheckerException catch (e) {
        errors.add(e);
      }
    }
    return errors;
  }

  /// Process one block of snippet code (the part inside of "```" markers). Uses
  /// a primitive heuristic to make snippet blocks into valid Dart code.
  ///
  /// `block` argument will get mutated, but is copied before this function returns.
  _SnippetFile _processBlock(
    _Line startingLine,
    List<String> block,
    List<_Line> assumptions,
    List<_Line> ignoreAssumptionsOnly,
    String filename,
    _SnippetFile? lastExample,
    List<_Line> customImports,
  ) {
    if (block.isEmpty) {
      throw _SnippetCheckerException(
        '${startingLine.asLocation(filename, 0)}: Empty ```dart block in snippet code.',
      );
    }
    bool hasEllipsis = false;
    for (int index = 0; index < block.length; index += 1) {
      final Match? match = _ellipsisRegExp.matchAsPrefix(block[index]);
      if (match != null) {
        hasEllipsis =
            true; // in case the "..." is implying some overridden members, add an ignore to silence relevant warnings
        break;
      }
    }
    bool hasStatefulWidgetComment = false;
    bool importPreviousExample = false;
    int index = startingLine.line;
    for (final String line in block) {
      if (line == '// (e.g. in a stateful widget)') {
        if (hasStatefulWidgetComment) {
          throw _SnippetCheckerException(
            'Example says it is in a stateful widget twice.',
            file: filename,
            line: index,
          );
        }
        hasStatefulWidgetComment = true;
      } else if (line == '// continuing from previous example...') {
        if (importPreviousExample) {
          throw _SnippetCheckerException(
            'Example says it continues from the previous example twice.',
            file: filename,
            line: index,
          );
        }
        if (lastExample == null) {
          throw _SnippetCheckerException(
            'Example says it continues from the previous example but it is the first example in the file.',
            file: filename,
            line: index,
          );
        }
        importPreviousExample = true;
      } else {
        break;
      }
      index += 1;
    }
    final List<_Line> preamble;
    if (importPreviousExample) {
      preamble = <_Line>[
        ...lastExample!.code, // includes assumptions
        if (hasEllipsis || hasStatefulWidgetComment)
          const _Line.generated(
            code: '// ignore_for_file: non_abstract_class_inherits_abstract_member',
          ),
      ];
    } else {
      preamble = <_Line>[
        if (hasEllipsis || hasStatefulWidgetComment)
          const _Line.generated(
            code: '// ignore_for_file: non_abstract_class_inherits_abstract_member',
          ),
        ...assumptions,
      ];
    }
    final String firstCodeLine =
        block.firstWhere((String line) => !line.startsWith(_nonCodeRegExp)).trim();
    final String lastCodeLine =
        block.lastWhere((String line) => !line.startsWith(_nonCodeRegExp)).trim();
    if (firstCodeLine.startsWith('import ')) {
      // probably an entire program
      if (importPreviousExample) {
        throw _SnippetCheckerException(
          'An example cannot both be self-contained (with its own imports) and say it wants to import the previous example.',
          file: filename,
          line: startingLine.line,
        );
      }
      if (hasStatefulWidgetComment) {
        throw _SnippetCheckerException(
          'An example cannot both be self-contained (with its own imports) and say it is in a stateful widget.',
          file: filename,
          line: startingLine.line,
        );
      }
      return _SnippetFile.fromStrings(
        startingLine,
        block.toList(),
        headersWithoutImports,
        <_Line>[
          ...ignoreAssumptionsOnly,
          if (hasEllipsis)
            const _Line.generated(
              code: '// ignore_for_file: non_abstract_class_inherits_abstract_member',
            ),
        ],
        'self-contained program',
        filename,
      );
    }

    final List<_Line> headers = switch ((importPreviousExample, customImports.length)) {
      (true, _) => <_Line>[],
      (false, 0) => headersWithImports,
      (false, _) => <_Line>[
        ...headersWithoutImports,
        const _Line.generated(code: '// ignore_for_file: unused_import'),
        ...customImports,
      ],
    };
    if (hasStatefulWidgetComment) {
      return _SnippetFile.fromStrings(
        startingLine,
        prefix: 'class _State extends State<StatefulWidget> {',
        block.toList(),
        postfix: '}',
        headers,
        preamble,
        'stateful widget',
        filename,
      );
    } else if (firstCodeLine.startsWith(_maybeGetterDeclarationRegExp) ||
        (firstCodeLine.startsWith(_maybeFunctionDeclarationRegExp) &&
            lastCodeLine.startsWith(_trailingCloseBraceRegExp)) ||
        block.any((String line) => line.startsWith(_topLevelDeclarationRegExp))) {
      // probably a top-level declaration
      return _SnippetFile.fromStrings(
        startingLine,
        block.toList(),
        headers,
        preamble,
        'top-level declaration',
        filename,
      );
    } else if (lastCodeLine.startsWith(_trailingSemicolonRegExp) ||
        block.any((String line) => line.startsWith(_statementRegExp))) {
      // probably a statement
      return _SnippetFile.fromStrings(
        startingLine,
        prefix: 'Future<void> function() async {',
        block.toList(),
        postfix: '}',
        headers,
        preamble,
        'statement',
        filename,
      );
    } else if (firstCodeLine.startsWith(_classDeclarationRegExp)) {
      // probably a static method
      return _SnippetFile.fromStrings(
        startingLine,
        prefix: 'class Class {',
        block.toList(),
        postfix: '}',
        headers,
        <_Line>[
          ...preamble,
          const _Line.generated(code: '// ignore_for_file: avoid_classes_with_only_static_members'),
        ],
        'class declaration',
        filename,
      );
    } else {
      // probably an expression
      if (firstCodeLine.startsWith(_namedArgumentRegExp)) {
        // This is for snippets like:
        //
        // ```dart
        // // bla bla
        // foo: 2,
        // ```
        //
        // This section removes the label.
        for (int index = 0; index < block.length; index += 1) {
          final Match? prefix = _namedArgumentRegExp.matchAsPrefix(block[index]);
          if (prefix != null) {
            block[index] = block[index].substring(prefix.group(0)!.length);
            break;
          }
        }
      }
      // strip trailing comma, if any
      for (int index = block.length - 1; index >= 0; index -= 1) {
        if (!block[index].startsWith(_nonCodeRegExp)) {
          final Match? lastLine = _trailingCommaRegExp.matchAsPrefix(block[index]);
          if (lastLine != null) {
            block[index] = lastLine.group(1)! + lastLine.group(2)!;
          }
          break;
        }
      }
      return _SnippetFile.fromStrings(
        startingLine,
        prefix: 'dynamic expression = ',
        block.toList(),
        postfix: ';',
        headers,
        preamble,
        'expression',
        filename,
      );
    }
  }

  /// Creates the configuration files necessary for the analyzer to consider
  /// the temporary directory a package, and sets which lint rules to enforce.
  void _createConfigurationFiles() {
    final File targetPubSpec = File(path.join(_tempDirectory.path, 'pubspec.yaml'));
    if (!targetPubSpec.existsSync()) {
      // Copying pubspec.yaml from examples/api into temp directory.
      final File sourcePubSpec = File(path.join(_flutterRoot, 'examples', 'api', 'pubspec.yaml'));
      if (!sourcePubSpec.existsSync()) {
        throw 'Cannot find pubspec.yaml at ${sourcePubSpec.path}, which is also used to analyze code snippets.';
      }
      sourcePubSpec.copySync(targetPubSpec.path);
    }
    final File targetAnalysisOptions = File(
      path.join(_tempDirectory.path, 'analysis_options.yaml'),
    );
    if (!targetAnalysisOptions.existsSync()) {
      // Use the same analysis_options.yaml configuration that's used for examples/api.
      final File sourceAnalysisOptions = File(
        path.join(_flutterRoot, 'examples', 'api', 'analysis_options.yaml'),
      );
      if (!sourceAnalysisOptions.existsSync()) {
        throw 'Cannot find analysis_options.yaml at ${sourceAnalysisOptions.path}, which is also used to analyze code snippets.';
      }
      targetAnalysisOptions
        ..createSync(recursive: true)
        ..writeAsStringSync('include: ${sourceAnalysisOptions.absolute.path}');
    }
  }

  /// Writes out a snippet section to the disk and returns the file.
  File _writeSnippetFile(_SnippetFile snippetFile) {
    final String snippetFileId = _createNameFromSource(
      'snippet',
      snippetFile.filename,
      snippetFile.indexLine,
    );
    final File outputFile = File(path.join(_tempDirectory.path, '$snippetFileId.dart'))
      ..createSync(recursive: true);
    final String contents =
        snippetFile.code.map<String>((_Line line) => line.code).join('\n').trimRight();
    outputFile.writeAsStringSync('$contents\n');
    return outputFile;
  }

  /// Starts the analysis phase of checking the snippets by invoking the analyzer
  /// and parsing its output. Returns the errors, if any.
  List<Object> _analyze(Map<String, _SnippetFile> snippets) {
    final List<String> analyzerOutput = _runAnalyzer();
    final List<Object> errors = <Object>[];
    final String kBullet = Platform.isWindows ? ' - ' : ' • ';
    // RegExp to match an error output line of the analyzer.
    final RegExp errorPattern = RegExp(
      '^ *(?<type>[a-z]+)'
      '$kBullet(?<description>.+)'
      '$kBullet(?<file>.+):(?<line>[0-9]+):(?<column>[0-9]+)'
      '$kBullet(?<code>[-a-z_]+)\$',
      caseSensitive: false,
    );

    for (final String error in analyzerOutput) {
      final RegExpMatch? match = errorPattern.firstMatch(error);
      if (match == null) {
        errors.add(_SnippetCheckerException('Could not parse analyzer output: $error'));
        continue;
      }
      final String message = match.namedGroup('description')!;
      final File file = File(path.join(_tempDirectory.path, match.namedGroup('file')));
      final List<String> fileContents = file.readAsLinesSync();
      final String lineString = match.namedGroup('line')!;
      final String columnString = match.namedGroup('column')!;
      final String errorCode = match.namedGroup('code')!;
      final int lineNumber = int.parse(lineString, radix: 10);
      final int columnNumber = int.parse(columnString, radix: 10);

      if (lineNumber < 1 || lineNumber > fileContents.length + 1) {
        errors.add(
          _AnalysisError(
            file.path,
            lineNumber,
            columnNumber,
            message,
            errorCode,
            _Line(line: lineNumber),
          ),
        );
        errors.add(
          _SnippetCheckerException(
            'Error message points to non-existent line number: $error',
            file: file.path,
            line: lineNumber,
          ),
        );
        continue;
      }

      final _SnippetFile? snippet = snippets[file.path];
      if (snippet == null) {
        errors.add(
          _SnippetCheckerException(
            "Unknown section for ${file.path}. Maybe the temporary directory wasn't empty?",
            file: file.path,
            line: lineNumber,
          ),
        );
        continue;
      }
      if (fileContents.length != snippet.code.length) {
        errors.add(
          _SnippetCheckerException(
            'Unexpected file contents for ${file.path}. File has ${fileContents.length} lines but we generated ${snippet.code.length} lines:\n${snippet.code.join("\n")}',
            file: file.path,
            line: lineNumber,
          ),
        );
        continue;
      }

      late final _Line actualSource;
      late final int actualLine;
      late final int actualColumn;
      late final String actualMessage;
      int delta = 0;
      while (true) {
        // find the nearest non-generated line to the error
        if ((lineNumber - delta > 0) &&
            (lineNumber - delta <= snippet.code.length) &&
            !snippet.code[lineNumber - delta - 1].generated) {
          actualSource = snippet.code[lineNumber - delta - 1];
          actualLine = actualSource.line;
          actualColumn = delta == 0 ? columnNumber : actualSource.code.length + 1;
          actualMessage = delta == 0 ? message : '$message -- in later generated code';
          break;
        }
        if ((lineNumber + delta < snippet.code.length) &&
            (lineNumber + delta >= 0) &&
            !snippet.code[lineNumber + delta].generated) {
          actualSource = snippet.code[lineNumber + delta];
          actualLine = actualSource.line;
          actualColumn = 1;
          actualMessage = '$message -- in earlier generated code';
          break;
        }
        delta += 1;
        assert((lineNumber - delta > 0) || (lineNumber + delta < snippet.code.length));
      }
      errors.add(
        _AnalysisError(
          snippet.filename,
          actualLine,
          actualColumn,
          '$actualMessage (${snippet.generatorComment})',
          errorCode,
          actualSource,
        ),
      );
    }
    return errors;
  }

  /// Invokes the analyzer on the given [directory] and returns the stdout (with some lines filtered).
  List<String> _runAnalyzer() {
    _createConfigurationFiles();
    // Run pub get to avoid output from getting dependencies in the analyzer
    // output.
    Process.runSync(_flutter, <String>[
      'pub',
      'get',
    ], workingDirectory: _tempDirectory.absolute.path);
    final ProcessResult result = Process.runSync(_flutter, <String>[
      '--no-wrap',
      'analyze',
      '--no-preamble',
      '--no-congratulate',
      '.',
    ], workingDirectory: _tempDirectory.absolute.path);
    final List<String> stderr = result.stderr.toString().trim().split('\n');
    final List<String> stdout = result.stdout.toString().trim().split('\n');
    // Remove output from building the flutter tool.
    stderr.removeWhere((String line) {
      return line.startsWith('Building flutter tool...') ||
          line.startsWith('Waiting for another flutter command to release the startup lock...') ||
          line.startsWith('Flutter assets will be downloaded from ');
    });
    // Check out the stderr to see if the analyzer had it's own issues.
    if (stderr.isNotEmpty && stderr.first.contains(RegExp(r' issues? found\. \(ran in '))) {
      stderr.removeAt(0);
      if (stderr.isNotEmpty && stderr.last.isEmpty) {
        stderr.removeLast();
      }
    }
    if (stderr.isNotEmpty && stderr.any((String line) => line.isNotEmpty)) {
      throw _SnippetCheckerException('Cannot analyze dartdocs; unexpected error output:\n$stderr');
    }
    if (stdout.isNotEmpty && stdout.first == 'Building flutter tool...') {
      stdout.removeAt(0);
    }
    if (stdout.isNotEmpty && stdout.first.isEmpty) {
      stdout.removeAt(0);
    }
    return stdout;
  }
}

/// A class to represent a section of snippet code, marked by "```dart ... ```", that ends up
/// in a file we then analyze (each snippet is in its own file).
class _SnippetFile {
  const _SnippetFile(this.code, this.generatorComment, this.filename, this.indexLine);

  factory _SnippetFile.fromLines(
    List<_Line> code,
    List<_Line> headers,
    List<_Line> preamble,
    String generatorComment,
    String filename,
  ) {
    while (code.isNotEmpty && code.last.code.isEmpty) {
      code.removeLast();
    }
    assert(code.isNotEmpty);
    final _Line firstLine = code.firstWhere((_Line line) => !line.generated);
    return _SnippetFile(
      <_Line>[
        ...headers,
        const _Line.generated(), // blank line
        if (preamble.isNotEmpty) ...preamble,
        if (preamble.isNotEmpty) const _Line.generated(), // blank line
        _Line.generated(code: '// From: $filename:${firstLine.line}'),
        ...code,
      ],
      generatorComment,
      filename,
      firstLine.line,
    );
  }

  factory _SnippetFile.fromStrings(
    _Line firstLine,
    List<String> code,
    List<_Line> headers,
    List<_Line> preamble,
    String generatorComment,
    String filename, {
    String? prefix,
    String? postfix,
  }) {
    final List<_Line> codeLines = <_Line>[
      if (prefix != null) _Line.generated(code: prefix),
      for (int i = 0; i < code.length; i += 1)
        _Line(code: code[i], line: firstLine.line + i, indent: firstLine.indent),
      if (postfix != null) _Line.generated(code: postfix),
    ];
    return _SnippetFile.fromLines(codeLines, headers, preamble, generatorComment, filename);
  }

  final List<_Line> code;
  final String generatorComment;
  final String filename;
  final int indexLine;
}

Future<void> _runInteractive({
  required String? tempDirectory,
  required List<Directory> flutterPackages,
  required String filePath,
  required Directory? dartUiLocation,
}) async {
  filePath = path.isAbsolute(filePath) ? filePath : path.join(path.current, filePath);
  final File file = File(filePath);
  if (!file.existsSync()) {
    stderr.writeln('Specified file ${file.absolute.path} does not exist or is not a file.');
    exit(1);
  }
  if (!path.isWithin(_flutterRoot, file.absolute.path) &&
      (dartUiLocation == null || !path.isWithin(dartUiLocation.path, file.absolute.path))) {
    stderr.writeln(
      'Specified file ${file.absolute.path} is not within the flutter root: '
      "$_flutterRoot${dartUiLocation != null ? ' or the dart:ui location: $dartUiLocation' : ''}",
    );
    exit(1);
  }

  print('Starting up in interactive mode on ${path.relative(filePath, from: _flutterRoot)} ...');
  print('Type "q" to quit, or "r" to force a reload.');

  final _SnippetChecker checker = _SnippetChecker(flutterPackages, tempDirectory: tempDirectory)
    .._createConfigurationFiles();

  ProcessSignal.sigint.watch().listen((_) {
    checker.cleanupTempDirectory();
    exit(0);
  });

  bool busy = false;
  Future<void> rerun() async {
    assert(!busy);
    try {
      busy = true;
      print('\nAnalyzing...');
      checker.recreateTempDirectory();
      final Map<String, _SnippetFile> snippets = <String, _SnippetFile>{};
      final Set<Object> errors = <Object>{};
      errors.addAll(await checker._extractSnippets(<File>[file], snippetMap: snippets));
      errors.addAll(checker._analyze(snippets));
      stderr.writeln('\u001B[2J\u001B[H'); // Clears the old results from the terminal.
      if (errors.isNotEmpty) {
        (errors.toList()..sort()).map(_stringify).forEach(stderr.writeln);
        stderr.writeln('Found ${errors.length} errors.');
      } else {
        stderr.writeln('No issues found.');
      }
    } finally {
      busy = false;
    }
  }

  await rerun();

  stdin.lineMode = false;
  stdin.echoMode = false;
  stdin.transform(utf8.decoder).listen((String input) async {
    switch (input.trim()) {
      case 'q':
        checker.cleanupTempDirectory();
        exit(0);
      case 'r' when !busy:
        rerun();
    }
  });
  Watcher(file.absolute.path).events.listen((_) => rerun());
}

String _stringify(Object object) => object.toString();
