// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:args/args.dart';
import 'package:gen_web_keyboard_keymap/benchmark_planner.dart';
import 'package:gen_web_keyboard_keymap/common.dart';
import 'package:gen_web_keyboard_keymap/github.dart';
import 'package:gen_web_keyboard_keymap/layout_types.dart';
import 'package:path/path.dart' as path;

const String kEnvGithubToken = 'GITHUB_TOKEN';

String _renderTemplate(String template, Map<String, String> dictionary) {
  String result = template;
  dictionary.forEach((String key, String value) {
    final String localResult = result.replaceAll('@@@$key@@@', value);
    if (localResult == result) {
      print('Template key $key is not used.');
    }
    result = localResult;
  });
  return result;
}

void _writeFileTo(String outputDir, String outputFileName, String body) {
  final String outputPath = path.join(outputDir, outputFileName);
  Directory(outputDir).createSync(recursive: true);
  File(outputPath).writeAsStringSync(body);
}

String _readSharedSegment(String path) {
  const String kSegmentStartMark = '/*@@@ SHARED SEGMENT START @@@*/';
  const String kSegmentEndMark = '/*@@@ SHARED SEGMENT END @@@*/';
  final List<String> lines = File(path).readAsStringSync().split('\n');
  // Defining the two variables as `late final` ensures that each mark is found
  // once and only once, otherwise assertion errors will be thrown.
  late final int startLine;
  late final int endLine;
  for (int lineNo = 0; lineNo < lines.length; lineNo += 1) {
    if (lines[lineNo] == kSegmentStartMark) {
      startLine = lineNo;
    } else if (lines[lineNo] == kSegmentEndMark) {
      endLine = lineNo;
    }
  }
  assert(startLine < endLine);
  return lines.sublist(startLine + 1, endLine).join('\n').trimRight();
}

typedef _ForEachAction<V> = void Function(String key, V value);
void _sortedForEach<V>(Map<String, V> map, _ForEachAction<V> action) {
  map.entries.toList()
    ..sort((MapEntry<String, V> a, MapEntry<String, V> b) => a.key.compareTo(b.key))
    ..forEach((MapEntry<String, V> entry) {
      action(entry.key, entry.value);
    });
}

String _escapeStringToDart(String origin) {
  // If there is no `'`, we can use the raw string surrounded by `'`.
  if (!origin.contains("'")) {
    return "r'$origin'";
  } else {
    // If there is no `"`, we can use the raw string surrounded by `"`.
    if (!origin.contains('"')) {
      return 'r"$origin"';
    } else {
      // If there are both kinds of quotes, we have to use non-raw string
      // and escape necessary characters.
      final String beforeQuote = origin
          .replaceAll(r'\', r'\\')
          .replaceAll(r'$', r'\$')
          .replaceAll("'", r"\'");
      return "'$beforeQuote'";
    }
  }
}

typedef _ValueCompare<T> = void Function(T?, T?, String path);
void _mapForEachEqual<T>(Map<String, T> a, Map<String, T> b, _ValueCompare<T> body, String path) {
  assert(a.length == b.length, '$path.length: ${a.length} vs ${b.length}');
  for (final String key in a.keys) {
    body(a[key], b[key], key);
  }
}

// Ensure that two maps are deeply equal.
//
// Differences will be thrown as assertion.
bool _verifyMap(Map<String, Map<String, int>> a, Map<String, Map<String, int>> b) {
  _mapForEachEqual(a, b, (Map<String, int>? aMap, Map<String, int>? bMap, String path) {
    _mapForEachEqual(aMap!, bMap!, (int? aValue, int? bValue, String path) {
      assert(aValue == bValue && aValue != null, '$path: $aValue vs $bValue');
    }, path);
  }, '');
  return true;
}

String _buildMapString(Iterable<Layout> layouts) {
  final Map<String, Map<String, int>> originalMap = combineLayouts(layouts);
  final List<String> compressed = marshallMappingData(originalMap);
  final Map<String, Map<String, int>> uncompressed = unmarshallMappingData(compressed.join());
  assert(_verifyMap(originalMap, uncompressed));
  return '  return unmarshallMappingData(\n'
      '${compressed.map((String line) => '    ${_escapeStringToDart(line)}\n').join()}'
      '  ); // ${compressed.join().length} characters';
}

String _buildTestCasesString(List<Layout> layouts) {
  final List<String> layoutsString = <String>[];
  for (final Layout layout in layouts) {
    final List<String> layoutEntries = <String>[];
    _sortedForEach(planLayout(layout.entries), (String eventCode, int logicalKey) {
      final LayoutEntry entry = layout.entries[eventCode]!;
      layoutEntries.add(
        "    verifyEntry(mapping, '$eventCode', <String>["
        '${entry.printables.map(_escapeStringToDart).join(', ')}'
        "], '${String.fromCharCode(logicalKey)}');",
      );
    });
    layoutsString.add('''
  group('${layout.language}', () {
${layoutEntries.join('\n')}
  });
''');
  }
  return layoutsString.join('\n').trimRight();
}

Future<void> main(List<String> rawArguments) async {
  final Map<String, String> env = Platform.environment;
  final ArgParser argParser = ArgParser();
  argParser.addFlag(
    'force',
    abbr: 'f',
    negatable: false,
    help: 'Make a new request to GitHub even if a cache is detected',
  );
  argParser.addFlag('help', abbr: 'h', negatable: false, help: 'Print help for this command.');

  final ArgResults parsedArguments = argParser.parse(rawArguments);

  if (parsedArguments['help'] as bool) {
    print(argParser.usage);
    exit(0);
  }

  bool enabledAssert = false;
  assert(() {
    enabledAssert = true;
    return true;
  }());
  if (!enabledAssert) {
    print(
      'Error: This script must be run with assert enabled. Please rerun with --enable-asserts.',
    );
    exit(1);
  }

  final String? envGithubToken = env[kEnvGithubToken];
  if (envGithubToken == null) {
    print(
      'Error: Environment variable $kEnvGithubToken not found.\n\n'
      'Set the environment variable $kEnvGithubToken as a GitHub personal access\n'
      'token for authentication. This token is only used for quota controlling\n'
      'and does not need any scopes. Create one at\n'
      'https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token.',
    );
    exit(1);
  }

  // The root of this package. The folder that is called
  // 'gen_web_locale_keymap' and contains 'pubspec.yaml'.
  final Directory packageRoot = Directory(path.dirname(Platform.script.toFilePath())).parent;

  // The root of the output package. The folder that is called
  // 'web_locale_keymap' and contains 'pubspec.yaml'.
  final String outputRoot = path.join(
    packageRoot.parent.parent.path,
    'third_party',
    'web_locale_keymap',
  );

  final GithubResult githubResult = await fetchFromGithub(
    githubToken: envGithubToken,
    force: parsedArguments['force'] as bool,
    cacheRoot: path.join(packageRoot.path, '.cache'),
  );

  final List<Layout> winLayouts =
      githubResult.layouts.where((Layout layout) => layout.platform == LayoutPlatform.win).toList();
  final List<Layout> linuxLayouts =
      githubResult.layouts
          .where((Layout layout) => layout.platform == LayoutPlatform.linux)
          .toList();
  final List<Layout> darwinLayouts =
      githubResult.layouts
          .where((Layout layout) => layout.platform == LayoutPlatform.darwin)
          .toList();

  // Generate the definition file.
  _writeFileTo(
    path.join(outputRoot, 'lib', 'web_locale_keymap'),
    'key_mappings.g.dart',
    _renderTemplate(
      File(path.join(packageRoot.path, 'data', 'key_mappings.dart.tmpl')).readAsStringSync(),
      <String, String>{
        'COMMIT_URL': githubResult.url,
        'WIN_MAPPING': _buildMapString(winLayouts),
        'LINUX_MAPPING': _buildMapString(linuxLayouts),
        'DARWIN_MAPPING': _buildMapString(darwinLayouts),
        'COMMON': _readSharedSegment(path.join(packageRoot.path, 'lib', 'common.dart')),
      },
    ),
  );

  // Generate the test cases.
  _writeFileTo(
    path.join(outputRoot, 'test'),
    'test_cases.g.dart',
    _renderTemplate(
      File(path.join(packageRoot.path, 'data', 'test_cases.dart.tmpl')).readAsStringSync(),
      <String, String>{
        'WIN_CASES': _buildTestCasesString(winLayouts),
        'LINUX_CASES': _buildTestCasesString(linuxLayouts),
        'DARWIN_CASES': _buildTestCasesString(darwinLayouts),
      },
    ),
  );
}
