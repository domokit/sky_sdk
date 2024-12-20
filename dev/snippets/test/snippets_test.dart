// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:pub_semver/pub_semver.dart';
import 'package:snippets/snippets.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

import '../bin/snippets.dart' as snippets_main;
import 'fake_process_manager.dart';

class FakeFlutterInformation extends FlutterInformation {
  FakeFlutterInformation(this.flutterRoot);

  final Directory flutterRoot;

  @override
  Directory getFlutterRoot() {
    return flutterRoot;
  }

  @override
  Map<String, dynamic> getFlutterInformation() {
    return <String, dynamic>{
      'flutterRoot': flutterRoot,
      'frameworkVersion': Version(2, 10, 0),
      'dartSdkVersion': Version(2, 12, 1),
    };
  }
}

void main() {
  group('Generator', () {
    late MemoryFileSystem memoryFileSystem = MemoryFileSystem();
    late FlutterRepoSnippetConfiguration configuration;
    late SnippetGenerator generator;
    late Directory tmpDir;

    void writeSkeleton(String type) {
      switch (type) {
        case 'dartpad':
          configuration.getHtmlSkeletonFile('dartpad').writeAsStringSync('''
<div>HTML Bits (DartPad-style)</div>
<iframe class="snippet-dartpad" src="https://dartpad.dev/embed-flutter.html?split=60&run=true&sample_id={{id}}&sample_channel={{channel}}"></iframe>
<div>More HTML Bits</div>
''');
        case 'sample':
        case 'snippet':
          configuration.getHtmlSkeletonFile(type).writeAsStringSync('''
<div>HTML Bits</div>
{{description}}
<pre>{{code}}</pre>
<pre>{{app}}</pre>
<div>More HTML Bits</div>
''');
      }
    }

    setUp(() {
      // Create a new filesystem.
      memoryFileSystem = MemoryFileSystem();
      tmpDir = memoryFileSystem.systemTempDirectory.createTempSync('flutter_snippets_test.');
      configuration = FlutterRepoSnippetConfiguration(
        flutterRoot: memoryFileSystem.directory(path.join(tmpDir.absolute.path, 'flutter')),
        filesystem: memoryFileSystem,
      );
      configuration.skeletonsDirectory.createSync(recursive: true);
      <String>['dartpad', 'sample', 'snippet'].forEach(writeSkeleton);
      FlutterInformation.instance = FakeFlutterInformation(configuration.flutterRoot);
      generator = SnippetGenerator(
        configuration: configuration,
        filesystem: memoryFileSystem,
        flutterRoot: configuration.skeletonsDirectory.parent,
      );
    });

    test('generates samples', () async {
      final File inputFile =
          memoryFileSystem.file(path.join(tmpDir.absolute.path, 'snippet_in.txt'))
            ..createSync(recursive: true)
            ..writeAsStringSync(r'''
A description of the sample.

On several lines.

** See code in examples/api/widgets/foo/foo_example.0.dart **
''');
      final String examplePath = path.join(
        configuration.flutterRoot.path,
        'examples/api/widgets/foo/foo_example.0.dart',
      );
      memoryFileSystem.file(examplePath)
        ..create(recursive: true)
        ..writeAsStringSync('''
// Copyright

// Flutter code sample for [MyElement].

void main() {
  runApp(MaterialApp(title: 'foo'));
}\n''');
      final File outputFile = memoryFileSystem.file(
        path.join(tmpDir.absolute.path, 'snippet_out.txt'),
      );
      final SnippetDartdocParser sampleParser = SnippetDartdocParser(memoryFileSystem);
      const String sourcePath = 'packages/flutter/lib/src/widgets/foo.dart';
      const int sourceLine = 222;
      final SourceElement element = sampleParser.parseFromDartdocToolFile(
        inputFile,
        element: 'MyElement',
        startLine: sourceLine,
        sourceFile: memoryFileSystem.file(sourcePath),
        type: 'sample',
      );

      expect(element.samples, isNotEmpty);
      element.samples.first.metadata.addAll(<String, Object?>{'channel': 'stable'});
      final String code = generator.generateCode(element.samples.first, output: outputFile);
      expect(code, contains("runApp(MaterialApp(title: 'foo'));"));
      final String html = generator.generateHtml(element.samples.first);
      expect(html, contains('<div>HTML Bits</div>'));
      expect(html, contains('<div>More HTML Bits</div>'));
      expect(html, contains(r'''runApp(MaterialApp(title: &#39;foo&#39;));'''));
      expect(html, isNot(contains('sample_channel=stable')));
      expect(
        html,
        contains(
          'A description of the sample.\n'
          '\n'
          'On several lines.{@inject-html}</div>',
        ),
      );
      expect(html, contains('void main() {'));

      final String outputContents = outputFile.readAsStringSync();
      expect(outputContents, contains('void main() {'));
    });

    test('generates snippets', () async {
      final File inputFile =
          memoryFileSystem.file(path.join(tmpDir.absolute.path, 'snippet_in.txt'))
            ..createSync(recursive: true)
            ..writeAsStringSync(r'''
A description of the snippet.

On several lines.

```code
void main() {
  print('The actual $name.');
}
```
''');

      final SnippetDartdocParser sampleParser = SnippetDartdocParser(memoryFileSystem);
      const String sourcePath = 'packages/flutter/lib/src/widgets/foo.dart';
      const int sourceLine = 222;
      final SourceElement element = sampleParser.parseFromDartdocToolFile(
        inputFile,
        element: 'MyElement',
        startLine: sourceLine,
        sourceFile: memoryFileSystem.file(sourcePath),
        type: 'snippet',
      );
      expect(element.samples, isNotEmpty);
      element.samples.first.metadata.addAll(<String, Object>{'channel': 'stable'});
      final String code = generator.generateCode(element.samples.first);
      expect(code, contains('// A description of the snippet.'));
      final String html = generator.generateHtml(element.samples.first);
      expect(html, contains('<div>HTML Bits</div>'));
      expect(html, contains('<div>More HTML Bits</div>'));
      expect(html, contains(r'  print(&#39;The actual $name.&#39;);'));
      expect(
        html,
        contains(
          '<div class="snippet-description">{@end-inject-html}A description of the snippet.\n\n'
          'On several lines.{@inject-html}</div>\n',
        ),
      );
      expect(html, contains('main() {'));
    });

    test('generates dartpad samples', () async {
      final File inputFile =
          memoryFileSystem.file(path.join(tmpDir.absolute.path, 'snippet_in.txt'))
            ..createSync(recursive: true)
            ..writeAsStringSync(r'''
A description of the snippet.

On several lines.

** See code in examples/api/widgets/foo/foo_example.0.dart **
''');
      final String examplePath = path.join(
        configuration.flutterRoot.path,
        'examples/api/widgets/foo/foo_example.0.dart',
      );
      memoryFileSystem.file(examplePath)
        ..create(recursive: true)
        ..writeAsStringSync('''
// Copyright

// Flutter code sample for [MyElement].

void main() {
  runApp(MaterialApp(title: 'foo'));
}\n''');

      final SnippetDartdocParser sampleParser = SnippetDartdocParser(memoryFileSystem);
      const String sourcePath = 'packages/flutter/lib/src/widgets/foo.dart';
      const int sourceLine = 222;
      final SourceElement element = sampleParser.parseFromDartdocToolFile(
        inputFile,
        element: 'MyElement',
        startLine: sourceLine,
        sourceFile: memoryFileSystem.file(sourcePath),
        type: 'dartpad',
      );
      expect(element.samples, isNotEmpty);
      element.samples.first.metadata.addAll(<String, Object>{'channel': 'stable'});
      final String code = generator.generateCode(element.samples.first);
      expect(code, contains("runApp(MaterialApp(title: 'foo'));"));
      final String html = generator.generateHtml(element.samples.first);
      expect(html, contains('<div>HTML Bits (DartPad-style)</div>'));
      expect(html, contains('<div>More HTML Bits</div>'));
      expect(
        html,
        contains(
          '<iframe class="snippet-dartpad" src="https://dartpad.dev/embed-flutter.html?split=60&run=true&sample_id=MyElement.0&sample_channel=stable"></iframe>\n',
        ),
      );
    });

    test('generates sample metadata', () async {
      final File inputFile =
          memoryFileSystem.file(path.join(tmpDir.absolute.path, 'snippet_in.txt'))
            ..createSync(recursive: true)
            ..writeAsStringSync(r'''
A description of the snippet.

On several lines.

```dart
void main() {
  print('The actual $name.');
}
```
''');

      final File outputFile = memoryFileSystem.file(
        path.join(tmpDir.absolute.path, 'snippet_out.dart'),
      );
      final File expectedMetadataFile = memoryFileSystem.file(
        path.join(tmpDir.absolute.path, 'snippet_out.json'),
      );

      final SnippetDartdocParser sampleParser = SnippetDartdocParser(memoryFileSystem);
      const String sourcePath = 'packages/flutter/lib/src/widgets/foo.dart';
      const int sourceLine = 222;
      final SourceElement element = sampleParser.parseFromDartdocToolFile(
        inputFile,
        element: 'MyElement',
        startLine: sourceLine,
        sourceFile: memoryFileSystem.file(sourcePath),
        type: 'sample',
      );
      expect(element.samples, isNotEmpty);
      element.samples.first.metadata.addAll(<String, Object>{'channel': 'stable'});
      generator.generateCode(element.samples.first, output: outputFile);
      expect(expectedMetadataFile.existsSync(), isTrue);
      final Map<String, dynamic> json =
          jsonDecode(expectedMetadataFile.readAsStringSync()) as Map<String, dynamic>;
      expect(json['id'], equals('MyElement.0'));
      expect(json['channel'], equals('stable'));
      expect(json['file'], equals('snippet_out.dart'));
      expect(json['description'], equals('A description of the snippet.\n\nOn several lines.'));
      expect(json['sourcePath'], equals('packages/flutter/lib/src/widgets/foo.dart'));
    });
  });

  group('snippets command line argument test', () {
    late MemoryFileSystem memoryFileSystem = MemoryFileSystem();
    late Directory tmpDir;
    late Directory flutterRoot;
    late FakeProcessManager fakeProcessManager;

    setUp(() {
      fakeProcessManager = FakeProcessManager();
      memoryFileSystem = MemoryFileSystem();
      tmpDir = memoryFileSystem.systemTempDirectory.createTempSync('flutter_snippets_test.');
      flutterRoot = memoryFileSystem.directory(path.join(tmpDir.absolute.path, 'flutter'))
        ..createSync(recursive: true);
    });

    test('command line arguments are parsed and passed to generator', () {
      final FakePlatform platform = FakePlatform(
        environment: <String, String>{
          'PACKAGE_NAME': 'dart:ui',
          'LIBRARY_NAME': 'library',
          'ELEMENT_NAME': 'element',
          'FLUTTER_ROOT': flutterRoot.absolute.path,
          // The details here don't really matter other than the flutter root.
          'FLUTTER_VERSION': '''
      {
        "frameworkVersion": "2.5.0-6.0.pre.55",
        "channel": "use_snippets_pkg",
        "repositoryUrl": "git@github.com:flutter/flutter.git",
        "frameworkRevision": "fec4641e1c88923ecd6c969e2ff8a0dd12dc0875",
        "frameworkCommitDate": "2021-08-11 15:19:48 -0700",
        "engineRevision": "d8bbebed60a77b3d4fe9c840dc94dfbce159d951",
        "dartSdkVersion": "2.14.0 (build 2.14.0-393.0.dev)",
        "flutterRoot": "${flutterRoot.absolute.path}"
      }''',
        },
      );
      final FlutterInformation flutterInformation = FlutterInformation(
        filesystem: memoryFileSystem,
        processManager: fakeProcessManager,
        platform: platform,
      );
      FlutterInformation.instance = flutterInformation;
      MockSnippetGenerator mockSnippetGenerator = MockSnippetGenerator();
      snippets_main.snippetGenerator = mockSnippetGenerator;
      String errorMessage = '';
      errorExit = (String message) {
        errorMessage = message;
      };

      snippets_main.platform = platform;
      snippets_main.filesystem = memoryFileSystem;
      snippets_main.processManager = fakeProcessManager;
      final File input = memoryFileSystem.file(tmpDir.childFile('input.snippet'))
        ..writeAsString('/// Test file');
      snippets_main.main(<String>['--input=${input.absolute.path}']);

      final Map<String, dynamic> metadata = mockSnippetGenerator.sample.metadata;
      // Ignore the channel, because channel is really just the branch, and will be
      // different on development workstations.
      metadata.remove('channel');
      expect(
        metadata,
        equals(<String, dynamic>{
          'id': 'dart_ui.library.element',
          'element': 'element',
          'sourcePath': 'unknown.dart',
          'sourceLine': 1,
          'serial': '',
          'package': 'dart:ui',
          'library': 'library',
        }),
      );

      snippets_main.main(<String>[]);
      expect(
        errorMessage,
        equals(
          'The --input option must be specified, either on the command line, or in the INPUT environment variable.',
        ),
      );
      errorMessage = '';

      snippets_main.main(<String>['--input=${input.absolute.path}', '--type=snippet']);
      expect(errorMessage, equals(''));
      errorMessage = '';

      mockSnippetGenerator = MockSnippetGenerator();
      snippets_main.snippetGenerator = mockSnippetGenerator;
      snippets_main.main(<String>[
        '--input=${input.absolute.path}',
        '--type=snippet',
        '--no-format-output',
      ]);
      expect(mockSnippetGenerator.formatOutput, equals(false));
      errorMessage = '';

      input.deleteSync();
      snippets_main.main(<String>['--input=${input.absolute.path}']);
      expect(errorMessage, equals('The input file ${input.absolute.path} does not exist.'));
      errorMessage = '';
    });
  });
}

class MockSnippetGenerator extends SnippetGenerator {
  late CodeSample sample;
  File? output;
  String? copyright;
  String? description;
  late bool formatOutput;
  late bool addSectionMarkers;
  late bool includeAssumptions;

  @override
  String generateCode(
    CodeSample sample, {
    File? output,
    String? copyright,
    String? description,
    bool formatOutput = true,
    bool addSectionMarkers = false,
    bool includeAssumptions = false,
  }) {
    this.sample = sample;
    this.output = output;
    this.copyright = copyright;
    this.description = description;
    this.formatOutput = formatOutput;
    this.addSectionMarkers = addSectionMarkers;
    this.includeAssumptions = includeAssumptions;

    return '';
  }

  @override
  String generateHtml(CodeSample sample) {
    return '';
  }
}
