// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:path/path.dart' as path;
import 'package:pub_semver/pub_semver.dart';
import 'package:snippets/snippets.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

import 'filesystem_resource_provider.dart';

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
  group('Parser', () {
    late MemoryFileSystem memoryFileSystem = MemoryFileSystem();
    late FlutterRepoSnippetConfiguration configuration;
    late SnippetGenerator generator;
    late Directory tmpDir;
    late Directory flutterRoot;

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
      flutterRoot = memoryFileSystem.directory(path.join(tmpDir.absolute.path, 'flutter'));
      configuration = FlutterRepoSnippetConfiguration(
        flutterRoot: flutterRoot,
        filesystem: memoryFileSystem,
      );
      configuration.skeletonsDirectory.createSync(recursive: true);
      <String>['dartpad', 'sample', 'snippet'].forEach(writeSkeleton);
      FlutterInformation.instance = FakeFlutterInformation(flutterRoot);
      generator = SnippetGenerator(
        configuration: configuration,
        filesystem: memoryFileSystem,
        flutterRoot: configuration.skeletonsDirectory.parent,
      );
    });

    test('parses from comments', () async {
      final File inputFile = _createSnippetSourceFile(tmpDir, memoryFileSystem);
      final Iterable<SourceElement> elements = getFileElements(
        inputFile,
        resourceProvider: FileSystemResourceProvider(memoryFileSystem),
      );
      expect(elements, isNotEmpty);
      final SnippetDartdocParser sampleParser = SnippetDartdocParser(memoryFileSystem);
      sampleParser.parseFromComments(elements);
      sampleParser.parseAndAddAssumptions(elements, inputFile);
      expect(elements.length, equals(7));
      int sampleCount = 0;
      for (final SourceElement element in elements) {
        expect(element.samples.length, greaterThanOrEqualTo(1));
        sampleCount += element.samples.length;
        final String code = generator.generateCode(element.samples.first);
        expect(code, contains('// Description'));
        expect(
          code,
          contains(
            RegExp('''^String elementName = '${element.elementName}';\$''', multiLine: true),
          ),
        );
        final String html = generator.generateHtml(element.samples.first);
        expect(
          html,
          contains(
            RegExp(
              '''^<pre>String elementName = &#39;${element.elementName}&#39;;.*\$''',
              multiLine: true,
            ),
          ),
        );
        expect(
          html,
          contains(
            '<div class="snippet-description">{@end-inject-html}Description{@inject-html}</div>\n',
          ),
        );
      }
      expect(sampleCount, equals(8));
    });
    test('parses dartpad samples from linked file', () async {
      final File inputFile = _createDartpadSourceFile(
        tmpDir,
        memoryFileSystem,
        flutterRoot,
        linked: true,
      );
      final Iterable<SourceElement> elements = getFileElements(
        inputFile,
        resourceProvider: FileSystemResourceProvider(memoryFileSystem),
      );
      expect(elements, isNotEmpty);
      final SnippetDartdocParser sampleParser = SnippetDartdocParser(memoryFileSystem);
      sampleParser.parseFromComments(elements);
      expect(elements.length, equals(1));
      int sampleCount = 0;
      for (final SourceElement element in elements) {
        expect(element.samples.length, greaterThanOrEqualTo(1));
        sampleCount += element.samples.length;
        final String code = generator.generateCode(element.samples.first);
        expect(code, contains('// Description'));
        expect(
          code,
          contains(RegExp('^void ${element.name}Sample\\(\\) \\{.*\$', multiLine: true)),
        );
        final String html = generator.generateHtml(element.samples.first);
        expect(
          html,
          contains(
            RegExp(
              '''^<iframe class="snippet-dartpad" src="https://dartpad.dev/.*sample_id=${element.name}.0.*></iframe>.*\$''',
              multiLine: true,
            ),
          ),
        );
      }
      expect(sampleCount, equals(1));
    });
    test('parses assumptions', () async {
      final File inputFile = _createSnippetSourceFile(tmpDir, memoryFileSystem);
      final SnippetDartdocParser sampleParser = SnippetDartdocParser(memoryFileSystem);
      final List<SourceLine> assumptions = sampleParser.parseAssumptions(inputFile);
      expect(assumptions.length, equals(1));
      expect(assumptions.first.text, equals('int integer = 3;'));
    });
  });
}

File _createSnippetSourceFile(Directory tmpDir, FileSystem filesystem) {
  return filesystem.file(path.join(tmpDir.absolute.path, 'snippet_in.dart'))
    ..createSync(recursive: true)
    ..writeAsStringSync(r'''
// Copyright

// @dart = 2.12

import 'foo.dart';

// Examples can assume:
// int integer = 3;

/// Top level variable comment
///
/// {@tool snippet}
/// Description
/// ```dart
/// String elementName = 'topLevelVariable';
/// ```
/// {@end-tool}
int topLevelVariable = 4;

/// Top level function comment
///
/// {@tool snippet}
/// Description
/// ```dart
/// String elementName = 'topLevelFunction';
/// ```
/// {@end-tool}
int topLevelFunction() {
  return integer;
}

/// Class comment
///
/// {@tool snippet}
/// Description
/// ```dart
/// String elementName = 'DocumentedClass';
/// ```
/// {@end-tool}
///
/// {@tool snippet}
/// Description2
/// ```dart
/// String elementName = 'DocumentedClass';
/// ```
/// {@end-tool}
class DocumentedClass {
  /// Constructor comment
  /// {@tool snippet}
  /// Description
  /// ```dart
  /// String elementName = 'DocumentedClass';
  /// ```
  /// {@end-tool}
  const DocumentedClass();

  /// Named constructor comment
  /// {@tool snippet}
  /// Description
  /// ```dart
  /// String elementName = 'DocumentedClass.name';
  /// ```
  /// {@end-tool}
  const DocumentedClass.name();

  /// Member variable comment
  /// {@tool snippet}
  /// Description
  /// ```dart
  /// String elementName = 'DocumentedClass.intMember';
  /// ```
  /// {@end-tool}
  int intMember;

  /// Member comment
  /// {@tool snippet}
  /// Description
  /// ```dart
  /// String elementName = 'DocumentedClass.member';
  /// ```
  /// {@end-tool}
  void member() {}
}
''');
}

File _createDartpadSourceFile(
  Directory tmpDir,
  FileSystem filesystem,
  Directory flutterRoot, {
  bool linked = false,
}) {
  final File linkedFile =
      filesystem.file(path.join(flutterRoot.absolute.path, 'linked_file.dart'))
        ..createSync(recursive: true)
        ..writeAsStringSync('''
// Copyright

import 'foo.dart';

// Description

void DocumentedClassSample() {
  String elementName = 'DocumentedClass';
}
''');

  final String source =
      linked
          ? '''
/// ** See code in ${path.relative(linkedFile.path, from: flutterRoot.absolute.path)} **'''
          : '''
/// ```dart
/// void DocumentedClassSample() {
///   String elementName = 'DocumentedClass';
/// }
/// ```''';

  return filesystem.file(path.join(tmpDir.absolute.path, 'snippet_in.dart'))
    ..createSync(recursive: true)
    ..writeAsStringSync('''
// Copyright

// @dart = 2.12

import 'foo.dart';

/// Class comment
///
/// {@tool dartpad --template=template}
/// Description
$source
/// {@end-tool}
class DocumentedClass {}
''');
}
