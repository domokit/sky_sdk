// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:path/path.dart' as path;

import 'configuration.dart';
import 'data_types.dart';
import 'util.dart';

/// Generates the snippet HTML, as well as saving the output snippet main to
/// the output directory.
class SnippetGenerator {
  SnippetGenerator({
    SnippetConfiguration? configuration,
    FileSystem filesystem = const LocalFileSystem(),
    Directory? flutterRoot,
  }) : flutterRoot = flutterRoot ?? FlutterInformation.instance.getFlutterRoot(),
       configuration =
           configuration ??
           FlutterRepoSnippetConfiguration(
             filesystem: filesystem,
             flutterRoot: flutterRoot ?? FlutterInformation.instance.getFlutterRoot(),
           );

  final Directory flutterRoot;

  /// The configuration used to determine where to get/save data for the
  /// snippet.
  final SnippetConfiguration configuration;

  static const JsonEncoder jsonEncoder = JsonEncoder.withIndent('    ');

  /// Interpolates the [injections] into an HTML skeleton file.
  ///
  /// The order of the injections is important.
  ///
  /// Takes into account the [type] and doesn't substitute in the id and the app
  /// if not a [SnippetType.sample] snippet.
  String interpolateSkeleton(CodeSample sample, String skeleton) {
    final List<String> codeParts = <String>[];
    const HtmlEscape htmlEscape = HtmlEscape();
    String? language;
    for (final SkeletonInjection injection in sample.parts) {
      if (!injection.name.startsWith('code')) {
        continue;
      }
      codeParts.addAll(injection.stringContents);
      if (injection.language.isNotEmpty) {
        language = injection.language;
      }
      codeParts.addAll(<String>['', '// ...', '']);
    }
    if (codeParts.length > 3) {
      codeParts.removeRange(codeParts.length - 3, codeParts.length);
    }
    // Only insert a div for the description if there actually is some text there.
    // This means that the {{description}} marker in the skeleton needs to
    // be inside of an {@inject-html} block.
    final String description =
        sample.description.trim().isNotEmpty
            ? '<div class="snippet-description">{@end-inject-html}${sample.description.trim()}{@inject-html}</div>'
            : '';

    // DartPad only supports stable or main as valid channels. Use main
    // if not on stable so that local runs will work (although they will
    // still take their sample code from the master docs server).
    final String channel = sample.metadata['channel'] == 'stable' ? 'stable' : 'main';

    final Map<String, String> substitutions = <String, String>{
      'description': description,
      'code': htmlEscape.convert(codeParts.join('\n')),
      'language': language ?? 'dart',
      'serial': '',
      'id': sample.metadata['id']! as String,
      'channel': channel,
      'element': sample.metadata['element'] as String? ?? sample.element,
      'app': '',
    };
    if (sample is ApplicationSample) {
      substitutions
        ..['serial'] = sample.metadata['serial']?.toString() ?? '0'
        ..['app'] = htmlEscape.convert(sample.output);
    }
    return skeleton.replaceAllMapped(RegExp('{{(${substitutions.keys.join('|')})}}'), (
      Match match,
    ) {
      return substitutions[match[1]]!;
    });
  }

  /// Consolidates all of the snippets and the assumptions into one snippet, in
  /// order to create a compilable result.
  Iterable<SourceLine> consolidateSnippets(List<CodeSample> samples, {bool addMarkers = false}) {
    if (samples.isEmpty) {
      return <SourceLine>[];
    }
    final Iterable<SnippetSample> snippets = samples.whereType<SnippetSample>();
    final List<SourceLine> snippetLines = <SourceLine>[...snippets.first.assumptions];
    for (final SnippetSample sample in snippets) {
      parseInput(sample);
      snippetLines.addAll(_processBlocks(sample));
    }
    return snippetLines;
  }

  /// A RegExp that matches a Dart constructor.
  static final RegExp _constructorRegExp = RegExp(r'(const\s+)?_*[A-Z][a-zA-Z0-9<>._]*\(');

  /// A serial number so that we can create unique expression names when we
  /// generate them.
  int _expressionId = 0;

  List<SourceLine> _surround(String prefix, Iterable<SourceLine> body, String suffix) {
    return <SourceLine>[
      if (prefix.isNotEmpty) SourceLine(prefix),
      ...body,
      if (suffix.isNotEmpty) SourceLine(suffix),
    ];
  }

  /// Process one block of sample code (the part inside of "```" markers).
  /// Splits any sections denoted by "// ..." into separate blocks to be
  /// processed separately. Uses a primitive heuristic to make sample blocks
  /// into valid Dart code.
  List<SourceLine> _processBlocks(CodeSample sample) {
    final List<SourceLine> block =
        sample.parts
            .expand<SourceLine>((SkeletonInjection injection) => injection.contents)
            .toList();
    if (block.isEmpty) {
      return <SourceLine>[];
    }
    return _processBlock(block);
  }

  List<SourceLine> _processBlock(List<SourceLine> block) {
    final String firstLine = block.first.text;
    if (firstLine.startsWith('new ') || firstLine.startsWith(_constructorRegExp)) {
      _expressionId += 1;
      return _surround('dynamic expression$_expressionId = ', block, ';');
    } else if (firstLine.startsWith('await ')) {
      _expressionId += 1;
      return _surround('Future<void> expression$_expressionId() async { ', block, ' }');
    } else if (block.first.text.startsWith('class ') || block.first.text.startsWith('enum ')) {
      return block;
    } else if ((block.first.text.startsWith('_') || block.first.text.startsWith('final ')) &&
        block.first.text.contains(' = ')) {
      _expressionId += 1;
      return _surround('void expression$_expressionId() { ', block.toList(), ' }');
    } else {
      final List<SourceLine> buffer = <SourceLine>[];
      int blocks = 0;
      SourceLine? subLine;
      final List<SourceLine> subsections = <SourceLine>[];
      for (int index = 0; index < block.length; index += 1) {
        // Each section of the dart code that is either split by a blank line, or with
        // '// ...' is treated as a separate code block.
        if (block[index].text.trim().isEmpty || block[index].text == '// ...') {
          if (subLine == null) {
            continue;
          }
          blocks += 1;
          subsections.addAll(_processBlock(buffer));
          buffer.clear();
          assert(buffer.isEmpty);
          subLine = null;
        } else if (block[index].text.startsWith('// ')) {
          if (buffer.length > 1) {
            // don't include leading comments
            // so that it doesn't start with "// " and get caught in this again
            buffer.add(SourceLine('/${block[index].text}'));
          }
        } else {
          subLine ??= block[index];
          buffer.add(block[index]);
        }
      }
      if (blocks > 0) {
        if (subLine != null) {
          subsections.addAll(_processBlock(buffer));
        }
        // Combine all of the subsections into one section, now that they've been processed.
        return subsections;
      } else {
        return block;
      }
    }
  }

  /// Parses the input for the various code and description segments, and
  /// returns a set of skeleton injections in the order found.
  List<SkeletonInjection> parseInput(CodeSample sample) {
    bool inCodeBlock = false;
    final List<SourceLine> description = <SourceLine>[];
    final List<SkeletonInjection> components = <SkeletonInjection>[];
    String? language;
    final RegExp codeStartEnd = RegExp(
      r'^\s*```(?<language>[-\w]+|[-\w]+ (?<section>[-\w]+))?\s*$',
    );
    for (final SourceLine line in sample.input) {
      final RegExpMatch? match = codeStartEnd.firstMatch(line.text);
      if (match != null) {
        // If we saw the start or end of a code block
        inCodeBlock = !inCodeBlock;
        if (match.namedGroup('language') != null) {
          language = match[1];
          if (match.namedGroup('section') != null) {
            components.add(
              SkeletonInjection(
                'code-${match.namedGroup('section')}',
                <SourceLine>[],
                language: language!,
              ),
            );
          } else {
            components.add(SkeletonInjection('code', <SourceLine>[], language: language!));
          }
        } else {
          language = null;
        }
        continue;
      }
      if (!inCodeBlock) {
        description.add(line);
      } else {
        assert(language != null);
        components.last.contents.add(line);
      }
    }
    final List<String> descriptionLines = <String>[];
    bool lastWasWhitespace = false;
    for (final String line in description.map<String>((SourceLine line) => line.text.trimRight())) {
      final bool onlyWhitespace = line.trim().isEmpty;
      if (onlyWhitespace && descriptionLines.isEmpty) {
        // Don't add whitespace lines until we see something without whitespace.
        lastWasWhitespace = onlyWhitespace;
        continue;
      }
      if (onlyWhitespace && lastWasWhitespace) {
        // Don't add more than one whitespace line in a row.
        continue;
      }
      descriptionLines.add(line);
      lastWasWhitespace = onlyWhitespace;
    }
    sample.description = descriptionLines.join('\n').trimRight();
    sample.parts = <SkeletonInjection>[
      if (sample is SnippetSample) SkeletonInjection('#assumptions', sample.assumptions),
      ...components,
    ];
    return sample.parts;
  }

  String _loadFileAsUtf8(File file) {
    return file.readAsStringSync();
  }

  /// Generate the HTML using the skeleton file for the type of the given sample.
  ///
  /// Returns a string with the HTML needed to embed in a web page for showing a
  /// sample on the web page.
  String generateHtml(CodeSample sample) {
    final String skeleton = _loadFileAsUtf8(configuration.getHtmlSkeletonFile(sample.type));
    return interpolateSkeleton(sample, skeleton);
  }

  // Sets the description string on the sample and in the sample metadata to a
  // comment version of the description.
  // Trims lines of extra whitespace, and strips leading and trailing blank
  // lines.
  String _getDescription(CodeSample sample) {
    return sample.description.splitMapJoin(
      '\n',
      onMatch: (Match match) => match.group(0)!,
      onNonMatch:
          (String nonmatch) => nonmatch.trimRight().isEmpty ? '//' : '// ${nonmatch.trimRight()}',
    );
  }

  /// The main routine for generating code samples from the source code doc comments.
  ///
  /// The `sample` is the block of sample code from a dartdoc comment.
  ///
  /// The optional `output` is the file to write the generated sample code to.
  ///
  /// If `includeAssumptions` is true, then the block in the "Examples can
  /// assume:" block will also be included in the output.
  ///
  /// Returns a string containing the resulting code sample.
  String generateCode(
    CodeSample sample, {
    File? output,
    String? copyright,
    String? description,
    bool includeAssumptions = false,
  }) {
    sample.metadata['copyright'] ??= copyright;
    final List<SkeletonInjection> snippetData = parseInput(sample);
    sample.description = description ?? sample.description;
    sample.metadata['description'] = _getDescription(sample);
    switch (sample) {
      case DartpadSample _:
      case ApplicationSample _:
        final String app = sample.sourceFileContents;
        sample.output = app;
        if (output != null) {
          output.writeAsStringSync(sample.output);

          final File metadataFile = configuration.filesystem.file(
            path.join(
              path.dirname(output.path),
              '${path.basenameWithoutExtension(output.path)}.json',
            ),
          );
          sample.metadata['file'] = path.basename(output.path);
          final Map<String, Object?> metadata = sample.metadata;
          if (metadata.containsKey('description')) {
            metadata['description'] = (metadata['description']! as String).replaceAll(
              RegExp(r'^// ?', multiLine: true),
              '',
            );
          }
          metadataFile.writeAsStringSync(jsonEncoder.convert(metadata));
        }
      case SnippetSample _:
        String app;
        if (sample.sourceFile == null) {
          String templateContents;
          if (includeAssumptions) {
            templateContents =
                '${headers.map<String>((SourceLine line) {
                  return line.text;
                }).join('\n')}\n{{#assumptions}}\n{{description}}\n{{code}}';
          } else {
            templateContents = '{{description}}\n{{code}}';
          }
          app = interpolateTemplate(
            snippetData,
            templateContents,
            sample.metadata,
            addCopyright: copyright != null,
          );
        } else {
          app = sample.inputAsString;
        }
        sample.output = app;
    }
    return sample.output;
  }

  /// Computes the headers needed for each snippet file.
  ///
  /// Not used for "sample" and "dartpad" samples, which use their own template.
  List<SourceLine> get headers {
    return _headers ??=
        <String>[
          '// generated code',
          '// ignore_for_file: unused_import',
          '// ignore_for_file: unused_element',
          '// ignore_for_file: unused_local_variable',
          "import 'dart:async';",
          "import 'dart:convert';",
          "import 'dart:math' as math;",
          "import 'dart:typed_data';",
          "import 'dart:ui' as ui;",
          "import 'package:flutter_test/flutter_test.dart';",
          for (final File file in _listDartFiles(
            FlutterInformation.instance
                .getFlutterRoot()
                .childDirectory('packages')
                .childDirectory('flutter')
                .childDirectory('lib'),
          )) ...<String>[
            '',
            '// ${file.path}',
            "import 'package:flutter/${path.basename(file.path)}';",
          ],
        ].map<SourceLine>((String code) => SourceLine(code)).toList();
  }

  List<SourceLine>? _headers;

  static List<File> _listDartFiles(Directory directory, {bool recursive = false}) {
    return directory
        .listSync(recursive: recursive, followLinks: false)
        .whereType<File>()
        .where((File file) => path.extension(file.path) == '.dart')
        .toList();
  }
}
