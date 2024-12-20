// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:args/args.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as p;

/// "Downloads" (i.e. decodes base64 encoded strings) goldens from buildbucket.
///
/// See ../README.md for motivation and usage.
final class BuildBucketGoldenScraper {
  /// Creates a scraper with the given configuration.
  BuildBucketGoldenScraper({
    required this.pathOrUrl,
    this.dryRun = false,
    String? engineSrcPath,
    StringSink? outSink,
  }) : engine =
           engineSrcPath != null
               ? Engine.fromSrcPath(engineSrcPath)
               : Engine.findWithin(p.dirname(p.fromUri(io.Platform.script))),
       _outSink = outSink ?? io.stdout;

  /// Creates a scraper from the command line arguments.
  ///
  /// Throws [FormatException] if the arguments are invalid.
  factory BuildBucketGoldenScraper.fromCommandLine(
    List<String> args, {
    StringSink? outSink,
    StringSink? errSink,
  }) {
    outSink ??= io.stdout;
    errSink ??= io.stderr;

    final ArgResults argResults = _argParser.parse(args);
    if (argResults['help'] as bool) {
      _usage(args);
    }
    final String? pathOrUrl = argResults.rest.isEmpty ? null : argResults.rest.first;
    if (pathOrUrl == null) {
      _usage(args);
    }
    return BuildBucketGoldenScraper(
      pathOrUrl: pathOrUrl,
      dryRun: argResults['dry-run'] as bool,
      outSink: outSink,
      engineSrcPath: argResults['engine-src-path'] as String?,
    );
  }

  static Never _usage(List<String> args) {
    final StringBuffer output = StringBuffer();
    output.writeln('Usage: build_bucket_golden_scraper [options] <path or URL>');
    output.writeln();
    output.writeln(_argParser.usage);
    throw FormatException(output.toString(), args.join(' '));
  }

  static final ArgParser _argParser =
      ArgParser()
        ..addFlag('help', abbr: 'h', help: 'Print this help message.', negatable: false)
        ..addFlag(
          'dry-run',
          help: "If true, don't write any files to disk (other than temporary files).",
          negatable: false,
        )
        ..addOption(
          'engine-src-path',
          help: 'The path to the engine source code.',
          valueHelp: 'path/that/contains/src (defaults to the directory containing this script)',
        );

  /// A local path or a URL to a buildbucket log file.
  final String pathOrUrl;

  /// If true, don't write any files to disk (other than temporary files).
  final bool dryRun;

  /// The path to the engine source code.
  final Engine engine;

  /// How to print output, typically [io.stdout].
  final StringSink _outSink;

  /// Runs the scraper.
  Future<int> run() async {
    // If the path is a URL, download it and store it in a temporary file.
    final Uri? maybeUri = Uri.tryParse(pathOrUrl);
    if (maybeUri == null) {
      throw FormatException('Invalid path or URL: $pathOrUrl');
    }

    final String contents;
    if (maybeUri.hasScheme) {
      contents = await _downloadFile(maybeUri);
    } else {
      final io.File readFile = io.File(pathOrUrl);
      if (!readFile.existsSync()) {
        throw FormatException('File does not exist: $pathOrUrl');
      }
      contents = readFile.readAsStringSync();
    }

    // Check that it is a buildbucket log file.
    if (!contents.contains(_buildBucketMagicString)) {
      throw FormatException('Not a buildbucket log file: $pathOrUrl');
    }

    // Check for occurences of a base64 encoded string.
    //
    // The format looks something like this:
    // [LINE N+0]: See also the base64 encoded /b/s/w/ir/cache/builder/src/flutter/testing/resources/performance_overlay_gold_120fps_new.png:
    // [LINE N+1]: {{BASE_64_ENCODED_IMAGE}}
    //
    // We want to extract the file name (relative to the engine root) and then
    // decode the base64 encoded string (and write it to disk if we are not in
    // dry-run mode).
    final List<_Golden> goldens = <_Golden>[];
    final List<String> lines = contents.split('\n');
    for (int i = 0; i < lines.length; i++) {
      final String line = lines[i];
      if (line.startsWith(_base64MagicString)) {
        final String relativePath = line.split(_buildBucketMagicString).last.split(':').first;

        // Remove the _new suffix from the file name.
        final String pathWithouNew = relativePath.replaceAll('_new', '');

        final String base64EncodedString = lines[i + 1];
        final List<int> bytes = base64Decode(base64EncodedString);
        final io.File outFile = io.File(p.join(engine.srcDir.path, pathWithouNew));
        goldens.add(_Golden(outFile, bytes));
      }
    }

    if (goldens.isEmpty) {
      _outSink.writeln('No goldens found.');
      return 0;
    }

    // Sort and de-duplicate the goldens.
    goldens.sort();
    final Set<_Golden> uniqueGoldens = goldens.toSet();

    // Write the goldens to disk (or pretend to in dry-run mode).
    _outSink.writeln('${dryRun ? 'Found' : 'Wrote'} ${uniqueGoldens.length} golden file changes:');
    for (final _Golden golden in uniqueGoldens) {
      final String truncatedPathAfterFlutterDir =
          golden.outFile.path.split('flutter${p.separator}').last;
      _outSink.writeln('  $truncatedPathAfterFlutterDir');
      if (!dryRun) {
        await golden.outFile.writeAsBytes(golden.bytes);
      }
    }
    if (dryRun) {
      _outSink.writeln('Run again without --dry-run to apply these changes.');
    }

    return 0;
  }

  static const String _buildBucketMagicString = '/b/s/w/ir/cache/builder/src/';
  static const String _base64MagicString = 'See also the base64 encoded $_buildBucketMagicString';

  static Future<String> _downloadFile(Uri uri) async {
    final io.HttpClient client = io.HttpClient();
    final io.HttpClientRequest request = await client.getUrl(uri);
    final io.HttpClientResponse response = await request.close();
    final StringBuffer contents = StringBuffer();
    await response.transform(utf8.decoder).forEach(contents.write);
    client.close();
    return contents.toString();
  }
}

@immutable
final class _Golden implements Comparable<_Golden> {
  const _Golden(this.outFile, this.bytes);

  /// Where to write the golden file.
  final io.File outFile;

  /// The bytes of the golden file to write.
  final List<int> bytes;

  @override
  int get hashCode => outFile.path.hashCode;

  @override
  bool operator ==(Object other) {
    return other is _Golden && other.outFile.path == outFile.path;
  }

  @override
  int compareTo(_Golden other) {
    return outFile.path.compareTo(other.outFile.path);
  }
}
