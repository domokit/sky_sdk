// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:args/args.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:git_repo_tools/git_repo_tools.dart';
import 'package:path/path.dart' as path;

/// Takes the images in `source_images`, writes text on them (i.e. git hash)
/// and saves them in `e2e_fixtures`. By default, no arguments are needed but
/// ImageMagick must be installed.
void main(List<String> args) async {
  final Engine? engine = Engine.tryFindWithin();

  final ArgParser parser =
      ArgParser()
        ..addFlag('help', abbr: 'h', help: 'Prints usage information.', negatable: false)
        ..addOption(
          'image-magick-convert-bin',
          help: 'The path to the ImageMagick `convert` executable.',
          defaultsTo: 'convert',
          hide: true,
        )
        ..addOption(
          'annotation',
          abbr: 'a',
          help: 'The text to write on the images.',
          defaultsTo:
              engine == null
                  ? null
                  : await GitRepo.fromRoot(engine.flutterDir).headSha(short: true),
        )
        ..addOption(
          'source',
          abbr: 's',
          help: 'The directory containing the images to be modified.',
          defaultsTo:
              engine == null
                  ? null
                  : path.join(
                    engine.flutterDir.path,
                    'testing',
                    'skia_gold_client',
                    'tool',
                    'source_images',
                  ),
        )
        ..addOption(
          'output',
          abbr: 'o',
          help: 'The directory to save the modified images in.',
          defaultsTo:
              engine == null
                  ? null
                  : path.join(
                    engine.flutterDir.path,
                    'testing',
                    'skia_gold_client',
                    'tool',
                    'e2e_fixtures',
                  ),
        );

  final ArgResults results = parser.parse(args);
  if (results['help'] as bool) {
    print(parser.usage);
    return;
  }

  final String relativeDir = engine?.flutterDir.path ?? '';
  final String imageMagickConvertBin = results['image-magick-convert-bin'] as String;
  final String annotation = results['annotation'] as String;
  final String source = results['source'] as String;
  final String output = results['output'] as String;

  print(
    'Writing annotation "$annotation" on images in '
    '${path.relative(source, from: relativeDir)} and saving them in '
    '${path.relative(output, from: relativeDir)}.',
  );

  final List<String> sourceImages =
      Directory(source).listSync().whereType<File>().map((File file) => file.path).toList();

  // For each source image, write the annotation and save it in the output directory.
  for (final String sourceImage in sourceImages) {
    final String outputImage = path.join(
      output,
      '${path.basenameWithoutExtension(sourceImage)}.png',
    );
    print('Writing to ${path.relative(outputImage, from: relativeDir)}');
    await Process.run(imageMagickConvertBin, <String>[
      sourceImage,
      '-fill',
      'white',
      '-undercolor',
      'black',
      '-gravity',
      'SouthEast',
      '-pointsize',
      '24',
      '-annotate',
      '+10+10',
      annotation,
      outputImage,
    ]);
  }

  print('Done: wrote ${sourceImages.length} image.');
}
