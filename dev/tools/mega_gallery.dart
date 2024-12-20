// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:args/args.dart';
import 'package:path/path.dart' as path;

/// If no `copies` param is passed in, we scale the generated app up to 60k lines.
const int kTargetLineCount = 60 * 1024;

/// Make `n` copies of flutter_gallery.
void main(List<String> args) {
  // If we're run from the `tools` dir, set the cwd to the repo root.
  if (path.basename(Directory.current.path) == 'tools') {
    Directory.current = Directory.current.parent.parent;
  }

  final ArgParser argParser = ArgParser();
  argParser.addOption('out');
  argParser.addOption('copies');
  argParser.addFlag('delete', negatable: false);
  argParser.addFlag('help', abbr: 'h', negatable: false);

  final ArgResults results = argParser.parse(args);

  if (results['help'] as bool) {
    print('Generate n copies of flutter_gallery.\n');
    print('usage: dart mega_gallery.dart <options>');
    print(argParser.usage);
    exit(0);
  }

  final Directory source = Directory(_normalize('dev/integration_tests/flutter_gallery'));
  final Directory out = Directory(_normalize(results['out'] as String));

  if (results['delete'] as bool) {
    if (out.existsSync()) {
      print('Deleting ${out.path}');
      out.deleteSync(recursive: true);
    }

    exit(0);
  }

  if (!results.wasParsed('out')) {
    print('The --out parameter is required.');
    print(argParser.usage);
    exit(1);
  }

  int copies;
  if (!results.wasParsed('copies')) {
    final SourceStats stats = getStatsFor(_dir(source, 'lib'));
    copies = (kTargetLineCount / stats.lines).round();
  } else {
    copies = int.parse(results['copies'] as String);
  }

  print('Making $copies copies of flutter_gallery.');
  print('');
  print('Stats:');
  print('  packages/flutter            : ${getStatsFor(Directory("packages/flutter"))}');
  print(
    '  dev/integration_tests/flutter_gallery    : ${getStatsFor(Directory("dev/integration_tests/flutter_gallery"))}',
  );

  final Directory lib = _dir(out, 'lib');
  if (lib.existsSync()) {
    lib.deleteSync(recursive: true);
  }

  // Copy everything that's not a symlink, dot directory, or build/.
  _copy(source, out);

  // Make n - 1 copies.
  for (int i = 1; i < copies; i++) {
    _copyGallery(out, i);
  }

  // Create a new entry-point.
  _createEntry(_file(out, 'lib/main.dart'), copies);

  // Update the pubspec.
  String pubspec = _file(out, 'pubspec.yaml').readAsStringSync();
  pubspec = pubspec.replaceAll('../../packages/flutter', '../../../packages/flutter');
  _file(out, 'pubspec.yaml').writeAsStringSync(pubspec);

  // Replace the (flutter_gallery specific) analysis_options.yaml file with a default one.
  _file(out, 'analysis_options.yaml').writeAsStringSync('''
analyzer:
  errors:
    # See analysis_options.yaml in the flutter root for context.
    deprecated_member_use: ignore
    deprecated_member_use_from_same_package: ignore
''');

  _file(out, '.dartignore').writeAsStringSync('');

  // Count source lines and number of files; tell how to run it.
  print('  ${path.relative(results["out"] as String)} : ${getStatsFor(out)}');
}

// TODO(devoncarew): Create an entry-point that builds a UI with all `n` copies.
void _createEntry(File mainFile, int copies) {
  final StringBuffer imports = StringBuffer();

  for (int i = 1; i < copies; i++) {
    imports.writeln('// ignore: unused_import');
    imports.writeln("import 'gallery_$i/main.dart' as main_$i;");
  }

  final String contents = '''
// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

import 'gallery/app.dart';
${imports.toString().trim()}

void main() {
  runApp(const GalleryApp());
}
''';

  mainFile.writeAsStringSync(contents);
}

void _copyGallery(Directory galleryDir, int index) {
  final Directory lib = _dir(galleryDir, 'lib');
  final Directory dest = _dir(lib, 'gallery_$index');
  dest.createSync();

  // Copy demo/, gallery/, and main.dart.
  _copy(_dir(lib, 'demo'), _dir(dest, 'demo'));
  _copy(_dir(lib, 'gallery'), _dir(dest, 'gallery'));
  _file(dest, 'main.dart').writeAsBytesSync(_file(lib, 'main.dart').readAsBytesSync());
}

void _copy(Directory source, Directory target) {
  if (!target.existsSync()) {
    target.createSync(recursive: true);
  }

  for (final FileSystemEntity entity in source.listSync(followLinks: false)) {
    final String name = path.basename(entity.path);

    switch (entity) {
      case Directory() when name != 'build' && !name.startsWith('.'):
        _copy(entity, Directory(path.join(target.path, name)));

      case File() when name != '.packages' && name != 'pubspec.lock':
        final File dest = File(path.join(target.path, name));
        dest.writeAsBytesSync(entity.readAsBytesSync());
    }
  }
}

Directory _dir(Directory parent, String name) => Directory(path.join(parent.path, name));
File _file(Directory parent, String name) => File(path.join(parent.path, name));
String _normalize(String filePath) => path.normalize(path.absolute(filePath));

class SourceStats {
  int files = 0;
  int lines = 0;

  @override
  String toString() => '${_comma(files).padLeft(3)} files, ${_comma(lines).padLeft(6)} lines';
}

SourceStats getStatsFor(Directory dir, [SourceStats? stats]) {
  stats ??= SourceStats();

  for (final FileSystemEntity entity in dir.listSync(followLinks: false)) {
    final String name = path.basename(entity.path);
    if (entity is File && name.endsWith('.dart')) {
      stats.files += 1;
      stats.lines += _lineCount(entity);
    } else if (entity is Directory && !name.startsWith('.')) {
      getStatsFor(entity, stats);
    }
  }

  return stats;
}

int _lineCount(File file) {
  return file.readAsLinesSync().where((String line) {
    line = line.trim();
    if (line.isEmpty || line.startsWith('//')) {
      return false;
    }
    return true;
  }).length;
}

String _comma(int count) {
  final String str = count.toString();
  if (str.length > 3) {
    return '${str.substring(0, str.length - 3)},${str.substring(str.length - 3)}';
  }
  return str;
}
