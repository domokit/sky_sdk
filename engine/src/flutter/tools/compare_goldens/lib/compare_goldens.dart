// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

bool _hasCommandOnPath(String name) {
  final ProcessResult result = Process.runSync('which', <String>[name]);
  return result.exitCode == 0;
}

List<String> _findPairs(Set<String> as, Set<String> bs) {
  final List<String> result = <String>[];
  for (final String a in as) {
    if (bs.contains(a)) {
      result.add(a);
    } else {
      print('Mix match file $a.');
    }
  }
  for (final String b in bs) {
    if (!as.contains(b)) {
      print('Mix match file $b.');
    }
  }
  return result;
}

String _basename(String path) {
  return path.split(Platform.pathSeparator).last;
}

Set<String> _grabPngFilenames(Directory dir) {
  return dir
      .listSync()
      .map((FileSystemEntity e) => _basename(e.path))
      .where((String e) => e.endsWith('.png'))
      .toSet();
}

/// The main entry point to the tool, execute it like `main`. Returns the
/// `exitCode`.
int run(List<String> args) {
  int returnCode = 0;
  if (!_hasCommandOnPath('compare')) {
    throw Exception(r'Could not find `compare` from ImageMagick on $PATH.');
  }
  if (args.length != 2) {
    throw Exception('Usage: compare_goldens.dart <dir path> <dir path>');
  }

  final Directory dirA = Directory(args[0]);
  if (!dirA.existsSync()) {
    throw Exception('Unable to find $dirA');
  }
  final Directory dirB = Directory(args[1]);
  if (!dirB.existsSync()) {
    throw Exception('Unable to find $dirB');
  }

  final Set<String> filesA = _grabPngFilenames(dirA);
  final Set<String> filesB = _grabPngFilenames(dirB);
  final List<String> pairs = _findPairs(filesA, filesB);

  if (filesA.length != pairs.length || filesB.length != pairs.length) {
    returnCode = 1;
  }

  int count = 0;
  for (final String name in pairs) {
    count += 1;
    final String pathA = <String>[dirA.path, name].join(Platform.pathSeparator);
    final String pathB = <String>[dirB.path, name].join(Platform.pathSeparator);
    final String output = 'diff_$name';
    print('compare ($count / ${pairs.length}) $name');
    final ProcessResult result = Process.runSync('compare', <String>[
      '-metric',
      'RMSE',
      '-fuzz',
      '5%',
      pathA,
      pathB,
      output,
    ]);
    if (result.exitCode != 0) {
      print('DIFF FOUND: saved to $output');
      returnCode = 1;
    } else {
      File(output).deleteSync();
    }
  }
  return returnCode;
}
