// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show utf8;
import 'dart:io';

String _basename(String path) {
  return path.split(Platform.pathSeparator).last;
}

String _generateDirListing(String dirPath) {
  final Directory dir = Directory(dirPath);
  final List<FileSystemEntity> entities = dir.listSync();
  entities.sort((FileSystemEntity a, FileSystemEntity b) => a.path.compareTo(b.path));
  return entities.map((FileSystemEntity entity) => _basename(entity.path)).join('\n');
}

String _strReplaceRange(String inputStr, int start, int end, String replacement) {
  return inputStr.substring(0, start) + replacement + inputStr.substring(end);
}

String _redirectPatch(String patch) {
  final RegExp inputPathExp = RegExp(r'^--- a(.*)', multiLine: true);
  final RegExp outputPathExp = RegExp(r'^\+\+\+ b(.*)', multiLine: true);

  final Match? inputPathMatch = inputPathExp.firstMatch(patch);
  final Match? outputPathMatch = outputPathExp.firstMatch(patch);

  assert(inputPathMatch != null);
  assert(outputPathMatch != null);

  if (inputPathMatch != null && outputPathMatch != null) {
    return _strReplaceRange(
      patch,
      outputPathMatch.start + 5, // +5 to account for '+++ b'
      outputPathMatch.end,
      inputPathMatch.group(1)!,
    );
  }
  throw Exception('Unable to find input and output paths');
}

File _makeTempFile(String prefix) {
  final Directory systemTempDir = Directory.systemTemp;
  final String filename = '$prefix-${DateTime.now().millisecondsSinceEpoch}';
  final String path = '${systemTempDir.path}${Platform.pathSeparator}$filename';
  final File result = File(path);
  result.createSync();
  return result;
}

/// Run the diff of the contents of a directory at [dirPath] and the contents of
/// a file at [goldenPath].  Returns 0 if there is no diff. Be aware that the
/// CWD should be inside of the git repository for the patch to be correct.
int dirContentsDiff(String goldenPath, String dirPath) {
  if (!File(goldenPath).existsSync()) {
    throw Exception('unable to find `$goldenPath`');
  }
  if (!Directory(dirPath).existsSync()) {
    throw Exception('unable to find `$dirPath`');
  }
  int result = 0;
  final File tempFile = _makeTempFile('dir_contents_diff');
  try {
    final String dirListing = _generateDirListing(dirPath);
    tempFile.writeAsStringSync(dirListing);
    final ProcessResult diffResult = Process.runSync(
      'git',
      <String>[
        'diff',
        // If you manually edit the golden file, many text editors will add
        // trailing whitespace. This flag ignores that because honestly it's
        // not a significant part of this test.
        '--ignore-space-at-eol',
        '-p',
        goldenPath,
        tempFile.path,
      ],
      runInShell: true,
      stdoutEncoding: utf8,
    );
    if (diffResult.exitCode != 0) {
      print('Unexpected diff in $goldenPath, use `git apply` with the following patch.\n');
      print(_redirectPatch(diffResult.stdout as String));
      result = 1;
    }
  } finally {
    tempFile.deleteSync();
  }

  return result;
}

/// The main entrypoint for the program, returns `exitCode`.
int run(List<String> args) {
  if (args.length != 2) {
    throw Exception('usage: <path to golden> <path to directory>');
  }
  final String goldenPath = args[0];
  final String dirPath = args[1];
  return dirContentsDiff(goldenPath, dirPath);
}
