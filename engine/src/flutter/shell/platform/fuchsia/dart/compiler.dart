// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:args/args.dart';
import 'package:crypto/crypto.dart';

import 'package:vm/kernel_front_end.dart'
    show createCompilerArgParser, runCompiler, successExitCode;

final ArgParser _argParser =
    createCompilerArgParser()..addFlag(
      'train',
      help: 'Run through sample command line to produce snapshot',
      negatable: false,
    );

String _usage = '''
Usage: compiler [options] input.dart

Options:
${_argParser.usage}
''';

Future<void> main(List<String> args) async {
  ArgResults options;
  try {
    options = _argParser.parse(args);

    if (options['train']) {
      final Directory temp = Directory.systemTemp.createTempSync('train_kernel_compiler');
      try {
        options = _argParser.parse(<String>['--manifest=flutter', '--data-dir=${temp.absolute}']);

        await runCompiler(options, _usage);
        return;
      } finally {
        temp.deleteSync(recursive: true);
      }
    }

    if (!options.rest.isNotEmpty) {
      throw Exception('Must specify input.dart');
    }
  } on Exception catch (error) {
    print('ERROR: $error\n');
    print(_usage);
    exitCode = 1;
    return;
  }

  final compilerExitCode = await runCompiler(options, _usage);
  if (compilerExitCode != successExitCode) {
    exitCode = compilerExitCode;
    return;
  }
}
