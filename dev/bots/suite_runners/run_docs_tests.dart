// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../run_command.dart';
import '../utils.dart';

Future<void> docsRunner() async {
  printProgress('${green}Running flutter doc tests$reset');
  await runCommand('./dev/bots/docs.sh', const <String>[
    '--output',
    'dev/docs/api_docs.zip',
    '--keep-staging',
    '--staging-dir',
    'dev/docs',
  ], workingDirectory: flutterRoot);
}
