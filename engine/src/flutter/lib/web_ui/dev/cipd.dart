// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:path/path.dart' as path;

import 'utils.dart';

// Determine if a `package` tagged with version:`versionTag` already exists in CIPD.
Future<bool> cipdKnowsPackageVersion({
  required String package,
  required String versionTag,
  bool isVerbose = false,
}) async {
  // $ cipd search $package -tag version:$versionTag
  // Instances:
  //   $package:CIPD_PACKAGE_ID
  // or:
  // No matching instances.
  final String logLevel = isVerbose ? 'debug' : 'warning';
  final String stdout = await evalProcess('cipd', <String>[
    'search',
    package,
    '--tag',
    'version:$versionTag',
    '--log-level',
    logLevel,
  ]);

  return stdout.contains('Instances:') && stdout.contains(package);
}

// Runs a CIPD command to upload a package defined by its `config` file.
Future<int> uploadDirectoryToCipd({
  required io.Directory directory,
  required String packageName,
  required String configFileName,
  required String description,
  required String root,
  required String version,
  bool isDryRun = false,
  bool isVerbose = false,
}) async {
  final String cipdConfig = '''
package: $packageName
description: $description
preserve_writable: true
root: $root
data:
  - dir: .
''';

  // Create the config manifest to upload to CIPD
  final io.File configFile = io.File(path.join(directory.path, configFileName));
  await configFile.writeAsString(cipdConfig);

  final String cipdCommand = isDryRun ? 'pkg-build' : 'create';
  // CIPD won't fully shut up even in 'error' mode
  final String logLevel = isVerbose ? 'debug' : 'warning';
  return runProcess('cipd', <String>[
    cipdCommand,
    '--pkg-def',
    path.basename(configFile.path),
    '--json-output',
    '${path.basenameWithoutExtension(configFile.path)}.json',
    '--log-level',
    logLevel,
    if (!isDryRun) ...<String>['--tag', 'version:$version', '--ref', version],
    if (isDryRun) ...<String>['--out', '${path.basenameWithoutExtension(configFile.path)}.zip'],
  ], workingDirectory: directory.path);
}
