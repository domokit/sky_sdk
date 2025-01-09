// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;
import 'package:process/process.dart';

import '../run_command.dart';
import '../utils.dart';

Future<void> verifyCodesignedTestRunner() async {
  printProgress('${green}Running binaries codesign verification$reset');
  await runCommand('flutter', <String>[
    'precache',
    '--android',
    '--ios',
    '--macos',
  ], workingDirectory: flutterRoot);

  await verifyExist(flutterRoot);
  await verifySignatures(flutterRoot);
}

const List<String> expectedEntitlements = <String>[
  'com.apple.security.cs.allow-jit',
  'com.apple.security.cs.allow-unsigned-executable-memory',
  'com.apple.security.cs.allow-dyld-environment-variables',
  'com.apple.security.network.client',
  'com.apple.security.network.server',
  'com.apple.security.cs.disable-library-validation',
];

/// Binaries that are expected to be codesigned and have entitlements.
///
/// This list should be kept in sync with the actual contents of Flutter's
/// cache.
List<String> binariesWithEntitlements(String flutterRoot) {
  return <String>[
    'artifacts/engine/android-arm-profile/darwin-x64/gen_snapshot',
    'artifacts/engine/android-arm-release/darwin-x64/gen_snapshot',
    'artifacts/engine/android-arm64-profile/darwin-x64/gen_snapshot',
    'artifacts/engine/android-arm64-release/darwin-x64/gen_snapshot',
    'artifacts/engine/android-x64-profile/darwin-x64/gen_snapshot',
    'artifacts/engine/android-x64-release/darwin-x64/gen_snapshot',
    'artifacts/engine/darwin-x64-profile/gen_snapshot',
    'artifacts/engine/darwin-x64-profile/gen_snapshot_arm64',
    'artifacts/engine/darwin-x64-profile/gen_snapshot_x64',
    'artifacts/engine/darwin-x64-release/gen_snapshot',
    'artifacts/engine/darwin-x64-release/gen_snapshot_arm64',
    'artifacts/engine/darwin-x64-release/gen_snapshot_x64',
    'artifacts/engine/darwin-x64/flutter_tester',
    'artifacts/engine/darwin-x64/gen_snapshot',
    'artifacts/engine/darwin-x64/gen_snapshot_arm64',
    'artifacts/engine/darwin-x64/gen_snapshot_x64',
    'artifacts/engine/ios-profile/gen_snapshot_arm64',
    'artifacts/engine/ios-release/gen_snapshot_arm64',
    'artifacts/engine/ios/gen_snapshot_arm64',
    'artifacts/libimobiledevice/idevicescreenshot',
    'artifacts/libimobiledevice/idevicesyslog',
    'artifacts/libimobiledevice/libimobiledevice-1.0.6.dylib',
    'artifacts/libplist/libplist-2.0.3.dylib',
    'artifacts/openssl/libcrypto.1.1.dylib',
    'artifacts/openssl/libssl.1.1.dylib',
    'artifacts/usbmuxd/iproxy',
    'artifacts/usbmuxd/libusbmuxd-2.0.6.dylib',
    'dart-sdk/bin/dart',
    'dart-sdk/bin/dartaotruntime',
    'dart-sdk/bin/utils/gen_snapshot',
    'dart-sdk/bin/utils/wasm-opt',
  ].map((String relativePath) => path.join(flutterRoot, 'bin', 'cache', relativePath)).toList();
}

/// Binaries that are only expected to be codesigned.
///
/// This list should be kept in sync with the actual contents of Flutter's
/// cache.
List<String> binariesWithoutEntitlements(String flutterRoot) {
  return <String>[
    'artifacts/engine/darwin-x64-profile/FlutterMacOS.xcframework/macos-arm64_x86_64/FlutterMacOS.framework/Versions/A/FlutterMacOS',
    'artifacts/engine/darwin-x64-release/FlutterMacOS.xcframework/macos-arm64_x86_64/FlutterMacOS.framework/Versions/A/FlutterMacOS',
    'artifacts/engine/darwin-x64/FlutterMacOS.xcframework/macos-arm64_x86_64/FlutterMacOS.framework/Versions/A/FlutterMacOS',
    'artifacts/engine/darwin-x64/font-subset',
    'artifacts/engine/darwin-x64/impellerc',
    'artifacts/engine/darwin-x64/libpath_ops.dylib',
    'artifacts/engine/darwin-x64/libtessellator.dylib',
    'artifacts/engine/ios-profile/Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
    'artifacts/engine/ios-profile/Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
    'artifacts/engine/ios-profile/extension_safe/Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
    'artifacts/engine/ios-profile/extension_safe/Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
    'artifacts/engine/ios-release/Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
    'artifacts/engine/ios-release/Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
    'artifacts/engine/ios-release/extension_safe/Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
    'artifacts/engine/ios-release/extension_safe/Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
    'artifacts/engine/ios/Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
    'artifacts/engine/ios/Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
    'artifacts/engine/ios/extension_safe/Flutter.xcframework/ios-arm64/Flutter.framework/Flutter',
    'artifacts/engine/ios/extension_safe/Flutter.xcframework/ios-arm64_x86_64-simulator/Flutter.framework/Flutter',
    'artifacts/ios-deploy/ios-deploy',
  ].map((String relativePath) => path.join(flutterRoot, 'bin', 'cache', relativePath)).toList();
}

/// Binaries that are not expected to be codesigned.
///
/// This list should be kept in sync with the actual contents of Flutter's cache.
List<String> unsignedBinaries(String flutterRoot) {
  return <String>[
    'artifacts/engine/darwin-x64-release/FlutterMacOS.xcframework/macos-arm64_x86_64/dSYMs/FlutterMacOS.framework.dSYM/Contents/Resources/DWARF/FlutterMacOS',
    'artifacts/engine/ios-release/Flutter.xcframework/ios-arm64/dSYMs/Flutter.framework.dSYM/Contents/Resources/DWARF/Flutter',
    'artifacts/engine/ios-release/Flutter.xcframework/ios-arm64_x86_64-simulator/dSYMs/Flutter.framework.dSYM/Contents/Resources/DWARF/Flutter',
    'artifacts/engine/ios-release/extension_safe/Flutter.xcframework/ios-arm64/dSYMs/Flutter.framework.dSYM/Contents/Resources/DWARF/Flutter',
    'artifacts/engine/ios-release/extension_safe/Flutter.xcframework/ios-arm64_x86_64-simulator/dSYMs/Flutter.framework.dSYM/Contents/Resources/DWARF/Flutter',
  ].map((String relativePath) => path.join(flutterRoot, 'bin', 'cache', relativePath)).toList();
}

/// xcframeworks that are expected to be codesigned.
///
/// This list should be kept in sync with the actual contents of Flutter's
/// cache.
List<String> signedXcframeworks(String flutterRoot) {
  return <String>[
    'artifacts/engine/ios-profile/Flutter.xcframework',
    'artifacts/engine/ios-profile/extension_safe/Flutter.xcframework',
    'artifacts/engine/ios-release/Flutter.xcframework',
    'artifacts/engine/ios-release/extension_safe/Flutter.xcframework',
    'artifacts/engine/ios/Flutter.xcframework',
    'artifacts/engine/ios/extension_safe/Flutter.xcframework',
    'artifacts/engine/darwin-x64-profile/FlutterMacOS.xcframework',
    'artifacts/engine/darwin-x64-release/FlutterMacOS.xcframework',
    'artifacts/engine/darwin-x64/FlutterMacOS.xcframework',
  ].map((String relativePath) => path.join(flutterRoot, 'bin', 'cache', relativePath)).toList();
}

/// Verify the existence of all expected binaries in cache.
///
/// This function ignores code signatures and entitlements, and is intended to
/// be run on every commit. It should throw if either new binaries are added
/// to the cache or expected binaries removed. In either case, this class'
/// [binariesWithEntitlements], [binariesWithoutEntitlements], and
/// [unsignedBinaries] lists should be updated accordingly.
Future<void> verifyExist(
  String flutterRoot, {
  @visibleForTesting ProcessManager processManager = const LocalProcessManager(),
}) async {
  final List<String> binaryPaths = await findBinaryPaths(
    path.join(flutterRoot, 'bin', 'cache'),
    processManager: processManager,
  );
  final List<String> expectedSigned =
      binariesWithEntitlements(flutterRoot) + binariesWithoutEntitlements(flutterRoot);
  final List<String> expectedUnsigned = unsignedBinaries(flutterRoot);
  final Set<String> foundFiles = <String>{
    for (final String binaryPath in binaryPaths)
      if (expectedSigned.contains(binaryPath))
        binaryPath
      else if (expectedUnsigned.contains(binaryPath))
        binaryPath
      else
        throw Exception('Found unexpected binary in cache: $binaryPath'),
  };

  if (foundFiles.length < expectedSigned.length) {
    final List<String> unfoundFiles = <String>[
      for (final String file in expectedSigned)
        if (!foundFiles.contains(file)) file,
    ];
    print(
      'Expected binaries not found in cache:\n\n${unfoundFiles.join('\n')}\n\n'
      'If this commit is removing binaries from the cache, this test should be fixed by\n'
      'removing the relevant entry from either the "binariesWithEntitlements" or\n'
      '"binariesWithoutEntitlements" getters in dev/tools/lib/codesign.dart.',
    );
    throw Exception('Did not find all expected binaries!');
  }

  print('All expected binaries present.');
}

/// Verify code signatures and entitlements of all binaries in the cache.
Future<void> verifySignatures(
  String flutterRoot, {
  @visibleForTesting ProcessManager processManager = const LocalProcessManager(),
}) async {
  final List<String> unsignedFiles = <String>[];
  final List<String> wrongEntitlementBinaries = <String>[];
  final List<String> unexpectedFiles = <String>[];
  final String cacheDirectory = path.join(flutterRoot, 'bin', 'cache');

  final List<String> binariesAndXcframeworks =
      (await findBinaryPaths(cacheDirectory, processManager: processManager)) +
      (await findXcframeworksPaths(cacheDirectory, processManager: processManager));

  for (final String pathToCheck in binariesAndXcframeworks) {
    bool verifySignature = false;
    bool verifyEntitlements = false;
    if (binariesWithEntitlements(flutterRoot).contains(pathToCheck)) {
      verifySignature = true;
      verifyEntitlements = true;
    }
    if (binariesWithoutEntitlements(flutterRoot).contains(pathToCheck)) {
      verifySignature = true;
    }
    if (signedXcframeworks(flutterRoot).contains(pathToCheck)) {
      verifySignature = true;
    }
    if (unsignedBinaries(flutterRoot).contains(pathToCheck)) {
      // Binary is expected to be unsigned. No need to check signature, entitlements.
      continue;
    }

    if (!verifySignature && !verifyEntitlements) {
      unexpectedFiles.add(pathToCheck);
      print('Unexpected binary or xcframework $pathToCheck found in cache!');
      continue;
    }
    print('Verifying the code signature of $pathToCheck');
    final io.ProcessResult codeSignResult = await processManager.run(<String>[
      'codesign',
      '-vvv',
      pathToCheck,
    ]);
    if (codeSignResult.exitCode != 0) {
      unsignedFiles.add(pathToCheck);
      print(
        'File "$pathToCheck" does not appear to be codesigned.\n'
        'The `codesign` command failed with exit code ${codeSignResult.exitCode}:\n'
        '${codeSignResult.stderr}\n',
      );
      continue;
    }
    if (verifyEntitlements) {
      print('Verifying entitlements of $pathToCheck');
      if (!(await hasExpectedEntitlements(
        pathToCheck,
        flutterRoot,
        processManager: processManager,
      ))) {
        wrongEntitlementBinaries.add(pathToCheck);
      }
    }
  }

  // First print all deviations from expectations
  if (unsignedFiles.isNotEmpty) {
    print('Found ${unsignedFiles.length} unsigned files:');
    unsignedFiles.forEach(print);
  }

  if (wrongEntitlementBinaries.isNotEmpty) {
    print('Found ${wrongEntitlementBinaries.length} files with unexpected entitlements:');
    wrongEntitlementBinaries.forEach(print);
  }

  if (unexpectedFiles.isNotEmpty) {
    print('Found ${unexpectedFiles.length} unexpected files in the cache:');
    unexpectedFiles.forEach(print);
  }

  // Finally, exit on any invalid state
  if (unsignedFiles.isNotEmpty) {
    throw Exception('Test failed because unsigned files detected.');
  }

  if (wrongEntitlementBinaries.isNotEmpty) {
    throw Exception(
      'Test failed because files found with the wrong entitlements:\n'
      '${wrongEntitlementBinaries.join('\n')}',
    );
  }

  if (unexpectedFiles.isNotEmpty) {
    throw Exception('Test failed because unexpected files found in the cache.');
  }
  print('Verified that files are codesigned and have expected entitlements.');
}

/// Find every binary file in the given [rootDirectory].
Future<List<String>> findBinaryPaths(
  String rootDirectory, {
  @visibleForTesting ProcessManager processManager = const LocalProcessManager(),
}) async {
  final List<String> allBinaryPaths = <String>[];
  final io.ProcessResult result = await processManager.run(<String>[
    'find',
    rootDirectory,
    '-type',
    'f',
  ]);
  final List<String> allFiles =
      (result.stdout as String).split('\n').where((String s) => s.isNotEmpty).toList();

  await Future.forEach(allFiles, (String filePath) async {
    if (await isBinary(filePath, processManager: processManager)) {
      allBinaryPaths.add(filePath);
      print('Found: $filePath\n');
    }
  });
  return allBinaryPaths;
}

/// Find every xcframework in the given [rootDirectory].
Future<List<String>> findXcframeworksPaths(
  String rootDirectory, {
  @visibleForTesting ProcessManager processManager = const LocalProcessManager(),
}) async {
  final io.ProcessResult result = await processManager.run(<String>[
    'find',
    rootDirectory,
    '-type',
    'd',
    '-name',
    '*xcframework',
  ]);
  final List<String> allXcframeworkPaths =
      LineSplitter.split(result.stdout as String).where((String s) => s.isNotEmpty).toList();
  for (final String path in allXcframeworkPaths) {
    print('Found: $path\n');
  }
  return allXcframeworkPaths;
}

/// Check mime-type of file at [filePath] to determine if it is binary.
Future<bool> isBinary(
  String filePath, {
  @visibleForTesting ProcessManager processManager = const LocalProcessManager(),
}) async {
  final io.ProcessResult result = await processManager.run(<String>[
    'file',
    '--mime-type',
    '-b', // is binary
    filePath,
  ]);
  return (result.stdout as String).contains('application/x-mach-binary');
}

/// Check if the binary has the expected entitlements.
Future<bool> hasExpectedEntitlements(
  String binaryPath,
  String flutterRoot, {
  @visibleForTesting ProcessManager processManager = const LocalProcessManager(),
}) async {
  final io.ProcessResult entitlementResult = await processManager.run(<String>[
    'codesign',
    '--display',
    '--entitlements',
    ':-',
    binaryPath,
  ]);

  if (entitlementResult.exitCode != 0) {
    print(
      'The `codesign --entitlements` command failed with exit code ${entitlementResult.exitCode}:\n'
      '${entitlementResult.stderr}\n',
    );
    return false;
  }

  bool passes = true;
  final String output = entitlementResult.stdout as String;
  for (final String entitlement in expectedEntitlements) {
    final bool entitlementExpected = binariesWithEntitlements(flutterRoot).contains(binaryPath);
    if (output.contains(entitlement) != entitlementExpected) {
      print(
        'File "$binaryPath" ${entitlementExpected ? 'does not have expected' : 'has unexpected'} '
        'entitlement $entitlement.',
      );
      passes = false;
    }
  }
  return passes;
}
