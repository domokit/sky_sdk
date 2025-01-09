// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import 'host_agent.dart';
import 'utils.dart';

typedef SimulatorFunction = Future<void> Function(String deviceId);

Future<String> fileType(String pathToBinary) {
  return eval('file', <String>[pathToBinary]);
}

Future<String?> minPhoneOSVersion(String pathToBinary) async {
  final String loadCommands = await eval('otool', <String>['-l', '-arch', 'arm64', pathToBinary]);
  if (!loadCommands.contains('LC_VERSION_MIN_IPHONEOS')) {
    return null;
  }

  String? minVersion;
  // Load command 7
  // cmd LC_VERSION_MIN_IPHONEOS
  // cmdsize 16
  // version 9.0
  // sdk 15.2
  //  ...
  final List<String> lines = LineSplitter.split(loadCommands).toList();
  lines.asMap().forEach((int index, String line) {
    if (line.contains('LC_VERSION_MIN_IPHONEOS') && lines.length - index - 1 > 3) {
      final String versionLine = lines.skip(index - 1).take(4).last;
      final RegExp versionRegex = RegExp(r'\s*version\s*(\S*)');
      minVersion = versionRegex.firstMatch(versionLine)?.group(1);
    }
  });
  return minVersion;
}

/// Creates and boots a new simulator, passes the new simulator's identifier to
/// `testFunction`.
///
/// Remember to call removeIOSSimulator in the test teardown.
Future<void> testWithNewIOSSimulator(
  String deviceName,
  SimulatorFunction testFunction, {
  String deviceTypeId = 'com.apple.CoreSimulator.SimDeviceType.iPhone-11',
}) async {
  final String availableRuntimes = await eval('xcrun', <String>[
    'simctl',
    'list',
    'runtimes',
  ], workingDirectory: flutterDirectory.path);

  final String runtimesForSelectedXcode = await eval('xcrun', <String>[
    'simctl',
    'runtime',
    'match',
    'list',
    '--json',
  ], workingDirectory: flutterDirectory.path);

  // Get the preferred runtime build for the selected Xcode version. Preferred
  // means the runtime was either bundled with Xcode, exactly matched your SDK
  // version, or it's indicated a better match for your SDK.
  final Map<String, Object?> decodeResult =
      json.decode(runtimesForSelectedXcode) as Map<String, Object?>;
  final String? iosKey =
      decodeResult.keys.where((String key) => key.contains('iphoneos')).firstOrNull;
  final String? runtimeBuildForSelectedXcode = switch (decodeResult[iosKey]) {
    {'preferredBuild': final String build} => build,
    _ => null,
  };

  String? iOSSimRuntime;

  final RegExp iOSRuntimePattern = RegExp(r'iOS .*\) - (.*)');

  // [availableRuntimes] may include runtime versions greater than the selected
  // Xcode's greatest supported version. Use [runtimeBuildForSelectedXcode] when
  // possible to pick which runtime to use.
  // For example, iOS 17 (released with Xcode 15) may be available even if the
  // selected Xcode version is 14.
  for (final String runtime in LineSplitter.split(availableRuntimes)) {
    if (runtimeBuildForSelectedXcode != null && !runtime.contains(runtimeBuildForSelectedXcode)) {
      continue;
    }
    // These seem to be in order, so allow matching multiple lines so it grabs
    // the last (hopefully latest) one.
    final RegExpMatch? iOSRuntimeMatch = iOSRuntimePattern.firstMatch(runtime);
    if (iOSRuntimeMatch != null) {
      iOSSimRuntime = iOSRuntimeMatch.group(1)!.trim();
      continue;
    }
  }
  if (iOSSimRuntime == null) {
    if (runtimeBuildForSelectedXcode != null) {
      throw 'iOS simulator runtime $runtimeBuildForSelectedXcode not found. Available runtimes:\n$availableRuntimes';
    } else {
      throw 'No iOS simulator runtime found. Available runtimes:\n$availableRuntimes';
    }
  }

  final String deviceId = await eval('xcrun', <String>[
    'simctl',
    'create',
    deviceName,
    deviceTypeId,
    iOSSimRuntime,
  ], workingDirectory: flutterDirectory.path);
  await eval('xcrun', <String>[
    'simctl',
    'boot',
    deviceId,
  ], workingDirectory: flutterDirectory.path);

  await testFunction(deviceId);
}

/// Shuts down and deletes simulator with deviceId.
Future<void> removeIOSSimulator(String? deviceId) async {
  if (deviceId != null && deviceId != '') {
    await eval(
      'xcrun',
      <String>['simctl', 'shutdown', deviceId],
      canFail: true,
      workingDirectory: flutterDirectory.path,
    );
    await eval(
      'xcrun',
      <String>['simctl', 'delete', deviceId],
      canFail: true,
      workingDirectory: flutterDirectory.path,
    );
  }
}

Future<bool> runXcodeTests({
  required String platformDirectory,
  required String destination,
  required String testName,
  String configuration = 'Release',
  bool skipCodesign = false,
}) async {
  final Map<String, String> environment = Platform.environment;
  String? developmentTeam;
  String? codeSignStyle;
  String? provisioningProfile;
  if (!skipCodesign) {
    // If not running on CI, inject the Flutter team code signing properties.
    developmentTeam = environment['FLUTTER_XCODE_DEVELOPMENT_TEAM'] ?? 'S8QB4VV633';
    codeSignStyle = environment['FLUTTER_XCODE_CODE_SIGN_STYLE'];
    provisioningProfile = environment['FLUTTER_XCODE_PROVISIONING_PROFILE_SPECIFIER'];
  }
  File? disabledSandboxEntitlementFile;
  if (platformDirectory.endsWith('macos')) {
    disabledSandboxEntitlementFile = _createDisabledSandboxEntitlementFile(
      platformDirectory,
      configuration,
    );
  }
  final String resultBundleTemp = Directory.systemTemp.createTempSync('flutter_xcresult.').path;
  final String resultBundlePath = path.join(resultBundleTemp, 'result');
  final int testResultExit = await exec(
    'xcodebuild',
    <String>[
      '-workspace',
      'Runner.xcworkspace',
      '-scheme',
      'Runner',
      '-configuration',
      configuration,
      '-destination',
      destination,
      '-resultBundlePath',
      resultBundlePath,
      'test',
      'COMPILER_INDEX_STORE_ENABLE=NO',
      if (developmentTeam != null) 'DEVELOPMENT_TEAM=$developmentTeam',
      if (codeSignStyle != null) 'CODE_SIGN_STYLE=$codeSignStyle',
      if (provisioningProfile != null) 'PROVISIONING_PROFILE_SPECIFIER=$provisioningProfile',
      if (disabledSandboxEntitlementFile != null)
        'CODE_SIGN_ENTITLEMENTS=${disabledSandboxEntitlementFile.path}',
    ],
    workingDirectory: platformDirectory,
    canFail: true,
  );

  if (testResultExit != 0) {
    final Directory? dumpDirectory = hostAgent.dumpDirectory;
    final Directory xcresultBundle = Directory(path.join(resultBundleTemp, 'result.xcresult'));
    if (dumpDirectory != null) {
      if (xcresultBundle.existsSync()) {
        // Zip the test results to the artifacts directory for upload.
        final String zipPath = path.join(
          dumpDirectory.path,
          '$testName-${DateTime.now().toLocal().toIso8601String()}.zip',
        );
        await exec(
          'zip',
          <String>['-r', '-9', '-q', zipPath, path.basename(xcresultBundle.path)],
          workingDirectory: resultBundleTemp,
          canFail: true, // Best effort to get the logs.
        );
      } else {
        print('xcresult bundle ${xcresultBundle.path} does not exist, skipping upload');
      }
    }
    return false;
  }
  return true;
}

/// Finds and copies macOS entitlements file. In the copy, disables sandboxing.
/// If entitlements file is not found, returns null.
///
/// As of macOS 14, testing a macOS sandbox app may prompt the user to grant
/// access to the app. To workaround this in CI, we create and use a entitlements
/// file with sandboxing disabled. See
/// https://developer.apple.com/documentation/security/app_sandbox/accessing_files_from_the_macos_app_sandbox.
File? _createDisabledSandboxEntitlementFile(String platformDirectory, String configuration) {
  String entitlementDefaultFileName;
  if (configuration == 'Release') {
    entitlementDefaultFileName = 'Release';
  } else {
    entitlementDefaultFileName = 'DebugProfile';
  }

  final String entitlementFilePath = path.join(
    platformDirectory,
    'Runner',
    '$entitlementDefaultFileName.entitlements',
  );
  final File entitlementFile = File(entitlementFilePath);

  if (!entitlementFile.existsSync()) {
    print('Unable to find entitlements file at ${entitlementFile.path}');
    return null;
  }

  final String originalEntitlementFileContents = entitlementFile.readAsStringSync();
  final String tempEntitlementPath =
      Directory.systemTemp.createTempSync('flutter_disable_sandbox_entitlement.').path;
  final File disabledSandboxEntitlementFile = File(
    path.join(
      tempEntitlementPath,
      '${entitlementDefaultFileName}WithDisabledSandboxing.entitlements',
    ),
  );
  disabledSandboxEntitlementFile.createSync(recursive: true);
  disabledSandboxEntitlementFile.writeAsStringSync(
    originalEntitlementFileContents.replaceAll(
      RegExp(r'<key>com\.apple\.security\.app-sandbox<\/key>[\S\s]*?<true\/>'),
      '''
<key>com.apple.security.app-sandbox</key>
	<false/>''',
    ),
  );

  return disabledSandboxEntitlementFile;
}

/// Returns global (external) symbol table entries, delimited by new lines.
Future<String> dumpSymbolTable(String filePath) {
  return eval('nm', <String>['--extern-only', '--just-symbol-name', filePath, '-arch', 'arm64']);
}
