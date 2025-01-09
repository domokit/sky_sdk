// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../src/common.dart';
import 'test_utils.dart';

const String xcodeBackendPath = 'bin/xcode_backend.sh';
const String xcodeBackendErrorHeader =
    '========================================================================';

// Acceptable $CONFIGURATION/$FLUTTER_BUILD_MODE values should be debug, profile, or release
const Map<String, String> unknownConfiguration = <String, String>{'CONFIGURATION': 'Custom'};

// $FLUTTER_BUILD_MODE will override $CONFIGURATION
const Map<String, String> unknownFlutterBuildMode = <String, String>{
  'FLUTTER_BUILD_MODE': 'Custom',
  'CONFIGURATION': 'Debug',
};

void main() {
  Future<void> expectXcodeBackendFails(Map<String, String> environment) async {
    final ProcessResult result = await Process.run(xcodeBackendPath, <String>[
      'build',
    ], environment: environment);
    expect(result.stderr, startsWith(xcodeBackendErrorHeader));
    expect(result.exitCode, isNot(0));
  }

  test('Xcode backend fails with no arguments', () async {
    final ProcessResult result = await Process.run(
      xcodeBackendPath,
      <String>[],
      environment: <String, String>{
        'SOURCE_ROOT': '../examples/hello_world',
        'FLUTTER_ROOT': '../..',
      },
    );
    expect(
      result.stderr,
      startsWith('error: Your Xcode project is incompatible with this version of Flutter.'),
    );
    expect(result.exitCode, isNot(0));
  }, skip: !io.Platform.isMacOS); // [intended] requires macos toolchain.

  test('Xcode backend fails for on unsupported configuration combinations', () async {
    await expectXcodeBackendFails(unknownConfiguration);
    await expectXcodeBackendFails(unknownFlutterBuildMode);
  }, skip: !io.Platform.isMacOS); // [intended] requires macos toolchain.

  test('Xcode backend warns archiving a non-release build.', () async {
    final ProcessResult result = await Process.run(
      xcodeBackendPath,
      <String>['build'],
      environment: <String, String>{'CONFIGURATION': 'Debug', 'ACTION': 'install'},
    );
    expect(result.stdout, contains('warning: Flutter archive not built in Release mode.'));
    expect(result.exitCode, isNot(0));
  }, skip: !io.Platform.isMacOS); // [intended] requires macos toolchain.

  group('vmService Bonjour service keys', () {
    late Directory buildDirectory;
    late File infoPlist;

    setUp(() {
      buildDirectory = globals.fs.systemTempDirectory.createTempSync(
        'flutter_tools_xcode_backend_test.',
      );
      infoPlist = buildDirectory.childFile('Info.plist');
    });

    test('handles when the Info.plist is missing', () async {
      final ProcessResult result = await Process.run(
        xcodeBackendPath,
        <String>['test_vm_service_bonjour_service'],
        environment: <String, String>{
          'CONFIGURATION': 'Debug',
          'BUILT_PRODUCTS_DIR': buildDirectory.path,
          'INFOPLIST_PATH': 'Info.plist',
        },
      );
      expect(result, const ProcessResultMatcher(stdoutPattern: 'Info.plist does not exist.'));
    });

    const String emptyPlist = '''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
</dict>
</plist>''';

    test('does not add keys in Release', () async {
      infoPlist.writeAsStringSync(emptyPlist);

      final ProcessResult result = await Process.run(
        xcodeBackendPath,
        <String>['test_vm_service_bonjour_service'],
        environment: <String, String>{
          'CONFIGURATION': 'Release',
          'BUILT_PRODUCTS_DIR': buildDirectory.path,
          'INFOPLIST_PATH': 'Info.plist',
        },
      );

      final String actualInfoPlist = infoPlist.readAsStringSync();
      expect(actualInfoPlist, isNot(contains('NSBonjourServices')));
      expect(actualInfoPlist, isNot(contains('dartVmService')));
      expect(actualInfoPlist, isNot(contains('NSLocalNetworkUsageDescription')));

      expect(result, const ProcessResultMatcher());
    });

    for (final String buildConfiguration in <String>['Debug', 'Profile']) {
      test('add keys in $buildConfiguration', () async {
        infoPlist.writeAsStringSync(emptyPlist);

        final ProcessResult result = await Process.run(
          xcodeBackendPath,
          <String>['test_vm_service_bonjour_service'],
          environment: <String, String>{
            'CONFIGURATION': buildConfiguration,
            'BUILT_PRODUCTS_DIR': buildDirectory.path,
            'INFOPLIST_PATH': 'Info.plist',
          },
        );

        final String actualInfoPlist = infoPlist.readAsStringSync();
        expect(actualInfoPlist, contains('NSBonjourServices'));
        expect(actualInfoPlist, contains('dartVmService'));
        expect(actualInfoPlist, contains('NSLocalNetworkUsageDescription'));

        expect(result, const ProcessResultMatcher());
      });
    }

    test(
      'adds to existing Bonjour services, does not override network usage description',
      () async {
        infoPlist.writeAsStringSync('''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>NSBonjourServices</key>
	<array>
		<string>_bogus._tcp</string>
	</array>
	<key>NSLocalNetworkUsageDescription</key>
	<string>Don't override this</string>
</dict>
</plist>''');

        final ProcessResult result = await Process.run(
          xcodeBackendPath,
          <String>['test_vm_service_bonjour_service'],
          environment: <String, String>{
            'CONFIGURATION': 'Debug',
            'BUILT_PRODUCTS_DIR': buildDirectory.path,
            'INFOPLIST_PATH': 'Info.plist',
          },
        );

        expect(infoPlist.readAsStringSync(), '''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>NSBonjourServices</key>
	<array>
		<string>_dartVmService._tcp</string>
		<string>_bogus._tcp</string>
	</array>
	<key>NSLocalNetworkUsageDescription</key>
	<string>Don't override this</string>
</dict>
</plist>
''');
        expect(result, const ProcessResultMatcher());
      },
    );

    test('does not add bonjour settings when port publication is disabled', () async {
      infoPlist.writeAsStringSync('''
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
</dict>
</plist>''');

      final ProcessResult result = await Process.run(
        xcodeBackendPath,
        <String>['test_vm_service_bonjour_service'],
        environment: <String, String>{
          'CONFIGURATION': 'Debug',
          'BUILT_PRODUCTS_DIR': buildDirectory.path,
          'INFOPLIST_PATH': 'Info.plist',
          'DISABLE_PORT_PUBLICATION': 'YES',
        },
      );

      expect(infoPlist.readAsStringSync().contains('NSBonjourServices'), isFalse);
      expect(infoPlist.readAsStringSync().contains('NSLocalNetworkUsageDescription'), isFalse);
      expect(result, const ProcessResultMatcher());
    });
  }, skip: !io.Platform.isMacOS); // [intended] requires macos toolchain.
}
