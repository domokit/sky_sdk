// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/create.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/doctor.dart';
import 'package:flutter_tools/src/doctor_validator.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_flutter_command_runner.dart';
import '../../src/testbed.dart';

class FakePub extends Fake implements Pub {
  int calledGetOffline = 0;
  int calledOnline = 0;

  @override
  Future<void> get({
    PubContext? context,
    required FlutterProject project,
    bool upgrade = false,
    bool offline = false,
    bool generateSyntheticPackage = false,
    bool generateSyntheticPackageForExample = false,
    String? flutterRootOverride,
    bool checkUpToDate = false,
    bool shouldSkipThirdPartyGenerator = true,
    PubOutputMode outputMode = PubOutputMode.all,
  }) async {
    project.directory
        .childDirectory('.dart_tool')
        .childFile('package_config.json')
        .createSync(recursive: true);
    if (offline) {
      calledGetOffline += 1;
    } else {
      calledOnline += 1;
    }
  }
}

void main() {
  group('usageValues', () {
    late Testbed testbed;
    late FakePub fakePub;

    setUpAll(() {
      Cache.disableLocking();
      Cache.flutterRoot = 'flutter';
    });

    setUp(() {
      testbed = Testbed(
        setup: () {
          fakePub = FakePub();
          Cache.flutterRoot = 'flutter';
          final List<String> filePaths = <String>[
            globals.fs.path.join('flutter', 'packages', 'flutter', 'pubspec.yaml'),
            globals.fs.path.join('flutter', 'packages', 'flutter_driver', 'pubspec.yaml'),
            globals.fs.path.join('flutter', 'packages', 'flutter_test', 'pubspec.yaml'),
            globals.fs.path.join(
              'flutter',
              'bin',
              'cache',
              'artifacts',
              'gradle_wrapper',
              'wrapper',
            ),
            globals.fs.path.join('usr', 'local', 'bin', 'adb'),
            globals.fs.path.join('Android', 'platform-tools', 'adb.exe'),
          ];
          for (final String filePath in filePaths) {
            globals.fs.file(filePath).createSync(recursive: true);
          }
          final List<String> templatePaths = <String>[
            globals.fs.path.join('flutter', 'packages', 'flutter_tools', 'templates', 'app'),
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              'templates',
              'app_integration_test',
            ),
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              'templates',
              'app_test_widget',
            ),
            globals.fs.path.join('flutter', 'packages', 'flutter_tools', 'templates', 'cocoapods'),
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              'templates',
              'module',
              'common',
            ),
            globals.fs.path.join('flutter', 'packages', 'flutter_tools', 'templates', 'package'),
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              'templates',
              'package_ffi',
            ),
            globals.fs.path.join('flutter', 'packages', 'flutter_tools', 'templates', 'plugin'),
            globals.fs.path.join('flutter', 'packages', 'flutter_tools', 'templates', 'plugin_ffi'),
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              'templates',
              'plugin_shared',
            ),
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              'templates',
              'plugin_cocoapods',
            ),
          ];
          for (final String templatePath in templatePaths) {
            globals.fs.directory(templatePath).createSync(recursive: true);
          }
          // Set up enough of the packages to satisfy the templating code.
          final File packagesFile = globals.fs.file(
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              '.dart_tool',
              'package_config.json',
            ),
          );
          final File flutterManifest = globals.fs.file(
            globals.fs.path.join(
              'flutter',
              'packages',
              'flutter_tools',
              'templates',
              'template_manifest.json',
            ),
          )..createSync(recursive: true);
          final Directory templateImagesDirectory = globals.fs.directory('flutter_template_images');
          templateImagesDirectory.createSync(recursive: true);
          packagesFile.createSync(recursive: true);
          packagesFile.writeAsStringSync(
            json.encode(<String, Object>{
              'configVersion': 2,
              'packages': <Object>[
                <String, Object>{
                  'name': 'flutter_template_images',
                  'languageVersion': '2.8',
                  'rootUri': templateImagesDirectory.uri.toString(),
                  'packageUri': 'lib/',
                },
              ],
            }),
          );
          flutterManifest.writeAsStringSync('{"files":[]}');
        },
        overrides: <Type, Generator>{
          DoctorValidatorsProvider: () => FakeDoctorValidatorsProvider(),
          FeatureFlags: () => TestFeatureFlags(isNativeAssetsEnabled: true),
        },
      );
    });

    testUsingContext(
      'set template type as usage value',
      () => testbed.run(() async {
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);

        await runner.run(<String>['create', '--no-pub', '--template=module', 'testy']);
        expect((await command.usageValues).commandCreateProjectType, 'module');

        await runner.run(<String>['create', '--no-pub', '--template=app', 'testy1']);
        expect((await command.usageValues).commandCreateProjectType, 'app');

        await runner.run(<String>['create', '--no-pub', '--template=package', 'testy3']);
        expect((await command.usageValues).commandCreateProjectType, 'package');

        await runner.run(<String>['create', '--no-pub', '--template=plugin', 'testy4']);
        expect((await command.usageValues).commandCreateProjectType, 'plugin');

        await runner.run(<String>['create', '--no-pub', '--template=plugin_ffi', 'testy5']);
        expect((await command.usageValues).commandCreateProjectType, 'plugin_ffi');

        await runner.run(<String>['create', '--no-pub', '--template=package_ffi', 'testy6']);
        expect((await command.usageValues).commandCreateProjectType, 'package_ffi');
      }),
      overrides: <Type, Generator>{Java: () => FakeJava()},
    );

    testUsingContext(
      'set iOS host language type as usage value',
      () => testbed.run(() async {
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);

        await runner.run(<String>['create', '--no-pub', '--template=app', 'testy']);
        expect((await command.usageValues).commandCreateIosLanguage, 'swift');

        await runner.run(<String>[
          'create',
          '--no-pub',
          '--template=app',
          '--ios-language=objc',
          'testy',
        ]);
        expect((await command.usageValues).commandCreateIosLanguage, 'objc');
      }),
      overrides: <Type, Generator>{Java: () => FakeJava()},
    );

    testUsingContext(
      'set Android host language type as usage value',
      () => testbed.run(() async {
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);

        await runner.run(<String>['create', '--no-pub', '--template=app', 'testy']);
        expect((await command.usageValues).commandCreateAndroidLanguage, 'kotlin');

        await runner.run(<String>[
          'create',
          '--no-pub',
          '--template=app',
          '--android-language=java',
          'testy',
        ]);
        expect((await command.usageValues).commandCreateAndroidLanguage, 'java');
      }),
      overrides: <Type, Generator>{Java: () => FakeJava()},
    );

    testUsingContext(
      'create --offline',
      () => testbed.run(() async {
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);
        await runner.run(<String>['create', 'testy', '--offline']);
        expect(fakePub.calledOnline, 0);
        expect(fakePub.calledGetOffline, 1);
        expect(command.argParser.options.containsKey('offline'), true);
        expect(command.shouldUpdateCache, true);
      }, overrides: <Type, Generator>{Java: () => null, Pub: () => fakePub}),
    );

    testUsingContext(
      'package_ffi template not enabled',
      () async {
        final CreateCommand command = CreateCommand();
        final CommandRunner<void> runner = createTestCommandRunner(command);

        expect(
          runner.run(<String>['create', '--no-pub', '--template=package_ffi', 'my_ffi_package']),
          throwsUsageException(
            message: '"package_ffi" is not an allowed value for option "--template"',
          ),
        );
      },
      overrides: <Type, Generator>{
        FeatureFlags:
            () => TestFeatureFlags(
              isNativeAssetsEnabled:
                  false, // ignore: avoid_redundant_argument_values, If we graduate the feature to true by default, don't break this test.
            ),
      },
    );
  });
}

class FakeDoctorValidatorsProvider implements DoctorValidatorsProvider {
  @override
  List<DoctorValidator> get validators => <DoctorValidator>[];

  @override
  List<Workflow> get workflows => <Workflow>[];
}
