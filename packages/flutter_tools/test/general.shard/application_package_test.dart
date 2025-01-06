// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/application_package.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/ios/application_package.dart';
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart';

void main() {
  group('Apk with partial Android SDK works', () {
    late FakeAndroidSdk sdk;
    late FakeProcessManager fakeProcessManager;
    late MemoryFileSystem fs;
    late Cache cache;

    final Map<Type, Generator> overrides = <Type, Generator>{
      AndroidSdk: () => sdk,
      ProcessManager: () => fakeProcessManager,
      FileSystem: () => fs,
      Cache: () => cache,
    };

    setUp(() async {
      sdk = FakeAndroidSdk();
      fakeProcessManager = FakeProcessManager.empty();
      fs = MemoryFileSystem.test();
      cache = Cache.test(processManager: FakeProcessManager.any());
      Cache.flutterRoot = '../..';
      sdk.licensesAvailable = true;
      final FlutterProject project = FlutterProject.fromDirectoryTest(fs.currentDirectory);
      fs
          .file(
            project.android.hostAppGradleRoot
                .childFile(globals.platform.isWindows ? 'gradlew.bat' : 'gradlew')
                .path,
          )
          .createSync(recursive: true);
    });

    testUsingContext('correct debug filename in module projects', () async {
      const String aaptPath = 'aaptPath';
      final File apkFile = globals.fs.file('app-debug.apk');
      final FakeAndroidSdkVersion sdkVersion = FakeAndroidSdkVersion();
      sdkVersion.aaptPath = aaptPath;
      sdk.latestVersion = sdkVersion;
      sdk.platformToolsAvailable = true;
      sdk.licensesAvailable = false;

      fakeProcessManager.addCommand(
        FakeCommand(
          command: <String>[aaptPath, 'dump', 'xmltree', apkFile.path, 'AndroidManifest.xml'],
          stdout: _aaptDataWithDefaultEnabledAndMainLauncherActivity,
        ),
      );

      fakeProcessManager.addCommand(
        FakeCommand(
          command: <String>[
            aaptPath,
            'dump',
            'xmltree',
            fs.path.join(
              'module_project',
              'build',
              'host',
              'outputs',
              'apk',
              'debug',
              'app-debug.apk',
            ),
            'AndroidManifest.xml',
          ],
          stdout: _aaptDataWithDefaultEnabledAndMainLauncherActivity,
        ),
      );

      await ApplicationPackageFactory.instance!.getPackageForPlatform(
        TargetPlatform.android_arm,
        applicationBinary: apkFile,
      );
      final BufferLogger logger = BufferLogger.test();
      final FlutterProject project = await aModuleProject();
      project.android.hostAppGradleRoot.childFile('build.gradle').createSync(recursive: true);
      final File appGradle = project.android.hostAppGradleRoot.childFile(
        fs.path.join('app', 'build.gradle'),
      );
      appGradle.createSync(recursive: true);
      appGradle.writeAsStringSync("def flutterPluginVersion = 'managed'");
      final File apkDebugFile = project.directory
          .childDirectory('build')
          .childDirectory('host')
          .childDirectory('outputs')
          .childDirectory('apk')
          .childDirectory('debug')
          .childFile('app-debug.apk');
      apkDebugFile.createSync(recursive: true);
      final AndroidApk? androidApk = await AndroidApk.fromAndroidProject(
        project.android,
        androidSdk: sdk,
        processManager: fakeProcessManager,
        userMessages: UserMessages(),
        processUtils: ProcessUtils(processManager: fakeProcessManager, logger: logger),
        logger: logger,
        fileSystem: fs,
        buildInfo: const BuildInfo(
          BuildMode.debug,
          null,
          treeShakeIcons: false,
          packageConfigPath: '.dart_tool/package_config.json',
        ),
      );
      expect(androidApk, isNotNull);
    }, overrides: overrides);

    testUsingContext(
      'Licenses not available, platform and buildtools available, apk exists',
      () async {
        const String aaptPath = 'aaptPath';
        final File apkFile = globals.fs.file('app-debug.apk');
        final FakeAndroidSdkVersion sdkVersion = FakeAndroidSdkVersion();
        sdkVersion.aaptPath = aaptPath;
        sdk.latestVersion = sdkVersion;
        sdk.platformToolsAvailable = true;
        sdk.licensesAvailable = false;

        fakeProcessManager.addCommand(
          FakeCommand(
            command: <String>[aaptPath, 'dump', 'xmltree', apkFile.path, 'AndroidManifest.xml'],
            stdout: _aaptDataWithDefaultEnabledAndMainLauncherActivity,
          ),
        );

        final ApplicationPackage applicationPackage =
            (await ApplicationPackageFactory.instance!.getPackageForPlatform(
              TargetPlatform.android_arm,
              applicationBinary: apkFile,
            ))!;
        expect(applicationPackage.name, 'app-debug.apk');
        expect(applicationPackage, isA<PrebuiltApplicationPackage>());
        expect(
          (applicationPackage as PrebuiltApplicationPackage).applicationPackage.path,
          apkFile.path,
        );
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: overrides,
    );

    testUsingContext('Licenses available, build tools not, apk exists', () async {
      sdk.latestVersion = null;
      final FlutterProject project = FlutterProject.fromDirectoryTest(fs.currentDirectory);
      project.android.hostAppGradleRoot
          .childFile('gradle.properties')
          .writeAsStringSync('irrelevant');

      final Directory gradleWrapperDir = cache.getArtifactDirectory('gradle_wrapper');

      gradleWrapperDir.fileSystem
          .directory(gradleWrapperDir.childDirectory('gradle').childDirectory('wrapper'))
          .createSync(recursive: true);
      gradleWrapperDir.childFile('gradlew').writeAsStringSync('irrelevant');
      gradleWrapperDir.childFile('gradlew.bat').writeAsStringSync('irrelevant');

      await ApplicationPackageFactory.instance!.getPackageForPlatform(
        TargetPlatform.android_arm,
        applicationBinary: globals.fs.file('app-debug.apk'),
      );
      expect(fakeProcessManager, hasNoRemainingExpectations);
    }, overrides: overrides);

    testUsingContext(
      'Licenses available, build tools available, does not call gradle dependencies',
      () async {
        final AndroidSdkVersion sdkVersion = FakeAndroidSdkVersion();
        sdk.latestVersion = sdkVersion;

        await ApplicationPackageFactory.instance!.getPackageForPlatform(TargetPlatform.android_arm);
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: overrides,
    );

    testWithoutContext('returns null when failed to extract manifest', () async {
      final Logger logger = BufferLogger.test();
      final AndroidApk? androidApk = AndroidApk.fromApk(
        fs.file(''),
        processManager: fakeProcessManager,
        logger: logger,
        userMessages: UserMessages(),
        androidSdk: sdk,
        processUtils: ProcessUtils(processManager: fakeProcessManager, logger: logger),
      );

      expect(androidApk, isNull);
      expect(fakeProcessManager, hasNoRemainingExpectations);
    });
  });

  group('ApkManifestData', () {
    testWithoutContext(
      'Parses manifest with an Activity that has enabled set to true, action set to android.intent.action.MAIN and category set to android.intent.category.LAUNCHER',
      () {
        final ApkManifestData data =
            ApkManifestData.parseFromXmlDump(
              _aaptDataWithExplicitEnabledAndMainLauncherActivity,
              BufferLogger.test(),
            )!;

        expect(data, isNotNull);
        expect(data.packageName, 'io.flutter.examples.hello_world');
        expect(data.launchableActivityName, 'io.flutter.examples.hello_world.MainActivity2');
      },
    );

    testWithoutContext(
      'Parses manifest with an Activity that has no value for its enabled field, action set to android.intent.action.MAIN and category set to android.intent.category.LAUNCHER',
      () {
        final ApkManifestData data =
            ApkManifestData.parseFromXmlDump(
              _aaptDataWithDefaultEnabledAndMainLauncherActivity,
              BufferLogger.test(),
            )!;

        expect(data, isNotNull);
        expect(data.packageName, 'io.flutter.examples.hello_world');
        expect(data.launchableActivityName, 'io.flutter.examples.hello_world.MainActivity2');
      },
    );

    testWithoutContext('Parses manifest with a dist namespace', () {
      final ApkManifestData data =
          ApkManifestData.parseFromXmlDump(_aaptDataWithDistNamespace, BufferLogger.test())!;

      expect(data, isNotNull);
      expect(data.packageName, 'io.flutter.examples.hello_world');
      expect(data.launchableActivityName, 'io.flutter.examples.hello_world.MainActivity');
    });

    testWithoutContext(
      'Error when parsing manifest with no Activity that has enabled set to true nor has no value for its enabled field',
      () {
        final BufferLogger logger = BufferLogger.test();
        final ApkManifestData? data = ApkManifestData.parseFromXmlDump(
          _aaptDataWithNoEnabledActivity,
          logger,
        );

        expect(data, isNull);
        expect(
          logger.errorText,
          'Error running io.flutter.examples.hello_world. Default activity not found\n',
        );
      },
    );

    testWithoutContext(
      'Error when parsing manifest with no Activity that has action set to android.intent.action.MAIN',
      () {
        final BufferLogger logger = BufferLogger.test();
        final ApkManifestData? data = ApkManifestData.parseFromXmlDump(
          _aaptDataWithNoMainActivity,
          logger,
        );

        expect(data, isNull);
        expect(
          logger.errorText,
          'Error running io.flutter.examples.hello_world. Default activity not found\n',
        );
      },
    );

    testWithoutContext(
      'Error when parsing manifest with no Activity that has category set to android.intent.category.LAUNCHER',
      () {
        final BufferLogger logger = BufferLogger.test();
        final ApkManifestData? data = ApkManifestData.parseFromXmlDump(
          _aaptDataWithNoLauncherActivity,
          logger,
        );

        expect(data, isNull);
        expect(
          logger.errorText,
          'Error running io.flutter.examples.hello_world. Default activity not found\n',
        );
      },
    );

    testWithoutContext(
      'Parsing manifest with Activity that has multiple category, android.intent.category.LAUNCHER and android.intent.category.DEFAULT',
      () {
        final ApkManifestData data =
            ApkManifestData.parseFromXmlDump(
              _aaptDataWithLauncherAndDefaultActivity,
              BufferLogger.test(),
            )!;

        expect(data, isNotNull);
        expect(data.packageName, 'io.flutter.examples.hello_world');
        expect(data.launchableActivityName, 'io.flutter.examples.hello_world.MainActivity');
      },
    );

    testWithoutContext('Parses manifest with missing application tag', () async {
      final ApkManifestData? data = ApkManifestData.parseFromXmlDump(
        _aaptDataWithoutApplication,
        BufferLogger.test(),
      );

      expect(data, isNull);
    });
  });

  group('PrebuiltIOSApp', () {
    late FakeOperatingSystemUtils os;
    late FakePlistParser testPlistParser;

    final Map<Type, Generator> overrides = <Type, Generator>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
      PlistParser: () => testPlistParser,
      OperatingSystemUtils: () => os,
    };

    setUp(() {
      os = FakeOperatingSystemUtils();
      testPlistParser = FakePlistParser();
    });

    testUsingContext('Error on non-existing file', () {
      final PrebuiltIOSApp? iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('not_existing.ipa')) as PrebuiltIOSApp?;
      expect(iosApp, isNull);
      expect(
        testLogger.errorText,
        'File "not_existing.ipa" does not exist. Use an app bundle or an ipa.\n',
      );
    }, overrides: overrides);

    testUsingContext('Error on non-app-bundle folder', () {
      globals.fs.directory('regular_folder').createSync();
      final PrebuiltIOSApp? iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('regular_folder')) as PrebuiltIOSApp?;
      expect(iosApp, isNull);
      expect(testLogger.errorText, 'Folder "regular_folder" is not an app bundle.\n');
    }, overrides: overrides);

    testUsingContext('Error on no info.plist', () {
      globals.fs.directory('bundle.app').createSync();
      final PrebuiltIOSApp? iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('bundle.app')) as PrebuiltIOSApp?;
      expect(iosApp, isNull);
      expect(testLogger.errorText, 'Invalid prebuilt iOS app. Does not contain Info.plist.\n');
    }, overrides: overrides);

    testUsingContext('Error on bad info.plist', () {
      globals.fs.directory('bundle.app').createSync();
      globals.fs.file('bundle.app/Info.plist').createSync();
      final PrebuiltIOSApp? iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('bundle.app')) as PrebuiltIOSApp?;
      expect(iosApp, isNull);
      expect(
        testLogger.errorText,
        contains('Invalid prebuilt iOS app. Info.plist does not contain bundle identifier\n'),
      );
    }, overrides: overrides);

    testUsingContext('Success with app bundle', () {
      globals.fs.directory('bundle.app').createSync();
      globals.fs.file('bundle.app/Info.plist').createSync();
      testPlistParser.setProperty('CFBundleIdentifier', 'fooBundleId');
      final PrebuiltIOSApp iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('bundle.app'))! as PrebuiltIOSApp;
      expect(testLogger.errorText, isEmpty);
      expect(iosApp.uncompressedBundle.path, 'bundle.app');
      expect(iosApp.id, 'fooBundleId');
      expect(iosApp.bundleName, 'bundle.app');
      expect(iosApp.applicationPackage.path, globals.fs.directory('bundle.app').path);
    }, overrides: overrides);

    testUsingContext('Bad ipa zip-file, no payload dir', () {
      globals.fs.file('app.ipa').createSync();
      final PrebuiltIOSApp? iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('app.ipa')) as PrebuiltIOSApp?;
      expect(iosApp, isNull);
      expect(
        testLogger.errorText,
        'Invalid prebuilt iOS ipa. Does not contain a "Payload" directory.\n',
      );
    }, overrides: overrides);

    testUsingContext('Bad ipa zip-file, two app bundles', () {
      globals.fs.file('app.ipa').createSync();
      os.onUnzip = (File zipFile, Directory targetDirectory) {
        if (zipFile.path != 'app.ipa') {
          return;
        }
        final String bundlePath1 = globals.fs.path.join(
          targetDirectory.path,
          'Payload',
          'bundle1.app',
        );
        final String bundlePath2 = globals.fs.path.join(
          targetDirectory.path,
          'Payload',
          'bundle2.app',
        );
        globals.fs.directory(bundlePath1).createSync(recursive: true);
        globals.fs.directory(bundlePath2).createSync(recursive: true);
      };
      final PrebuiltIOSApp? iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('app.ipa')) as PrebuiltIOSApp?;
      expect(iosApp, isNull);
      expect(
        testLogger.errorText,
        'Invalid prebuilt iOS ipa. Does not contain a single app bundle.\n',
      );
    }, overrides: overrides);

    testUsingContext('Success with ipa', () {
      globals.fs.file('app.ipa').createSync();
      os.onUnzip = (File zipFile, Directory targetDirectory) {
        if (zipFile.path != 'app.ipa') {
          return;
        }
        final Directory bundleAppDir = globals.fs.directory(
          globals.fs.path.join(targetDirectory.path, 'Payload', 'bundle.app'),
        );
        bundleAppDir.createSync(recursive: true);
        testPlistParser.setProperty('CFBundleIdentifier', 'fooBundleId');
        globals.fs.file(globals.fs.path.join(bundleAppDir.path, 'Info.plist')).createSync();
      };
      final PrebuiltIOSApp iosApp =
          IOSApp.fromPrebuiltApp(globals.fs.file('app.ipa'))! as PrebuiltIOSApp;
      expect(testLogger.errorText, isEmpty);
      expect(iosApp.uncompressedBundle.path, endsWith('bundle.app'));
      expect(iosApp.id, 'fooBundleId');
      expect(iosApp.bundleName, 'bundle.app');
      expect(iosApp.applicationPackage.path, globals.fs.file('app.ipa').path);
    }, overrides: overrides);

    testUsingContext('returns null when there is no ios or .ios directory', () async {
      globals.fs.file('pubspec.yaml').createSync();
      final BuildableIOSApp? iosApp =
          await IOSApp.fromIosProject(
                FlutterProject.fromDirectory(globals.fs.currentDirectory).ios,
                null,
              )
              as BuildableIOSApp?;

      expect(iosApp, null);
    }, overrides: overrides);

    testUsingContext('returns null when there is no Runner.xcodeproj', () async {
      globals.fs.file('pubspec.yaml').createSync();
      globals.fs.file('ios/FooBar.xcodeproj').createSync(recursive: true);
      final BuildableIOSApp? iosApp =
          await IOSApp.fromIosProject(
                FlutterProject.fromDirectory(globals.fs.currentDirectory).ios,
                null,
              )
              as BuildableIOSApp?;

      expect(iosApp, null);
    }, overrides: overrides);

    testUsingContext('returns null when there is no Runner.xcodeproj/project.pbxproj', () async {
      globals.fs.file('pubspec.yaml').createSync();
      globals.fs.file('ios/Runner.xcodeproj').createSync(recursive: true);
      final BuildableIOSApp? iosApp =
          await IOSApp.fromIosProject(
                FlutterProject.fromDirectory(globals.fs.currentDirectory).ios,
                null,
              )
              as BuildableIOSApp?;

      expect(iosApp, null);
    }, overrides: overrides);

    testUsingContext('returns null when there with no product identifier', () async {
      globals.fs.file('pubspec.yaml').createSync();
      final Directory project = globals.fs.directory('ios/Runner.xcodeproj')
        ..createSync(recursive: true);
      project.childFile('project.pbxproj').createSync();
      final BuildableIOSApp? iosApp =
          await IOSApp.fromIosProject(
                FlutterProject.fromDirectory(globals.fs.currentDirectory).ios,
                null,
              )
              as BuildableIOSApp?;

      expect(iosApp, null);
    }, overrides: overrides);

    testUsingContext('handles project paths with periods in app name', () async {
      final BuildableIOSApp iosApp = BuildableIOSApp(
        IosProject.fromFlutter(FlutterProject.fromDirectory(globals.fs.currentDirectory)),
        'com.foo.bar',
        'Name.With.Dots',
      );
      expect(iosApp.name, 'Name.With.Dots');
      expect(iosApp.archiveBundleOutputPath, 'build/ios/archive/Name.With.Dots.xcarchive');
      expect(iosApp.deviceBundlePath, 'build/ios/iphoneos/Name.With.Dots.app');
      expect(iosApp.simulatorBundlePath, 'build/ios/iphonesimulator/Name.With.Dots.app');
      expect(
        iosApp.builtInfoPlistPathAfterArchive,
        'build/ios/archive/Name.With.Dots.xcarchive/Products/Applications/Name.With.Dots.app/Info.plist',
      );
    }, overrides: overrides);

    testUsingContext('returns project app icon dirname', () async {
      final BuildableIOSApp iosApp = BuildableIOSApp(
        IosProject.fromFlutter(FlutterProject.fromDirectory(globals.fs.currentDirectory)),
        'com.foo.bar',
        'Runner',
      );
      final String iconDirSuffix = globals.fs.path.join(
        'Runner',
        'Assets.xcassets',
        'AppIcon.appiconset',
      );
      expect(iosApp.projectAppIconDirName, globals.fs.path.join('ios', iconDirSuffix));
    }, overrides: overrides);

    testUsingContext('returns template app icon dirname for Contents.json', () async {
      final BuildableIOSApp iosApp = BuildableIOSApp(
        IosProject.fromFlutter(FlutterProject.fromDirectory(globals.fs.currentDirectory)),
        'com.foo.bar',
        'Runner',
      );
      final String iconDirSuffix = globals.fs.path.join(
        'Runner',
        'Assets.xcassets',
        'AppIcon.appiconset',
      );
      expect(
        iosApp.templateAppIconDirNameForContentsJson,
        globals.fs.path.join(
          Cache.flutterRoot!,
          'packages',
          'flutter_tools',
          'templates',
          'app',
          'ios.tmpl',
          iconDirSuffix,
        ),
      );
    }, overrides: overrides);

    testUsingContext('returns template app icon dirname for images', () async {
      final String toolsDir = globals.fs.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools');
      final String packageConfigPath = globals.fs.path.join(
        toolsDir,
        '.dart_tool',
        'package_config.json',
      );
      globals.fs.file(packageConfigPath)
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "flutter_template_images",
      "rootUri": "/flutter_template_images",
      "packageUri": "lib/",
      "languageVersion": "2.12"
    }
  ]
}
''');
      final BuildableIOSApp iosApp = BuildableIOSApp(
        IosProject.fromFlutter(FlutterProject.fromDirectory(globals.fs.currentDirectory)),
        'com.foo.bar',
        'Runner',
      );
      final String iconDirSuffix = globals.fs.path.join(
        'Runner',
        'Assets.xcassets',
        'AppIcon.appiconset',
      );
      expect(
        await iosApp.templateAppIconDirNameForImages,
        globals.fs.path.absolute(
          'flutter_template_images',
          'templates',
          'app',
          'ios.tmpl',
          iconDirSuffix,
        ),
      );
    }, overrides: overrides);

    testUsingContext('returns project launch image dirname', () async {
      final BuildableIOSApp iosApp = BuildableIOSApp(
        IosProject.fromFlutter(FlutterProject.fromDirectory(globals.fs.currentDirectory)),
        'com.foo.bar',
        'Runner',
      );
      final String launchImageDirSuffix = globals.fs.path.join(
        'Runner',
        'Assets.xcassets',
        'LaunchImage.imageset',
      );
      expect(iosApp.projectLaunchImageDirName, globals.fs.path.join('ios', launchImageDirSuffix));
    }, overrides: overrides);

    testUsingContext('returns template launch image dirname for Contents.json', () async {
      final BuildableIOSApp iosApp = BuildableIOSApp(
        IosProject.fromFlutter(FlutterProject.fromDirectory(globals.fs.currentDirectory)),
        'com.foo.bar',
        'Runner',
      );
      final String launchImageDirSuffix = globals.fs.path.join(
        'Runner',
        'Assets.xcassets',
        'LaunchImage.imageset',
      );
      expect(
        iosApp.templateLaunchImageDirNameForContentsJson,
        globals.fs.path.join(
          Cache.flutterRoot!,
          'packages',
          'flutter_tools',
          'templates',
          'app',
          'ios.tmpl',
          launchImageDirSuffix,
        ),
      );
    }, overrides: overrides);

    testUsingContext('returns template launch image dirname for images', () async {
      final String toolsDir = globals.fs.path.join(Cache.flutterRoot!, 'packages', 'flutter_tools');
      final String packageConfigPath = globals.fs.path.join(
        toolsDir,
        '.dart_tool',
        'package_config.json',
      );
      globals.fs.file(packageConfigPath)
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "flutter_template_images",
      "rootUri": "/flutter_template_images",
      "packageUri": "lib/",
      "languageVersion": "2.12"
    }
  ]
}
''');
      final BuildableIOSApp iosApp = BuildableIOSApp(
        IosProject.fromFlutter(FlutterProject.fromDirectory(globals.fs.currentDirectory)),
        'com.foo.bar',
        'Runner',
      );
      final String launchImageDirSuffix = globals.fs.path.join(
        'Runner',
        'Assets.xcassets',
        'LaunchImage.imageset',
      );
      expect(
        await iosApp.templateLaunchImageDirNameForImages,
        globals.fs.path.absolute(
          'flutter_template_images',
          'templates',
          'app',
          'ios.tmpl',
          launchImageDirSuffix,
        ),
      );
    }, overrides: overrides);
  });
}

const String _aaptDataWithExplicitEnabledAndMainLauncherActivity = '''
N: android=http://schemas.android.com/apk/res/android
  E: manifest (line=7)
    A: android:versionCode(0x0101021b)=(type 0x10)0x1
    A: android:versionName(0x0101021c)="0.0.1" (Raw: "0.0.1")
    A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
    E: uses-sdk (line=12)
      A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
      A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1b
    E: uses-permission (line=21)
      A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
    E: application (line=29)
      A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
      A: android:icon(0x01010002)=@0x7f010000
      A: android:name(0x01010003)="io.flutter.app.FlutterApplication" (Raw: "io.flutter.app.FlutterApplication")
      A: android:debuggable(0x0101000f)=(type 0x12)0xffffffff
      E: activity (line=34)
        A: android:theme(0x01010000)=@0x1030009
        A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
        A: android:enabled(0x0101000e)=(type 0x12)0x0
        A: android:launchMode(0x0101001d)=(type 0x10)0x1
        A: android:configChanges(0x0101001f)=(type 0x11)0x400035b4
        A: android:windowSoftInputMode(0x0101022b)=(type 0x11)0x10
        A: android:hardwareAccelerated(0x010102d3)=(type 0x12)0xffffffff
        E: intent-filter (line=42)
          E: action (line=43)
            A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")
          E: category (line=45)
            A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")
      E: activity (line=48)
        A: android:theme(0x01010000)=@0x1030009
        A: android:label(0x01010001)="app2" (Raw: "app2")
        A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity2" (Raw: "io.flutter.examples.hello_world.MainActivity2")
        A: android:enabled(0x0101000e)=(type 0x12)0xffffffff
        E: intent-filter (line=53)
          E: action (line=54)
            A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")
          E: category (line=56)
            A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")''';

const String _aaptDataWithDefaultEnabledAndMainLauncherActivity = '''
N: android=http://schemas.android.com/apk/res/android
  E: manifest (line=7)
    A: android:versionCode(0x0101021b)=(type 0x10)0x1
    A: android:versionName(0x0101021c)="0.0.1" (Raw: "0.0.1")
    A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
    E: uses-sdk (line=12)
      A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
      A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1b
    E: uses-permission (line=21)
      A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
    E: application (line=29)
      A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
      A: android:icon(0x01010002)=@0x7f010000
      A: android:name(0x01010003)="io.flutter.app.FlutterApplication" (Raw: "io.flutter.app.FlutterApplication")
      A: android:debuggable(0x0101000f)=(type 0x12)0xffffffff
      E: activity (line=34)
        A: android:theme(0x01010000)=@0x1030009
        A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
        A: android:enabled(0x0101000e)=(type 0x12)0x0
        A: android:launchMode(0x0101001d)=(type 0x10)0x1
        A: android:configChanges(0x0101001f)=(type 0x11)0x400035b4
        A: android:windowSoftInputMode(0x0101022b)=(type 0x11)0x10
        A: android:hardwareAccelerated(0x010102d3)=(type 0x12)0xffffffff
        E: intent-filter (line=42)
          E: action (line=43)
            A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")
          E: category (line=45)
            A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")
      E: activity (line=48)
        A: android:theme(0x01010000)=@0x1030009
        A: android:label(0x01010001)="app2" (Raw: "app2")
        A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity2" (Raw: "io.flutter.examples.hello_world.MainActivity2")
        E: intent-filter (line=53)
          E: action (line=54)
            A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")
          E: category (line=56)
            A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")''';

const String _aaptDataWithNoEnabledActivity = '''
N: android=http://schemas.android.com/apk/res/android
  E: manifest (line=7)
    A: android:versionCode(0x0101021b)=(type 0x10)0x1
    A: android:versionName(0x0101021c)="0.0.1" (Raw: "0.0.1")
    A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
    E: uses-sdk (line=12)
      A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
      A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1b
    E: uses-permission (line=21)
      A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
    E: application (line=29)
      A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
      A: android:icon(0x01010002)=@0x7f010000
      A: android:name(0x01010003)="io.flutter.app.FlutterApplication" (Raw: "io.flutter.app.FlutterApplication")
      A: android:debuggable(0x0101000f)=(type 0x12)0xffffffff
      E: activity (line=34)
        A: android:theme(0x01010000)=@0x1030009
        A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
        A: android:enabled(0x0101000e)=(type 0x12)0x0
        A: android:launchMode(0x0101001d)=(type 0x10)0x1
        A: android:configChanges(0x0101001f)=(type 0x11)0x400035b4
        A: android:windowSoftInputMode(0x0101022b)=(type 0x11)0x10
        A: android:hardwareAccelerated(0x010102d3)=(type 0x12)0xffffffff
        E: intent-filter (line=42)
          E: action (line=43)
            A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")
          E: category (line=45)
            A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")''';

const String _aaptDataWithNoMainActivity = '''
N: android=http://schemas.android.com/apk/res/android
  E: manifest (line=7)
    A: android:versionCode(0x0101021b)=(type 0x10)0x1
    A: android:versionName(0x0101021c)="0.0.1" (Raw: "0.0.1")
    A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
    E: uses-sdk (line=12)
      A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
      A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1b
    E: uses-permission (line=21)
      A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
    E: application (line=29)
      A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
      A: android:icon(0x01010002)=@0x7f010000
      A: android:name(0x01010003)="io.flutter.app.FlutterApplication" (Raw: "io.flutter.app.FlutterApplication")
      A: android:debuggable(0x0101000f)=(type 0x12)0xffffffff
      E: activity (line=34)
        A: android:theme(0x01010000)=@0x1030009
        A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
        A: android:enabled(0x0101000e)=(type 0x12)0xffffffff
        A: android:launchMode(0x0101001d)=(type 0x10)0x1
        A: android:configChanges(0x0101001f)=(type 0x11)0x400035b4
        A: android:windowSoftInputMode(0x0101022b)=(type 0x11)0x10
        A: android:hardwareAccelerated(0x010102d3)=(type 0x12)0xffffffff
        E: intent-filter (line=42)
          E: category (line=43)
            A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")''';

const String _aaptDataWithNoLauncherActivity = '''
N: android=http://schemas.android.com/apk/res/android
  E: manifest (line=7)
    A: android:versionCode(0x0101021b)=(type 0x10)0x1
    A: android:versionName(0x0101021c)="0.0.1" (Raw: "0.0.1")
    A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
    E: uses-sdk (line=12)
      A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
      A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1b
    E: uses-permission (line=21)
      A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
    E: application (line=29)
      A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
      A: android:icon(0x01010002)=@0x7f010000
      A: android:name(0x01010003)="io.flutter.app.FlutterApplication" (Raw: "io.flutter.app.FlutterApplication")
      A: android:debuggable(0x0101000f)=(type 0x12)0xffffffff
      E: activity (line=34)
        A: android:theme(0x01010000)=@0x1030009
        A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
        A: android:enabled(0x0101000e)=(type 0x12)0xffffffff
        A: android:launchMode(0x0101001d)=(type 0x10)0x1
        A: android:configChanges(0x0101001f)=(type 0x11)0x400035b4
        A: android:windowSoftInputMode(0x0101022b)=(type 0x11)0x10
        A: android:hardwareAccelerated(0x010102d3)=(type 0x12)0xffffffff
        E: intent-filter (line=42)
          E: action (line=43)
            A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")''';

const String _aaptDataWithLauncherAndDefaultActivity = '''
N: android=http://schemas.android.com/apk/res/android
  N: dist=http://schemas.android.com/apk/distribution
    E: manifest (line=7)
      A: android:versionCode(0x0101021b)=(type 0x10)0x1
      A: android:versionName(0x0101021c)="1.0" (Raw: "1.0")
      A: android:compileSdkVersion(0x01010572)=(type 0x10)0x1c
      A: android:compileSdkVersionCodename(0x01010573)="9" (Raw: "9")
      A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
      A: platformBuildVersionCode=(type 0x10)0x1
      A: platformBuildVersionName=(type 0x4)0x3f800000
      E: uses-sdk (line=13)
        A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
        A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1c
      E: dist:module (line=17)
        A: dist:instant=(type 0x12)0xffffffff
      E: uses-permission (line=24)
        A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
      E: application (line=32)
        A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
        A: android:icon(0x01010002)=@0x7f010000
        A: android:name(0x01010003)="io.flutter.app.FlutterApplication" (Raw: "io.flutter.app.FlutterApplication")
        E: activity (line=36)
          A: android:theme(0x01010000)=@0x01030009
          A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
          A: android:launchMode(0x0101001d)=(type 0x10)0x1
          A: android:configChanges(0x0101001f)=(type 0x11)0x400037b4
          A: android:windowSoftInputMode(0x0101022b)=(type 0x11)0x10
          A: android:hardwareAccelerated(0x010102d3)=(type 0x12)0xffffffff
          E: intent-filter (line=43)
            E: action (line=44)
              A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")
            E: category (line=46)
              A: android:name(0x01010003)="android.intent.category.DEFAULT" (Raw: "android.intent.category.DEFAULT")
            E: category (line=47)
              A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")
''';

const String _aaptDataWithDistNamespace = '''
N: android=http://schemas.android.com/apk/res/android
  N: dist=http://schemas.android.com/apk/distribution
    E: manifest (line=7)
      A: android:versionCode(0x0101021b)=(type 0x10)0x1
      A: android:versionName(0x0101021c)="1.0" (Raw: "1.0")
      A: android:compileSdkVersion(0x01010572)=(type 0x10)0x1c
      A: android:compileSdkVersionCodename(0x01010573)="9" (Raw: "9")
      A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
      A: platformBuildVersionCode=(type 0x10)0x1
      A: platformBuildVersionName=(type 0x4)0x3f800000
      E: uses-sdk (line=13)
        A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
        A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1c
      E: dist:module (line=17)
        A: dist:instant=(type 0x12)0xffffffff
      E: uses-permission (line=24)
        A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
      E: application (line=32)
        A: android:label(0x01010001)="hello_world" (Raw: "hello_world")
        A: android:icon(0x01010002)=@0x7f010000
        A: android:name(0x01010003)="io.flutter.app.FlutterApplication" (Raw: "io.flutter.app.FlutterApplication")
        E: activity (line=36)
          A: android:theme(0x01010000)=@0x01030009
          A: android:name(0x01010003)="io.flutter.examples.hello_world.MainActivity" (Raw: "io.flutter.examples.hello_world.MainActivity")
          A: android:launchMode(0x0101001d)=(type 0x10)0x1
          A: android:configChanges(0x0101001f)=(type 0x11)0x400037b4
          A: android:windowSoftInputMode(0x0101022b)=(type 0x11)0x10
          A: android:hardwareAccelerated(0x010102d3)=(type 0x12)0xffffffff
          E: intent-filter (line=43)
            E: action (line=44)
              A: android:name(0x01010003)="android.intent.action.MAIN" (Raw: "android.intent.action.MAIN")
            E: category (line=46)
              A: android:name(0x01010003)="android.intent.category.LAUNCHER" (Raw: "android.intent.category.LAUNCHER")
''';

const String _aaptDataWithoutApplication = '''
N: android=http://schemas.android.com/apk/res/android
  N: dist=http://schemas.android.com/apk/distribution
    E: manifest (line=7)
      A: android:versionCode(0x0101021b)=(type 0x10)0x1
      A: android:versionName(0x0101021c)="1.0" (Raw: "1.0")
      A: android:compileSdkVersion(0x01010572)=(type 0x10)0x1c
      A: android:compileSdkVersionCodename(0x01010573)="9" (Raw: "9")
      A: package="io.flutter.examples.hello_world" (Raw: "io.flutter.examples.hello_world")
      A: platformBuildVersionCode=(type 0x10)0x1
      A: platformBuildVersionName=(type 0x4)0x3f800000
      E: uses-sdk (line=13)
        A: android:minSdkVersion(0x0101020c)=(type 0x10)0x10
        A: android:targetSdkVersion(0x01010270)=(type 0x10)0x1c
      E: dist:module (line=17)
        A: dist:instant=(type 0x12)0xffffffff
      E: uses-permission (line=24)
        A: android:name(0x01010003)="android.permission.INTERNET" (Raw: "android.permission.INTERNET")
''';

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  void Function(File, Directory)? onUnzip;

  @override
  void unzip(File file, Directory targetDirectory) {
    onUnzip?.call(file, targetDirectory);
  }
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  late bool platformToolsAvailable;

  @override
  late bool licensesAvailable;

  @override
  AndroidSdkVersion? latestVersion;
}

class FakeAndroidSdkVersion extends Fake implements AndroidSdkVersion {
  @override
  late String aaptPath;
}

Future<FlutterProject> aModuleProject() async {
  final Directory directory = globals.fs.directory('module_project');
  directory.childDirectory('.dart_tool').childFile('package_config.json')
    ..createSync(recursive: true)
    ..writeAsStringSync('{"configVersion":2,"packages":[]}');
  directory.childFile('pubspec.yaml').writeAsStringSync('''
name: my_module
flutter:
  module:
    androidPackage: com.example
''');
  return FlutterProject.fromDirectory(directory);
}
