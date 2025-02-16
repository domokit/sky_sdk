// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';

import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';

import '../src/common.dart';
import '../src/context.dart';

void main() {
  late BufferLogger logger;
  setUp(() {
    logger = BufferLogger.test();
  });

  group('Validate build number', () {
    testWithoutContext('CFBundleVersion for iOS', () async {
      String? buildName = validatedBuildNumberForPlatform(TargetPlatform.ios, 'xyz', logger);
      expect(buildName, isNull);
      buildName = validatedBuildNumberForPlatform(TargetPlatform.ios, '0.0.1', logger);
      expect(buildName, '0.0.1');
      buildName = validatedBuildNumberForPlatform(TargetPlatform.ios, '123.xyz', logger);
      expect(buildName, '123');
      buildName = validatedBuildNumberForPlatform(TargetPlatform.ios, '123.456.xyz', logger);
      expect(buildName, '123.456');
    });

    testWithoutContext('versionCode for Android', () async {
      String? buildName = validatedBuildNumberForPlatform(
        TargetPlatform.android_arm,
        '123.abc+-',
        logger,
      );
      expect(buildName, '123');
      buildName = validatedBuildNumberForPlatform(TargetPlatform.android_arm, 'abc', logger);
      expect(buildName, '1');
    });
  });

  group('Validate build name', () {
    testWithoutContext('CFBundleShortVersionString for iOS', () async {
      String? buildName = validatedBuildNameForPlatform(TargetPlatform.ios, 'xyz', logger);
      expect(buildName, isNull);
      buildName = validatedBuildNameForPlatform(TargetPlatform.ios, '0.0.1', logger);
      expect(buildName, '0.0.1');

      buildName = validatedBuildNameForPlatform(TargetPlatform.ios, '123.456.xyz', logger);
      expect(logger.traceText, contains('Invalid build-name'));
      expect(buildName, '123.456.0');

      buildName = validatedBuildNameForPlatform(TargetPlatform.ios, '123.xyz', logger);
      expect(buildName, '123.0.0');
    });

    testWithoutContext('versionName for Android', () async {
      String? buildName = validatedBuildNameForPlatform(
        TargetPlatform.android_arm,
        '123.abc+-',
        logger,
      );
      expect(buildName, '123.abc+-');
      buildName = validatedBuildNameForPlatform(TargetPlatform.android_arm, 'abc+-', logger);
      expect(buildName, 'abc+-');
    });

    testWithoutContext('build mode configuration is correct', () {
      expect(BuildMode.debug.isRelease, false);
      expect(BuildMode.debug.isPrecompiled, false);
      expect(BuildMode.debug.isJit, true);

      expect(BuildMode.profile.isRelease, false);
      expect(BuildMode.profile.isPrecompiled, true);
      expect(BuildMode.profile.isJit, false);

      expect(BuildMode.release.isRelease, true);
      expect(BuildMode.release.isPrecompiled, true);
      expect(BuildMode.release.isJit, false);

      expect(BuildMode.jitRelease.isRelease, true);
      expect(BuildMode.jitRelease.isPrecompiled, false);
      expect(BuildMode.jitRelease.isJit, true);

      expect(BuildMode.fromCliName('debug'), BuildMode.debug);
      expect(BuildMode.fromCliName('profile'), BuildMode.profile);
      expect(BuildMode.fromCliName('jit_release'), BuildMode.jitRelease);
      expect(BuildMode.fromCliName('release'), BuildMode.release);
      expect(() => BuildMode.fromCliName('foo'), throwsArgumentError);
    });
  });

  testWithoutContext('getDartNameForDarwinArch returns name used in Dart SDK', () {
    expect(DarwinArch.armv7.dartName, 'armv7');
    expect(DarwinArch.arm64.dartName, 'arm64');
    expect(DarwinArch.x86_64.dartName, 'x64');
  });

  testWithoutContext('getNameForDarwinArch returns Apple names', () {
    expect(DarwinArch.armv7.name, 'armv7');
    expect(DarwinArch.arm64.name, 'arm64');
    expect(DarwinArch.x86_64.name, 'x86_64');
  });

  testWithoutContext('getNameForTargetPlatform on Darwin arches', () {
    expect(getNameForTargetPlatform(TargetPlatform.ios, darwinArch: DarwinArch.arm64), 'ios-arm64');
    expect(getNameForTargetPlatform(TargetPlatform.ios, darwinArch: DarwinArch.armv7), 'ios-armv7');
    expect(
      getNameForTargetPlatform(TargetPlatform.ios, darwinArch: DarwinArch.x86_64),
      'ios-x86_64',
    );
    expect(getNameForTargetPlatform(TargetPlatform.android), isNot(contains('ios')));
  });

  testUsingContext(
    'defaultIOSArchsForEnvironment',
    () {
      expect(
        defaultIOSArchsForEnvironment(
          EnvironmentType.physical,
          Artifacts.testLocalEngine(
            localEngineHost: 'host_debug_unopt',
            localEngine: 'ios_debug_unopt',
          ),
        ).single,
        DarwinArch.arm64,
      );

      expect(
        defaultIOSArchsForEnvironment(
          EnvironmentType.simulator,
          Artifacts.testLocalEngine(
            localEngineHost: 'host_debug_unopt',
            localEngine: 'ios_debug_sim_unopt',
          ),
        ).single,
        DarwinArch.x86_64,
      );

      expect(
        defaultIOSArchsForEnvironment(
          EnvironmentType.simulator,
          Artifacts.testLocalEngine(
            localEngineHost: 'host_debug_unopt',
            localEngine: 'ios_debug_sim_unopt_arm64',
          ),
        ).single,
        DarwinArch.arm64,
      );

      expect(
        defaultIOSArchsForEnvironment(EnvironmentType.physical, Artifacts.test()).single,
        DarwinArch.arm64,
      );

      expect(
        defaultIOSArchsForEnvironment(EnvironmentType.simulator, Artifacts.test()),
        <DarwinArch>[DarwinArch.x86_64, DarwinArch.arm64],
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'defaultMacOSArchsForEnvironment',
    () {
      expect(
        defaultMacOSArchsForEnvironment(
          Artifacts.testLocalEngine(
            localEngineHost: 'host_debug_unopt',
            localEngine: 'host_debug_unopt',
          ),
        ).single,
        DarwinArch.x86_64,
      );

      expect(
        defaultMacOSArchsForEnvironment(
          Artifacts.testLocalEngine(
            localEngineHost: 'host_debug_unopt',
            localEngine: 'host_debug_unopt_arm64',
          ),
        ).single,
        DarwinArch.arm64,
      );

      expect(defaultMacOSArchsForEnvironment(Artifacts.test()), <DarwinArch>[
        DarwinArch.x86_64,
        DarwinArch.arm64,
      ]);
    },
    overrides: <Type, Generator>{
      FileSystem: () => MemoryFileSystem.test(),
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testWithoutContext('getIOSArchForName on Darwin arches', () {
    expect(getIOSArchForName('armv7'), DarwinArch.armv7);
    expect(getIOSArchForName('arm64'), DarwinArch.arm64);
    expect(getIOSArchForName('arm64e'), DarwinArch.arm64);
    expect(getIOSArchForName('x86_64'), DarwinArch.x86_64);
    expect(() => getIOSArchForName('bogus'), throwsException);
  });

  testWithoutContext('named BuildInfo has correct defaults', () {
    expect(BuildInfo.debug.mode, BuildMode.debug);
    expect(BuildInfo.debug.trackWidgetCreation, true);

    expect(BuildInfo.profile.mode, BuildMode.profile);
    expect(BuildInfo.profile.trackWidgetCreation, false);

    expect(BuildInfo.release.mode, BuildMode.release);
    expect(BuildInfo.release.trackWidgetCreation, false);
  });

  testWithoutContext('toBuildSystemEnvironment encoding of standard values', () {
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.debug,
      '',
      treeShakeIcons: true,
      trackWidgetCreation: true,
      dartDefines: <String>['foo=2', 'bar=2'],
      dartObfuscation: true,
      splitDebugInfoPath: 'foo/',
      frontendServerStarterPath: 'foo/bar/frontend_server_starter.dart',
      extraFrontEndOptions: <String>['--enable-experiment=non-nullable', 'bar'],
      extraGenSnapshotOptions: <String>['--enable-experiment=non-nullable', 'fizz'],
      packageConfigPath: 'foo/.dart_tool/package_config.json',
      codeSizeDirectory: 'foo/code-size',
      fileSystemRoots: <String>['test5', 'test6'],
      fileSystemScheme: 'scheme',
      buildName: '122',
      buildNumber: '22',
    );

    expect(buildInfo.toBuildSystemEnvironment(), <String, String>{
      'BuildMode': 'debug',
      'DartDefines': 'Zm9vPTI=,YmFyPTI=',
      'DartObfuscation': 'true',
      'FrontendServerStarterPath': 'foo/bar/frontend_server_starter.dart',
      'ExtraFrontEndOptions': '--enable-experiment=non-nullable,bar',
      'ExtraGenSnapshotOptions': '--enable-experiment=non-nullable,fizz',
      'SplitDebugInfo': 'foo/',
      'TrackWidgetCreation': 'true',
      'TreeShakeIcons': 'true',
      'CodeSizeDirectory': 'foo/code-size',
      'FileSystemRoots': 'test5,test6',
      'FileSystemScheme': 'scheme',
      'BuildName': '122',
      'BuildNumber': '22',
    });
  });

  testWithoutContext('toEnvironmentConfig encoding of standard values', () {
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.debug,
      'strawberry',
      treeShakeIcons: true,
      trackWidgetCreation: true,
      dartDefines: <String>['foo=2', 'bar=2'],
      dartObfuscation: true,
      splitDebugInfoPath: 'foo/',
      frontendServerStarterPath: 'foo/bar/frontend_server_starter.dart',
      extraFrontEndOptions: <String>['--enable-experiment=non-nullable', 'bar'],
      extraGenSnapshotOptions: <String>['--enable-experiment=non-nullable', 'fizz'],
      packageConfigPath: 'foo/.dart_tool/package_config.json',
      codeSizeDirectory: 'foo/code-size',
      // These values are ignored by toEnvironmentConfig
      androidProjectArgs: <String>['foo=bar', 'fizz=bazz'],
    );

    expect(buildInfo.toEnvironmentConfig(), <String, String>{
      'TREE_SHAKE_ICONS': 'true',
      'TRACK_WIDGET_CREATION': 'true',
      'DART_DEFINES': 'Zm9vPTI=,YmFyPTI=',
      'DART_OBFUSCATION': 'true',
      'SPLIT_DEBUG_INFO': 'foo/',
      'FRONTEND_SERVER_STARTER_PATH': 'foo/bar/frontend_server_starter.dart',
      'EXTRA_FRONT_END_OPTIONS': '--enable-experiment=non-nullable,bar',
      'EXTRA_GEN_SNAPSHOT_OPTIONS': '--enable-experiment=non-nullable,fizz',
      'PACKAGE_CONFIG': 'foo/.dart_tool/package_config.json',
      'CODE_SIZE_DIRECTORY': 'foo/code-size',
      'FLAVOR': 'strawberry',
    });
  });

  testWithoutContext('toGradleConfig encoding of standard values', () {
    const BuildInfo buildInfo = BuildInfo(
      BuildMode.debug,
      '',
      treeShakeIcons: true,
      trackWidgetCreation: true,
      dartDefines: <String>['foo=2', 'bar=2'],
      dartObfuscation: true,
      splitDebugInfoPath: 'foo/',
      frontendServerStarterPath: 'foo/bar/frontend_server_starter.dart',
      extraFrontEndOptions: <String>['--enable-experiment=non-nullable', 'bar'],
      extraGenSnapshotOptions: <String>['--enable-experiment=non-nullable', 'fizz'],
      packageConfigPath: 'foo/.dart_tool/package_config.json',
      codeSizeDirectory: 'foo/code-size',
      androidProjectArgs: <String>['foo=bar', 'fizz=bazz'],
    );

    expect(buildInfo.toGradleConfig(), <String>[
      '-Pdart-defines=Zm9vPTI=,YmFyPTI=',
      '-Pdart-obfuscation=true',
      '-Pfrontend-server-starter-path=foo/bar/frontend_server_starter.dart',
      '-Pextra-front-end-options=--enable-experiment=non-nullable,bar',
      '-Pextra-gen-snapshot-options=--enable-experiment=non-nullable,fizz',
      '-Psplit-debug-info=foo/',
      '-Ptrack-widget-creation=true',
      '-Ptree-shake-icons=true',
      '-Pcode-size-directory=foo/code-size',
      '-Pfoo=bar',
      '-Pfizz=bazz',
    ]);
  });

  testWithoutContext('encodeDartDefines encodes define values with base64 encoded components', () {
    expect(encodeDartDefines(<String>['"hello"']), 'ImhlbGxvIg==');
    expect(
      encodeDartDefines(<String>['https://www.google.com']),
      'aHR0cHM6Ly93d3cuZ29vZ2xlLmNvbQ==',
    );
    expect(encodeDartDefines(<String>['2,3,4', '5']), 'MiwzLDQ=,NQ==');
    expect(encodeDartDefines(<String>['true', 'false', 'flase']), 'dHJ1ZQ==,ZmFsc2U=,Zmxhc2U=');
    expect(encodeDartDefines(<String>['1232,456', '2']), 'MTIzMiw0NTY=,Mg==');
  });

  testWithoutContext('decodeDartDefines decodes base64 encoded dart defines', () {
    expect(
      decodeDartDefines(<String, String>{kDartDefines: 'ImhlbGxvIg=='}, kDartDefines),
      <String>['"hello"'],
    );
    expect(
      decodeDartDefines(<String, String>{
        kDartDefines: 'aHR0cHM6Ly93d3cuZ29vZ2xlLmNvbQ==',
      }, kDartDefines),
      <String>['https://www.google.com'],
    );
    expect(
      decodeDartDefines(<String, String>{kDartDefines: 'MiwzLDQ=,NQ=='}, kDartDefines),
      <String>['2,3,4', '5'],
    );
    expect(
      decodeDartDefines(<String, String>{kDartDefines: 'dHJ1ZQ==,ZmFsc2U=,Zmxhc2U='}, kDartDefines),
      <String>['true', 'false', 'flase'],
    );
    expect(
      decodeDartDefines(<String, String>{kDartDefines: 'MTIzMiw0NTY=,Mg=='}, kDartDefines),
      <String>['1232,456', '2'],
    );
  });
}
