// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/ios.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../../../src/common.dart';
import '../../../src/context.dart';
import '../../../src/fake_process_manager.dart';
import '../../../src/fakes.dart';

final Platform macPlatform = FakePlatform(
  operatingSystem: 'macos',
  environment: <String, String>{},
);

const List<String> _kSharedConfig = <String>[
  '-dynamiclib',
  '-miphoneos-version-min=12.0',
  '-Xlinker',
  '-rpath',
  '-Xlinker',
  '@executable_path/Frameworks',
  '-Xlinker',
  '-rpath',
  '-Xlinker',
  '@loader_path/Frameworks',
  '-fapplication-extension',
  '-install_name',
  '@rpath/App.framework/App',
  '-isysroot',
  'path/to/iPhoneOS.sdk',
];

void main() {
  late Environment environment;
  late MemoryFileSystem fileSystem;
  late FakeProcessManager processManager;
  late Artifacts artifacts;
  late BufferLogger logger;
  late TestUsage usage;
  late FakeAnalytics fakeAnalytics;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    processManager = FakeProcessManager.empty();
    logger = BufferLogger.test();
    artifacts = Artifacts.test();
    usage = TestUsage();
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: fileSystem,
      fakeFlutterVersion: FakeFlutterVersion(),
    );
    environment = Environment.test(
      fileSystem.currentDirectory,
      defines: <String, String>{kTargetPlatform: 'ios'},
      inputs: <String, String>{},
      processManager: processManager,
      artifacts: artifacts,
      logger: logger,
      fileSystem: fileSystem,
      engineVersion: '2',
      usage: usage,
      analytics: fakeAnalytics,
    );
  });

  testWithoutContext('iOS AOT targets has analyticsName', () {
    expect(const AotAssemblyRelease().analyticsName, 'ios_aot');
    expect(const AotAssemblyProfile().analyticsName, 'ios_aot');
  });

  testUsingContext(
    'DebugUniversalFramework creates simulator binary',
    () async {
      environment.defines[kIosArchs] = 'x86_64';
      environment.defines[kSdkRoot] = 'path/to/iPhoneSimulator.sdk';
      final String appFrameworkPath =
          environment.buildDir.childDirectory('App.framework').childFile('App').path;
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            'xcrun',
            'clang',
            '-x',
            'c',
            '-arch',
            'x86_64',
            fileSystem.path.absolute(
              fileSystem.path.join('.tmp_rand0', 'flutter_tools_stub_source.rand0', 'debug_app.cc'),
            ),
            '-dynamiclib',
            '-miphonesimulator-version-min=12.0',
            '-Xlinker',
            '-rpath',
            '-Xlinker',
            '@executable_path/Frameworks',
            '-Xlinker',
            '-rpath',
            '-Xlinker',
            '@loader_path/Frameworks',
            '-fapplication-extension',
            '-install_name',
            '@rpath/App.framework/App',
            '-isysroot',
            'path/to/iPhoneSimulator.sdk',
            '-o',
            appFrameworkPath,
          ],
        ),
        FakeCommand(
          command: <String>['xattr', '-r', '-d', 'com.apple.FinderInfo', appFrameworkPath],
        ),
        FakeCommand(
          command: <String>[
            'codesign',
            '--force',
            '--sign',
            '-',
            '--timestamp=none',
            appFrameworkPath,
          ],
        ),
      ]);

      await const DebugUniversalFramework().build(environment);
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'DebugUniversalFramework creates expected binary with arm64 only arch',
    () async {
      environment.defines[kIosArchs] = 'arm64';
      environment.defines[kSdkRoot] = 'path/to/iPhoneOS.sdk';
      final String appFrameworkPath =
          environment.buildDir.childDirectory('App.framework').childFile('App').path;
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            'xcrun',
            'clang',
            '-x',
            'c',
            // iphone only gets 64 bit arch based on kIosArchs
            '-arch',
            'arm64',
            fileSystem.path.absolute(
              fileSystem.path.join('.tmp_rand0', 'flutter_tools_stub_source.rand0', 'debug_app.cc'),
            ),
            ..._kSharedConfig,
            '-o',
            appFrameworkPath,
          ],
        ),
        FakeCommand(
          command: <String>['xattr', '-r', '-d', 'com.apple.FinderInfo', appFrameworkPath],
        ),
        FakeCommand(
          command: <String>[
            'codesign',
            '--force',
            '--sign',
            '-',
            '--timestamp=none',
            appFrameworkPath,
          ],
        ),
      ]);

      await const DebugUniversalFramework().build(environment);
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'DebugIosApplicationBundle',
    () async {
      environment.defines[kBundleSkSLPath] = 'bundle.sksl';
      environment.defines[kBuildMode] = 'debug';
      environment.defines[kCodesignIdentity] = 'ABC123';
      // Precompiled dart data

      fileSystem
          .file(artifacts.getArtifactPath(Artifact.vmSnapshotData, mode: BuildMode.debug))
          .createSync();
      fileSystem
          .file(artifacts.getArtifactPath(Artifact.isolateSnapshotData, mode: BuildMode.debug))
          .createSync();
      // Project info
      fileSystem.file('pubspec.yaml').writeAsStringSync('name: hello');
      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);
      // Plist file
      fileSystem
          .file(fileSystem.path.join('ios', 'Flutter', 'AppFrameworkInfo.plist'))
          .createSync(recursive: true);
      // App kernel
      environment.buildDir.childFile('app.dill').createSync(recursive: true);
      environment.buildDir.childFile('native_assets.json').createSync();
      // Stub framework
      environment.buildDir
          .childDirectory('App.framework')
          .childFile('App')
          .createSync(recursive: true);
      // sksl bundle
      fileSystem
          .file('bundle.sksl')
          .writeAsStringSync(
            json.encode(<String, Object>{
              'engineRevision': '2',
              'platform': 'ios',
              'data': <String, Object>{'A': 'B'},
            }),
          );

      final Directory frameworkDirectory = environment.outputDir.childDirectory('App.framework');
      final File frameworkDirectoryBinary = frameworkDirectory.childFile('App');
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            'xattr',
            '-r',
            '-d',
            'com.apple.FinderInfo',
            frameworkDirectoryBinary.path,
          ],
        ),
        FakeCommand(
          command: <String>[
            'codesign',
            '--force',
            '--sign',
            'ABC123',
            '--timestamp=none',
            frameworkDirectoryBinary.path,
          ],
        ),
      ]);

      await const DebugIosApplicationBundle().build(environment);
      expect(processManager, hasNoRemainingExpectations);

      expect(frameworkDirectoryBinary, exists);
      expect(frameworkDirectory.childFile('Info.plist'), exists);

      final Directory assetDirectory = frameworkDirectory.childDirectory('flutter_assets');
      expect(assetDirectory.childFile('kernel_blob.bin'), exists);
      expect(assetDirectory.childFile('AssetManifest.json'), exists);
      expect(assetDirectory.childFile('vm_snapshot_data'), exists);
      expect(assetDirectory.childFile('isolate_snapshot_data'), exists);
      expect(assetDirectory.childFile('io.flutter.shaders.json'), exists);
      expect(
        assetDirectory.childFile('io.flutter.shaders.json').readAsStringSync(),
        '{"data":{"A":"B"}}',
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'DebugIosApplicationBundle with impeller and shader compilation',
    () async {
      // Create impellerc to work around fallback detection logic.
      fileSystem
          .file(artifacts.getHostArtifact(HostArtifact.impellerc))
          .createSync(recursive: true);

      environment.defines[kBuildMode] = 'debug';
      environment.defines[kCodesignIdentity] = 'ABC123';
      // Precompiled dart data

      fileSystem
          .file(artifacts.getArtifactPath(Artifact.vmSnapshotData, mode: BuildMode.debug))
          .createSync();
      fileSystem
          .file(artifacts.getArtifactPath(Artifact.isolateSnapshotData, mode: BuildMode.debug))
          .createSync();
      // Project info
      fileSystem
          .file('pubspec.yaml')
          .writeAsStringSync('name: hello\nflutter:\n  shaders:\n    - shader.glsl');
      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);
      // Plist file
      fileSystem
          .file(fileSystem.path.join('ios', 'Flutter', 'AppFrameworkInfo.plist'))
          .createSync(recursive: true);
      // Shader file
      fileSystem.file('shader.glsl').writeAsStringSync('test');
      // App kernel
      environment.buildDir.childFile('app.dill').createSync(recursive: true);
      environment.buildDir.childFile('native_assets.json').createSync();
      // Stub framework
      environment.buildDir
          .childDirectory('App.framework')
          .childFile('App')
          .createSync(recursive: true);

      final Directory frameworkDirectory = environment.outputDir.childDirectory('App.framework');
      final File frameworkDirectoryBinary = frameworkDirectory.childFile('App');
      processManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            'HostArtifact.impellerc',
            '--sksl',
            '--runtime-stage-metal',
            '--iplr',
            '--sl=/App.framework/flutter_assets/shader.glsl',
            '--spirv=/App.framework/flutter_assets/shader.glsl.spirv',
            '--input=/shader.glsl',
            '--input-type=frag',
            '--include=/',
            '--include=/./shader_lib',
          ],
        ),
        FakeCommand(
          command: <String>[
            'xattr',
            '-r',
            '-d',
            'com.apple.FinderInfo',
            frameworkDirectoryBinary.path,
          ],
        ),
        FakeCommand(
          command: <String>[
            'codesign',
            '--force',
            '--sign',
            'ABC123',
            '--timestamp=none',
            frameworkDirectoryBinary.path,
          ],
        ),
      ]);

      await const DebugIosApplicationBundle().build(environment);
      expect(processManager, hasNoRemainingExpectations);

      expect(frameworkDirectoryBinary, exists);
      expect(frameworkDirectory.childFile('Info.plist'), exists);

      final Directory assetDirectory = frameworkDirectory.childDirectory('flutter_assets');
      expect(assetDirectory.childFile('kernel_blob.bin'), exists);
      expect(assetDirectory.childFile('AssetManifest.json'), exists);
      expect(assetDirectory.childFile('vm_snapshot_data'), exists);
      expect(assetDirectory.childFile('isolate_snapshot_data'), exists);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'ReleaseIosApplicationBundle build',
    () async {
      environment.defines[kBuildMode] = 'release';
      environment.defines[kCodesignIdentity] = 'ABC123';
      environment.defines[kXcodeAction] = 'build';

      // Project info
      fileSystem.file('pubspec.yaml').writeAsStringSync('name: hello');
      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);
      // Plist file
      fileSystem
          .file(fileSystem.path.join('ios', 'Flutter', 'AppFrameworkInfo.plist'))
          .createSync(recursive: true);

      // Real framework
      environment.buildDir
          .childDirectory('App.framework')
          .childFile('App')
          .createSync(recursive: true);
      environment.buildDir.childFile('native_assets.json').createSync();

      // Input dSYM
      environment.buildDir
          .childDirectory('App.framework.dSYM')
          .childDirectory('Contents')
          .childDirectory('Resources')
          .childDirectory('DWARF')
          .childFile('App')
          .createSync(recursive: true);

      final Directory frameworkDirectory = environment.outputDir.childDirectory('App.framework');
      final File frameworkDirectoryBinary = frameworkDirectory.childFile('App');
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            'xattr',
            '-r',
            '-d',
            'com.apple.FinderInfo',
            frameworkDirectoryBinary.path,
          ],
        ),
        FakeCommand(
          command: <String>[
            'codesign',
            '--force',
            '--sign',
            'ABC123',
            frameworkDirectoryBinary.path,
          ],
        ),
      ]);

      await const ReleaseIosApplicationBundle().build(environment);
      expect(processManager, hasNoRemainingExpectations);

      expect(frameworkDirectoryBinary, exists);
      expect(frameworkDirectory.childFile('Info.plist'), exists);
      expect(
        environment.outputDir
            .childDirectory('App.framework.dSYM')
            .childDirectory('Contents')
            .childDirectory('Resources')
            .childDirectory('DWARF')
            .childFile('App'),
        exists,
      );

      final Directory assetDirectory = frameworkDirectory.childDirectory('flutter_assets');
      expect(assetDirectory.childFile('kernel_blob.bin'), isNot(exists));
      expect(assetDirectory.childFile('AssetManifest.json'), exists);
      expect(assetDirectory.childFile('vm_snapshot_data'), isNot(exists));
      expect(assetDirectory.childFile('isolate_snapshot_data'), isNot(exists));
      expect(usage.events, isEmpty);
      expect(fakeAnalytics.sentEvents, isEmpty);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'ReleaseIosApplicationBundle sends archive success event',
    () async {
      environment.defines[kBuildMode] = 'release';
      environment.defines[kXcodeAction] = 'install';

      fileSystem
          .file(fileSystem.path.join('ios', 'Flutter', 'AppFrameworkInfo.plist'))
          .createSync(recursive: true);

      environment.buildDir
          .childDirectory('App.framework')
          .childFile('App')
          .createSync(recursive: true);
      environment.buildDir.childFile('native_assets.json').createSync();

      final Directory frameworkDirectory = environment.outputDir.childDirectory('App.framework');
      final File frameworkDirectoryBinary = frameworkDirectory.childFile('App');
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            'xattr',
            '-r',
            '-d',
            'com.apple.FinderInfo',
            frameworkDirectoryBinary.path,
          ],
        ),
        FakeCommand(
          command: <String>['codesign', '--force', '--sign', '-', frameworkDirectoryBinary.path],
        ),
      ]);

      await const ReleaseIosApplicationBundle().build(environment);
      expect(
        usage.events,
        contains(const TestUsageEvent('assemble', 'ios-archive', label: 'success')),
      );
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.appleUsageEvent(workflow: 'assemble', parameter: 'ios-archive', result: 'success'),
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'ReleaseIosApplicationBundle sends archive fail event',
    () async {
      environment.defines[kBuildMode] = 'release';
      environment.defines[kXcodeAction] = 'install';

      // Throws because the project files are not set up.
      await expectLater(
        () => const ReleaseIosApplicationBundle().build(environment),
        throwsA(const TypeMatcher<FileSystemException>()),
      );
      expect(
        usage.events,
        contains(const TestUsageEvent('assemble', 'ios-archive', label: 'fail')),
      );
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.appleUsageEvent(workflow: 'assemble', parameter: 'ios-archive', result: 'fail'),
        ),
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'AotAssemblyRelease throws exception if asked to build for simulator',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        defines: <String, String>{
          kTargetPlatform: 'ios',
          kSdkRoot: 'path/to/iPhoneSimulator.sdk',
          kBuildMode: 'release',
          kIosArchs: 'x86_64',
        },
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
      );

      expect(
        const AotAssemblyRelease().build(environment),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'description',
            contains('release/profile builds are only supported for physical devices.'),
          ),
        ),
      );
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  testUsingContext(
    'AotAssemblyRelease throws exception if sdk root is missing',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        defines: <String, String>{kTargetPlatform: 'ios'},
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
      );
      environment.defines[kBuildMode] = 'release';
      environment.defines[kIosArchs] = 'x86_64';

      expect(
        const AotAssemblyRelease().build(environment),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'description',
            contains('required define SdkRoot but it was not provided'),
          ),
        ),
      );
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      Platform: () => macPlatform,
    },
  );

  group('copies Flutter.framework', () {
    late Directory outputDir;
    late File binary;
    late FakeCommand copyPhysicalFrameworkCommand;
    late FakeCommand copyPhysicalFrameworkDsymCommand;
    late FakeCommand copyPhysicalFrameworkDsymCommandFailure;
    late FakeCommand lipoCommandNonFatResult;
    late FakeCommand lipoVerifyArm64Command;
    late FakeCommand xattrCommand;
    late FakeCommand adHocCodesignCommand;

    setUp(() {
      final FileSystem fileSystem = MemoryFileSystem.test();
      outputDir = fileSystem.directory('output');
      binary = outputDir.childDirectory('Flutter.framework').childFile('Flutter');

      copyPhysicalFrameworkCommand = FakeCommand(
        command: <String>[
          'rsync',
          '-av',
          '--delete',
          '--filter',
          '- .DS_Store/',
          '--chmod=Du=rwx,Dgo=rx,Fu=rw,Fgo=r',
          'Artifact.flutterFramework.TargetPlatform.ios.debug.EnvironmentType.physical',
          outputDir.path,
        ],
      );

      copyPhysicalFrameworkDsymCommand = FakeCommand(
        command: <String>[
          'rsync',
          '-av',
          '--delete',
          '--filter',
          '- .DS_Store/',
          '--chmod=Du=rwx,Dgo=rx,Fu=rw,Fgo=r',
          'Artifact.flutterFrameworkDsym.TargetPlatform.ios.debug.EnvironmentType.physical',
          outputDir.path,
        ],
      );

      copyPhysicalFrameworkDsymCommandFailure = FakeCommand(
        command: <String>[
          'rsync',
          '-av',
          '--delete',
          '--filter',
          '- .DS_Store/',
          '--chmod=Du=rwx,Dgo=rx,Fu=rw,Fgo=r',
          'Artifact.flutterFrameworkDsym.TargetPlatform.ios.debug.EnvironmentType.physical',
          outputDir.path,
        ],
        exitCode: 1,
      );

      lipoCommandNonFatResult = FakeCommand(
        command: <String>['lipo', '-info', binary.path],
        stdout: 'Non-fat file:',
      );

      lipoVerifyArm64Command = FakeCommand(
        command: <String>['lipo', binary.path, '-verify_arch', 'arm64'],
      );

      xattrCommand = FakeCommand(
        command: <String>['xattr', '-r', '-d', 'com.apple.FinderInfo', binary.path],
      );

      adHocCodesignCommand = FakeCommand(
        command: <String>['codesign', '--force', '--sign', '-', '--timestamp=none', binary.path],
      );
    });

    testWithoutContext('iphonesimulator', () async {
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'x86_64', kSdkRoot: 'path/to/iPhoneSimulator.sdk'},
      );

      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            'rsync',
            '-av',
            '--delete',
            '--filter',
            '- .DS_Store/',
            '--chmod=Du=rwx,Dgo=rx,Fu=rw,Fgo=r',
            'Artifact.flutterFramework.TargetPlatform.ios.debug.EnvironmentType.simulator',
            outputDir.path,
          ],
          onRun: (_) => binary.createSync(recursive: true),
        ),
        lipoCommandNonFatResult,
        FakeCommand(command: <String>['lipo', binary.path, '-verify_arch', 'x86_64']),
        xattrCommand,
        adHocCodesignCommand,
      ]);
      await const DebugUnpackIOS().build(environment);

      expect(
        logger.traceText,
        contains('Skipping lipo for non-fat file output/Flutter.framework/Flutter'),
      );
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('fails when frameworks missing', () async {
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'arm64', kSdkRoot: 'path/to/iPhoneOS.sdk'},
      );
      processManager.addCommand(copyPhysicalFrameworkCommand);
      await expectLater(
        const DebugUnpackIOS().build(environment),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'description',
            contains('Flutter.framework/Flutter does not exist, cannot thin'),
          ),
        ),
      );
    });

    testWithoutContext('fails when framework dSYM copy fails', () async {
      binary.createSync(recursive: true);
      final Directory dSYM = fileSystem.directory(
        artifacts.getArtifactPath(
          Artifact.flutterFrameworkDsym,
          platform: TargetPlatform.ios,
          mode: BuildMode.debug,
          environmentType: EnvironmentType.physical,
        ),
      );
      dSYM.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'arm64', kSdkRoot: 'path/to/iPhoneOS.sdk'},
      );
      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        copyPhysicalFrameworkDsymCommandFailure,
      ]);
      await expectLater(
        const DebugUnpackIOS().build(environment),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'description',
            contains('Failed to copy framework dSYM'),
          ),
        ),
      );
    });

    testWithoutContext('fails when requested archs missing from framework', () async {
      binary.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'arm64 x86_64', kSdkRoot: 'path/to/iPhoneOS.sdk'},
      );

      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        FakeCommand(
          command: <String>['lipo', '-info', binary.path],
          stdout: 'Architectures in the fat file:',
        ),
        FakeCommand(
          command: <String>['lipo', binary.path, '-verify_arch', 'arm64', 'x86_64'],
          exitCode: 1,
        ),
      ]);

      await expectLater(
        const DebugUnpackIOS().build(environment),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'description',
            contains(
              'does not contain architectures "arm64 x86_64".\n\n'
              'lipo -info:\nArchitectures in the fat file:',
            ),
          ),
        ),
      );
    });

    testWithoutContext('fails when lipo extract fails', () async {
      binary.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'arm64 x86_64', kSdkRoot: 'path/to/iPhoneOS.sdk'},
      );

      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        FakeCommand(
          command: <String>['lipo', '-info', binary.path],
          stdout: 'Architectures in the fat file:',
        ),
        FakeCommand(command: <String>['lipo', binary.path, '-verify_arch', 'arm64', 'x86_64']),
        FakeCommand(
          command: <String>[
            'lipo',
            '-output',
            binary.path,
            '-extract',
            'arm64',
            '-extract',
            'x86_64',
            binary.path,
          ],
          exitCode: 1,
          stderr: 'lipo error',
        ),
      ]);

      await expectLater(
        const DebugUnpackIOS().build(environment),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'description',
            contains(
              'Failed to extract architectures "arm64 x86_64" for output/Flutter.framework/Flutter.\n\n'
              'stderr:\n'
              'lipo error\n\n'
              'lipo -info:\nArchitectures in the fat file:',
            ),
          ),
        ),
      );
    });

    testWithoutContext('skips thin framework', () async {
      binary.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'arm64', kSdkRoot: 'path/to/iPhoneOS.sdk'},
      );

      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        lipoCommandNonFatResult,
        lipoVerifyArm64Command,
        xattrCommand,
        adHocCodesignCommand,
      ]);
      await const DebugUnpackIOS().build(environment);

      expect(
        logger.traceText,
        contains('Skipping lipo for non-fat file output/Flutter.framework/Flutter'),
      );

      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('thins fat framework', () async {
      binary.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'arm64 x86_64', kSdkRoot: 'path/to/iPhoneOS.sdk'},
      );

      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        FakeCommand(
          command: <String>['lipo', '-info', binary.path],
          stdout: 'Architectures in the fat file:',
        ),
        FakeCommand(command: <String>['lipo', binary.path, '-verify_arch', 'arm64', 'x86_64']),
        FakeCommand(
          command: <String>[
            'lipo',
            '-output',
            binary.path,
            '-extract',
            'arm64',
            '-extract',
            'x86_64',
            binary.path,
          ],
        ),
        xattrCommand,
        adHocCodesignCommand,
      ]);

      await const DebugUnpackIOS().build(environment);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('strips framework', () async {
      binary.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{kIosArchs: 'arm64', kSdkRoot: 'path/to/iPhoneOS.sdk'},
      );

      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        lipoCommandNonFatResult,
        lipoVerifyArm64Command,
        xattrCommand,
        adHocCodesignCommand,
      ]);
      await const DebugUnpackIOS().build(environment);

      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('fails when codesign fails', () async {
      binary.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{
          kIosArchs: 'arm64',
          kSdkRoot: 'path/to/iPhoneOS.sdk',
          kCodesignIdentity: 'ABC123',
        },
      );

      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        lipoCommandNonFatResult,
        lipoVerifyArm64Command,
        xattrCommand,
        FakeCommand(
          command: <String>[
            'codesign',
            '--force',
            '--sign',
            'ABC123',
            '--timestamp=none',
            binary.path,
          ],
          exitCode: 1,
          stderr: 'codesign error',
          stdout: 'codesign info',
        ),
      ]);

      await expectLater(
        const DebugUnpackIOS().build(environment),
        throwsA(
          isException.having(
            (Exception exception) => exception.toString(),
            'description',
            contains(
              'Failed to codesign output/Flutter.framework/Flutter with identity ABC123.\ncodesign info\ncodesign error',
            ),
          ),
        ),
      );

      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('codesigns framework', () async {
      binary.createSync(recursive: true);
      final Directory dSYM = fileSystem.directory(
        artifacts.getArtifactPath(
          Artifact.flutterFrameworkDsym,
          platform: TargetPlatform.ios,
          mode: BuildMode.debug,
          environmentType: EnvironmentType.physical,
        ),
      );
      dSYM.createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        processManager: processManager,
        artifacts: artifacts,
        logger: logger,
        fileSystem: fileSystem,
        outputDir: outputDir,
        defines: <String, String>{
          kIosArchs: 'arm64',
          kSdkRoot: 'path/to/iPhoneOS.sdk',
          kCodesignIdentity: 'ABC123',
        },
      );

      processManager.addCommands(<FakeCommand>[
        copyPhysicalFrameworkCommand,
        copyPhysicalFrameworkDsymCommand,
        lipoCommandNonFatResult,
        lipoVerifyArm64Command,
        xattrCommand,
        FakeCommand(
          command: <String>[
            'codesign',
            '--force',
            '--sign',
            'ABC123',
            '--timestamp=none',
            binary.path,
          ],
        ),
      ]);
      await const DebugUnpackIOS().build(environment);

      expect(processManager, hasNoRemainingExpectations);
    });
  });
}
