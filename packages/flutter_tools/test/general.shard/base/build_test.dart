// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/build.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/macos/xcode.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';

const FakeCommand kWhichSysctlCommand = FakeCommand(command: <String>['which', 'sysctl']);

const FakeCommand kARMCheckCommand = FakeCommand(
  command: <String>['sysctl', 'hw.optional.arm64'],
  exitCode: 1,
);

const List<String> kDefaultClang = <String>[
  '-miphoneos-version-min=12.0',
  '-isysroot',
  'path/to/sdk',
  '-dynamiclib',
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
  '-o',
  'build/foo/App.framework/App',
  'build/foo/snapshot_assembly.o',
];

void main() {
  group('GenSnapshot', () {
    late GenSnapshot genSnapshot;
    late Artifacts artifacts;
    late FakeProcessManager processManager;
    late BufferLogger logger;

    setUp(() async {
      artifacts = Artifacts.test();
      logger = BufferLogger.test();
      processManager = FakeProcessManager.list(<FakeCommand>[]);
      genSnapshot = GenSnapshot(
        artifacts: artifacts,
        logger: logger,
        processManager: processManager,
      );
    });

    testWithoutContext('android_x64', () async {
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_x64,
              mode: BuildMode.release,
            ),
            '--additional_arg',
          ],
        ),
      );

      final int result = await genSnapshot.run(
        snapshotType: SnapshotType(TargetPlatform.android_x64, BuildMode.release),
        additionalArgs: <String>['--additional_arg'],
      );
      expect(result, 0);
    });

    testWithoutContext('iOS arm64', () async {
      final String genSnapshotPath = artifacts.getArtifactPath(
        Artifact.genSnapshot,
        platform: TargetPlatform.ios,
        mode: BuildMode.release,
      );
      processManager.addCommand(
        FakeCommand(command: <String>['${genSnapshotPath}_arm64', '--additional_arg']),
      );

      final int result = await genSnapshot.run(
        snapshotType: SnapshotType(TargetPlatform.ios, BuildMode.release),
        darwinArch: DarwinArch.arm64,
        additionalArgs: <String>['--additional_arg'],
      );
      expect(result, 0);
    });

    testWithoutContext('--strip filters error output from gen_snapshot', () async {
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_x64,
              mode: BuildMode.release,
            ),
            '--strip',
          ],
          stderr: 'ABC\n${GenSnapshot.kIgnoredWarnings.join('\n')}\nXYZ\n',
        ),
      );

      final int result = await genSnapshot.run(
        snapshotType: SnapshotType(TargetPlatform.android_x64, BuildMode.release),
        additionalArgs: <String>['--strip'],
      );

      expect(result, 0);
      expect(logger.errorText, contains('ABC'));
      for (final String ignoredWarning in GenSnapshot.kIgnoredWarnings) {
        expect(logger.errorText, isNot(contains(ignoredWarning)));
      }
      expect(logger.errorText, contains('XYZ'));
    });
  });

  group('AOTSnapshotter', () {
    late MemoryFileSystem fileSystem;
    late AOTSnapshotter snapshotter;
    late Artifacts artifacts;
    late FakeProcessManager processManager;

    setUp(() async {
      fileSystem = MemoryFileSystem.test();
      artifacts = Artifacts.test();
      processManager = FakeProcessManager.empty();
      snapshotter = AOTSnapshotter(
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        xcode: Xcode.test(processManager: processManager),
        artifacts: artifacts,
        processManager: processManager,
      );
    });

    testWithoutContext('does not build iOS with debug build mode', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');

      expect(
        await snapshotter.build(
          platform: TargetPlatform.ios,
          darwinArch: DarwinArch.arm64,
          sdkRoot: 'path/to/sdk',
          buildMode: BuildMode.debug,
          mainPath: 'main.dill',
          outputPath: outputPath,
          dartObfuscation: false,
        ),
        isNot(equals(0)),
      );
    });

    testWithoutContext('does not build android-arm with debug build mode', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');

      expect(
        await snapshotter.build(
          platform: TargetPlatform.android_arm,
          buildMode: BuildMode.debug,
          mainPath: 'main.dill',
          outputPath: outputPath,
          dartObfuscation: false,
        ),
        isNot(0),
      );
    });

    testWithoutContext('does not build android-arm64 with debug build mode', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');

      expect(
        await snapshotter.build(
          platform: TargetPlatform.android_arm64,
          buildMode: BuildMode.debug,
          mainPath: 'main.dill',
          outputPath: outputPath,
          dartObfuscation: false,
        ),
        isNot(0),
      );
    });

    testWithoutContext('builds iOS snapshot with dwarfStackTraces', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      final String assembly = fileSystem.path.join(outputPath, 'snapshot_assembly.S');
      final String debugPath = fileSystem.path.join('foo', 'app.ios-arm64.symbols');
      final String genSnapshotPath = artifacts.getArtifactPath(
        Artifact.genSnapshot,
        platform: TargetPlatform.ios,
        mode: BuildMode.profile,
      );
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            '${genSnapshotPath}_arm64',
            '--deterministic',
            '--snapshot_kind=app-aot-assembly',
            '--assembly=$assembly',
            '--dwarf-stack-traces',
            '--resolve-dwarf-paths',
            '--save-debugging-info=$debugPath',
            'main.dill',
          ],
        ),
        kWhichSysctlCommand,
        kARMCheckCommand,
        const FakeCommand(
          command: <String>[
            'xcrun',
            'cc',
            '-arch',
            'arm64',
            '-miphoneos-version-min=12.0',
            '-isysroot',
            'path/to/sdk',
            '-c',
            'build/foo/snapshot_assembly.S',
            '-o',
            'build/foo/snapshot_assembly.o',
          ],
        ),
        const FakeCommand(command: <String>['xcrun', 'clang', '-arch', 'arm64', ...kDefaultClang]),
        const FakeCommand(
          command: <String>[
            'xcrun',
            'dsymutil',
            '-o',
            'build/foo/App.framework.dSYM',
            'build/foo/App.framework/App',
          ],
        ),
        const FakeCommand(
          command: <String>[
            'xcrun',
            'strip',
            '-x',
            'build/foo/App.framework/App',
            '-o',
            'build/foo/App.framework/App',
          ],
        ),
      ]);

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.ios,
        buildMode: BuildMode.profile,
        mainPath: 'main.dill',
        outputPath: outputPath,
        darwinArch: DarwinArch.arm64,
        sdkRoot: 'path/to/sdk',
        splitDebugInfo: 'foo',
        dartObfuscation: false,
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('builds iOS snapshot with obfuscate', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      final String assembly = fileSystem.path.join(outputPath, 'snapshot_assembly.S');
      final String genSnapshotPath = artifacts.getArtifactPath(
        Artifact.genSnapshot,
        platform: TargetPlatform.ios,
        mode: BuildMode.profile,
      );
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            '${genSnapshotPath}_arm64',
            '--deterministic',
            '--snapshot_kind=app-aot-assembly',
            '--assembly=$assembly',
            '--obfuscate',
            'main.dill',
          ],
        ),
        kWhichSysctlCommand,
        kARMCheckCommand,
        const FakeCommand(
          command: <String>[
            'xcrun',
            'cc',
            '-arch',
            'arm64',
            '-miphoneos-version-min=12.0',
            '-isysroot',
            'path/to/sdk',
            '-c',
            'build/foo/snapshot_assembly.S',
            '-o',
            'build/foo/snapshot_assembly.o',
          ],
        ),
        const FakeCommand(command: <String>['xcrun', 'clang', '-arch', 'arm64', ...kDefaultClang]),
        const FakeCommand(
          command: <String>[
            'xcrun',
            'dsymutil',
            '-o',
            'build/foo/App.framework.dSYM',
            'build/foo/App.framework/App',
          ],
        ),
        const FakeCommand(
          command: <String>[
            'xcrun',
            'strip',
            '-x',
            'build/foo/App.framework/App',
            '-o',
            'build/foo/App.framework/App',
          ],
        ),
      ]);

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.ios,
        buildMode: BuildMode.profile,
        mainPath: 'main.dill',
        outputPath: outputPath,
        darwinArch: DarwinArch.arm64,
        sdkRoot: 'path/to/sdk',
        dartObfuscation: true,
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('builds iOS snapshot', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      final String genSnapshotPath = artifacts.getArtifactPath(
        Artifact.genSnapshot,
        platform: TargetPlatform.ios,
        mode: BuildMode.release,
      );
      processManager.addCommands(<FakeCommand>[
        FakeCommand(
          command: <String>[
            '${genSnapshotPath}_arm64',
            '--deterministic',
            '--snapshot_kind=app-aot-assembly',
            '--assembly=${fileSystem.path.join(outputPath, 'snapshot_assembly.S')}',
            'main.dill',
          ],
        ),
        kWhichSysctlCommand,
        kARMCheckCommand,
        const FakeCommand(
          command: <String>[
            'xcrun',
            'cc',
            '-arch',
            'arm64',
            '-miphoneos-version-min=12.0',
            '-isysroot',
            'path/to/sdk',
            '-c',
            'build/foo/snapshot_assembly.S',
            '-o',
            'build/foo/snapshot_assembly.o',
          ],
        ),
        const FakeCommand(command: <String>['xcrun', 'clang', '-arch', 'arm64', ...kDefaultClang]),
        const FakeCommand(
          command: <String>[
            'xcrun',
            'dsymutil',
            '-o',
            'build/foo/App.framework.dSYM',
            'build/foo/App.framework/App',
          ],
        ),
        const FakeCommand(
          command: <String>[
            'xcrun',
            'strip',
            '-x',
            'build/foo/App.framework/App',
            '-o',
            'build/foo/App.framework/App',
          ],
        ),
      ]);

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.ios,
        buildMode: BuildMode.release,
        mainPath: 'main.dill',
        outputPath: outputPath,
        darwinArch: DarwinArch.arm64,
        sdkRoot: 'path/to/sdk',
        dartObfuscation: false,
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('builds shared library for android-arm (32bit)', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_arm,
              mode: BuildMode.release,
            ),
            '--deterministic',
            '--snapshot_kind=app-aot-elf',
            '--elf=build/foo/app.so',
            '--strip',
            '--no-sim-use-hardfp',
            '--no-use-integer-division',
            'main.dill',
          ],
        ),
      );

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.android_arm,
        buildMode: BuildMode.release,
        mainPath: 'main.dill',
        outputPath: outputPath,
        dartObfuscation: false,
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('builds shared library for android-arm with dwarf stack traces', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      final String debugPath = fileSystem.path.join('foo', 'app.android-arm.symbols');
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_arm,
              mode: BuildMode.release,
            ),
            '--deterministic',
            '--snapshot_kind=app-aot-elf',
            '--elf=build/foo/app.so',
            '--strip',
            '--no-sim-use-hardfp',
            '--no-use-integer-division',
            '--dwarf-stack-traces',
            '--resolve-dwarf-paths',
            '--save-debugging-info=$debugPath',
            'main.dill',
          ],
        ),
      );

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.android_arm,
        buildMode: BuildMode.release,
        mainPath: 'main.dill',
        outputPath: outputPath,
        splitDebugInfo: 'foo',
        dartObfuscation: false,
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('builds shared library for android-arm with obfuscate', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_arm,
              mode: BuildMode.release,
            ),
            '--deterministic',
            '--snapshot_kind=app-aot-elf',
            '--elf=build/foo/app.so',
            '--strip',
            '--no-sim-use-hardfp',
            '--no-use-integer-division',
            '--obfuscate',
            'main.dill',
          ],
        ),
      );

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.android_arm,
        buildMode: BuildMode.release,
        mainPath: 'main.dill',
        outputPath: outputPath,
        dartObfuscation: true,
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext(
      'builds shared library for android-arm without dwarf stack traces due to empty string',
      () async {
        final String outputPath = fileSystem.path.join('build', 'foo');
        processManager.addCommand(
          FakeCommand(
            command: <String>[
              artifacts.getArtifactPath(
                Artifact.genSnapshot,
                platform: TargetPlatform.android_arm,
                mode: BuildMode.release,
              ),
              '--deterministic',
              '--snapshot_kind=app-aot-elf',
              '--elf=build/foo/app.so',
              '--strip',
              '--no-sim-use-hardfp',
              '--no-use-integer-division',
              'main.dill',
            ],
          ),
        );

        final int genSnapshotExitCode = await snapshotter.build(
          platform: TargetPlatform.android_arm,
          buildMode: BuildMode.release,
          mainPath: 'main.dill',
          outputPath: outputPath,
          splitDebugInfo: '',
          dartObfuscation: false,
        );

        expect(genSnapshotExitCode, 0);
        expect(processManager, hasNoRemainingExpectations);
      },
    );

    testWithoutContext('builds shared library for android-arm64', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_arm64,
              mode: BuildMode.release,
            ),
            '--deterministic',
            '--snapshot_kind=app-aot-elf',
            '--elf=build/foo/app.so',
            '--strip',
            'main.dill',
          ],
        ),
      );

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.android_arm64,
        buildMode: BuildMode.release,
        mainPath: 'main.dill',
        outputPath: outputPath,
        dartObfuscation: false,
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('--no-strip in extraGenSnapshotOptions suppresses --strip', () async {
      final String outputPath = fileSystem.path.join('build', 'foo');
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_arm64,
              mode: BuildMode.release,
            ),
            '--deterministic',
            '--snapshot_kind=app-aot-elf',
            '--elf=build/foo/app.so',
            'main.dill',
          ],
        ),
      );

      final int genSnapshotExitCode = await snapshotter.build(
        platform: TargetPlatform.android_arm64,
        buildMode: BuildMode.release,
        mainPath: 'main.dill',
        outputPath: outputPath,
        dartObfuscation: false,
        extraGenSnapshotOptions: const <String>['--no-strip'],
      );

      expect(genSnapshotExitCode, 0);
      expect(processManager, hasNoRemainingExpectations);
    });
  });
}
