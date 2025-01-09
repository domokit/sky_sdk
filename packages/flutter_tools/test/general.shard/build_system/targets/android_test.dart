// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/deferred_component.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/depfile.dart';
import 'package:flutter_tools/src/build_system/targets/android.dart';
import 'package:flutter_tools/src/convert.dart';

import '../../../src/common.dart';
import '../../../src/context.dart';
import '../../../src/fake_process_manager.dart';

void main() {
  late FakeProcessManager processManager;
  late FileSystem fileSystem;
  late Artifacts artifacts;
  late Logger logger;

  setUp(() {
    logger = BufferLogger.test();
    fileSystem = MemoryFileSystem.test();
    processManager = FakeProcessManager.empty();
    artifacts = Artifacts.test();
  });

  testWithoutContext('Android AOT targets has analyticsName', () {
    expect(androidArmProfile.analyticsName, 'android_aot');
  });

  testUsingContext('debug bundle contains expected resources', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{kBuildMode: 'debug'},
      processManager: processManager,
      artifacts: artifacts,
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);

    // create pre-requisites.
    environment.buildDir.childFile('app.dill').writeAsStringSync('abcd');
    environment.buildDir.childFile('native_assets.json').createSync();
    fileSystem
        .file(artifacts.getArtifactPath(Artifact.vmSnapshotData, mode: BuildMode.debug))
        .createSync(recursive: true);
    fileSystem
        .file(artifacts.getArtifactPath(Artifact.isolateSnapshotData, mode: BuildMode.debug))
        .createSync(recursive: true);

    await const DebugAndroidApplication().build(environment);

    expect(
      fileSystem
          .file(fileSystem.path.join('out', 'flutter_assets', 'isolate_snapshot_data'))
          .existsSync(),
      true,
    );
    expect(
      fileSystem
          .file(fileSystem.path.join('out', 'flutter_assets', 'vm_snapshot_data'))
          .existsSync(),
      true,
    );
    expect(
      fileSystem
          .file(fileSystem.path.join('out', 'flutter_assets', 'kernel_blob.bin'))
          .existsSync(),
      true,
    );
  });

  testUsingContext('debug bundle contains expected resources with bundle SkSL', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{kBuildMode: 'debug'},
      inputs: <String, String>{kBundleSkSLPath: 'bundle.sksl'},
      processManager: processManager,
      artifacts: artifacts,
      fileSystem: fileSystem,
      logger: logger,
      engineVersion: '2',
    );
    environment.buildDir.createSync(recursive: true);
    fileSystem
        .file('bundle.sksl')
        .writeAsStringSync(
          json.encode(<String, Object>{
            'engineRevision': '2',
            'platform': 'android',
            'data': <String, Object>{'A': 'B'},
          }),
        );

    // create pre-requisites.
    environment.buildDir.childFile('app.dill').writeAsStringSync('abcd');
    environment.buildDir.childFile('native_assets.json').createSync();
    fileSystem
        .file(artifacts.getArtifactPath(Artifact.vmSnapshotData, mode: BuildMode.debug))
        .createSync(recursive: true);
    fileSystem
        .file(artifacts.getArtifactPath(Artifact.isolateSnapshotData, mode: BuildMode.debug))
        .createSync(recursive: true);

    await const DebugAndroidApplication().build(environment);

    expect(
      fileSystem.file(fileSystem.path.join('out', 'flutter_assets', 'isolate_snapshot_data')),
      exists,
    );
    expect(
      fileSystem.file(fileSystem.path.join('out', 'flutter_assets', 'vm_snapshot_data')),
      exists,
    );
    expect(
      fileSystem.file(fileSystem.path.join('out', 'flutter_assets', 'kernel_blob.bin')),
      exists,
    );
    expect(
      fileSystem.file(fileSystem.path.join('out', 'flutter_assets', 'io.flutter.shaders.json')),
      exists,
    );
  });

  testWithoutContext('profile bundle contains expected resources', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{kBuildMode: 'profile'},
      artifacts: artifacts,
      processManager: processManager,
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);

    // create pre-requisites.
    environment.buildDir.childFile('app.so').writeAsStringSync('abcd');

    await const ProfileAndroidApplication().build(environment);

    expect(fileSystem.file(fileSystem.path.join('out', 'app.so')).existsSync(), true);
  });

  testWithoutContext('release bundle contains expected resources', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{kBuildMode: 'release'},
      artifacts: artifacts,
      processManager: processManager,
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);

    // create pre-requisites.
    environment.buildDir.childFile('app.so').writeAsStringSync('abcd');

    await const ReleaseAndroidApplication().build(environment);

    expect(fileSystem.file(fileSystem.path.join('out', 'app.so')).existsSync(), true);
  });

  testUsingContext('AndroidAot can build provided target platform', () async {
    processManager = FakeProcessManager.empty();
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{kBuildMode: 'release'},
      artifacts: artifacts,
      processManager: processManager,
      fileSystem: fileSystem,
      logger: logger,
    );
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
          '--elf=${environment.buildDir.childDirectory('arm64-v8a').childFile('app.so').path}',
          '--strip',
          environment.buildDir.childFile('app.dill').path,
        ],
      ),
    );
    environment.buildDir.createSync(recursive: true);
    environment.buildDir.childFile('app.dill').createSync();
    environment.buildDir.childFile('native_assets.json').createSync();
    const AndroidAot androidAot = AndroidAot(TargetPlatform.android_arm64, BuildMode.release);

    await androidAot.build(environment);

    expect(processManager, hasNoRemainingExpectations);
  });

  testUsingContext('AndroidAot provide code size information.', () async {
    processManager = FakeProcessManager.empty();
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{kBuildMode: 'release', kCodeSizeDirectory: 'code_size_1'},
      artifacts: artifacts,
      processManager: processManager,
      fileSystem: fileSystem,
      logger: logger,
    );
    processManager.addCommand(
      FakeCommand(
        command: <String>[
          artifacts.getArtifactPath(
            Artifact.genSnapshot,
            platform: TargetPlatform.android_arm64,
            mode: BuildMode.release,
          ),
          '--deterministic',
          '--write-v8-snapshot-profile-to=code_size_1/snapshot.arm64-v8a.json',
          '--trace-precompiler-to=code_size_1/trace.arm64-v8a.json',
          '--snapshot_kind=app-aot-elf',
          '--elf=${environment.buildDir.childDirectory('arm64-v8a').childFile('app.so').path}',
          '--strip',
          environment.buildDir.childFile('app.dill').path,
        ],
      ),
    );
    environment.buildDir.createSync(recursive: true);
    environment.buildDir.childFile('app.dill').createSync();
    environment.buildDir.childFile('native_assets.json').createSync();
    const AndroidAot androidAot = AndroidAot(TargetPlatform.android_arm64, BuildMode.release);

    await androidAot.build(environment);

    expect(processManager, hasNoRemainingExpectations);
  });

  testUsingContext('kExtraGenSnapshotOptions passes values to gen_snapshot', () async {
    processManager = FakeProcessManager.empty();
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{
        kBuildMode: 'release',
        kExtraGenSnapshotOptions: 'foo,bar,baz=2',
        kTargetPlatform: 'android-arm',
      },
      processManager: processManager,
      artifacts: artifacts,
      fileSystem: fileSystem,
      logger: logger,
    );
    processManager.addCommand(
      FakeCommand(
        command: <String>[
          artifacts.getArtifactPath(
            Artifact.genSnapshot,
            platform: TargetPlatform.android_arm64,
            mode: BuildMode.release,
          ),
          '--deterministic',
          'foo',
          'bar',
          'baz=2',
          '--snapshot_kind=app-aot-elf',
          '--elf=${environment.buildDir.childDirectory('arm64-v8a').childFile('app.so').path}',
          '--strip',
          environment.buildDir.childFile('app.dill').path,
        ],
      ),
    );
    environment.buildDir.createSync(recursive: true);
    environment.buildDir.childFile('app.dill').createSync();
    environment.buildDir.childFile('native_assets.json').createSync();

    await const AndroidAot(TargetPlatform.android_arm64, BuildMode.release).build(environment);
  });

  testUsingContext(
    '--no-strip in kExtraGenSnapshotOptions suppresses --strip gen_snapshot flag',
    () async {
      processManager = FakeProcessManager.empty();
      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        outputDir: fileSystem.directory('out')..createSync(),
        defines: <String, String>{
          kBuildMode: 'release',
          kExtraGenSnapshotOptions: 'foo,--no-strip,bar',
          kTargetPlatform: 'android-arm',
        },
        processManager: processManager,
        artifacts: artifacts,
        fileSystem: fileSystem,
        logger: logger,
      );
      processManager.addCommand(
        FakeCommand(
          command: <String>[
            artifacts.getArtifactPath(
              Artifact.genSnapshot,
              platform: TargetPlatform.android_arm64,
              mode: BuildMode.release,
            ),
            '--deterministic',
            'foo',
            'bar',
            '--snapshot_kind=app-aot-elf',
            '--elf=${environment.buildDir.childDirectory('arm64-v8a').childFile('app.so').path}',
            environment.buildDir.childFile('app.dill').path,
          ],
        ),
      );
      environment.buildDir.createSync(recursive: true);
      environment.buildDir.childFile('app.dill').createSync();
      environment.buildDir.childFile('native_assets.json').createSync();

      await const AndroidAot(TargetPlatform.android_arm64, BuildMode.release).build(environment);
    },
  );

  testWithoutContext('android aot bundle copies so from abi directory', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      defines: <String, String>{kBuildMode: 'release'},
      processManager: processManager,
      artifacts: artifacts,
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);
    const AndroidAot androidAot = AndroidAot(TargetPlatform.android_arm64, BuildMode.release);
    const AndroidAotBundle androidAotBundle = AndroidAotBundle(androidAot);
    // Create required files.
    environment.buildDir
        .childDirectory('arm64-v8a')
        .childFile('app.so')
        .createSync(recursive: true);

    await androidAotBundle.build(environment);

    expect(
      environment.outputDir.childDirectory('arm64-v8a').childFile('app.so').existsSync(),
      true,
    );
  });

  test('copyDeferredComponentSoFiles copies all files to correct locations', () {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('/out')..createSync(),
      defines: <String, String>{kBuildMode: 'release'},
      processManager: processManager,
      artifacts: artifacts,
      fileSystem: fileSystem,
      logger: logger,
    );
    final File so1 = fileSystem.file('/unit2/abi1/part.so');
    so1.createSync(recursive: true);
    so1.writeAsStringSync('lib1');
    final File so2 = fileSystem.file('/unit3/abi1/part.so');
    so2.createSync(recursive: true);
    so2.writeAsStringSync('lib2');
    final File so3 = fileSystem.file('/unit4/abi1/part.so');
    so3.createSync(recursive: true);
    so3.writeAsStringSync('lib3');

    final File so4 = fileSystem.file('/unit2/abi2/part.so');
    so4.createSync(recursive: true);
    so4.writeAsStringSync('lib1');
    final File so5 = fileSystem.file('/unit3/abi2/part.so');
    so5.createSync(recursive: true);
    so5.writeAsStringSync('lib2');
    final File so6 = fileSystem.file('/unit4/abi2/part.so');
    so6.createSync(recursive: true);
    so6.writeAsStringSync('lib3');

    final List<DeferredComponent> components = <DeferredComponent>[
      DeferredComponent(name: 'component2', libraries: <String>['lib1']),
      DeferredComponent(name: 'component3', libraries: <String>['lib2']),
    ];
    final List<LoadingUnit> loadingUnits = <LoadingUnit>[
      LoadingUnit(id: 2, libraries: <String>['lib1'], path: '/unit2/abi1/part.so'),
      LoadingUnit(id: 3, libraries: <String>['lib2'], path: '/unit3/abi1/part.so'),
      LoadingUnit(id: 4, libraries: <String>['lib3'], path: '/unit4/abi1/part.so'),

      LoadingUnit(id: 2, libraries: <String>['lib1'], path: '/unit2/abi2/part.so'),
      LoadingUnit(id: 3, libraries: <String>['lib2'], path: '/unit3/abi2/part.so'),
      LoadingUnit(id: 4, libraries: <String>['lib3'], path: '/unit4/abi2/part.so'),
    ];
    for (final DeferredComponent component in components) {
      component.assignLoadingUnits(loadingUnits);
    }
    final Directory buildDir = fileSystem.directory('/build');
    if (!buildDir.existsSync()) {
      buildDir.createSync(recursive: true);
    }
    final Depfile depfile = copyDeferredComponentSoFiles(
      environment,
      components,
      loadingUnits,
      buildDir,
      <String>['abi1', 'abi2'],
      BuildMode.release,
    );
    expect(depfile.inputs.length, 6);
    expect(depfile.outputs.length, 6);

    expect(depfile.inputs[0].path, so1.path);
    expect(depfile.inputs[1].path, so2.path);
    expect(depfile.inputs[2].path, so4.path);
    expect(depfile.inputs[3].path, so5.path);
    expect(depfile.inputs[4].path, so3.path);
    expect(depfile.inputs[5].path, so6.path);

    expect(depfile.outputs[0].readAsStringSync(), so1.readAsStringSync());
    expect(depfile.outputs[1].readAsStringSync(), so2.readAsStringSync());
    expect(depfile.outputs[2].readAsStringSync(), so4.readAsStringSync());
    expect(depfile.outputs[3].readAsStringSync(), so5.readAsStringSync());
    expect(depfile.outputs[4].readAsStringSync(), so3.readAsStringSync());
    expect(depfile.outputs[5].readAsStringSync(), so6.readAsStringSync());

    expect(
      depfile.outputs[0].path,
      '/build/component2/intermediates/flutter/release/deferred_libs/abi1/libapp.so-2.part.so',
    );
    expect(
      depfile.outputs[1].path,
      '/build/component3/intermediates/flutter/release/deferred_libs/abi1/libapp.so-3.part.so',
    );

    expect(
      depfile.outputs[2].path,
      '/build/component2/intermediates/flutter/release/deferred_libs/abi2/libapp.so-2.part.so',
    );
    expect(
      depfile.outputs[3].path,
      '/build/component3/intermediates/flutter/release/deferred_libs/abi2/libapp.so-3.part.so',
    );

    expect(depfile.outputs[4].path, '/out/abi1/app.so-4.part.so');
    expect(depfile.outputs[5].path, '/out/abi2/app.so-4.part.so');
  });

  test('copyDeferredComponentSoFiles copies files for only listed abis', () {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('/out')..createSync(),
      defines: <String, String>{kBuildMode: 'release'},
      processManager: processManager,
      artifacts: artifacts,
      fileSystem: fileSystem,
      logger: logger,
    );
    final File so1 = fileSystem.file('/unit2/abi1/part.so');
    so1.createSync(recursive: true);
    so1.writeAsStringSync('lib1');
    final File so2 = fileSystem.file('/unit3/abi1/part.so');
    so2.createSync(recursive: true);
    so2.writeAsStringSync('lib2');
    final File so3 = fileSystem.file('/unit4/abi1/part.so');
    so3.createSync(recursive: true);
    so3.writeAsStringSync('lib3');

    final File so4 = fileSystem.file('/unit2/abi2/part.so');
    so4.createSync(recursive: true);
    so4.writeAsStringSync('lib1');
    final File so5 = fileSystem.file('/unit3/abi2/part.so');
    so5.createSync(recursive: true);
    so5.writeAsStringSync('lib2');
    final File so6 = fileSystem.file('/unit4/abi2/part.so');
    so6.createSync(recursive: true);
    so6.writeAsStringSync('lib3');

    final List<DeferredComponent> components = <DeferredComponent>[
      DeferredComponent(name: 'component2', libraries: <String>['lib1']),
      DeferredComponent(name: 'component3', libraries: <String>['lib2']),
    ];
    final List<LoadingUnit> loadingUnits = <LoadingUnit>[
      LoadingUnit(id: 2, libraries: <String>['lib1'], path: '/unit2/abi1/part.so'),
      LoadingUnit(id: 3, libraries: <String>['lib2'], path: '/unit3/abi1/part.so'),
      LoadingUnit(id: 4, libraries: <String>['lib3'], path: '/unit4/abi1/part.so'),

      LoadingUnit(id: 2, libraries: <String>['lib1'], path: '/unit2/abi2/part.so'),
      LoadingUnit(id: 3, libraries: <String>['lib2'], path: '/unit3/abi2/part.so'),
      LoadingUnit(id: 4, libraries: <String>['lib3'], path: '/unit4/abi2/part.so'),
    ];
    for (final DeferredComponent component in components) {
      component.assignLoadingUnits(loadingUnits);
    }
    final Directory buildDir = fileSystem.directory('/build');
    if (!buildDir.existsSync()) {
      buildDir.createSync(recursive: true);
    }
    final Depfile depfile = copyDeferredComponentSoFiles(
      environment,
      components,
      loadingUnits,
      buildDir,
      <String>['abi1'],
      BuildMode.release,
    );
    expect(depfile.inputs.length, 3);
    expect(depfile.outputs.length, 3);

    expect(depfile.inputs[0].path, so1.path);
    expect(depfile.inputs[1].path, so2.path);
    expect(depfile.inputs[2].path, so3.path);

    expect(depfile.outputs[0].readAsStringSync(), so1.readAsStringSync());
    expect(depfile.outputs[1].readAsStringSync(), so2.readAsStringSync());
    expect(depfile.outputs[2].readAsStringSync(), so3.readAsStringSync());

    expect(
      depfile.outputs[0].path,
      '/build/component2/intermediates/flutter/release/deferred_libs/abi1/libapp.so-2.part.so',
    );
    expect(
      depfile.outputs[1].path,
      '/build/component3/intermediates/flutter/release/deferred_libs/abi1/libapp.so-3.part.so',
    );

    expect(depfile.outputs[2].path, '/out/abi1/app.so-4.part.so');
  });

  testUsingContext(
    'DebugAndroidApplication with impeller and shader compilation',
    () async {
      // Create impellerc to work around fallback detection logic.
      fileSystem
          .file(artifacts.getHostArtifact(HostArtifact.impellerc))
          .createSync(recursive: true);

      final Environment environment = Environment.test(
        fileSystem.currentDirectory,
        outputDir: fileSystem.directory('out')..createSync(),
        defines: <String, String>{kBuildMode: 'debug'},
        processManager: processManager,
        artifacts: artifacts,
        fileSystem: fileSystem,
        logger: logger,
      );
      environment.buildDir.createSync(recursive: true);

      // create pre-requisites.
      environment.buildDir.childFile('app.dill').writeAsStringSync('abcd');
      environment.buildDir.childFile('native_assets.json').createSync();
      fileSystem
          .file(artifacts.getArtifactPath(Artifact.vmSnapshotData, mode: BuildMode.debug))
          .createSync(recursive: true);
      fileSystem
          .file(artifacts.getArtifactPath(Artifact.isolateSnapshotData, mode: BuildMode.debug))
          .createSync(recursive: true);
      fileSystem
          .file('pubspec.yaml')
          .writeAsStringSync('name: hello\nflutter:\n  shaders:\n    - shader.glsl');
      fileSystem
          .directory('.dart_tool')
          .childFile('package_config.json')
          .createSync(recursive: true);
      fileSystem.file('shader.glsl').writeAsStringSync('test');

      processManager.addCommands(<FakeCommand>[
        const FakeCommand(
          command: <String>[
            'HostArtifact.impellerc',
            '--sksl',
            '--runtime-stage-gles',
            '--runtime-stage-gles3',
            '--runtime-stage-vulkan',
            '--iplr',
            '--sl=out/flutter_assets/shader.glsl',
            '--spirv=out/flutter_assets/shader.glsl.spirv',
            '--input=/shader.glsl',
            '--input-type=frag',
            '--include=/',
            '--include=/./shader_lib',
          ],
        ),
      ]);

      await const DebugAndroidApplication().build(environment);
      expect(processManager, hasNoRemainingExpectations);

      expect(
        fileSystem
            .file(fileSystem.path.join('out', 'flutter_assets', 'isolate_snapshot_data'))
            .existsSync(),
        true,
      );
      expect(
        fileSystem
            .file(fileSystem.path.join('out', 'flutter_assets', 'vm_snapshot_data'))
            .existsSync(),
        true,
      );
      expect(
        fileSystem
            .file(fileSystem.path.join('out', 'flutter_assets', 'kernel_blob.bin'))
            .existsSync(),
        true,
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );
}
