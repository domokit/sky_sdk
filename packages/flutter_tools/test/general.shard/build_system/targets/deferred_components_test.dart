// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/build_system/targets/android.dart';
import 'package:flutter_tools/src/build_system/targets/deferred_components.dart';

import '../../../src/common.dart';
import '../../../src/context.dart';

// These tests perform a simple check to verify if the check/task was executed at all.
// Detailed per-check tests are in android/deferred_components_setup_validator_test.dart.
void main() {
  late FileSystem fileSystem;
  late BufferLogger logger;

  setUp(() {
    logger = BufferLogger.test();
    fileSystem = MemoryFileSystem.test();
  });

  testUsingContext('checkAppAndroidManifestComponentLoadingUnitMapping checks runs', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      buildDir: fileSystem.directory('build')..createSync(),
      projectDir: fileSystem.directory('project')..createSync(),
      defines: <String, String>{kDeferredComponents: 'true'},
      artifacts: Artifacts.test(),
      processManager: FakeProcessManager.empty(),
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);
    const AndroidAot androidAot = AndroidAot(TargetPlatform.android_arm64, BuildMode.release);
    const AndroidAotBundle androidAotBundle = AndroidAotBundle(androidAot);
    final AndroidAotDeferredComponentsBundle androidDefBundle = AndroidAotDeferredComponentsBundle(
      androidAotBundle,
    );
    final DeferredComponentsGenSnapshotValidatorTarget validatorTarget =
        DeferredComponentsGenSnapshotValidatorTarget(
          deferredComponentsDependencies: <AndroidAotDeferredComponentsBundle>[androidDefBundle],
          nonDeferredComponentsDependencies: <Target>[],
          title: 'test checks',
          exitOnFail: false,
        );

    await validatorTarget.build(environment);

    // We check the inputs to determine if the task was executed.
    expect(validatorTarget.validator!.inputs.length, 3);
    expect(validatorTarget.validator!.inputs[0].path, 'project/pubspec.yaml');
    expect(
      validatorTarget.validator!.inputs[1].path,
      'project/android/app/src/main/AndroidManifest.xml',
    );
  });

  testUsingContext('checkAgainstLoadingUnitsCache checks runs', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      buildDir: fileSystem.directory('build')..createSync(),
      projectDir: fileSystem.directory('project')..createSync(),
      defines: <String, String>{kDeferredComponents: 'true'},
      artifacts: Artifacts.test(),
      processManager: FakeProcessManager.empty(),
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);
    const AndroidAot androidAot = AndroidAot(TargetPlatform.android_arm64, BuildMode.release);
    const AndroidAotBundle androidAotBundle = AndroidAotBundle(androidAot);
    final AndroidAotDeferredComponentsBundle androidDefBundle = AndroidAotDeferredComponentsBundle(
      androidAotBundle,
    );
    final DeferredComponentsGenSnapshotValidatorTarget validatorTarget =
        DeferredComponentsGenSnapshotValidatorTarget(
          deferredComponentsDependencies: <AndroidAotDeferredComponentsBundle>[androidDefBundle],
          nonDeferredComponentsDependencies: <Target>[],
          title: 'test checks',
          exitOnFail: false,
        );

    await validatorTarget.build(environment);

    // We check the inputs to determine if the task was executed.
    expect(validatorTarget.validator!.inputs.length, 3);
    expect(
      validatorTarget.validator!.inputs[2].path,
      'project/deferred_components_loading_units.yaml',
    );
  });

  testUsingContext('writeLoadingUnitsCache task runs', () async {
    final Environment environment = Environment.test(
      fileSystem.currentDirectory,
      outputDir: fileSystem.directory('out')..createSync(),
      buildDir: fileSystem.directory('build')..createSync(),
      projectDir: fileSystem.directory('project')..createSync(),
      defines: <String, String>{kDeferredComponents: 'true'},
      artifacts: Artifacts.test(),
      processManager: FakeProcessManager.empty(),
      fileSystem: fileSystem,
      logger: logger,
    );
    environment.buildDir.createSync(recursive: true);
    const AndroidAot androidAot = AndroidAot(TargetPlatform.android_arm64, BuildMode.release);
    const AndroidAotBundle androidAotBundle = AndroidAotBundle(androidAot);
    final AndroidAotDeferredComponentsBundle androidDefBundle = AndroidAotDeferredComponentsBundle(
      androidAotBundle,
    );
    final DeferredComponentsGenSnapshotValidatorTarget validatorTarget =
        DeferredComponentsGenSnapshotValidatorTarget(
          deferredComponentsDependencies: <AndroidAotDeferredComponentsBundle>[androidDefBundle],
          nonDeferredComponentsDependencies: <Target>[],
          title: 'test checks',
          exitOnFail: false,
        );

    await validatorTarget.build(environment);

    // We check the inputs to determine if the task was executed.
    expect(validatorTarget.validator!.outputs.length, 1);
    expect(
      validatorTarget.validator!.outputs[0].path,
      'project/deferred_components_loading_units.yaml',
    );
  });
}
