// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/test/web_test_compiler.dart';
import 'package:flutter_tools/src/web/compile.dart';
import 'package:test/expect.dart';

import '../../src/context.dart';

void main() {
  testUsingContext('web test compiler issues valid compile command', () async {
    final BufferLogger logger = BufferLogger.test();
    final MemoryFileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('project/test/fake_test.dart').createSync(recursive: true);
    fileSystem.file('build/out').createSync(recursive: true);
    fileSystem.file('build/build/out.sources').createSync(recursive: true);
    fileSystem.file('build/build/out.json')
      ..createSync()
      ..writeAsStringSync('{}');
    fileSystem.file('build/build/out.map').createSync();
    fileSystem.file('build/build/out.metadata').createSync();
    final FakePlatform platform = FakePlatform(environment: <String, String>{});
    final Config config = Config(
      Config.kFlutterSettings,
      fileSystem: fileSystem,
      logger: logger,
      platform: platform,
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <Pattern>[
          'Artifact.engineDartAotRuntime.TargetPlatform.web_javascript',
          'Artifact.frontendServerSnapshotForEngineDartSdk.TargetPlatform.web_javascript',
          '--sdk-root',
          'HostArtifact.flutterWebSdk/',
          '--incremental',
          '--target=dartdevc',
          '--experimental-emit-debug-metadata',
          '-DFLUTTER_WEB_USE_SKIA=true',
          '-DFLUTTER_WEB_USE_SKWASM=false',
          '--output-dill',
          'build/out',
          '--packages',
          '.dart_tool/package_config.json',
          '-Ddart.vm.profile=false',
          '-Ddart.vm.product=false',
          '--enable-asserts',
          '--filesystem-root',
          'project/test',
          '--filesystem-root',
          'build',
          '--filesystem-scheme',
          'org-dartlang-app',
          '--initialize-from-dill',
          RegExp(r'^build\/(?:[a-z0-9]{32})\.cache\.dill$'),
          '--platform',
          'file:///HostArtifact.webPlatformKernelFolder/ddc_outline_sound.dill',
          '--verbosity=error',
          '--sound-null-safety',
        ],
        stdout: 'result abc\nline0\nline1\nabc\nabc build/out 0',
      ),
    ]);
    final WebTestCompiler compiler = WebTestCompiler(
      logger: logger,
      fileSystem: fileSystem,
      platform: FakePlatform(environment: <String, String>{}),
      artifacts: Artifacts.test(),
      processManager: processManager,
      config: config,
    );

    const BuildInfo buildInfo = BuildInfo(
      BuildMode.debug,
      '',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    );

    await compiler.initialize(
      projectDirectory: fileSystem.directory('project'),
      testOutputDir: 'build',
      testFiles: <String>['project/test/fake_test.dart'],
      buildInfo: buildInfo,
      webRenderer: WebRendererMode.canvaskit,
      useWasm: false,
    );

    expect(processManager.hasRemainingExpectations, isFalse);
  });
}
