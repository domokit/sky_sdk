// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/devfs.dart';

import '../../../src/common.dart';
import '../../../src/fake_process_manager.dart';

const String fragDir = '/shaders';
const String shaderLibDir = '/./shader_lib';
const String fragPath = '/shaders/my_shader.frag';
const String notFragPath = '/shaders/not_a_frag.file';
const String outputSpirvPath = '/output/shaders/my_shader.frag.spirv';
const String outputPath = '/output/shaders/my_shader.frag';

void main() {
  late BufferLogger logger;
  late MemoryFileSystem fileSystem;
  late Artifacts artifacts;
  late String impellerc;

  setUp(() {
    artifacts = Artifacts.test();
    fileSystem = MemoryFileSystem.test();
    logger = BufferLogger.test();
    impellerc = artifacts.getHostArtifact(HostArtifact.impellerc).path;

    fileSystem.file(impellerc).createSync(recursive: true);
    fileSystem.file(fragPath).createSync(recursive: true);
    fileSystem.file(notFragPath).createSync(recursive: true);
  });

  testWithoutContext('compileShader invokes impellerc for .frag files and web target', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--iplr',
          '--json',
          '--sl=$outputPath',
          '--spirv=$outputSpirvPath',
          '--input=$fragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        onRun: (_) {
          fileSystem.file(outputPath).createSync(recursive: true);
          fileSystem.file(outputSpirvPath).createSync(recursive: true);
        },
      ),
    ]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );

    expect(
      await shaderCompiler.compileShader(
        input: fileSystem.file(fragPath),
        outputPath: outputPath,
        targetPlatform: TargetPlatform.web_javascript,
      ),
      true,
    );
    expect(fileSystem.file(outputPath).existsSync(), true);
    expect(fileSystem.file(outputSpirvPath).existsSync(), false);
  });

  testWithoutContext(
    'compileShader invokes impellerc for .frag files and metal ios target',
    () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            impellerc,
            '--sksl',
            '--runtime-stage-metal',
            '--iplr',
            '--sl=$outputPath',
            '--spirv=$outputPath.spirv',
            '--input=$fragPath',
            '--input-type=frag',
            '--include=$fragDir',
            '--include=$shaderLibDir',
          ],
          onRun: (_) {
            fileSystem.file(outputPath).createSync(recursive: true);
          },
        ),
      ]);
      final ShaderCompiler shaderCompiler = ShaderCompiler(
        processManager: processManager,
        logger: logger,
        fileSystem: fileSystem,
        artifacts: artifacts,
      );

      expect(
        await shaderCompiler.compileShader(
          input: fileSystem.file(fragPath),
          outputPath: outputPath,
          targetPlatform: TargetPlatform.ios,
        ),
        true,
      );
      expect(fileSystem.file(outputPath).existsSync(), true);
    },
  );

  testWithoutContext('compileShader invokes impellerc for .frag files and Android', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--runtime-stage-gles',
          '--runtime-stage-gles3',
          '--runtime-stage-vulkan',
          '--iplr',
          '--sl=$outputPath',
          '--spirv=$outputPath.spirv',
          '--input=$fragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        onRun: (_) {
          fileSystem.file(outputPath).createSync(recursive: true);
        },
      ),
    ]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );

    expect(
      await shaderCompiler.compileShader(
        input: fileSystem.file(fragPath),
        outputPath: outputPath,
        targetPlatform: TargetPlatform.android,
      ),
      true,
    );
    expect(fileSystem.file(outputPath).existsSync(), true);
  });

  testWithoutContext('compileShader invokes impellerc for non-.frag files', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--iplr',
          '--json',
          '--sl=$outputPath',
          '--spirv=$outputSpirvPath',
          '--input=$notFragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        onRun: (_) {
          fileSystem.file(outputPath).createSync(recursive: true);
          fileSystem.file(outputSpirvPath).createSync(recursive: true);
        },
      ),
    ]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );

    expect(
      await shaderCompiler.compileShader(
        input: fileSystem.file(notFragPath),
        outputPath: outputPath,
        targetPlatform: TargetPlatform.web_javascript,
      ),
      true,
    );
    expect(fileSystem.file(outputPath).existsSync(), true);
    expect(fileSystem.file(outputSpirvPath).existsSync(), false);
  });

  testWithoutContext('compileShader throws an exception when impellerc fails', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--iplr',
          '--json',
          '--sl=$outputPath',
          '--spirv=$outputSpirvPath',
          '--input=$notFragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        stdout: 'impellerc stdout',
        stderr: 'impellerc stderr',
        exitCode: 1,
      ),
    ]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );

    try {
      await shaderCompiler.compileShader(
        input: fileSystem.file(notFragPath),
        outputPath: outputPath,
        targetPlatform: TargetPlatform.web_javascript,
      );
      fail('unreachable');
    } on ShaderCompilerException catch (e) {
      expect(e.toString(), contains('impellerc stdout:\nimpellerc stdout'));
      expect(e.toString(), contains('impellerc stderr:\nimpellerc stderr'));
    }

    expect(fileSystem.file(outputPath).existsSync(), false);
  });

  testWithoutContext('DevelopmentShaderCompiler can compile for android non-impeller', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--runtime-stage-gles',
          '--runtime-stage-gles3',
          '--runtime-stage-vulkan',
          '--iplr',
          '--sl=/.tmp_rand0/0.8255140718871702.temp',
          '--spirv=/.tmp_rand0/0.8255140718871702.temp.spirv',
          '--input=$fragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        onRun: (_) {
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv').createSync();
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp')
            ..createSync()
            ..writeAsBytesSync(<int>[1, 2, 3, 4]);
        },
      ),
    ]);
    fileSystem.file(fragPath).writeAsBytesSync(<int>[1, 2, 3, 4]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );
    final DevelopmentShaderCompiler developmentShaderCompiler = DevelopmentShaderCompiler(
      shaderCompiler: shaderCompiler,
      fileSystem: fileSystem,
      random: math.Random(0),
    );

    developmentShaderCompiler.configureCompiler(TargetPlatform.android);

    final DevFSContent? content = await developmentShaderCompiler.recompileShader(
      DevFSFileContent(fileSystem.file(fragPath)),
    );

    expect(await content!.contentsAsBytes(), <int>[1, 2, 3, 4]);
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv'), isNot(exists));
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp'), isNot(exists));
  });

  testWithoutContext(
    'DevelopmentShaderCompiler can compile for Flutter Tester with Impeller and Vulkan',
    () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            impellerc,
            '--sksl',
            '--runtime-stage-vulkan',
            '--iplr',
            '--sl=/.tmp_rand0/0.8255140718871702.temp',
            '--spirv=/.tmp_rand0/0.8255140718871702.temp.spirv',
            '--input=$fragPath',
            '--input-type=frag',
            '--include=$fragDir',
            '--include=$shaderLibDir',
          ],
          onRun: (_) {
            fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv').createSync();
            fileSystem.file('/.tmp_rand0/0.8255140718871702.temp')
              ..createSync()
              ..writeAsBytesSync(<int>[1, 2, 3, 4]);
          },
        ),
      ]);
      fileSystem.file(fragPath).writeAsBytesSync(<int>[1, 2, 3, 4]);
      final ShaderCompiler shaderCompiler = ShaderCompiler(
        processManager: processManager,
        logger: logger,
        fileSystem: fileSystem,
        artifacts: artifacts,
      );
      final DevelopmentShaderCompiler developmentShaderCompiler = DevelopmentShaderCompiler(
        shaderCompiler: shaderCompiler,
        fileSystem: fileSystem,
        random: math.Random(0),
      );

      developmentShaderCompiler.configureCompiler(TargetPlatform.tester);

      final DevFSContent? content = await developmentShaderCompiler.recompileShader(
        DevFSFileContent(fileSystem.file(fragPath)),
      );

      expect(await content!.contentsAsBytes(), <int>[1, 2, 3, 4]);
      expect(processManager.hasRemainingExpectations, false);
    },
  );

  testWithoutContext('DevelopmentShaderCompiler can compile for android with impeller', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--runtime-stage-gles',
          '--runtime-stage-gles3',
          '--runtime-stage-vulkan',
          '--iplr',
          '--sl=/.tmp_rand0/0.8255140718871702.temp',
          '--spirv=/.tmp_rand0/0.8255140718871702.temp.spirv',
          '--input=$fragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        onRun: (_) {
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv').createSync();
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp')
            ..createSync()
            ..writeAsBytesSync(<int>[1, 2, 3, 4]);
        },
      ),
    ]);
    fileSystem.file(fragPath).writeAsBytesSync(<int>[1, 2, 3, 4]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );
    final DevelopmentShaderCompiler developmentShaderCompiler = DevelopmentShaderCompiler(
      shaderCompiler: shaderCompiler,
      fileSystem: fileSystem,
      random: math.Random(0),
    );

    developmentShaderCompiler.configureCompiler(TargetPlatform.android);

    final DevFSContent? content = await developmentShaderCompiler.recompileShader(
      DevFSFileContent(fileSystem.file(fragPath)),
    );

    expect(await content!.contentsAsBytes(), <int>[1, 2, 3, 4]);
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv'), isNot(exists));
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp'), isNot(exists));
  });

  testWithoutContext(
    'DevelopmentShaderCompiler can compile for Flutter Tester with Impeller and Vulkan',
    () async {
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            impellerc,
            '--sksl',
            '--runtime-stage-vulkan',
            '--iplr',
            '--sl=/.tmp_rand0/0.8255140718871702.temp',
            '--spirv=/.tmp_rand0/0.8255140718871702.temp.spirv',
            '--input=$fragPath',
            '--input-type=frag',
            '--include=$fragDir',
            '--include=$shaderLibDir',
          ],
          onRun: (List<String> args) {
            fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv').createSync();
            fileSystem.file('/.tmp_rand0/0.8255140718871702.temp')
              ..createSync()
              ..writeAsBytesSync(<int>[1, 2, 3, 4]);
          },
        ),
      ]);
      fileSystem.file(fragPath).writeAsBytesSync(<int>[1, 2, 3, 4]);
      final ShaderCompiler shaderCompiler = ShaderCompiler(
        processManager: processManager,
        logger: logger,
        fileSystem: fileSystem,
        artifacts: artifacts,
      );
      final DevelopmentShaderCompiler developmentShaderCompiler = DevelopmentShaderCompiler(
        shaderCompiler: shaderCompiler,
        fileSystem: fileSystem,
        random: math.Random(0),
      );

      developmentShaderCompiler.configureCompiler(TargetPlatform.tester);

      final DevFSContent? content = await developmentShaderCompiler.recompileShader(
        DevFSFileContent(fileSystem.file(fragPath)),
      );

      expect(await content!.contentsAsBytes(), <int>[1, 2, 3, 4]);
      expect(processManager.hasRemainingExpectations, false);
    },
  );

  testWithoutContext('DevelopmentShaderCompiler can compile for android with impeller', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--runtime-stage-gles',
          '--runtime-stage-gles3',
          '--runtime-stage-vulkan',
          '--iplr',
          '--sl=/.tmp_rand0/0.8255140718871702.temp',
          '--spirv=/.tmp_rand0/0.8255140718871702.temp.spirv',
          '--input=$fragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        onRun: (List<String> args) {
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv').createSync();
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp')
            ..createSync()
            ..writeAsBytesSync(<int>[1, 2, 3, 4]);
        },
      ),
    ]);
    fileSystem.file(fragPath).writeAsBytesSync(<int>[1, 2, 3, 4]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );
    final DevelopmentShaderCompiler developmentShaderCompiler = DevelopmentShaderCompiler(
      shaderCompiler: shaderCompiler,
      fileSystem: fileSystem,
      random: math.Random(0),
    );

    developmentShaderCompiler.configureCompiler(TargetPlatform.android);

    final DevFSContent? content = await developmentShaderCompiler.recompileShader(
      DevFSFileContent(fileSystem.file(fragPath)),
    );

    expect(await content!.contentsAsBytes(), <int>[1, 2, 3, 4]);
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv'), isNot(exists));
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp'), isNot(exists));
  });

  testWithoutContext('DevelopmentShaderCompiler can compile JSON for web targets', () async {
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          impellerc,
          '--sksl',
          '--iplr',
          '--json',
          '--sl=/.tmp_rand0/0.8255140718871702.temp',
          '--spirv=/.tmp_rand0/0.8255140718871702.temp.spirv',
          '--input=$fragPath',
          '--input-type=frag',
          '--include=$fragDir',
          '--include=$shaderLibDir',
        ],
        onRun: (_) {
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv').createSync();
          fileSystem.file('/.tmp_rand0/0.8255140718871702.temp')
            ..createSync()
            ..writeAsBytesSync(<int>[1, 2, 3, 4]);
        },
      ),
    ]);
    fileSystem.file(fragPath).writeAsBytesSync(<int>[1, 2, 3, 4]);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: processManager,
      logger: logger,
      fileSystem: fileSystem,
      artifacts: artifacts,
    );
    final DevelopmentShaderCompiler developmentShaderCompiler = DevelopmentShaderCompiler(
      shaderCompiler: shaderCompiler,
      fileSystem: fileSystem,
      random: math.Random(0),
    );

    developmentShaderCompiler.configureCompiler(TargetPlatform.web_javascript);

    final DevFSContent? content = await developmentShaderCompiler.recompileShader(
      DevFSFileContent(fileSystem.file(fragPath)),
    );

    expect(await content!.contentsAsBytes(), <int>[1, 2, 3, 4]);
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp.spirv'), isNot(exists));
    expect(fileSystem.file('/.tmp_rand0/0.8255140718871702.temp'), isNot(exists));
  });
}
