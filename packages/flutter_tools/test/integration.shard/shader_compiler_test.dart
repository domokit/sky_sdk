// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/globals.dart' as globals;

import '../src/common.dart';
import '../src/context.dart';

void main() {
  late BufferLogger logger;

  setUp(() {
    logger = BufferLogger.test();
  });

  Future<void> testCompileShader(String source) async {
    final Directory tmpDir = globals.fs.systemTempDirectory.createTempSync('shader_compiler_test.');
    final File file = tmpDir.childFile('test_shader.frag')..writeAsStringSync(source);
    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: globals.processManager,
      logger: logger,
      fileSystem: globals.fs,
      artifacts: globals.artifacts!,
    );
    await shaderCompiler.compileShader(
      input: file,
      outputPath: tmpDir.childFile('test_shader.frag.out').path,
      targetPlatform: TargetPlatform.tester,
    );
  }

  testUsingContext('impellerc .iplr output has correct permissions', () async {
    if (globals.platform.isWindows) {
      return;
    }

    final String flutterRoot = getFlutterRoot();
    final String inkSparklePath = globals.fs.path.join(
      flutterRoot,
      'packages',
      'flutter',
      'lib',
      'src',
      'material',
      'shaders',
      'ink_sparkle.frag',
    );
    final Directory tmpDir = globals.fs.systemTempDirectory.createTempSync('shader_compiler_test.');
    final String inkSparkleOutputPath = globals.fs.path.join(tmpDir.path, 'ink_sparkle.frag');

    final ShaderCompiler shaderCompiler = ShaderCompiler(
      processManager: globals.processManager,
      logger: logger,
      fileSystem: globals.fs,
      artifacts: globals.artifacts!,
    );
    final bool compileResult = await shaderCompiler.compileShader(
      input: globals.fs.file(inkSparklePath),
      outputPath: inkSparkleOutputPath,
      targetPlatform: TargetPlatform.tester,
    );
    final File resultFile = globals.fs.file(inkSparkleOutputPath);

    expect(compileResult, true);
    expect(resultFile.existsSync(), true);

    final int expectedMode = int.parse('644', radix: 8);
    expect(resultFile.statSync().mode & expectedMode, equals(expectedMode));
  });

  testUsingContext('Compilation error with in storage', () async {
    const String kShaderWithInput = '''
in float foo;

out vec4 fragColor;

void main() {
  fragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
''';

    expect(
      () => testCompileShader(kShaderWithInput),
      throwsA(
        isA<ShaderCompilerException>().having(
          (ShaderCompilerException exception) => exception.message,
          'message',
          contains('SkSL does not support inputs'),
        ),
      ),
    );
  });

  testUsingContext('Compilation error with UBO', () async {
    const String kShaderWithInput = '''
uniform Data {
  vec4 foo;
} data;

out vec4 fragColor;

void main() {
  fragColor = data.foo;
}
''';

    expect(
      () => testCompileShader(kShaderWithInput),
      throwsA(
        isA<ShaderCompilerException>().having(
          (ShaderCompilerException exception) => exception.message,
          'message',
          contains('SkSL does not support UBOs or SSBOs'),
        ),
      ),
    );
  });

  testUsingContext(
    'Compilation error with texture arguments besides position or sampler',
    () async {
      const String kShaderWithInput = '''
uniform sampler2D tex;

out vec4 fragColor;

void main() {
  fragColor = texture(tex, vec2(0.5, 0.3), 0.5);
}
''';

      expect(
        () => testCompileShader(kShaderWithInput),
        throwsA(
          isA<ShaderCompilerException>().having(
            (ShaderCompilerException exception) => exception.message,
            'message',
            contains('Only sampler and position arguments are supported in texture() calls'),
          ),
        ),
      );
    },
  );

  testUsingContext('Compilation error with uint8 uniforms', () async {
    const String kShaderWithInput = '''
#version 310 es

layout(location = 0) uniform uint foo;
layout(location = 0) out vec4 fragColor;

void main() {}
''';

    expect(
      () => testCompileShader(kShaderWithInput),
      throwsA(
        isA<ShaderCompilerException>().having(
          (ShaderCompilerException exception) => exception.message,
          'message',
          contains('SkSL does not support unsigned integers'),
        ),
      ),
    );
  });
}
