// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:dwds/dwds.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/isolated/devfs_web.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';

void main() {
  late FileSystem fileSystem;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
  });

  testWithoutContext('WebExpressionCompiler handles successful expression compilation', () async {
    fileSystem.file('compilerOutput').writeAsStringSync('a');
    final ResidentCompiler residentCompiler = FakeResidentCompiler(
      const CompilerOutput('compilerOutput', 0, <Uri>[]),
    );
    final ExpressionCompiler expressionCompiler = WebExpressionCompiler(
      residentCompiler,
      fileSystem: fileSystem,
    );

    final ExpressionCompilationResult result = await expressionCompiler.compileExpressionToJs(
      '',
      '',
      1,
      1,
      <String, String>{},
      <String, String>{},
      '',
      '',
    );

    expectResult(result, false, 'a');
  });

  testWithoutContext('WebExpressionCompiler handles compilation error', () async {
    fileSystem.file('compilerOutput').writeAsStringSync('Error: a');
    final ResidentCompiler residentCompiler = FakeResidentCompiler(
      const CompilerOutput('compilerOutput', 1, <Uri>[]),
    );
    final ExpressionCompiler expressionCompiler = WebExpressionCompiler(
      residentCompiler,
      fileSystem: fileSystem,
    );

    final ExpressionCompilationResult result = await expressionCompiler.compileExpressionToJs(
      '',
      '',
      1,
      1,
      <String, String>{},
      <String, String>{},
      '',
      '',
    );

    expectResult(result, true, 'Error: a');
  });

  testWithoutContext('WebExpressionCompiler handles internal error', () async {
    final ResidentCompiler residentCompiler = FakeResidentCompiler(null);
    final ExpressionCompiler expressionCompiler = WebExpressionCompiler(
      residentCompiler,
      fileSystem: fileSystem,
    );

    final ExpressionCompilationResult result = await expressionCompiler.compileExpressionToJs(
      '',
      '',
      1,
      1,
      <String, String>{},
      <String, String>{},
      '',
      'a',
    );

    expectResult(result, true, "InternalError: frontend server failed to compile 'a'");
  });
}

void expectResult(ExpressionCompilationResult result, bool isError, String value) {
  expect(
    result,
    const TypeMatcher<ExpressionCompilationResult>()
        .having((ExpressionCompilationResult instance) => instance.isError, 'isError', isError)
        .having((ExpressionCompilationResult instance) => instance.result, 'result', value),
  );
}

class FakeResidentCompiler extends Fake implements ResidentCompiler {
  FakeResidentCompiler(this.output);

  final CompilerOutput? output;

  @override
  Future<CompilerOutput?> compileExpressionToJs(
    String libraryUri,
    int line,
    int column,
    Map<String, String> jsModules,
    Map<String, String> jsFrameValues,
    String moduleName,
    String expression,
  ) async {
    return output;
  }
}
