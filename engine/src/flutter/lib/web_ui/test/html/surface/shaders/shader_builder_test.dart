// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

import '../../../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  const String mat2Sample = 'mat2(1.1, 2.1, 1.2, 2.2)';
  const String mat3Sample =
      'mat3(1.1, 2.1, 3.1, // first column (not row!)\n'
      '1.2, 2.2, 3.2, // second column\n'
      '1.3, 2.3, 3.3  // third column\n'
      ')';
  const String mat4Sample =
      'mat3(1.1, 2.1, 3.1, 4.1,\n'
      '1.2, 2.2, 3.2, 4.2,\n'
      '1.3, 2.3, 3.3, 4.3,\n'
      '1.4, 2.4, 3.4, 4.4,\n'
      ')';

  setUpAll(() async {
    await bootstrapAndRunApp();
  });

  group('Shader Declarations', () {
    test('Constant declaration WebGL1', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl1);
      builder.addConst(ShaderType.kBool, 'false');
      builder.addConst(ShaderType.kInt, '0');
      builder.addConst(ShaderType.kFloat, '1.0');
      builder.addConst(ShaderType.kBVec2, 'bvec2(false, false)');
      builder.addConst(ShaderType.kBVec3, 'bvec3(false, false, true)');
      builder.addConst(ShaderType.kBVec4, 'bvec4(true, true, false, false)');
      builder.addConst(ShaderType.kIVec2, 'ivec2(1, 2)');
      builder.addConst(ShaderType.kIVec3, 'ivec3(1, 2, 3)');
      builder.addConst(ShaderType.kIVec4, 'ivec4(1, 2, 3, 4)');
      builder.addConst(ShaderType.kVec2, 'vec2(1.0, 2.0)');
      builder.addConst(ShaderType.kVec3, 'vec3(1.0, 2.0, 3.0)');
      builder.addConst(ShaderType.kVec4, 'vec4(1.0, 2.0, 3.0, 4.0)');
      builder.addConst(ShaderType.kMat2, mat2Sample);
      builder.addConst(ShaderType.kMat2, mat2Sample, name: 'transform1');
      builder.addConst(ShaderType.kMat3, mat3Sample);
      builder.addConst(ShaderType.kMat4, mat4Sample);
      expect(
        builder.build(),
        'const bool c_0 = false;\n'
        'const int c_1 = 0;\n'
        'const float c_2 = 1.0;\n'
        'const bvec2 c_3 = bvec2(false, false);\n'
        'const bvec3 c_4 = bvec3(false, false, true);\n'
        'const bvec4 c_5 = bvec4(true, true, false, false);\n'
        'const ivec2 c_6 = ivec2(1, 2);\n'
        'const ivec3 c_7 = ivec3(1, 2, 3);\n'
        'const ivec4 c_8 = ivec4(1, 2, 3, 4);\n'
        'const vec2 c_9 = vec2(1.0, 2.0);\n'
        'const vec3 c_10 = vec3(1.0, 2.0, 3.0);\n'
        'const vec4 c_11 = vec4(1.0, 2.0, 3.0, 4.0);\n'
        'const mat2 c_12 = $mat2Sample;\n'
        'const mat2 transform1 = $mat2Sample;\n'
        'const mat3 c_13 = $mat3Sample;\n'
        'const mat4 c_14 = $mat4Sample;\n',
      );
    });

    test('Constant declaration WebGL2', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl2);
      builder.addConst(ShaderType.kBool, 'false');
      builder.addConst(ShaderType.kInt, '0');
      builder.addConst(ShaderType.kFloat, '1.0');
      builder.addConst(ShaderType.kBVec2, 'bvec2(false, false)');
      builder.addConst(ShaderType.kBVec3, 'bvec3(false, false, true)');
      builder.addConst(ShaderType.kBVec4, 'bvec4(true, true, false, false)');
      builder.addConst(ShaderType.kIVec2, 'ivec2(1, 2)');
      builder.addConst(ShaderType.kIVec3, 'ivec3(1, 2, 3)');
      builder.addConst(ShaderType.kIVec4, 'ivec4(1, 2, 3, 4)');
      builder.addConst(ShaderType.kVec2, 'vec2(1.0, 2.0)');
      builder.addConst(ShaderType.kVec3, 'vec3(1.0, 2.0, 3.0)');
      builder.addConst(ShaderType.kVec4, 'vec4(1.0, 2.0, 3.0, 4.0)');
      builder.addConst(ShaderType.kMat2, mat2Sample);
      builder.addConst(ShaderType.kMat2, mat2Sample, name: 'transform2');
      builder.addConst(ShaderType.kMat3, mat3Sample);
      builder.addConst(ShaderType.kMat4, mat4Sample);
      expect(
        builder.build(),
        '#version 300 es\n'
        'const bool c_0 = false;\n'
        'const int c_1 = 0;\n'
        'const float c_2 = 1.0;\n'
        'const bvec2 c_3 = bvec2(false, false);\n'
        'const bvec3 c_4 = bvec3(false, false, true);\n'
        'const bvec4 c_5 = bvec4(true, true, false, false);\n'
        'const ivec2 c_6 = ivec2(1, 2);\n'
        'const ivec3 c_7 = ivec3(1, 2, 3);\n'
        'const ivec4 c_8 = ivec4(1, 2, 3, 4);\n'
        'const vec2 c_9 = vec2(1.0, 2.0);\n'
        'const vec3 c_10 = vec3(1.0, 2.0, 3.0);\n'
        'const vec4 c_11 = vec4(1.0, 2.0, 3.0, 4.0);\n'
        'const mat2 c_12 = $mat2Sample;\n'
        'const mat2 transform2 = $mat2Sample;\n'
        'const mat3 c_13 = $mat3Sample;\n'
        'const mat4 c_14 = $mat4Sample;\n',
      );
    });

    test('Attribute declaration WebGL1', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl1);
      builder.addIn(ShaderType.kVec4, name: 'position');
      builder.addIn(ShaderType.kVec4);
      expect(
        builder.build(),
        'attribute vec4 position;\n'
        'attribute vec4 attr_0;\n',
      );
    });

    test('in declaration WebGL1', () {
      final ShaderBuilder builder = ShaderBuilder.fragment(WebGLVersion.webgl1);
      builder.addIn(ShaderType.kVec4, name: 'position');
      builder.addIn(ShaderType.kVec4);
      expect(
        builder.build(),
        'varying vec4 position;\n'
        'varying vec4 attr_0;\n',
      );
    });

    test('Attribute declaration WebGL2', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl2);
      builder.addIn(ShaderType.kVec4, name: 'position');
      builder.addIn(ShaderType.kVec4);
      expect(
        builder.build(),
        '#version 300 es\n'
        'in vec4 position;\n'
        'in vec4 attr_0;\n',
      );
    });

    test('Uniform declaration WebGL1', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl1);
      final ShaderDeclaration variable = builder.addUniform(ShaderType.kVec4, name: 'v1');
      expect(variable.name, 'v1');
      expect(variable.dataType, ShaderType.kVec4);
      expect(variable.storage, ShaderStorageQualifier.kUniform);
      builder.addUniform(ShaderType.kVec4);
      expect(
        builder.build(),
        'uniform vec4 v1;\n'
        'uniform vec4 uni_0;\n',
      );
    });

    test('Uniform declaration WebGL2', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl2);
      final ShaderDeclaration variable = builder.addUniform(ShaderType.kVec4, name: 'v1');
      expect(variable.name, 'v1');
      expect(variable.dataType, ShaderType.kVec4);
      expect(variable.storage, ShaderStorageQualifier.kUniform);
      builder.addUniform(ShaderType.kVec4);
      expect(
        builder.build(),
        '#version 300 es\n'
        'uniform vec4 v1;\n'
        'uniform vec4 uni_0;\n',
      );
    });

    test('Float precision', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl2);
      builder.floatPrecision = ShaderPrecision.kLow;
      builder.addUniform(ShaderType.kFloat, name: 'f1');
      expect(
        builder.build(),
        '#version 300 es\n'
        'precision lowp float;\n'
        'uniform float f1;\n',
      );
    });

    test('Integer precision', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl2);
      builder.integerPrecision = ShaderPrecision.kLow;
      builder.addUniform(ShaderType.kInt, name: 'i1');
      expect(
        builder.build(),
        '#version 300 es\n'
        'precision lowp int;\n'
        'uniform int i1;\n',
      );
    });

    test('Method', () {
      final ShaderBuilder builder = ShaderBuilder(WebGLVersion.webgl2);
      builder.floatPrecision = ShaderPrecision.kMedium;
      final ShaderDeclaration variable = builder.addUniform(ShaderType.kFloat, name: 'f1');
      final ShaderMethod m = builder.addMethod('main');
      m.addStatement('f1 = 5.0;');
      expect(
        builder.build(),
        '#version 300 es\n'
        'precision mediump float;\n'
        'uniform float ${variable.name};\n'
        'void main() {\n'
        '  f1 = 5.0;\n'
        '}\n',
      );
    });
  });
}
