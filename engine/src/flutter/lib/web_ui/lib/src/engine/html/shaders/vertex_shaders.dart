// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

import '../../browser_detection.dart';
import 'shader_builder.dart';

/// Provides common shaders used for gradients and drawVertices APIs.
abstract final class VertexShaders {
  static final Uint16List vertexIndicesForRect = Uint16List.fromList(<int>[0, 1, 2, 2, 3, 0]);

  /// Cached vertex shaders.
  static String? _baseVertexShader;
  static String? _textureVertexShader;

  /// Creates a vertex shader transforms pixel space [Vertices.positions] to
  /// final clipSpace -1..1 coordinates with inverted Y Axis.
  ///     #version 300 es
  ///     layout (location=0) in vec4 position;
  ///     layout (location=1) in vec4 color;
  ///     uniform mat4 u_ctransform;
  ///     uniform vec4 u_scale;
  ///     uniform vec4 u_shift;
  ///     out vec4 vColor;
  ///     void main() {
  ///       gl_Position = ((u_ctransform * position) * u_scale) + u_shift;
  ///       v_color = color.zyxw;
  ///     }
  static String writeBaseVertexShader() {
    if (_baseVertexShader == null) {
      final ShaderBuilder builder = ShaderBuilder(webGLVersion);
      builder.addIn(ShaderType.kVec4, name: 'position');
      builder.addIn(ShaderType.kVec4, name: 'color');
      builder.addUniform(ShaderType.kMat4, name: 'u_ctransform');
      builder.addUniform(ShaderType.kVec4, name: 'u_scale');
      builder.addUniform(ShaderType.kVec4, name: 'u_shift');
      builder.addOut(ShaderType.kVec4, name: 'v_color');
      final ShaderMethod method = builder.addMethod('main');
      method.addStatement('gl_Position = ((u_ctransform * position) * u_scale) + u_shift;');
      method.addStatement('v_color = color.zyxw;');
      _baseVertexShader = builder.build();
    }
    return _baseVertexShader!;
  }

  static String writeTextureVertexShader() {
    if (_textureVertexShader == null) {
      final ShaderBuilder builder = ShaderBuilder(webGLVersion);
      builder.addIn(ShaderType.kVec4, name: 'position');
      builder.addUniform(ShaderType.kMat4, name: 'u_ctransform');
      builder.addUniform(ShaderType.kVec4, name: 'u_scale');
      builder.addUniform(ShaderType.kVec4, name: 'u_textransform');
      builder.addUniform(ShaderType.kVec4, name: 'u_shift');
      builder.addOut(ShaderType.kVec2, name: 'v_texcoord');
      final ShaderMethod method = builder.addMethod('main');
      method.addStatement('gl_Position = ((u_ctransform * position) * u_scale) + u_shift;');
      method.addStatement(
        'v_texcoord = vec2((u_textransform.z + position.x) * u_textransform.x, '
        '((u_textransform.w + position.y) * u_textransform.y));',
      );
      _textureVertexShader = builder.build();
    }
    return _textureVertexShader!;
  }
}

abstract final class FragmentShaders {
  static String writeTextureFragmentShader(
    bool isWebGl2,
    ui.TileMode? tileModeX,
    ui.TileMode? tileModeY,
  ) {
    final ShaderBuilder builder = ShaderBuilder.fragment(webGLVersion);
    builder.floatPrecision = ShaderPrecision.kMedium;
    builder.addIn(ShaderType.kVec2, name: 'v_texcoord');
    builder.addUniform(ShaderType.kSampler2D, name: 'u_texture');
    final ShaderMethod method = builder.addMethod('main');
    if (isWebGl2 ||
        tileModeX == null ||
        tileModeY == null ||
        (tileModeX == ui.TileMode.clamp && tileModeY == ui.TileMode.clamp)) {
      method.addStatement(
        '${builder.fragmentColor.name} = '
        '${builder.texture2DFunction}(u_texture, v_texcoord);',
      );
    } else {
      // Repeat and mirror are not supported for webgl1. Write code to
      // adjust texture coordinate.
      //
      // This will write u and v floats, clamp/repeat and mirror the value and
      // pass it to sampler.
      method.addTileStatements('v_texcoord.x', 'u', tileModeX);
      method.addTileStatements('v_texcoord.y', 'v', tileModeY);
      method.addStatement('vec2 uv = vec2(u, v);');
      method.addStatement(
        '${builder.fragmentColor.name} = '
        '${builder.texture2DFunction}(u_texture, uv);',
      );
    }
    return builder.build();
  }
}
