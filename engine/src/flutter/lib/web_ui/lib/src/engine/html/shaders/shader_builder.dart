// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:ui/ui.dart' as ui;

import '../../browser_detection.dart';

/// Creates shader program for target webgl version.
///
/// See spec at https://www.khronos.org/registry/webgl/specs/latest/1.0/.
///
/// Differences in WebGL2 vs WebGL1.
///   - WebGL2 needs '#version 300 es' to enable the new shading language
///   - vertex attributes have the qualifier 'in' instead of 'attribute'
///   - GLSL 3.00 defines texture and other new and future reserved words.
///   - varying is now called `in`.
///   - GLSL 1.00 has a predefined variable gl_FragColor which now needs to be
///     defined as `out vec4 fragmentColor`.
///   - Texture lookup functions texture2D and textureCube have now been
///     replaced with texture.
///
///  Example usage:
///  ShaderBuilder builder = ShaderBuilder(WebGlVersion.webgl2);
///  ShaderDeclaration u1 = builder.addUniform(ShaderType.kVec4);
///  ShaderMethod method = builder.addMethod('main');
///  method.addStatement('${u1.name} = vec4(1.0, 1.0, 1.0, 0.0);');
///  source = builder.build();
class ShaderBuilder {
  ShaderBuilder(this.version)
    : isWebGl2 = version == WebGLVersion.webgl2,
      _isFragmentShader = false;

  ShaderBuilder.fragment(this.version)
    : isWebGl2 = version == WebGLVersion.webgl2,
      _isFragmentShader = true;

  /// WebGL version.
  final int version;
  final List<ShaderDeclaration> declarations = <ShaderDeclaration>[];
  final List<ShaderMethod> _methods = <ShaderMethod>[];

  /// Precision for integer variables.
  int? integerPrecision;

  /// Precision floating point variables.
  int? floatPrecision;

  /// Counter for generating unique name if name is not specified for attribute.
  int _attribCounter = 0;

  /// Counter for generating unique name if name is not specified for varying.
  int _varyingCounter = 0;

  /// Counter for generating unique name if name is not specified for uniform.
  int _uniformCounter = 0;

  /// Counter for generating unique name if name is not specified for constant.
  int _constCounter = 0;

  final bool isWebGl2;
  final bool _isFragmentShader;

  static const String kOpenGlEs3Header = '#version 300 es';

  /// Lazily allocated fragment color output.
  ShaderDeclaration? _fragmentColorDeclaration;

  /// Returns fragment color declaration for fragment shader.
  ///
  /// This is hard coded for webgl1 as gl_FragColor.
  ShaderDeclaration get fragmentColor {
    _fragmentColorDeclaration ??= ShaderDeclaration(
      isWebGl2 ? 'gFragColor' : 'gl_FragColor',
      ShaderType.kVec4,
      ShaderStorageQualifier.kVarying,
    );
    return _fragmentColorDeclaration!;
  }

  /// Adds an attribute.
  ///
  /// The attribute variable is assigned a value from a object buffer as a
  /// series of graphics primitives are rendered. The value is only accessible
  /// in the vertex shader.
  ShaderDeclaration addIn(int dataType, {String? name}) {
    final ShaderDeclaration attrib = ShaderDeclaration(
      name ?? 'attr_${_attribCounter++}',
      dataType,
      ShaderStorageQualifier.kAttribute,
    );
    declarations.add(attrib);
    return attrib;
  }

  /// Adds a constant.
  ShaderDeclaration addConst(int dataType, String value, {String? name}) {
    final ShaderDeclaration declaration = ShaderDeclaration.constant(
      name ?? 'c_${_constCounter++}',
      dataType,
      value,
    );
    declarations.add(declaration);
    return declaration;
  }

  /// Adds a uniform variable.
  ///
  /// The variable is assigned a value before a gl.draw call.
  /// It is accessible in both the vertex and fragment shaders.
  ///
  ShaderDeclaration addUniform(int dataType, {String? name}) {
    final ShaderDeclaration uniform = ShaderDeclaration(
      name ?? 'uni_${_uniformCounter++}',
      dataType,
      ShaderStorageQualifier.kUniform,
    );
    declarations.add(uniform);
    return uniform;
  }

  /// Adds a varying variable.
  ///
  /// The variable is assigned a value by a vertex shader and
  /// interpolated across the surface of a graphics primitive for each
  /// input to a fragment shader.
  /// It can be used in a fragment shader, but not changed.
  ShaderDeclaration addOut(int dataType, {String? name}) {
    final ShaderDeclaration varying = ShaderDeclaration(
      name ?? 'output_${_varyingCounter++}',
      dataType,
      ShaderStorageQualifier.kVarying,
    );
    declarations.add(varying);
    return varying;
  }

  void _writeVariableDeclaration(StringBuffer sb, ShaderDeclaration variable) {
    switch (variable.storage) {
      case ShaderStorageQualifier.kConst:
        _buffer.write('const ');
      case ShaderStorageQualifier.kAttribute:
        _buffer.write(
          isWebGl2
              ? 'in '
              : _isFragmentShader
              ? 'varying '
              : 'attribute ',
        );
      case ShaderStorageQualifier.kUniform:
        _buffer.write('uniform ');
      case ShaderStorageQualifier.kVarying:
        _buffer.write(isWebGl2 ? 'out ' : 'varying ');
    }
    _buffer.write('${typeToString(variable.dataType)} ${variable.name}');
    if (variable.storage == ShaderStorageQualifier.kConst) {
      _buffer.write(' = ${variable.constValue}');
    }
    _buffer.writeln(';');
  }

  final StringBuffer _buffer = StringBuffer();

  static String typeToString(int dataType) {
    switch (dataType) {
      case ShaderType.kBool:
        return 'bool';
      case ShaderType.kInt:
        return 'int';
      case ShaderType.kFloat:
        return 'float';
      case ShaderType.kBVec2:
        return 'bvec2';
      case ShaderType.kBVec3:
        return 'bvec3';
      case ShaderType.kBVec4:
        return 'bvec4';
      case ShaderType.kIVec2:
        return 'ivec2';
      case ShaderType.kIVec3:
        return 'ivec3';
      case ShaderType.kIVec4:
        return 'ivec4';
      case ShaderType.kVec2:
        return 'vec2';
      case ShaderType.kVec3:
        return 'vec3';
      case ShaderType.kVec4:
        return 'vec4';
      case ShaderType.kMat2:
        return 'mat2';
      case ShaderType.kMat3:
        return 'mat3';
      case ShaderType.kMat4:
        return 'mat4';
      case ShaderType.kSampler1D:
        return 'sampler1D';
      case ShaderType.kSampler2D:
        return 'sampler2D';
      case ShaderType.kSampler3D:
        return 'sampler3D';
      case ShaderType.kVoid:
        return 'void';
    }
    throw ArgumentError();
  }

  ShaderMethod addMethod(String name) {
    final ShaderMethod method = ShaderMethod(name);
    _methods.add(method);
    return method;
  }

  String build() {
    // Write header.
    if (isWebGl2) {
      _buffer.writeln(kOpenGlEs3Header);
    }
    // Write optional precision.
    if (integerPrecision != null) {
      _buffer.writeln('precision ${_precisionToString(integerPrecision!)} int;');
    }
    if (floatPrecision != null) {
      _buffer.writeln('precision ${_precisionToString(floatPrecision!)} float;');
    }
    if (isWebGl2 && _fragmentColorDeclaration != null) {
      _writeVariableDeclaration(_buffer, _fragmentColorDeclaration!);
    }
    for (final ShaderDeclaration decl in declarations) {
      _writeVariableDeclaration(_buffer, decl);
    }
    for (final ShaderMethod method in _methods) {
      method.write(_buffer);
    }
    return _buffer.toString();
  }

  String _precisionToString(int precision) =>
      precision == ShaderPrecision.kLow
          ? 'lowp'
          : precision == ShaderPrecision.kMedium
          ? 'mediump'
          : 'highp';

  String get texture2DFunction => isWebGl2 ? 'texture' : 'texture2D';
}

class ShaderMethod {
  ShaderMethod(this.name);

  final String returnType = 'void';
  final String name;
  final List<String> _statements = <String>[];
  int _indentLevel = 1;

  void indent() {
    ++_indentLevel;
  }

  void unindent() {
    assert(_indentLevel != 1);
    --_indentLevel;
  }

  void addStatement(String statement) {
    String itemToAdd = statement;
    assert(() {
      itemToAdd = '  ' * _indentLevel + statement;
      return true;
    }());
    _statements.add(itemToAdd);
  }

  /// Adds statements to compute tiling in 0..1 coordinate space.
  ///
  /// For clamp we simply assign source value to destination.
  ///
  /// For repeat, we use fractional part of source value.
  ///   float destination = fract(source);
  ///
  /// For mirror, we repeat every 2 units, by scaling and measuring distance
  /// from floor.
  ///   float destination = 1.0 - source;
  ///   destination = abs((destination - 2.0 * floor(destination * 0.5)) - 1.0);
  void addTileStatements(String source, String destination, ui.TileMode tileMode) {
    switch (tileMode) {
      case ui.TileMode.repeated:
        addStatement('float $destination = fract($source);');
      case ui.TileMode.mirror:
        addStatement('float $destination = ($source - 1.0);');
        addStatement(
          '$destination = '
          'abs(($destination - 2.0 * floor($destination * 0.5)) - 1.0);',
        );
      case ui.TileMode.clamp:
      case ui.TileMode.decal:
        addStatement('float $destination = $source;');
    }
  }

  void write(StringBuffer buffer) {
    buffer.writeln('$returnType $name() {');
    _statements.forEach(buffer.writeln);
    buffer.writeln('}');
  }
}

/// WebGl Shader data types.
abstract class ShaderType {
  // Basic types.
  static const int kBool = 0;
  static const int kInt = 1;
  static const int kFloat = 2;
  // Vector types.
  static const int kBVec2 = 3;
  static const int kBVec3 = 4;
  static const int kBVec4 = 5;
  static const int kIVec2 = 6;
  static const int kIVec3 = 7;
  static const int kIVec4 = 8;
  static const int kVec2 = 9;
  static const int kVec3 = 10;
  static const int kVec4 = 11;
  static const int kMat2 = 12;
  static const int kMat3 = 13;
  static const int kMat4 = 14;
  // Textures.
  static const int kSampler1D = 15;
  static const int kSampler2D = 16;
  static const int kSampler3D = 17;
  // Other.
  static const int kVoid = 18;
}

/// Precision of int and float types.
///
/// Integers: 8 bit, 10 bit and 16 bits.
/// Float: 8 bit. 14 bit and 62 bits.
abstract class ShaderPrecision {
  static const int kLow = 0;
  static const int kMedium = 1;
  static const int kHigh = 2;
}

/// GL Variable storage qualifiers.
abstract class ShaderStorageQualifier {
  static const int kConst = 0;
  static const int kAttribute = 1;
  static const int kUniform = 2;
  static const int kVarying = 3;
}

/// Shader variable and constant declaration.
class ShaderDeclaration {
  ShaderDeclaration(this.name, this.dataType, this.storage)
    : assert(!_isGLSLReservedWord(name)),
      constValue = '';

  /// Constructs a constant.
  ShaderDeclaration.constant(this.name, this.dataType, this.constValue)
    : storage = ShaderStorageQualifier.kConst;

  final String name;
  final int dataType;
  final int storage;
  final String constValue;
}

// These are used only in debug mode to assert if used as variable name.
// https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.4.10.pdf
const List<String> _kReservedWords = <String>[
  'attribute',
  'const',
  'uniform',
  'varying',
  'layout',
  'centroid',
  'flat',
  'smooth',
  'noperspective',
  'patch', 'sample',
  'break', 'continue',
  'do', 'for', 'while', 'switch', 'case', 'default', 'if', 'else',
  'subroutine',
  'in', 'out', 'inout', 'float', 'double', 'int',
  'void',
  'bool', 'true', 'false',
  'invariant',
  'discard', 'return',
  'mat2', 'mat3', 'mat4', 'dmat2', 'dmat3', 'dmat4',
  'mat2x2', 'mat2x3', 'mat2x4', 'dmat2x2', 'dmat2x3', 'dmat2x4',
  'mat3x2', 'mat3x3', 'mat3x4', 'dmat3x2', 'dmat3x3', 'dmat3x4',
  'mat4x2', 'mat4x3', 'mat4x4', 'dmat4x2', 'dmat4x3', 'dmat4x4',
  'vec2', 'vec3', 'vec4', 'ivec2', 'ivec3', 'ivec4', 'bvec2', 'bvec3', 'bvec4',
  'dvec2', 'dvec3', 'dvec4',
  'uint', 'uvec2', 'uvec3', 'uvec4',
  'lowp', 'mediump', 'highp', 'precision',
  'sampler1D', 'sampler2D', 'sampler3D', 'samplerCube',
  'sampler1DShadow', 'sampler2DShadow', 'samplerCubeShadow',
  'sampler1DArray', 'sampler2DArray',
  'sampler1DArrayShadow', 'sampler2DArrayShadow',
  'isampler1D', 'isampler2D', 'isampler3D', 'isamplerCube',
  'isampler1DArray', 'isampler2DArray',
  'usampler1D', 'usampler2D', 'usampler3D', 'usamplerCube',
  'usampler1DArray', 'usampler2DArray',
  'sampler2DRect', 'sampler2DRectShadow', 'isampler2DRect', 'usampler2DRect',
  'samplerBuffer', 'isamplerBuffer', 'usamplerBuffer',
  'sampler2DMS', 'isampler2DMS', 'usampler2DMS',
  'sampler2DMSArray', 'isampler2DMSArray', 'usampler2DMSArray',
  'samplerCubeArray', 'samplerCubeArrayShadow', 'isamplerCubeArray',
  'usamplerCubeArray',
  'struct',
  'texture',

  // Reserved for future use, see
  // https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.4.10.pdf
  'active', 'asm', 'cast', 'class', 'common', 'enum', 'extern', 'external',
  'filter', 'fixed', 'fvec2', 'fvec3', 'fvec4', 'goto', 'half', 'hvec2',
  'hvec3', 'hvec4', 'iimage1D', 'iimage1DArray', 'iimage2D', 'iimage2DArray',
  'iimage3D', 'iimageBuffer', 'iimageCube', 'image1D', 'image1DArray',
  'image1DArrayShadow', 'image1DShadow', 'image2D', 'image2DArray',
  'image2DArrayShadow', 'image2DShadow', 'image3D', 'imageBuffer',
  'imageCube', 'inline', 'input', 'interface', 'long',
  'namespace', 'noinline', 'output', 'packed', 'partition', 'public',
  'row_major', 'sampler3DRect', 'short', 'sizeof', 'static', 'superp', 'template', 'this',
  'typedef', 'uimage1D', 'uimage1DArray', 'uimage2D', 'uimage2DArray',
  'uimage3D', 'uimageBuffer', 'uimageCube', 'union', 'unsigned',
  'using', 'volatile',
];

bool _isGLSLReservedWord(String name) {
  return _kReservedWords.contains(name);
}
