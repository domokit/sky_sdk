// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ffi';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

// A shared interface for shaders for which you can acquire a native handle
abstract class SkwasmShader implements ui.Shader {
  ShaderHandle get handle;
}

// An implementation that handles the storage, disposal, and finalization of
// a native shader handle.
class SkwasmNativeShader extends SkwasmObjectWrapper<RawShader> implements SkwasmShader {
  SkwasmNativeShader(ShaderHandle handle) : super(handle, _registry);

  static final SkwasmFinalizationRegistry<RawShader> _registry =
      SkwasmFinalizationRegistry<RawShader>(shaderDispose);
}

class SkwasmGradient extends SkwasmNativeShader implements ui.Gradient {
  factory SkwasmGradient.linear({
    required ui.Offset from,
    required ui.Offset to,
    required List<ui.Color> colors,
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix4,
  }) => withStackScope((StackScope scope) {
    assert(() {
      validateColorStops(colors, colorStops);
      return true;
    }());

    final RawPointArray endPoints = scope.convertPointArrayToNative(<ui.Offset>[from, to]);
    final RawColorArray nativeColors = scope.convertColorArrayToNative(colors);
    final Pointer<Float> stops =
        colorStops != null ? scope.convertDoublesToNative(colorStops) : nullptr;
    final Pointer<Float> matrix =
        matrix4 != null ? scope.convertMatrix4toSkMatrix(matrix4) : nullptr;
    final ShaderHandle handle = shaderCreateLinearGradient(
      endPoints,
      nativeColors,
      stops,
      colors.length,
      tileMode.index,
      matrix,
    );
    return SkwasmGradient._(handle);
  });

  factory SkwasmGradient.radial({
    required ui.Offset center,
    required double radius,
    required List<ui.Color> colors,
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix4,
  }) => withStackScope((StackScope scope) {
    assert(() {
      validateColorStops(colors, colorStops);
      return true;
    }());

    final RawColorArray rawColors = scope.convertColorArrayToNative(colors);
    final Pointer<Float> rawStops =
        colorStops != null ? scope.convertDoublesToNative(colorStops) : nullptr;
    final Pointer<Float> matrix =
        matrix4 != null ? scope.convertMatrix4toSkMatrix(matrix4) : nullptr;
    final ShaderHandle handle = shaderCreateRadialGradient(
      center.dx,
      center.dy,
      radius,
      rawColors,
      rawStops,
      colors.length,
      tileMode.index,
      matrix,
    );
    return SkwasmGradient._(handle);
  });

  factory SkwasmGradient.conical({
    required ui.Offset focal,
    required double focalRadius,
    required ui.Offset center,
    required double centerRadius,
    required List<ui.Color> colors,
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    Float32List? matrix4,
  }) => withStackScope((StackScope scope) {
    assert(() {
      validateColorStops(colors, colorStops);
      return true;
    }());

    final RawPointArray endPoints = scope.convertPointArrayToNative(<ui.Offset>[focal, center]);
    final RawColorArray rawColors = scope.convertColorArrayToNative(colors);
    final Pointer<Float> rawStops =
        colorStops != null ? scope.convertDoublesToNative(colorStops) : nullptr;
    final Pointer<Float> matrix =
        matrix4 != null ? scope.convertMatrix4toSkMatrix(matrix4) : nullptr;
    final ShaderHandle handle = shaderCreateConicalGradient(
      endPoints,
      focalRadius,
      centerRadius,
      rawColors,
      rawStops,
      colors.length,
      tileMode.index,
      matrix,
    );
    return SkwasmGradient._(handle);
  });

  factory SkwasmGradient.sweep({
    required ui.Offset center,
    required List<ui.Color> colors,
    List<double>? colorStops,
    ui.TileMode tileMode = ui.TileMode.clamp,
    required double startAngle,
    required double endAngle,
    Float32List? matrix4,
  }) => withStackScope((StackScope scope) {
    assert(() {
      validateColorStops(colors, colorStops);
      return true;
    }());

    final RawColorArray rawColors = scope.convertColorArrayToNative(colors);
    final Pointer<Float> rawStops =
        colorStops != null ? scope.convertDoublesToNative(colorStops) : nullptr;
    final Pointer<Float> matrix =
        matrix4 != null ? scope.convertMatrix4toSkMatrix(matrix4) : nullptr;
    final ShaderHandle handle = shaderCreateSweepGradient(
      center.dx,
      center.dy,
      rawColors,
      rawStops,
      colors.length,
      tileMode.index,
      ui.toDegrees(startAngle),
      ui.toDegrees(endAngle),
      matrix,
    );
    return SkwasmGradient._(handle);
  });

  SkwasmGradient._(super.handle);

  @override
  String toString() => 'Gradient()';
}

class SkwasmImageShader extends SkwasmNativeShader implements ui.ImageShader {
  SkwasmImageShader._(super.handle);

  factory SkwasmImageShader.imageShader(
    SkwasmImage image,
    ui.TileMode tmx,
    ui.TileMode tmy,
    Float64List? matrix4,
    ui.FilterQuality? filterQuality,
  ) {
    if (matrix4 != null) {
      return withStackScope((StackScope scope) {
        final RawMatrix33 localMatrix = scope.convertMatrix4toSkMatrix(matrix4);
        return SkwasmImageShader._(
          shaderCreateFromImage(
            image.handle,
            tmx.index,
            tmy.index,
            (filterQuality ?? ui.FilterQuality.none).index,
            localMatrix,
          ),
        );
      });
    } else {
      return SkwasmImageShader._(
        shaderCreateFromImage(
          image.handle,
          tmx.index,
          tmy.index,
          (filterQuality ?? ui.FilterQuality.none).index,
          nullptr,
        ),
      );
    }
  }
}

class SkwasmFragmentProgram extends SkwasmObjectWrapper<RawRuntimeEffect>
    implements ui.FragmentProgram {
  SkwasmFragmentProgram._(
    this.name,
    RuntimeEffectHandle handle,
    this.floatUniformCount,
    this.childShaderCount,
  ) : super(handle, _registry);

  factory SkwasmFragmentProgram.fromBytes(String name, Uint8List bytes) {
    final ShaderData shaderData = ShaderData.fromBytes(bytes);

    // TODO(jacksongardner): Can we avoid this copy?
    final List<int> sourceData = utf8.encode(shaderData.source);
    final SkStringHandle sourceString = skStringAllocate(sourceData.length);
    final Pointer<Int8> sourceBuffer = skStringGetData(sourceString);
    int i = 0;
    for (final int byte in sourceData) {
      sourceBuffer[i] = byte;
      i++;
    }
    final RuntimeEffectHandle handle = runtimeEffectCreate(sourceString);
    skStringFree(sourceString);
    return SkwasmFragmentProgram._(name, handle, shaderData.floatCount, shaderData.textureCount);
  }

  static final SkwasmFinalizationRegistry<RawRuntimeEffect> _registry =
      SkwasmFinalizationRegistry<RawRuntimeEffect>(runtimeEffectDispose);

  final String name;
  final int floatUniformCount;
  final int childShaderCount;

  @override
  ui.FragmentShader fragmentShader() => SkwasmFragmentShader(this);

  int get uniformSize => runtimeEffectGetUniformSize(handle);
}

class SkwasmShaderData extends SkwasmObjectWrapper<RawSkData> {
  SkwasmShaderData(int size) : super(skDataCreate(size), _registry);

  static final SkwasmFinalizationRegistry<RawSkData> _registry =
      SkwasmFinalizationRegistry<RawSkData>(skDataDispose);
}

// This class does not inherit from SkwasmNativeShader, as its handle might
// change over time if the uniforms or image shaders are changed. Instead this
// wraps a SkwasmNativeShader that it creates and destroys on demand. It does
// implement SkwasmShader though, in order to provide the handle for the
// underlying shader object.
class SkwasmFragmentShader implements SkwasmShader, ui.FragmentShader {
  SkwasmFragmentShader(SkwasmFragmentProgram program)
    : _program = program,
      _uniformData = SkwasmShaderData(program.uniformSize),
      _floatUniformCount = program.floatUniformCount,
      _childShaders = List<SkwasmShader?>.filled(program.childShaderCount, null);

  @override
  ShaderHandle get handle {
    if (_nativeShader == null) {
      final ShaderHandle newHandle = withStackScope((StackScope s) {
        Pointer<ShaderHandle> childShaders = nullptr;
        if (_childShaders.isNotEmpty) {
          childShaders = s.allocPointerArray(_childShaders.length).cast<ShaderHandle>();
          for (int i = 0; i < _childShaders.length; i++) {
            final SkwasmShader? child = _childShaders[i];
            childShaders[i] = child != null ? child.handle : nullptr;
          }
        }
        return shaderCreateRuntimeEffectShader(
          _program.handle,
          _uniformData.handle,
          childShaders,
          _childShaders.length,
        );
      });
      _nativeShader = SkwasmNativeShader(newHandle);
    }
    return _nativeShader!.handle;
  }

  SkwasmShader? _nativeShader;
  final SkwasmFragmentProgram _program;
  final SkwasmShaderData _uniformData;
  bool _isDisposed = false;
  final int _floatUniformCount;
  final List<SkwasmShader?> _childShaders;

  @override
  void dispose() {
    assert(!_isDisposed);
    _nativeShader?.dispose();
    _uniformData.dispose();
    _isDisposed = true;
  }

  @override
  bool get debugDisposed => _isDisposed;

  @override
  void setFloat(int index, double value) {
    if (_nativeShader != null) {
      // Invalidate the previous shader so that it is recreated with the new
      // uniform data.
      _nativeShader!.dispose();
      _nativeShader = null;
    }
    final Pointer<Float> dataPointer = skDataGetPointer(_uniformData.handle).cast<Float>();
    dataPointer[index] = value;
  }

  @override
  void setImageSampler(int index, ui.Image image) {
    if (_nativeShader != null) {
      // Invalidate the previous shader so that it is recreated with the new
      // child shaders.
      _nativeShader!.dispose();
      _nativeShader = null;
    }

    final SkwasmImageShader shader = SkwasmImageShader.imageShader(
      image as SkwasmImage,
      ui.TileMode.clamp,
      ui.TileMode.clamp,
      null,
      ui.FilterQuality.none,
    );
    final SkwasmShader? oldShader = _childShaders[index];
    _childShaders[index] = shader;
    oldShader?.dispose();

    final Pointer<Float> dataPointer = skDataGetPointer(_uniformData.handle).cast<Float>();
    dataPointer[_floatUniformCount + index * 2] = image.width.toDouble();
    dataPointer[_floatUniformCount + index * 2 + 1] = image.height.toDouble();
  }
}
