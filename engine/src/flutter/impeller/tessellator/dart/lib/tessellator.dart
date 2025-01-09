// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: camel_case_types
import 'dart:ffi' as ffi;
import 'dart:io';
import 'dart:typed_data';

/// Determines the winding rule that decides how the interior of a Path is
/// calculated.
///
/// This enum is used by the [VerticesBuilder.tessellate] method.
// must match ordering in geometry/path.h
enum FillType {
  /// The interior is defined by a non-zero sum of signed edge crossings.
  nonZero,

  /// The interior is defined by an odd number of edge crossings.
  evenOdd,
}

/// Information about how to approximate points on a curved path segment.
///
/// In particular, the values in this object control how many vertices to
/// generate when approximating curves, and what tolerances to use when
/// calculating the sharpness of curves.
///
/// Used by [VerticesBuilder.tessellate].
class SmoothingApproximation {
  /// Creates a new smoothing approximation instance with default values.
  const SmoothingApproximation({this.scale = 1.0, this.angleTolerance = 0.0, this.cuspLimit = 0.0});

  /// The scaling coefficient to use when translating to screen coordinates.
  ///
  /// Values approaching 0.0 will generate smoother looking curves with a
  /// greater number of vertices, and will be more expensive to calculate.
  final double scale;

  /// The tolerance value in radians for calculating sharp angles.
  ///
  /// Values approaching 0.0 will provide more accurate approximation of sharp
  /// turns. A 0.0 value means angle conditions are not considered at all.
  final double angleTolerance;

  /// An angle in radians at which to introduce bevel cuts.
  ///
  /// Values greater than zero will restirct the sharpness of bevel cuts on
  /// turns.
  final double cuspLimit;
}

/// Creates vertices from path commands.
///
/// First, build up the path contours with the [moveTo], [lineTo], [cubicTo],
/// and [close] methods. All methods expect absolute coordinates.
///
/// Then, use the [tessellate] method to create a [Float32List] of vertex pairs.
///
/// Finally, use the [dispose] method to clean up native resources. After
/// [dispose] has been called, this class must not be used again.
class VerticesBuilder {
  /// Constructs a [VerticesBuilder] instance to which path commands can be
  /// added.
  VerticesBuilder() : _builder = _createPathFn();

  ffi.Pointer<_PathBuilder>? _builder;
  final List<ffi.Pointer<_Vertices>> _vertices = <ffi.Pointer<_Vertices>>[];

  /// Adds a move verb to the absolute coordinates x,y.
  void moveTo(double x, double y) {
    assert(_builder != null);
    _moveToFn(_builder!, x, y);
  }

  /// Adds a line verb to the absolute coordinates x,y.
  void lineTo(double x, double y) {
    assert(_builder != null);
    _lineToFn(_builder!, x, y);
  }

  /// Adds a cubic Bezier curve with x1,y1 as the first control point, x2,y2 as
  /// the second control point, and end point x3,y3.
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) {
    assert(_builder != null);
    _cubicToFn(_builder!, x1, y1, x2, y2, x3, y3);
  }

  /// Adds a close command to the start of the current contour.
  void close() {
    assert(_builder != null);
    _closeFn(_builder!, true);
  }

  /// Tessellates the path created by the previous method calls into a list of
  /// vertices.
  Float32List tessellate({
    FillType fillType = FillType.nonZero,
    SmoothingApproximation smoothing = const SmoothingApproximation(),
  }) {
    assert(_vertices.isEmpty);
    assert(_builder != null);
    final ffi.Pointer<_Vertices> vertices = _tessellateFn(
      _builder!,
      fillType.index,
      smoothing.scale,
      smoothing.angleTolerance,
      smoothing.cuspLimit,
    );
    _vertices.add(vertices);
    return vertices.ref.points.asTypedList(vertices.ref.size);
  }

  /// Releases native resources.
  ///
  /// After calling dispose, this class must not be used again.
  void dispose() {
    assert(_builder != null);
    _vertices.forEach(_destroyVerticesFn);
    _destroyFn(_builder!);
    _vertices.clear();
    _builder = null;
  }
}

// TODO(dnfield): Figure out where to put this.
// https://github.com/flutter/flutter/issues/99563
final ffi.DynamicLibrary _dylib = () {
  if (Platform.isWindows) {
    return ffi.DynamicLibrary.open('tessellator.dll');
  } else if (Platform.isIOS || Platform.isMacOS) {
    return ffi.DynamicLibrary.open('libtessellator.dylib');
  } else if (Platform.isAndroid || Platform.isLinux) {
    return ffi.DynamicLibrary.open('libtessellator.so');
  }
  throw UnsupportedError('Unknown platform: ${Platform.operatingSystem}');
}();

final class _Vertices extends ffi.Struct {
  external ffi.Pointer<ffi.Float> points;

  @ffi.Uint32()
  external int size;
}

final class _PathBuilder extends ffi.Opaque {}

typedef _CreatePathBuilderType = ffi.Pointer<_PathBuilder> Function();
typedef _create_path_builder_type = ffi.Pointer<_PathBuilder> Function();

final _CreatePathBuilderType _createPathFn = _dylib
    .lookupFunction<_create_path_builder_type, _CreatePathBuilderType>('CreatePathBuilder');

typedef _MoveToType = void Function(ffi.Pointer<_PathBuilder>, double, double);
typedef _move_to_type = ffi.Void Function(ffi.Pointer<_PathBuilder>, ffi.Float, ffi.Float);

final _MoveToType _moveToFn = _dylib.lookupFunction<_move_to_type, _MoveToType>('MoveTo');

typedef _LineToType = void Function(ffi.Pointer<_PathBuilder>, double, double);
typedef _line_to_type = ffi.Void Function(ffi.Pointer<_PathBuilder>, ffi.Float, ffi.Float);

final _LineToType _lineToFn = _dylib.lookupFunction<_line_to_type, _LineToType>('LineTo');

typedef _CubicToType =
    void Function(ffi.Pointer<_PathBuilder>, double, double, double, double, double, double);
typedef _cubic_to_type =
    ffi.Void Function(
      ffi.Pointer<_PathBuilder>,
      ffi.Float,
      ffi.Float,
      ffi.Float,
      ffi.Float,
      ffi.Float,
      ffi.Float,
    );

final _CubicToType _cubicToFn = _dylib.lookupFunction<_cubic_to_type, _CubicToType>('CubicTo');

typedef _CloseType = void Function(ffi.Pointer<_PathBuilder>, bool);
typedef _close_type = ffi.Void Function(ffi.Pointer<_PathBuilder>, ffi.Bool);

final _CloseType _closeFn = _dylib.lookupFunction<_close_type, _CloseType>('Close');

typedef _TessellateType =
    ffi.Pointer<_Vertices> Function(ffi.Pointer<_PathBuilder>, int, double, double, double);
typedef _tessellate_type =
    ffi.Pointer<_Vertices> Function(
      ffi.Pointer<_PathBuilder>,
      ffi.Int,
      ffi.Float,
      ffi.Float,
      ffi.Float,
    );

final _TessellateType _tessellateFn = _dylib.lookupFunction<_tessellate_type, _TessellateType>(
  'Tessellate',
);

typedef _DestroyType = void Function(ffi.Pointer<_PathBuilder>);
typedef _destroy_type = ffi.Void Function(ffi.Pointer<_PathBuilder>);

final _DestroyType _destroyFn = _dylib.lookupFunction<_destroy_type, _DestroyType>(
  'DestroyPathBuilder',
);

typedef _DestroyVerticesType = void Function(ffi.Pointer<_Vertices>);
typedef _destroy_vertices_type = ffi.Void Function(ffi.Pointer<_Vertices>);

final _DestroyVerticesType _destroyVerticesFn = _dylib
    .lookupFunction<_destroy_vertices_type, _DestroyVerticesType>('DestroyVertices');
