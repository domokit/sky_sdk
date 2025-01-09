// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: camel_case_types, non_constant_identifier_names

import 'dart:ffi' as ffi;
import 'dart:io';
import 'dart:typed_data';

/// Determines the winding rule that decides how the interior of a Path is
/// calculated.
///
/// This enum is used by the [Path] constructor
// must match ordering in //flutter/third_party/skia/include/core/SkPathTypes.h
enum FillType {
  /// The interior is defined by a non-zero sum of signed edge crossings.
  nonZero,

  /// The interior is defined by an odd number of edge crossings.
  evenOdd,
}

/// A set of operations applied to two paths.
// Sync with //flutter/third_party/skia/include/pathops/SkPathOps.h
enum PathOp {
  /// Subtracts the second path from the first.
  difference,

  /// Creates a new path representing the intersection of the first and second.
  intersect,

  /// Creates a new path representing the union of the first and second
  /// (includive-or).
  union,

  /// Creates a new path representing the exclusive-or of two paths.
  xor,

  /// Creates a new path that subtracts the first path from the second.s
  reversedDifference,
}

/// The commands used in a [Path] object.
///
/// This enumeration is a subset of the commands that SkPath supports.
// Sync with //flutter/third_party/skia/include/core/SkPathTypes.h
enum PathVerb {
  /// Picks up the pen and moves it without drawing. Uses two point values.
  moveTo,

  /// A straight line from the current point to the specified point.
  lineTo,

  _quadTo,
  _conicTo,

  /// A cubic bezier curve from the current point.
  ///
  /// The next two points are used as the first control point. The next two
  /// points form the second control point. The next two points form the
  /// target point.
  cubicTo,

  /// A straight line from the current point to the last [moveTo] point.
  close,
}

/// A proxy class for [Path.replay].
///
/// Allows implementations to easily inspect the contents of a [Path].
abstract class PathProxy {
  /// Picks up the pen and moves to absolute coordinates x,y.
  void moveTo(double x, double y);

  /// Draws a straight line from the current point to absolute coordinates x,y.
  void lineTo(double x, double y);

  /// Creates a cubic Bezier curve from the current point to point x3,y3 using
  /// x1,y1 as the first control point and x2,y2 as the second.
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3);

  /// Draws a straight line from the current point to the last [moveTo] point.
  void close();

  /// Called by [Path.replay] to indicate that a new path is being played.
  void reset() {}
}

/// A path proxy that can print the SVG path-data representation of this path.
class SvgPathProxy implements PathProxy {
  final StringBuffer _buffer = StringBuffer();

  @override
  void reset() {
    _buffer.clear();
  }

  @override
  void close() {
    _buffer.write('Z');
  }

  @override
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) {
    _buffer.write('C$x1,$y1 $x2,$y2 $x3,$y3');
  }

  @override
  void lineTo(double x, double y) {
    _buffer.write('L$x,$y');
  }

  @override
  void moveTo(double x, double y) {
    _buffer.write('M$x,$y');
  }

  @override
  String toString() => _buffer.toString();
}

/// Creates a path object to operate on.
///
/// First, build up the path contours with the [moveTo], [lineTo], [cubicTo],
/// and [close] methods. All methods expect absolute coordinates.
///
/// Finally, use the [dispose] method to clean up native resources. After
/// [dispose] has been called, this class must not be used again.
class Path implements PathProxy {
  /// Creates an empty path object with the specified fill type.
  Path([FillType fillType = FillType.nonZero]) : _path = _createPathFn(fillType.index);

  /// Creates a copy of this path.
  factory Path.from(Path other) {
    final Path result = Path(other.fillType);
    other.replay(result);
    return result;
  }

  /// The [FillType] of this path.
  FillType get fillType {
    assert(_path != null);
    return FillType.values[_getFillTypeFn(_path!)];
  }

  ffi.Pointer<_SkPath>? _path;
  ffi.Pointer<_PathData>? _pathData;

  /// The number of points used by each [PathVerb].
  static const Map<PathVerb, int> pointsPerVerb = <PathVerb, int>{
    PathVerb.moveTo: 2,
    PathVerb.lineTo: 2,
    PathVerb.cubicTo: 6,
    PathVerb.close: 0,
  };

  /// Makes the appropriate calls using [verbs] and [points] to replay this path
  /// on [proxy].
  ///
  /// Calls [PathProxy.reset] first if [reset] is true.
  void replay(PathProxy proxy, {bool reset = true}) {
    if (reset) {
      proxy.reset();
    }
    int index = 0;
    for (final PathVerb verb in verbs.toList()) {
      switch (verb) {
        case PathVerb.moveTo:
          proxy.moveTo(points[index++], points[index++]);
        case PathVerb.lineTo:
          proxy.lineTo(points[index++], points[index++]);
        case PathVerb._quadTo:
          assert(false);
        case PathVerb._conicTo:
          assert(false);
        case PathVerb.cubicTo:
          proxy.cubicTo(
            points[index++],
            points[index++],
            points[index++],
            points[index++],
            points[index++],
            points[index++],
          );
        case PathVerb.close:
          proxy.close();
      }
    }
    assert(index == points.length);
  }

  /// The list of path verbs in this path.
  ///
  /// This may not match the verbs supplied by calls to [moveTo], [lineTo],
  /// [cubicTo], and [close] after [applyOp] is invoked.
  ///
  /// This list determines the meaning of the [points] array.
  Iterable<PathVerb> get verbs {
    _updatePathData();
    final int count = _pathData!.ref.verb_count;
    return List<PathVerb>.generate(count, (int index) {
      return PathVerb.values[_pathData!.ref.verbs[index]];
    }, growable: false);
  }

  /// The list of points to use with [verbs].
  ///
  /// Each verb uses a specific number of points, specified by the
  /// [pointsPerVerb] map.
  Float32List get points {
    _updatePathData();
    return _pathData!.ref.points.asTypedList(_pathData!.ref.point_count);
  }

  void _updatePathData() {
    assert(_path != null);
    _pathData ??= _dataFn(_path!);
  }

  void _resetPathData() {
    if (_pathData != null) {
      _destroyDataFn(_pathData!);
    }
    _pathData = null;
  }

  @override
  void moveTo(double x, double y) {
    assert(_path != null);
    _resetPathData();
    _moveToFn(_path!, x, y);
  }

  @override
  void lineTo(double x, double y) {
    assert(_path != null);
    _resetPathData();
    _lineToFn(_path!, x, y);
  }

  @override
  void cubicTo(double x1, double y1, double x2, double y2, double x3, double y3) {
    assert(_path != null);
    _resetPathData();
    _cubicToFn(_path!, x1, y1, x2, y2, x3, y3);
  }

  @override
  void close() {
    assert(_path != null);
    _resetPathData();
    _closeFn(_path!, true);
  }

  @override
  void reset() {
    assert(_path != null);
    _resetPathData();
    _resetFn(_path!);
  }

  /// Releases native resources.
  ///
  /// After calling dispose, this class must not be used again.
  void dispose() {
    assert(_path != null);
    _resetPathData();
    _destroyFn(_path!);
    _path = null;
  }

  /// Applies the operation described by [op] to this path using [other].
  Path applyOp(Path other, PathOp op) {
    assert(_path != null);
    assert(other._path != null);
    final Path result = Path.from(this);
    _opFn(result._path!, other._path!, op.index);
    return result;
  }
}

// TODO(dnfield): Figure out where to put this.
// https://github.com/flutter/flutter/issues/99563
final ffi.DynamicLibrary _dylib = () {
  if (Platform.isWindows) {
    return ffi.DynamicLibrary.open('path_ops.dll');
  } else if (Platform.isIOS || Platform.isMacOS) {
    return ffi.DynamicLibrary.open('libpath_ops.dylib');
  } else if (Platform.isAndroid || Platform.isLinux) {
    return ffi.DynamicLibrary.open('libpath_ops.so');
  }
  throw UnsupportedError('Unknown platform: ${Platform.operatingSystem}');
}();

final class _SkPath extends ffi.Opaque {}

final class _PathData extends ffi.Struct {
  external ffi.Pointer<ffi.Uint8> verbs;

  @ffi.Size()
  external int verb_count;

  external ffi.Pointer<ffi.Float> points;

  @ffi.Size()
  external int point_count;
}

typedef _CreatePathType = ffi.Pointer<_SkPath> Function(int);
typedef _create_path_type = ffi.Pointer<_SkPath> Function(ffi.Int);

final _CreatePathType _createPathFn = _dylib.lookupFunction<_create_path_type, _CreatePathType>(
  'CreatePath',
);

typedef _MoveToType = void Function(ffi.Pointer<_SkPath>, double, double);
typedef _move_to_type = ffi.Void Function(ffi.Pointer<_SkPath>, ffi.Float, ffi.Float);

final _MoveToType _moveToFn = _dylib.lookupFunction<_move_to_type, _MoveToType>('MoveTo');

typedef _LineToType = void Function(ffi.Pointer<_SkPath>, double, double);
typedef _line_to_type = ffi.Void Function(ffi.Pointer<_SkPath>, ffi.Float, ffi.Float);

final _LineToType _lineToFn = _dylib.lookupFunction<_line_to_type, _LineToType>('LineTo');

typedef _CubicToType =
    void Function(ffi.Pointer<_SkPath>, double, double, double, double, double, double);
typedef _cubic_to_type =
    ffi.Void Function(
      ffi.Pointer<_SkPath>,
      ffi.Float,
      ffi.Float,
      ffi.Float,
      ffi.Float,
      ffi.Float,
      ffi.Float,
    );

final _CubicToType _cubicToFn = _dylib.lookupFunction<_cubic_to_type, _CubicToType>('CubicTo');

typedef _CloseType = void Function(ffi.Pointer<_SkPath>, bool);
typedef _close_type = ffi.Void Function(ffi.Pointer<_SkPath>, ffi.Bool);

final _CloseType _closeFn = _dylib.lookupFunction<_close_type, _CloseType>('Close');

typedef _ResetType = void Function(ffi.Pointer<_SkPath>);
typedef _reset_type = ffi.Void Function(ffi.Pointer<_SkPath>);

final _ResetType _resetFn = _dylib.lookupFunction<_reset_type, _ResetType>('Reset');

typedef _DestroyType = void Function(ffi.Pointer<_SkPath>);
typedef _destroy_type = ffi.Void Function(ffi.Pointer<_SkPath>);

final _DestroyType _destroyFn = _dylib.lookupFunction<_destroy_type, _DestroyType>('DestroyPath');

typedef _OpType = void Function(ffi.Pointer<_SkPath>, ffi.Pointer<_SkPath>, int);
typedef _op_type = ffi.Void Function(ffi.Pointer<_SkPath>, ffi.Pointer<_SkPath>, ffi.Int);

final _OpType _opFn = _dylib.lookupFunction<_op_type, _OpType>('Op');

typedef _PathDataType = ffi.Pointer<_PathData> Function(ffi.Pointer<_SkPath>);
typedef _path_data_type = ffi.Pointer<_PathData> Function(ffi.Pointer<_SkPath>);

final _PathDataType _dataFn = _dylib.lookupFunction<_path_data_type, _PathDataType>('Data');

typedef _DestroyDataType = void Function(ffi.Pointer<_PathData>);
typedef _destroy_data_type = ffi.Void Function(ffi.Pointer<_PathData>);

final _DestroyDataType _destroyDataFn = _dylib.lookupFunction<_destroy_data_type, _DestroyDataType>(
  'DestroyData',
);

typedef _GetFillTypeType = int Function(ffi.Pointer<_SkPath>);
typedef _get_fill_type_type = ffi.Int32 Function(ffi.Pointer<_SkPath>);

final _GetFillTypeType _getFillTypeFn = _dylib
    .lookupFunction<_get_fill_type_type, _GetFillTypeType>('GetFillType');
