// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/rendering.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:vector_math/vector_math_64.dart';

import 'events.dart';

export 'dart:ui' show Offset;

export 'package:vector_math/vector_math_64.dart' show Matrix4;

export 'events.dart' show PointerEvent;

/// An object that can hit-test pointers.
abstract interface class HitTestable {
  /// Deprecated. Use [hitTestInView] instead.
  @Deprecated(
    'Use hitTestInView and specify the view to hit test. '
    'This feature was deprecated after v3.11.0-20.0.pre.',
  )
  void hitTest(HitTestResult result, Offset position);

  /// Fills the provided [HitTestResult] with [HitTestEntry]s for objects that
  /// are hit at the given `position` in the view identified by `viewId`.
  void hitTestInView(HitTestResult result, Offset position, int viewId);
}

/// An object that can dispatch events.
abstract interface class HitTestDispatcher {
  /// Override this method to dispatch events.
  void dispatchEvent(PointerEvent event, HitTestResult result);
}

/// An object that can handle events.
abstract interface class HitTestTarget {
  /// Override this method to receive events.
  void handleEvent(PointerEvent event, HitTestEntry<HitTestTarget> entry);
}

/// Data collected during a hit test about a specific [HitTestTarget].
///
/// Subclass this object to pass additional information from the hit test phase
/// to the event propagation phase.
@optionalTypeArgs
class HitTestEntry<T extends HitTestTarget> {
  /// Creates a hit test entry.
  HitTestEntry(this.target);

  /// The [HitTestTarget] encountered during the hit test.
  final T target;

  @override
  String toString() => '${describeIdentity(this)}($target)';

  /// Returns a matrix describing how [PointerEvent]s delivered to this
  /// [HitTestEntry] should be transformed from the global coordinate space of
  /// the screen to the local coordinate space of [target].
  ///
  /// See also:
  ///
  ///  * [BoxHitTestResult.addWithPaintTransform], which is used during hit testing
  ///    to build up this transform.
  Matrix4? get transform => _transform;
  Matrix4? _transform;
}

// A type of data that can be applied to a matrix by left-multiplication.
@immutable
abstract class _TransformPart {
  const _TransformPart();

  // Apply this transform part to `rhs` from the left.
  //
  // This should work as if this transform part is first converted to a matrix
  // and then left-multiplied to `rhs`.
  //
  // For example, if this transform part is a vector `v1`, whose corresponding
  // matrix is `m1 = Matrix4.translation(v1)`, then the result of
  // `_VectorTransformPart(v1).multiply(rhs)` should equal to `m1 * rhs`.
  Matrix4 multiply(Matrix4 rhs);
}

class _MatrixTransformPart extends _TransformPart {
  const _MatrixTransformPart(this.matrix);

  final Matrix4 matrix;

  @override
  Matrix4 multiply(Matrix4 rhs) {
    return matrix.multiplied(rhs);
  }
}

class _OffsetTransformPart extends _TransformPart {
  const _OffsetTransformPart(this.offset);

  final Offset offset;

  @override
  Matrix4 multiply(Matrix4 rhs) {
    return rhs.clone()..leftTranslate(offset.dx, offset.dy);
  }
}

/// The result of performing a hit test.
class HitTestResult {
  /// Creates an empty hit test result.
  HitTestResult()
    : _path = <HitTestEntry>[],
      _transforms = <Matrix4>[Matrix4.identity()],
      _localTransforms = <_TransformPart>[];

  /// Wraps `result` (usually a subtype of [HitTestResult]) to create a
  /// generic [HitTestResult].
  ///
  /// The [HitTestEntry]s added to the returned [HitTestResult] are also
  /// added to the wrapped `result` (both share the same underlying data
  /// structure to store [HitTestEntry]s).
  HitTestResult.wrap(HitTestResult result)
    : _path = result._path,
      _transforms = result._transforms,
      _localTransforms = result._localTransforms;

  /// An unmodifiable list of [HitTestEntry] objects recorded during the hit test.
  ///
  /// The first entry in the path is the most specific, typically the one at
  /// the leaf of tree being hit tested. Event propagation starts with the most
  /// specific (i.e., first) entry and proceeds in order through the path.
  Iterable<HitTestEntry> get path => _path;
  final List<HitTestEntry> _path;

  // A stack of transform parts.
  //
  // The transform part stack leading from global to the current object is stored
  // in 2 parts:
  //
  //  * `_transforms` are globalized matrices, meaning they have been multiplied
  //    by the ancestors and are thus relative to the global coordinate space.
  //  * `_localTransforms` are local transform parts, which are relative to the
  //    parent's coordinate space.
  //
  // When new transform parts are added they're appended to `_localTransforms`,
  // and are converted to global ones and moved to `_transforms` only when used.
  final List<Matrix4> _transforms;
  final List<_TransformPart> _localTransforms;

  // Globalize all transform parts in `_localTransforms` and move them to
  // _transforms.
  void _globalizeTransforms() {
    if (_localTransforms.isEmpty) {
      return;
    }
    Matrix4 last = _transforms.last;
    for (final _TransformPart part in _localTransforms) {
      last = part.multiply(last);
      _transforms.add(last);
    }
    _localTransforms.clear();
  }

  Matrix4 get _lastTransform {
    _globalizeTransforms();
    assert(_localTransforms.isEmpty);
    return _transforms.last;
  }

  /// Add a [HitTestEntry] to the path.
  ///
  /// The new entry is added at the end of the path, which means entries should
  /// be added in order from most specific to least specific, typically during an
  /// upward walk of the tree being hit tested.
  void add(HitTestEntry entry) {
    assert(entry._transform == null);
    entry._transform = _lastTransform;
    _path.add(entry);
  }

  /// Pushes a new transform matrix that is to be applied to all future
  /// [HitTestEntry]s added via [add] until it is removed via [popTransform].
  ///
  /// This method is only to be used by subclasses, which must provide
  /// coordinate space specific public wrappers around this function for their
  /// users (see [BoxHitTestResult.addWithPaintTransform] for such an example).
  ///
  /// The provided `transform` matrix should describe how to transform
  /// [PointerEvent]s from the coordinate space of the method caller to the
  /// coordinate space of its children. In most cases `transform` is derived
  /// from running the inverted result of [RenderObject.applyPaintTransform]
  /// through [PointerEvent.removePerspectiveTransform] to remove
  /// the perspective component.
  ///
  /// If the provided `transform` is a translation matrix, it is much faster
  /// to use [pushOffset] with the translation offset instead.
  ///
  /// [HitTestable]s need to call this method indirectly through a convenience
  /// method defined on a subclass before hit testing a child that does not
  /// have the same origin as the parent. After hit testing the child,
  /// [popTransform] has to be called to remove the child-specific `transform`.
  ///
  /// See also:
  ///
  ///  * [pushOffset], which is similar to [pushTransform] but is limited to
  ///    translations, and is faster in such cases.
  ///  * [BoxHitTestResult.addWithPaintTransform], which is a public wrapper
  ///    around this function for hit testing on [RenderBox]s.
  @protected
  void pushTransform(Matrix4 transform) {
    assert(
      _debugVectorMoreOrLessEquals(transform.getRow(2), Vector4(0, 0, 1, 0)) &&
          _debugVectorMoreOrLessEquals(transform.getColumn(2), Vector4(0, 0, 1, 0)),
      'The third row and third column of a transform matrix for pointer '
      'events must be Vector4(0, 0, 1, 0) to ensure that a transformed '
      'point is directly under the pointing device. Did you forget to run the paint '
      'matrix through PointerEvent.removePerspectiveTransform? '
      'The provided matrix is:\n$transform',
    );
    _localTransforms.add(_MatrixTransformPart(transform));
  }

  /// Pushes a new translation offset that is to be applied to all future
  /// [HitTestEntry]s added via [add] until it is removed via [popTransform].
  ///
  /// This method is only to be used by subclasses, which must provide
  /// coordinate space specific public wrappers around this function for their
  /// users (see [BoxHitTestResult.addWithPaintOffset] for such an example).
  ///
  /// The provided `offset` should describe how to transform [PointerEvent]s from
  /// the coordinate space of the method caller to the coordinate space of its
  /// children. Usually `offset` is the inverse of the offset of the child
  /// relative to the parent.
  ///
  /// [HitTestable]s need to call this method indirectly through a convenience
  /// method defined on a subclass before hit testing a child that does not
  /// have the same origin as the parent. After hit testing the child,
  /// [popTransform] has to be called to remove the child-specific `transform`.
  ///
  /// See also:
  ///
  ///  * [pushTransform], which is similar to [pushOffset] but allows general
  ///    transform besides translation.
  ///  * [BoxHitTestResult.addWithPaintOffset], which is a public wrapper
  ///    around this function for hit testing on [RenderBox]s.
  ///  * [SliverHitTestResult.addWithAxisOffset], which is a public wrapper
  ///    around this function for hit testing on [RenderSliver]s.
  @protected
  void pushOffset(Offset offset) {
    _localTransforms.add(_OffsetTransformPart(offset));
  }

  /// Removes the last transform added via [pushTransform] or [pushOffset].
  ///
  /// This method is only to be used by subclasses, which must provide
  /// coordinate space specific public wrappers around this function for their
  /// users (see [BoxHitTestResult.addWithPaintTransform] for such an example).
  ///
  /// This method must be called after hit testing is done on a child that
  /// required a call to [pushTransform] or [pushOffset].
  ///
  /// See also:
  ///
  ///  * [pushTransform] and [pushOffset], which describes the use case of this
  ///    function pair in more details.
  @protected
  void popTransform() {
    if (_localTransforms.isNotEmpty) {
      _localTransforms.removeLast();
    } else {
      _transforms.removeLast();
    }
    assert(_transforms.isNotEmpty);
  }

  bool _debugVectorMoreOrLessEquals(
    Vector4 a,
    Vector4 b, {
    double epsilon = precisionErrorTolerance,
  }) {
    bool result = true;
    assert(() {
      final Vector4 difference = a - b;
      result = difference.storage.every((double component) => component.abs() < epsilon);
      return true;
    }());
    return result;
  }

  @override
  String toString() => 'HitTestResult(${_path.isEmpty ? "<empty path>" : _path.join(", ")})';
}
