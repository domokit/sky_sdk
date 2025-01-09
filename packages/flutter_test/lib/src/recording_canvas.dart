// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'mock_canvas.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';

/// An [Invocation] and the [stack] trace that led to it.
///
/// Used by [TestRecordingCanvas] to trace canvas calls.
class RecordedInvocation {
  /// Create a record for an invocation list.
  const RecordedInvocation(this.invocation, {required this.stack});

  /// The method that was called and its arguments.
  ///
  /// The arguments preserve identity, but not value. Thus, if two invocations
  /// were made with the same [Paint] object, but with that object configured
  /// differently each time, then they will both have the same object as their
  /// argument, and inspecting that object will return the object's current
  /// values (mostly likely those passed to the second call).
  final Invocation invocation;

  /// The stack trace at the time of the method call.
  final StackTrace stack;

  @override
  String toString() => _describeInvocation(invocation);

  /// Converts [stack] to a string using the [FlutterError.defaultStackFilter]
  /// logic.
  String stackToString({String indent = ''}) {
    return indent +
        FlutterError.defaultStackFilter(stack.toString().trimRight().split('\n')).join('\n$indent');
  }
}

// Examples can assume:
// late WidgetTester tester;

/// A [Canvas] for tests that records its method calls.
///
/// This class can be used in conjunction with [TestRecordingPaintingContext]
/// to record the [Canvas] method calls made by a renderer. For example:
///
/// ```dart
/// RenderBox box = tester.renderObject(find.text('ABC'));
/// TestRecordingCanvas canvas = TestRecordingCanvas();
/// TestRecordingPaintingContext context = TestRecordingPaintingContext(canvas);
/// box.paint(context, Offset.zero);
/// // Now test the expected canvas.invocations.
/// ```
///
/// In some cases it may be useful to define a subclass that overrides the
/// [Canvas] methods the test is checking and squirrels away the parameters
/// that the test requires.
///
/// For simple tests, consider using the [paints] matcher, which overlays a
/// pattern matching API over [TestRecordingCanvas].
class TestRecordingCanvas implements Canvas {
  /// All of the method calls on this canvas.
  final List<RecordedInvocation> invocations = <RecordedInvocation>[];

  int _saveCount = 0;

  @override
  int getSaveCount() => _saveCount;

  @override
  void save() {
    _saveCount += 1;
    invocations.add(RecordedInvocation(_MethodCall(#save), stack: StackTrace.current));
  }

  @override
  void saveLayer(Rect? bounds, Paint paint) {
    _saveCount += 1;
    invocations.add(
      RecordedInvocation(
        _MethodCall(#saveLayer, <dynamic>[bounds, paint]),
        stack: StackTrace.current,
      ),
    );
  }

  @override
  void restore() {
    _saveCount -= 1;
    assert(_saveCount >= 0);
    invocations.add(RecordedInvocation(_MethodCall(#restore), stack: StackTrace.current));
  }

  @override
  void noSuchMethod(Invocation invocation) {
    invocations.add(RecordedInvocation(invocation, stack: StackTrace.current));
  }
}

/// A [PaintingContext] for tests that use [TestRecordingCanvas].
class TestRecordingPaintingContext extends ClipContext implements PaintingContext {
  /// Creates a [PaintingContext] for tests that use [TestRecordingCanvas].
  TestRecordingPaintingContext(this.canvas);

  final List<OpacityLayer> _createdLayers = <OpacityLayer>[];

  @override
  final Canvas canvas;

  @override
  void paintChild(RenderObject child, Offset offset) {
    child.paint(this, offset);
  }

  @override
  ClipRectLayer? pushClipRect(
    bool needsCompositing,
    Offset offset,
    Rect clipRect,
    PaintingContextCallback painter, {
    Clip clipBehavior = Clip.hardEdge,
    ClipRectLayer? oldLayer,
  }) {
    clipRectAndPaint(
      clipRect.shift(offset),
      clipBehavior,
      clipRect.shift(offset),
      () => painter(this, offset),
    );
    return null;
  }

  @override
  ClipRRectLayer? pushClipRRect(
    bool needsCompositing,
    Offset offset,
    Rect bounds,
    RRect clipRRect,
    PaintingContextCallback painter, {
    Clip clipBehavior = Clip.antiAlias,
    ClipRRectLayer? oldLayer,
  }) {
    clipRRectAndPaint(
      clipRRect.shift(offset),
      clipBehavior,
      bounds.shift(offset),
      () => painter(this, offset),
    );
    return null;
  }

  @override
  ClipPathLayer? pushClipPath(
    bool needsCompositing,
    Offset offset,
    Rect bounds,
    Path clipPath,
    PaintingContextCallback painter, {
    Clip clipBehavior = Clip.antiAlias,
    ClipPathLayer? oldLayer,
  }) {
    clipPathAndPaint(
      clipPath.shift(offset),
      clipBehavior,
      bounds.shift(offset),
      () => painter(this, offset),
    );
    return null;
  }

  @override
  TransformLayer? pushTransform(
    bool needsCompositing,
    Offset offset,
    Matrix4 transform,
    PaintingContextCallback painter, {
    TransformLayer? oldLayer,
  }) {
    canvas.save();
    canvas.transform(transform.storage);
    painter(this, offset);
    canvas.restore();
    return null;
  }

  @override
  OpacityLayer pushOpacity(
    Offset offset,
    int alpha,
    PaintingContextCallback painter, {
    OpacityLayer? oldLayer,
  }) {
    canvas.saveLayer(null, Paint()); // TODO(ianh): Expose the alpha somewhere.
    painter(this, offset);
    canvas.restore();
    final OpacityLayer layer = OpacityLayer();
    _createdLayers.add(layer);
    return layer;
  }

  /// Releases allocated resources.
  @mustCallSuper
  void dispose() {
    for (final OpacityLayer layer in _createdLayers) {
      layer.dispose();
    }
    _createdLayers.clear();
  }

  @override
  void pushLayer(
    Layer childLayer,
    PaintingContextCallback painter,
    Offset offset, {
    Rect? childPaintBounds,
  }) {
    painter(this, offset);
  }

  @override
  VoidCallback addCompositionCallback(CompositionCallback callback) => () {};

  @override
  void noSuchMethod(Invocation invocation) {}
}

class _MethodCall implements Invocation {
  _MethodCall(this._name, [this._arguments = const <dynamic>[]]);
  final Symbol _name;
  final List<dynamic> _arguments;
  @override
  bool get isAccessor => false;
  @override
  bool get isGetter => false;
  @override
  bool get isMethod => true;
  @override
  bool get isSetter => false;
  @override
  Symbol get memberName => _name;
  @override
  Map<Symbol, dynamic> get namedArguments => <Symbol, dynamic>{};
  @override
  List<dynamic> get positionalArguments => _arguments;
  @override
  List<Type> get typeArguments => const <Type>[];
}

String _valueName(Object? value) {
  if (value is double) {
    return value.toStringAsFixed(1);
  }
  return value.toString();
}

// Workaround for https://github.com/dart-lang/sdk/issues/28372
String _symbolName(Symbol symbol) {
  // WARNING: Assumes a fixed format for Symbol.toString which is *not*
  // guaranteed anywhere.
  final String s = '$symbol';
  return s.substring(8, s.length - 2);
}

// Workaround for https://github.com/dart-lang/sdk/issues/28373
String _describeInvocation(Invocation call) {
  final StringBuffer buffer = StringBuffer();
  buffer.write(_symbolName(call.memberName));
  if (call.isSetter) {
    buffer.write(call.positionalArguments[0].toString());
  } else if (call.isMethod) {
    buffer.write('(');
    buffer.writeAll(call.positionalArguments.map<String>(_valueName), ', ');
    String separator = call.positionalArguments.isEmpty ? '' : ', ';
    call.namedArguments.forEach((Symbol name, Object? value) {
      buffer.write(separator);
      buffer.write(_symbolName(name));
      buffer.write(': ');
      buffer.write(_valueName(value));
      separator = ', ';
    });
    buffer.write(')');
  }
  return buffer.toString();
}
