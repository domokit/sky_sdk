// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;
import 'dart:ui' show PointerChange;

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'mouse_tracker_test_utils.dart';

typedef MethodCallHandler = Future<dynamic> Function(MethodCall call);
typedef SimpleAnnotationFinder = Iterable<HitTestTarget> Function(Offset offset);

void main() {
  final TestMouseTrackerFlutterBinding binding = TestMouseTrackerFlutterBinding();
  MethodCallHandler? methodCallHandler;

  // Only one of `logCursors` and `cursorHandler` should be specified.
  void setUpMouseTracker({
    required SimpleAnnotationFinder annotationFinder,
    List<_CursorUpdateDetails>? logCursors,
    MethodCallHandler? cursorHandler,
  }) {
    assert(logCursors == null || cursorHandler == null);
    methodCallHandler =
        logCursors != null
            ? (MethodCall call) async {
              logCursors.add(_CursorUpdateDetails.wrap(call));
              return;
            }
            : cursorHandler;

    binding.setHitTest((BoxHitTestResult result, Offset position) {
      for (final HitTestTarget target in annotationFinder(position)) {
        result.addWithRawTransform(
          transform: Matrix4.identity(),
          position: position,
          hitTest: (BoxHitTestResult result, Offset position) {
            result.add(HitTestEntry(target));
            return true;
          },
        );
      }
      return true;
    });
  }

  void dispatchRemoveDevice([int device = 0]) {
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.remove, Offset.zero, device: device)],
      ),
    );
  }

  setUp(() {
    binding.postFrameCallbacks.clear();
    binding.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.mouseCursor,
      (MethodCall call) => methodCallHandler?.call(call),
    );
  });

  tearDown(() {
    binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.mouseCursor, null);
  });

  test('Should work on platforms that does not support mouse cursor', () async {
    const TestAnnotationTarget annotation = TestAnnotationTarget(
      cursor: SystemMouseCursors.grabbing,
    );

    setUpMouseTracker(
      annotationFinder: (Offset position) => <TestAnnotationTarget>[annotation],
      cursorHandler: (MethodCall call) async {
        return null;
      },
    );

    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );
    addTearDown(dispatchRemoveDevice);

    // Passes if no errors are thrown
  });

  test('pointer is added and removed out of any annotations', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    TestAnnotationTarget? annotation;
    setUpMouseTracker(
      annotationFinder:
          (Offset position) => <TestAnnotationTarget>[if (annotation != null) annotation],
      logCursors: logCursors,
    );

    // Pointer is added outside of the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.basic.kind),
    ]);
    logCursors.clear();

    // Pointer moves into the annotation
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing);
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(5.0, 0.0))],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.grabbing.kind),
    ]);
    logCursors.clear();

    // Pointer moves within the annotation
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing);
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(10.0, 0.0))],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[]);
    logCursors.clear();

    // Pointer moves out of the annotation
    annotation = null;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.hover, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.basic.kind),
    ]);
    logCursors.clear();

    // Pointer is removed outside of the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.remove, Offset.zero)]),
    );

    expect(logCursors, const <_CursorUpdateDetails>[]);
  });

  test('pointer is added and removed in an annotation', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    TestAnnotationTarget? annotation;
    setUpMouseTracker(
      annotationFinder:
          (Offset position) => <TestAnnotationTarget>[if (annotation != null) annotation],
      logCursors: logCursors,
    );

    // Pointer is added in the annotation.
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing);
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.grabbing.kind),
    ]);
    logCursors.clear();

    // Pointer moves out of the annotation
    annotation = null;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(5.0, 0.0))],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.basic.kind),
    ]);
    logCursors.clear();

    // Pointer moves around out of the annotation
    annotation = null;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(10.0, 0.0))],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[]);
    logCursors.clear();

    // Pointer moves back into the annotation
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing);
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.hover, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.grabbing.kind),
    ]);
    logCursors.clear();

    // Pointer is removed within the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.remove, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[]);
  });

  test('pointer change caused by new frames', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    TestAnnotationTarget? annotation;
    setUpMouseTracker(
      annotationFinder:
          (Offset position) => <TestAnnotationTarget>[if (annotation != null) annotation],
      logCursors: logCursors,
    );

    // Pointer is added outside of the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.basic.kind),
    ]);
    logCursors.clear();

    // Synthesize a new frame while changing annotation
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing);
    binding.scheduleMouseTrackerPostFrameCheck();
    binding.flushPostFrameCallbacks(Duration.zero);

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.grabbing.kind),
    ]);
    logCursors.clear();

    // Synthesize a new frame without changing annotation
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing);
    binding.scheduleMouseTrackerPostFrameCheck();

    expect(logCursors, <_CursorUpdateDetails>[]);
    logCursors.clear();

    // Pointer is removed outside of the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.remove, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[]);
  });

  test('The first annotation with non-deferring cursor is used', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    late List<TestAnnotationTarget> annotations;
    setUpMouseTracker(
      annotationFinder: (Offset position) sync* {
        yield* annotations;
      },
      logCursors: logCursors,
    );

    annotations = <TestAnnotationTarget>[
      const TestAnnotationTarget(),
      const TestAnnotationTarget(cursor: SystemMouseCursors.click),
      const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing),
    ];
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.click.kind),
    ]);
    logCursors.clear();

    // Remove
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.remove, const Offset(5.0, 0.0))],
      ),
    );
  });

  test('Annotations with deferring cursors are ignored', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    late List<TestAnnotationTarget> annotations;
    setUpMouseTracker(
      annotationFinder: (Offset position) sync* {
        yield* annotations;
      },
      logCursors: logCursors,
    );

    annotations = <TestAnnotationTarget>[
      const TestAnnotationTarget(),
      const TestAnnotationTarget(),
      const TestAnnotationTarget(cursor: SystemMouseCursors.grabbing),
    ];
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.grabbing.kind),
    ]);
    logCursors.clear();

    // Remove
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.remove, const Offset(5.0, 0.0))],
      ),
    );
  });

  test('Finding no annotation is equivalent to specifying default cursor', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    TestAnnotationTarget? annotation;
    setUpMouseTracker(
      annotationFinder:
          (Offset position) => <TestAnnotationTarget>[if (annotation != null) annotation],
      logCursors: logCursors,
    );

    // Pointer is added outside of the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.basic.kind),
    ]);
    logCursors.clear();

    // Pointer moved to an annotation specified with the default cursor
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.basic);
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(5.0, 0.0))],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[]);
    logCursors.clear();

    // Pointer moved to no annotations
    annotation = null;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.hover, Offset.zero)]),
    );

    expect(logCursors, <_CursorUpdateDetails>[]);
    logCursors.clear();

    // Remove
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.remove, Offset.zero)]),
    );
  });

  test('Removing a pointer resets it back to the default cursor', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    TestAnnotationTarget? annotation;
    setUpMouseTracker(
      annotationFinder:
          (Offset position) => <TestAnnotationTarget>[if (annotation != null) annotation],
      logCursors: logCursors,
    );

    // Pointer is added to the annotation, then removed
    annotation = const TestAnnotationTarget(cursor: SystemMouseCursors.click);
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.add, Offset.zero),
          _pointerData(PointerChange.hover, const Offset(5.0, 0.0)),
          _pointerData(PointerChange.remove, const Offset(5.0, 0.0)),
        ],
      ),
    );

    logCursors.clear();

    // Pointer is added out of the annotation
    annotation = null;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.add, Offset.zero)]),
    );
    addTearDown(dispatchRemoveDevice);

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 0, kind: SystemMouseCursors.basic.kind),
    ]);
    logCursors.clear();
  });

  test('Pointing devices display cursors separately', () {
    final List<_CursorUpdateDetails> logCursors = <_CursorUpdateDetails>[];
    setUpMouseTracker(
      annotationFinder: (Offset position) sync* {
        if (position.dx > 200) {
          yield const TestAnnotationTarget(cursor: SystemMouseCursors.forbidden);
        } else if (position.dx > 100) {
          yield const TestAnnotationTarget(cursor: SystemMouseCursors.click);
        } else {}
      },
      logCursors: logCursors,
    );

    // Pointers are added outside of the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.add, Offset.zero, device: 1),
          _pointerData(PointerChange.add, Offset.zero, device: 2),
        ],
      ),
    );
    addTearDown(() => dispatchRemoveDevice(1));
    addTearDown(() => dispatchRemoveDevice(2));

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 1, kind: SystemMouseCursors.basic.kind),
      _CursorUpdateDetails.activateSystemCursor(device: 2, kind: SystemMouseCursors.basic.kind),
    ]);
    logCursors.clear();

    // Pointer 1 moved to cursor "click"
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.hover, const Offset(101.0, 0.0), device: 1),
        ],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 1, kind: SystemMouseCursors.click.kind),
    ]);
    logCursors.clear();

    // Pointer 2 moved to cursor "click"
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.hover, const Offset(102.0, 0.0), device: 2),
        ],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 2, kind: SystemMouseCursors.click.kind),
    ]);
    logCursors.clear();

    // Pointer 2 moved to cursor "forbidden"
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.hover, const Offset(202.0, 0.0), device: 2),
        ],
      ),
    );

    expect(logCursors, <_CursorUpdateDetails>[
      _CursorUpdateDetails.activateSystemCursor(device: 2, kind: SystemMouseCursors.forbidden.kind),
    ]);
    logCursors.clear();
  });
}

ui.PointerData _pointerData(
  PointerChange change,
  Offset logicalPosition, {
  int device = 0,
  PointerDeviceKind kind = PointerDeviceKind.mouse,
}) {
  final double devicePixelRatio =
      RendererBinding.instance.platformDispatcher.implicitView!.devicePixelRatio;
  return ui.PointerData(
    change: change,
    physicalX: logicalPosition.dx * devicePixelRatio,
    physicalY: logicalPosition.dy * devicePixelRatio,
    kind: kind,
    device: device,
  );
}

class _CursorUpdateDetails extends MethodCall {
  const _CursorUpdateDetails(super.method, Map<String, dynamic> super.arguments);

  _CursorUpdateDetails.wrap(MethodCall call)
    : super(call.method, Map<String, dynamic>.from(call.arguments as Map<dynamic, dynamic>));

  _CursorUpdateDetails.activateSystemCursor({required int device, required String kind})
    : this('activateSystemCursor', <String, dynamic>{'device': device, 'kind': kind});
  @override
  Map<String, dynamic> get arguments => super.arguments as Map<String, dynamic>;

  @override
  bool operator ==(Object other) {
    if (identical(other, this)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _CursorUpdateDetails &&
        other.method == method &&
        other.arguments.length == arguments.length &&
        other.arguments.entries.every(
          (MapEntry<String, dynamic> entry) =>
              arguments.containsKey(entry.key) && arguments[entry.key] == entry.value,
        );
  }

  @override
  int get hashCode => Object.hash(method, arguments);

  @override
  String toString() {
    return '_CursorUpdateDetails(method: $method, arguments: $arguments)';
  }
}
