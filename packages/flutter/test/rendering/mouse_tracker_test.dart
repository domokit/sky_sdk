// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;
import 'dart:ui' show PointerChange;

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'mouse_tracker_test_utils.dart';

MouseTracker get _mouseTracker => RendererBinding.instance.mouseTracker;

typedef SimpleAnnotationFinder = Iterable<TestAnnotationEntry> Function(Offset offset);

void main() {
  final TestMouseTrackerFlutterBinding binding = TestMouseTrackerFlutterBinding();
  void setUpMouseAnnotationFinder(SimpleAnnotationFinder annotationFinder) {
    binding.setHitTest((BoxHitTestResult result, Offset position) {
      for (final TestAnnotationEntry entry in annotationFinder(position)) {
        result.addWithRawTransform(
          transform: entry.transform,
          position: position,
          hitTest: (BoxHitTestResult result, Offset position) {
            result.add(entry);
            return true;
          },
        );
      }
      return true;
    });
  }

  // Set up a trivial test environment that includes one annotation.
  // This annotation records the enter, hover, and exit events it receives to
  // `logEvents`.
  // This annotation also contains a cursor with a value of `testCursor`.
  // The mouse tracker records the cursor requests it receives to `logCursors`.
  TestAnnotationTarget setUpWithOneAnnotation({required List<PointerEvent> logEvents}) {
    final TestAnnotationTarget oneAnnotation = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) {
        logEvents.add(event);
      },
      onHover: (PointerHoverEvent event) {
        logEvents.add(event);
      },
      onExit: (PointerExitEvent event) {
        logEvents.add(event);
      },
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      yield TestAnnotationEntry(oneAnnotation);
    });
    return oneAnnotation;
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
  });

  final Matrix4 translate10by20 = Matrix4.translationValues(10, 20, 0);

  for (final ui.PointerDeviceKind pointerDeviceKind in <ui.PointerDeviceKind>[
    ui.PointerDeviceKind.mouse,
    ui.PointerDeviceKind.stylus,
  ]) {
    test(
      'should detect enter, hover, and exit from Added, Hover, and Removed events for stylus',
      () {
        final List<PointerEvent> events = <PointerEvent>[];
        setUpWithOneAnnotation(logEvents: events);

        final List<bool> listenerLogs = <bool>[];
        _mouseTracker.addListener(() {
          listenerLogs.add(_mouseTracker.mouseIsConnected);
        });

        expect(_mouseTracker.mouseIsConnected, isFalse);

        // Pointer enters the annotation.
        RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
          ui.PointerDataPacket(
            data: <ui.PointerData>[
              _pointerData(PointerChange.add, Offset.zero, kind: pointerDeviceKind),
            ],
          ),
        );
        addTearDown(() => dispatchRemoveDevice());

        expect(
          events,
          _equalToEventsOnCriticalFields(<BaseEventMatcher>[
            EventMatcher<PointerEnterEvent>(const PointerEnterEvent()),
          ]),
        );
        expect(listenerLogs, <bool>[true]);
        events.clear();
        listenerLogs.clear();

        // Pointer hovers the annotation.
        RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
          ui.PointerDataPacket(
            data: <ui.PointerData>[
              _pointerData(PointerChange.hover, const Offset(1.0, 101.0), kind: pointerDeviceKind),
            ],
          ),
        );
        expect(
          events,
          _equalToEventsOnCriticalFields(<BaseEventMatcher>[
            EventMatcher<PointerHoverEvent>(const PointerHoverEvent(position: Offset(1.0, 101.0))),
          ]),
        );
        expect(_mouseTracker.mouseIsConnected, isTrue);
        expect(listenerLogs, isEmpty);
        events.clear();

        // Pointer is removed while on the annotation.
        RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
          ui.PointerDataPacket(
            data: <ui.PointerData>[
              _pointerData(PointerChange.remove, const Offset(1.0, 101.0), kind: pointerDeviceKind),
            ],
          ),
        );
        expect(
          events,
          _equalToEventsOnCriticalFields(<BaseEventMatcher>[
            EventMatcher<PointerExitEvent>(const PointerExitEvent(position: Offset(1.0, 101.0))),
          ]),
        );
        expect(listenerLogs, <bool>[false]);
        events.clear();
        listenerLogs.clear();

        // Pointer is added on the annotation.
        RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
          ui.PointerDataPacket(
            data: <ui.PointerData>[
              _pointerData(PointerChange.add, const Offset(0.0, 301.0), kind: pointerDeviceKind),
            ],
          ),
        );
        expect(
          events,
          _equalToEventsOnCriticalFields(<BaseEventMatcher>[
            EventMatcher<PointerEnterEvent>(const PointerEnterEvent(position: Offset(0.0, 301.0))),
          ]),
        );
        expect(listenerLogs, <bool>[true]);
        events.clear();
        listenerLogs.clear();
      },
    );
  }

  // Regression test for https://github.com/flutter/flutter/issues/90838
  test('should not crash if the first event is a Removed event', () {
    final List<PointerEvent> events = <PointerEvent>[];
    setUpWithOneAnnotation(logEvents: events);
    binding.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(data: <ui.PointerData>[_pointerData(PointerChange.remove, Offset.zero)]),
    );
    events.clear();
  });

  test('should correctly handle multiple devices', () {
    final List<PointerEvent> events = <PointerEvent>[];
    setUpWithOneAnnotation(logEvents: events);

    expect(_mouseTracker.mouseIsConnected, isFalse);

    // The first mouse is added on the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.add, Offset.zero),
          _pointerData(PointerChange.hover, const Offset(0.0, 1.0)),
        ],
      ),
    );
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerEnterEvent>(const PointerEnterEvent()),
        EventMatcher<PointerHoverEvent>(const PointerHoverEvent(position: Offset(0.0, 1.0))),
      ]),
    );
    expect(_mouseTracker.mouseIsConnected, isTrue);
    events.clear();

    // The second mouse is added on the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.add, const Offset(0.0, 401.0), device: 1),
          _pointerData(PointerChange.hover, const Offset(1.0, 401.0), device: 1),
        ],
      ),
    );
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerEnterEvent>(
          const PointerEnterEvent(position: Offset(0.0, 401.0), device: 1),
        ),
        EventMatcher<PointerHoverEvent>(
          const PointerHoverEvent(position: Offset(1.0, 401.0), device: 1),
        ),
      ]),
    );
    expect(_mouseTracker.mouseIsConnected, isTrue);
    events.clear();

    // The first mouse moves on the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(0.0, 101.0))],
      ),
    );
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerHoverEvent>(const PointerHoverEvent(position: Offset(0.0, 101.0))),
      ]),
    );
    expect(_mouseTracker.mouseIsConnected, isTrue);
    events.clear();

    // The second mouse moves on the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.hover, const Offset(1.0, 501.0), device: 1),
        ],
      ),
    );
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerHoverEvent>(
          const PointerHoverEvent(position: Offset(1.0, 501.0), device: 1),
        ),
      ]),
    );
    expect(_mouseTracker.mouseIsConnected, isTrue);
    events.clear();

    // The first mouse is removed while on the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.remove, const Offset(0.0, 101.0))],
      ),
    );
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerExitEvent>(const PointerExitEvent(position: Offset(0.0, 101.0))),
      ]),
    );
    expect(_mouseTracker.mouseIsConnected, isTrue);
    events.clear();

    // The second mouse still moves on the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.hover, const Offset(1.0, 601.0), device: 1),
        ],
      ),
    );
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerHoverEvent>(
          const PointerHoverEvent(position: Offset(1.0, 601.0), device: 1),
        ),
      ]),
    );
    expect(_mouseTracker.mouseIsConnected, isTrue);
    events.clear();

    // The second mouse is removed while on the annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.remove, const Offset(1.0, 601.0), device: 1),
        ],
      ),
    );
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerExitEvent>(
          const PointerExitEvent(position: Offset(1.0, 601.0), device: 1),
        ),
      ]),
    );
    expect(_mouseTracker.mouseIsConnected, isFalse);
    events.clear();
  });

  test('should not handle non-hover events', () {
    final List<PointerEvent> events = <PointerEvent>[];
    setUpWithOneAnnotation(logEvents: events);

    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.add, const Offset(0.0, 101.0)),
          _pointerData(PointerChange.down, const Offset(0.0, 101.0)),
        ],
      ),
    );
    addTearDown(() => dispatchRemoveDevice());
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        // This Enter event is triggered by the [PointerAddedEvent] The
        // [PointerDownEvent] is ignored by [MouseTracker].
        EventMatcher<PointerEnterEvent>(const PointerEnterEvent(position: Offset(0.0, 101.0))),
      ]),
    );
    events.clear();

    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.move, const Offset(0.0, 201.0))],
      ),
    );
    expect(events, _equalToEventsOnCriticalFields(<BaseEventMatcher>[]));
    events.clear();

    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.up, const Offset(0.0, 301.0))],
      ),
    );
    expect(events, _equalToEventsOnCriticalFields(<BaseEventMatcher>[]));
    events.clear();
  });

  test('should correctly handle when the annotation appears or disappears on the pointer', () {
    late bool isInHitRegion;
    final List<Object> events = <PointerEvent>[];
    final TestAnnotationTarget annotation = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => events.add(event),
      onHover: (PointerHoverEvent event) => events.add(event),
      onExit: (PointerExitEvent event) => events.add(event),
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      if (isInHitRegion) {
        yield TestAnnotationEntry(annotation, Matrix4.translationValues(10, 20, 0));
      }
    });

    isInHitRegion = false;

    // Connect a mouse when there is no annotation.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.add, const Offset(0.0, 100.0))],
      ),
    );
    addTearDown(() => dispatchRemoveDevice());
    expect(events, _equalToEventsOnCriticalFields(<BaseEventMatcher>[]));
    expect(_mouseTracker.mouseIsConnected, isTrue);
    events.clear();

    // Adding an annotation should trigger Enter event.
    isInHitRegion = true;
    binding.scheduleMouseTrackerPostFrameCheck();
    expect(binding.postFrameCallbacks, hasLength(1));

    binding.flushPostFrameCallbacks(Duration.zero);
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerEnterEvent>(
          const PointerEnterEvent(position: Offset(0, 100)).transformed(translate10by20),
        ),
      ]),
    );
    events.clear();

    // Removing an annotation should trigger events.
    isInHitRegion = false;
    binding.scheduleMouseTrackerPostFrameCheck();
    expect(binding.postFrameCallbacks, hasLength(1));

    binding.flushPostFrameCallbacks(Duration.zero);
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerExitEvent>(
          const PointerExitEvent(position: Offset(0.0, 100.0)).transformed(translate10by20),
        ),
      ]),
    );
    expect(binding.postFrameCallbacks, hasLength(0));
  });

  test('should correctly handle when the annotation moves in or out of the pointer', () {
    late bool isInHitRegion;
    final List<Object> events = <PointerEvent>[];
    final TestAnnotationTarget annotation = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => events.add(event),
      onHover: (PointerHoverEvent event) => events.add(event),
      onExit: (PointerExitEvent event) => events.add(event),
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      if (isInHitRegion) {
        yield TestAnnotationEntry(annotation, Matrix4.translationValues(10, 20, 0));
      }
    });

    isInHitRegion = false;

    // Connect a mouse.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.add, const Offset(0.0, 100.0))],
      ),
    );
    addTearDown(() => dispatchRemoveDevice());
    events.clear();

    // During a frame, the annotation moves into the pointer.
    isInHitRegion = true;
    expect(binding.postFrameCallbacks, hasLength(0));
    binding.scheduleMouseTrackerPostFrameCheck();
    expect(binding.postFrameCallbacks, hasLength(1));

    binding.flushPostFrameCallbacks(Duration.zero);
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerEnterEvent>(
          const PointerEnterEvent(position: Offset(0.0, 100.0)).transformed(translate10by20),
        ),
      ]),
    );
    events.clear();

    expect(binding.postFrameCallbacks, hasLength(0));

    // During a frame, the annotation moves out of the pointer.
    isInHitRegion = false;
    expect(binding.postFrameCallbacks, hasLength(0));
    binding.scheduleMouseTrackerPostFrameCheck();
    expect(binding.postFrameCallbacks, hasLength(1));

    binding.flushPostFrameCallbacks(Duration.zero);
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerExitEvent>(
          const PointerExitEvent(position: Offset(0.0, 100.0)).transformed(translate10by20),
        ),
      ]),
    );
    expect(binding.postFrameCallbacks, hasLength(0));
  });

  test('should correctly handle when the pointer is added or removed on the annotation', () {
    late bool isInHitRegion;
    final List<Object> events = <PointerEvent>[];
    final TestAnnotationTarget annotation = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => events.add(event),
      onHover: (PointerHoverEvent event) => events.add(event),
      onExit: (PointerExitEvent event) => events.add(event),
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      if (isInHitRegion) {
        yield TestAnnotationEntry(annotation, Matrix4.translationValues(10, 20, 0));
      }
    });

    isInHitRegion = false;

    // Connect a mouse in the region. Should trigger Enter.
    isInHitRegion = true;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.add, const Offset(0.0, 100.0))],
      ),
    );

    expect(binding.postFrameCallbacks, hasLength(0));
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerEnterEvent>(
          const PointerEnterEvent(position: Offset(0.0, 100.0)).transformed(translate10by20),
        ),
      ]),
    );
    events.clear();

    // Disconnect the mouse from the region. Should trigger Exit.
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.remove, const Offset(0.0, 100.0))],
      ),
    );
    expect(binding.postFrameCallbacks, hasLength(0));
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerExitEvent>(
          const PointerExitEvent(position: Offset(0.0, 100.0)).transformed(translate10by20),
        ),
      ]),
    );
  });

  test('should correctly handle when the pointer moves in or out of the annotation', () {
    late bool isInHitRegion;
    final List<Object> events = <PointerEvent>[];
    final TestAnnotationTarget annotation = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => events.add(event),
      onHover: (PointerHoverEvent event) => events.add(event),
      onExit: (PointerExitEvent event) => events.add(event),
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      if (isInHitRegion) {
        yield TestAnnotationEntry(annotation, Matrix4.translationValues(10, 20, 0));
      }
    });

    isInHitRegion = false;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.add, const Offset(200.0, 100.0))],
      ),
    );
    addTearDown(() => dispatchRemoveDevice());

    expect(binding.postFrameCallbacks, hasLength(0));
    events.clear();

    // Moves the mouse into the region. Should trigger Enter.
    isInHitRegion = true;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(0.0, 100.0))],
      ),
    );
    expect(binding.postFrameCallbacks, hasLength(0));
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerEnterEvent>(
          const PointerEnterEvent(position: Offset(0.0, 100.0)).transformed(translate10by20),
        ),
        EventMatcher<PointerHoverEvent>(
          const PointerHoverEvent(position: Offset(0.0, 100.0)).transformed(translate10by20),
        ),
      ]),
    );
    events.clear();

    // Moves the mouse out of the region. Should trigger Exit.
    isInHitRegion = false;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(200.0, 100.0))],
      ),
    );
    expect(binding.postFrameCallbacks, hasLength(0));
    expect(
      events,
      _equalToEventsOnCriticalFields(<BaseEventMatcher>[
        EventMatcher<PointerExitEvent>(
          const PointerExitEvent(position: Offset(200.0, 100.0)).transformed(translate10by20),
        ),
      ]),
    );
  });

  test('should not schedule post-frame callbacks when no mouse is connected', () {
    setUpMouseAnnotationFinder((Offset position) sync* {});

    // Connect a touch device, which should not be recognized by MouseTracker
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.add, const Offset(0.0, 100.0), kind: PointerDeviceKind.touch),
        ],
      ),
    );
    expect(_mouseTracker.mouseIsConnected, isFalse);

    expect(binding.postFrameCallbacks, hasLength(0));
  });

  test('should not flip out if not all mouse events are listened to', () {
    bool isInHitRegionOne = true;
    bool isInHitRegionTwo = false;
    final TestAnnotationTarget annotation1 = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) {},
    );
    final TestAnnotationTarget annotation2 = TestAnnotationTarget(
      onExit: (PointerExitEvent event) {},
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      if (isInHitRegionOne) {
        yield TestAnnotationEntry(annotation1);
      } else if (isInHitRegionTwo) {
        yield TestAnnotationEntry(annotation2);
      }
    });

    isInHitRegionOne = false;
    isInHitRegionTwo = true;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[
          _pointerData(PointerChange.add, const Offset(0.0, 101.0)),
          _pointerData(PointerChange.hover, const Offset(1.0, 101.0)),
        ],
      ),
    );
    addTearDown(() => dispatchRemoveDevice());

    // Passes if no errors are thrown.
  });

  test('should trigger callbacks between parents and children in correct order', () {
    // This test simulates the scenario of a layer being the child of another.
    //
    //   ———————————
    //   |A        |
    //   |  —————— |
    //   |  |B   | |
    //   |  —————— |
    //   ———————————

    late bool isInB;
    final List<String> logs = <String>[];
    final TestAnnotationTarget annotationA = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => logs.add('enterA'),
      onExit: (PointerExitEvent event) => logs.add('exitA'),
      onHover: (PointerHoverEvent event) => logs.add('hoverA'),
    );
    final TestAnnotationTarget annotationB = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => logs.add('enterB'),
      onExit: (PointerExitEvent event) => logs.add('exitB'),
      onHover: (PointerHoverEvent event) => logs.add('hoverB'),
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      // Children's annotations come before parents'.
      if (isInB) {
        yield TestAnnotationEntry(annotationB);
        yield TestAnnotationEntry(annotationA);
      }
    });

    // Starts out of A.
    isInB = false;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.add, const Offset(0.0, 1.0))],
      ),
    );
    addTearDown(() => dispatchRemoveDevice());
    expect(logs, <String>[]);

    // Moves into B within one frame.
    isInB = true;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(0.0, 10.0))],
      ),
    );
    expect(logs, <String>['enterA', 'enterB', 'hoverB', 'hoverA']);
    logs.clear();

    // Moves out of A within one frame.
    isInB = false;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(0.0, 20.0))],
      ),
    );
    expect(logs, <String>['exitB', 'exitA']);
  });

  test('should trigger callbacks between disjoint siblings in correctly order', () {
    // This test simulates the scenario of 2 sibling layers that do not overlap
    // with each other.
    //
    //   ————————  ————————
    //   |A     |  |B     |
    //   |      |  |      |
    //   ————————  ————————

    late bool isInA;
    late bool isInB;
    final List<String> logs = <String>[];
    final TestAnnotationTarget annotationA = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => logs.add('enterA'),
      onExit: (PointerExitEvent event) => logs.add('exitA'),
      onHover: (PointerHoverEvent event) => logs.add('hoverA'),
    );
    final TestAnnotationTarget annotationB = TestAnnotationTarget(
      onEnter: (PointerEnterEvent event) => logs.add('enterB'),
      onExit: (PointerExitEvent event) => logs.add('exitB'),
      onHover: (PointerHoverEvent event) => logs.add('hoverB'),
    );
    setUpMouseAnnotationFinder((Offset position) sync* {
      if (isInA) {
        yield TestAnnotationEntry(annotationA);
      } else if (isInB) {
        yield TestAnnotationEntry(annotationB);
      }
    });

    // Starts within A.
    isInA = true;
    isInB = false;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.add, const Offset(0.0, 1.0))],
      ),
    );
    addTearDown(() => dispatchRemoveDevice());
    expect(logs, <String>['enterA']);
    logs.clear();

    // Moves into B within one frame.
    isInA = false;
    isInB = true;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(0.0, 10.0))],
      ),
    );
    expect(logs, <String>['exitA', 'enterB', 'hoverB']);
    logs.clear();

    // Moves into A within one frame.
    isInA = true;
    isInB = false;
    RendererBinding.instance.platformDispatcher.onPointerDataPacket!(
      ui.PointerDataPacket(
        data: <ui.PointerData>[_pointerData(PointerChange.hover, const Offset(0.0, 1.0))],
      ),
    );
    expect(logs, <String>['exitB', 'enterA', 'hoverA']);
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

class BaseEventMatcher extends Matcher {
  BaseEventMatcher(this.expected);

  final PointerEvent expected;

  bool _matchesField(
    Map<dynamic, dynamic> matchState,
    String field,
    dynamic actual,
    dynamic expected,
  ) {
    if (actual != expected) {
      addStateInfo(matchState, <dynamic, dynamic>{
        'field': field,
        'expected': expected,
        'actual': actual,
      });
      return false;
    }
    return true;
  }

  @override
  bool matches(dynamic untypedItem, Map<dynamic, dynamic> matchState) {
    final PointerEvent actual = untypedItem as PointerEvent;
    if (!((_matchesField(matchState, 'kind', actual.kind, PointerDeviceKind.mouse) ||
            _matchesField(matchState, 'kind', actual.kind, PointerDeviceKind.stylus)) &&
        _matchesField(matchState, 'position', actual.position, expected.position) &&
        _matchesField(matchState, 'device', actual.device, expected.device) &&
        _matchesField(matchState, 'localPosition', actual.localPosition, expected.localPosition))) {
      return false;
    }
    return true;
  }

  @override
  Description describe(Description description) {
    return description.add('event (critical fields only) ').addDescriptionOf(expected);
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    return mismatchDescription
        .add('has ')
        .addDescriptionOf(matchState['actual'])
        .add(" at field `${matchState['field']}`, which doesn't match the expected ")
        .addDescriptionOf(matchState['expected']);
  }
}

class EventMatcher<T extends PointerEvent> extends BaseEventMatcher {
  EventMatcher(T super.expected);

  @override
  bool matches(dynamic untypedItem, Map<dynamic, dynamic> matchState) {
    if (untypedItem is! T) {
      return false;
    }

    return super.matches(untypedItem, matchState);
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    if (item is! T) {
      return mismatchDescription
          .add('is ')
          .addDescriptionOf(item.runtimeType)
          .add(' and is not a subtype of ')
          .addDescriptionOf(T);
    }
    return super.describeMismatch(item, mismatchDescription, matchState, verbose);
  }
}

class _EventListCriticalFieldsMatcher extends Matcher {
  _EventListCriticalFieldsMatcher(this._expected);

  final Iterable<BaseEventMatcher> _expected;

  @override
  bool matches(dynamic untypedItem, Map<dynamic, dynamic> matchState) {
    if (untypedItem is! Iterable<PointerEvent>) {
      return false;
    }
    final Iterable<PointerEvent> item = untypedItem;
    final Iterator<PointerEvent> iterator = item.iterator;
    if (item.length != _expected.length) {
      return false;
    }
    int i = 0;
    for (final BaseEventMatcher matcher in _expected) {
      iterator.moveNext();
      final Map<dynamic, dynamic> subState = <dynamic, dynamic>{};
      final PointerEvent actual = iterator.current;
      if (!matcher.matches(actual, subState)) {
        addStateInfo(matchState, <dynamic, dynamic>{
          'index': i,
          'expected': matcher.expected,
          'actual': actual,
          'matcher': matcher,
          'state': subState,
        });
        return false;
      }
      i++;
    }
    return true;
  }

  @override
  Description describe(Description description) {
    return description.add('event list (critical fields only) ').addDescriptionOf(_expected);
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    if (item is! Iterable<PointerEvent>) {
      return mismatchDescription.add(
        'is type ${item.runtimeType} instead of Iterable<PointerEvent>',
      );
    } else if (item.length != _expected.length) {
      return mismatchDescription.add('has length ${item.length} instead of ${_expected.length}');
    } else if (matchState['matcher'] == null) {
      return mismatchDescription.add('met unexpected fatal error');
    } else {
      mismatchDescription
          .add('has\n  ')
          .addDescriptionOf(matchState['actual'])
          .add("\nat index ${matchState['index']}, which doesn't match\n  ")
          .addDescriptionOf(matchState['expected'])
          .add('\nsince it ');
      final Description subDescription = StringDescription();
      final Matcher matcher = matchState['matcher'] as Matcher;
      matcher.describeMismatch(
        matchState['actual'],
        subDescription,
        matchState['state'] as Map<dynamic, dynamic>,
        verbose,
      );
      mismatchDescription.add(subDescription.toString());
      return mismatchDescription;
    }
  }
}

Matcher _equalToEventsOnCriticalFields(List<BaseEventMatcher> source) {
  return _EventListCriticalFieldsMatcher(source);
}
