// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

typedef HandleEventCallback = void Function(PointerEvent event);

class TestGestureFlutterBinding extends BindingBase with GestureBinding, SchedulerBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  /// The singleton instance of this object.
  ///
  /// Provides access to the features exposed by this class. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [TestGestureFlutterBinding.ensureInitialized].
  static TestGestureFlutterBinding get instance => BindingBase.checkInstance(_instance);
  static TestGestureFlutterBinding? _instance;

  /// Returns an instance of the [TestGestureFlutterBinding], creating and
  /// initializing it if necessary.
  static TestGestureFlutterBinding ensureInitialized() {
    if (_instance == null) {
      TestGestureFlutterBinding();
    }
    return _instance!;
  }

  HandleEventCallback? onHandlePointerEvent;

  @override
  void handlePointerEvent(PointerEvent event) {
    onHandlePointerEvent?.call(event);
    super.handlePointerEvent(event);
  }

  HandleEventCallback? onHandleEvent;

  @override
  void handleEvent(PointerEvent event, HitTestEntry entry) {
    super.handleEvent(event, entry);
    onHandleEvent?.call(event);
  }
}

void main() {
  final TestGestureFlutterBinding binding = TestGestureFlutterBinding.ensureInitialized();

  test('Pointer tap events', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.down),
        ui.PointerData(change: ui.PointerChange.up),
      ],
    );

    final List<PointerEvent> events = <PointerEvent>[];
    TestGestureFlutterBinding.instance.onHandleEvent = events.add;

    GestureBinding.instance.platformDispatcher.onPointerDataPacket?.call(packet);
    expect(events.length, 2);
    expect(events[0], isA<PointerDownEvent>());
    expect(events[1], isA<PointerUpEvent>());
  });

  test('Pointer move events', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.down),
        ui.PointerData(change: ui.PointerChange.move),
        ui.PointerData(change: ui.PointerChange.up),
      ],
    );

    final List<PointerEvent> events = <PointerEvent>[];
    binding.onHandleEvent = events.add;

    GestureBinding.instance.platformDispatcher.onPointerDataPacket?.call(packet);
    expect(events.length, 3);
    expect(events[0], isA<PointerDownEvent>());
    expect(events[1], isA<PointerMoveEvent>());
    expect(events[2], isA<PointerUpEvent>());
  });

  test('Pointer hover events', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.add),
        ui.PointerData(change: ui.PointerChange.hover),
        ui.PointerData(change: ui.PointerChange.hover),
        ui.PointerData(change: ui.PointerChange.remove),
        ui.PointerData(change: ui.PointerChange.add),
        ui.PointerData(change: ui.PointerChange.hover),
      ],
    );

    final List<PointerEvent> pointerRouterEvents = <PointerEvent>[];
    GestureBinding.instance.pointerRouter.addGlobalRoute(pointerRouterEvents.add);

    final List<PointerEvent> events = <PointerEvent>[];
    binding.onHandleEvent = events.add;

    GestureBinding.instance.platformDispatcher.onPointerDataPacket?.call(packet);
    expect(events.length, 3);
    expect(events[0], isA<PointerHoverEvent>());
    expect(events[1], isA<PointerHoverEvent>());
    expect(events[2], isA<PointerHoverEvent>());
    expect(
      pointerRouterEvents.length,
      6,
      reason: 'pointerRouterEvents contains: $pointerRouterEvents',
    );
    expect(pointerRouterEvents[0], isA<PointerAddedEvent>());
    expect(pointerRouterEvents[1], isA<PointerHoverEvent>());
    expect(pointerRouterEvents[2], isA<PointerHoverEvent>());
    expect(pointerRouterEvents[3], isA<PointerRemovedEvent>());
    expect(pointerRouterEvents[4], isA<PointerAddedEvent>());
    expect(pointerRouterEvents[5], isA<PointerHoverEvent>());
  });

  test('Pointer cancel events', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[ui.PointerData(change: ui.PointerChange.down), ui.PointerData()],
    );

    final List<PointerEvent> events = <PointerEvent>[];
    binding.onHandleEvent = events.add;

    GestureBinding.instance.platformDispatcher.onPointerDataPacket?.call(packet);
    expect(events.length, 2);
    expect(events[0], isA<PointerDownEvent>());
    expect(events[1], isA<PointerCancelEvent>());
  });

  test('Can cancel pointers', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.down),
        ui.PointerData(change: ui.PointerChange.up),
      ],
    );

    final List<PointerEvent> events = <PointerEvent>[];
    binding.onHandleEvent = (PointerEvent event) {
      events.add(event);
      if (event is PointerDownEvent) {
        binding.cancelPointer(event.pointer);
      }
    };

    GestureBinding.instance.platformDispatcher.onPointerDataPacket?.call(packet);
    expect(events.length, 2);
    expect(events[0], isA<PointerDownEvent>());
    expect(events[1], isA<PointerCancelEvent>());
  });

  const double devicePixelRatio = 2.5;

  test('Can expand add and hover pointers', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.add, device: 24),
        ui.PointerData(change: ui.PointerChange.hover, device: 24),
        ui.PointerData(change: ui.PointerChange.remove, device: 24),
        ui.PointerData(change: ui.PointerChange.add, device: 24),
        ui.PointerData(change: ui.PointerChange.hover, device: 24),
      ],
    );

    final List<PointerEvent> events =
        PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

    expect(events.length, 5);
    expect(events[0], isA<PointerAddedEvent>());
    expect(events[1], isA<PointerHoverEvent>());
    expect(events[2], isA<PointerRemovedEvent>());
    expect(events[3], isA<PointerAddedEvent>());
    expect(events[4], isA<PointerHoverEvent>());
  });

  test('Can handle malformed scrolling event.', () {
    ui.PointerDataPacket packet = const ui.PointerDataPacket(
      data: <ui.PointerData>[ui.PointerData(change: ui.PointerChange.add, device: 24)],
    );
    List<PointerEvent> events =
        PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

    expect(events.length, 1);
    expect(events[0], isA<PointerAddedEvent>());

    // Send packet contains malformed scroll events.
    packet = const ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(
          signalKind: ui.PointerSignalKind.scroll,
          device: 24,
          scrollDeltaX: double.infinity,
          scrollDeltaY: 10,
        ),
        ui.PointerData(
          signalKind: ui.PointerSignalKind.scroll,
          device: 24,
          scrollDeltaX: double.nan,
          scrollDeltaY: 10,
        ),
        ui.PointerData(
          signalKind: ui.PointerSignalKind.scroll,
          device: 24,
          scrollDeltaX: double.negativeInfinity,
          scrollDeltaY: 10,
        ),
        ui.PointerData(
          signalKind: ui.PointerSignalKind.scroll,
          device: 24,
          scrollDeltaY: double.infinity,
          scrollDeltaX: 10,
        ),
        ui.PointerData(
          signalKind: ui.PointerSignalKind.scroll,
          device: 24,
          scrollDeltaY: double.nan,
          scrollDeltaX: 10,
        ),
        ui.PointerData(
          signalKind: ui.PointerSignalKind.scroll,
          device: 24,
          scrollDeltaY: double.negativeInfinity,
          scrollDeltaX: 10,
        ),
      ],
    );
    events = PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();
    expect(events.length, 0);

    // Send packet with a valid scroll event.
    packet = const ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(
          signalKind: ui.PointerSignalKind.scroll,
          device: 24,
          scrollDeltaX: 10,
          scrollDeltaY: 10,
        ),
      ],
    );
    // Make sure PointerEventConverter can expand when device pixel ratio is valid.
    events = PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();
    expect(events.length, 1);
    expect(events[0], isA<PointerScrollEvent>());

    // Make sure PointerEventConverter returns none when device pixel ratio is invalid.
    events = PointerEventConverter.expand(packet.data, (int viewId) => 0).toList();
    expect(events.length, 0);
  });

  test('Can expand pointer scroll events', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.add),
        ui.PointerData(change: ui.PointerChange.hover, signalKind: ui.PointerSignalKind.scroll),
      ],
    );

    final List<PointerEvent> events =
        PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

    expect(events.length, 2);
    expect(events[0], isA<PointerAddedEvent>());
    expect(events[1], isA<PointerScrollEvent>());
  });

  test('Should synthesize kPrimaryButton for touch when no button is set', () {
    final Offset location = const Offset(10.0, 10.0) * devicePixelRatio;
    final ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(
          change: ui.PointerChange.add,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.hover,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.down,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(change: ui.PointerChange.up, physicalX: location.dx, physicalY: location.dy),
      ],
    );

    final List<PointerEvent> events =
        PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

    expect(events.length, 5);
    expect(events[0], isA<PointerAddedEvent>());
    expect(events[0].buttons, equals(0));
    expect(events[1], isA<PointerHoverEvent>());
    expect(events[1].buttons, equals(0));
    expect(events[2], isA<PointerDownEvent>());
    expect(events[2].buttons, equals(kPrimaryButton));
    expect(events[3], isA<PointerMoveEvent>());
    expect(events[3].buttons, equals(kPrimaryButton));
    expect(events[4], isA<PointerUpEvent>());
    expect(events[4].buttons, equals(0));
  });

  test('Should not synthesize kPrimaryButton for touch when a button is set', () {
    final Offset location = const Offset(10.0, 10.0) * devicePixelRatio;
    final ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(
          change: ui.PointerChange.add,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.hover,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.down,
          buttons: kSecondaryButton,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          buttons: kSecondaryButton,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(change: ui.PointerChange.up, physicalX: location.dx, physicalY: location.dy),
      ],
    );

    final List<PointerEvent> events =
        PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

    expect(events.length, 5);
    expect(events[0], isA<PointerAddedEvent>());
    expect(events[0].buttons, equals(0));
    expect(events[1], isA<PointerHoverEvent>());
    expect(events[1].buttons, equals(0));
    expect(events[2], isA<PointerDownEvent>());
    expect(events[2].buttons, equals(kSecondaryButton));
    expect(events[3], isA<PointerMoveEvent>());
    expect(events[3].buttons, equals(kSecondaryButton));
    expect(events[4], isA<PointerUpEvent>());
    expect(events[4].buttons, equals(0));
  });

  test('Should synthesize kPrimaryButton for stylus when no button is set', () {
    final Offset location = const Offset(10.0, 10.0) * devicePixelRatio;
    for (final PointerDeviceKind kind in <PointerDeviceKind>[
      PointerDeviceKind.stylus,
      PointerDeviceKind.invertedStylus,
    ]) {
      final ui.PointerDataPacket packet = ui.PointerDataPacket(
        data: <ui.PointerData>[
          ui.PointerData(
            change: ui.PointerChange.add,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.hover,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.down,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.move,
            buttons: kSecondaryStylusButton,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.up,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
        ],
      );

      final List<PointerEvent> events =
          PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

      expect(events.length, 5);
      expect(events[0], isA<PointerAddedEvent>());
      expect(events[0].buttons, equals(0));
      expect(events[1], isA<PointerHoverEvent>());
      expect(events[1].buttons, equals(0));
      expect(events[2], isA<PointerDownEvent>());
      expect(events[2].buttons, equals(kPrimaryButton));
      expect(events[3], isA<PointerMoveEvent>());
      expect(events[3].buttons, equals(kSecondaryStylusButton));
      expect(events[4], isA<PointerUpEvent>());
      expect(events[4].buttons, equals(0));
    }
  });

  test('Should synthesize kPrimaryButton for unknown devices when no button is set', () {
    final Offset location = const Offset(10.0, 10.0) * devicePixelRatio;
    const PointerDeviceKind kind = PointerDeviceKind.unknown;
    final ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(
          change: ui.PointerChange.add,
          kind: kind,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.hover,
          kind: kind,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.down,
          kind: kind,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.move,
          buttons: kSecondaryButton,
          kind: kind,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
        ui.PointerData(
          change: ui.PointerChange.up,
          kind: kind,
          physicalX: location.dx,
          physicalY: location.dy,
        ),
      ],
    );

    final List<PointerEvent> events =
        PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

    expect(events.length, 5);
    expect(events[0], isA<PointerAddedEvent>());
    expect(events[0].buttons, equals(0));
    expect(events[1], isA<PointerHoverEvent>());
    expect(events[1].buttons, equals(0));
    expect(events[2], isA<PointerDownEvent>());
    expect(events[2].buttons, equals(kPrimaryButton));
    expect(events[3], isA<PointerMoveEvent>());
    expect(events[3].buttons, equals(kSecondaryButton));
    expect(events[4], isA<PointerUpEvent>());
    expect(events[4].buttons, equals(0));
  });

  test('Should not synthesize kPrimaryButton for mouse', () {
    final Offset location = const Offset(10.0, 10.0) * devicePixelRatio;
    for (final PointerDeviceKind kind in <PointerDeviceKind>[PointerDeviceKind.mouse]) {
      final ui.PointerDataPacket packet = ui.PointerDataPacket(
        data: <ui.PointerData>[
          ui.PointerData(
            change: ui.PointerChange.add,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.hover,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.down,
            kind: kind,
            buttons: kMiddleMouseButton,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.move,
            kind: kind,
            buttons: kMiddleMouseButton | kSecondaryMouseButton,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
          ui.PointerData(
            change: ui.PointerChange.up,
            kind: kind,
            physicalX: location.dx,
            physicalY: location.dy,
          ),
        ],
      );

      final List<PointerEvent> events =
          PointerEventConverter.expand(packet.data, (int viewId) => devicePixelRatio).toList();

      expect(events.length, 5);
      expect(events[0], isA<PointerAddedEvent>());
      expect(events[0].buttons, equals(0));
      expect(events[1], isA<PointerHoverEvent>());
      expect(events[1].buttons, equals(0));
      expect(events[2], isA<PointerDownEvent>());
      expect(events[2].buttons, equals(kMiddleMouseButton));
      expect(events[3], isA<PointerMoveEvent>());
      expect(events[3].buttons, equals(kMiddleMouseButton | kSecondaryMouseButton));
      expect(events[4], isA<PointerUpEvent>());
      expect(events[4].buttons, equals(0));
    }
  });

  test('Pointer pan/zoom events', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.panZoomStart),
        ui.PointerData(change: ui.PointerChange.panZoomUpdate),
        ui.PointerData(change: ui.PointerChange.panZoomEnd),
      ],
    );

    final List<PointerEvent> events = <PointerEvent>[];
    binding.onHandleEvent = events.add;

    binding.platformDispatcher.onPointerDataPacket?.call(packet);
    expect(events.length, 3);
    expect(events[0], isA<PointerPanZoomStartEvent>());
    expect(events[1], isA<PointerPanZoomUpdateEvent>());
    expect(events[2], isA<PointerPanZoomEndEvent>());
  });

  test('Error handling', () {
    const ui.PointerDataPacket packet = ui.PointerDataPacket(
      data: <ui.PointerData>[
        ui.PointerData(change: ui.PointerChange.down),
        ui.PointerData(change: ui.PointerChange.up),
      ],
    );

    final List<String> events = <String>[];
    binding.onHandlePointerEvent = (PointerEvent event) {
      throw Exception('zipzapzooey $event');
    };
    FlutterError.onError = (FlutterErrorDetails details) {
      events.add(details.toString());
    };
    try {
      GestureBinding.instance.platformDispatcher.onPointerDataPacket?.call(packet);
      expect(events.length, 1);
      expect(
        events[0],
        contains('while handling a pointer data\npacket'),
      ); // The default stringifying behavior uses 65 character wrapWidth.
      expect(events[0], contains('zipzapzooey'));
      expect(events[0], contains('PointerDownEvent'));
      expect(
        events[0],
        isNot(contains('PointerUpEvent')),
      ); // Failure happens on the first message, remaining messages aren't processed.
    } finally {
      binding.onHandlePointerEvent = null;
      FlutterError.onError = FlutterError.presentError;
    }
  });

  test('PointerEventConverter processes view IDs', () {
    const int startID = 987654;
    const List<ui.PointerData> data = <ui.PointerData>[
      ui.PointerData(
        viewId: startID + 0,
        change: ui.PointerChange.cancel, // ignore: avoid_redundant_argument_values
      ),
      ui.PointerData(viewId: startID + 1, change: ui.PointerChange.add),
      ui.PointerData(viewId: startID + 2, change: ui.PointerChange.remove),
      ui.PointerData(viewId: startID + 3, change: ui.PointerChange.hover),
      ui.PointerData(viewId: startID + 4, change: ui.PointerChange.down),
      ui.PointerData(viewId: startID + 5, change: ui.PointerChange.move),
      ui.PointerData(viewId: startID + 6, change: ui.PointerChange.up),
      ui.PointerData(viewId: startID + 7, change: ui.PointerChange.panZoomStart),
      ui.PointerData(viewId: startID + 8, change: ui.PointerChange.panZoomUpdate),
      ui.PointerData(viewId: startID + 9, change: ui.PointerChange.panZoomEnd),
    ];

    final List<int> viewIds = <int>[];
    double devicePixelRatioGetter(int viewId) {
      viewIds.add(viewId);
      return viewId / 10.0;
    }

    final List<PointerEvent> events =
        PointerEventConverter.expand(data, devicePixelRatioGetter).toList();

    final List<int> expectedViewIds = List<int>.generate(10, (int index) => startID + index);
    expect(viewIds, expectedViewIds);
    expect(events, hasLength(10));
    expect(events.map((PointerEvent event) => event.viewId), expectedViewIds);
  });
}
