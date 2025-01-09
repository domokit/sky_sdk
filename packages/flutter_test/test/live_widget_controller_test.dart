// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

// This test is very fragile and bypasses some zone-related checks.
// It is written this way to verify some invariants that would otherwise
// be difficult to check.
// Do not use this test as a guide for writing good Flutter code.

class TestBinding extends WidgetsFlutterBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  @override
  bool debugCheckZone(String entryPoint) {
    return true;
  }

  static TestBinding get instance => BindingBase.checkInstance(_instance);
  static TestBinding? _instance;

  static TestBinding ensureInitialized() {
    if (TestBinding._instance == null) {
      TestBinding();
    }
    return TestBinding.instance;
  }
}

class CountButton extends StatefulWidget {
  const CountButton({super.key});

  @override
  State<CountButton> createState() => _CountButtonState();
}

class _CountButtonState extends State<CountButton> {
  int counter = 0;
  @override
  Widget build(BuildContext context) {
    return ElevatedButton(
      child: Text('Counter $counter'),
      onPressed: () {
        setState(() {
          counter += 1;
        });
      },
    );
  }
}

class AnimateSample extends StatefulWidget {
  const AnimateSample({super.key});

  @override
  State<AnimateSample> createState() => _AnimateSampleState();
}

class _AnimateSampleState extends State<AnimateSample> with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this, duration: const Duration(seconds: 1))..forward();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _controller,
      builder: (BuildContext context, _) => Text('Value: ${_controller.value}'),
    );
  }
}

void main() {
  TestBinding.ensureInitialized();

  test('Test pump on LiveWidgetController', () async {
    runApp(const MaterialApp(home: Center(child: CountButton())));

    await SchedulerBinding.instance.endOfFrame;
    final WidgetController controller = LiveWidgetController(WidgetsBinding.instance);
    await controller.tap(find.text('Counter 0'));
    expect(find.text('Counter 0'), findsOneWidget);
    expect(find.text('Counter 1'), findsNothing);
    await controller.pump();
    expect(find.text('Counter 0'), findsNothing);
    expect(find.text('Counter 1'), findsOneWidget);
  });

  test('Test pumpAndSettle on LiveWidgetController', () async {
    runApp(const MaterialApp(home: Center(child: AnimateSample())));
    await SchedulerBinding.instance.endOfFrame;
    final WidgetController controller = LiveWidgetController(WidgetsBinding.instance);
    expect(find.text('Value: 1.0'), findsNothing);
    await controller.pumpAndSettle();
    expect(find.text('Value: 1.0'), findsOneWidget);
  });

  test('Input event array on LiveWidgetController', () async {
    final List<String> logs = <String>[];
    runApp(
      MaterialApp(
        home: Listener(
          onPointerDown: (PointerDownEvent event) => logs.add('down ${event.buttons}'),
          onPointerMove: (PointerMoveEvent event) => logs.add('move ${event.buttons}'),
          onPointerUp: (PointerUpEvent event) => logs.add('up ${event.buttons}'),
          child: const Text('test'),
        ),
      ),
    );
    await SchedulerBinding.instance.endOfFrame;
    final WidgetController controller = LiveWidgetController(WidgetsBinding.instance);

    final Offset location = controller.getCenter(find.text('test'));
    final List<PointerEventRecord> records = <PointerEventRecord>[
      PointerEventRecord(Duration.zero, <PointerEvent>[
        // Typically PointerAddedEvent is not used in testers, but for records
        // captured on a device it is usually what starts a gesture.
        PointerAddedEvent(position: location),
        PointerDownEvent(position: location, buttons: kSecondaryMouseButton, pointer: 1),
      ]),
      ...<PointerEventRecord>[
        for (
          Duration t = const Duration(milliseconds: 5);
          t < const Duration(milliseconds: 80);
          t += const Duration(milliseconds: 16)
        )
          PointerEventRecord(t, <PointerEvent>[
            PointerMoveEvent(
              timeStamp: t - const Duration(milliseconds: 1),
              position: location,
              buttons: kSecondaryMouseButton,
              pointer: 1,
            ),
          ]),
      ],
      PointerEventRecord(const Duration(milliseconds: 80), <PointerEvent>[
        PointerUpEvent(
          timeStamp: const Duration(milliseconds: 79),
          position: location,
          buttons: kSecondaryMouseButton,
          pointer: 1,
        ),
      ]),
    ];
    final List<Duration> timeDiffs = await controller.handlePointerEventRecord(records);

    expect(timeDiffs.length, records.length);
    for (final Duration diff in timeDiffs) {
      // Allow some freedom of time delay in real world.
      // TODO(pdblasi-google): The expected wiggle room should be -1, but occasional
      // results were reaching -6. This assert has been adjusted to reduce flakiness,
      // but the root cause is still unknown. (https://github.com/flutter/flutter/issues/109638)
      assert(
        diff.inMilliseconds > -7,
        'timeDiffs were: $timeDiffs (offending time was ${diff.inMilliseconds}ms)',
      );
    }

    const String b = '$kSecondaryMouseButton';
    expect(logs.first, 'down $b');
    for (int i = 1; i < logs.length - 1; i++) {
      expect(logs[i], 'move $b');
    }
    expect(logs.last, 'up $b');
  });
}
