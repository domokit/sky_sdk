// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

// Only check the initial lines of the message, since the message walks the
// entire widget tree back, and any changes to the widget tree break these
// tests if we check the entire message.
void _expectStartsWith(List<String?> actual, List<String?> matcher) {
  expect(actual.sublist(0, matcher.length), equals(matcher));
}

void main() {
  final _MockLiveTestWidgetsFlutterBinding binding = _MockLiveTestWidgetsFlutterBinding();

  testWidgets('Should print message on pointer events', (WidgetTester tester) async {
    final List<String?> printedMessages = <String?>[];

    int invocations = 0;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: GestureDetector(
            onTap: () {
              invocations++;
            },
            child: const Text('Test'),
          ),
        ),
      ),
    );

    final Size windowCenter = tester.view.physicalSize / tester.view.devicePixelRatio / 2;
    final double windowCenterX = windowCenter.width;
    final double windowCenterY = windowCenter.height;

    final Offset widgetCenter = tester.getRect(find.byType(Text)).center;
    expect(widgetCenter.dx, windowCenterX);
    expect(widgetCenter.dy, windowCenterY);

    await binding.collectDebugPrints(printedMessages, () async {
      await tester.tap(find.byType(Text));
    });
    await tester.pump();
    expect(invocations, 0);

    _expectStartsWith(
      printedMessages,
      '''
Some possible finders for the widgets at Offset(400.0, 300.0):
  find.text('Test')
'''.trim().split('\n'),
    );
    printedMessages.clear();

    await binding.collectDebugPrints(printedMessages, () async {
      await tester.tapAt(const Offset(1, 1));
    });
    expect(
      printedMessages,
      equals(
        '''
No widgets found at Offset(1.0, 1.0).
'''.trim().split('\n'),
      ),
    );
  });

  testWidgets('Should print message on pointer events with setSurfaceSize', (
    WidgetTester tester,
  ) async {
    final List<String?> printedMessages = <String?>[];

    int invocations = 0;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: GestureDetector(
            onTap: () {
              invocations++;
            },
            child: const Text('Test'),
          ),
        ),
      ),
    );

    final Size originalSize = tester.binding.renderView.size;
    await tester.binding.setSurfaceSize(const Size(2000, 1800));
    try {
      await tester.pump();

      final Offset widgetCenter = tester.getRect(find.byType(Text)).center;
      expect(widgetCenter.dx, 1000);
      expect(widgetCenter.dy, 900);

      await binding.collectDebugPrints(printedMessages, () async {
        await tester.tap(find.byType(Text));
      });
      await tester.pump();
      expect(invocations, 0);

      _expectStartsWith(
        printedMessages,
        '''
Some possible finders for the widgets at Offset(1000.0, 900.0):
  find.text('Test')
'''.trim().split('\n'),
      );
      printedMessages.clear();

      await binding.collectDebugPrints(printedMessages, () async {
        await tester.tapAt(const Offset(1, 1));
      });
      expect(
        printedMessages,
        equals(
          '''
No widgets found at Offset(1.0, 1.0).
'''.trim().split('\n'),
        ),
      );
    } finally {
      await tester.binding.setSurfaceSize(originalSize);
    }
  });
}

class _MockLiveTestWidgetsFlutterBinding extends LiveTestWidgetsFlutterBinding {
  @override
  void handlePointerEventForSource(
    PointerEvent event, {
    TestBindingEventSource source = TestBindingEventSource.device,
  }) {
    // In this test we use `WidgetTester.tap` to simulate real device touches.
    // `WidgetTester.tap` sends events in the local coordinate system, while
    // real devices touches sends event in the global coordinate system.
    // See the documentation of [handlePointerEventForSource] for details.
    if (source == TestBindingEventSource.test) {
      final RenderView renderView = renderViews.firstWhere(
        (RenderView r) => r.flutterView.viewId == event.viewId,
      );
      final PointerEvent globalEvent = event.copyWith(
        position: localToGlobal(event.position, renderView),
      );
      return super.handlePointerEventForSource(globalEvent);
    }
    return super.handlePointerEventForSource(event, source: source);
  }

  List<String?>? _storeDebugPrints;

  @override
  DebugPrintCallback get debugPrintOverride {
    return _storeDebugPrints == null
        ? super.debugPrintOverride
        : ((String? message, {int? wrapWidth}) => _storeDebugPrints!.add(message));
  }

  // Execute `task` while redirecting [debugPrint] to appending to `store`.
  Future<void> collectDebugPrints(List<String?>? store, AsyncValueGetter<void> task) async {
    _storeDebugPrints = store;
    try {
      await task();
    } finally {
      _storeDebugPrints = null;
    }
  }
}
