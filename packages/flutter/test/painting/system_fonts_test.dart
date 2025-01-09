// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

Future<void> verifyMarkedNeedsLayoutDuringTransientCallbacksPhase(
  WidgetTester tester,
  RenderObject renderObject,
) async {
  assert(!renderObject.debugNeedsLayout);

  const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
  await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
    'flutter/system',
    SystemChannels.system.codec.encodeMessage(data),
    (ByteData? data) {},
  );

  final Completer<bool> animation = Completer<bool>();
  tester.binding.scheduleFrameCallback((Duration timeStamp) {
    animation.complete(renderObject.debugNeedsLayout);
  });

  // The fonts change does not mark the render object as needing layout
  // immediately.
  expect(renderObject.debugNeedsLayout, isFalse);
  await tester.pump();
  expect(await animation.future, isTrue);
}

void main() {
  testWidgets('RenderParagraph relayout upon system fonts changes', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: Text('text widget')));
    final RenderObject renderObject = tester.renderObject(find.text('text widget'));
    await verifyMarkedNeedsLayoutDuringTransientCallbacksPhase(tester, renderObject);
  });

  testWidgets(
    'Safe to query a RelayoutWhenSystemFontsChangeMixin for text layout after system fonts changes',
    (WidgetTester tester) async {
      final _RenderCustomRelayoutWhenSystemFontsChange child =
          _RenderCustomRelayoutWhenSystemFontsChange();
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: WidgetToRenderBoxAdapter(renderBox: child),
        ),
      );
      const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/system',
        SystemChannels.system.codec.encodeMessage(data),
        (ByteData? data) {},
      );
      expect(child.hasValidTextLayout, isTrue);
    },
  );

  testWidgets('RenderEditable relayout upon system fonts changes', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: SelectableText('text widget')));

    final EditableTextState state = tester.state(find.byType(EditableText));
    await verifyMarkedNeedsLayoutDuringTransientCallbacksPhase(tester, state.renderEditable);
  });

  testWidgets('Banner repaint upon system fonts changes', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Banner(
        message: 'message',
        location: BannerLocation.topStart,
        textDirection: TextDirection.ltr,
        layoutDirection: TextDirection.ltr,
      ),
    );
    const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/system',
      SystemChannels.system.codec.encodeMessage(data),
      (ByteData? data) {},
    );
    final RenderObject renderObject = tester.renderObject(find.byType(Banner));
    expect(renderObject.debugNeedsPaint, isTrue);
  });

  testWidgets(
    'CupertinoDatePicker reset cache upon system fonts change - date time mode',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(home: CupertinoDatePicker(onDateTimeChanged: (DateTime dateTime) {})),
      );
      final dynamic state = tester.state(find.byType(CupertinoDatePicker));
      // ignore: avoid_dynamic_calls
      final Map<int, double> cache = state.estimatedColumnWidths as Map<int, double>;
      expect(cache.isNotEmpty, isTrue);
      const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/system',
        SystemChannels.system.codec.encodeMessage(data),
        (ByteData? data) {},
      );
      // Cache should be cleaned.
      expect(cache.isEmpty, isTrue);
      final Element element = tester.element(find.byType(CupertinoDatePicker));
      expect(element.dirty, isTrue);
    },
    // TODO(yjbanov): cupertino does not work on the Web yet: https://github.com/flutter/flutter/issues/41920
    skip: isBrowser,
  );

  testWidgets(
    'CupertinoDatePicker reset cache upon system fonts change - date mode',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoDatePicker(
            mode: CupertinoDatePickerMode.date,
            onDateTimeChanged: (DateTime dateTime) {},
          ),
        ),
      );
      final dynamic state = tester.state(find.byType(CupertinoDatePicker));
      // ignore: avoid_dynamic_calls
      final Map<int, double> cache = state.estimatedColumnWidths as Map<int, double>;
      // Simulates font missing.
      cache.clear();
      const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/system',
        SystemChannels.system.codec.encodeMessage(data),
        (ByteData? data) {},
      );
      // Cache should be replenished
      expect(cache.isNotEmpty, isTrue);
      final Element element = tester.element(find.byType(CupertinoDatePicker));
      expect(element.dirty, isTrue);
    },
    // TODO(yjbanov): cupertino does not work on the Web yet: https://github.com/flutter/flutter/issues/41920
    skip: isBrowser,
  );

  testWidgets(
    'CupertinoDatePicker reset cache upon system fonts change - time mode',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(home: CupertinoTimerPicker(onTimerDurationChanged: (Duration d) {})),
      );
      final dynamic state = tester.state(find.byType(CupertinoTimerPicker));
      // Simulates wrong metrics due to font missing.
      // ignore: avoid_dynamic_calls
      state.numberLabelWidth = 0.0;
      // ignore: avoid_dynamic_calls
      state.numberLabelHeight = 0.0;
      // ignore: avoid_dynamic_calls
      state.numberLabelBaseline = 0.0;
      const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/system',
        SystemChannels.system.codec.encodeMessage(data),
        (ByteData? data) {},
      );
      // Metrics should be refreshed
      // ignore: avoid_dynamic_calls
      expect(state.numberLabelWidth, lessThan(46.0 + precisionErrorTolerance));
      // ignore: avoid_dynamic_calls
      expect(state.numberLabelHeight, lessThan(23.0 + precisionErrorTolerance));
      // ignore: avoid_dynamic_calls
      expect(state.numberLabelBaseline, lessThan(18.400070190429688 + precisionErrorTolerance));
      final Element element = tester.element(find.byType(CupertinoTimerPicker));
      expect(element.dirty, isTrue);
    },
    // TODO(yjbanov): cupertino does not work on the Web yet: https://github.com/flutter/flutter/issues/41920
    skip: isBrowser,
  );

  testWidgets('RangeSlider relayout upon system fonts changes more than once', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: RangeSlider(
            values: const RangeValues(0.0, 1.0),
            onChanged: (RangeValues values) {},
          ),
        ),
      ),
    );
    const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/system',
      SystemChannels.system.codec.encodeMessage(data),
      (ByteData? data) {},
    );
    final RenderObject renderObject = tester.renderObject(
      find.byWidgetPredicate(
        (Widget widget) => widget.runtimeType.toString() == '_RangeSliderRenderObjectWidget',
      ),
    );
    await verifyMarkedNeedsLayoutDuringTransientCallbacksPhase(tester, renderObject);
  });

  testWidgets('Slider relayout upon system fonts changes', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(home: Material(child: Slider(value: 0.0, onChanged: (double value) {}))),
    );
    // _RenderSlider is the last render object in the tree.
    final RenderObject renderObject = tester.allRenderObjects.last;
    await verifyMarkedNeedsLayoutDuringTransientCallbacksPhase(tester, renderObject);
  });

  testWidgets('TimePicker relayout upon system fonts changes', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('X'),
                  onPressed: () {
                    showTimePicker(
                      context: context,
                      initialTime: const TimeOfDay(hour: 7, minute: 0),
                      builder: (BuildContext context, Widget? child) {
                        return Directionality(
                          key: const Key('parent'),
                          textDirection: TextDirection.ltr,
                          child: child!,
                        );
                      },
                    );
                  },
                );
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle();
    const Map<String, dynamic> data = <String, dynamic>{'type': 'fontsChange'};
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/system',
      SystemChannels.system.codec.encodeMessage(data),
      (ByteData? data) {},
    );
    final RenderObject renderObject = tester.renderObject(
      find.descendant(
        of: find.byKey(const Key('parent')),
        matching: find.byKey(const ValueKey<String>('time-picker-dial')),
      ),
    );
    expect(renderObject.debugNeedsPaint, isTrue);
  });
}

class _RenderCustomRelayoutWhenSystemFontsChange extends RenderBox
    with RelayoutWhenSystemFontsChangeMixin {
  bool hasValidTextLayout = false;

  @override
  void systemFontsDidChange() {
    super.systemFontsDidChange();
    hasValidTextLayout = false;
  }

  @override
  void performLayout() {
    size = constraints.biggest;
    hasValidTextLayout = true;
  }
}
