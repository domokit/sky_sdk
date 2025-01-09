// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SemanticsDebugger will schedule a frame', (WidgetTester tester) async {
    await tester.pumpWidget(SemanticsDebugger(child: Container()));

    expect(tester.binding.hasScheduledFrame, isTrue);
  });

  testWidgets('SemanticsDebugger smoke test', (WidgetTester tester) async {
    // This is a smoketest to verify that adding a debugger doesn't crash.
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Semantics(),
            Semantics(container: true),
            Semantics(label: 'label', textDirection: TextDirection.ltr),
          ],
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Stack(
            children: <Widget>[
              Semantics(),
              Semantics(container: true),
              Semantics(label: 'label', textDirection: TextDirection.ltr),
            ],
          ),
        ),
      ),
    );

    expect(true, isTrue); // expect that we reach here without crashing
  });

  testWidgets('SemanticsDebugger draw persistent color based on structure', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Stack(
            children: <Widget>[Semantics(container: true, child: Semantics(container: true))],
          ),
        ),
      ),
    );

    expect(
      find.byType(SemanticsDebugger),
      paints
        ..rect()
        ..rect(color: const Color(0xFFF866FF)),
    );
  });

  testWidgets('SemanticsDebugger reparents subtree', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Stack(
            children: <Widget>[
              Semantics(label: 'label1', textDirection: TextDirection.ltr),
              Positioned(
                key: key,
                left: 0.0,
                top: 0.0,
                width: 100.0,
                height: 100.0,
                child: Semantics(label: 'label2', textDirection: TextDirection.ltr),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Stack(
            children: <Widget>[
              Semantics(label: 'label1', textDirection: TextDirection.ltr),
              Semantics(
                container: true,
                child: Stack(
                  children: <Widget>[
                    Positioned(
                      key: key,
                      left: 0.0,
                      top: 0.0,
                      width: 100.0,
                      height: 100.0,
                      child: Semantics(label: 'label2', textDirection: TextDirection.ltr),
                    ),
                    Semantics(label: 'label3', textDirection: TextDirection.ltr),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Stack(
            children: <Widget>[
              Semantics(label: 'label1', textDirection: TextDirection.ltr),
              Semantics(
                container: true,
                child: Stack(
                  children: <Widget>[
                    Positioned(
                      key: key,
                      left: 0.0,
                      top: 0.0,
                      width: 100.0,
                      height: 100.0,
                      child: Semantics(label: 'label2', textDirection: TextDirection.ltr),
                    ),
                    Semantics(label: 'label3', textDirection: TextDirection.ltr),
                    Semantics(label: 'label4', textDirection: TextDirection.ltr),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('SemanticsDebugger interaction test', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Material(
            child: ListView(
              children: <Widget>[
                ElevatedButton(
                  onPressed: () {
                    log.add('top');
                  },
                  child: const Text('TOP'),
                ),
                ElevatedButton(
                  onPressed: () {
                    log.add('bottom');
                  },
                  child: const Text('BOTTOM'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('TOP'), warnIfMissed: false); // hitting the debugger
    expect(log, equals(<String>['top']));
    log.clear();

    await tester.tap(find.text('BOTTOM'), warnIfMissed: false); // hitting the debugger
    expect(log, equals(<String>['bottom']));
    log.clear();
  });

  testWidgets('SemanticsDebugger interaction test - negative', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Material(
            child: ListView(
              children: <Widget>[
                ElevatedButton(
                  onPressed: () {
                    log.add('top');
                  },
                  child: const Text('TOP', textDirection: TextDirection.ltr),
                ),
                ExcludeSemantics(
                  child: ElevatedButton(
                    onPressed: () {
                      log.add('bottom');
                    },
                    child: const Text('BOTTOM', textDirection: TextDirection.ltr),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('TOP'), warnIfMissed: false); // hitting the debugger
    expect(log, equals(<String>['top']));
    log.clear();

    await tester.tap(find.text('BOTTOM'), warnIfMissed: false); // hitting the debugger
    expect(log, equals(<String>[]));
    log.clear();
  });

  testWidgets('SemanticsDebugger scroll test', (WidgetTester tester) async {
    final Key childKey = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: ListView(
            children: <Widget>[Container(key: childKey, height: 5000.0, color: Colors.green[500])],
          ),
        ),
      ),
    );

    expect(tester.getTopLeft(find.byKey(childKey)).dy, equals(0.0));

    await tester.fling(
      find.byType(ListView),
      const Offset(0.0, -200.0),
      200.0,
      warnIfMissed: false,
    ); // hitting the debugger);
    await tester.pump();

    expect(tester.getTopLeft(find.byKey(childKey)).dy, equals(-480.0));

    await tester.fling(
      find.byType(ListView),
      const Offset(200.0, 0.0),
      200.0,
      warnIfMissed: false,
    ); // hitting the debugger);
    await tester.pump();

    expect(tester.getTopLeft(find.byKey(childKey)).dy, equals(-480.0));

    await tester.fling(
      find.byType(ListView),
      const Offset(-200.0, 0.0),
      200.0,
      warnIfMissed: false,
    ); // hitting the debugger);
    await tester.pump();

    expect(tester.getTopLeft(find.byKey(childKey)).dy, equals(-480.0));

    await tester.fling(
      find.byType(ListView),
      const Offset(0.0, 200.0),
      200.0,
      warnIfMissed: false,
    ); // hitting the debugger);
    await tester.pump();

    expect(tester.getTopLeft(find.byKey(childKey)).dy, equals(0.0));
  });

  testWidgets('SemanticsDebugger long press', (WidgetTester tester) async {
    bool didLongPress = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: GestureDetector(
            onLongPress: () {
              expect(didLongPress, isFalse);
              didLongPress = true;
            },
            child: const Text('target', textDirection: TextDirection.ltr),
          ),
        ),
      ),
    );

    await tester.longPress(find.text('target'), warnIfMissed: false); // hitting the debugger
    expect(didLongPress, isTrue);
  });

  testWidgets('SemanticsDebugger slider', (WidgetTester tester) async {
    double value = 0.75;

    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: SemanticsDebugger(
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: MediaQueryData.fromView(tester.view),
                child: Material(
                  child: Center(
                    child: Slider(
                      value: value,
                      onChanged: (double newValue) {
                        value = newValue;
                      },
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    // The fling below must be such that the velocity estimation examines an
    // offset greater than the kTouchSlop. Too slow or too short a distance, and
    // it won't trigger. The actual distance moved doesn't matter since this is
    // interpreted as a gesture by the semantics debugger and sent to the widget
    // as a semantic action that always moves by 10% of the complete track.
    await tester.fling(
      find.byType(Slider),
      const Offset(-100.0, 0.0),
      2000.0,
      warnIfMissed: false,
    ); // hitting the debugger
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        expect(value, equals(0.65));
      case TargetPlatform.linux:
      case TargetPlatform.windows:
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        expect(value, equals(0.70));
    }
  }, variant: TargetPlatformVariant.all());

  testWidgets('SemanticsDebugger checkbox', (WidgetTester tester) async {
    final Key keyTop = UniqueKey();
    final Key keyBottom = UniqueKey();

    bool? valueTop = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          child: Material(
            child: ListView(
              children: <Widget>[
                Checkbox(
                  key: keyTop,
                  value: valueTop,
                  onChanged: (bool? newValue) {
                    valueTop = newValue;
                  },
                ),
                Checkbox(key: keyBottom, value: false, onChanged: null),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(keyTop), warnIfMissed: false); // hitting the debugger
    expect(valueTop, isTrue);
    valueTop = false;
    expect(valueTop, isFalse);

    await tester.tap(find.byKey(keyBottom), warnIfMissed: false); // hitting the debugger
    expect(valueTop, isFalse);
  });

  testWidgets('SemanticsDebugger checkbox message', (WidgetTester tester) async {
    final Key checkbox = UniqueKey();
    final Key checkboxUnchecked = UniqueKey();
    final Key checkboxDisabled = UniqueKey();
    final Key checkboxDisabledUnchecked = UniqueKey();
    final Key debugger = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          key: debugger,
          child: Material(
            child: ListView(
              children: <Widget>[
                Semantics(
                  container: true,
                  key: checkbox,
                  child: Checkbox(value: true, onChanged: (bool? _) {}),
                ),
                Semantics(
                  container: true,
                  key: checkboxUnchecked,
                  child: Checkbox(value: false, onChanged: (bool? _) {}),
                ),
                Semantics(
                  container: true,
                  key: checkboxDisabled,
                  child: const Checkbox(value: true, onChanged: null),
                ),
                Semantics(
                  container: true,
                  key: checkboxDisabledUnchecked,
                  child: const Checkbox(value: false, onChanged: null),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      _getMessageShownInSemanticsDebugger(
        widgetKey: checkbox,
        debuggerKey: debugger,
        tester: tester,
      ),
      'checked',
    );
    expect(
      _getMessageShownInSemanticsDebugger(
        widgetKey: checkboxUnchecked,
        debuggerKey: debugger,
        tester: tester,
      ),
      'unchecked',
    );
    expect(
      _getMessageShownInSemanticsDebugger(
        widgetKey: checkboxDisabled,
        debuggerKey: debugger,
        tester: tester,
      ),
      'checked; disabled',
    );
    expect(
      _getMessageShownInSemanticsDebugger(
        widgetKey: checkboxDisabledUnchecked,
        debuggerKey: debugger,
        tester: tester,
      ),
      'unchecked; disabled',
    );
  });

  testWidgets(
    'SemanticsDebugger ignores duplicated label and tooltip for Android',
    (WidgetTester tester) async {
      final Key child = UniqueKey();
      final Key debugger = UniqueKey();
      final bool isPlatformAndroid = defaultTargetPlatform == TargetPlatform.android;
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: SemanticsDebugger(
            key: debugger,
            child: Material(
              child: Semantics(container: true, key: child, label: 'text', tooltip: 'text'),
            ),
          ),
        ),
      );

      expect(
        _getMessageShownInSemanticsDebugger(
          widgetKey: child,
          debuggerKey: debugger,
          tester: tester,
        ),
        isPlatformAndroid ? 'text' : 'text\ntext',
      );
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('SemanticsDebugger textfield', (WidgetTester tester) async {
    final UniqueKey textField = UniqueKey();
    final UniqueKey debugger = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: SemanticsDebugger(key: debugger, child: Material(child: TextField(key: textField))),
      ),
    );

    final dynamic semanticsDebuggerPainter = _getSemanticsDebuggerPainter(
      debuggerKey: debugger,
      tester: tester,
    );
    final RenderObject renderTextfield = tester.renderObject(
      find.descendant(of: find.byKey(textField), matching: find.byType(Semantics)).first,
    );

    expect(
      // ignore: avoid_dynamic_calls
      semanticsDebuggerPainter.getMessage(renderTextfield.debugSemantics),
      'textfield',
    );
  });

  testWidgets('SemanticsDebugger label style is used in the painter.', (WidgetTester tester) async {
    final UniqueKey debugger = UniqueKey();
    const TextStyle labelStyle = TextStyle(color: Colors.amber);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SemanticsDebugger(
          key: debugger,
          labelStyle: labelStyle,
          child: Semantics(label: 'label', textDirection: TextDirection.ltr),
        ),
      ),
    );

    expect(
      // ignore: avoid_dynamic_calls
      _getSemanticsDebuggerPainter(debuggerKey: debugger, tester: tester).labelStyle,
      labelStyle,
    );
  });

  testWidgets('SemanticsDebugger label for rtl.', (WidgetTester tester) async {
    final UniqueKey debugger = UniqueKey();
    final Key label = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: SemanticsDebugger(
          key: debugger,
          child: Semantics(label: 'ملصق', textDirection: TextDirection.rtl, key: label),
        ),
      ),
    );

    expect(
      _getMessageShownInSemanticsDebugger(widgetKey: label, debuggerKey: debugger, tester: tester),
      '\u2067ملصق\u2069',
    );
  });

  testWidgets('SemanticsDebugger turns on semantics.', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/147665.
    expect(tester.binding.semanticsEnabled, isFalse);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: SemanticsDebugger(child: Semantics(label: 'Hello World')),
      ),
    );
    expect(tester.binding.semanticsEnabled, isTrue);
  }, semanticsEnabled: false);
}

String _getMessageShownInSemanticsDebugger({
  required Key widgetKey,
  required Key debuggerKey,
  required WidgetTester tester,
}) {
  final dynamic semanticsDebuggerPainter = _getSemanticsDebuggerPainter(
    debuggerKey: debuggerKey,
    tester: tester,
  );
  // ignore: avoid_dynamic_calls
  return semanticsDebuggerPainter.getMessage(
        tester.renderObject(find.byKey(widgetKey)).debugSemantics,
      )
      as String;
}

dynamic _getSemanticsDebuggerPainter({required Key debuggerKey, required WidgetTester tester}) {
  final CustomPaint customPaint =
      tester
              .widgetList(
                find.descendant(of: find.byKey(debuggerKey), matching: find.byType(CustomPaint)),
              )
              .first
          as CustomPaint;
  final dynamic semanticsDebuggerPainter = customPaint.foregroundPainter;
  expect(semanticsDebuggerPainter.runtimeType.toString(), '_SemanticsDebuggerPainter');
  return semanticsDebuggerPainter;
}
