// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class HoverClient extends StatefulWidget {
  const HoverClient({super.key, this.onHover, this.child, this.onEnter, this.onExit});

  final ValueChanged<bool>? onHover;
  final Widget? child;
  final VoidCallback? onEnter;
  final VoidCallback? onExit;

  @override
  HoverClientState createState() => HoverClientState();
}

class HoverClientState extends State<HoverClient> {
  void _onExit(PointerExitEvent details) {
    widget.onExit?.call();
    widget.onHover?.call(false);
  }

  void _onEnter(PointerEnterEvent details) {
    widget.onEnter?.call();
    widget.onHover?.call(true);
  }

  @override
  Widget build(BuildContext context) {
    return MouseRegion(onEnter: _onEnter, onExit: _onExit, child: widget.child);
  }
}

class HoverFeedback extends StatefulWidget {
  const HoverFeedback({super.key, this.onEnter, this.onExit});

  final VoidCallback? onEnter;
  final VoidCallback? onExit;

  @override
  State<HoverFeedback> createState() => _HoverFeedbackState();
}

class _HoverFeedbackState extends State<HoverFeedback> {
  bool _hovering = false;

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: HoverClient(
        onHover: (bool hovering) => setState(() => _hovering = hovering),
        onEnter: widget.onEnter,
        onExit: widget.onExit,
        child: Text(_hovering ? 'HOVERING' : 'not hovering'),
      ),
    );
  }
}

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/73330
  testWidgets('hitTestBehavior test - HitTestBehavior.deferToChild/opaque', (
    WidgetTester tester,
  ) async {
    bool onEnter = false;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          hitTestBehavior: HitTestBehavior.deferToChild,
          onEnter: (_) => onEnter = true,
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await tester.pump();

    // The child is null, so `onEnter` does not trigger.
    expect(onEnter, false);

    // Update to the default value `HitTestBehavior.opaque`
    await tester.pumpWidget(Center(child: MouseRegion(onEnter: (_) => onEnter = true)));

    expect(onEnter, true);
  });

  testWidgets('hitTestBehavior test - HitTestBehavior.deferToChild and non-opaque', (
    WidgetTester tester,
  ) async {
    bool onEnterRegion1 = false;
    bool onEnterRegion2 = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            SizedBox(
              width: 50.0,
              height: 50.0,
              child: MouseRegion(onEnter: (_) => onEnterRegion1 = true),
            ),
            SizedBox(
              width: 50.0,
              height: 50.0,
              child: MouseRegion(
                opaque: false,
                hitTestBehavior: HitTestBehavior.deferToChild,
                onEnter: (_) => onEnterRegion2 = true,
                child: Container(
                  color: const Color.fromARGB(0xff, 0xff, 0x10, 0x19),
                  width: 50.0,
                  height: 50.0,
                ),
              ),
            ),
          ],
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await tester.pump();

    expect(onEnterRegion2, true);
    expect(onEnterRegion1, true);
  });

  testWidgets('hitTestBehavior test - HitTestBehavior.translucent', (WidgetTester tester) async {
    bool onEnterRegion1 = false;
    bool onEnterRegion2 = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            SizedBox(
              width: 50.0,
              height: 50.0,
              child: MouseRegion(onEnter: (_) => onEnterRegion1 = true),
            ),
            SizedBox(
              width: 50.0,
              height: 50.0,
              child: MouseRegion(
                hitTestBehavior: HitTestBehavior.translucent,
                onEnter: (_) => onEnterRegion2 = true,
              ),
            ),
          ],
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await tester.pump();

    expect(onEnterRegion2, true);
    expect(onEnterRegion1, true);
  });

  testWidgets('onEnter and onExit can be triggered with mouse buttons pressed', (
    WidgetTester tester,
  ) async {
    PointerEnterEvent? enter;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: Container(
            color: const Color.fromARGB(0xff, 0xff, 0x00, 0x00),
            width: 100.0,
            height: 100.0,
          ),
          onEnter: (PointerEnterEvent details) => enter = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await gesture.down(Offset.zero); // Press the mouse button.
    await tester.pump();
    enter = null;
    exit = null;
    // Trigger the enter event.
    await gesture.moveTo(const Offset(400.0, 300.0));
    expect(enter, isNotNull);
    expect(enter!.position, equals(const Offset(400.0, 300.0)));
    expect(enter!.localPosition, equals(const Offset(50.0, 50.0)));
    expect(exit, isNull);

    // Trigger the exit event.
    await gesture.moveTo(const Offset(1.0, 1.0));
    expect(exit, isNotNull);
    expect(exit!.position, equals(const Offset(1.0, 1.0)));
    expect(exit!.localPosition, equals(const Offset(-349.0, -249.0)));
  });

  testWidgets('detects pointer enter', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: Container(
            color: const Color.fromARGB(0xff, 0xff, 0x00, 0x00),
            width: 100.0,
            height: 100.0,
          ),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await tester.pump();
    move = null;
    enter = null;
    exit = null;
    await gesture.moveTo(const Offset(400.0, 300.0));
    expect(move, isNotNull);
    expect(move!.position, equals(const Offset(400.0, 300.0)));
    expect(move!.localPosition, equals(const Offset(50.0, 50.0)));
    expect(enter, isNotNull);
    expect(enter!.position, equals(const Offset(400.0, 300.0)));
    expect(enter!.localPosition, equals(const Offset(50.0, 50.0)));
    expect(exit, isNull);
  });

  testWidgets('detects pointer exiting', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: Container(
            color: const Color.fromARGB(0xff, 0xff, 0x00, 0x00),
            width: 100.0,
            height: 100.0,
          ),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await gesture.moveTo(const Offset(400.0, 300.0));
    await tester.pump();
    move = null;
    enter = null;
    exit = null;
    await gesture.moveTo(const Offset(1.0, 1.0));
    expect(move, isNull);
    expect(enter, isNull);
    expect(exit, isNotNull);
    expect(exit!.position, equals(const Offset(1.0, 1.0)));
    expect(exit!.localPosition, equals(const Offset(-349.0, -249.0)));
  });

  testWidgets('triggers pointer enter when a mouse is connected', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    await tester.pump();

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(400, 300));
    expect(move, isNull);
    expect(enter, isNotNull);
    expect(enter!.position, equals(const Offset(400.0, 300.0)));
    expect(enter!.localPosition, equals(const Offset(50.0, 50.0)));
    expect(exit, isNull);
  });

  testWidgets('triggers pointer exit when a mouse is disconnected', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    await tester.pump();

    TestGesture? gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(400, 300));
    addTearDown(() => gesture?.removePointer);
    await tester.pump();
    move = null;
    enter = null;
    exit = null;
    await gesture.removePointer();
    gesture = null;
    expect(move, isNull);
    expect(enter, isNull);
    expect(exit, isNotNull);
    expect(exit!.position, equals(const Offset(400.0, 300.0)));
    expect(exit!.localPosition, equals(const Offset(50.0, 50.0)));
    exit = null;
    await tester.pump();
    expect(move, isNull);
    expect(enter, isNull);
    expect(exit, isNull);
  });

  testWidgets('triggers pointer enter when widget appears', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(const Center(child: SizedBox(width: 100.0, height: 100.0)));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await gesture.moveTo(const Offset(400.0, 300.0));
    await tester.pump();
    expect(enter, isNull);
    expect(move, isNull);
    expect(exit, isNull);
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    await tester.pump();
    expect(move, isNull);
    expect(enter, isNotNull);
    expect(enter!.position, equals(const Offset(400.0, 300.0)));
    expect(enter!.localPosition, equals(const Offset(50.0, 50.0)));
    expect(exit, isNull);
  });

  testWidgets("doesn't trigger pointer exit when widget disappears", (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    await gesture.moveTo(const Offset(400.0, 300.0));
    await tester.pump();
    move = null;
    enter = null;
    exit = null;
    await tester.pumpWidget(const Center(child: SizedBox(width: 100.0, height: 100.0)));
    expect(enter, isNull);
    expect(move, isNull);
    expect(exit, isNull);
  });

  testWidgets('triggers pointer enter when widget moves in', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Container(
        alignment: Alignment.topLeft,
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(401.0, 301.0));
    await tester.pump();
    expect(enter, isNull);
    expect(move, isNull);
    expect(exit, isNull);
    await tester.pumpWidget(
      Container(
        alignment: Alignment.center,
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    await tester.pump();
    expect(enter, isNotNull);
    expect(enter!.position, equals(const Offset(401.0, 301.0)));
    expect(enter!.localPosition, equals(const Offset(51.0, 51.0)));
    expect(move, isNull);
    expect(exit, isNull);
  });

  testWidgets('triggers pointer exit when widget moves out', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Container(
        alignment: Alignment.center,
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(400, 300));
    await tester.pump();
    enter = null;
    move = null;
    exit = null;
    await tester.pumpWidget(
      Container(
        alignment: Alignment.topLeft,
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    await tester.pump();
    expect(enter, isNull);
    expect(move, isNull);
    expect(exit, isNotNull);
    expect(exit!.position, equals(const Offset(400, 300)));
    expect(exit!.localPosition, equals(const Offset(50, 50)));
  });

  testWidgets('detects hover from touch devices', (WidgetTester tester) async {
    PointerEnterEvent? enter;
    PointerHoverEvent? move;
    PointerExitEvent? exit;
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: Container(
            color: const Color.fromARGB(0xff, 0xff, 0x00, 0x00),
            width: 100.0,
            height: 100.0,
          ),
          onEnter: (PointerEnterEvent details) => enter = details,
          onHover: (PointerHoverEvent details) => move = details,
          onExit: (PointerExitEvent details) => exit = details,
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture();
    await gesture.addPointer(location: Offset.zero);
    await tester.pump();
    move = null;
    enter = null;
    exit = null;
    await gesture.moveTo(const Offset(400.0, 300.0));
    expect(move, isNotNull);
    expect(move!.position, equals(const Offset(400.0, 300.0)));
    expect(move!.localPosition, equals(const Offset(50.0, 50.0)));
    expect(enter, isNull);
    expect(exit, isNull);
  });

  testWidgets('Hover works with nested listeners', (WidgetTester tester) async {
    final UniqueKey key1 = UniqueKey();
    final UniqueKey key2 = UniqueKey();
    final List<PointerEnterEvent> enter1 = <PointerEnterEvent>[];
    final List<PointerHoverEvent> move1 = <PointerHoverEvent>[];
    final List<PointerExitEvent> exit1 = <PointerExitEvent>[];
    final List<PointerEnterEvent> enter2 = <PointerEnterEvent>[];
    final List<PointerHoverEvent> move2 = <PointerHoverEvent>[];
    final List<PointerExitEvent> exit2 = <PointerExitEvent>[];
    void clearLists() {
      enter1.clear();
      move1.clear();
      exit1.clear();
      enter2.clear();
      move2.clear();
      exit2.clear();
    }

    await tester.pumpWidget(Container());
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(const Offset(400.0, 0.0));
    await tester.pump();
    await tester.pumpWidget(
      Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          MouseRegion(
            onEnter: (PointerEnterEvent details) => enter1.add(details),
            onHover: (PointerHoverEvent details) => move1.add(details),
            onExit: (PointerExitEvent details) => exit1.add(details),
            key: key1,
            child: Container(
              width: 200,
              height: 200,
              padding: const EdgeInsets.all(50.0),
              child: MouseRegion(
                key: key2,
                onEnter: (PointerEnterEvent details) => enter2.add(details),
                onHover: (PointerHoverEvent details) => move2.add(details),
                onExit: (PointerExitEvent details) => exit2.add(details),
                child: Container(),
              ),
            ),
          ),
        ],
      ),
    );
    Offset center = tester.getCenter(find.byKey(key2));
    await gesture.moveTo(center);
    await tester.pump();
    expect(move2, isNotEmpty);
    expect(enter2, isNotEmpty);
    expect(exit2, isEmpty);
    expect(move1, isNotEmpty);
    expect(move1.last.position, equals(center));
    expect(enter1, isNotEmpty);
    expect(enter1.last.position, equals(center));
    expect(exit1, isEmpty);
    clearLists();

    // Now make sure that exiting the child only triggers the child exit, not
    // the parent too.
    center = center - const Offset(75.0, 0.0);
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(move2, isEmpty);
    expect(enter2, isEmpty);
    expect(exit2, isNotEmpty);
    expect(move1, isNotEmpty);
    expect(move1.last.position, equals(center));
    expect(enter1, isEmpty);
    expect(exit1, isEmpty);
    clearLists();
  });

  testWidgets('Hover transfers between two listeners', (WidgetTester tester) async {
    final UniqueKey key1 = UniqueKey();
    final UniqueKey key2 = UniqueKey();
    final List<PointerEnterEvent> enter1 = <PointerEnterEvent>[];
    final List<PointerHoverEvent> move1 = <PointerHoverEvent>[];
    final List<PointerExitEvent> exit1 = <PointerExitEvent>[];
    final List<PointerEnterEvent> enter2 = <PointerEnterEvent>[];
    final List<PointerHoverEvent> move2 = <PointerHoverEvent>[];
    final List<PointerExitEvent> exit2 = <PointerExitEvent>[];
    void clearLists() {
      enter1.clear();
      move1.clear();
      exit1.clear();
      enter2.clear();
      move2.clear();
      exit2.clear();
    }

    await tester.pumpWidget(Container());
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(const Offset(400.0, 0.0));
    await tester.pump();
    await tester.pumpWidget(
      Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          MouseRegion(
            key: key1,
            child: const SizedBox(width: 100.0, height: 100.0),
            onEnter: (PointerEnterEvent details) => enter1.add(details),
            onHover: (PointerHoverEvent details) => move1.add(details),
            onExit: (PointerExitEvent details) => exit1.add(details),
          ),
          MouseRegion(
            key: key2,
            child: const SizedBox(width: 100.0, height: 100.0),
            onEnter: (PointerEnterEvent details) => enter2.add(details),
            onHover: (PointerHoverEvent details) => move2.add(details),
            onExit: (PointerExitEvent details) => exit2.add(details),
          ),
        ],
      ),
    );
    final Offset center1 = tester.getCenter(find.byKey(key1));
    final Offset center2 = tester.getCenter(find.byKey(key2));
    await gesture.moveTo(center1);
    await tester.pump();
    expect(move1, isNotEmpty);
    expect(move1.last.position, equals(center1));
    expect(enter1, isNotEmpty);
    expect(enter1.last.position, equals(center1));
    expect(exit1, isEmpty);
    expect(move2, isEmpty);
    expect(enter2, isEmpty);
    expect(exit2, isEmpty);
    clearLists();
    await gesture.moveTo(center2);
    await tester.pump();
    expect(move1, isEmpty);
    expect(enter1, isEmpty);
    expect(exit1, isNotEmpty);
    expect(exit1.last.position, equals(center2));
    expect(move2, isNotEmpty);
    expect(move2.last.position, equals(center2));
    expect(enter2, isNotEmpty);
    expect(enter2.last.position, equals(center2));
    expect(exit2, isEmpty);
    clearLists();
    await gesture.moveTo(const Offset(400.0, 450.0));
    await tester.pump();
    expect(move1, isEmpty);
    expect(enter1, isEmpty);
    expect(exit1, isEmpty);
    expect(move2, isEmpty);
    expect(enter2, isEmpty);
    expect(exit2, isNotEmpty);
    expect(exit2.last.position, equals(const Offset(400.0, 450.0)));
    clearLists();
    await tester.pumpWidget(Container());
    expect(move1, isEmpty);
    expect(enter1, isEmpty);
    expect(exit1, isEmpty);
    expect(move2, isEmpty);
    expect(enter2, isEmpty);
    expect(exit2, isEmpty);
  });

  testWidgets('applies mouse cursor', (WidgetTester tester) async {
    await tester.pumpWidget(
      const _Scaffold(
        topLeft: MouseRegion(
          cursor: SystemMouseCursors.text,
          child: SizedBox(width: 10, height: 10),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(100, 100));

    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    await gesture.moveTo(const Offset(5, 5));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    await gesture.moveTo(const Offset(100, 100));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('MouseRegion uses updated callbacks', (WidgetTester tester) async {
    final List<String> logs = <String>[];
    Widget hoverableContainer({
      PointerEnterEventListener? onEnter,
      PointerHoverEventListener? onHover,
      PointerExitEventListener? onExit,
    }) {
      return Container(
        alignment: Alignment.topLeft,
        child: MouseRegion(
          onEnter: onEnter,
          onHover: onHover,
          onExit: onExit,
          child: Container(
            color: const Color.fromARGB(0xff, 0xff, 0x00, 0x00),
            width: 100.0,
            height: 100.0,
          ),
        ),
      );
    }

    await tester.pumpWidget(
      hoverableContainer(
        onEnter: (PointerEnterEvent details) {
          logs.add('enter1');
        },
        onHover: (PointerHoverEvent details) {
          logs.add('hover1');
        },
        onExit: (PointerExitEvent details) {
          logs.add('exit1');
        },
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(150.0, 150.0));

    // Start outside, move inside, then move outside
    await gesture.moveTo(const Offset(150.0, 150.0));
    await tester.pump();
    expect(logs, isEmpty);
    logs.clear();
    await gesture.moveTo(const Offset(50.0, 50.0));
    await tester.pump();
    await gesture.moveTo(const Offset(150.0, 150.0));
    await tester.pump();
    expect(logs, <String>['enter1', 'hover1', 'exit1']);
    logs.clear();

    // Same tests but with updated callbacks
    await tester.pumpWidget(
      hoverableContainer(
        onEnter: (PointerEnterEvent details) => logs.add('enter2'),
        onHover: (PointerHoverEvent details) => logs.add('hover2'),
        onExit: (PointerExitEvent details) => logs.add('exit2'),
      ),
    );
    await gesture.moveTo(const Offset(150.0, 150.0));
    await tester.pump();
    await gesture.moveTo(const Offset(50.0, 50.0));
    await tester.pump();
    await gesture.moveTo(const Offset(150.0, 150.0));
    await tester.pump();
    expect(logs, <String>['enter2', 'hover2', 'exit2']);
  });

  testWidgets('needsCompositing set when parent class needsCompositing is set', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MouseRegion(
        onEnter: (PointerEnterEvent _) {},
        child: const RepaintBoundary(child: Placeholder()),
      ),
    );

    RenderMouseRegion listener = tester.renderObject(find.byType(MouseRegion).first);
    expect(listener.needsCompositing, isTrue);

    await tester.pumpWidget(
      MouseRegion(onEnter: (PointerEnterEvent _) {}, child: const Placeholder()),
    );

    listener = tester.renderObject(find.byType(MouseRegion).first);
    expect(listener.needsCompositing, isFalse);
  });

  testWidgets('works with transform', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/31986.
    final Key key = UniqueKey();
    const double scaleFactor = 2.0;
    const double localWidth = 150.0;
    const double localHeight = 100.0;
    final List<PointerEvent> events = <PointerEvent>[];

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Transform.scale(
            scale: scaleFactor,
            child: MouseRegion(
              onEnter: (PointerEnterEvent event) {
                events.add(event);
              },
              onHover: (PointerHoverEvent event) {
                events.add(event);
              },
              onExit: (PointerExitEvent event) {
                events.add(event);
              },
              child: Container(
                key: key,
                color: Colors.blue,
                height: localHeight,
                width: localWidth,
                child: const Text('Hi'),
              ),
            ),
          ),
        ),
      ),
    );

    final Offset topLeft = tester.getTopLeft(find.byKey(key));
    final Offset topRight = tester.getTopRight(find.byKey(key));
    final Offset bottomLeft = tester.getBottomLeft(find.byKey(key));
    expect(topRight.dx - topLeft.dx, scaleFactor * localWidth);
    expect(bottomLeft.dy - topLeft.dy, scaleFactor * localHeight);

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(topLeft - const Offset(1, 1));
    await tester.pump();
    expect(events, isEmpty);

    await gesture.moveTo(topLeft + const Offset(1, 1));
    await tester.pump();
    expect(events, hasLength(2));
    expect(events.first, isA<PointerEnterEvent>());
    expect(events.last, isA<PointerHoverEvent>());
    events.clear();

    await gesture.moveTo(bottomLeft + const Offset(1, -1));
    await tester.pump();
    expect(events.single, isA<PointerHoverEvent>());
    expect(events.single.delta, const Offset(0.0, scaleFactor * localHeight - 2));
    events.clear();

    await gesture.moveTo(bottomLeft + const Offset(1, 1));
    await tester.pump();
    expect(events.single, isA<PointerExitEvent>());
    events.clear();
  });

  testWidgets('needsCompositing is always false', (WidgetTester tester) async {
    // Pretend that we have a mouse connected.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();

    await tester.pumpWidget(Transform.scale(scale: 2.0, child: const MouseRegion(opaque: false)));
    final RenderMouseRegion mouseRegion = tester.renderObject(find.byType(MouseRegion));
    expect(mouseRegion.needsCompositing, isFalse);
    // No TransformLayer for `Transform.scale` is added because composting is
    // not required and therefore the transform is executed on the canvas
    // directly. (One TransformLayer is always present for the root
    // transform.)
    expect(tester.layers.whereType<TransformLayer>(), hasLength(1));

    // Test that needsCompositing stays false with callback change
    await tester.pumpWidget(
      Transform.scale(
        scale: 2.0,
        child: MouseRegion(opaque: false, onHover: (PointerHoverEvent _) {}),
      ),
    );
    expect(mouseRegion.needsCompositing, isFalse);
    // If compositing was required, a dedicated TransformLayer for
    // `Transform.scale` would be added.
    expect(tester.layers.whereType<TransformLayer>(), hasLength(1));
  });

  testWidgets("Callbacks aren't called during build", (WidgetTester tester) async {
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);

    int numEntrances = 0;
    int numExits = 0;

    await tester.pumpWidget(
      Center(
        child: HoverFeedback(
          onEnter: () {
            numEntrances += 1;
          },
          onExit: () {
            numExits += 1;
          },
        ),
      ),
    );

    await gesture.moveTo(tester.getCenter(find.byType(Text)));
    await tester.pumpAndSettle();
    expect(numEntrances, equals(1));
    expect(numExits, equals(0));
    expect(find.text('HOVERING'), findsOneWidget);

    await tester.pumpWidget(Container());
    await tester.pump();
    expect(numEntrances, equals(1));
    expect(numExits, equals(0));

    await tester.pumpWidget(
      Center(
        child: HoverFeedback(
          onEnter: () {
            numEntrances += 1;
          },
          onExit: () {
            numExits += 1;
          },
        ),
      ),
    );
    await tester.pump();
    expect(numEntrances, equals(2));
    expect(numExits, equals(0));
  });

  testWidgets("MouseRegion activate/deactivate don't duplicate annotations", (
    WidgetTester tester,
  ) async {
    final GlobalKey feedbackKey = GlobalKey();
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();

    int numEntrances = 0;
    int numExits = 0;

    await tester.pumpWidget(
      Center(
        child: HoverFeedback(
          key: feedbackKey,
          onEnter: () {
            numEntrances += 1;
          },
          onExit: () {
            numExits += 1;
          },
        ),
      ),
    );

    await gesture.moveTo(tester.getCenter(find.byType(Text)));
    await tester.pumpAndSettle();
    expect(numEntrances, equals(1));
    expect(numExits, equals(0));
    expect(find.text('HOVERING'), findsOneWidget);

    await tester.pumpWidget(
      Center(
        child: HoverFeedback(
          key: feedbackKey,
          onEnter: () {
            numEntrances += 1;
          },
          onExit: () {
            numExits += 1;
          },
        ),
      ),
    );
    await tester.pump();
    expect(numEntrances, equals(1));
    expect(numExits, equals(0));
    await tester.pumpWidget(Container());
    await tester.pump();
    expect(numEntrances, equals(1));
    expect(numExits, equals(0));
  });

  testWidgets('Exit event when unplugging mouse should have a position', (
    WidgetTester tester,
  ) async {
    final List<PointerEnterEvent> enter = <PointerEnterEvent>[];
    final List<PointerHoverEvent> hover = <PointerHoverEvent>[];
    final List<PointerExitEvent> exit = <PointerExitEvent>[];

    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          onEnter: (PointerEnterEvent e) => enter.add(e),
          onHover: (PointerHoverEvent e) => hover.add(e),
          onExit: (PointerExitEvent e) => exit.add(e),
          child: const SizedBox(height: 100.0, width: 100.0),
        ),
      ),
    );

    // Plug-in a mouse and move it to the center of the container.
    TestGesture? gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: Offset.zero);
    addTearDown(() => gesture?.removePointer());
    await tester.pumpAndSettle();
    await gesture.moveTo(tester.getCenter(find.byType(SizedBox)));

    expect(enter.length, 1);
    expect(enter.single.position, const Offset(400.0, 300.0));
    expect(hover.length, 1);
    expect(hover.single.position, const Offset(400.0, 300.0));
    expect(exit.length, 0);

    enter.clear();
    hover.clear();
    exit.clear();

    // Unplug the mouse.
    await gesture.removePointer();
    gesture = null;
    await tester.pumpAndSettle();

    expect(enter.length, 0);
    expect(hover.length, 0);
    expect(exit.length, 1);
    expect(exit.single.position, const Offset(400.0, 300.0));
    expect(exit.single.delta, Offset.zero);
  });

  testWidgets('detects pointer enter with closure arguments', (WidgetTester tester) async {
    await tester.pumpWidget(const _HoverClientWithClosures());
    expect(find.text('not hovering'), findsOneWidget);

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    // Move to a position out of MouseRegion
    await gesture.moveTo(tester.getBottomRight(find.byType(MouseRegion)) + const Offset(10, -10));
    await tester.pumpAndSettle();
    expect(find.text('not hovering'), findsOneWidget);

    // Move into MouseRegion
    await gesture.moveBy(const Offset(-20, 0));
    await tester.pumpAndSettle();
    expect(find.text('HOVERING'), findsOneWidget);
  });

  testWidgets('MouseRegion paints child once and only once when MouseRegion is inactive', (
    WidgetTester tester,
  ) async {
    int paintCount = 0;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          onEnter: (PointerEnterEvent e) {},
          child: CustomPaint(
            painter: _DelegatedPainter(
              onPaint: () {
                paintCount += 1;
              },
            ),
            child: const Text('123'),
          ),
        ),
      ),
    );

    expect(paintCount, 1);
  });

  testWidgets('MouseRegion paints child once and only once when MouseRegion is active', (
    WidgetTester tester,
  ) async {
    int paintCount = 0;

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          onEnter: (PointerEnterEvent e) {},
          child: CustomPaint(
            painter: _DelegatedPainter(
              onPaint: () {
                paintCount += 1;
              },
            ),
            child: const Text('123'),
          ),
        ),
      ),
    );

    expect(paintCount, 1);
  });

  testWidgets('A MouseRegion mounted under the pointer should take effect in the next postframe', (
    WidgetTester tester,
  ) async {
    bool hovered = false;

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(5, 5));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return _ColumnContainer(
            children: <Widget>[Text(hovered ? 'hover outer' : 'unhover outer')],
          );
        },
      ),
    );

    expect(find.text('unhover outer'), findsOneWidget);

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return _ColumnContainer(
            children: <Widget>[
              HoverClient(
                onHover: (bool value) {
                  setState(() {
                    hovered = value;
                  });
                },
                child: Text(hovered ? 'hover inner' : 'unhover inner'),
              ),
              Text(hovered ? 'hover outer' : 'unhover outer'),
            ],
          );
        },
      ),
    );

    expect(find.text('unhover outer'), findsOneWidget);
    expect(find.text('unhover inner'), findsOneWidget);

    await tester.pump();

    expect(find.text('hover outer'), findsOneWidget);
    expect(find.text('hover inner'), findsOneWidget);
    expect(tester.binding.hasScheduledFrame, isFalse);
  });

  testWidgets('A MouseRegion unmounted under the pointer should not trigger state change', (
    WidgetTester tester,
  ) async {
    bool hovered = true;

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(5, 5));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return _ColumnContainer(
            children: <Widget>[
              HoverClient(
                onHover: (bool value) {
                  setState(() {
                    hovered = value;
                  });
                },
                child: Text(hovered ? 'hover inner' : 'unhover inner'),
              ),
              Text(hovered ? 'hover outer' : 'unhover outer'),
            ],
          );
        },
      ),
    );

    expect(find.text('hover outer'), findsOneWidget);
    expect(find.text('hover inner'), findsOneWidget);
    expect(tester.binding.hasScheduledFrame, isTrue);

    await tester.pump();
    expect(find.text('hover outer'), findsOneWidget);
    expect(find.text('hover inner'), findsOneWidget);
    expect(tester.binding.hasScheduledFrame, isFalse);

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return _ColumnContainer(
            children: <Widget>[Text(hovered ? 'hover outer' : 'unhover outer')],
          );
        },
      ),
    );

    expect(find.text('hover outer'), findsOneWidget);
    expect(tester.binding.hasScheduledFrame, isFalse);
  });

  testWidgets('A MouseRegion moved into the mouse should take effect in the next postframe', (
    WidgetTester tester,
  ) async {
    bool hovered = false;
    final List<bool> logHovered = <bool>[];
    bool moved = false;
    late StateSetter mySetState;

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(5, 5));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          mySetState = setState;
          return _ColumnContainer(
            children: <Widget>[
              Container(
                height: 100,
                width: 10,
                alignment: moved ? Alignment.topLeft : Alignment.bottomLeft,
                child: SizedBox(
                  height: 10,
                  width: 10,
                  child: HoverClient(
                    onHover: (bool value) {
                      setState(() {
                        hovered = value;
                      });
                      logHovered.add(value);
                    },
                    child: Text(hovered ? 'hover inner' : 'unhover inner'),
                  ),
                ),
              ),
              Text(hovered ? 'hover outer' : 'unhover outer'),
            ],
          );
        },
      ),
    );

    expect(find.text('unhover inner'), findsOneWidget);
    expect(find.text('unhover outer'), findsOneWidget);
    expect(logHovered, isEmpty);
    expect(tester.binding.hasScheduledFrame, isFalse);

    mySetState(() {
      moved = true;
    });
    // The first frame is for the widget movement to take effect.
    await tester.pump();
    expect(find.text('unhover inner'), findsOneWidget);
    expect(find.text('unhover outer'), findsOneWidget);
    expect(logHovered, <bool>[true]);
    logHovered.clear();

    // The second frame is for the mouse hover to take effect.
    await tester.pump();
    expect(find.text('hover inner'), findsOneWidget);
    expect(find.text('hover outer'), findsOneWidget);
    expect(logHovered, isEmpty);
    expect(tester.binding.hasScheduledFrame, isFalse);
  });

  group('MouseRegion respects opacity:', () {
    // A widget that contains 3 MouseRegions:
    //                           y
    //   ——————————————————————  0
    //   | ———————————     A  |  20
    //   | | B       |        |
    //   | |     ———————————  |  50
    //   | |     |       C |  |
    //   | ——————|         |  |  100
    //   |       |         |  |
    //   |       ———————————  |  130
    //   ——————————————————————  150
    // x 0 20   50  100   130 150
    Widget tripleRegions({bool? opaqueC, required void Function(String) addLog}) {
      // Same as MouseRegion, but when opaque is null, use the default value.
      Widget mouseRegionWithOptionalOpaque({
        void Function(PointerEnterEvent e)? onEnter,
        void Function(PointerHoverEvent e)? onHover,
        void Function(PointerExitEvent e)? onExit,
        Widget? child,
        bool? opaque,
      }) {
        if (opaque == null) {
          return MouseRegion(onEnter: onEnter, onHover: onHover, onExit: onExit, child: child);
        }
        return MouseRegion(
          onEnter: onEnter,
          onHover: onHover,
          onExit: onExit,
          opaque: opaque,
          child: child,
        );
      }

      return Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: MouseRegion(
            onEnter: (PointerEnterEvent e) {
              addLog('enterA');
            },
            onHover: (PointerHoverEvent e) {
              addLog('hoverA');
            },
            onExit: (PointerExitEvent e) {
              addLog('exitA');
            },
            child: SizedBox(
              width: 150,
              height: 150,
              child: Stack(
                children: <Widget>[
                  Positioned(
                    left: 20,
                    top: 20,
                    width: 80,
                    height: 80,
                    child: MouseRegion(
                      onEnter: (PointerEnterEvent e) {
                        addLog('enterB');
                      },
                      onHover: (PointerHoverEvent e) {
                        addLog('hoverB');
                      },
                      onExit: (PointerExitEvent e) {
                        addLog('exitB');
                      },
                    ),
                  ),
                  Positioned(
                    left: 50,
                    top: 50,
                    width: 80,
                    height: 80,
                    child: mouseRegionWithOptionalOpaque(
                      opaque: opaqueC,
                      onEnter: (PointerEnterEvent e) {
                        addLog('enterC');
                      },
                      onHover: (PointerHoverEvent e) {
                        addLog('hoverC');
                      },
                      onExit: (PointerExitEvent e) {
                        addLog('exitC');
                      },
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      );
    }

    testWidgets('a transparent one should allow MouseRegions behind it to receive pointers', (
      WidgetTester tester,
    ) async {
      final List<String> logs = <String>[];
      await tester.pumpWidget(tripleRegions(opaqueC: false, addLog: (String log) => logs.add(log)));

      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await tester.pumpAndSettle();

      // Move to the overlapping area.
      await gesture.moveTo(const Offset(75, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['enterA', 'enterB', 'enterC', 'hoverC', 'hoverB', 'hoverA']);
      logs.clear();

      // Move to the B only area.
      await gesture.moveTo(const Offset(25, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['exitC', 'hoverB', 'hoverA']);
      logs.clear();

      // Move back to the overlapping area.
      await gesture.moveTo(const Offset(75, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['enterC', 'hoverC', 'hoverB', 'hoverA']);
      logs.clear();

      // Move to the C only area.
      await gesture.moveTo(const Offset(125, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['exitB', 'hoverC', 'hoverA']);
      logs.clear();

      // Move back to the overlapping area.
      await gesture.moveTo(const Offset(75, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['enterB', 'hoverC', 'hoverB', 'hoverA']);
      logs.clear();

      // Move out.
      await gesture.moveTo(const Offset(160, 160));
      await tester.pumpAndSettle();
      expect(logs, <String>['exitC', 'exitB', 'exitA']);
    });

    testWidgets('an opaque one should prevent MouseRegions behind it receiving pointers', (
      WidgetTester tester,
    ) async {
      final List<String> logs = <String>[];
      await tester.pumpWidget(tripleRegions(opaqueC: true, addLog: (String log) => logs.add(log)));

      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await tester.pumpAndSettle();

      // Move to the overlapping area.
      await gesture.moveTo(const Offset(75, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['enterA', 'enterC', 'hoverC', 'hoverA']);
      logs.clear();

      // Move to the B only area.
      await gesture.moveTo(const Offset(25, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['exitC', 'enterB', 'hoverB', 'hoverA']);
      logs.clear();

      // Move back to the overlapping area.
      await gesture.moveTo(const Offset(75, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['exitB', 'enterC', 'hoverC', 'hoverA']);
      logs.clear();

      // Move to the C only area.
      await gesture.moveTo(const Offset(125, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['hoverC', 'hoverA']);
      logs.clear();

      // Move back to the overlapping area.
      await gesture.moveTo(const Offset(75, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['hoverC', 'hoverA']);
      logs.clear();

      // Move out.
      await gesture.moveTo(const Offset(160, 160));
      await tester.pumpAndSettle();
      expect(logs, <String>['exitC', 'exitA']);
    });

    testWidgets('opaque should default to true', (WidgetTester tester) async {
      final List<String> logs = <String>[];
      await tester.pumpWidget(tripleRegions(addLog: (String log) => logs.add(log)));

      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await tester.pumpAndSettle();

      // Move to the overlapping area.
      await gesture.moveTo(const Offset(75, 75));
      await tester.pumpAndSettle();
      expect(logs, <String>['enterA', 'enterC', 'hoverC', 'hoverA']);
      logs.clear();

      // Move out.
      await gesture.moveTo(const Offset(160, 160));
      await tester.pumpAndSettle();
      expect(logs, <String>['exitC', 'exitA']);
    });
  });

  testWidgets('an empty opaque MouseRegion is effective', (WidgetTester tester) async {
    bool bottomRegionIsHovered = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Align(
              alignment: Alignment.topLeft,
              child: MouseRegion(
                onEnter: (_) {
                  bottomRegionIsHovered = true;
                },
                onHover: (_) {
                  bottomRegionIsHovered = true;
                },
                onExit: (_) {
                  bottomRegionIsHovered = true;
                },
                child: const SizedBox(width: 10, height: 10),
              ),
            ),
            const MouseRegion(),
          ],
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(20, 20));

    await gesture.moveTo(const Offset(5, 5));
    await tester.pump();
    await gesture.moveTo(const Offset(20, 20));
    await tester.pump();
    expect(bottomRegionIsHovered, isFalse);
  });

  testWidgets("Changing MouseRegion's callbacks is effective and doesn't repaint", (
    WidgetTester tester,
  ) async {
    final List<String> logs = <String>[];
    const Key key = ValueKey<int>(1);

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(20, 20));

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            onEnter: (_) {
              logs.add('enter1');
            },
            onHover: (_) {
              logs.add('hover1');
            },
            onExit: (_) {
              logs.add('exit1');
            },
            child: CustomPaint(
              painter: _DelegatedPainter(
                onPaint: () {
                  logs.add('paint');
                },
                key: key,
              ),
            ),
          ),
        ),
      ),
    );
    expect(logs, <String>['paint']);
    logs.clear();

    await gesture.moveTo(const Offset(5, 5));
    expect(logs, <String>['enter1', 'hover1']);
    logs.clear();

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            onEnter: (_) {
              logs.add('enter2');
            },
            onHover: (_) {
              logs.add('hover2');
            },
            onExit: (_) {
              logs.add('exit2');
            },
            child: CustomPaint(
              painter: _DelegatedPainter(
                onPaint: () {
                  logs.add('paint');
                },
                key: key,
              ),
            ),
          ),
        ),
      ),
    );
    expect(logs, isEmpty);

    await gesture.moveTo(const Offset(6, 6));
    expect(logs, <String>['hover2']);
    logs.clear();

    // Compare: It repaints if the MouseRegion is deactivated.
    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            opaque: false,
            child: CustomPaint(
              painter: _DelegatedPainter(
                onPaint: () {
                  logs.add('paint');
                },
                key: key,
              ),
            ),
          ),
        ),
      ),
    );
    expect(logs, <String>['paint']);
  });

  testWidgets('Changing MouseRegion.opaque is effective and repaints', (WidgetTester tester) async {
    final List<String> logs = <String>[];

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(5, 5));

    void handleHover(PointerHoverEvent _) {}
    void handlePaintChild() {
      logs.add('paint');
    }

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            onHover: handleHover,
            child: CustomPaint(painter: _DelegatedPainter(onPaint: handlePaintChild)),
          ),
        ),
        background: MouseRegion(
          onEnter: (_) {
            logs.add('hover-enter');
          },
        ),
      ),
    );
    expect(logs, <String>['paint']);
    logs.clear();

    expect(logs, isEmpty);
    logs.clear();

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            opaque: false,
            // Dummy callback so that MouseRegion stays affective after opaque
            // turns false.
            onHover: handleHover,
            child: CustomPaint(painter: _DelegatedPainter(onPaint: handlePaintChild)),
          ),
        ),
        background: MouseRegion(
          onEnter: (_) {
            logs.add('hover-enter');
          },
        ),
      ),
    );

    expect(logs, <String>['paint', 'hover-enter']);
  });

  testWidgets('Changing MouseRegion.cursor is effective and repaints', (WidgetTester tester) async {
    final List<String> logPaints = <String>[];
    final List<String> logEnters = <String>[];

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(100, 100));

    void onPaintChild() {
      logPaints.add('paint');
    }

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            onEnter: (_) {
              logEnters.add('enter');
            },
            child: CustomPaint(painter: _DelegatedPainter(onPaint: onPaintChild)),
          ),
        ),
      ),
    );
    await gesture.moveTo(const Offset(5, 5));

    expect(logPaints, <String>['paint']);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
    expect(logEnters, <String>['enter']);
    logPaints.clear();
    logEnters.clear();

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            cursor: SystemMouseCursors.text,
            onEnter: (_) {
              logEnters.add('enter');
            },
            child: CustomPaint(painter: _DelegatedPainter(onPaint: onPaintChild)),
          ),
        ),
      ),
    );

    expect(logPaints, <String>['paint']);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    expect(logEnters, isEmpty);
    logPaints.clear();
    logEnters.clear();
  });

  testWidgets('Changing whether MouseRegion.cursor is null is effective and repaints', (
    WidgetTester tester,
  ) async {
    final List<String> logEnters = <String>[];
    final List<String> logPaints = <String>[];

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(100, 100));

    void onPaintChild() {
      logPaints.add('paint');
    }

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: MouseRegion(
              cursor: SystemMouseCursors.text,
              onEnter: (_) {
                logEnters.add('enter');
              },
              child: CustomPaint(painter: _DelegatedPainter(onPaint: onPaintChild)),
            ),
          ),
        ),
      ),
    );
    await gesture.moveTo(const Offset(5, 5));

    expect(logPaints, <String>['paint']);
    expect(logEnters, <String>['enter']);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    logPaints.clear();
    logEnters.clear();

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: MouseRegion(
              onEnter: (_) {
                logEnters.add('enter');
              },
              child: CustomPaint(painter: _DelegatedPainter(onPaint: onPaintChild)),
            ),
          ),
        ),
      ),
    );

    expect(logPaints, <String>['paint']);
    expect(logEnters, isEmpty);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
    logPaints.clear();
    logEnters.clear();

    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          height: 10,
          width: 10,
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: MouseRegion(
              cursor: SystemMouseCursors.text,
              child: CustomPaint(painter: _DelegatedPainter(onPaint: onPaintChild)),
            ),
          ),
        ),
      ),
    );

    expect(logPaints, <String>['paint']);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    expect(logEnters, isEmpty);
    logPaints.clear();
    logEnters.clear();
  });

  testWidgets('Does not trigger side effects during a reparent', (WidgetTester tester) async {
    final List<String> logEnters = <String>[];
    final List<String> logExits = <String>[];
    final List<String> logCursors = <String>[];

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(100, 100));
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.mouseCursor, (
      _,
    ) async {
      logCursors.add('cursor');
      return null;
    });

    final GlobalKey key = GlobalKey();

    // Pump a row of 2 SizedBox's, each taking 50px of width.
    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          width: 100,
          height: 50,
          child: Row(
            children: <Widget>[
              SizedBox(
                width: 50,
                height: 50,
                child: MouseRegion(
                  key: key,
                  onEnter: (_) {
                    logEnters.add('enter');
                  },
                  onExit: (_) {
                    logEnters.add('enter');
                  },
                  cursor: SystemMouseCursors.click,
                ),
              ),
              const SizedBox(width: 50, height: 50),
            ],
          ),
        ),
      ),
    );

    // Move to the mouse region inside the first box.
    await gesture.moveTo(const Offset(40, 5));

    expect(logEnters, <String>['enter']);
    expect(logExits, isEmpty);
    expect(logCursors, isNotEmpty);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
    logEnters.clear();
    logExits.clear();
    logCursors.clear();

    // Move MouseRegion to the second box while resizing them so that the
    // mouse is still on the MouseRegion
    await tester.pumpWidget(
      _Scaffold(
        topLeft: SizedBox(
          width: 100,
          height: 50,
          child: Row(
            children: <Widget>[
              const SizedBox(width: 30, height: 50),
              SizedBox(
                width: 70,
                height: 50,
                child: MouseRegion(
                  key: key,
                  onEnter: (_) {
                    logEnters.add('enter');
                  },
                  onExit: (_) {
                    logEnters.add('enter');
                  },
                  cursor: SystemMouseCursors.click,
                ),
              ),
            ],
          ),
        ),
      ),
    );

    expect(logEnters, isEmpty);
    expect(logExits, isEmpty);
    expect(logCursors, isEmpty);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  testWidgets("RenderMouseRegion's debugFillProperties when default", (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    final RenderMouseRegion renderMouseRegion = RenderMouseRegion();
    addTearDown(renderMouseRegion.dispose);

    renderMouseRegion.debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'parentData: MISSING',
      'constraints: MISSING',
      'size: MISSING',
      'behavior: opaque',
      'listeners: <none>',
    ]);
  });

  testWidgets("RenderMouseRegion's debugFillProperties when full", (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    final RenderErrorBox renderErrorBox = RenderErrorBox();
    addTearDown(renderErrorBox.dispose);

    final RenderMouseRegion renderMouseRegion = RenderMouseRegion(
      onEnter: (PointerEnterEvent event) {},
      onExit: (PointerExitEvent event) {},
      onHover: (PointerHoverEvent event) {},
      cursor: SystemMouseCursors.click,
      validForMouseTracker: false,
      child: renderErrorBox,
    );
    addTearDown(renderMouseRegion.dispose);

    renderMouseRegion.debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'parentData: MISSING',
      'constraints: MISSING',
      'size: MISSING',
      'behavior: opaque',
      'listeners: enter, hover, exit',
      'cursor: SystemMouseCursor(click)',
      'invalid for MouseTracker',
    ]);
  });

  testWidgets('No new frames are scheduled when mouse moves without triggering callbacks', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Center(
        child: MouseRegion(
          child: const SizedBox(width: 100.0, height: 100.0),
          onEnter: (PointerEnterEvent details) {},
          onHover: (PointerHoverEvent details) {},
          onExit: (PointerExitEvent details) {},
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: const Offset(400.0, 300.0));
    await tester.pumpAndSettle();
    await gesture.moveBy(const Offset(10.0, 10.0));
    expect(tester.binding.hasScheduledFrame, isFalse);
  });

  // Regression test for https://github.com/flutter/flutter/issues/67044
  testWidgets('Handle mouse events should ignore the detached MouseTrackerAnnotation', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Draggable<int>(
            feedback: Container(width: 20, height: 20, color: Colors.blue),
            childWhenDragging: Container(width: 20, height: 20, color: Colors.yellow),
            child: ElevatedButton(child: const Text('Drag me'), onPressed: () {}),
          ),
        ),
      ),
    );

    // Long press the button with mouse.
    final Offset textFieldPos = tester.getCenter(find.byType(Text));
    final TestGesture gesture = await tester.startGesture(
      textFieldPos,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump(const Duration(seconds: 2));
    await tester.pumpAndSettle();

    // Drag the Draggable Widget will replace the child with [childWhenDragging].
    await gesture.moveBy(const Offset(10.0, 10.0));
    await tester.pump(); // Trigger detach the button.

    // Continue drag mouse should not trigger any assert.
    await gesture.moveBy(const Offset(10.0, 10.0));

    // Dispose gesture
    await gesture.cancel();
    expect(tester.takeException(), isNull);
  });

  testWidgets('stylus input works', (WidgetTester tester) async {
    bool onEnter = false;
    bool onExit = false;
    bool onHover = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: MouseRegion(
            onEnter: (_) => onEnter = true,
            onExit: (_) => onExit = true,
            onHover: (_) => onHover = true,
            child: const SizedBox(width: 10.0, height: 10.0),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.stylus);
    await gesture.addPointer(location: const Offset(20.0, 20.0));
    await tester.pump();

    expect(onEnter, false);
    expect(onHover, false);
    expect(onExit, false);

    await gesture.moveTo(const Offset(5.0, 5.0));
    await tester.pump();

    expect(onEnter, true);
    expect(onHover, true);
    expect(onExit, false);

    await gesture.moveTo(const Offset(20.0, 20.0));
    await tester.pump();

    expect(onEnter, true);
    expect(onHover, true);
    expect(onExit, true);
  });
}

// Render widget `topLeft` at the top-left corner, stacking on top of the widget
// `background`.
class _Scaffold extends StatelessWidget {
  const _Scaffold({this.topLeft, this.background});

  final Widget? topLeft;
  final Widget? background;

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Stack(
        children: <Widget>[
          if (background != null) background!,
          Align(alignment: Alignment.topLeft, child: topLeft),
        ],
      ),
    );
  }
}

class _DelegatedPainter extends CustomPainter {
  _DelegatedPainter({this.key, required this.onPaint});
  final Key? key;
  final VoidCallback onPaint;

  @override
  void paint(Canvas canvas, Size size) {
    onPaint();
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) =>
      !(oldDelegate is _DelegatedPainter && key == oldDelegate.key);
}

class _HoverClientWithClosures extends StatefulWidget {
  const _HoverClientWithClosures();

  @override
  _HoverClientWithClosuresState createState() => _HoverClientWithClosuresState();
}

class _HoverClientWithClosuresState extends State<_HoverClientWithClosures> {
  bool _hovering = false;

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: MouseRegion(
        onEnter: (PointerEnterEvent _) {
          setState(() {
            _hovering = true;
          });
        },
        onExit: (PointerExitEvent _) {
          setState(() {
            _hovering = false;
          });
        },
        child: Text(_hovering ? 'HOVERING' : 'not hovering'),
      ),
    );
  }
}

// A column that aligns to the top left.
class _ColumnContainer extends StatelessWidget {
  const _ColumnContainer({required this.children});

  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: children),
    );
  }
}
