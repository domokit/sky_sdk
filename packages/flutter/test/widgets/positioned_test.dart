// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Positioned constructors', (WidgetTester tester) async {
    final Widget child = Container();
    final Positioned a = Positioned(
      left: 101.0,
      right: 201.0,
      top: 301.0,
      bottom: 401.0,
      child: child,
    );
    expect(a.left, 101.0);
    expect(a.right, 201.0);
    expect(a.top, 301.0);
    expect(a.bottom, 401.0);
    expect(a.width, null);
    expect(a.height, null);
    final Positioned b = Positioned.fromRect(
      rect: const Rect.fromLTRB(102.0, 302.0, 202.0, 502.0),
      child: child,
    );
    expect(b.left, 102.0);
    expect(b.right, null);
    expect(b.top, 302.0);
    expect(b.bottom, null);
    expect(b.width, 100.0);
    expect(b.height, 200.0);
    final Positioned c = Positioned.fromRelativeRect(
      rect: const RelativeRect.fromLTRB(103.0, 303.0, 203.0, 403.0),
      child: child,
    );
    expect(c.left, 103.0);
    expect(c.right, 203.0);
    expect(c.top, 303.0);
    expect(c.bottom, 403.0);
    expect(c.width, null);
    expect(c.height, null);
  });

  testWidgets('Can animate position data', (WidgetTester tester) async {
    final RelativeRectTween rect = RelativeRectTween(
      begin: RelativeRect.fromRect(
        const Rect.fromLTRB(10.0, 20.0, 20.0, 30.0),
        const Rect.fromLTRB(0.0, 10.0, 100.0, 110.0),
      ),
      end: RelativeRect.fromRect(
        const Rect.fromLTRB(80.0, 90.0, 90.0, 100.0),
        const Rect.fromLTRB(0.0, 10.0, 100.0, 110.0),
      ),
    );
    final AnimationController controller = AnimationController(
      duration: const Duration(seconds: 10),
      vsync: tester,
    );
    addTearDown(controller.dispose);
    final List<Size> sizes = <Size>[];
    final List<Offset> positions = <Offset>[];
    final GlobalKey key = GlobalKey();

    void recordMetrics() {
      final RenderBox box = key.currentContext!.findRenderObject()! as RenderBox;
      final BoxParentData boxParentData = box.parentData! as BoxParentData;
      sizes.add(box.size);
      positions.add(boxParentData.offset);
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            height: 100.0,
            width: 100.0,
            child: Stack(
              children: <Widget>[
                PositionedTransition(rect: rect.animate(controller), child: Container(key: key)),
              ],
            ),
          ),
        ),
      ),
    ); // t=0
    recordMetrics();
    final Completer<void> completer = Completer<void>();
    controller.forward().whenComplete(completer.complete);
    expect(completer.isCompleted, isFalse);
    await tester.pump(); // t=0 again
    expect(completer.isCompleted, isFalse);
    recordMetrics();
    await tester.pump(const Duration(seconds: 1)); // t=1
    expect(completer.isCompleted, isFalse);
    recordMetrics();
    await tester.pump(const Duration(seconds: 1)); // t=2
    expect(completer.isCompleted, isFalse);
    recordMetrics();
    await tester.pump(const Duration(seconds: 3)); // t=5
    expect(completer.isCompleted, isFalse);
    recordMetrics();
    await tester.pump(const Duration(seconds: 5)); // t=10
    expect(completer.isCompleted, isFalse);
    recordMetrics();

    expect(
      sizes,
      equals(<Size>[
        const Size(10.0, 10.0),
        const Size(10.0, 10.0),
        const Size(10.0, 10.0),
        const Size(10.0, 10.0),
        const Size(10.0, 10.0),
        const Size(10.0, 10.0),
      ]),
    );
    expect(
      positions,
      equals(<Offset>[
        const Offset(10.0, 10.0),
        const Offset(10.0, 10.0),
        const Offset(17.0, 17.0),
        const Offset(24.0, 24.0),
        const Offset(45.0, 45.0),
        const Offset(80.0, 80.0),
      ]),
    );

    controller.stop(canceled: false);
    await tester.pump();
    expect(completer.isCompleted, isTrue);
  });
}
