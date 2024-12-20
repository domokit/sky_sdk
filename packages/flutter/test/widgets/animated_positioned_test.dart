// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedPositioned.fromRect control test', (WidgetTester tester) async {
    final AnimatedPositioned positioned = AnimatedPositioned.fromRect(
      rect: const Rect.fromLTWH(7.0, 5.0, 12.0, 16.0),
      duration: const Duration(milliseconds: 200),
      child: Container(),
    );

    expect(positioned.left, equals(7.0));
    expect(positioned.top, equals(5.0));
    expect(positioned.width, equals(12.0));
    expect(positioned.height, equals(16.0));
    expect(positioned, hasOneLineDescription);
  });

  testWidgets('AnimatedPositioned - basics (VISUAL)', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          AnimatedPositioned(
            left: 50.0,
            top: 30.0,
            width: 70.0,
            height: 110.0,
            duration: const Duration(seconds: 2),
            child: Container(key: key),
          ),
        ],
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(50.0 + 70.0 / 2.0, 30.0 + 110.0 / 2.0)),
    );

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(50.0 + 70.0 / 2.0, 30.0 + 110.0 / 2.0)),
    );

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          AnimatedPositioned(
            left: 37.0,
            top: 31.0,
            width: 59.0,
            height: 71.0,
            duration: const Duration(seconds: 2),
            child: Container(key: key),
          ),
        ],
      ),
    );

    const Offset first = Offset(50.0 + 70.0 / 2.0, 30.0 + 110.0 / 2.0);
    const Offset last = Offset(37.0 + 59.0 / 2.0, 31.0 + 71.0 / 2.0);

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(first));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(Offset.lerp(first, last, 0.5)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(last));

    expect(box, hasAGoodToStringDeep);
    expect(
      box.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderLimitedBox#00000\n'
        ' │ parentData: top=31.0; left=37.0; width=59.0; height=71.0;\n'
        ' │   offset=Offset(37.0, 31.0) (can use size)\n'
        ' │ constraints: BoxConstraints(w=59.0, h=71.0)\n'
        ' │ size: Size(59.0, 71.0)\n'
        ' │ maxWidth: 0.0\n'
        ' │ maxHeight: 0.0\n'
        ' │\n'
        ' └─child: RenderConstrainedBox#00000\n'
        '     parentData: <none> (can use size)\n'
        '     constraints: BoxConstraints(w=59.0, h=71.0)\n'
        '     size: Size(59.0, 71.0)\n'
        '     additionalConstraints: BoxConstraints(biggest)\n',
      ),
    );
  });

  testWidgets('AnimatedPositionedDirectional - basics (LTR)', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 50.0,
              top: 30.0,
              width: 70.0,
              height: 110.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(50.0 + 70.0 / 2.0, 30.0 + 110.0 / 2.0)),
    );

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(50.0 + 70.0 / 2.0, 30.0 + 110.0 / 2.0)),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 37.0,
              top: 31.0,
              width: 59.0,
              height: 71.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    const Offset first = Offset(50.0 + 70.0 / 2.0, 30.0 + 110.0 / 2.0);
    const Offset last = Offset(37.0 + 59.0 / 2.0, 31.0 + 71.0 / 2.0);

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(first));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(Offset.lerp(first, last, 0.5)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(last));

    expect(box, hasAGoodToStringDeep);
    expect(
      box.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderLimitedBox#00000\n'
        ' │ parentData: top=31.0; left=37.0; width=59.0; height=71.0;\n'
        ' │   offset=Offset(37.0, 31.0) (can use size)\n'
        ' │ constraints: BoxConstraints(w=59.0, h=71.0)\n'
        ' │ size: Size(59.0, 71.0)\n'
        ' │ maxWidth: 0.0\n'
        ' │ maxHeight: 0.0\n'
        ' │\n'
        ' └─child: RenderConstrainedBox#00000\n'
        '     parentData: <none> (can use size)\n'
        '     constraints: BoxConstraints(w=59.0, h=71.0)\n'
        '     size: Size(59.0, 71.0)\n'
        '     additionalConstraints: BoxConstraints(biggest)\n',
      ),
    );
  });

  testWidgets('AnimatedPositionedDirectional - basics (RTL)', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 50.0,
              top: 30.0,
              width: 70.0,
              height: 110.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(800.0 - 50.0 - 70.0 / 2.0, 30.0 + 110.0 / 2.0)),
    );

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(const Offset(800.0 - 50.0 - 70.0 / 2.0, 30.0 + 110.0 / 2.0)),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 37.0,
              top: 31.0,
              width: 59.0,
              height: 71.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    const Offset first = Offset(800.0 - 50.0 - 70.0 / 2.0, 30.0 + 110.0 / 2.0);
    const Offset last = Offset(800.0 - 37.0 - 59.0 / 2.0, 31.0 + 71.0 / 2.0);

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(first));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(Offset.lerp(first, last, 0.5)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(last));

    expect(box, hasAGoodToStringDeep);
    expect(
      box.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderLimitedBox#00000\n'
        ' │ parentData: top=31.0; right=37.0; width=59.0; height=71.0;\n'
        ' │   offset=Offset(704.0, 31.0) (can use size)\n'
        ' │ constraints: BoxConstraints(w=59.0, h=71.0)\n'
        ' │ size: Size(59.0, 71.0)\n'
        ' │ maxWidth: 0.0\n'
        ' │ maxHeight: 0.0\n'
        ' │\n'
        ' └─child: RenderConstrainedBox#00000\n'
        '     parentData: <none> (can use size)\n'
        '     constraints: BoxConstraints(w=59.0, h=71.0)\n'
        '     size: Size(59.0, 71.0)\n'
        '     additionalConstraints: BoxConstraints(biggest)\n',
      ),
    );
  });

  testWidgets('AnimatedPositioned - interrupted animation (VISUAL)', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          AnimatedPositioned(
            left: 0.0,
            top: 0.0,
            width: 100.0,
            height: 100.0,
            duration: const Duration(seconds: 2),
            child: Container(key: key),
          ),
        ],
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          AnimatedPositioned(
            left: 100.0,
            top: 100.0,
            width: 100.0,
            height: 100.0,
            duration: const Duration(seconds: 2),
            child: Container(key: key),
          ),
        ],
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(100.0, 100.0)));

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          AnimatedPositioned(
            left: 150.0,
            top: 150.0,
            width: 100.0,
            height: 100.0,
            duration: const Duration(seconds: 2),
            child: Container(key: key),
          ),
        ],
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(100.0, 100.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(150.0, 150.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(200.0, 200.0)));
  });

  testWidgets('AnimatedPositioned - switching variables (VISUAL)', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          AnimatedPositioned(
            left: 0.0,
            top: 0.0,
            width: 100.0,
            height: 100.0,
            duration: const Duration(seconds: 2),
            child: Container(key: key),
          ),
        ],
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pumpWidget(
      Stack(
        textDirection: TextDirection.ltr,
        children: <Widget>[
          AnimatedPositioned(
            left: 0.0,
            top: 100.0,
            right: 100.0, // 700.0 from the left
            height: 100.0,
            duration: const Duration(seconds: 2),
            child: Container(key: key),
          ),
        ],
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(350.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(350.0, 100.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(350.0, 150.0)));
  });

  testWidgets('AnimatedPositionedDirectional - interrupted animation (LTR)', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 0.0,
              top: 0.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 100.0,
              top: 100.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(100.0, 100.0)));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 150.0,
              top: 150.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(100.0, 100.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(150.0, 150.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(200.0, 200.0)));
  });

  testWidgets('AnimatedPositionedDirectional - switching variables (LTR)', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 0.0,
              top: 0.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(50.0, 50.0)));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 0.0,
              top: 100.0,
              end: 100.0, // 700.0 from the start
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(350.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(350.0, 100.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(350.0, 150.0)));
  });

  testWidgets('AnimatedPositionedDirectional - interrupted animation (RTL)', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 0.0,
              top: 0.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(750.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(750.0, 50.0)));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 100.0,
              top: 100.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(750.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(700.0, 100.0)));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 150.0,
              top: 150.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(700.0, 100.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(650.0, 150.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(600.0, 200.0)));
  });

  testWidgets('AnimatedPositionedDirectional - switching variables (RTL)', (
    WidgetTester tester,
  ) async {
    final GlobalKey key = GlobalKey();

    RenderBox box;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 0.0,
              top: 0.0,
              width: 100.0,
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(750.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(750.0, 50.0)));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Stack(
          children: <Widget>[
            AnimatedPositionedDirectional(
              start: 0.0,
              top: 100.0,
              end: 100.0, // 700.0 from the start
              height: 100.0,
              duration: const Duration(seconds: 2),
              child: Container(key: key),
            ),
          ],
        ),
      ),
    );

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(450.0, 50.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(450.0, 100.0)));

    await tester.pump(const Duration(seconds: 1));

    box = key.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(box.size.center(Offset.zero)), equals(const Offset(450.0, 150.0)));
  });
}
