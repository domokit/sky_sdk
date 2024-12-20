// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedContainer.debugFillProperties', (WidgetTester tester) async {
    final AnimatedContainer container = AnimatedContainer(
      constraints: const BoxConstraints.tightFor(width: 17.0, height: 23.0),
      decoration: const BoxDecoration(color: Color(0xFF00FF00)),
      foregroundDecoration: const BoxDecoration(color: Color(0x7F0000FF)),
      margin: const EdgeInsets.all(10.0),
      padding: const EdgeInsets.all(7.0),
      transform: Matrix4.translationValues(4.0, 3.0, 0.0),
      width: 50.0,
      height: 75.0,
      curve: Curves.ease,
      duration: const Duration(milliseconds: 200),
    );

    expect(container, hasOneLineDescription);
  });

  testWidgets('AnimatedContainer control test', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();

    const BoxDecoration decorationA = BoxDecoration(color: Color(0xFF00FF00));

    const BoxDecoration decorationB = BoxDecoration(color: Color(0xFF0000FF));

    BoxDecoration actualDecoration;

    await tester.pumpWidget(
      AnimatedContainer(
        key: key,
        duration: const Duration(milliseconds: 200),
        decoration: decorationA,
      ),
    );

    final RenderDecoratedBox box = key.currentContext!.findRenderObject()! as RenderDecoratedBox;
    actualDecoration = box.decoration as BoxDecoration;
    expect(actualDecoration.color, equals(decorationA.color));

    await tester.pumpWidget(
      AnimatedContainer(
        key: key,
        duration: const Duration(milliseconds: 200),
        decoration: decorationB,
      ),
    );

    expect(key.currentContext!.findRenderObject(), equals(box));
    actualDecoration = box.decoration as BoxDecoration;
    expect(actualDecoration.color, equals(decorationA.color));

    await tester.pump(const Duration(seconds: 1));

    actualDecoration = box.decoration as BoxDecoration;
    expect(actualDecoration.color, equals(decorationB.color));

    expect(box, hasAGoodToStringDeep);
    expect(
      box.toStringDeep(minLevel: DiagnosticLevel.info, wrapWidth: 300),
      equalsIgnoringHashCodes(
        'RenderDecoratedBox#00000\n'
        ' │ parentData: <none>\n'
        ' │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        ' │ size: Size(800.0, 600.0)\n'
        ' │ decoration: BoxDecoration:\n'
        ' │   color: ${const Color(0xff0000ff)}\n'
        ' │ configuration: ImageConfiguration(bundle: '
        'PlatformAssetBundle#00000(), devicePixelRatio: 3.0, platform: '
        'android)\n'
        ' │\n'
        ' └─child: RenderPadding#00000\n'
        '   │ parentData: <none> (can use size)\n'
        '   │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '   │ size: Size(800.0, 600.0)\n'
        '   │ padding: EdgeInsets.zero\n'
        '   │\n'
        '   └─child: RenderLimitedBox#00000\n'
        '     │ parentData: offset=Offset(0.0, 0.0) (can use size)\n'
        '     │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '     │ size: Size(800.0, 600.0)\n'
        '     │ maxWidth: 0.0\n'
        '     │ maxHeight: 0.0\n'
        '     │\n'
        '     └─child: RenderConstrainedBox#00000\n'
        '         parentData: <none> (can use size)\n'
        '         constraints: BoxConstraints(w=800.0, h=600.0)\n'
        '         size: Size(800.0, 600.0)\n'
        '         additionalConstraints: BoxConstraints(biggest)\n',
      ),
    );
  });

  testWidgets('AnimatedContainer overanimate test', (WidgetTester tester) async {
    await tester.pumpWidget(
      AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        color: const Color(0xFF00FF00),
      ),
    );
    expect(tester.binding.transientCallbackCount, 0);
    await tester.pump(const Duration(seconds: 1));
    expect(tester.binding.transientCallbackCount, 0);
    await tester.pumpWidget(
      AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        color: const Color(0xFF00FF00),
      ),
    );
    expect(tester.binding.transientCallbackCount, 0);
    await tester.pump(const Duration(seconds: 1));
    expect(tester.binding.transientCallbackCount, 0);
    await tester.pumpWidget(
      AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        color: const Color(0xFF0000FF),
      ),
    );
    expect(
      tester.binding.transientCallbackCount,
      1,
    ); // this is the only time an animation should have started!
    await tester.pump(const Duration(seconds: 1));
    expect(tester.binding.transientCallbackCount, 0);
    await tester.pumpWidget(
      AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        color: const Color(0xFF0000FF),
      ),
    );
    expect(tester.binding.transientCallbackCount, 0);
  });

  testWidgets('AnimatedContainer padding visual-to-directional animation', (
    WidgetTester tester,
  ) async {
    final Key target = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          padding: const EdgeInsets.only(right: 50.0),
          child: SizedBox.expand(key: target),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(target)), const Size(750.0, 600.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(750.0, 0.0));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          padding: const EdgeInsetsDirectional.only(start: 100.0),
          child: SizedBox.expand(key: target),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(target)), const Size(750.0, 600.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(750.0, 0.0));

    await tester.pump(const Duration(milliseconds: 100));

    expect(tester.getSize(find.byKey(target)), const Size(725.0, 600.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(725.0, 0.0));

    await tester.pump(const Duration(milliseconds: 500));

    expect(tester.getSize(find.byKey(target)), const Size(700.0, 600.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(700.0, 0.0));
  });

  testWidgets('AnimatedContainer alignment visual-to-directional animation', (
    WidgetTester tester,
  ) async {
    final Key target = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          alignment: Alignment.topRight,
          child: SizedBox(key: target, width: 100.0, height: 200.0),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(800.0, 0.0));

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          alignment: AlignmentDirectional.bottomStart,
          child: SizedBox(key: target, width: 100.0, height: 200.0),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(800.0, 0.0));

    await tester.pump(const Duration(milliseconds: 100));

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(800.0, 200.0));

    await tester.pump(const Duration(milliseconds: 500));

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopRight(find.byKey(target)), const Offset(800.0, 400.0));
  });

  testWidgets('Animation rerun', (WidgetTester tester) async {
    await tester.pumpWidget(
      Center(
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          width: 100.0,
          height: 100.0,
          child: const Text('X', textDirection: TextDirection.ltr),
        ),
      ),
    );

    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    RenderBox text = tester.renderObject(find.text('X'));
    expect(text.size.width, equals(100.0));
    expect(text.size.height, equals(100.0));

    await tester.pump(const Duration(milliseconds: 1000));

    await tester.pumpWidget(
      Center(
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          width: 200.0,
          height: 200.0,
          child: const Text('X', textDirection: TextDirection.ltr),
        ),
      ),
    );
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    text = tester.renderObject(find.text('X'));
    expect(text.size.width, greaterThan(110.0));
    expect(text.size.width, lessThan(190.0));
    expect(text.size.height, greaterThan(110.0));
    expect(text.size.height, lessThan(190.0));

    await tester.pump(const Duration(milliseconds: 1000));

    expect(text.size.width, equals(200.0));
    expect(text.size.height, equals(200.0));

    await tester.pumpWidget(
      Center(
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          width: 200.0,
          height: 100.0,
          child: const Text('X', textDirection: TextDirection.ltr),
        ),
      ),
    );
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));

    expect(text.size.width, equals(200.0));
    expect(text.size.height, greaterThan(110.0));
    expect(text.size.height, lessThan(190.0));

    await tester.pump(const Duration(milliseconds: 1000));

    expect(text.size.width, equals(200.0));
    expect(text.size.height, equals(100.0));
  });

  testWidgets('AnimatedContainer sets transformAlignment', (WidgetTester tester) async {
    final Key target = UniqueKey();

    await tester.pumpWidget(
      Center(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: AnimatedContainer(
            duration: const Duration(milliseconds: 200),
            transform: Matrix4.diagonal3Values(0.5, 0.5, 1),
            transformAlignment: Alignment.topLeft,
            child: SizedBox(key: target, width: 100.0, height: 200.0),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopLeft(find.byKey(target)), const Offset(350.0, 200.0));

    await tester.pumpWidget(
      Center(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: AnimatedContainer(
            duration: const Duration(milliseconds: 200),
            transform: Matrix4.diagonal3Values(0.5, 0.5, 1),
            transformAlignment: Alignment.bottomRight,
            child: SizedBox(key: target, width: 100.0, height: 200.0),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopLeft(find.byKey(target)), const Offset(350.0, 200.0));

    await tester.pump(const Duration(milliseconds: 100));

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopLeft(find.byKey(target)), const Offset(375.0, 250.0));

    await tester.pump(const Duration(milliseconds: 500));

    expect(tester.getSize(find.byKey(target)), const Size(100.0, 200.0));
    expect(tester.getTopLeft(find.byKey(target)), const Offset(400.0, 300.0));
  });

  testWidgets('AnimatedContainer sets clipBehavior', (WidgetTester tester) async {
    await tester.pumpWidget(
      AnimatedContainer(
        decoration: const BoxDecoration(color: Color(0xFFED1D7F)),
        duration: const Duration(milliseconds: 200),
      ),
    );
    expect(tester.firstWidget<Container>(find.byType(Container)).clipBehavior, Clip.none);
    await tester.pumpWidget(
      AnimatedContainer(
        decoration: const BoxDecoration(color: Color(0xFFED1D7F)),
        duration: const Duration(milliseconds: 200),
        clipBehavior: Clip.antiAlias,
      ),
    );
    expect(tester.firstWidget<Container>(find.byType(Container)).clipBehavior, Clip.antiAlias);
  });
}
