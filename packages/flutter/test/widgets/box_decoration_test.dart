// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['reduced-test-set'])
library;

import 'dart:async';
import 'dart:math' as math;
import 'dart:ui' as ui show Image;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import '../image_data.dart';

class TestImageProvider extends ImageProvider<TestImageProvider> {
  TestImageProvider(this.future);

  final Future<void> future;

  static final List<ui.Image> _images = <ui.Image>[];

  static Future<void> prepareImages(int count) async {
    for (int i = 0; i < count; i++) {
      _images.add(await decodeImageFromList(Uint8List.fromList(kTransparentImage)));
    }
  }

  @override
  Future<TestImageProvider> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<TestImageProvider>(this);
  }

  @override
  ImageStreamCompleter loadImage(TestImageProvider key, ImageDecoderCallback decode) {
    assert(_images.isNotEmpty, 'ask for more images in `prepareImages`');
    final ui.Image image = _images.last;
    _images.removeLast();

    return OneFrameImageStreamCompleter(
      future.then<ImageInfo>((void value) {
        final ImageInfo result = ImageInfo(image: image);
        return result;
      }),
    );
  }
}

Future<void> main() async {
  AutomatedTestWidgetsFlutterBinding();
  await TestImageProvider.prepareImages(2);

  testWidgets('DecoratedBox handles loading images', (WidgetTester tester) async {
    addTearDown(imageCache.clear);
    final GlobalKey key = GlobalKey();
    final Completer<void> completer = Completer<void>();
    await tester.pumpWidget(
      KeyedSubtree(
        key: key,
        child: DecoratedBox(
          decoration: BoxDecoration(
            image: DecorationImage(image: TestImageProvider(completer.future)),
          ),
        ),
      ),
    );
    expect(tester.binding.hasScheduledFrame, isFalse);
    completer.complete();
    await tester.idle();
    expect(tester.binding.hasScheduledFrame, isTrue);
    await tester.pump();
    expect(tester.binding.hasScheduledFrame, isFalse);
  });

  testWidgets('Moving a DecoratedBox', (WidgetTester tester) async {
    addTearDown(imageCache.clear);
    final Completer<void> completer = Completer<void>();
    final Widget subtree = KeyedSubtree(
      key: GlobalKey(),
      child: RepaintBoundary(
        child: DecoratedBox(
          decoration: BoxDecoration(
            image: DecorationImage(image: TestImageProvider(completer.future)),
          ),
        ),
      ),
    );
    await tester.pumpWidget(subtree);
    await tester.idle();
    expect(tester.binding.hasScheduledFrame, isFalse);
    await tester.pumpWidget(Container(child: subtree));
    await tester.idle();
    expect(tester.binding.hasScheduledFrame, isFalse);
    completer.complete(); // schedules microtask, does not run it
    expect(tester.binding.hasScheduledFrame, isFalse);
    await tester.idle(); // runs microtask
    expect(tester.binding.hasScheduledFrame, isTrue);
    await tester.pump();
    await tester.idle();
    expect(tester.binding.hasScheduledFrame, isFalse);
  });

  testWidgets('Circles can have uniform borders', (WidgetTester tester) async {
    await tester.pumpWidget(
      Container(
        padding: const EdgeInsets.all(50.0),
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          border: Border.all(width: 10.0, color: const Color(0x80FF00FF)),
          color: Colors.teal[600],
        ),
      ),
    );
  });

  testWidgets('Bordered Container insets its child', (WidgetTester tester) async {
    const Key key = Key('outerContainer');
    await tester.pumpWidget(
      Center(
        child: Container(
          key: key,
          decoration: BoxDecoration(border: Border.all(width: 10.0)),
          child: const SizedBox(width: 25.0, height: 25.0),
        ),
      ),
    );
    expect(tester.getSize(find.byKey(key)), equals(const Size(45.0, 45.0)));
  });

  testWidgets('BoxDecoration paints its border correctly', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/7672

    const Key key = Key('Container with BoxDecoration');
    Widget buildFrame(Border border) {
      return Center(
        child: Container(
          key: key,
          width: 100.0,
          height: 50.0,
          decoration: BoxDecoration(border: border),
        ),
      );
    }

    const Color black = Color(0xFF000000);

    await tester.pumpWidget(buildFrame(Border.all()));
    expect(
      find.byKey(key),
      paints..rect(color: black, style: PaintingStyle.stroke, strokeWidth: 1.0),
    );

    await tester.pumpWidget(buildFrame(Border.all(width: 0.0)));
    expect(
      find.byKey(key),
      paints..rect(color: black, style: PaintingStyle.stroke, strokeWidth: 0.0),
    );

    const Color green = Color(0xFF00FF00);
    const BorderSide greenSide = BorderSide(color: green, width: 10.0);

    await tester.pumpWidget(buildFrame(const Border(top: greenSide)));
    expect(find.byKey(key), paints..path(color: green, style: PaintingStyle.fill));

    await tester.pumpWidget(buildFrame(const Border(left: greenSide)));
    expect(find.byKey(key), paints..path(color: green, style: PaintingStyle.fill));

    await tester.pumpWidget(buildFrame(const Border(right: greenSide)));
    expect(find.byKey(key), paints..path(color: green, style: PaintingStyle.fill));

    await tester.pumpWidget(buildFrame(const Border(bottom: greenSide)));
    expect(find.byKey(key), paints..path(color: green, style: PaintingStyle.fill));

    const Color blue = Color(0xFF0000FF);
    const BorderSide blueSide = BorderSide(color: blue, width: 0.0);

    await tester.pumpWidget(
      buildFrame(const Border(top: blueSide, right: greenSide, bottom: greenSide)),
    );
    expect(
      find.byKey(key),
      paints
        ..path() // There's not much point checking the arguments to these calls because paintBorder
        ..path() // reuses the same Paint object each time, configured differently, and so they will
        ..path(), // all appear to have the same settings here (that of the last call).
    );
  });

  testWidgets('BoxDecoration paints its border correctly', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/12165
    await tester.pumpWidget(
      Column(
        children: <Widget>[
          Container(
            // There's not currently a way to verify that this paints the same size as the others,
            // so the pattern below just asserts that there's four paths but doesn't check the geometry.
            width: 100.0,
            height: 100.0,
            decoration: const BoxDecoration(
              border: Border(
                top: BorderSide(width: 10.0, color: Color(0xFFEEEEEE)),
                left: BorderSide(width: 10.0, color: Color(0xFFFFFFFF)),
                right: BorderSide(width: 10.0, color: Color(0xFFFFFFFF)),
                bottom: BorderSide(width: 10.0, color: Color(0xFFFFFFFF)),
              ),
            ),
          ),
          Container(
            width: 100.0,
            height: 100.0,
            decoration: BoxDecoration(
              border: Border.all(width: 10.0, color: const Color(0xFFFFFFFF)),
            ),
          ),
          Container(
            width: 100.0,
            height: 100.0,
            decoration: BoxDecoration(
              border: Border.all(width: 10.0, color: const Color(0xFFFFFFFF)),
              borderRadius: const BorderRadius.only(topRight: Radius.circular(10.0)),
            ),
          ),
          Container(
            width: 100.0,
            height: 100.0,
            decoration: BoxDecoration(
              border: Border.all(width: 10.0, color: const Color(0xFFFFFFFF)),
              shape: BoxShape.circle,
            ),
          ),
        ],
      ),
    );
    expect(
      find.byType(Column),
      paints
        ..path()
        ..path()
        ..path()
        ..path()
        ..rect(rect: const Rect.fromLTRB(355.0, 105.0, 445.0, 195.0))
        ..drrect(
          outer: RRect.fromLTRBAndCorners(
            350.0,
            200.0,
            450.0,
            300.0,
            topRight: const Radius.circular(10.0),
          ),
          inner: RRect.fromLTRBAndCorners(360.0, 210.0, 440.0, 290.0),
        )
        ..circle(x: 400.0, y: 350.0, radius: 45.0),
    );
  });

  testWidgets('Can hit test on BoxDecoration', (WidgetTester tester) async {
    late List<int> itemsTapped;

    const Key key = Key('Container with BoxDecoration');
    Widget buildFrame(Border border) {
      itemsTapped = <int>[];
      return Center(
        child: GestureDetector(
          behavior: HitTestBehavior.deferToChild,
          child: Container(
            key: key,
            width: 100.0,
            height: 50.0,
            decoration: BoxDecoration(border: border),
          ),
          onTap: () {
            itemsTapped.add(1);
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Border.all()));
    expect(itemsTapped, isEmpty);

    await tester.tap(find.byKey(key));
    expect(itemsTapped, <int>[1]);

    await tester.tapAt(const Offset(350.0, 275.0));
    expect(itemsTapped, <int>[1, 1]);

    await tester.tapAt(const Offset(449.0, 324.0));
    expect(itemsTapped, <int>[1, 1, 1]);
  });

  testWidgets('Can hit test on BoxDecoration circle', (WidgetTester tester) async {
    late List<int> itemsTapped;

    const Key key = Key('Container with BoxDecoration');
    Widget buildFrame(Border border) {
      itemsTapped = <int>[];
      return Center(
        child: GestureDetector(
          behavior: HitTestBehavior.deferToChild,
          child: Container(
            key: key,
            width: 100.0,
            height: 50.0,
            decoration: BoxDecoration(border: border, shape: BoxShape.circle),
          ),
          onTap: () {
            itemsTapped.add(1);
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Border.all()));
    expect(itemsTapped, isEmpty);

    await tester.tapAt(Offset.zero);
    expect(itemsTapped, isEmpty);

    await tester.tapAt(const Offset(350.0, 275.0));
    expect(itemsTapped, isEmpty);

    await tester.tapAt(const Offset(400.0, 300.0));
    expect(itemsTapped, <int>[1]);

    await tester.tap(find.byKey(key));
    expect(itemsTapped, <int>[1, 1]);
  });

  testWidgets('Can hit test on BoxDecoration border', (WidgetTester tester) async {
    late List<int> itemsTapped;
    const Key key = Key('Container with BoxDecoration');
    Widget buildFrame(Border border) {
      itemsTapped = <int>[];
      return Center(
        child: GestureDetector(
          behavior: HitTestBehavior.deferToChild,
          child: Container(
            key: key,
            width: 100.0,
            height: 50.0,
            decoration: BoxDecoration(
              border: border,
              borderRadius: const BorderRadius.all(Radius.circular(20.0)),
            ),
          ),
          onTap: () {
            itemsTapped.add(1);
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Border.all()));

    expect(itemsTapped, isEmpty);

    await tester.tapAt(Offset.zero);
    expect(itemsTapped, isEmpty);

    await tester.tapAt(const Offset(350.0, 275.0));
    expect(itemsTapped, isEmpty);

    await tester.tapAt(const Offset(400.0, 300.0));
    expect(itemsTapped, <int>[1]);

    await tester.tap(find.byKey(key));
    expect(itemsTapped, <int>[1, 1]);
  });

  testWidgets('BoxDecoration not tap outside rounded angles - Top Left', (
    WidgetTester tester,
  ) async {
    const double height = 50.0;
    const double width = 50.0;
    const double radius = 12.3;

    late List<int> itemsTapped;
    const Key key = Key('Container with BoxDecoration');
    Widget buildFrame(Border border) {
      itemsTapped = <int>[];
      return Align(
        alignment: Alignment.topLeft,
        child: GestureDetector(
          behavior: HitTestBehavior.deferToChild,
          child: Container(
            key: key,
            width: width,
            height: height,
            decoration: BoxDecoration(border: border, borderRadius: BorderRadius.circular(radius)),
          ),
          onTap: () {
            itemsTapped.add(1);
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Border.all()));

    expect(itemsTapped, isEmpty);
    // x, y
    const Offset topLeft = Offset.zero;
    const Offset borderTopTangent = Offset(radius - 1, 0.0);
    const Offset borderLeftTangent = Offset(0.0, radius - 1);
    //the borderDiagonalOffset is the backslash line
    //\\######@@@
    //#\\###@####
    //##\\@######
    //##@########
    //@##########
    //@##########
    const double borderDiagonalOffset = radius - radius * math.sqrt1_2;
    const Offset fartherBorderRadiusPoint = Offset(borderDiagonalOffset, borderDiagonalOffset);

    await tester.tapAt(topLeft);
    expect(itemsTapped, isEmpty, reason: 'top left tapped');

    await tester.tapAt(borderTopTangent);
    expect(itemsTapped, isEmpty, reason: 'border top tapped');

    await tester.tapAt(borderLeftTangent);
    expect(itemsTapped, isEmpty, reason: 'border left tapped');

    await tester.tapAt(fartherBorderRadiusPoint);
    expect(itemsTapped, isEmpty, reason: 'border center tapped');

    await tester.tap(find.byKey(key));
    expect(itemsTapped, <int>[1]);
  });

  testWidgets('BoxDecoration tap inside rounded angles - Top Left', (WidgetTester tester) async {
    const double height = 50.0;
    const double width = 50.0;
    const double radius = 12.3;

    late List<int> itemsTapped;
    const Key key = Key('Container with BoxDecoration');
    Widget buildFrame(Border border) {
      itemsTapped = <int>[];
      return Align(
        alignment: Alignment.topLeft,
        child: GestureDetector(
          behavior: HitTestBehavior.deferToChild,
          child: Container(
            key: key,
            width: width,
            height: height,
            decoration: BoxDecoration(border: border, borderRadius: BorderRadius.circular(radius)),
          ),
          onTap: () {
            itemsTapped.add(1);
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Border.all()));

    expect(itemsTapped, isEmpty);
    // x, y
    const Offset borderTopTangent = Offset(radius, 0.0);
    const Offset borderLeftTangent = Offset(0.0, radius);
    const double borderDiagonalOffset = radius - radius * math.sqrt1_2;
    const Offset fartherBorderRadiusPoint = Offset(
      borderDiagonalOffset + 1,
      borderDiagonalOffset + 1,
    );

    await tester.tapAt(borderTopTangent);
    expect(itemsTapped, <int>[1], reason: 'border Top not tapped');

    await tester.tapAt(borderLeftTangent);
    expect(itemsTapped, <int>[1, 1], reason: 'border Left not tapped');

    await tester.tapAt(fartherBorderRadiusPoint);
    expect(itemsTapped, <int>[1, 1, 1], reason: 'border center not tapped');

    await tester.tap(find.byKey(key));
    expect(itemsTapped, <int>[1, 1, 1, 1]);
  });

  testWidgets('BoxDecoration rounded angles other corner works', (WidgetTester tester) async {
    const double height = 50.0;
    const double width = 50.0;
    const double radius = 20;

    late List<int> itemsTapped;
    const Key key = Key('Container with BoxDecoration');
    Widget buildFrame(Border border) {
      itemsTapped = <int>[];
      return Align(
        alignment: Alignment.topLeft,
        child: GestureDetector(
          behavior: HitTestBehavior.deferToChild,
          child: Container(
            key: key,
            width: width,
            height: height,
            decoration: BoxDecoration(border: border, borderRadius: BorderRadius.circular(radius)),
          ),
          onTap: () {
            itemsTapped.add(1);
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame(Border.all()));

    expect(itemsTapped, isEmpty);

    await tester.tap(find.byKey(key));
    expect(itemsTapped, <int>[1]);

    // x, y
    const Offset topRightOutside = Offset(width, 0.0);
    const Offset topRightInside = Offset(width - radius, radius);
    const Offset bottomRightOutside = Offset(width, height);
    const Offset bottomRightInside = Offset(width - radius, height - radius);
    const Offset bottomLeftOutside = Offset(0, height);
    const Offset bottomLeftInside = Offset(radius, height - radius);
    const Offset topLeftOutside = Offset.zero;
    const Offset topLeftInside = Offset(radius, radius);

    await tester.tapAt(topRightInside);
    expect(itemsTapped, <int>[1, 1], reason: 'top right not tapped');

    await tester.tapAt(topRightOutside);
    expect(itemsTapped, <int>[1, 1], reason: 'top right tapped');

    await tester.tapAt(bottomRightInside);
    expect(itemsTapped, <int>[1, 1, 1], reason: 'bottom right not tapped');

    await tester.tapAt(bottomRightOutside);
    expect(itemsTapped, <int>[1, 1, 1], reason: 'bottom right tapped');

    await tester.tapAt(bottomLeftInside);
    expect(itemsTapped, <int>[1, 1, 1, 1], reason: 'bottom left not tapped');

    await tester.tapAt(bottomLeftOutside);
    expect(itemsTapped, <int>[1, 1, 1, 1], reason: 'bottom left tapped');

    await tester.tapAt(topLeftInside);
    expect(itemsTapped, <int>[1, 1, 1, 1, 1], reason: 'top left not tapped');

    await tester.tapAt(topLeftOutside);
    expect(itemsTapped, <int>[1, 1, 1, 1, 1], reason: 'top left tapped');
  });

  testWidgets("BoxDecoration doesn't crash with BorderRadiusDirectional", (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/88039

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Container(
          decoration: BoxDecoration(
            border: Border.all(),
            borderRadius: const BorderRadiusDirectional.all(Radius.circular(1.0)),
          ),
        ),
      ),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/13675
  testWidgets('Border avoids clipping edges when possible', (WidgetTester tester) async {
    final Key key = UniqueKey();
    Widget buildWidget(Color color) {
      final List<Widget> circles = <Widget>[];
      for (int i = 100; i > 25; i--) {
        final double radius = i * 2.5;
        final double angle = i * 0.5;
        final double x = radius * math.cos(angle);
        final double y = radius * math.sin(angle);
        final Widget circle = Positioned(
          left: 275 - x,
          top: 275 - y,
          child: Container(
            width: 250,
            height: 250,
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(75),
              color: Colors.black,
              border: Border.all(color: color, width: 50),
            ),
          ),
        );
        circles.add(circle);
      }

      return Center(
        key: key,
        child: Container(
          width: 800,
          height: 800,
          decoration: const ShapeDecoration(
            color: Colors.orangeAccent,
            shape: CircleBorder(side: BorderSide(strokeAlign: BorderSide.strokeAlignOutside)),
          ),
          child: Directionality(textDirection: TextDirection.ltr, child: Stack(children: circles)),
        ),
      );
    }

    await tester.pumpWidget(buildWidget(const Color(0xffffffff)));
    await expectLater(
      find.byKey(key),
      matchesGoldenFile('painting.box_decoration.border.should_be_white.png'),
    );

    await tester.pumpWidget(buildWidget(const Color(0xfeffffff)));
    await expectLater(
      find.byKey(key),
      matchesGoldenFile('painting.box_decoration.border.show_lines_due_to_opacity.png'),
    );
  });
}
