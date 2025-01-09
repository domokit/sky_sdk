// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class Page extends StatefulWidget {
  const Page({super.key, required this.title, required this.onDispose});

  final String title;

  final void Function()? onDispose;

  @override
  State<Page> createState() => _PageState();
}

class _PageState extends State<Page> {
  @override
  void dispose() {
    widget.onDispose?.call();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(child: FilledButton(onPressed: () {}, child: Text(widget.title)));
  }
}

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/21506.
  testWidgets('InkSplash receives textDirection', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Button Border Test')),
          body: Center(child: ElevatedButton(child: const Text('Test'), onPressed: () {})),
        ),
      ),
    );
    await tester.tap(find.text('Test'));
    // start ink animation which asserts for a textDirection.
    await tester.pumpAndSettle(const Duration(milliseconds: 30));
    expect(tester.takeException(), isNull);
  });

  testWidgets('Material2 - InkWell with NoSplash splashFactory paints nothing', (
    WidgetTester tester,
  ) async {
    Widget buildFrame({InteractiveInkFeatureFactory? splashFactory}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: Material(
              child: InkWell(splashFactory: splashFactory, onTap: () {}, child: const Text('test')),
            ),
          ),
        ),
      );
    }

    // NoSplash.splashFactory, no splash circles drawn
    await tester.pumpWidget(buildFrame(splashFactory: NoSplash.splashFactory));
    {
      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('test')));
      final MaterialInkController material = Material.of(tester.element(find.text('test')));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 0));
      await gesture.up();
      await tester.pumpAndSettle();
    }

    // Default splashFactory (from Theme.of().splashFactory), one splash circle drawn.
    await tester.pumpWidget(buildFrame());
    {
      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('test')));
      final MaterialInkController material = Material.of(tester.element(find.text('test')));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));
      await gesture.up();
      await tester.pumpAndSettle();
    }
  });

  testWidgets('Material3 - InkWell with NoSplash splashFactory paints nothing', (
    WidgetTester tester,
  ) async {
    Widget buildFrame({InteractiveInkFeatureFactory? splashFactory}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: Material(
              child: InkWell(splashFactory: splashFactory, onTap: () {}, child: const Text('test')),
            ),
          ),
        ),
      );
    }

    // NoSplash.splashFactory, one rect is drawn for the highlight.
    await tester.pumpWidget(buildFrame(splashFactory: NoSplash.splashFactory));
    {
      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('test')));
      final MaterialInkController material = Material.of(tester.element(find.text('test')));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawRect, 1));
      await gesture.up();
      await tester.pumpAndSettle();
    }

    // Default splashFactory (from Theme.of().splashFactory), two rects are drawn for the splash and highlight.
    await tester.pumpWidget(buildFrame());
    {
      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('test')));
      final MaterialInkController material = Material.of(tester.element(find.text('test')));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawRect, (kIsWeb && isSkiaWeb ? 1 : 2)));
      await gesture.up();
      await tester.pumpAndSettle();
    }
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  // Regression test for https://github.com/flutter/flutter/issues/136441.
  testWidgets('PageView item can dispose when widget with NoSplash.splashFactory is tapped', (
    WidgetTester tester,
  ) async {
    final PageController controller = PageController();
    final List<int> disposedPageIndexes = <int>[];
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(splashFactory: NoSplash.splashFactory),
        home: Scaffold(
          body: PageView.builder(
            controller: controller,
            itemBuilder: (BuildContext context, int index) {
              return Page(
                title: 'Page $index',
                onDispose: () {
                  disposedPageIndexes.add(index);
                },
              );
            },
            itemCount: 3,
          ),
        ),
      ),
    );
    controller.jumpToPage(1);
    await tester.pumpAndSettle();
    await tester.tap(find.text('Page 1'));
    await tester.pumpAndSettle();
    controller.jumpToPage(0);
    await tester.pumpAndSettle();
    expect(disposedPageIndexes, <int>[0, 1]);
    controller.dispose();
  });
}
