// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

final Key blockKey = UniqueKey();
const double expandedAppbarHeight = 250.0;
final Key finderKey = UniqueKey();

void main() {
  testWidgets('FlexibleSpaceBar stretch mode default zoomBackground', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            physics: const BouncingScrollPhysics(),
            key: blockKey,
            slivers: <Widget>[
              SliverAppBar(
                expandedHeight: expandedAppbarHeight,
                pinned: true,
                stretch: true,
                flexibleSpace: FlexibleSpaceBar(background: Container(key: finderKey)),
              ),
              SliverToBoxAdapter(child: Container(height: 10000.0)),
            ],
          ),
        ),
      ),
    );

    // Scrolling up into the overscroll area causes the appBar to expand in size.
    // This overscroll effect enlarges the background in step with the appbar.
    final Finder appbarContainer = find.byKey(finderKey);
    final Size sizeBeforeScroll = tester.getSize(appbarContainer);
    await slowDrag(tester, blockKey, const Offset(0.0, 100.0));
    final Size sizeAfterScroll = tester.getSize(appbarContainer);

    expect(sizeBeforeScroll.height, lessThan(sizeAfterScroll.height));
  });

  testWidgets('FlexibleSpaceBar stretch mode blurBackground', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: CustomScrollView(
            physics: const BouncingScrollPhysics(),
            key: blockKey,
            slivers: <Widget>[
              SliverAppBar(
                expandedHeight: expandedAppbarHeight,
                pinned: true,
                stretch: true,
                flexibleSpace: RepaintBoundary(
                  child: FlexibleSpaceBar(
                    stretchModes: const <StretchMode>[StretchMode.blurBackground],
                    background: Row(
                      children: <Widget>[
                        Expanded(child: Container(color: Colors.red)),
                        Expanded(child: Container(color: Colors.blue)),
                      ],
                    ),
                  ),
                ),
              ),
              SliverToBoxAdapter(child: Container(height: 10000.0)),
            ],
          ),
        ),
      ),
    );

    // Scrolling up into the overscroll area causes the background to blur.
    await slowDrag(tester, blockKey, const Offset(0.0, 100.0));
    await expectLater(
      find.byType(FlexibleSpaceBar),
      matchesGoldenFile('flexible_space_bar_stretch_mode.blur_background.png'),
    );
  });

  testWidgets('FlexibleSpaceBar stretch mode fadeTitle', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            physics: const BouncingScrollPhysics(),
            key: blockKey,
            slivers: <Widget>[
              SliverAppBar(
                expandedHeight: expandedAppbarHeight,
                pinned: true,
                stretch: true,
                flexibleSpace: FlexibleSpaceBar(
                  stretchModes: const <StretchMode>[StretchMode.fadeTitle],
                  title: Text('Title', key: finderKey),
                ),
              ),
              SliverToBoxAdapter(child: Container(height: 10000.0)),
            ],
          ),
        ),
      ),
    );
    await slowDrag(tester, blockKey, const Offset(0.0, 10.0));
    Opacity opacityWidget = tester.widget<Opacity>(
      find.ancestor(of: find.text('Title'), matching: find.byType(Opacity)).first,
    );
    expect(opacityWidget.opacity.round(), equals(1));
    await slowDrag(tester, blockKey, const Offset(0.0, 100.0));
    opacityWidget = tester.widget<Opacity>(
      find.ancestor(of: find.text('Title'), matching: find.byType(Opacity)).first,
    );
    expect(opacityWidget.opacity, equals(0.0));
  });

  testWidgets('FlexibleSpaceBar stretch mode ignored for non-overscroll physics', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: CustomScrollView(
            physics: const ClampingScrollPhysics(),
            key: blockKey,
            slivers: <Widget>[
              SliverAppBar(
                expandedHeight: expandedAppbarHeight,
                stretch: true,
                pinned: true,
                flexibleSpace: FlexibleSpaceBar(
                  stretchModes: const <StretchMode>[StretchMode.blurBackground],
                  background: Container(key: finderKey),
                ),
              ),
              SliverToBoxAdapter(child: Container(height: 10000.0)),
            ],
          ),
        ),
      ),
    );

    final Finder appbarContainer = find.byKey(finderKey);
    final Size sizeBeforeScroll = tester.getSize(appbarContainer);
    await slowDrag(tester, blockKey, const Offset(0.0, 100.0));
    final Size sizeAfterScroll = tester.getSize(appbarContainer);

    expect(sizeBeforeScroll.height, equals(sizeAfterScroll.height));
  });
}

Future<void> slowDrag(WidgetTester tester, Key widget, Offset offset) async {
  final Offset target = tester.getCenter(find.byKey(widget));
  final TestGesture gesture = await tester.startGesture(target);
  await gesture.moveBy(offset);
  await tester.pump(const Duration(milliseconds: 10));
  await gesture.up();
}
