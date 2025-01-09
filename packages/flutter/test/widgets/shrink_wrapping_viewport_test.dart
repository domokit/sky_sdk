// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../rendering/rendering_tester.dart' show TestClipPaintingContext;

void main() {
  testWidgets('ShrinkWrappingViewport respects clipBehavior', (WidgetTester tester) async {
    Widget build(ShrinkWrappingViewport child) {
      return Directionality(textDirection: TextDirection.ltr, child: child);
    }

    final ViewportOffset offset1 = ViewportOffset.zero();
    addTearDown(offset1.dispose);

    await tester.pumpWidget(
      build(
        ShrinkWrappingViewport(
          offset: offset1,
          slivers: <Widget>[SliverToBoxAdapter(child: Container(height: 2000.0))],
        ),
      ),
    );

    // 1st, check that the render object has received the default clip behavior.
    final RenderShrinkWrappingViewport renderObject =
        tester.allRenderObjects.whereType<RenderShrinkWrappingViewport>().first;
    expect(renderObject.clipBehavior, equals(Clip.hardEdge));

    // 2nd, check that the painting context has received the default clip behavior.
    final TestClipPaintingContext context = TestClipPaintingContext();
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.hardEdge));

    final ViewportOffset offset2 = ViewportOffset.zero();
    addTearDown(offset2.dispose);

    // 3rd, pump a new widget to check that the render object can update its clip behavior.
    await tester.pumpWidget(
      build(
        ShrinkWrappingViewport(
          offset: offset2,
          slivers: <Widget>[SliverToBoxAdapter(child: Container(height: 2000.0))],
          clipBehavior: Clip.antiAlias,
        ),
      ),
    );
    expect(renderObject.clipBehavior, equals(Clip.antiAlias));

    // 4th, check that a non-default clip behavior can be sent to the painting context.
    renderObject.paint(context, Offset.zero);
    expect(context.clipBehavior, equals(Clip.antiAlias));
  });
}
