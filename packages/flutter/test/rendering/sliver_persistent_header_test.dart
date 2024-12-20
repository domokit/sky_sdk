// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  // Regression test for https://github.com/flutter/flutter/issues/35426.
  test('RenderSliverFloatingPersistentHeader maxScrollObstructionExtent is 0', () {
    final TestRenderSliverFloatingPersistentHeader header =
        TestRenderSliverFloatingPersistentHeader(child: RenderSizedBox(const Size(400.0, 100.0)));
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 0,
      children: <RenderSliver>[header],
    );
    layout(root);

    expect(header.geometry!.maxScrollObstructionExtent, 0);
  });

  test('RenderSliverFloatingPinnedPersistentHeader maxScrollObstructionExtent is minExtent', () {
    final TestRenderSliverFloatingPinnedPersistentHeader header =
        TestRenderSliverFloatingPinnedPersistentHeader(
          child: RenderSizedBox(const Size(400.0, 100.0)),
        );
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 0,
      children: <RenderSliver>[header],
    );
    layout(root);

    expect(header.geometry!.maxScrollObstructionExtent, 100.0);
  });
}

class TestRenderSliverFloatingPersistentHeader extends RenderSliverFloatingPersistentHeader {
  TestRenderSliverFloatingPersistentHeader({required RenderBox child})
    : super(child: child, vsync: null, showOnScreenConfiguration: null);

  @override
  double get maxExtent => 200;

  @override
  double get minExtent => 100;
}

class TestRenderSliverFloatingPinnedPersistentHeader
    extends RenderSliverFloatingPinnedPersistentHeader {
  TestRenderSliverFloatingPinnedPersistentHeader({required RenderBox child})
    : super(child: child, vsync: null, showOnScreenConfiguration: null);

  @override
  double get maxExtent => 200;

  @override
  double get minExtent => 100;
}
