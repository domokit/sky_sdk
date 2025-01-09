// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Test sliver which always attempts to paint itself whether it is visible or not.
// Use for checking if slivers which take sliver children paints optimally.
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';

class RenderMockSliverToBoxAdapter extends RenderSliverToBoxAdapter {
  RenderMockSliverToBoxAdapter({super.child, required this.incrementCounter});
  final void Function() incrementCounter;

  @override
  void paint(PaintingContext context, Offset offset) {
    incrementCounter();
  }
}

class MockSliverToBoxAdapter extends SingleChildRenderObjectWidget {
  /// Creates a sliver that contains a single box widget.
  const MockSliverToBoxAdapter({super.key, super.child, required this.incrementCounter});

  final void Function() incrementCounter;

  @override
  RenderMockSliverToBoxAdapter createRenderObject(BuildContext context) =>
      RenderMockSliverToBoxAdapter(incrementCounter: incrementCounter);
}
