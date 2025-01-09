// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('RenderSliverFixedExtentList layout test - rounding error', () {
    final List<RenderBox> children = <RenderBox>[
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
    ];
    final TestRenderSliverBoxChildManager childManager = TestRenderSliverBoxChildManager(
      children: children,
    );
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 0,
      children: <RenderSliver>[childManager.createRenderSliverFillViewport()],
    );
    layout(root);
    expect(children[0].attached, true);
    expect(children[1].attached, false);

    root.offset = ViewportOffset.fixed(600);
    pumpFrame();
    expect(children[0].attached, false);
    expect(children[1].attached, true);

    // Simulate double precision error.
    root.offset = ViewportOffset.fixed(1199.999999999998);
    pumpFrame();
    expect(children[1].attached, false);
    expect(children[2].attached, true);
  });

  group('getMaxChildIndexForScrollOffset', () {
    // Regression test for https://github.com/flutter/flutter/issues/68182

    const double genericItemExtent = 600.0;
    const double extraValueToNotHaveRoundingIssues = 1e-10;
    const double extraValueToHaveRoundingIssues = 1e-11;

    test('should be 0 when item extent is 0', () {
      const double offsetValueWhichDoesNotCare = 1234;
      final int actual = testGetMaxChildIndexForScrollOffset(offsetValueWhichDoesNotCare, 0);
      expect(actual, 0);
    });

    test('should be 0 when offset is 0', () {
      final int actual = testGetMaxChildIndexForScrollOffset(0, genericItemExtent);
      expect(actual, 0);
    });

    test('should be 0 when offset is equal to item extent', () {
      final int actual = testGetMaxChildIndexForScrollOffset(genericItemExtent, genericItemExtent);
      expect(actual, 0);
    });

    test('should be 1 when offset is greater than item extent', () {
      final int actual = testGetMaxChildIndexForScrollOffset(
        genericItemExtent + 1,
        genericItemExtent,
      );
      expect(actual, 1);
    });

    test('should be 1 when offset is slightly greater than item extent', () {
      final int actual = testGetMaxChildIndexForScrollOffset(
        genericItemExtent + extraValueToNotHaveRoundingIssues,
        genericItemExtent,
      );
      expect(actual, 1);
    });

    test('should be 4 when offset is four times and a half greater than item extent', () {
      final int actual = testGetMaxChildIndexForScrollOffset(
        genericItemExtent * 4.5,
        genericItemExtent,
      );
      expect(actual, 4);
    });

    test('should be 5 when offset is 6 times greater than item extent', () {
      const double anotherGenericItemExtent = 414.0;
      final int actual = testGetMaxChildIndexForScrollOffset(
        anotherGenericItemExtent * 6,
        anotherGenericItemExtent,
      );
      expect(actual, 5);
    });

    test(
      'should be 5 when offset is 6 times greater than a specific item extent where the division will return more than 13 zero decimals',
      () {
        const double itemExtentSpecificForAProblematicScreenSize = 411.42857142857144;
        final int actual = testGetMaxChildIndexForScrollOffset(
          itemExtentSpecificForAProblematicScreenSize * 6 + extraValueToHaveRoundingIssues,
          itemExtentSpecificForAProblematicScreenSize,
        );
        expect(actual, 5);
      },
    );

    test('should be 0 when offset is a bit greater than item extent', () {
      final int actual = testGetMaxChildIndexForScrollOffset(
        genericItemExtent + extraValueToHaveRoundingIssues,
        genericItemExtent,
      );
      expect(actual, 0);
    });
  });

  test('Implements paintsChild correctly', () {
    final List<RenderBox> children = <RenderBox>[
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
    ];
    final TestRenderSliverBoxChildManager childManager = TestRenderSliverBoxChildManager(
      children: children,
    );
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 0,
      children: <RenderSliver>[childManager.createRenderSliverFillViewport()],
    );
    layout(root);
    expect(children.first.parent, isA<RenderSliverMultiBoxAdaptor>());

    final RenderSliverMultiBoxAdaptor parent =
        children.first.parent! as RenderSliverMultiBoxAdaptor;
    expect(parent.paintsChild(children[0]), true);
    expect(parent.paintsChild(children[1]), false);
    expect(parent.paintsChild(children[2]), false);

    root.offset = ViewportOffset.fixed(600);
    pumpFrame();
    expect(parent.paintsChild(children[0]), false);
    expect(parent.paintsChild(children[1]), true);
    expect(parent.paintsChild(children[2]), false);

    root.offset = ViewportOffset.fixed(1200);
    pumpFrame();
    expect(parent.paintsChild(children[0]), false);
    expect(parent.paintsChild(children[1]), false);
    expect(parent.paintsChild(children[2]), true);
  });

  test('RenderSliverFillViewport correctly references itemExtent, non-zero offset', () {
    final List<RenderBox> children = <RenderBox>[
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
    ];
    final TestRenderSliverBoxChildManager childManager = TestRenderSliverBoxChildManager(
      children: children,
    );
    final RenderSliverFillViewport sliver = childManager.createRenderSliverFillViewport();
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.fixed(1200.0),
      cacheExtent: 100,
      children: <RenderSliver>[sliver],
    );
    layout(root);
    // These are a bogus itemExtents, and motivate the deprecation. The sliver
    // knows its itemExtent, and should use its configured extent rather than
    // whatever is provided through these methods.
    // Also, the API is a bit redundant, so we clean!
    // In this case, the true item extent is 600 to fill the viewport.
    expect(sliver.constraints.scrollOffset, 1200.0);
    expect(sliver.itemExtent, 600.0);
    final double layoutOffset = sliver.indexToLayoutOffset(
      150.0, // itemExtent
      10,
    );
    expect(layoutOffset, 6000.0);
    final int minIndex = sliver.getMinChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150.0, // itemExtent
    );
    expect(minIndex, 2);
    final int maxIndex = sliver.getMaxChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150, // itemExtent
    );
    expect(maxIndex, 1);
    final double maxScrollOffset = sliver.computeMaxScrollOffset(
      sliver.constraints,
      150, // itemExtent
    );
    expect(maxScrollOffset, 1800.0);
  });

  test('RenderSliverFillViewport correctly references itemExtent, zero offset', () {
    final List<RenderBox> children = <RenderBox>[
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
    ];
    final TestRenderSliverBoxChildManager childManager = TestRenderSliverBoxChildManager(
      children: children,
    );
    final RenderSliverFillViewport sliver = childManager.createRenderSliverFillViewport();
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 100,
      children: <RenderSliver>[sliver],
    );
    layout(root);
    // These are a bogus itemExtents, and motivate the deprecation. The sliver
    // knows its itemExtent, and should use its configured extent rather than
    // whatever is provided through these methods.
    // Also, the API is a bit redundant, so we clean!
    // In this case, the true item extent is 600 to fill the viewport.
    expect(sliver.constraints.scrollOffset, 0.0);
    expect(sliver.itemExtent, 600.0);
    final double layoutOffset = sliver.indexToLayoutOffset(
      150.0, // itemExtent
      10,
    );
    expect(layoutOffset, 6000.0);
    final int minIndex = sliver.getMinChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150.0, // itemExtent
    );
    expect(minIndex, 0);
    final int maxIndex = sliver.getMaxChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150, // itemExtent
    );
    expect(maxIndex, 0);
    final double maxScrollOffset = sliver.computeMaxScrollOffset(
      sliver.constraints,
      150, // itemExtent
    );
    expect(maxScrollOffset, 1800.0);
  });

  test('RenderSliverFixedExtentList correctly references itemExtent, non-zero offset', () {
    final List<RenderBox> children = <RenderBox>[
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
    ];
    final TestRenderSliverBoxChildManager childManager = TestRenderSliverBoxChildManager(
      children: children,
    );
    final RenderSliverFixedExtentList sliver = childManager.createRenderSliverFixedExtentList(30.0);
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.fixed(45.0),
      cacheExtent: 100,
      children: <RenderSliver>[sliver],
    );
    layout(root);
    // These are a bogus itemExtents, and motivate the deprecation. The sliver
    // knows its itemExtent, and should use its configured extent rather than
    // whatever is provided through these methods.
    // Also, the API is a bit redundant, so we clean!
    // In this case, the true item extent is 30.0.
    expect(sliver.constraints.scrollOffset, 45.0);
    expect(sliver.constraints.viewportMainAxisExtent, 600.0);
    expect(sliver.itemExtent, 30.0);
    final double layoutOffset = sliver.indexToLayoutOffset(
      150.0, // itemExtent
      10,
    );
    expect(layoutOffset, 300.0);
    final int minIndex = sliver.getMinChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150.0, // itemExtent
    );
    expect(minIndex, 1);
    final int maxIndex = sliver.getMaxChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150, // itemExtent
    );
    expect(maxIndex, 1);
    final double maxScrollOffset = sliver.computeMaxScrollOffset(
      sliver.constraints,
      150, // itemExtent
    );
    expect(maxScrollOffset, 90.0);
  });

  test('RenderSliverFixedExtentList correctly references itemExtent, zero offset', () {
    final List<RenderBox> children = <RenderBox>[
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
    ];
    final TestRenderSliverBoxChildManager childManager = TestRenderSliverBoxChildManager(
      children: children,
    );
    final RenderSliverFixedExtentList sliver = childManager.createRenderSliverFixedExtentList(30.0);
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 100,
      children: <RenderSliver>[sliver],
    );
    layout(root);
    // These are a bogus itemExtents, and motivate the deprecation. The sliver
    // knows its itemExtent, and should use its configured extent rather than
    // whatever is provided through these methods.
    // Also, the API is a bit redundant, so we clean!
    // In this case, the true item extent is 30.0.
    expect(sliver.constraints.scrollOffset, 0.0);
    expect(sliver.constraints.viewportMainAxisExtent, 600.0);
    expect(sliver.itemExtent, 30.0);
    final double layoutOffset = sliver.indexToLayoutOffset(
      150.0, // itemExtent
      10,
    );
    expect(layoutOffset, 300.0);
    final int minIndex = sliver.getMinChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150.0, // itemExtent
    );
    expect(minIndex, 0);
    final int maxIndex = sliver.getMaxChildIndexForScrollOffset(
      sliver.constraints.scrollOffset,
      150, // itemExtent
    );
    expect(maxIndex, 0);
    final double maxScrollOffset = sliver.computeMaxScrollOffset(
      sliver.constraints,
      150, // itemExtent
    );
    expect(maxScrollOffset, 90.0);
  });

  test('RenderSliverMultiBoxAdaptor has calculate leading and trailing garbage', () {
    final List<RenderBox> children = <RenderBox>[
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
      RenderSizedBox(const Size(400.0, 100.0)),
    ];
    final TestRenderSliverBoxChildManager childManager = TestRenderSliverBoxChildManager(
      children: children,
    );
    final RenderSliverFixedExtentList sliver = childManager.createRenderSliverFixedExtentList(30.0);
    final RenderViewport root = RenderViewport(
      crossAxisDirection: AxisDirection.right,
      offset: ViewportOffset.zero(),
      cacheExtent: 100,
      children: <RenderSliver>[sliver],
    );
    layout(root);
    // There are 3 children. If I want to garbage collect based on keeping only
    // the middle child, then I should get 1 for leading and 1 for trailing.
    expect(sliver.calculateLeadingGarbage(firstIndex: 1), 1);
    expect(sliver.calculateTrailingGarbage(lastIndex: 1), 1);
  });
}

int testGetMaxChildIndexForScrollOffset(double scrollOffset, double itemExtent) {
  final TestRenderSliverFixedExtentBoxAdaptor renderSliver = TestRenderSliverFixedExtentBoxAdaptor(
    itemExtent: itemExtent,
  );
  return renderSliver.getMaxChildIndexForScrollOffset(scrollOffset, itemExtent);
}

class TestRenderSliverBoxChildManager extends RenderSliverBoxChildManager {
  TestRenderSliverBoxChildManager({required this.children});

  RenderSliverMultiBoxAdaptor? _renderObject;
  List<RenderBox> children;

  RenderSliverFillViewport createRenderSliverFillViewport() {
    assert(_renderObject == null);
    _renderObject = RenderSliverFillViewport(childManager: this);
    return _renderObject! as RenderSliverFillViewport;
  }

  RenderSliverFixedExtentList createRenderSliverFixedExtentList(double itemExtent) {
    assert(_renderObject == null);
    _renderObject = RenderSliverFixedExtentList(childManager: this, itemExtent: itemExtent);
    return _renderObject! as RenderSliverFixedExtentList;
  }

  int? _currentlyUpdatingChildIndex;

  @override
  void createChild(int index, {required RenderBox? after}) {
    if (index < 0 || index >= children.length) {
      return;
    }
    try {
      _currentlyUpdatingChildIndex = index;
      _renderObject!.insert(children[index], after: after);
    } finally {
      _currentlyUpdatingChildIndex = null;
    }
  }

  @override
  void removeChild(RenderBox child) {
    _renderObject!.remove(child);
  }

  @override
  double estimateMaxScrollOffset(
    SliverConstraints constraints, {
    int? firstIndex,
    int? lastIndex,
    double? leadingScrollOffset,
    double? trailingScrollOffset,
  }) {
    assert(lastIndex! >= firstIndex!);
    return children.length *
        (trailingScrollOffset! - leadingScrollOffset!) /
        (lastIndex! - firstIndex! + 1);
  }

  @override
  int get childCount => children.length;

  @override
  void didAdoptChild(RenderBox child) {
    assert(_currentlyUpdatingChildIndex != null);
    final SliverMultiBoxAdaptorParentData childParentData =
        child.parentData! as SliverMultiBoxAdaptorParentData;
    childParentData.index = _currentlyUpdatingChildIndex;
  }

  @override
  void setDidUnderflow(bool value) {}
}

class TestRenderSliverFixedExtentBoxAdaptor extends RenderSliverFixedExtentBoxAdaptor {
  TestRenderSliverFixedExtentBoxAdaptor({required double itemExtent})
    : _itemExtent = itemExtent,
      super(childManager: TestRenderSliverBoxChildManager(children: <RenderBox>[]));

  final double _itemExtent;

  @override
  int getMaxChildIndexForScrollOffset(double scrollOffset, double itemExtent) {
    return super.getMaxChildIndexForScrollOffset(scrollOffset, itemExtent);
  }

  @override
  double get itemExtent => _itemExtent;
}
