// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('LimitedBox: parent max size is unconstrained', () {
    final RenderBox child = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 300.0, height: 400.0),
    );
    final RenderBox parent = RenderConstrainedOverflowBox(
      minWidth: 0.0,
      maxWidth: double.infinity,
      minHeight: 0.0,
      maxHeight: double.infinity,
      child: RenderLimitedBox(maxWidth: 100.0, maxHeight: 200.0, child: child),
    );
    layout(parent);
    expect(child.size.width, 100.0);
    expect(child.size.height, 200.0);

    expect(parent, hasAGoodToStringDeep);
    expect(
      parent.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderConstrainedOverflowBox#00000 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ parentData: <none>\n'
        ' │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        ' │ size: Size(800.0, 600.0)\n'
        ' │ alignment: Alignment.center\n'
        ' │ minWidth: 0.0\n'
        ' │ maxWidth: Infinity\n'
        ' │ minHeight: 0.0\n'
        ' │ maxHeight: Infinity\n'
        ' │ fit: max\n'
        ' │\n'
        ' └─child: RenderLimitedBox#00000 relayoutBoundary=up1 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        '   │ parentData: offset=Offset(350.0, 200.0) (can use size)\n'
        '   │ constraints: BoxConstraints(unconstrained)\n'
        '   │ size: Size(100.0, 200.0)\n'
        '   │ maxWidth: 100.0\n'
        '   │ maxHeight: 200.0\n'
        '   │\n'
        '   └─child: RenderConstrainedBox#00000 relayoutBoundary=up2 NEEDS-PAINT\n'
        '       parentData: <none> (can use size)\n'
        '       constraints: BoxConstraints(0.0<=w<=100.0, 0.0<=h<=200.0)\n'
        '       size: Size(100.0, 200.0)\n'
        '       additionalConstraints: BoxConstraints(w=300.0, h=400.0)\n',
      ),
    );
  });

  test('LimitedBox: parent maxWidth is unconstrained', () {
    final RenderBox child = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 300.0, height: 400.0),
    );
    final RenderBox parent = RenderConstrainedOverflowBox(
      minWidth: 0.0,
      maxWidth: double.infinity,
      minHeight: 500.0,
      maxHeight: 500.0,
      child: RenderLimitedBox(maxWidth: 100.0, maxHeight: 200.0, child: child),
    );
    layout(parent);
    expect(child.size.width, 100.0);
    expect(child.size.height, 500.0);
  });

  test('LimitedBox: parent maxHeight is unconstrained', () {
    final RenderBox child = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints.tightFor(width: 300.0, height: 400.0),
    );
    final RenderBox parent = RenderConstrainedOverflowBox(
      minWidth: 500.0,
      maxWidth: 500.0,
      minHeight: 0.0,
      maxHeight: double.infinity,
      child: RenderLimitedBox(maxWidth: 100.0, maxHeight: 200.0, child: child),
    );
    layout(parent);

    expect(child.size.width, 500.0);
    expect(child.size.height, 200.0);
  });

  test('LimitedBox: no child', () {
    RenderBox box;
    final RenderBox parent = RenderConstrainedOverflowBox(
      minWidth: 10.0,
      maxWidth: 500.0,
      minHeight: 0.0,
      maxHeight: double.infinity,
      child: box = RenderLimitedBox(maxWidth: 100.0, maxHeight: 200.0),
    );
    layout(parent);
    expect(box.size, const Size(10.0, 0.0));

    expect(parent, hasAGoodToStringDeep);
    expect(
      parent.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderConstrainedOverflowBox#00000 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ parentData: <none>\n'
        ' │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        ' │ size: Size(800.0, 600.0)\n'
        ' │ alignment: Alignment.center\n'
        ' │ minWidth: 10.0\n'
        ' │ maxWidth: 500.0\n'
        ' │ minHeight: 0.0\n'
        ' │ maxHeight: Infinity\n'
        ' │ fit: max\n'
        ' │\n'
        ' └─child: RenderLimitedBox#00000 relayoutBoundary=up1 NEEDS-PAINT\n'
        '     parentData: offset=Offset(395.0, 300.0) (can use size)\n'
        '     constraints: BoxConstraints(10.0<=w<=500.0, 0.0<=h<=Infinity)\n'
        '     size: Size(10.0, 0.0)\n'
        '     maxWidth: 100.0\n'
        '     maxHeight: 200.0\n',
      ),
    );
  });

  test('LimitedBox: no child use parent', () {
    RenderBox box;
    final RenderBox parent = RenderConstrainedOverflowBox(
      minWidth: 10.0,
      child: box = RenderLimitedBox(maxWidth: 100.0, maxHeight: 200.0),
    );
    layout(parent);
    expect(box.size, const Size(10.0, 600.0));

    expect(parent, hasAGoodToStringDeep);
    expect(
      parent.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderConstrainedOverflowBox#00000 NEEDS-PAINT NEEDS-COMPOSITING-BITS-UPDATE\n'
        ' │ parentData: <none>\n'
        ' │ constraints: BoxConstraints(w=800.0, h=600.0)\n'
        ' │ size: Size(800.0, 600.0)\n'
        ' │ alignment: Alignment.center\n'
        ' │ minWidth: 10.0\n'
        ' │ maxWidth: use parent maxWidth constraint\n'
        ' │ minHeight: use parent minHeight constraint\n'
        ' │ maxHeight: use parent maxHeight constraint\n'
        ' │ fit: max\n'
        ' │\n'
        ' └─child: RenderLimitedBox#00000 relayoutBoundary=up1 NEEDS-PAINT\n'
        '     parentData: offset=Offset(395.0, 0.0) (can use size)\n'
        '     constraints: BoxConstraints(10.0<=w<=800.0, h=600.0)\n'
        '     size: Size(10.0, 600.0)\n'
        '     maxWidth: 100.0\n'
        '     maxHeight: 200.0\n',
      ),
    );
  });
}
