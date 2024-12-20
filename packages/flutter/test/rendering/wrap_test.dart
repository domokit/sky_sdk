// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'rendering_tester.dart';

void main() {
  TestRenderingFlutterBinding.ensureInitialized();

  test('Wrap test; toStringDeep', () {
    final RenderWrap renderWrap = RenderWrap();
    expect(renderWrap, hasAGoodToStringDeep);
    expect(
      renderWrap.toStringDeep(minLevel: DiagnosticLevel.info),
      equalsIgnoringHashCodes(
        'RenderWrap#00000 NEEDS-LAYOUT NEEDS-PAINT DETACHED\n'
        '   parentData: MISSING\n'
        '   constraints: MISSING\n'
        '   size: MISSING\n'
        '   direction: horizontal\n'
        '   alignment: start\n'
        '   spacing: 0.0\n'
        '   runAlignment: start\n'
        '   runSpacing: 0.0\n'
        '   crossAxisAlignment: 0.0\n',
      ),
    );
  });

  test('Compute intrinsic height test', () {
    final List<RenderBox> children = <RenderBox>[
      RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
      ),
      RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
      ),
      RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
      ),
    ];

    final RenderWrap renderWrap = RenderWrap();

    children.forEach(renderWrap.add);

    renderWrap.spacing = 5;
    renderWrap.runSpacing = 5;
    renderWrap.direction = Axis.horizontal;

    expect(renderWrap.computeMaxIntrinsicHeight(245), 165);
    expect(renderWrap.computeMaxIntrinsicHeight(250), 80);
    expect(renderWrap.computeMaxIntrinsicHeight(80), 250);
    expect(renderWrap.computeMaxIntrinsicHeight(79), 250);
    expect(renderWrap.computeMinIntrinsicHeight(245), 165);
    expect(renderWrap.computeMinIntrinsicHeight(250), 80);
    expect(renderWrap.computeMinIntrinsicHeight(80), 250);
    expect(renderWrap.computeMinIntrinsicHeight(79), 250);
  });

  test('Compute intrinsic height test for width-in-height-out children', () {
    const double lineHeight = 15.0;
    final RenderWrap renderWrap = RenderWrap();
    renderWrap.add(
      RenderParagraph(
        const TextSpan(
          text: 'A very very very very very very very very long text',
          style: TextStyle(fontSize: lineHeight),
        ),
        textDirection: TextDirection.ltr,
      ),
    );

    renderWrap.spacing = 0;
    renderWrap.runSpacing = 0;
    renderWrap.direction = Axis.horizontal;

    expect(renderWrap.computeMaxIntrinsicHeight(double.infinity), lineHeight);
    expect(renderWrap.computeMaxIntrinsicHeight(600), 2 * lineHeight);
    expect(renderWrap.computeMaxIntrinsicHeight(300), 3 * lineHeight);
  });

  test('Compute intrinsic width test for height-in-width-out children', () {
    const double lineHeight = 15.0;
    final RenderWrap renderWrap = RenderWrap();
    renderWrap.add(
      // Rotates a width-in-height-out render object to make it height-in-width-out.
      RenderRotatedBox(
        quarterTurns: 1,
        child: RenderParagraph(
          const TextSpan(
            text: 'A very very very very very very very very long text',
            style: TextStyle(fontSize: lineHeight),
          ),
          textDirection: TextDirection.ltr,
        ),
      ),
    );

    renderWrap.spacing = 0;
    renderWrap.runSpacing = 0;
    renderWrap.direction = Axis.vertical;

    expect(renderWrap.computeMaxIntrinsicWidth(double.infinity), lineHeight);
    expect(renderWrap.computeMaxIntrinsicWidth(600), 2 * lineHeight);
    expect(renderWrap.computeMaxIntrinsicWidth(300), 3 * lineHeight);
  });

  test('Compute intrinsic width test', () {
    final List<RenderBox> children = <RenderBox>[
      RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
      ),
      RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
      ),
      RenderConstrainedBox(
        additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
      ),
    ];

    final RenderWrap renderWrap = RenderWrap();

    children.forEach(renderWrap.add);

    renderWrap.spacing = 5;
    renderWrap.runSpacing = 5;
    renderWrap.direction = Axis.vertical;

    expect(renderWrap.computeMaxIntrinsicWidth(245), 165);
    expect(renderWrap.computeMaxIntrinsicWidth(250), 80);
    expect(renderWrap.computeMaxIntrinsicWidth(80), 250);
    expect(renderWrap.computeMaxIntrinsicWidth(79), 250);
    expect(renderWrap.computeMinIntrinsicWidth(245), 165);
    expect(renderWrap.computeMinIntrinsicWidth(250), 80);
    expect(renderWrap.computeMinIntrinsicWidth(80), 250);
    expect(renderWrap.computeMinIntrinsicWidth(79), 250);
  });

  test('Compute intrinsic height for only one run', () {
    final RenderBox child = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
    );

    final RenderWrap renderWrap = RenderWrap();
    renderWrap.add(child);

    renderWrap.spacing = 5;
    renderWrap.runSpacing = 5;
    renderWrap.direction = Axis.horizontal;

    expect(renderWrap.computeMaxIntrinsicHeight(100), 80);
    expect(renderWrap.computeMaxIntrinsicHeight(79), 80);
    expect(renderWrap.computeMaxIntrinsicHeight(80), 80);
    expect(renderWrap.computeMinIntrinsicHeight(100), 80);
    expect(renderWrap.computeMinIntrinsicHeight(79), 80);
    expect(renderWrap.computeMinIntrinsicHeight(80), 80);
  });

  test('Compute intrinsic width for only one run', () {
    final RenderBox child = RenderConstrainedBox(
      additionalConstraints: const BoxConstraints(minWidth: 80, minHeight: 80),
    );

    final RenderWrap renderWrap = RenderWrap();
    renderWrap.add(child);

    renderWrap.spacing = 5;
    renderWrap.runSpacing = 5;
    renderWrap.direction = Axis.vertical;

    expect(renderWrap.computeMaxIntrinsicWidth(100), 80);
    expect(renderWrap.computeMaxIntrinsicWidth(79), 80);
    expect(renderWrap.computeMaxIntrinsicWidth(80), 80);
    expect(renderWrap.computeMinIntrinsicWidth(100), 80);
    expect(renderWrap.computeMinIntrinsicWidth(79), 80);
    expect(renderWrap.computeMinIntrinsicWidth(80), 80);
  });

  test('Wrap respects clipBehavior', () {
    const BoxConstraints viewport = BoxConstraints(maxHeight: 100.0, maxWidth: 100.0);
    final TestClipPaintingContext context = TestClipPaintingContext();

    for (final Clip? clip in <Clip?>[null, ...Clip.values]) {
      final RenderWrap wrap;
      switch (clip) {
        case Clip.none:
        case Clip.hardEdge:
        case Clip.antiAlias:
        case Clip.antiAliasWithSaveLayer:
          wrap = RenderWrap(
            textDirection: TextDirection.ltr,
            children: <RenderBox>[box200x200],
            clipBehavior: clip!,
          );
        case null:
          wrap = RenderWrap(textDirection: TextDirection.ltr, children: <RenderBox>[box200x200]);
      }
      layout(
        wrap,
        constraints: viewport,
        phase: EnginePhase.composite,
        onErrors: expectNoFlutterErrors,
      );
      context.paintChild(wrap, Offset.zero);
      // By default, clipBehavior should be Clip.none
      expect(context.clipBehavior, equals(clip ?? Clip.none));
    }
  });
}
