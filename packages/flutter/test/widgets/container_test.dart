// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  group('Container control tests:', () {
    final Container container = Container(
      alignment: Alignment.bottomRight,
      padding: const EdgeInsets.all(7.0),
      // uses color, not decoration:
      color: const Color(0xFF00FF00),
      foregroundDecoration: const BoxDecoration(color: Color(0x7F0000FF)),
      width: 53.0,
      height: 76.0,
      constraints: const BoxConstraints(
        minWidth: 50.0,
        maxWidth: 55.0,
        minHeight: 78.0,
        maxHeight: 82.0,
      ),
      margin: const EdgeInsets.all(5.0),
      child: const SizedBox(
        width: 25.0,
        height: 33.0,
        child: DecoratedBox(
          // uses decoration, not color:
          decoration: BoxDecoration(color: Color(0xFFFFFF00)),
        ),
      ),
    );

    testWidgets('paints as expected', (WidgetTester tester) async {
      await tester.pumpWidget(Align(alignment: Alignment.topLeft, child: container));

      final RenderBox box = tester.renderObject(find.byType(Container));
      expect(box, isNotNull);

      expect(
        box,
        paints
          ..rect(rect: const Rect.fromLTWH(5.0, 5.0, 53.0, 78.0), color: const Color(0xFF00FF00))
          ..rect(rect: const Rect.fromLTWH(26.0, 43.0, 25.0, 33.0), color: const Color(0xFFFFFF00))
          ..rect(rect: const Rect.fromLTWH(5.0, 5.0, 53.0, 78.0), color: const Color(0x7F0000FF)),
      );
    });

    group('diagnostics', () {
      testWidgets('has reasonable default diagnostics', (WidgetTester tester) async {
        await tester.pumpWidget(Align(alignment: Alignment.topLeft, child: container));

        final RenderBox box = tester.renderObject(find.byType(Container));

        expect(container, hasOneLineDescription);
        expect(box, hasAGoodToStringDeep);
      });

      testWidgets('has expected info diagnostics', (WidgetTester tester) async {
        await tester.pumpWidget(Align(alignment: Alignment.topLeft, child: container));

        final RenderBox box = tester.renderObject(find.byType(Container));

        expect(
          box.toStringDeep(minLevel: DiagnosticLevel.info, wrapWidth: 640),
          equalsIgnoringHashCodes(
            'RenderPadding#00000 relayoutBoundary=up1\n'
            ' │ parentData: offset=Offset(0.0, 0.0) (can use size)\n'
            ' │ constraints: BoxConstraints(0.0<=w<=800.0, 0.0<=h<=600.0)\n'
            ' │ size: Size(63.0, 88.0)\n'
            ' │ padding: EdgeInsets.all(5.0)\n'
            ' │\n'
            ' └─child: RenderConstrainedBox#00000 relayoutBoundary=up2\n'
            '   │ parentData: offset=Offset(5.0, 5.0) (can use size)\n'
            '   │ constraints: BoxConstraints(0.0<=w<=790.0, 0.0<=h<=590.0)\n'
            '   │ size: Size(53.0, 78.0)\n'
            '   │ additionalConstraints: BoxConstraints(w=53.0, h=78.0)\n'
            '   │\n'
            '   └─child: RenderDecoratedBox#00000\n'
            '     │ parentData: <none> (can use size)\n'
            '     │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '     │ size: Size(53.0, 78.0)\n'
            '     │ decoration: BoxDecoration:\n'
            '     │   color: ${const Color(0x7f0000ff)}\n'
            '     │ configuration: ImageConfiguration(bundle: PlatformAssetBundle#00000(), devicePixelRatio: 3.0, platform: android)\n'
            '     │\n'
            '     └─child: _RenderColoredBox#00000\n'
            '       │ parentData: <none> (can use size)\n'
            '       │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '       │ size: Size(53.0, 78.0)\n'
            '       │ behavior: opaque\n'
            '       │\n'
            '       └─child: RenderPadding#00000\n'
            '         │ parentData: <none> (can use size)\n'
            '         │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '         │ size: Size(53.0, 78.0)\n'
            '         │ padding: EdgeInsets.all(7.0)\n'
            '         │\n'
            '         └─child: RenderPositionedBox#00000\n'
            '           │ parentData: offset=Offset(7.0, 7.0) (can use size)\n'
            '           │ constraints: BoxConstraints(w=39.0, h=64.0)\n'
            '           │ size: Size(39.0, 64.0)\n'
            '           │ alignment: Alignment.bottomRight\n'
            '           │ widthFactor: expand\n'
            '           │ heightFactor: expand\n'
            '           │\n'
            '           └─child: RenderConstrainedBox#00000 relayoutBoundary=up1\n'
            '             │ parentData: offset=Offset(14.0, 31.0) (can use size)\n'
            '             │ constraints: BoxConstraints(0.0<=w<=39.0, 0.0<=h<=64.0)\n'
            '             │ size: Size(25.0, 33.0)\n'
            '             │ additionalConstraints: BoxConstraints(w=25.0, h=33.0)\n'
            '             │\n'
            '             └─child: RenderDecoratedBox#00000\n'
            '                 parentData: <none> (can use size)\n'
            '                 constraints: BoxConstraints(w=25.0, h=33.0)\n'
            '                 size: Size(25.0, 33.0)\n'
            '                 decoration: BoxDecoration:\n'
            '                   color: ${const Color(0xffffff00)}\n'
            '                 configuration: ImageConfiguration(bundle: PlatformAssetBundle#00000(), devicePixelRatio: 3.0, platform: android)\n',
          ),
        );
      });

      testWidgets('has expected debug diagnostics', (WidgetTester tester) async {
        await tester.pumpWidget(Align(alignment: Alignment.topLeft, child: container));

        final RenderBox box = tester.renderObject(find.byType(Container));

        expect(
          // Using the redundant value to ensure the test is explicitly for
          // debug diagnostics, regardless of any changes to the default value.
          // ignore: avoid_redundant_argument_values
          box.toStringDeep(minLevel: DiagnosticLevel.debug, wrapWidth: 600),
          equalsIgnoringHashCodes(
            'RenderPadding#0f959 relayoutBoundary=up1\n'
            ' │ creator: Padding ← Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← _MediaQueryFromView ← '
            '_PipelineOwnerScope ← _ViewScope ← ⋯\n'
            ' │ parentData: offset=Offset(0.0, 0.0) (can use size)\n'
            ' │ constraints: BoxConstraints(0.0<=w<=800.0, 0.0<=h<=600.0)\n'
            ' │ size: Size(63.0, 88.0)\n'
            ' │ padding: EdgeInsets.all(5.0)\n'
            ' │\n'
            ' └─child: RenderConstrainedBox#df6d6 relayoutBoundary=up2\n'
            '   │ creator: ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← FocusTraversalGroup ← MediaQuery ← '
            '_MediaQueryFromView ← _PipelineOwnerScope ← ⋯\n'
            '   │ parentData: offset=Offset(5.0, 5.0) (can use size)\n'
            '   │ constraints: BoxConstraints(0.0<=w<=790.0, 0.0<=h<=590.0)\n'
            '   │ size: Size(53.0, 78.0)\n'
            '   │ additionalConstraints: BoxConstraints(w=53.0, h=78.0)\n'
            '   │\n'
            '   └─child: RenderDecoratedBox#7b39b\n'
            '     │ creator: DecoratedBox ← ConstrainedBox ← Padding ← '
            'Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← _MediaQueryFromView ← ⋯\n'
            '     │ parentData: <none> (can use size)\n'
            '     │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '     │ size: Size(53.0, 78.0)\n'
            '     │ decoration: BoxDecoration:\n'
            '     │   color: ${const Color(0x7f0000ff)}\n'
            '     │ configuration: ImageConfiguration(bundle: '
            'PlatformAssetBundle#fe53b(), devicePixelRatio: 3.0, platform: '
            'android)\n'
            '     │\n'
            '     └─child: _RenderColoredBox#6bd0d\n'
            '       │ creator: ColoredBox ← DecoratedBox ← ConstrainedBox ← '
            'Padding ← Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← ⋯\n'
            '       │ parentData: <none> (can use size)\n'
            '       │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '       │ size: Size(53.0, 78.0)\n'
            '       │ behavior: opaque\n'
            '       │\n'
            '       └─child: RenderPadding#d92f7\n'
            '         │ creator: Padding ← ColoredBox ← DecoratedBox ← '
            'ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← FocusTraversalGroup ← ⋯\n'
            '         │ parentData: <none> (can use size)\n'
            '         │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '         │ size: Size(53.0, 78.0)\n'
            '         │ padding: EdgeInsets.all(7.0)\n'
            '         │\n'
            '         └─child: RenderPositionedBox#aaa32\n'
            '           │ creator: Align ← Padding ← ColoredBox ← '
            'DecoratedBox ← ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← ⋯\n'
            '           │ parentData: offset=Offset(7.0, 7.0) (can use size)\n'
            '           │ constraints: BoxConstraints(w=39.0, h=64.0)\n'
            '           │ size: Size(39.0, 64.0)\n'
            '           │ alignment: Alignment.bottomRight\n'
            '           │ widthFactor: expand\n'
            '           │ heightFactor: expand\n'
            '           │\n'
            '           └─child: RenderConstrainedBox#49805 relayoutBoundary=up1\n'
            '             │ creator: SizedBox ← Align ← Padding ← ColoredBox ← '
            'DecoratedBox ← ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← ⋯\n'
            '             │ parentData: offset=Offset(14.0, 31.0) (can use size)\n'
            '             │ constraints: BoxConstraints(0.0<=w<=39.0, 0.0<=h<=64.0)\n'
            '             │ size: Size(25.0, 33.0)\n'
            '             │ additionalConstraints: BoxConstraints(w=25.0, h=33.0)\n'
            '             │\n'
            '             └─child: RenderDecoratedBox#7843f\n'
            '                 creator: DecoratedBox ← SizedBox ← Align ← '
            'Padding ← ColoredBox ← DecoratedBox ← ConstrainedBox ← Padding ← '
            'Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← ⋯\n'
            '                 parentData: <none> (can use size)\n'
            '                 constraints: BoxConstraints(w=25.0, h=33.0)\n'
            '                 size: Size(25.0, 33.0)\n'
            '                 decoration: BoxDecoration:\n'
            '                   color: ${const Color(0xffffff00)}\n'
            '                 configuration: ImageConfiguration(bundle: '
            'PlatformAssetBundle#fe53b(), devicePixelRatio: 3.0, platform: '
            'android)\n',
          ),
        );
      });

      testWidgets('has expected fine diagnostics', (WidgetTester tester) async {
        await tester.pumpWidget(Align(alignment: Alignment.topLeft, child: container));

        final RenderBox box = tester.renderObject(find.byType(Container));

        expect(
          box.toStringDeep(minLevel: DiagnosticLevel.fine, wrapWidth: 600),
          equalsIgnoringHashCodes(
            'RenderPadding#68510 relayoutBoundary=up1\n'
            ' │ creator: Padding ← Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← _MediaQueryFromView ← '
            '_PipelineOwnerScope ← _ViewScope ← ⋯\n'
            ' │ parentData: offset=Offset(0.0, 0.0) (can use size)\n'
            ' │ constraints: BoxConstraints(0.0<=w<=800.0, 0.0<=h<=600.0)\n'
            ' │ layer: null\n'
            ' │ semantics node: null\n'
            ' │ size: Size(63.0, 88.0)\n'
            ' │ padding: EdgeInsets.all(5.0)\n'
            ' │ textDirection: null\n'
            ' │\n'
            ' └─child: RenderConstrainedBox#69988 relayoutBoundary=up2\n'
            '   │ creator: ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← FocusTraversalGroup ← MediaQuery ← '
            '_MediaQueryFromView ← _PipelineOwnerScope ← ⋯\n'
            '   │ parentData: offset=Offset(5.0, 5.0) (can use size)\n'
            '   │ constraints: BoxConstraints(0.0<=w<=790.0, 0.0<=h<=590.0)\n'
            '   │ layer: null\n'
            '   │ semantics node: null\n'
            '   │ size: Size(53.0, 78.0)\n'
            '   │ additionalConstraints: BoxConstraints(w=53.0, h=78.0)\n'
            '   │\n'
            '   └─child: RenderDecoratedBox#c7049\n'
            '     │ creator: DecoratedBox ← ConstrainedBox ← Padding ← '
            'Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← _MediaQueryFromView ← ⋯\n'
            '     │ parentData: <none> (can use size)\n'
            '     │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '     │ layer: null\n'
            '     │ semantics node: null\n'
            '     │ size: Size(53.0, 78.0)\n'
            '     │ decoration: BoxDecoration:\n'
            '     │   color: ${const Color(0x7f0000ff)}\n'
            '     │   image: null\n'
            '     │   border: null\n'
            '     │   borderRadius: null\n'
            '     │   boxShadow: null\n'
            '     │   gradient: null\n'
            '     │   shape: rectangle\n'
            '     │ configuration: ImageConfiguration(bundle: '
            'PlatformAssetBundle#23b2a(), devicePixelRatio: 3.0, platform: android)\n'
            '     │\n'
            '     └─child: _RenderColoredBox#c8805\n'
            '       │ creator: ColoredBox ← DecoratedBox ← ConstrainedBox ← '
            'Padding ← Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← ⋯\n'
            '       │ parentData: <none> (can use size)\n'
            '       │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '       │ layer: null\n'
            '       │ semantics node: null\n'
            '       │ size: Size(53.0, 78.0)\n'
            '       │ behavior: opaque\n'
            '       │\n'
            '       └─child: RenderPadding#0fab7\n'
            '         │ creator: Padding ← ColoredBox ← DecoratedBox ← '
            'ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← FocusTraversalGroup ← ⋯\n'
            '         │ parentData: <none> (can use size)\n'
            '         │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '         │ layer: null\n'
            '         │ semantics node: null\n'
            '         │ size: Size(53.0, 78.0)\n'
            '         │ padding: EdgeInsets.all(7.0)\n'
            '         │ textDirection: null\n'
            '         │\n'
            '         └─child: RenderPositionedBox#458fb\n'
            '           │ creator: Align ← Padding ← ColoredBox ← '
            'DecoratedBox ← ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← ⋯\n'
            '           │ parentData: offset=Offset(7.0, 7.0) (can use size)\n'
            '           │ constraints: BoxConstraints(w=39.0, h=64.0)\n'
            '           │ layer: null\n'
            '           │ semantics node: null\n'
            '           │ size: Size(39.0, 64.0)\n'
            '           │ alignment: Alignment.bottomRight\n'
            '           │ textDirection: null\n'
            '           │ widthFactor: expand\n'
            '           │ heightFactor: expand\n'
            '           │\n'
            '           └─child: RenderConstrainedBox#16613 relayoutBoundary=up1\n'
            '             │ creator: SizedBox ← Align ← Padding ← ColoredBox ← '
            'DecoratedBox ← ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← ⋯\n'
            '             │ parentData: offset=Offset(14.0, 31.0) (can use size)\n'
            '             │ constraints: BoxConstraints(0.0<=w<=39.0, 0.0<=h<=64.0)\n'
            '             │ layer: null\n'
            '             │ semantics node: null\n'
            '             │ size: Size(25.0, 33.0)\n'
            '             │ additionalConstraints: BoxConstraints(w=25.0, h=33.0)\n'
            '             │\n'
            '             └─child: RenderDecoratedBox#52bc3\n'
            '                 creator: DecoratedBox ← SizedBox ← Align ← '
            'Padding ← ColoredBox ← DecoratedBox ← ConstrainedBox ← Padding ← '
            'Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← ⋯\n'
            '                 parentData: <none> (can use size)\n'
            '                 constraints: BoxConstraints(w=25.0, h=33.0)\n'
            '                 layer: null\n'
            '                 semantics node: null\n'
            '                 size: Size(25.0, 33.0)\n'
            '                 decoration: BoxDecoration:\n'
            '                   color: ${const Color(0xffffff00)}\n'
            '                   image: null\n'
            '                   border: null\n'
            '                   borderRadius: null\n'
            '                   boxShadow: null\n'
            '                   gradient: null\n'
            '                   shape: rectangle\n'
            '                 configuration: ImageConfiguration(bundle: '
            'PlatformAssetBundle#23b2a(), devicePixelRatio: 3.0, platform: android)\n',
          ),
        );
      });

      testWidgets('has expected hidden diagnostics', (WidgetTester tester) async {
        await tester.pumpWidget(Align(alignment: Alignment.topLeft, child: container));

        final RenderBox box = tester.renderObject(find.byType(Container));

        expect(
          box.toStringDeep(minLevel: DiagnosticLevel.hidden, wrapWidth: 600),
          equalsIgnoringHashCodes(
            'RenderPadding#4a353 relayoutBoundary=up1\n'
            ' │ needsCompositing: false\n'
            ' │ creator: Padding ← Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← _MediaQueryFromView ← '
            '_PipelineOwnerScope ← _ViewScope ← ⋯\n'
            ' │ parentData: offset=Offset(0.0, 0.0) (can use size)\n'
            ' │ constraints: BoxConstraints(0.0<=w<=800.0, 0.0<=h<=600.0)\n'
            ' │ layer: null\n'
            ' │ semantics node: null\n'
            ' │ isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            ' │ isSemanticBoundary: false\n'
            ' │ size: Size(63.0, 88.0)\n'
            ' │ padding: EdgeInsets.all(5.0)\n'
            ' │ textDirection: null\n'
            ' │\n'
            ' └─child: RenderConstrainedBox#e3b23 relayoutBoundary=up2\n'
            '   │ needsCompositing: false\n'
            '   │ creator: ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← FocusTraversalGroup ← MediaQuery ← '
            '_MediaQueryFromView ← _PipelineOwnerScope ← ⋯\n'
            '   │ parentData: offset=Offset(5.0, 5.0) (can use size)\n'
            '   │ constraints: BoxConstraints(0.0<=w<=790.0, 0.0<=h<=590.0)\n'
            '   │ layer: null\n'
            '   │ semantics node: null\n'
            '   │ isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            '   │ isSemanticBoundary: false\n'
            '   │ size: Size(53.0, 78.0)\n'
            '   │ additionalConstraints: BoxConstraints(w=53.0, h=78.0)\n'
            '   │\n'
            '   └─child: RenderDecoratedBox#1ca6c\n'
            '     │ needsCompositing: false\n'
            '     │ creator: DecoratedBox ← ConstrainedBox ← Padding ← '
            'Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← _MediaQueryFromView ← ⋯\n'
            '     │ parentData: <none> (can use size)\n'
            '     │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '     │ layer: null\n'
            '     │ semantics node: null\n'
            '     │ isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            '     │ isSemanticBoundary: false\n'
            '     │ size: Size(53.0, 78.0)\n'
            '     │ decoration: BoxDecoration:\n'
            '     │   color: ${const Color(0x7f0000ff)}\n'
            '     │   image: null\n'
            '     │   border: null\n'
            '     │   borderRadius: null\n'
            '     │   boxShadow: null\n'
            '     │   gradient: null\n'
            '     │   shape: rectangle\n'
            '     │ configuration: ImageConfiguration(bundle: '
            'PlatformAssetBundle#fe2c8(), devicePixelRatio: 3.0, platform: '
            'android)\n'
            '     │\n'
            '     └─child: _RenderColoredBox#cff14\n'
            '       │ needsCompositing: false\n'
            '       │ creator: ColoredBox ← DecoratedBox ← ConstrainedBox ← '
            'Padding ← Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← _FocusInheritedScope ← Focus ← '
            'FocusTraversalGroup ← MediaQuery ← ⋯\n'
            '       │ parentData: <none> (can use size)\n'
            '       │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '       │ layer: null\n'
            '       │ semantics node: null\n'
            '       │ isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            '       │ isSemanticBoundary: false\n'
            '       │ size: Size(53.0, 78.0)\n'
            '       │ behavior: opaque\n'
            '       │\n'
            '       └─child: RenderPadding#f6d0f\n'
            '         │ needsCompositing: false\n'
            '         │ creator: Padding ← ColoredBox ← DecoratedBox ← '
            'ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← FocusTraversalGroup ← ⋯\n'
            '         │ parentData: <none> (can use size)\n'
            '         │ constraints: BoxConstraints(w=53.0, h=78.0)\n'
            '         │ layer: null\n'
            '         │ semantics node: null\n'
            '         │ isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            '         │ isSemanticBoundary: false\n'
            '         │ size: Size(53.0, 78.0)\n'
            '         │ padding: EdgeInsets.all(7.0)\n'
            '         │ textDirection: null\n'
            '         │\n'
            '         └─child: RenderPositionedBox#4f7d2\n'
            '           │ needsCompositing: false\n'
            '           │ creator: Align ← Padding ← ColoredBox ← DecoratedBox '
            '← ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← Focus ← ⋯\n'
            '           │ parentData: offset=Offset(7.0, 7.0) (can use size)\n'
            '           │ constraints: BoxConstraints(w=39.0, h=64.0)\n'
            '           │ layer: null\n'
            '           │ semantics node: null\n'
            '           │ isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            '           │ isSemanticBoundary: false\n'
            '           │ size: Size(39.0, 64.0)\n'
            '           │ alignment: Alignment.bottomRight\n'
            '           │ textDirection: null\n'
            '           │ widthFactor: expand\n'
            '           │ heightFactor: expand\n'
            '           │\n'
            '           └─child: RenderConstrainedBox#81408 relayoutBoundary=up1\n'
            '             │ needsCompositing: false\n'
            '             │ creator: SizedBox ← Align ← Padding ← ColoredBox ← '
            'DecoratedBox ← ConstrainedBox ← Padding ← Container ← Align ← '
            '_FocusInheritedScope ← _FocusScopeWithExternalFocusNode ← '
            '_FocusInheritedScope ← ⋯\n'
            '             │ parentData: offset=Offset(14.0, 31.0) (can use size)\n'
            '             │ constraints: BoxConstraints(0.0<=w<=39.0, 0.0<=h<=64.0)\n'
            '             │ layer: null\n'
            '             │ semantics node: null\n'
            '             │ isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            '             │ isSemanticBoundary: false\n'
            '             │ size: Size(25.0, 33.0)\n'
            '             │ additionalConstraints: BoxConstraints(w=25.0, h=33.0)\n'
            '             │\n'
            '             └─child: RenderDecoratedBox#b5693\n'
            '                 needsCompositing: false\n'
            '                 creator: DecoratedBox ← SizedBox ← Align ← '
            'Padding ← ColoredBox ← DecoratedBox ← ConstrainedBox ← Padding ← '
            'Container ← Align ← _FocusInheritedScope ← '
            '_FocusScopeWithExternalFocusNode ← ⋯\n'
            '                 parentData: <none> (can use size)\n'
            '                 constraints: BoxConstraints(w=25.0, h=33.0)\n'
            '                 layer: null\n'
            '                 semantics node: null\n'
            '                 isBlockingSemanticsOfPreviouslyPaintedNodes: false\n'
            '                 isSemanticBoundary: false\n'
            '                 size: Size(25.0, 33.0)\n'
            '                 decoration: BoxDecoration:\n'
            '                   color: ${const Color(0xffffff00)}\n'
            '                   image: null\n'
            '                   border: null\n'
            '                   borderRadius: null\n'
            '                   boxShadow: null\n'
            '                   gradient: null\n'
            '                   shape: rectangle\n'
            '                 configuration: ImageConfiguration(bundle: '
            'PlatformAssetBundle#fe2c8(), devicePixelRatio: 3.0, platform: android)\n',
          ),
        );
      });

      testWidgets('painting error has expected diagnostics', (WidgetTester tester) async {
        await tester.pumpWidget(Align(alignment: Alignment.topLeft, child: container));

        final RenderBox decoratedBox = tester.renderObject(find.byType(DecoratedBox).last);
        final PaintingContext context = _MockPaintingContext();
        late FlutterError error;
        try {
          decoratedBox.paint(context, Offset.zero);
        } on FlutterError catch (e) {
          error = e;
        }
        expect(error, isNotNull);
        expect(
          error.toStringDeep(wrapWidth: 600),
          'FlutterError\n'
          '   BoxDecoration painter had mismatching save and restore calls.\n'
          '   Before painting the decoration, the canvas save count was 0. '
          'After painting it, the canvas save count was 2. Every call to '
          'save() or saveLayer() must be matched by a call to restore().\n'
          '   The decoration was:\n'
          '     BoxDecoration(color: ${const Color(0xffffff00)})\n'
          '   The painter was:\n'
          '     BoxPainter for BoxDecoration(color: '
          '${const Color(0xffffff00)})\n',
        );
      });
    });
  });

  testWidgets('Can be placed in an infinite box', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(children: <Widget>[Container()]),
      ),
    );
  });

  testWidgets('Container transformAlignment', (WidgetTester tester) async {
    final Key key = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Stack(
          children: <Widget>[
            Positioned(
              top: 100.0,
              left: 100.0,
              child: Container(width: 100.0, height: 100.0, color: const Color(0xFF0000FF)),
            ),
            Positioned(
              top: 100.0,
              left: 100.0,
              child: Container(
                width: 100.0,
                height: 100.0,
                key: key,
                transform: Matrix4.diagonal3Values(0.5, 0.5, 1.0),
                transformAlignment: Alignment.centerRight,
                child: Container(color: const Color(0xFF00FFFF)),
              ),
            ),
          ],
        ),
      ),
    );

    final Finder finder = find.byKey(key);

    expect(tester.getSize(finder), equals(const Size(100, 100)));

    expect(tester.getTopLeft(finder), equals(const Offset(100, 100)));
    expect(tester.getTopRight(finder), equals(const Offset(200, 100)));

    expect(tester.getBottomLeft(finder), equals(const Offset(100, 200)));
    expect(tester.getBottomRight(finder), equals(const Offset(200, 200)));
  });

  testWidgets('giving clipBehaviour Clip.None, will not add a ClipPath to the tree', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Container(
        decoration: const BoxDecoration(borderRadius: BorderRadius.all(Radius.circular(1))),
        child: const SizedBox(),
      ),
    );

    expect(find.byType(ClipPath), findsNothing);
  });

  testWidgets('giving clipBehaviour not a Clip.None, will add a ClipPath to the tree', (
    WidgetTester tester,
  ) async {
    final Container container = Container(
      clipBehavior: Clip.hardEdge,
      decoration: const BoxDecoration(borderRadius: BorderRadius.all(Radius.circular(1))),
      child: const SizedBox(),
    );

    await tester.pumpWidget(container);

    expect(find.byType(ClipPath), findsOneWidget);
  });

  testWidgets('getClipPath() works for lots of kinds of decorations', (WidgetTester tester) async {
    Future<void> test(Decoration decoration) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.rtl,
          child: Center(
            child: SizedBox(
              width: 100.0,
              height: 100.0,
              child: RepaintBoundary(
                child: Container(
                  clipBehavior: Clip.hardEdge,
                  decoration: decoration,
                  child: ColoredBox(color: Colors.yellow.withOpacity(0.5)),
                ),
              ),
            ),
          ),
        ),
      );
      await expectLater(
        find.byType(Container),
        matchesGoldenFile('container_test.getClipPath.${decoration.runtimeType}.png'),
      );
    }

    await test(const BoxDecoration());
    await test(const UnderlineTabIndicator());
    await test(const ShapeDecoration(shape: StadiumBorder()));
    await test(const FlutterLogoDecoration());
  });

  testWidgets('Container is hittable only when having decorations', (WidgetTester tester) async {
    bool tapped = false;
    await tester.pumpWidget(
      GestureDetector(
        onTap: () {
          tapped = true;
        },
        child: Container(decoration: const BoxDecoration(color: Colors.black)),
      ),
    );

    await tester.tap(find.byType(Container));
    expect(tapped, true);
    tapped = false;

    await tester.pumpWidget(
      GestureDetector(
        onTap: () {
          tapped = true;
        },
        child: Container(foregroundDecoration: const BoxDecoration(color: Colors.black)),
      ),
    );

    await tester.tap(find.byType(Container));
    expect(tapped, true);
    tapped = false;

    await tester.pumpWidget(
      GestureDetector(
        onTap: () {
          tapped = true;
        },
        child: Container(color: Colors.black),
      ),
    );

    await tester.tap(find.byType(Container));
    expect(tapped, true);
    tapped = false;

    // Everything but color or decorations
    await tester.pumpWidget(
      GestureDetector(
        onTap: () {
          tapped = true;
        },
        child: Center(
          child: Container(
            alignment: Alignment.bottomRight,
            padding: const EdgeInsets.all(2),
            width: 50,
            height: 50,
            margin: const EdgeInsets.all(2),
            transform: Matrix4.rotationZ(1),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(Container), warnIfMissed: false);
    expect(tapped, false);
  });

  testWidgets(
    'Container discards alignment when the child parameter is null and constraints is not Tight',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        Container(
          decoration: const BoxDecoration(borderRadius: BorderRadius.all(Radius.circular(1))),
          alignment: Alignment.centerLeft,
        ),
      );

      expect(find.byType(Align), findsNothing);
    },
  );

  testWidgets('using clipBehaviour and shadow, should not clip the shadow', (
    WidgetTester tester,
  ) async {
    final Container container = Container(
      clipBehavior: Clip.hardEdge,
      decoration: const BoxDecoration(
        borderRadius: BorderRadius.all(Radius.circular(30)),
        color: Colors.red,
        boxShadow: <BoxShadow>[BoxShadow(color: Colors.blue, spreadRadius: 10, blurRadius: 20.0)],
      ),
      child: const SizedBox(width: 50, height: 50),
    );

    await tester.pumpWidget(
      RepaintBoundary(child: Padding(padding: const EdgeInsets.all(30.0), child: container)),
    );

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('container.clipBehaviour.with.shadow.png'),
    );
  });
}

class _MockPaintingContext extends Fake implements PaintingContext {
  @override
  final Canvas canvas = _MockCanvas();
}

class _MockCanvas extends Fake implements Canvas {
  int saveCount = 0;

  @override
  int getSaveCount() {
    return saveCount++;
  }

  @override
  void drawRect(Rect rect, Paint paint) {}
}
