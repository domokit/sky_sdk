// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  group('BlockSemantics', () {
    testWidgets('hides semantic nodes of siblings', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      await tester.pumpWidget(
        Stack(
          textDirection: TextDirection.ltr,
          children: <Widget>[
            Semantics(label: 'layer#1', textDirection: TextDirection.ltr, child: Container()),
            const BlockSemantics(),
            Semantics(label: 'layer#2', textDirection: TextDirection.ltr, child: Container()),
          ],
        ),
      );

      expect(semantics, isNot(includesNodeWith(label: 'layer#1')));

      await tester.pumpWidget(
        Stack(
          textDirection: TextDirection.ltr,
          children: <Widget>[
            Semantics(label: 'layer#1', textDirection: TextDirection.ltr, child: Container()),
          ],
        ),
      );

      expect(semantics, includesNodeWith(label: 'layer#1'));

      semantics.dispose();
    });

    testWidgets('does not hides semantic nodes of siblings outside the current semantic boundary', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Semantics(label: '#1', child: Container()),
              Semantics(
                label: '#2',
                container: true,
                explicitChildNodes: true,
                child: Stack(
                  children: <Widget>[
                    Semantics(label: 'NOT#2.1', child: Container()),
                    Semantics(
                      label: '#2.2',
                      child: BlockSemantics(
                        child: Semantics(container: true, label: '#2.2.1', child: Container()),
                      ),
                    ),
                    Semantics(label: '#2.3', child: Container()),
                  ],
                ),
              ),
              Semantics(label: '#3', child: Container()),
            ],
          ),
        ),
      );

      expect(semantics, includesNodeWith(label: '#1'));
      expect(semantics, includesNodeWith(label: '#2'));
      expect(semantics, isNot(includesNodeWith(label: 'NOT#2.1')));
      expect(semantics, includesNodeWith(label: '#2.2'));
      expect(semantics, includesNodeWith(label: '#2.2.1'));
      expect(semantics, includesNodeWith(label: '#2.3'));
      expect(semantics, includesNodeWith(label: '#3'));

      semantics.dispose();
    });

    testWidgets('node is semantic boundary and blocking previously painted nodes', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final GlobalKey stackKey = GlobalKey();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            key: stackKey,
            children: <Widget>[
              Semantics(label: 'NOT#1', child: Container()),
              BoundaryBlockSemantics(child: Semantics(label: '#2.1', child: Container())),
              Semantics(label: '#3', child: Container()),
            ],
          ),
        ),
      );

      expect(semantics, isNot(includesNodeWith(label: 'NOT#1')));
      expect(semantics, includesNodeWith(label: '#2.1'));
      expect(semantics, includesNodeWith(label: '#3'));

      semantics.dispose();
    });
  });
}

class BoundaryBlockSemantics extends SingleChildRenderObjectWidget {
  const BoundaryBlockSemantics({super.key, required Widget super.child});

  @override
  RenderBoundaryBlockSemantics createRenderObject(BuildContext context) =>
      RenderBoundaryBlockSemantics();
}

class RenderBoundaryBlockSemantics extends RenderProxyBox {
  RenderBoundaryBlockSemantics();

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);

    config
      ..isBlockingSemanticsOfPreviouslyPaintedNodes = true
      ..isSemanticBoundary = true;
  }
}
