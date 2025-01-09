// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('OverflowBox control test', (WidgetTester tester) async {
    final GlobalKey inner = GlobalKey();
    await tester.pumpWidget(
      Align(
        alignment: Alignment.bottomRight,
        child: SizedBox(
          width: 10.0,
          height: 20.0,
          child: OverflowBox(
            minWidth: 0.0,
            maxWidth: 100.0,
            minHeight: 0.0,
            maxHeight: 50.0,
            child: Container(key: inner),
          ),
        ),
      ),
    );
    final RenderBox box = inner.currentContext!.findRenderObject()! as RenderBox;
    expect(box.localToGlobal(Offset.zero), equals(const Offset(745.0, 565.0)));
    expect(box.size, equals(const Size(100.0, 50.0)));
  });

  // Adapted from https://github.com/flutter/flutter/issues/129094
  group('when fit is OverflowBoxFit.deferToChild', () {
    group('OverflowBox behavior with long and short content', () {
      for (final bool contentSuperLong in <bool>[false, true]) {
        testWidgets('contentSuperLong=$contentSuperLong', (WidgetTester tester) async {
          final GlobalKey<State<StatefulWidget>> key = GlobalKey();

          final Column child = Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[SizedBox(width: 100, height: contentSuperLong ? 10000 : 100)],
          );

          await tester.pumpWidget(
            Directionality(
              textDirection: TextDirection.ltr,
              child: Stack(
                children: <Widget>[
                  Container(
                    key: key,
                    child: OverflowBox(
                      maxHeight: 1000000,
                      fit: OverflowBoxFit.deferToChild,
                      child: child,
                    ),
                  ),
                ],
              ),
            ),
          );

          expect(tester.getBottomLeft(find.byKey(key)).dy, contentSuperLong ? 600 : 100);
        });
      }
    });

    testWidgets('no child', (WidgetTester tester) async {
      final GlobalKey<State<StatefulWidget>> key = GlobalKey();

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Stack(
            children: <Widget>[
              Container(
                key: key,
                child: const OverflowBox(
                  maxHeight: 1000000,
                  fit: OverflowBoxFit.deferToChild,
                  // no child
                ),
              ),
            ],
          ),
        ),
      );

      expect(tester.getBottomLeft(find.byKey(key)).dy, 0);
    });
  });

  testWidgets('OverflowBox implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const OverflowBox(
      minWidth: 1.0,
      maxWidth: 2.0,
      minHeight: 3.0,
      maxHeight: 4.0,
    ).debugFillProperties(builder);
    final List<String> description =
        builder.properties
            .where((DiagnosticsNode n) => !n.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode n) => n.toString())
            .toList();
    expect(description, <String>[
      'alignment: Alignment.center',
      'minWidth: 1.0',
      'maxWidth: 2.0',
      'minHeight: 3.0',
      'maxHeight: 4.0',
      'fit: max',
    ]);
  });

  testWidgets('SizedOverflowBox alignment', (WidgetTester tester) async {
    final GlobalKey inner = GlobalKey();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Center(
          child: SizedOverflowBox(
            size: const Size(100.0, 100.0),
            alignment: Alignment.topRight,
            child: SizedBox(height: 50.0, width: 50.0, key: inner),
          ),
        ),
      ),
    );
    final RenderBox box = inner.currentContext!.findRenderObject()! as RenderBox;
    expect(box.size, equals(const Size(50.0, 50.0)));
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(
        const Offset(
          (800.0 - 100.0) / 2.0 + 100.0 - 50.0 / 2.0,
          (600.0 - 100.0) / 2.0 + 0.0 + 50.0 / 2.0,
        ),
      ),
    );
  });

  testWidgets('SizedOverflowBox alignment (direction-sensitive)', (WidgetTester tester) async {
    final GlobalKey inner = GlobalKey();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Center(
          child: SizedOverflowBox(
            size: const Size(100.0, 100.0),
            alignment: AlignmentDirectional.bottomStart,
            child: SizedBox(height: 50.0, width: 50.0, key: inner),
          ),
        ),
      ),
    );
    final RenderBox box = inner.currentContext!.findRenderObject()! as RenderBox;
    expect(box.size, equals(const Size(50.0, 50.0)));
    expect(
      box.localToGlobal(box.size.center(Offset.zero)),
      equals(
        const Offset(
          (800.0 - 100.0) / 2.0 + 100.0 - 50.0 / 2.0,
          (600.0 - 100.0) / 2.0 + 100.0 - 50.0 / 2.0,
        ),
      ),
    );
  });
}
