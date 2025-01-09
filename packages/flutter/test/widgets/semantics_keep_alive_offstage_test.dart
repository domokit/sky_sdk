// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets(
    'Un-layouted RenderObject in keep alive offstage area do not crash semantics compiler',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/20313.

      final SemanticsTester semantics = SemanticsTester(tester);

      const String initialLabel = 'Foo';
      const double bottomScrollOffset = 3000.0;

      final ScrollController controller = ScrollController(initialScrollOffset: bottomScrollOffset);
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        _buildTestWidget(extraPadding: false, text: initialLabel, controller: controller),
      );
      await tester.pumpAndSettle();

      // The ProblemWidget has been instantiated (it is on screen).
      expect(tester.widgetList(find.widgetWithText(ProblemWidget, initialLabel)), hasLength(1));
      expect(semantics, includesNodeWith(label: initialLabel));

      controller.jumpTo(0.0);
      await tester.pumpAndSettle();

      // The ProblemWidget is not on screen...
      expect(tester.widgetList(find.widgetWithText(ProblemWidget, initialLabel)), hasLength(0));
      // ... but still in the tree as offstage.
      expect(
        tester.widgetList(find.widgetWithText(ProblemWidget, initialLabel, skipOffstage: false)),
        hasLength(1),
      );
      expect(semantics, isNot(includesNodeWith(label: initialLabel)));

      // Introduce a new Padding widget to offstage subtree that will not get its
      // size calculated because it's offstage.
      await tester.pumpWidget(
        _buildTestWidget(extraPadding: true, text: initialLabel, controller: controller),
      );
      final RenderPadding renderPadding = tester.renderObject(
        find.byKey(paddingWidget, skipOffstage: false),
      );
      expect(renderPadding.hasSize, isFalse);
      expect(semantics, isNot(includesNodeWith(label: initialLabel)));

      // Change the semantics of the offstage ProblemWidget without crashing.
      const String newLabel = 'Bar';
      expect(newLabel, isNot(equals(initialLabel)));
      await tester.pumpWidget(
        _buildTestWidget(extraPadding: true, text: newLabel, controller: controller),
      );

      // The label has changed.
      expect(
        tester.widgetList(find.widgetWithText(ProblemWidget, initialLabel, skipOffstage: false)),
        hasLength(0),
      );
      expect(
        tester.widgetList(find.widgetWithText(ProblemWidget, newLabel, skipOffstage: false)),
        hasLength(1),
      );
      expect(semantics, isNot(includesNodeWith(label: initialLabel)));
      expect(semantics, isNot(includesNodeWith(label: newLabel)));

      // Bringing the offstage node back on the screen produces correct semantics tree.
      controller.jumpTo(bottomScrollOffset);
      await tester.pumpAndSettle();

      expect(tester.widgetList(find.widgetWithText(ProblemWidget, initialLabel)), hasLength(0));
      expect(tester.widgetList(find.widgetWithText(ProblemWidget, newLabel)), hasLength(1));
      expect(semantics, isNot(includesNodeWith(label: initialLabel)));
      expect(semantics, includesNodeWith(label: newLabel));

      semantics.dispose();
    },
  );
}

final Key paddingWidget = GlobalKey();

Widget _buildTestWidget({
  required bool extraPadding,
  required String text,
  required ScrollController controller,
}) {
  return MaterialApp(
    home: Scaffold(
      body: Column(
        children: <Widget>[
          Expanded(child: Container()),
          SizedBox(
            height: 500.0,
            child: ListView(
              controller: controller,
              children: List<Widget>.generate(10, (int i) {
                return Container(
                  color: i.isEven ? Colors.red : Colors.blue,
                  height: 250.0,
                  child: Text('Item $i'),
                );
              })..add(ProblemWidget(extraPadding: extraPadding, text: text)),
            ),
          ),
          Expanded(child: Container()),
        ],
      ),
    ),
  );
}

class ProblemWidget extends StatefulWidget {
  const ProblemWidget({super.key, required this.extraPadding, required this.text});

  final bool extraPadding;
  final String text;

  @override
  State<ProblemWidget> createState() => ProblemWidgetState();
}

class ProblemWidgetState extends State<ProblemWidget>
    with AutomaticKeepAliveClientMixin<ProblemWidget> {
  @override
  Widget build(BuildContext context) {
    super.build(context);
    Widget child = Semantics(container: true, child: Text(widget.text));
    if (widget.extraPadding) {
      child = Semantics(
        container: true,
        child: Padding(key: paddingWidget, padding: const EdgeInsets.all(20.0), child: child),
      );
    }
    return child;
  }

  @override
  bool get wantKeepAlive => true;
}
