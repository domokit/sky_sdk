// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

int globalGeneration = 0;

class GenerationText extends StatefulWidget {
  const GenerationText(this.value, {super.key});
  final int value;
  @override
  State<GenerationText> createState() => _GenerationTextState();
}

class _GenerationTextState extends State<GenerationText> {
  _GenerationTextState() : generation = globalGeneration;
  final int generation;
  @override
  Widget build(BuildContext context) =>
      Text('${widget.value}:$generation ', textDirection: TextDirection.ltr);
}

// Creates a SliverList with `keys.length` children and each child having a key from `keys` and a text of `key:generation`.
// The generation is increased with every call to this method.
Future<void> test(WidgetTester tester, double offset, List<int> keys) {
  final ViewportOffset viewportOffset = ViewportOffset.fixed(offset);
  addTearDown(viewportOffset.dispose);
  globalGeneration += 1;
  return tester.pumpWidget(
    Directionality(
      textDirection: TextDirection.ltr,
      child: Viewport(
        cacheExtent: 0.0,
        offset: viewportOffset,
        slivers: <Widget>[
          SliverList(
            delegate: SliverChildListDelegate(
              keys.map<Widget>((int key) {
                return SizedBox(
                  key: GlobalObjectKey(key),
                  height: 100.0,
                  child: GenerationText(key),
                );
              }).toList(),
            ),
          ),
        ],
      ),
    ),
  );
}

// `answerKey`: Expected offsets of visible SliverList children in global coordinate system.
// `text`: A space-separated list of expected `key:generation` pairs for the visible SliverList children.
void verify(WidgetTester tester, List<Offset> answerKey, String text) {
  final List<Offset> testAnswers =
      tester
          .renderObjectList<RenderBox>(find.byType(SizedBox))
          .map<Offset>((RenderBox target) => target.localToGlobal(Offset.zero))
          .toList();
  expect(testAnswers, equals(answerKey));
  final String foundText = tester
      .widgetList<Text>(find.byType(Text))
      .map<String>((Text widget) => widget.data!)
      .reduce((String value, String element) => value + element);
  expect(foundText, equals(text));
}

void main() {
  testWidgets('Viewport+SliverBlock with GlobalKey reparenting', (WidgetTester tester) async {
    await test(tester, 0.0, <int>[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 100.0),
      const Offset(0.0, 200.0),
      const Offset(0.0, 300.0),
      const Offset(0.0, 400.0),
      const Offset(0.0, 500.0),
    ], '1:1 2:1 3:1 4:1 5:1 6:1 ');
    // gen 2 - flipping the order:
    await test(tester, 0.0, <int>[9, 8, 7, 6, 5, 4, 3, 2, 1]);
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 100.0),
      const Offset(0.0, 200.0),
      const Offset(0.0, 300.0),
      const Offset(0.0, 400.0),
      const Offset(0.0, 500.0),
    ], '9:2 8:2 7:2 6:1 5:1 4:1 ');
    // gen 3 - flipping the order back:
    await test(tester, 0.0, <int>[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 100.0),
      const Offset(0.0, 200.0),
      const Offset(0.0, 300.0),
      const Offset(0.0, 400.0),
      const Offset(0.0, 500.0),
    ], '1:3 2:3 3:3 4:1 5:1 6:1 ');
    // gen 4 - removal:
    await test(tester, 0.0, <int>[1, 2, 3, 5, 6, 7, 8, 9]);
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 100.0),
      const Offset(0.0, 200.0),
      const Offset(0.0, 300.0),
      const Offset(0.0, 400.0),
      const Offset(0.0, 500.0),
    ], '1:3 2:3 3:3 5:1 6:1 7:4 ');
    // gen 5 - insertion:
    await test(tester, 0.0, <int>[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 100.0),
      const Offset(0.0, 200.0),
      const Offset(0.0, 300.0),
      const Offset(0.0, 400.0),
      const Offset(0.0, 500.0),
    ], '1:3 2:3 3:3 4:5 5:1 6:1 ');
    // gen 6 - adjacent reordering:
    await test(tester, 0.0, <int>[1, 2, 3, 5, 4, 6, 7, 8, 9]);
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 100.0),
      const Offset(0.0, 200.0),
      const Offset(0.0, 300.0),
      const Offset(0.0, 400.0),
      const Offset(0.0, 500.0),
    ], '1:3 2:3 3:3 5:1 4:5 6:1 ');
    // gen 7 - scrolling:
    await test(tester, 120.0, <int>[1, 2, 3, 5, 4, 6, 7, 8, 9]);
    verify(tester, <Offset>[
      const Offset(0.0, -20.0),
      const Offset(0.0, 80.0),
      const Offset(0.0, 180.0),
      const Offset(0.0, 280.0),
      const Offset(0.0, 380.0),
      const Offset(0.0, 480.0),
      const Offset(0.0, 580.0),
    ], '2:3 3:3 5:1 4:5 6:1 7:7 8:7 ');
  });
}
