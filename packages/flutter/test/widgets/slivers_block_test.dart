// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

Future<void> test(WidgetTester tester, double offset) {
  final ViewportOffset viewportOffset = ViewportOffset.fixed(offset);
  addTearDown(viewportOffset.dispose);
  return tester.pumpWidget(
    Directionality(
      textDirection: TextDirection.ltr,
      child: Viewport(
        offset: viewportOffset,
        slivers: <Widget>[
          SliverList(
            delegate: SliverChildListDelegate(const <Widget>[
              SizedBox(height: 400.0, child: Text('a')),
              SizedBox(height: 400.0, child: Text('b')),
              SizedBox(height: 400.0, child: Text('c')),
              SizedBox(height: 400.0, child: Text('d')),
              SizedBox(height: 400.0, child: Text('e')),
            ]),
          ),
        ],
      ),
    ),
  );
}

Future<void> testWithConstChildDelegate(WidgetTester tester, double offset) {
  final ViewportOffset viewportOffset = ViewportOffset.fixed(offset);
  addTearDown(viewportOffset.dispose);
  return tester.pumpWidget(
    Directionality(
      textDirection: TextDirection.ltr,
      child: Viewport(
        offset: viewportOffset,
        slivers: const <Widget>[
          SliverList(
            delegate: SliverChildListDelegate.fixed(<Widget>[
              SizedBox(height: 400.0, child: Text('a')),
              SizedBox(height: 400.0, child: Text('b')),
              SizedBox(height: 400.0, child: Text('c')),
              SizedBox(height: 400.0, child: Text('d')),
              SizedBox(height: 400.0, child: Text('e')),
            ]),
          ),
        ],
      ),
    ),
  );
}

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
  testWidgets('Viewport+SliverBlock basic test', (WidgetTester tester) async {
    await test(tester, 0.0);
    expect(
      tester.renderObject<RenderBox>(find.byType(Viewport)).size,
      equals(const Size(800.0, 600.0)),
    );
    verify(tester, <Offset>[Offset.zero, const Offset(0.0, 400.0)], 'ab');

    await test(tester, 200.0);
    verify(tester, <Offset>[const Offset(0.0, -200.0), const Offset(0.0, 200.0)], 'ab');

    await test(tester, 600.0);
    verify(tester, <Offset>[const Offset(0.0, -200.0), const Offset(0.0, 200.0)], 'bc');

    await test(tester, 900.0);
    verify(tester, <Offset>[const Offset(0.0, -100.0), const Offset(0.0, 300.0)], 'cd');

    await test(tester, 200.0);
    verify(tester, <Offset>[const Offset(0.0, -200.0), const Offset(0.0, 200.0)], 'ab');
  });

  testWidgets('Viewport+SliverBlock basic test with constant SliverChildListDelegate', (
    WidgetTester tester,
  ) async {
    await testWithConstChildDelegate(tester, 0.0);
    expect(
      tester.renderObject<RenderBox>(find.byType(Viewport)).size,
      equals(const Size(800.0, 600.0)),
    );
    verify(tester, <Offset>[Offset.zero, const Offset(0.0, 400.0)], 'ab');

    await testWithConstChildDelegate(tester, 200.0);
    verify(tester, <Offset>[const Offset(0.0, -200.0), const Offset(0.0, 200.0)], 'ab');

    await testWithConstChildDelegate(tester, 600.0);
    verify(tester, <Offset>[const Offset(0.0, -200.0), const Offset(0.0, 200.0)], 'bc');

    await testWithConstChildDelegate(tester, 900.0);
    verify(tester, <Offset>[const Offset(0.0, -100.0), const Offset(0.0, 300.0)], 'cd');

    await testWithConstChildDelegate(tester, 200.0);
    verify(tester, <Offset>[const Offset(0.0, -200.0), const Offset(0.0, 200.0)], 'ab');
  });

  testWidgets('Viewport with GlobalKey reparenting', (WidgetTester tester) async {
    final Key key1 = GlobalKey();
    final ViewportOffset offset = ViewportOffset.zero();
    addTearDown(offset.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(<Widget>[
                const SizedBox(height: 251.0, child: Text('a')),
                const SizedBox(height: 252.0, child: Text('b')),
                SizedBox(key: key1, height: 253.0, child: const Text('c')),
              ]),
            ),
          ],
        ),
      ),
    );
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 251.0),
      const Offset(0.0, 503.0),
    ], 'abc');
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(<Widget>[
                SizedBox(key: key1, height: 253.0, child: const Text('c')),
                const SizedBox(height: 251.0, child: Text('a')),
                const SizedBox(height: 252.0, child: Text('b')),
              ]),
            ),
          ],
        ),
      ),
    );
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 253.0),
      const Offset(0.0, 504.0),
    ], 'cab');
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(<Widget>[
                const SizedBox(height: 251.0, child: Text('a')),
                SizedBox(key: key1, height: 253.0, child: const Text('c')),
                const SizedBox(height: 252.0, child: Text('b')),
              ]),
            ),
          ],
        ),
      ),
    );
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 251.0),
      const Offset(0.0, 504.0),
    ], 'acb');
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(const <Widget>[
                SizedBox(height: 251.0, child: Text('a')),
                SizedBox(height: 252.0, child: Text('b')),
              ]),
            ),
          ],
        ),
      ),
    );
    verify(tester, <Offset>[Offset.zero, const Offset(0.0, 251.0)], 'ab');
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(<Widget>[
                const SizedBox(height: 251.0, child: Text('a')),
                SizedBox(key: key1, height: 253.0, child: const Text('c')),
                const SizedBox(height: 252.0, child: Text('b')),
              ]),
            ),
          ],
        ),
      ),
    );
    verify(tester, <Offset>[
      Offset.zero,
      const Offset(0.0, 251.0),
      const Offset(0.0, 504.0),
    ], 'acb');
  });

  testWidgets('Viewport overflow clipping of SliverToBoxAdapter', (WidgetTester tester) async {
    final ViewportOffset offset1 = ViewportOffset.zero();
    addTearDown(offset1.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset1,
          slivers: const <Widget>[
            SliverToBoxAdapter(child: SizedBox(height: 400.0, child: Text('a'))),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), isNot(paints..clipRect()));

    final ViewportOffset offset2 = ViewportOffset.fixed(100.0);
    addTearDown(offset2.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset2,
          slivers: const <Widget>[
            SliverToBoxAdapter(child: SizedBox(height: 400.0, child: Text('a'))),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), paints..clipRect());

    final ViewportOffset offset3 = ViewportOffset.fixed(100.0);
    addTearDown(offset3.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset3,
          slivers: const <Widget>[
            SliverToBoxAdapter(child: SizedBox(height: 4000.0, child: Text('a'))),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), paints..clipRect());

    final ViewportOffset offset4 = ViewportOffset.zero();
    addTearDown(offset4.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset4,
          slivers: const <Widget>[
            SliverToBoxAdapter(child: SizedBox(height: 4000.0, child: Text('a'))),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), paints..clipRect());
  });

  testWidgets('Viewport overflow clipping of SliverBlock', (WidgetTester tester) async {
    final ViewportOffset offset1 = ViewportOffset.zero();
    addTearDown(offset1.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset1,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(const <Widget>[
                SizedBox(height: 400.0, child: Text('a')),
              ]),
            ),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), isNot(paints..clipRect()));

    final ViewportOffset offset2 = ViewportOffset.fixed(100.0);
    addTearDown(offset2.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset2,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(const <Widget>[
                SizedBox(height: 400.0, child: Text('a')),
              ]),
            ),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), paints..clipRect());

    final ViewportOffset offset3 = ViewportOffset.fixed(100.0);
    addTearDown(offset3.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset3,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(const <Widget>[
                SizedBox(height: 4000.0, child: Text('a')),
              ]),
            ),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), paints..clipRect());

    final ViewportOffset offset4 = ViewportOffset.zero();
    addTearDown(offset4.dispose);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Viewport(
          offset: offset4,
          slivers: <Widget>[
            SliverList(
              delegate: SliverChildListDelegate(const <Widget>[
                SizedBox(height: 4000.0, child: Text('a')),
              ]),
            ),
          ],
        ),
      ),
    );

    expect(find.byType(Viewport), paints..clipRect());
  });
}
