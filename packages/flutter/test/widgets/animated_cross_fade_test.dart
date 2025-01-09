// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('AnimatedCrossFade test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: AnimatedCrossFade(
            firstChild: SizedBox(width: 100.0, height: 100.0),
            secondChild: SizedBox(width: 200.0, height: 200.0),
            duration: Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showFirst,
          ),
        ),
      ),
    );

    expect(find.byType(FadeTransition), findsNWidgets(2));
    RenderBox box = tester.renderObject(find.byType(AnimatedCrossFade));
    expect(box.size.width, equals(100.0));
    expect(box.size.height, equals(100.0));

    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: AnimatedCrossFade(
            firstChild: SizedBox(width: 100.0, height: 100.0),
            secondChild: SizedBox(width: 200.0, height: 200.0),
            duration: Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showSecond,
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));

    expect(find.byType(FadeTransition), findsNWidgets(2));
    box = tester.renderObject(find.byType(AnimatedCrossFade));
    expect(box.size.width, equals(150.0));
    expect(box.size.height, equals(150.0));
  });

  testWidgets('AnimatedCrossFade test showSecond', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: AnimatedCrossFade(
            firstChild: SizedBox(width: 100.0, height: 100.0),
            secondChild: SizedBox(width: 200.0, height: 200.0),
            duration: Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showSecond,
          ),
        ),
      ),
    );

    expect(find.byType(FadeTransition), findsNWidgets(2));
    final RenderBox box = tester.renderObject(find.byType(AnimatedCrossFade));
    expect(box.size.width, equals(200.0));
    expect(box.size.height, equals(200.0));
  });

  testWidgets('AnimatedCrossFade alignment (VISUAL)', (WidgetTester tester) async {
    final Key firstKey = UniqueKey();
    final Key secondKey = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: AnimatedCrossFade(
            alignment: Alignment.bottomRight,
            firstChild: SizedBox(key: firstKey, width: 100.0, height: 100.0),
            secondChild: SizedBox(key: secondKey, width: 200.0, height: 200.0),
            duration: const Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showFirst,
          ),
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: AnimatedCrossFade(
            alignment: Alignment.bottomRight,
            firstChild: SizedBox(key: firstKey, width: 100.0, height: 100.0),
            secondChild: SizedBox(key: secondKey, width: 200.0, height: 200.0),
            duration: const Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showSecond,
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));

    final RenderBox box1 = tester.renderObject(find.byKey(firstKey));
    final RenderBox box2 = tester.renderObject(find.byKey(secondKey));
    expect(box1.localToGlobal(Offset.zero), const Offset(275.0, 175.0));
    expect(box2.localToGlobal(Offset.zero), const Offset(275.0, 175.0));
  });

  testWidgets('AnimatedCrossFade alignment (LTR)', (WidgetTester tester) async {
    final Key firstKey = UniqueKey();
    final Key secondKey = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: AnimatedCrossFade(
            alignment: AlignmentDirectional.bottomEnd,
            firstChild: SizedBox(key: firstKey, width: 100.0, height: 100.0),
            secondChild: SizedBox(key: secondKey, width: 200.0, height: 200.0),
            duration: const Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showFirst,
          ),
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: AnimatedCrossFade(
            alignment: AlignmentDirectional.bottomEnd,
            firstChild: SizedBox(key: firstKey, width: 100.0, height: 100.0),
            secondChild: SizedBox(key: secondKey, width: 200.0, height: 200.0),
            duration: const Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showSecond,
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));

    final RenderBox box1 = tester.renderObject(find.byKey(firstKey));
    final RenderBox box2 = tester.renderObject(find.byKey(secondKey));
    expect(box1.localToGlobal(Offset.zero), const Offset(275.0, 175.0));
    expect(box2.localToGlobal(Offset.zero), const Offset(275.0, 175.0));
  });

  testWidgets('AnimatedCrossFade alignment (RTL)', (WidgetTester tester) async {
    final Key firstKey = UniqueKey();
    final Key secondKey = UniqueKey();

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Center(
          child: AnimatedCrossFade(
            alignment: AlignmentDirectional.bottomEnd,
            firstChild: SizedBox(key: firstKey, width: 100.0, height: 100.0),
            secondChild: SizedBox(key: secondKey, width: 200.0, height: 200.0),
            duration: const Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showFirst,
          ),
        ),
      ),
    );

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Center(
          child: AnimatedCrossFade(
            alignment: AlignmentDirectional.bottomEnd,
            firstChild: SizedBox(key: firstKey, width: 100.0, height: 100.0),
            secondChild: SizedBox(key: secondKey, width: 200.0, height: 200.0),
            duration: const Duration(milliseconds: 200),
            crossFadeState: CrossFadeState.showSecond,
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));

    final RenderBox box1 = tester.renderObject(find.byKey(firstKey));
    final RenderBox box2 = tester.renderObject(find.byKey(secondKey));
    expect(box1.localToGlobal(Offset.zero), const Offset(325.0, 175.0));
    expect(box2.localToGlobal(Offset.zero), const Offset(325.0, 175.0));
  });

  Widget crossFadeWithWatcher({bool towardsSecond = false}) {
    return Directionality(
      textDirection: TextDirection.ltr,
      child: AnimatedCrossFade(
        firstChild: const _TickerWatchingWidget(),
        secondChild: Container(),
        crossFadeState: towardsSecond ? CrossFadeState.showSecond : CrossFadeState.showFirst,
        duration: const Duration(milliseconds: 50),
      ),
    );
  }

  testWidgets('AnimatedCrossFade preserves widget state', (WidgetTester tester) async {
    await tester.pumpWidget(crossFadeWithWatcher());

    _TickerWatchingWidgetState findState() => tester.state(find.byType(_TickerWatchingWidget));
    final _TickerWatchingWidgetState state = findState();

    await tester.pumpWidget(crossFadeWithWatcher(towardsSecond: true));
    for (int i = 0; i < 3; i += 1) {
      await tester.pump(const Duration(milliseconds: 25));
      expect(findState(), same(state));
    }
  });

  testWidgets('AnimatedCrossFade switches off TickerMode and semantics on faded out widget', (
    WidgetTester tester,
  ) async {
    ExcludeSemantics findSemantics() {
      return tester.widget(
        find.descendant(
          of: find.byKey(const ValueKey<CrossFadeState>(CrossFadeState.showFirst)),
          matching: find.byType(ExcludeSemantics),
        ),
      );
    }

    await tester.pumpWidget(crossFadeWithWatcher());

    final _TickerWatchingWidgetState state = tester.state(find.byType(_TickerWatchingWidget));
    expect(state.ticker.muted, false);
    expect(findSemantics().excluding, false);

    await tester.pumpWidget(crossFadeWithWatcher(towardsSecond: true));
    for (int i = 0; i < 2; i += 1) {
      await tester.pump(const Duration(milliseconds: 25));
      // Animations are kept alive in the middle of cross-fade
      expect(state.ticker.muted, false);
      // Semantics are turned off immediately on the widget that's fading out
      expect(findSemantics().excluding, true);
    }

    // In the final state both animations and semantics should be off on the
    // widget that's faded out.
    await tester.pump(const Duration(milliseconds: 25));
    expect(state.ticker.muted, true);
    expect(findSemantics().excluding, true);
  });

  testWidgets('AnimatedCrossFade.layoutBuilder', (WidgetTester tester) async {
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedCrossFade(
          firstChild: Text('AAA', textDirection: TextDirection.ltr),
          secondChild: Text('BBB', textDirection: TextDirection.ltr),
          crossFadeState: CrossFadeState.showFirst,
          duration: Duration(milliseconds: 50),
        ),
      ),
    );
    expect(find.text('AAA'), findsOneWidget);
    expect(find.text('BBB'), findsOneWidget);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedCrossFade(
          firstChild: const Text('AAA', textDirection: TextDirection.ltr),
          secondChild: const Text('BBB', textDirection: TextDirection.ltr),
          crossFadeState: CrossFadeState.showFirst,
          duration: const Duration(milliseconds: 50),
          layoutBuilder: (Widget a, Key aKey, Widget b, Key bKey) => a,
        ),
      ),
    );
    expect(find.text('AAA'), findsOneWidget);
    expect(find.text('BBB'), findsNothing);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedCrossFade(
          firstChild: const Text('AAA', textDirection: TextDirection.ltr),
          secondChild: const Text('BBB', textDirection: TextDirection.ltr),
          crossFadeState: CrossFadeState.showSecond,
          duration: const Duration(milliseconds: 50),
          layoutBuilder: (Widget a, Key aKey, Widget b, Key bKey) => a,
        ),
      ),
    );
    expect(find.text('BBB'), findsOneWidget);
    expect(find.text('AAA'), findsNothing);
  });

  testWidgets('AnimatedCrossFade test focus', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedCrossFade(
          firstChild: TextButton(onPressed: () {}, child: const Text('AAA')),
          secondChild: TextButton(onPressed: () {}, child: const Text('BBB')),
          crossFadeState: CrossFadeState.showFirst,
          duration: const Duration(milliseconds: 50),
        ),
      ),
    );

    final FocusNode visibleNode = Focus.of(tester.element(find.text('AAA')), scopeOk: true);
    visibleNode.requestFocus();
    await tester.pump();
    expect(visibleNode.hasPrimaryFocus, isTrue);

    final FocusNode hiddenNode = Focus.of(tester.element(find.text('BBB')), scopeOk: true);
    hiddenNode.requestFocus();
    await tester.pump();
    expect(hiddenNode.hasPrimaryFocus, isFalse);
  });

  testWidgets('AnimatedCrossFade bottom child can have focus', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: AnimatedCrossFade(
          firstChild: TextButton(onPressed: () {}, child: const Text('AAA')),
          secondChild: TextButton(onPressed: () {}, child: const Text('BBB')),
          crossFadeState: CrossFadeState.showFirst,
          duration: const Duration(milliseconds: 50),
          excludeBottomFocus: false,
        ),
      ),
    );

    final FocusNode visibleNode = Focus.of(tester.element(find.text('AAA')), scopeOk: true);
    visibleNode.requestFocus();
    await tester.pump();
    expect(visibleNode.hasPrimaryFocus, isTrue);

    final FocusNode hiddenNode = Focus.of(tester.element(find.text('BBB')), scopeOk: true);
    hiddenNode.requestFocus();
    await tester.pump();
    expect(hiddenNode.hasPrimaryFocus, isTrue);
  });

  testWidgets('AnimatedCrossFade second child do not receive touch events', (
    WidgetTester tester,
  ) async {
    int numberOfTouchEventNoticed = 0;

    Future<void> buildAnimatedFrame(CrossFadeState crossFadeState) {
      return tester.pumpWidget(
        SizedBox(
          width: 300,
          height: 600,
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: AnimatedCrossFade(
              firstChild: const Text('AAA'),
              secondChild: TextButton(
                style: TextButton.styleFrom(minimumSize: const Size(double.infinity, 600)),
                onPressed: () {
                  numberOfTouchEventNoticed++;
                },
                child: const Text('BBB'),
              ),
              crossFadeState: crossFadeState,
              duration: const Duration(milliseconds: 50),
            ),
          ),
        ),
      );
    }

    Future<void> touchSecondButton() async {
      final TestGesture gestureTouchSecondButton = await tester.startGesture(
        const Offset(150, 300),
      );

      return gestureTouchSecondButton.up();
    }

    await buildAnimatedFrame(CrossFadeState.showSecond);

    await touchSecondButton();
    expect(numberOfTouchEventNoticed, 1);

    await buildAnimatedFrame(CrossFadeState.showFirst);
    await touchSecondButton();
    await touchSecondButton();

    expect(numberOfTouchEventNoticed, 1);
  });
}

class _TickerWatchingWidget extends StatefulWidget {
  const _TickerWatchingWidget();

  @override
  State<StatefulWidget> createState() => _TickerWatchingWidgetState();
}

class _TickerWatchingWidgetState extends State<_TickerWatchingWidget>
    with SingleTickerProviderStateMixin {
  late Ticker ticker;

  @override
  void initState() {
    super.initState();
    ticker = createTicker((_) {})..start();
  }

  @override
  Widget build(BuildContext context) => Container();

  @override
  void dispose() {
    ticker.dispose();
    super.dispose();
  }
}
