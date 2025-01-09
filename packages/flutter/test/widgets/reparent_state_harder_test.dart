// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

// This is a regression test for https://github.com/flutter/flutter/issues/5588.

class OrderSwitcher extends StatefulWidget {
  const OrderSwitcher({super.key, required this.a, required this.b});

  final Widget a;
  final Widget b;

  @override
  OrderSwitcherState createState() => OrderSwitcherState();
}

class OrderSwitcherState extends State<OrderSwitcher> {
  bool _aFirst = true;

  void switchChildren() {
    setState(() {
      _aFirst = false;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      textDirection: TextDirection.ltr,
      children:
          _aFirst
              ? <Widget>[KeyedSubtree(child: widget.a), widget.b]
              : <Widget>[KeyedSubtree(child: widget.b), widget.a],
    );
  }
}

class DummyStatefulWidget extends StatefulWidget {
  const DummyStatefulWidget(Key? key) : super(key: key);

  @override
  DummyStatefulWidgetState createState() => DummyStatefulWidgetState();
}

class DummyStatefulWidgetState extends State<DummyStatefulWidget> {
  @override
  Widget build(BuildContext context) => const Text('LEAF', textDirection: TextDirection.ltr);
}

class RekeyableDummyStatefulWidgetWrapper extends StatefulWidget {
  const RekeyableDummyStatefulWidgetWrapper({super.key, required this.initialKey});
  final GlobalKey initialKey;
  @override
  RekeyableDummyStatefulWidgetWrapperState createState() =>
      RekeyableDummyStatefulWidgetWrapperState();
}

class RekeyableDummyStatefulWidgetWrapperState extends State<RekeyableDummyStatefulWidgetWrapper> {
  GlobalKey? _key;

  @override
  void initState() {
    super.initState();
    _key = widget.initialKey;
  }

  void _setChild(GlobalKey? value) {
    setState(() {
      _key = value;
    });
  }

  @override
  Widget build(BuildContext context) {
    return DummyStatefulWidget(_key);
  }
}

void main() {
  testWidgets('Handle GlobalKey reparenting in weird orders', (WidgetTester tester) async {
    // This is a bit of a weird test so let's try to explain it a bit.
    //
    // Basically what's happening here is that we have a complicated tree, and
    // in one frame, we change it to a slightly different tree with a specific
    // set of mutations:
    //
    // * The keyA subtree is regrafted to be one level higher, but later than
    //   the keyB subtree.
    // * The keyB subtree is, similarly, moved one level deeper, but earlier, than
    //   the keyA subtree.
    // * The keyD subtree is replaced by the previously earlier and shallower
    //   keyC subtree. This happens during a LayoutBuilder layout callback, so it
    //   happens long after A and B have finished their dance.
    //
    // The net result is that when keyC is moved, it has already been marked
    // dirty from being removed then reinserted into the tree (redundantly, as
    // it turns out, though this isn't known at the time), and has already been
    // visited once by the code that tries to clean nodes (though at that point
    // nothing happens since it isn't in the tree).
    //
    // This test verifies that none of the asserts go off during this dance.

    final GlobalKey<OrderSwitcherState> keyRoot = GlobalKey(debugLabel: 'Root');
    final GlobalKey keyA = GlobalKey(debugLabel: 'A');
    final GlobalKey keyB = GlobalKey(debugLabel: 'B');
    final GlobalKey keyC = GlobalKey(debugLabel: 'C');
    final GlobalKey keyD = GlobalKey(debugLabel: 'D');
    await tester.pumpWidget(
      OrderSwitcher(
        key: keyRoot,
        a: KeyedSubtree(key: keyA, child: RekeyableDummyStatefulWidgetWrapper(initialKey: keyC)),
        b: KeyedSubtree(
          key: keyB,
          child: Builder(
            builder: (BuildContext context) {
              return Builder(
                builder: (BuildContext context) {
                  return Builder(
                    builder: (BuildContext context) {
                      return LayoutBuilder(
                        builder: (BuildContext context, BoxConstraints constraints) {
                          return RekeyableDummyStatefulWidgetWrapper(initialKey: keyD);
                        },
                      );
                    },
                  );
                },
              );
            },
          ),
        ),
      ),
    );

    expect(find.byKey(keyA), findsOneWidget);
    expect(find.byKey(keyB), findsOneWidget);
    expect(find.byKey(keyC), findsOneWidget);
    expect(find.byKey(keyD), findsOneWidget);
    expect(find.byType(RekeyableDummyStatefulWidgetWrapper), findsNWidgets(2));
    expect(find.byType(DummyStatefulWidget), findsNWidgets(2));

    keyRoot.currentState!.switchChildren();
    final List<State> states =
        tester.stateList(find.byType(RekeyableDummyStatefulWidgetWrapper)).toList();
    final RekeyableDummyStatefulWidgetWrapperState a =
        states[0] as RekeyableDummyStatefulWidgetWrapperState;
    a._setChild(null);
    final RekeyableDummyStatefulWidgetWrapperState b =
        states[1] as RekeyableDummyStatefulWidgetWrapperState;
    b._setChild(keyC);
    await tester.pump();

    expect(find.byKey(keyA), findsOneWidget);
    expect(find.byKey(keyB), findsOneWidget);
    expect(find.byKey(keyC), findsOneWidget);
    expect(find.byKey(keyD), findsNothing);
    expect(find.byType(RekeyableDummyStatefulWidgetWrapper), findsNWidgets(2));
    expect(find.byType(DummyStatefulWidget), findsNWidgets(2));
  });
}
