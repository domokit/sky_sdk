// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import 'test_widgets.dart';

class TestInherited extends InheritedWidget {
  const TestInherited({super.key, required super.child, this.shouldNotify = true});

  final bool shouldNotify;

  @override
  bool updateShouldNotify(InheritedWidget oldWidget) {
    return shouldNotify;
  }
}

class ValueInherited extends InheritedWidget {
  const ValueInherited({super.key, required super.child, required this.value});

  final int value;

  @override
  bool updateShouldNotify(ValueInherited oldWidget) => value != oldWidget.value;
}

class ExpectFail extends StatefulWidget {
  const ExpectFail(this.onError, {super.key});
  final VoidCallback onError;

  @override
  ExpectFailState createState() => ExpectFailState();
}

class ExpectFailState extends State<ExpectFail> {
  @override
  void initState() {
    super.initState();
    try {
      context.dependOnInheritedWidgetOfExactType<TestInherited>(); // should fail
    } catch (e) {
      widget.onError();
    }
  }

  @override
  Widget build(BuildContext context) => Container();
}

class ChangeNotifierInherited extends InheritedNotifier<ChangeNotifier> {
  const ChangeNotifierInherited({super.key, required super.child, super.notifier});
}

class ThemedCard extends SingleChildRenderObjectWidget {
  const ThemedCard({super.key}) : super(child: const SizedBox.expand());

  @override
  RenderPhysicalShape createRenderObject(BuildContext context) {
    final CardThemeData cardTheme = CardTheme.of(context);

    return RenderPhysicalShape(
      clipper: ShapeBorderClipper(shape: cardTheme.shape ?? const RoundedRectangleBorder()),
      clipBehavior: cardTheme.clipBehavior ?? Clip.antiAlias,
      color: cardTheme.color ?? Colors.white,
      elevation: cardTheme.elevation ?? 0.0,
      shadowColor: cardTheme.shadowColor ?? Colors.black,
    );
  }

  @override
  void updateRenderObject(BuildContext context, RenderPhysicalShape renderObject) {
    final CardThemeData cardTheme = CardTheme.of(context);

    renderObject
      ..clipper = ShapeBorderClipper(shape: cardTheme.shape ?? const RoundedRectangleBorder())
      ..clipBehavior = cardTheme.clipBehavior ?? Clip.antiAlias
      ..color = cardTheme.color ?? Colors.white
      ..elevation = cardTheme.elevation ?? 0.0
      ..shadowColor = cardTheme.shadowColor ?? Colors.black;
  }
}

void main() {
  testWidgets('Inherited notifies dependents', (WidgetTester tester) async {
    final List<TestInherited> log = <TestInherited>[];

    final Builder builder = Builder(
      builder: (BuildContext context) {
        log.add(context.dependOnInheritedWidgetOfExactType<TestInherited>()!);
        return Container();
      },
    );

    final TestInherited first = TestInherited(child: builder);
    await tester.pumpWidget(first);

    expect(log, equals(<TestInherited>[first]));

    final TestInherited second = TestInherited(shouldNotify: false, child: builder);
    await tester.pumpWidget(second);

    expect(log, equals(<TestInherited>[first]));

    final TestInherited third = TestInherited(child: builder);
    await tester.pumpWidget(third);

    expect(log, equals(<TestInherited>[first, third]));
  });

  testWidgets('Update inherited when reparenting state', (WidgetTester tester) async {
    final GlobalKey globalKey = GlobalKey();
    final List<TestInherited> log = <TestInherited>[];

    TestInherited build() {
      return TestInherited(
        key: UniqueKey(),
        child: Container(
          key: globalKey,
          child: Builder(
            builder: (BuildContext context) {
              log.add(context.dependOnInheritedWidgetOfExactType<TestInherited>()!);
              return Container();
            },
          ),
        ),
      );
    }

    final TestInherited first = build();
    await tester.pumpWidget(first);

    expect(log, equals(<TestInherited>[first]));

    final TestInherited second = build();
    await tester.pumpWidget(second);

    expect(log, equals(<TestInherited>[first, second]));
  });

  testWidgets('Update inherited when removing node', (WidgetTester tester) async {
    final List<String> log = <String>[];

    await tester.pumpWidget(
      ValueInherited(
        value: 1,
        child: FlipWidget(
          left: ValueInherited(
            value: 2,
            child: ValueInherited(
              value: 3,
              child: Builder(
                builder: (BuildContext context) {
                  final ValueInherited v =
                      context.dependOnInheritedWidgetOfExactType<ValueInherited>()!;
                  log.add('a: ${v.value}');
                  return const Text('', textDirection: TextDirection.ltr);
                },
              ),
            ),
          ),
          right: ValueInherited(
            value: 2,
            child: Builder(
              builder: (BuildContext context) {
                final ValueInherited v =
                    context.dependOnInheritedWidgetOfExactType<ValueInherited>()!;
                log.add('b: ${v.value}');
                return const Text('', textDirection: TextDirection.ltr);
              },
            ),
          ),
        ),
      ),
    );

    expect(log, equals(<String>['a: 3']));
    log.clear();

    await tester.pump();

    expect(log, equals(<String>[]));
    log.clear();

    flipStatefulWidget(tester);
    await tester.pump();

    expect(log, equals(<String>['b: 2']));
    log.clear();

    flipStatefulWidget(tester);
    await tester.pump();

    expect(log, equals(<String>['a: 3']));
    log.clear();
  });

  testWidgets('Update inherited when removing node and child has global key', (
    WidgetTester tester,
  ) async {
    final List<String> log = <String>[];

    final Key key = GlobalKey();

    await tester.pumpWidget(
      ValueInherited(
        value: 1,
        child: FlipWidget(
          left: ValueInherited(
            value: 2,
            child: ValueInherited(
              value: 3,
              child: Container(
                key: key,
                child: Builder(
                  builder: (BuildContext context) {
                    final ValueInherited v =
                        context.dependOnInheritedWidgetOfExactType<ValueInherited>()!;
                    log.add('a: ${v.value}');
                    return const Text('', textDirection: TextDirection.ltr);
                  },
                ),
              ),
            ),
          ),
          right: ValueInherited(
            value: 2,
            child: Container(
              key: key,
              child: Builder(
                builder: (BuildContext context) {
                  final ValueInherited v =
                      context.dependOnInheritedWidgetOfExactType<ValueInherited>()!;
                  log.add('b: ${v.value}');
                  return const Text('', textDirection: TextDirection.ltr);
                },
              ),
            ),
          ),
        ),
      ),
    );

    expect(log, equals(<String>['a: 3']));
    log.clear();

    await tester.pump();

    expect(log, equals(<String>[]));
    log.clear();

    flipStatefulWidget(tester);
    await tester.pump();

    expect(log, equals(<String>['b: 2']));
    log.clear();

    flipStatefulWidget(tester);
    await tester.pump();

    expect(log, equals(<String>['a: 3']));
    log.clear();
  });

  testWidgets('Update inherited when removing node and child has global key with constant child', (
    WidgetTester tester,
  ) async {
    final List<int> log = <int>[];

    final Key key = GlobalKey();

    final Widget child = Builder(
      builder: (BuildContext context) {
        final ValueInherited v = context.dependOnInheritedWidgetOfExactType<ValueInherited>()!;
        log.add(v.value);
        return const Text('', textDirection: TextDirection.ltr);
      },
    );

    await tester.pumpWidget(
      ValueInherited(
        value: 1,
        child: FlipWidget(
          left: ValueInherited(
            value: 2,
            child: ValueInherited(value: 3, child: Container(key: key, child: child)),
          ),
          right: ValueInherited(value: 2, child: Container(key: key, child: child)),
        ),
      ),
    );

    expect(log, equals(<int>[3]));
    log.clear();

    await tester.pump();

    expect(log, equals(<int>[]));
    log.clear();

    flipStatefulWidget(tester);
    await tester.pump();

    expect(log, equals(<int>[2]));
    log.clear();

    flipStatefulWidget(tester);
    await tester.pump();

    expect(log, equals(<int>[3]));
    log.clear();
  });

  testWidgets(
    'Update inherited when removing node and child has global key with constant child, minimised',
    (WidgetTester tester) async {
      final List<int> log = <int>[];

      final Widget child = Builder(
        key: GlobalKey(),
        builder: (BuildContext context) {
          final ValueInherited v = context.dependOnInheritedWidgetOfExactType<ValueInherited>()!;
          log.add(v.value);
          return const Text('', textDirection: TextDirection.ltr);
        },
      );

      await tester.pumpWidget(
        ValueInherited(
          value: 2,
          child: FlipWidget(left: ValueInherited(value: 3, child: child), right: child),
        ),
      );

      expect(log, equals(<int>[3]));
      log.clear();

      await tester.pump();

      expect(log, equals(<int>[]));
      log.clear();

      flipStatefulWidget(tester);
      await tester.pump();

      expect(log, equals(<int>[2]));
      log.clear();

      flipStatefulWidget(tester);
      await tester.pump();

      expect(log, equals(<int>[3]));
      log.clear();
    },
  );

  testWidgets(
    'Inherited widget notifies descendants when descendant previously failed to find a match',
    (WidgetTester tester) async {
      int? inheritedValue = -1;

      final Widget inner = Container(
        key: GlobalKey(),
        child: Builder(
          builder: (BuildContext context) {
            final ValueInherited? widget =
                context.dependOnInheritedWidgetOfExactType<ValueInherited>();
            inheritedValue = widget?.value;
            return Container();
          },
        ),
      );

      await tester.pumpWidget(inner);
      expect(inheritedValue, isNull);

      inheritedValue = -2;
      await tester.pumpWidget(ValueInherited(value: 3, child: inner));
      expect(inheritedValue, equals(3));
    },
  );

  testWidgets(
    "Inherited widget doesn't notify descendants when descendant did not previously fail to find a match and had no dependencies",
    (WidgetTester tester) async {
      int buildCount = 0;

      final Widget inner = Container(
        key: GlobalKey(),
        child: Builder(
          builder: (BuildContext context) {
            buildCount += 1;
            return Container();
          },
        ),
      );

      await tester.pumpWidget(inner);
      expect(buildCount, equals(1));

      await tester.pumpWidget(ValueInherited(value: 3, child: inner));
      expect(buildCount, equals(1));
    },
  );

  testWidgets(
    'Inherited widget does notify descendants when descendant did not previously fail to find a match but did have other dependencies',
    (WidgetTester tester) async {
      int buildCount = 0;

      final Widget inner = Container(
        key: GlobalKey(),
        child: TestInherited(
          shouldNotify: false,
          child: Builder(
            builder: (BuildContext context) {
              context.dependOnInheritedWidgetOfExactType<TestInherited>();
              buildCount += 1;
              return Container();
            },
          ),
        ),
      );

      await tester.pumpWidget(inner);
      expect(buildCount, equals(1));

      await tester.pumpWidget(ValueInherited(value: 3, child: inner));
      expect(buildCount, equals(2));
    },
  );

  testWidgets("BuildContext.getInheritedWidgetOfExactType doesn't create a dependency", (
    WidgetTester tester,
  ) async {
    int buildCount = 0;
    final GlobalKey<void> inheritedKey = GlobalKey();
    final ChangeNotifier notifier = ChangeNotifier();
    addTearDown(notifier.dispose);

    final Widget builder = Builder(
      builder: (BuildContext context) {
        expect(
          context.getInheritedWidgetOfExactType<ChangeNotifierInherited>(),
          equals(inheritedKey.currentWidget),
        );
        buildCount += 1;
        return Container();
      },
    );

    final Widget inner = ChangeNotifierInherited(
      key: inheritedKey,
      notifier: notifier,
      child: builder,
    );

    await tester.pumpWidget(inner);
    expect(buildCount, equals(1));
    notifier.notifyListeners();
    await tester.pumpWidget(inner);
    expect(buildCount, equals(1));
  });

  testWidgets('initState() dependency on Inherited asserts', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/5491
    bool exceptionCaught = false;

    final TestInherited parent = TestInherited(
      child: ExpectFail(() {
        exceptionCaught = true;
      }),
    );
    await tester.pumpWidget(parent);

    expect(exceptionCaught, isTrue);
  });

  testWidgets('InheritedNotifier', (WidgetTester tester) async {
    int buildCount = 0;
    final ChangeNotifier notifier = ChangeNotifier();
    addTearDown(notifier.dispose);

    final Widget builder = Builder(
      builder: (BuildContext context) {
        context.dependOnInheritedWidgetOfExactType<ChangeNotifierInherited>();
        buildCount += 1;
        return Container();
      },
    );

    final Widget inner = ChangeNotifierInherited(notifier: notifier, child: builder);
    await tester.pumpWidget(inner);
    expect(buildCount, equals(1));

    await tester.pumpWidget(inner);
    expect(buildCount, equals(1));

    await tester.pump();
    expect(buildCount, equals(1));

    notifier.notifyListeners();
    await tester.pump();
    expect(buildCount, equals(2));

    await tester.pumpWidget(inner);
    expect(buildCount, equals(2));

    await tester.pumpWidget(ChangeNotifierInherited(child: builder));
    expect(buildCount, equals(3));
  });

  testWidgets('InheritedWidgets can trigger RenderObject updates', (WidgetTester tester) async {
    CardThemeData cardThemeData = const CardThemeData(color: Colors.white);
    late StateSetter setState;

    // Verifies that the "themed card" is rendered
    // with the appropriate inherited theme data.
    void expectCardToMatchTheme() {
      final RenderPhysicalShape renderShape = tester.renderObject(find.byType(ThemedCard));

      if (cardThemeData.color != null) {
        expect(renderShape.color, cardThemeData.color);
      }
      if (cardThemeData.elevation != null) {
        expect(renderShape.elevation, cardThemeData.elevation);
      }
      if (cardThemeData.shadowColor != null) {
        expect(renderShape.shadowColor, cardThemeData.shadowColor);
      }
      if (cardThemeData.shape != null) {
        final CustomClipper<Path>? clipper = renderShape.clipper;
        expect(clipper, isA<ShapeBorderClipper>());
        expect((clipper! as ShapeBorderClipper).shape, cardThemeData.shape);
      }
      if (cardThemeData.clipBehavior != null) {
        expect(renderShape.clipBehavior, cardThemeData.clipBehavior);
      }
    }

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter stateSetter) {
          setState = stateSetter;
          return Theme(data: ThemeData(cardTheme: cardThemeData), child: const ThemedCard());
        },
      ),
    );
    expectCardToMatchTheme();

    setState(() {
      cardThemeData = const CardThemeData(
        shape: BeveledRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(20))),
      );
    });
    await tester.pump();
    expectCardToMatchTheme();

    setState(() {
      cardThemeData = const CardThemeData(clipBehavior: Clip.hardEdge);
    });
    await tester.pump();
    expectCardToMatchTheme();

    setState(() {
      cardThemeData = const CardThemeData(
        elevation: 5.0,
        shadowColor: Colors.blueGrey,
        shape: ContinuousRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(8.0))),
        clipBehavior: Clip.antiAliasWithSaveLayer,
      );
    });
    await tester.pump();
    expectCardToMatchTheme();
  });
}
