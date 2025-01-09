// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

// A simple "flat" InheritedModel: the data model is just 3 integer
// valued fields: a, b, c.
class ABCModel extends InheritedModel<String> {
  const ABCModel({super.key, this.a, this.b, this.c, this.aspects, required super.child});

  final int? a;
  final int? b;
  final int? c;

  // The aspects (fields) of this model that widgets can depend on with
  // inheritFrom.
  //
  // This property is null by default, which means that the model supports
  // all 3 fields.
  final Set<String>? aspects;

  @override
  bool isSupportedAspect(Object aspect) {
    return aspects == null || aspects!.contains(aspect);
  }

  @override
  bool updateShouldNotify(ABCModel old) {
    return !setEquals<String>(aspects, old.aspects) || a != old.a || b != old.b || c != old.c;
  }

  @override
  bool updateShouldNotifyDependent(ABCModel old, Set<String> dependencies) {
    return !setEquals<String>(aspects, old.aspects) ||
        (a != old.a && dependencies.contains('a')) ||
        (b != old.b && dependencies.contains('b')) ||
        (c != old.c && dependencies.contains('c'));
  }

  static ABCModel? of(BuildContext context, {String? fieldName}) {
    return InheritedModel.inheritFrom<ABCModel>(context, aspect: fieldName);
  }
}

class ShowABCField extends StatefulWidget {
  const ShowABCField({super.key, required this.fieldName});

  final String fieldName;

  @override
  State<ShowABCField> createState() => _ShowABCFieldState();
}

class _ShowABCFieldState extends State<ShowABCField> {
  int _buildCount = 0;

  @override
  Widget build(BuildContext context) {
    final ABCModel abc = ABCModel.of(context, fieldName: widget.fieldName)!;
    final int? value = widget.fieldName == 'a' ? abc.a : (widget.fieldName == 'b' ? abc.b : abc.c);
    return Text('${widget.fieldName}: $value [${_buildCount++}]');
  }
}

void main() {
  testWidgets('InheritedModel basics', (WidgetTester tester) async {
    int a = 0;
    int b = 1;
    int c = 2;

    final Widget abcPage = StatefulBuilder(
      builder: (BuildContext context, StateSetter setState) {
        const Widget showA = ShowABCField(fieldName: 'a');
        const Widget showB = ShowABCField(fieldName: 'b');
        const Widget showC = ShowABCField(fieldName: 'c');

        // Unconditionally depends on the ABCModel: rebuilt when any
        // aspect of the model changes.
        final Widget showABC = Builder(
          builder: (BuildContext context) {
            final ABCModel abc = ABCModel.of(context)!;
            return Text('a: ${abc.a} b: ${abc.b} c: ${abc.c}');
          },
        );

        return Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return ABCModel(
                a: a,
                b: b,
                c: c,
                child: Center(
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: <Widget>[
                      showA,
                      showB,
                      showC,
                      showABC,
                      ElevatedButton(
                        child: const Text('Increment a'),
                        onPressed: () {
                          // Rebuilds the ABCModel which triggers a rebuild
                          // of showA because showA depends on the 'a' aspect
                          // of the ABCModel.
                          setState(() {
                            a += 1;
                          });
                        },
                      ),
                      ElevatedButton(
                        child: const Text('Increment b'),
                        onPressed: () {
                          // Rebuilds the ABCModel which triggers a rebuild
                          // of showB because showB depends on the 'b' aspect
                          // of the ABCModel.
                          setState(() {
                            b += 1;
                          });
                        },
                      ),
                      ElevatedButton(
                        child: const Text('Increment c'),
                        onPressed: () {
                          // Rebuilds the ABCModel which triggers a rebuild
                          // of showC because showC depends on the 'c' aspect
                          // of the ABCModel.
                          setState(() {
                            c += 1;
                          });
                        },
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
        );
      },
    );

    await tester.pumpWidget(MaterialApp(home: abcPage));

    expect(find.text('a: 0 [0]'), findsOneWidget);
    expect(find.text('b: 1 [0]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 0 b: 1 c: 2'), findsOneWidget);

    await tester.tap(find.text('Increment a'));
    await tester.pumpAndSettle();
    // Verify that field 'a' was incremented, but only the showA
    // and showABC widgets were rebuilt.
    expect(find.text('a: 1 [1]'), findsOneWidget);
    expect(find.text('b: 1 [0]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 1 b: 1 c: 2'), findsOneWidget);

    // Verify that field 'a' was incremented, but only the showA
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment a'));
    await tester.pumpAndSettle();
    expect(find.text('a: 2 [2]'), findsOneWidget);
    expect(find.text('b: 1 [0]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 2 b: 1 c: 2'), findsOneWidget);

    // Verify that field 'b' was incremented, but only the showB
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment b'));
    await tester.pumpAndSettle();
    expect(find.text('a: 2 [2]'), findsOneWidget);
    expect(find.text('b: 2 [1]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 2 b: 2 c: 2'), findsOneWidget);

    // Verify that field 'c' was incremented, but only the showC
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment c'));
    await tester.pumpAndSettle();
    expect(find.text('a: 2 [2]'), findsOneWidget);
    expect(find.text('b: 2 [1]'), findsOneWidget);
    expect(find.text('c: 3 [1]'), findsOneWidget);
    expect(find.text('a: 2 b: 2 c: 3'), findsOneWidget);
  });

  testWidgets('Looking up an non existent InheritedModel ancestor returns null', (
    WidgetTester tester,
  ) async {
    ABCModel? inheritedModel;

    await tester.pumpWidget(
      Builder(
        builder: (BuildContext context) {
          inheritedModel = InheritedModel.inheritFrom(context);
          return Container();
        },
      ),
    );
    // Shouldn't crash first of all.

    expect(inheritedModel, null);
  });

  testWidgets('Inner InheritedModel shadows the outer one', (WidgetTester tester) async {
    int a = 0;
    int b = 1;
    int c = 2;

    // Same as in abcPage in the "InheritedModel basics" test except:
    // there are two ABCModels and the inner model's "a" and "b"
    // properties shadow (override) the outer model. Further complicating
    // matters: the inner model only supports the model's "a" aspect,
    // so showB and showC will depend on the outer model.
    final Widget abcPage = StatefulBuilder(
      builder: (BuildContext context, StateSetter setState) {
        const Widget showA = ShowABCField(fieldName: 'a');
        const Widget showB = ShowABCField(fieldName: 'b');
        const Widget showC = ShowABCField(fieldName: 'c');

        // Unconditionally depends on the closest ABCModel ancestor.
        // Which is the inner model, for which b,c are null.
        final Widget showABC = Builder(
          builder: (BuildContext context) {
            final ABCModel abc = ABCModel.of(context)!;
            return Text(
              'a: ${abc.a} b: ${abc.b} c: ${abc.c}',
              style: Theme.of(context).textTheme.titleLarge,
            );
          },
        );

        return Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return ABCModel(
                // The "outer" model
                a: a,
                b: b,
                c: c,
                child: ABCModel(
                  // The "inner" model
                  a: 100 + a,
                  b: 100 + b,
                  aspects: const <String>{'a'},
                  child: Center(
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: <Widget>[
                        showA,
                        showB,
                        showC,
                        const SizedBox(height: 24.0),
                        showABC,
                        const SizedBox(height: 24.0),
                        ElevatedButton(
                          child: const Text('Increment a'),
                          onPressed: () {
                            setState(() {
                              a += 1;
                            });
                          },
                        ),
                        ElevatedButton(
                          child: const Text('Increment b'),
                          onPressed: () {
                            setState(() {
                              b += 1;
                            });
                          },
                        ),
                        ElevatedButton(
                          child: const Text('Increment c'),
                          onPressed: () {
                            setState(() {
                              c += 1;
                            });
                          },
                        ),
                      ],
                    ),
                  ),
                ),
              );
            },
          ),
        );
      },
    );

    await tester.pumpWidget(MaterialApp(home: abcPage));
    expect(find.text('a: 100 [0]'), findsOneWidget);
    expect(find.text('b: 1 [0]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 100 b: 101 c: null'), findsOneWidget);

    await tester.tap(find.text('Increment a'));
    await tester.pumpAndSettle();
    // Verify that field 'a' was incremented, but only the showA
    // and showABC widgets were rebuilt.
    expect(find.text('a: 101 [1]'), findsOneWidget);
    expect(find.text('b: 1 [0]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 101 b: 101 c: null'), findsOneWidget);

    await tester.tap(find.text('Increment a'));
    await tester.pumpAndSettle();
    // Verify that field 'a' was incremented, but only the showA
    // and showABC widgets were rebuilt.
    expect(find.text('a: 102 [2]'), findsOneWidget);
    expect(find.text('b: 1 [0]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 102 b: 101 c: null'), findsOneWidget);

    // Verify that field 'b' was incremented, but only the showB
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment b'));
    await tester.pumpAndSettle();
    expect(find.text('a: 102 [2]'), findsOneWidget);
    expect(find.text('b: 2 [1]'), findsOneWidget);
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 102 b: 102 c: null'), findsOneWidget);

    // Verify that field 'c' was incremented, but only the showC
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment c'));
    await tester.pumpAndSettle();
    expect(find.text('a: 102 [2]'), findsOneWidget);
    expect(find.text('b: 2 [1]'), findsOneWidget);
    expect(find.text('c: 3 [1]'), findsOneWidget);
    expect(find.text('a: 102 b: 102 c: null'), findsOneWidget);
  });

  testWidgets('InheritedModel inner models supported aspect change', (WidgetTester tester) async {
    int a = 0;
    int b = 1;
    int c = 2;
    Set<String>? innerModelAspects = <String>{'a'};

    // Same as in abcPage in the "Inner InheritedModel shadows the outer one"
    // test except: the "Add b aspect" changes adds 'b' to the set of
    // aspects supported by the inner model.
    final Widget abcPage = StatefulBuilder(
      builder: (BuildContext context, StateSetter setState) {
        const Widget showA = ShowABCField(fieldName: 'a');
        const Widget showB = ShowABCField(fieldName: 'b');
        const Widget showC = ShowABCField(fieldName: 'c');

        // Unconditionally depends on the closest ABCModel ancestor.
        // Which is the inner model, for which b,c are null.
        final Widget showABC = Builder(
          builder: (BuildContext context) {
            final ABCModel abc = ABCModel.of(context)!;
            return Text(
              'a: ${abc.a} b: ${abc.b} c: ${abc.c}',
              style: Theme.of(context).textTheme.titleLarge,
            );
          },
        );

        return Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return ABCModel(
                // The "outer" model
                a: a,
                b: b,
                c: c,
                child: ABCModel(
                  // The "inner" model
                  a: 100 + a,
                  b: 100 + b,
                  aspects: innerModelAspects,
                  child: Center(
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: <Widget>[
                        showA,
                        showB,
                        showC,
                        const SizedBox(height: 24.0),
                        showABC,
                        const SizedBox(height: 24.0),
                        ElevatedButton(
                          child: const Text('Increment a'),
                          onPressed: () {
                            setState(() {
                              a += 1;
                            });
                          },
                        ),
                        ElevatedButton(
                          child: const Text('Increment b'),
                          onPressed: () {
                            setState(() {
                              b += 1;
                            });
                          },
                        ),
                        ElevatedButton(
                          child: const Text('Increment c'),
                          onPressed: () {
                            setState(() {
                              c += 1;
                            });
                          },
                        ),
                        ElevatedButton(
                          child: const Text('rebuild'),
                          onPressed: () {
                            setState(() {
                              // Rebuild both models
                            });
                          },
                        ),
                      ],
                    ),
                  ),
                ),
              );
            },
          ),
        );
      },
    );

    innerModelAspects = <String>{'a'};
    await tester.pumpWidget(MaterialApp(home: abcPage));
    expect(find.text('a: 100 [0]'), findsOneWidget); // showA depends on the inner model
    expect(find.text('b: 1 [0]'), findsOneWidget); // showB depends on the outer model
    expect(find.text('c: 2 [0]'), findsOneWidget);
    expect(find.text('a: 100 b: 101 c: null'), findsOneWidget); // inner model's a, b, c

    innerModelAspects = <String>{'a', 'b'};
    await tester.tap(find.text('rebuild'));
    await tester.pumpAndSettle();
    expect(
      find.text('a: 100 [1]'),
      findsOneWidget,
    ); // rebuilt showA still depend on the inner model
    expect(find.text('b: 101 [1]'), findsOneWidget); // rebuilt showB now depends on the inner model
    expect(find.text('c: 2 [1]'), findsOneWidget); // rebuilt showC still depends on the outer model
    expect(find.text('a: 100 b: 101 c: null'), findsOneWidget); // inner model's a, b, c

    // Verify that field 'a' was incremented, but only the showA
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment a'));
    await tester.pumpAndSettle();
    expect(
      find.text('a: 101 [2]'),
      findsOneWidget,
    ); // rebuilt showA still depends on the inner model
    expect(find.text('b: 101 [1]'), findsOneWidget);
    expect(find.text('c: 2 [1]'), findsOneWidget);
    expect(find.text('a: 101 b: 101 c: null'), findsOneWidget);

    // Verify that field 'b' was incremented, but only the showB
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment b'));
    await tester.pumpAndSettle();
    expect(
      find.text('a: 101 [2]'),
      findsOneWidget,
    ); // rebuilt showB still depends on the inner model
    expect(find.text('b: 102 [2]'), findsOneWidget);
    expect(find.text('c: 2 [1]'), findsOneWidget);
    expect(find.text('a: 101 b: 102 c: null'), findsOneWidget);

    // Verify that field 'c' was incremented, but only the showC
    // and showABC widgets were rebuilt.
    await tester.tap(find.text('Increment c'));
    await tester.pumpAndSettle();
    expect(find.text('a: 101 [2]'), findsOneWidget);
    expect(find.text('b: 102 [2]'), findsOneWidget);
    expect(find.text('c: 3 [2]'), findsOneWidget); // rebuilt showC still depends on the outer model
    expect(find.text('a: 101 b: 102 c: null'), findsOneWidget);

    innerModelAspects = <String>{'a', 'b', 'c'};
    await tester.tap(find.text('rebuild'));
    await tester.pumpAndSettle();
    expect(
      find.text('a: 101 [3]'),
      findsOneWidget,
    ); // rebuilt showA still depend on the inner model
    expect(
      find.text('b: 102 [3]'),
      findsOneWidget,
    ); // rebuilt showB still depends on the inner model
    expect(
      find.text('c: null [3]'),
      findsOneWidget,
    ); // rebuilt showC now depends on the inner model
    expect(find.text('a: 101 b: 102 c: null'), findsOneWidget); // inner model's a, b, c

    // Now the inner model supports no aspects
    innerModelAspects = <String>{};
    await tester.tap(find.text('rebuild'));
    await tester.pumpAndSettle();
    expect(find.text('a: 1 [4]'), findsOneWidget); // rebuilt showA now depends on the outer model
    expect(find.text('b: 2 [4]'), findsOneWidget); // rebuilt showB now depends on the outer model
    expect(find.text('c: 3 [4]'), findsOneWidget); // rebuilt showC now depends on the outer model
    expect(find.text('a: 101 b: 102 c: null'), findsOneWidget); // inner model's a, b, c

    // Now the inner model supports all aspects
    innerModelAspects = null;
    await tester.tap(find.text('rebuild'));
    await tester.pumpAndSettle();
    expect(find.text('a: 101 [5]'), findsOneWidget); // rebuilt showA now depends on the inner model
    expect(find.text('b: 102 [5]'), findsOneWidget); // rebuilt showB now depends on the inner model
    expect(
      find.text('c: null [5]'),
      findsOneWidget,
    ); // rebuilt showC now depends on the inner model
    expect(find.text('a: 101 b: 102 c: null'), findsOneWidget); // inner model's a, b, c
  });
}
