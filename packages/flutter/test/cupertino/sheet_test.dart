// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/navigator_utils.dart';

void main() {
  testWidgets('Sheet route does not cover the whole screen', (WidgetTester tester) async {
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    Navigator.push<void>(
                      scaffoldKey.currentContext!,
                      CupertinoSheetRoute<void>(
                        builder: (BuildContext context) {
                          return const CupertinoPageScaffold(child: Text('Page 2'));
                        },
                      ),
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsOneWidget);
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      greaterThan(0.0),
    );
  });

  testWidgets('Previous route moves slight downward when sheet route is pushed', (
    WidgetTester tester,
  ) async {
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Column(
            children: <Widget>[
              const Text('Page 1'),
              CupertinoButton(
                onPressed: () {
                  Navigator.push<void>(
                    scaffoldKey.currentContext!,
                    CupertinoSheetRoute<void>(
                      builder: (BuildContext context) {
                        return const CupertinoPageScaffold(child: Text('Page 2'));
                      },
                    ),
                  );
                },
                child: const Text('Push Page 2'),
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      equals(0.0),
    );
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();

    // Previous page is still visible behind the new sheet.
    expect(find.text('Page 1'), findsOneWidget);
    final Offset pageOneOffset = tester.getTopLeft(
      find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
    );
    expect(pageOneOffset.dy, greaterThan(0.0));
    expect(pageOneOffset.dx, greaterThan(0.0));
    expect(find.text('Page 2'), findsOneWidget);
    final double pageTwoYOffset =
        tester
            .getTopLeft(
              find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
            )
            .dy;
    expect(pageTwoYOffset, greaterThan(pageOneOffset.dy));
  });

  testWidgets('If a sheet covers another sheet, then the previous sheet moves slightly upwards', (
    WidgetTester tester,
  ) async {
    final GlobalKey scaffoldKey = GlobalKey();

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Column(
            children: <Widget>[
              const Text('Page 1'),
              CupertinoButton(
                onPressed: () {
                  Navigator.push<void>(
                    scaffoldKey.currentContext!,
                    CupertinoSheetRoute<void>(
                      builder: (BuildContext context) {
                        return CupertinoPageScaffold(
                          child: Column(
                            children: <Widget>[
                              const Text('Page 2'),
                              CupertinoButton(
                                onPressed: () {
                                  Navigator.push<void>(
                                    scaffoldKey.currentContext!,
                                    CupertinoSheetRoute<void>(
                                      builder: (BuildContext context) {
                                        return const CupertinoPageScaffold(child: Text('Page 3'));
                                      },
                                    ),
                                  );
                                },
                                child: const Text('Push Page 3'),
                              ),
                            ],
                          ),
                        );
                      },
                    ),
                  );
                },
                child: const Text('Push Page 2'),
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      equals(0.0),
    );
    expect(find.text('Page 2'), findsNothing);
    expect(find.text('Page 3'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsOneWidget);
    expect(find.text('Page 3'), findsNothing);
    final double previousPageTwoDY =
        tester
            .getTopLeft(
              find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
            )
            .dy;

    await tester.tap(find.text('Push Page 3'));
    await tester.pumpAndSettle();

    expect(find.text('Page 3'), findsOneWidget);
    expect(previousPageTwoDY, greaterThan(0.0));
    expect(
      previousPageTwoDY,
      greaterThan(
        tester
            .getTopLeft(
              find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
            )
            .dy,
      ),
    );
  });

  testWidgets('by default showCupertinoSheet does not enable nested navigation', (
    WidgetTester tester,
  ) async {
    final GlobalKey scaffoldKey = GlobalKey();

    Widget sheetScaffoldContent(BuildContext context) {
      return Column(
        children: <Widget>[
          const Text('Page 2'),
          CupertinoButton(
            onPressed: () {
              Navigator.push<void>(
                context,
                CupertinoPageRoute<void>(
                  builder: (BuildContext context) {
                    return CupertinoPageScaffold(
                      child: Column(
                        children: <Widget>[
                          const Text('Page 3'),
                          CupertinoButton(onPressed: () {}, child: const Text('Pop Page 3')),
                        ],
                      ),
                    );
                  },
                ),
              );
            },
            child: const Text('Push Page 3'),
          ),
        ],
      );
    }

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    showCupertinoSheet<void>(
                      context: scaffoldKey.currentContext!,
                      pageBuilder: (BuildContext context) {
                        return CupertinoPageScaffold(child: sheetScaffoldContent(context));
                      },
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsOneWidget);
    expect(find.text('Page 3'), findsNothing);

    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      greaterThan(0.0),
    );

    await tester.tap(find.text('Push Page 3'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsNothing);
    expect(find.text('Page 3'), findsOneWidget);
    // New route should be at the top of the screen.
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 3'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      equals(0.0),
    );
  });

  testWidgets('useNestedNavigation set to true enables nested navigation', (
    WidgetTester tester,
  ) async {
    final GlobalKey scaffoldKey = GlobalKey();

    Widget sheetScaffoldContent(BuildContext context) {
      return Column(
        children: <Widget>[
          const Text('Page 2'),
          CupertinoButton(
            onPressed: () {
              Navigator.push<void>(
                context,
                CupertinoPageRoute<void>(
                  builder: (BuildContext context) {
                    return CupertinoPageScaffold(
                      child: Column(
                        children: <Widget>[
                          const Text('Page 3'),
                          CupertinoButton(onPressed: () {}, child: const Text('Pop Page 3')),
                        ],
                      ),
                    );
                  },
                ),
              );
            },
            child: const Text('Push Page 3'),
          ),
        ],
      );
    }

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    showCupertinoSheet<void>(
                      context: scaffoldKey.currentContext!,
                      useNestedNavigation: true,
                      pageBuilder: (BuildContext context) {
                        return CupertinoPageScaffold(child: sheetScaffoldContent(context));
                      },
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsOneWidget);
    expect(find.text('Page 3'), findsNothing);

    final double pageTwoDY =
        tester
            .getTopLeft(
              find.ancestor(of: find.text('Page 2'), matching: find.byType(CupertinoPageScaffold)),
            )
            .dy;
    expect(pageTwoDY, greaterThan(0.0));

    await tester.tap(find.text('Push Page 3'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsNothing);
    expect(find.text('Page 3'), findsOneWidget);

    // New route should be at the same height as the previous route.
    final double pageThreeDY =
        tester
            .getTopLeft(
              find.ancestor(of: find.text('Page 3'), matching: find.byType(CupertinoPageScaffold)),
            )
            .dy;
    expect(pageThreeDY, greaterThan(0.0));
    expect(pageThreeDY, equals(pageTwoDY));
  });

  testWidgets('useNestedNavigation handles programmatic pops', (WidgetTester tester) async {
    final GlobalKey scaffoldKey = GlobalKey();

    Widget sheetScaffoldContent(BuildContext context) {
      return Column(
        children: <Widget>[
          const Text('Page 2'),
          CupertinoButton(
            onPressed: () => Navigator.of(context).maybePop(),
            child: const Text('Go Back'),
          ),
        ],
      );
    }

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    showCupertinoSheet<void>(
                      context: scaffoldKey.currentContext!,
                      useNestedNavigation: true,
                      pageBuilder: (BuildContext context) {
                        return CupertinoPageScaffold(child: sheetScaffoldContent(context));
                      },
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    // The first page is at the top of the screen.
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      equals(0.0),
    );
    expect(find.text('Page 2'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsOneWidget);

    // The first page, which is behind the top sheet but still partially visibile, is moved downwards.
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      greaterThan(0.0),
    );

    await tester.tap(find.text('Go Back'));
    await tester.pumpAndSettle();

    // The first page would correctly transition back and sit at the top of the screen.
    expect(find.text('Page 1'), findsOneWidget);
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      equals(0.0),
    );
    expect(find.text('Page 2'), findsNothing);
  });

  testWidgets('useNestedNavigation handles system pop gestures', (WidgetTester tester) async {
    final GlobalKey scaffoldKey = GlobalKey();

    Widget sheetScaffoldContent(BuildContext context) {
      return Column(
        children: <Widget>[
          const Text('Page 2'),
          CupertinoButton(
            onPressed: () {
              Navigator.of(context).push(
                CupertinoPageRoute<void>(
                  builder: (BuildContext context) {
                    return CupertinoPageScaffold(
                      child: Column(
                        children: <Widget>[
                          const Text('Page 3'),
                          CupertinoButton(
                            onPressed: () {
                              Navigator.of(context).pop();
                            },
                            child: const Text('Go back'),
                          ),
                        ],
                      ),
                    );
                  },
                ),
              );
            },
            child: const Text('Push Page 3'),
          ),
        ],
      );
    }

    await tester.pumpWidget(
      CupertinoApp(
        home: CupertinoPageScaffold(
          key: scaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    showCupertinoSheet<void>(
                      context: scaffoldKey.currentContext!,
                      useNestedNavigation: true,
                      pageBuilder: (BuildContext context) {
                        return CupertinoPageScaffold(child: sheetScaffoldContent(context));
                      },
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    // The first page is at the top of the screen.
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      equals(0.0),
    );
    expect(find.text('Page 2'), findsNothing);
    expect(find.text('Page 3'), findsNothing);

    await tester.tap(find.text('Push Page 2'));
    await tester.pumpAndSettle();

    expect(find.text('Page 2'), findsOneWidget);
    expect(find.text('Page 3'), findsNothing);

    // The first page, which is behind the top sheet but still partially visibile, is moved downwards.
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      greaterThan(0.0),
    );

    await tester.tap(find.text('Push Page 3'));
    await tester.pumpAndSettle();

    expect(find.text('Page 3'), findsOneWidget);

    // Simulate a system back gesture.
    await simulateSystemBack();
    await tester.pumpAndSettle();

    // Go back to the first page within the sheet.
    expect(find.text('Page 2'), findsOneWidget);
    expect(find.text('Page 3'), findsNothing);

    // The first page is still stacked behind the sheet.
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      greaterThan(0.0),
    );

    await simulateSystemBack();
    await tester.pumpAndSettle();

    // The first page would correctly transition back and sit at the top of the screen.
    expect(find.text('Page 1'), findsOneWidget);
    expect(
      tester
          .getTopLeft(
            find.ancestor(of: find.text('Page 1'), matching: find.byType(CupertinoPageScaffold)),
          )
          .dy,
      equals(0.0),
    );
    expect(find.text('Page 2'), findsNothing);
  });

  testWidgets('sheet has route settings', (WidgetTester tester) async {
    await tester.pumpWidget(
      CupertinoApp(
        initialRoute: '/',
        onGenerateRoute: (RouteSettings settings) {
          if (settings.name == '/') {
            return PageRouteBuilder<void>(
              pageBuilder: (
                BuildContext context,
                Animation<double> animation,
                Animation<double> secondaryAnimation,
              ) {
                return CupertinoPageScaffold(
                  navigationBar: const CupertinoNavigationBar(middle: Text('Page 1')),
                  child: Container(),
                );
              },
            );
          }
          return CupertinoSheetRoute<void>(
            builder: (BuildContext context) {
              return CupertinoPageScaffold(
                navigationBar: CupertinoNavigationBar(middle: Text('Page: ${settings.name}')),
                child: Container(),
              );
            },
          );
        },
      ),
    );

    expect(find.text('Page 1'), findsOneWidget);
    expect(find.text('Page 2'), findsNothing);

    tester.state<NavigatorState>(find.byType(Navigator)).pushNamed('/next');
    await tester.pumpAndSettle();

    expect(find.text('Page: /next'), findsOneWidget);
  });

  group('drag dismiss gesture', () {
    Widget dragGestureApp(GlobalKey homeScaffoldKey, GlobalKey sheetScaffoldKey) {
      return CupertinoApp(
        home: CupertinoPageScaffold(
          key: homeScaffoldKey,
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Page 1'),
                CupertinoButton(
                  onPressed: () {
                    showCupertinoSheet<void>(
                      context: homeScaffoldKey.currentContext!,
                      pageBuilder: (BuildContext context) {
                        return CupertinoPageScaffold(
                          key: sheetScaffoldKey,
                          child: const Center(child: Text('Page 2')),
                        );
                      },
                    );
                  },
                  child: const Text('Push Page 2'),
                ),
              ],
            ),
          ),
        ),
      );
    }

    testWidgets('partial drag and drop does not pop the sheet', (WidgetTester tester) async {
      final GlobalKey homeKey = GlobalKey();
      final GlobalKey sheetKey = GlobalKey();

      await tester.pumpWidget(dragGestureApp(homeKey, sheetKey));

      await tester.tap(find.text('Push Page 2'));
      await tester.pumpAndSettle();

      expect(find.text('Page 2'), findsOneWidget);

      RenderBox box = tester.renderObject(find.byKey(sheetKey)) as RenderBox;
      final double initialPosition = box.localToGlobal(Offset.zero).dy;

      final TestGesture gesture = await tester.startGesture(const Offset(100, 200));
      // Partial drag down
      await gesture.moveBy(const Offset(0, 200));
      await tester.pump();

      box = tester.renderObject(find.byKey(sheetKey)) as RenderBox;
      final double middlePosition = box.localToGlobal(Offset.zero).dy;
      expect(middlePosition, greaterThan(initialPosition));

      // Release gesture. Sheet should not pop and slide back up.
      await gesture.up();
      await tester.pumpAndSettle();

      expect(find.text('Page 2'), findsOneWidget);

      box = tester.renderObject(find.byKey(sheetKey)) as RenderBox;
      final double finalPosition = box.localToGlobal(Offset.zero).dy;

      expect(finalPosition, lessThan(middlePosition));
      expect(finalPosition, equals(initialPosition));
    });

    testWidgets('dropping the drag further down the page pops the sheet', (
      WidgetTester tester,
    ) async {
      final GlobalKey homeKey = GlobalKey();
      final GlobalKey sheetKey = GlobalKey();

      await tester.pumpWidget(dragGestureApp(homeKey, sheetKey));

      await tester.tap(find.text('Push Page 2'));
      await tester.pumpAndSettle();

      expect(find.text('Page 2'), findsOneWidget);

      final TestGesture gesture = await tester.startGesture(const Offset(100, 200));
      await gesture.moveBy(const Offset(0, 350));
      await tester.pump();

      await gesture.up();
      await tester.pumpAndSettle();

      expect(find.text('Page 2'), findsNothing);
    });

    testWidgets('dismissing with a drag pops all nested routes', (WidgetTester tester) async {
      final GlobalKey homeKey = GlobalKey();
      final GlobalKey sheetKey = GlobalKey();

      Widget sheetScaffoldContent(BuildContext context) {
        return Column(
          children: <Widget>[
            const Text('Page 2'),
            CupertinoButton(
              onPressed: () {
                Navigator.of(context).push(
                  CupertinoPageRoute<void>(
                    builder: (BuildContext context) {
                      return const CupertinoPageScaffold(child: Center(child: Text('Page 3')));
                    },
                  ),
                );
              },
              child: const Text('Push Page 3'),
            ),
          ],
        );
      }

      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoPageScaffold(
            key: homeKey,
            child: Center(
              child: Column(
                children: <Widget>[
                  const Text('Page 1'),
                  CupertinoButton(
                    onPressed: () {
                      showCupertinoSheet<void>(
                        context: homeKey.currentContext!,
                        useNestedNavigation: true,
                        pageBuilder: (BuildContext context) {
                          return CupertinoPageScaffold(
                            key: sheetKey,
                            child: sheetScaffoldContent(context),
                          );
                        },
                      );
                    },
                    child: const Text('Push Page 2'),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text('Push Page 2'));
      await tester.pumpAndSettle();

      expect(find.text('Page 2'), findsOneWidget);

      await tester.tap(find.text('Push Page 3'));
      await tester.pumpAndSettle();

      expect(find.text('Page 3'), findsOneWidget);

      final TestGesture gesture = await tester.startGesture(const Offset(100, 200));
      await gesture.moveBy(const Offset(0, 350));
      await tester.pump();

      await gesture.up();
      await tester.pumpAndSettle();

      expect(find.text('Page 2'), findsNothing);
      expect(find.text('Page 3'), findsNothing);
    });

    testWidgets('Popping the sheet during drag should not crash', (WidgetTester tester) async {
      final GlobalKey homeKey = GlobalKey();
      final GlobalKey sheetKey = GlobalKey();

      await tester.pumpWidget(dragGestureApp(homeKey, sheetKey));

      await tester.tap(find.text('Push Page 2'));
      await tester.pumpAndSettle();

      final TestGesture gesture = await tester.createGesture();

      await gesture.down(const Offset(100, 200));

      // Need 2 events to form a valid drag
      await tester.pump(const Duration(milliseconds: 100));
      await gesture.moveTo(const Offset(100, 300), timeStamp: const Duration(milliseconds: 100));
      await tester.pump(const Duration(milliseconds: 200));
      await gesture.moveTo(const Offset(100, 500), timeStamp: const Duration(milliseconds: 200));

      Navigator.of(homeKey.currentContext!).pop();

      await tester.pumpAndSettle();

      expect(find.text('Page 1'), findsOneWidget);
      expect(find.text('Page 2'), findsNothing);

      await gesture.up();
      await tester.pumpAndSettle();

      expect(find.text('Page 1'), findsOneWidget);
    });

    testWidgets('Sheet should not block nested scroll', (WidgetTester tester) async {
      final GlobalKey homeKey = GlobalKey();

      Widget sheetScaffoldContent(BuildContext context) {
        return ListView(
          children: const <Widget>[
            Text('Top of Scroll'),
            SizedBox(width: double.infinity, height: 100),
            Text('Middle of Scroll'),
            SizedBox(width: double.infinity, height: 100),
          ],
        );
      }

      await tester.pumpWidget(
        CupertinoApp(
          home: CupertinoPageScaffold(
            key: homeKey,
            child: Center(
              child: Column(
                children: <Widget>[
                  const Text('Page 1'),
                  CupertinoButton(
                    onPressed: () {
                      showCupertinoSheet<void>(
                        context: homeKey.currentContext!,
                        pageBuilder: (BuildContext context) {
                          return CupertinoPageScaffold(child: sheetScaffoldContent(context));
                        },
                      );
                    },
                    child: const Text('Push Page 2'),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text('Push Page 2'));
      await tester.pumpAndSettle();

      expect(find.text('Top of Scroll'), findsOneWidget);
      final double startPosition = tester.getTopLeft(find.text('Middle of Scroll')).dy;

      final TestGesture gesture = await tester.createGesture();

      await gesture.down(const Offset(100, 100));

      // Need 2 events to form a valid drag.
      await tester.pump(const Duration(milliseconds: 100));
      await gesture.moveTo(const Offset(100, 80), timeStamp: const Duration(milliseconds: 100));
      await tester.pump(const Duration(milliseconds: 200));
      await gesture.moveTo(const Offset(100, 50), timeStamp: const Duration(milliseconds: 200));

      await tester.pumpAndSettle();

      final double endPosition = tester.getTopLeft(find.text('Middle of Scroll')).dy;

      // Final position should be higher.
      expect(endPosition, lessThan(startPosition));

      await gesture.up();
      await tester.pumpAndSettle();
    });
  });
}
