// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  // Pumps and ensures that the BottomSheet animates non-linearly.
  Future<void> checkNonLinearAnimation(WidgetTester tester) async {
    final Offset firstPosition = tester.getCenter(find.text('One'));
    await tester.pump(const Duration(milliseconds: 30));
    final Offset secondPosition = tester.getCenter(find.text('One'));
    await tester.pump(const Duration(milliseconds: 30));
    final Offset thirdPosition = tester.getCenter(find.text('One'));

    final double dyDelta1 = secondPosition.dy - firstPosition.dy;
    final double dyDelta2 = thirdPosition.dy - secondPosition.dy;

    // If the animation were linear, these two values would be the same.
    expect(dyDelta1, isNot(moreOrLessEquals(dyDelta2, epsilon: 0.1)));
  }

  testWidgets('Persistent draggableScrollableSheet localHistoryEntries test', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/110123
    Widget buildFrame(Widget? bottomSheet) {
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          body: const Center(child: Text('body')),
          bottomSheet: bottomSheet,
          floatingActionButton: const FloatingActionButton(onPressed: null, child: Text('fab')),
        ),
      );
    }

    final Widget draggableScrollableSheet = DraggableScrollableSheet(
      expand: false,
      snap: true,
      initialChildSize: 0.3,
      minChildSize: 0.3,
      builder: (_, ScrollController controller) {
        return ListView.builder(
          itemExtent: 50.0,
          itemCount: 50,
          itemBuilder: (_, int index) => Text('Item $index'),
          controller: controller,
        );
      },
    );

    await tester.pumpWidget(buildFrame(draggableScrollableSheet));
    await tester.pumpAndSettle();

    expect(find.byType(BackButton).hitTestable(), findsNothing);

    await tester.drag(find.text('Item 2'), const Offset(0, -200.0));
    await tester.pumpAndSettle();
    // We've started to drag up, we should have a back button now for a11y
    expect(find.byType(BackButton).hitTestable(), findsOneWidget);

    await tester.fling(find.text('Item 2'), const Offset(0, 200.0), 2000.0);
    await tester.pumpAndSettle();
    // BackButton should be hidden
    expect(find.byType(BackButton).hitTestable(), findsNothing);

    // Show the back button again
    await tester.drag(find.text('Item 2'), const Offset(0, -200.0));
    await tester.pumpAndSettle();
    expect(find.byType(BackButton).hitTestable(), findsOneWidget);

    // Remove the draggableScrollableSheet should hide the back button
    await tester.pumpWidget(buildFrame(null));
    expect(find.byType(BackButton).hitTestable(), findsNothing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/83668
  testWidgets('Scaffold.bottomSheet update test', (WidgetTester tester) async {
    Widget buildFrame(Widget? bottomSheet) {
      return MaterialApp(home: Scaffold(body: const Placeholder(), bottomSheet: bottomSheet));
    }

    await tester.pumpWidget(buildFrame(const Text('I love Flutter!')));
    await tester.pumpWidget(buildFrame(null));

    // The disappearing animation has not yet been completed.
    await tester.pumpWidget(buildFrame(const Text('I love Flutter!')));
  });

  testWidgets(
    'Verify that a BottomSheet can be rebuilt with ScaffoldFeatureController.setState()',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
      int buildCount = 0;

      await tester.pumpWidget(
        MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
      );

      final PersistentBottomSheetController bottomSheet = scaffoldKey.currentState!.showBottomSheet(
        (_) {
          return Builder(
            builder: (BuildContext context) {
              buildCount += 1;
              return Container(height: 200.0);
            },
          );
        },
      );

      await tester.pump();
      expect(buildCount, equals(1));
      bottomSheet.setState!(() {});
      await tester.pump();
      expect(buildCount, equals(2));
    },
  );

  testWidgets('Verify that a persistent BottomSheet cannot be dismissed', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: const Center(child: Text('body')),
          bottomSheet: DraggableScrollableSheet(
            expand: false,
            builder: (_, ScrollController controller) {
              return ListView(
                controller: controller,
                shrinkWrap: true,
                children: const <Widget>[
                  SizedBox(height: 100.0, child: Text('One')),
                  SizedBox(height: 100.0, child: Text('Two')),
                  SizedBox(height: 100.0, child: Text('Three')),
                ],
              );
            },
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(find.text('Two'), findsOneWidget);

    await tester.drag(find.text('Two'), const Offset(0.0, 400.0));
    await tester.pumpAndSettle();

    expect(find.text('Two'), findsOneWidget);
  });

  testWidgets('Verify that a scrollable BottomSheet can be dismissed', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
      return ListView(
        shrinkWrap: true,
        primary: false,
        children: const <Widget>[
          SizedBox(height: 100.0, child: Text('One')),
          SizedBox(height: 100.0, child: Text('Two')),
          SizedBox(height: 100.0, child: Text('Three')),
        ],
      );
    });

    await tester.pumpAndSettle();

    expect(find.text('Two'), findsOneWidget);

    await tester.drag(find.text('Two'), const Offset(0.0, 400.0));
    await tester.pumpAndSettle();

    expect(find.text('Two'), findsNothing);
  });

  testWidgets(
    'Verify DraggableScrollableSheet.shouldCloseOnMinExtent == false prevents dismissal',
    (WidgetTester tester) async {
      final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

      await tester.pumpWidget(
        MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
      );

      scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
        return DraggableScrollableSheet(
          expand: false,
          shouldCloseOnMinExtent: false,
          builder: (_, ScrollController controller) {
            return ListView(
              controller: controller,
              shrinkWrap: true,
              children: const <Widget>[
                SizedBox(height: 100.0, child: Text('One')),
                SizedBox(height: 100.0, child: Text('Two')),
                SizedBox(height: 100.0, child: Text('Three')),
              ],
            );
          },
        );
      });

      await tester.pumpAndSettle();

      expect(find.text('Two'), findsOneWidget);

      await tester.drag(find.text('Two'), const Offset(0.0, 400.0));
      await tester.pumpAndSettle();

      expect(find.text('Two'), findsOneWidget);
    },
  );

  testWidgets('Verify that a BottomSheet animates non-linearly', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
      return ListView(
        shrinkWrap: true,
        primary: false,
        children: const <Widget>[
          SizedBox(height: 100.0, child: Text('One')),
          SizedBox(height: 100.0, child: Text('Two')),
          SizedBox(height: 100.0, child: Text('Three')),
        ],
      );
    });
    await tester.pump();
    await checkNonLinearAnimation(tester);

    await tester.pumpAndSettle();

    expect(find.text('Two'), findsOneWidget);

    await tester.drag(find.text('Two'), const Offset(0.0, 200.0));
    await checkNonLinearAnimation(tester);
    await tester.pumpAndSettle();

    expect(find.text('Two'), findsNothing);
  });

  testWidgets('Verify that a scrollControlled BottomSheet can be dismissed', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
      return DraggableScrollableSheet(
        expand: false,
        builder: (_, ScrollController controller) {
          return ListView(
            shrinkWrap: true,
            controller: controller,
            children: const <Widget>[
              SizedBox(height: 100.0, child: Text('One')),
              SizedBox(height: 100.0, child: Text('Two')),
              SizedBox(height: 100.0, child: Text('Three')),
            ],
          );
        },
      );
    });

    await tester.pumpAndSettle();

    expect(find.text('Two'), findsOneWidget);

    await tester.drag(find.text('Two'), const Offset(0.0, 400.0));
    await tester.pumpAndSettle();

    expect(find.text('Two'), findsNothing);
  });

  testWidgets('Verify that a persistent BottomSheet can fling up and hide the fab', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          body: const Center(child: Text('body')),
          bottomSheet: DraggableScrollableSheet(
            expand: false,
            builder: (_, ScrollController controller) {
              return ListView.builder(
                itemExtent: 50.0,
                itemCount: 50,
                itemBuilder: (_, int index) => Text('Item $index'),
                controller: controller,
              );
            },
          ),
          floatingActionButton: const FloatingActionButton(onPressed: null, child: Text('fab')),
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 22'), findsNothing);
    expect(find.byType(FloatingActionButton), findsOneWidget);
    expect(find.byType(FloatingActionButton).hitTestable(), findsOneWidget);
    expect(find.byType(BackButton).hitTestable(), findsNothing);

    await tester.drag(find.text('Item 2'), const Offset(0, -20.0));
    await tester.pumpAndSettle();

    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 22'), findsNothing);
    expect(find.byType(FloatingActionButton), findsOneWidget);
    expect(find.byType(FloatingActionButton).hitTestable(), findsOneWidget);

    await tester.fling(find.text('Item 2'), const Offset(0.0, -600.0), 2000.0);
    await tester.pumpAndSettle();

    expect(find.text('Item 2'), findsNothing);
    expect(find.text('Item 22'), findsOneWidget);
    expect(find.byType(FloatingActionButton), findsOneWidget);
    expect(find.byType(FloatingActionButton).hitTestable(), findsNothing);
  });

  testWidgets('Verify that a back button resets a persistent BottomSheet', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(),
          body: const Center(child: Text('body')),
          bottomSheet: DraggableScrollableSheet(
            expand: false,
            builder: (_, ScrollController controller) {
              return ListView.builder(
                itemExtent: 50.0,
                itemCount: 50,
                itemBuilder: (_, int index) => Text('Item $index'),
                controller: controller,
              );
            },
          ),
          floatingActionButton: const FloatingActionButton(onPressed: null, child: Text('fab')),
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 22'), findsNothing);
    expect(find.byType(BackButton).hitTestable(), findsNothing);

    await tester.drag(find.text('Item 2'), const Offset(0, -20.0));
    await tester.pumpAndSettle();

    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 22'), findsNothing);
    // We've started to drag up, we should have a back button now for a11y
    expect(find.byType(BackButton).hitTestable(), findsOneWidget);

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();

    expect(find.byType(BackButton).hitTestable(), findsNothing);
    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 22'), findsNothing);

    await tester.fling(find.text('Item 2'), const Offset(0.0, -600.0), 2000.0);
    await tester.pumpAndSettle();

    expect(find.text('Item 2'), findsNothing);
    expect(find.text('Item 22'), findsOneWidget);
    expect(find.byType(BackButton).hitTestable(), findsOneWidget);

    await tester.tap(find.byType(BackButton));
    await tester.pumpAndSettle();

    expect(find.byType(BackButton).hitTestable(), findsNothing);
    expect(find.text('Item 2'), findsOneWidget);
    expect(find.text('Item 22'), findsNothing);
  });

  testWidgets('Verify that a scrollable BottomSheet hides the fab when scrolled up', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          key: scaffoldKey,
          body: const Center(child: Text('body')),
          floatingActionButton: const FloatingActionButton(onPressed: null, child: Text('fab')),
        ),
      ),
    );

    scaffoldKey.currentState!.showBottomSheet((BuildContext context) {
      return DraggableScrollableSheet(
        expand: false,
        builder: (_, ScrollController controller) {
          return ListView(
            controller: controller,
            shrinkWrap: true,
            children: const <Widget>[
              SizedBox(height: 100.0, child: Text('One')),
              SizedBox(height: 100.0, child: Text('Two')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
              SizedBox(height: 100.0, child: Text('Three')),
            ],
          );
        },
      );
    });

    await tester.pumpAndSettle();

    expect(find.text('Two'), findsOneWidget);
    expect(find.byType(FloatingActionButton).hitTestable(), findsOneWidget);

    await tester.drag(find.text('Two'), const Offset(0.0, -600.0));
    await tester.pumpAndSettle();

    expect(find.text('Two'), findsOneWidget);
    expect(find.byType(FloatingActionButton), findsOneWidget);
    expect(find.byType(FloatingActionButton).hitTestable(), findsNothing);
  });

  testWidgets('showBottomSheet()', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(MaterialApp(home: Scaffold(body: Placeholder(key: key))));

    int buildCount = 0;
    showBottomSheet(
      context: key.currentContext!,
      builder: (BuildContext context) {
        return Builder(
          builder: (BuildContext context) {
            buildCount += 1;
            return Container(height: 200.0);
          },
        );
      },
    );
    await tester.pump();
    expect(buildCount, equals(1));
  });

  testWidgets('Scaffold removes top MediaQuery padding', (WidgetTester tester) async {
    late BuildContext scaffoldContext;
    late BuildContext bottomSheetContext;

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.all(50.0)),
          child: Scaffold(
            resizeToAvoidBottomInset: false,
            body: Builder(
              builder: (BuildContext context) {
                scaffoldContext = context;
                return Container();
              },
            ),
          ),
        ),
      ),
    );

    await tester.pump();

    showBottomSheet(
      context: scaffoldContext,
      builder: (BuildContext context) {
        bottomSheetContext = context;
        return Container();
      },
    );

    await tester.pump();

    expect(
      MediaQuery.of(bottomSheetContext).padding,
      const EdgeInsets.only(bottom: 50.0, left: 50.0, right: 50.0),
    );
  });

  testWidgets('Scaffold.bottomSheet', (WidgetTester tester) async {
    final Key bottomSheetKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: const Placeholder(),
          bottomSheet: Container(
            key: bottomSheetKey,
            alignment: Alignment.center,
            height: 200.0,
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('showModalBottomSheet'),
                  onPressed: () {
                    showModalBottomSheet<void>(
                      context: context,
                      builder: (BuildContext context) => const Text('modal bottom sheet'),
                    );
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(find.text('showModalBottomSheet'), findsOneWidget);
    expect(tester.getSize(find.byKey(bottomSheetKey)), const Size(800.0, 200.0));
    expect(tester.getTopLeft(find.byKey(bottomSheetKey)), const Offset(0.0, 400.0));

    // Show the modal bottomSheet
    await tester.tap(find.text('showModalBottomSheet'));
    await tester.pumpAndSettle();
    expect(find.text('modal bottom sheet'), findsOneWidget);

    // Dismiss the modal bottomSheet by tapping above the sheet
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle();
    expect(find.text('modal bottom sheet'), findsNothing);
    expect(find.text('showModalBottomSheet'), findsOneWidget);

    // Remove the persistent bottomSheet
    await tester.pumpWidget(const MaterialApp(home: Scaffold(body: Placeholder())));
    await tester.pumpAndSettle();
    expect(find.text('showModalBottomSheet'), findsNothing);
    expect(find.byKey(bottomSheetKey), findsNothing);
  });

  // Regression test for https://github.com/flutter/flutter/issues/71435
  testWidgets('Scaffold.bottomSheet should be updated without creating a new RO'
      ' when the new widget has the same key and type.', (WidgetTester tester) async {
    Widget buildFrame(String text) {
      return MaterialApp(home: Scaffold(body: const Placeholder(), bottomSheet: Text(text)));
    }

    await tester.pumpWidget(buildFrame('I love Flutter!'));
    final RenderParagraph renderBeforeUpdate = tester.renderObject(find.text('I love Flutter!'));

    await tester.pumpWidget(buildFrame('Flutter is the best!'));
    await tester.pumpAndSettle();
    final RenderParagraph renderAfterUpdate = tester.renderObject(
      find.text('Flutter is the best!'),
    );

    expect(renderBeforeUpdate, renderAfterUpdate);
  });

  testWidgets('Verify that visual properties are passed through', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    const Color color = Colors.pink;
    const double elevation = 9.0;
    const ShapeBorder shape = BeveledRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(12)),
    );
    const Clip clipBehavior = Clip.antiAlias;

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    scaffoldKey.currentState!.showBottomSheet(
      (BuildContext context) {
        return ListView(
          shrinkWrap: true,
          primary: false,
          children: const <Widget>[
            SizedBox(height: 100.0, child: Text('One')),
            SizedBox(height: 100.0, child: Text('Two')),
            SizedBox(height: 100.0, child: Text('Three')),
          ],
        );
      },
      backgroundColor: color,
      elevation: elevation,
      shape: shape,
      clipBehavior: clipBehavior,
    );

    await tester.pumpAndSettle();

    final BottomSheet bottomSheet = tester.widget(find.byType(BottomSheet));
    expect(bottomSheet.backgroundColor, color);
    expect(bottomSheet.elevation, elevation);
    expect(bottomSheet.shape, shape);
    expect(bottomSheet.clipBehavior, clipBehavior);
  });

  testWidgets('PersistentBottomSheetController.close dismisses the bottom sheet', (
    WidgetTester tester,
  ) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(home: Scaffold(key: scaffoldKey, body: const Center(child: Text('body')))),
    );

    final PersistentBottomSheetController bottomSheet = scaffoldKey.currentState!.showBottomSheet((
      _,
    ) {
      return Builder(
        builder: (BuildContext context) {
          return Container(height: 200.0);
        },
      );
    });

    await tester.pump();
    expect(find.byType(BottomSheet), findsOneWidget);

    bottomSheet.close();
    await tester.pump();
    expect(find.byType(BottomSheet), findsNothing);
  });
}
