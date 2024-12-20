// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SharedAppData basics', (WidgetTester tester) async {
    int columnBuildCount = 0;
    int child1BuildCount = 0;
    int child2BuildCount = 0;
    late void Function(BuildContext context) setSharedAppDataValue;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SharedAppData(
          child: Builder(
            builder: (BuildContext context) {
              columnBuildCount += 1;
              return GestureDetector(
                behavior: HitTestBehavior.opaque,
                onTap: () {
                  setSharedAppDataValue.call(context);
                },
                child: Column(
                  children: <Widget>[
                    Builder(
                      builder: (BuildContext context) {
                        child1BuildCount += 1;
                        return Text(
                          SharedAppData.getValue<String, String>(
                            context,
                            'child1Text',
                            () => 'null',
                          ),
                        );
                      },
                    ),
                    Builder(
                      builder: (BuildContext context) {
                        child2BuildCount += 1;
                        return Text(
                          SharedAppData.getValue<String, String>(
                            context,
                            'child2Text',
                            () => 'null',
                          ),
                        );
                      },
                    ),
                  ],
                ),
              );
            },
          ),
        ),
      ),
    );

    expect(columnBuildCount, 1);
    expect(child1BuildCount, 1);
    expect(child2BuildCount, 1);
    expect(find.text('null').evaluate().length, 2);

    // SharedAppData.setValue<String, String>(context, 'child1Text', 'child1')
    // causes the first Text widget to be rebuilt with its text to be
    // set to 'child1'. Nothing else is rebuilt.
    setSharedAppDataValue = (BuildContext context) {
      SharedAppData.setValue<String, String>(context, 'child1Text', 'child1');
    };
    await tester.tap(find.byType(GestureDetector));
    await tester.pump();
    expect(columnBuildCount, 1);
    expect(child1BuildCount, 2);
    expect(child2BuildCount, 1);
    expect(find.text('child1'), findsOneWidget);
    expect(find.text('null'), findsOneWidget);

    // SharedAppData.setValue<String, String>(context, 'child2Text', 'child1')
    // causes the second Text widget to be rebuilt with its text to be
    // set to 'child2'. Nothing else is rebuilt.
    setSharedAppDataValue = (BuildContext context) {
      SharedAppData.setValue<String, String>(context, 'child2Text', 'child2');
    };
    await tester.tap(find.byType(GestureDetector));
    await tester.pump();
    expect(columnBuildCount, 1);
    expect(child1BuildCount, 2);
    expect(child2BuildCount, 2);
    expect(find.text('child1'), findsOneWidget);
    expect(find.text('child2'), findsOneWidget);

    // Resetting a key's value to the same value does not
    // cause any widgets to be rebuilt.
    setSharedAppDataValue = (BuildContext context) {
      SharedAppData.setValue<String, String>(context, 'child1Text', 'child1');
      SharedAppData.setValue<String, String>(context, 'child2Text', 'child2');
    };
    await tester.tap(find.byType(GestureDetector));
    await tester.pump();
    expect(columnBuildCount, 1);
    expect(child1BuildCount, 2);
    expect(child2BuildCount, 2);

    // More of the same, resetting the values to null..

    setSharedAppDataValue = (BuildContext context) {
      SharedAppData.setValue<String, String>(context, 'child1Text', 'null');
    };
    await tester.tap(find.byType(GestureDetector));
    await tester.pump();
    expect(columnBuildCount, 1);
    expect(child1BuildCount, 3);
    expect(child2BuildCount, 2);
    expect(find.text('null'), findsOneWidget);
    expect(find.text('child2'), findsOneWidget);

    setSharedAppDataValue = (BuildContext context) {
      SharedAppData.setValue<String, String>(context, 'child2Text', 'null');
    };
    await tester.tap(find.byType(GestureDetector));
    await tester.pump();
    expect(columnBuildCount, 1);
    expect(child1BuildCount, 3);
    expect(child2BuildCount, 3);
    expect(find.text('null').evaluate().length, 2);
  });

  testWidgets('WidgetsApp SharedAppData ', (WidgetTester tester) async {
    int parentBuildCount = 0;
    int childBuildCount = 0;

    await tester.pumpWidget(
      WidgetsApp(
        color: const Color(0xff00ff00),
        builder: (BuildContext context, Widget? child) {
          parentBuildCount += 1;
          return GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTap: () {
              SharedAppData.setValue<String, String>(context, 'childText', 'child');
            },
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  childBuildCount += 1;
                  return Text(
                    SharedAppData.getValue<String, String>(context, 'childText', () => 'null'),
                  );
                },
              ),
            ),
          );
        },
      ),
    );

    expect(find.text('null'), findsOneWidget);
    expect(parentBuildCount, 1);
    expect(childBuildCount, 1);

    await tester.tap(find.byType(GestureDetector));
    await tester.pump();
    expect(parentBuildCount, 1);
    expect(childBuildCount, 2);
    expect(find.text('child'), findsOneWidget);
  });

  testWidgets('WidgetsApp SharedAppData Shadowing', (WidgetTester tester) async {
    int innerTapCount = 0;
    int outerTapCount = 0;

    await tester.pumpWidget(
      WidgetsApp(
        color: const Color(0xff00ff00),
        builder: (BuildContext context, Widget? child) {
          return GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTap: () {
              outerTapCount += 1;
              SharedAppData.setValue<String, String>(context, 'childText', 'child');
            },
            child: Center(
              child: SharedAppData(
                child: Builder(
                  builder: (BuildContext context) {
                    return GestureDetector(
                      onTap: () {
                        innerTapCount += 1;
                        SharedAppData.setValue<String, String>(context, 'childText', 'child');
                      },
                      child: Text(
                        SharedAppData.getValue<String, String>(context, 'childText', () => 'null'),
                      ),
                    );
                  },
                ),
              ),
            ),
          );
        },
      ),
    );

    expect(find.text('null'), findsOneWidget);

    await tester.tapAt(const Offset(10, 10));
    await tester.pump();
    expect(outerTapCount, 1);
    expect(innerTapCount, 0);
    expect(find.text('null'), findsOneWidget);

    await tester.tap(find.text('null'));
    await tester.pump();
    expect(outerTapCount, 1);
    expect(innerTapCount, 1);
    expect(find.text('child'), findsOneWidget);
  });
}
