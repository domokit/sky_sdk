// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:ui' as ui;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

RenderBox getRenderSegmentedControl(WidgetTester tester) {
  return tester.allRenderObjects.firstWhere((RenderObject currentObject) {
        return currentObject.toStringShort().contains('_RenderSegmentedControl');
      })
      as RenderBox;
}

StatefulBuilder setupSimpleSegmentedControl() {
  final Map<int, Widget> children = <int, Widget>{};
  children[0] = const Text('Child 1');
  children[1] = const Text('Child 2');
  int sharedValue = 0;

  return StatefulBuilder(
    builder: (BuildContext context, StateSetter setState) {
      return boilerplate(
        child: CupertinoSegmentedControl<int>(
          children: children,
          onValueChanged: (int newValue) {
            setState(() {
              sharedValue = newValue;
            });
          },
          groupValue: sharedValue,
        ),
      );
    },
  );
}

Widget boilerplate({required Widget child}) {
  return Directionality(textDirection: TextDirection.ltr, child: Center(child: child));
}

int getChildCount(WidgetTester tester) {
  return (getRenderSegmentedControl(tester)
          as RenderBoxContainerDefaultsMixin<RenderBox, ContainerBoxParentData<RenderBox>>)
      .getChildrenAsList()
      .length;
}

ui.RRect getSurroundingRect(WidgetTester tester, {int child = 0}) {
  return ((getRenderSegmentedControl(tester)
                      as RenderBoxContainerDefaultsMixin<
                        RenderBox,
                        ContainerBoxParentData<RenderBox>
                      >)
                  .getChildrenAsList()[child]
                  .parentData!
              as dynamic)
          .surroundingRect
      as ui.RRect;
}

Size getChildSize(WidgetTester tester, {int child = 0}) {
  return (getRenderSegmentedControl(tester)
          as RenderBoxContainerDefaultsMixin<RenderBox, ContainerBoxParentData<RenderBox>>)
      .getChildrenAsList()[child]
      .size;
}

Color getBorderColor(WidgetTester tester) {
  return (getRenderSegmentedControl(tester) as dynamic).borderColor as Color;
}

int? getSelectedIndex(WidgetTester tester) {
  return (getRenderSegmentedControl(tester) as dynamic).selectedIndex as int?;
}

Color getBackgroundColor(WidgetTester tester, int childIndex) {
  // Using dynamic so the test can access a private class.
  // ignore: avoid_dynamic_calls
  return (getRenderSegmentedControl(tester) as dynamic).backgroundColors[childIndex] as Color;
}

void main() {
  testWidgets('Tap changes toggle state', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');
    children[2] = const Text('Child 3');

    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(sharedValue, 0);

    await tester.tap(find.byKey(const ValueKey<String>('Segmented Control')));

    expect(sharedValue, 1);
  });

  testWidgets('Need at least 2 children', (WidgetTester tester) async {
    await expectLater(
      () => tester.pumpWidget(
        boilerplate(
          child: CupertinoSegmentedControl<int>(
            children: const <int, Widget>{},
            onValueChanged: (int newValue) {},
          ),
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains('children.length'),
        ),
      ),
    );

    await expectLater(
      () => tester.pumpWidget(
        boilerplate(
          child: CupertinoSegmentedControl<int>(
            children: const <int, Widget>{0: Text('Child 1')},
            onValueChanged: (int newValue) {},
          ),
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains('children.length'),
        ),
      ),
    );
  });

  testWidgets('Padding works', (WidgetTester tester) async {
    const Key key = Key('Container');

    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const SizedBox(height: double.infinity, child: Text('Child 1'));
    children[1] = const SizedBox(height: double.infinity, child: Text('Child 2'));

    Future<void> verifyPadding({EdgeInsets? padding}) async {
      final EdgeInsets effectivePadding = padding ?? const EdgeInsets.symmetric(horizontal: 16);
      final Rect segmentedControlRect = tester.getRect(find.byKey(key));
      expect(
        tester.getTopLeft(find.byWidget(children[0]!)),
        segmentedControlRect.topLeft.translate(
          effectivePadding.topLeft.dx,
          effectivePadding.topLeft.dy,
        ),
      );
      expect(
        tester.getBottomLeft(find.byWidget(children[0]!)),
        segmentedControlRect.bottomLeft.translate(
          effectivePadding.bottomLeft.dx,
          effectivePadding.bottomLeft.dy,
        ),
      );

      expect(
        tester.getTopRight(find.byWidget(children[1]!)),
        segmentedControlRect.topRight.translate(
          effectivePadding.topRight.dx,
          effectivePadding.topRight.dy,
        ),
      );
      expect(
        tester.getBottomRight(find.byWidget(children[1]!)),
        segmentedControlRect.bottomRight.translate(
          effectivePadding.bottomRight.dx,
          effectivePadding.bottomRight.dy,
        ),
      );
    }

    await tester.pumpWidget(
      boilerplate(
        child: CupertinoSegmentedControl<int>(
          key: key,
          children: children,
          onValueChanged: (int newValue) {},
        ),
      ),
    );

    // Default padding works.
    await verifyPadding();

    // Switch to Child 2 padding should remain the same.
    await tester.tap(find.text('Child 2'));
    await tester.pumpAndSettle();

    await verifyPadding();

    await tester.pumpWidget(
      boilerplate(
        child: CupertinoSegmentedControl<int>(
          key: key,
          padding: const EdgeInsets.fromLTRB(1, 3, 5, 7),
          children: children,
          onValueChanged: (int newValue) {},
        ),
      ),
    );

    // Custom padding works.
    await verifyPadding(padding: const EdgeInsets.fromLTRB(1, 3, 5, 7));

    // Switch back to Child 1 padding should remain the same.
    await tester.tap(find.text('Child 1'));
    await tester.pumpAndSettle();

    await verifyPadding(padding: const EdgeInsets.fromLTRB(1, 3, 5, 7));
  });

  testWidgets('Value attribute must be the key of one of the children widgets', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    await expectLater(
      () => tester.pumpWidget(
        boilerplate(
          child: CupertinoSegmentedControl<int>(
            children: children,
            onValueChanged: (int newValue) {},
            groupValue: 2,
          ),
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains('children'),
        ),
      ),
    );
  });

  testWidgets('Widgets have correct default text/icon styles, change correctly on selection', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Icon(IconData(1));

    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    await tester.pumpAndSettle();

    DefaultTextStyle textStyle = tester.widget(find.widgetWithText(DefaultTextStyle, 'Child 1'));
    IconTheme iconTheme = tester.widget(find.widgetWithIcon(IconTheme, const IconData(1)));

    expect(textStyle.style.color, isSameColorAs(CupertinoColors.white));
    expect(iconTheme.data.color, CupertinoColors.activeBlue);

    await tester.tap(find.widgetWithIcon(IconTheme, const IconData(1)));
    await tester.pumpAndSettle();

    textStyle = tester.widget(find.widgetWithText(DefaultTextStyle, 'Child 1'));
    iconTheme = tester.widget(find.widgetWithIcon(IconTheme, const IconData(1)));

    expect(textStyle.style.color, CupertinoColors.activeBlue);
    expect(iconTheme.data.color, isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Segmented controls respects themes', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Icon(IconData(1));

    int sharedValue = 0;

    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            );
          },
        ),
      ),
    );

    DefaultTextStyle textStyle = tester.widget(
      find.widgetWithText(DefaultTextStyle, 'Child 1').first,
    );
    IconThemeData iconTheme = IconTheme.of(tester.element(find.byIcon(const IconData(1))));

    expect(textStyle.style.color, isSameColorAs(CupertinoColors.white));
    expect(iconTheme.color, isSameColorAs(CupertinoColors.systemBlue.darkColor));

    await tester.tap(find.byIcon(const IconData(1)));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 500));

    textStyle = tester.widget(find.widgetWithText(DefaultTextStyle, 'Child 1').first);
    iconTheme = IconTheme.of(tester.element(find.byIcon(const IconData(1))));

    expect(textStyle.style.color, isSameColorAs(CupertinoColors.systemBlue.darkColor));
    expect(iconTheme.color, isSameColorAs(CupertinoColors.white));
  });

  testWidgets('SegmentedControl is correct when user provides custom colors', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Icon(IconData(1));

    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
              unselectedColor: CupertinoColors.lightBackgroundGray,
              selectedColor: CupertinoColors.activeGreen.color,
              borderColor: CupertinoColors.black,
              pressedColor: const Color(0x638CFC7B),
            ),
          );
        },
      ),
    );

    DefaultTextStyle textStyle = tester.widget(find.widgetWithText(DefaultTextStyle, 'Child 1'));
    IconTheme iconTheme = tester.widget(find.widgetWithIcon(IconTheme, const IconData(1)));

    expect(getBorderColor(tester), CupertinoColors.black);
    expect(textStyle.style.color, CupertinoColors.lightBackgroundGray);
    expect(iconTheme.data.color, CupertinoColors.activeGreen.color);
    expect(getBackgroundColor(tester, 0), CupertinoColors.activeGreen.color);
    expect(getBackgroundColor(tester, 1), CupertinoColors.lightBackgroundGray);

    await tester.tap(find.widgetWithIcon(IconTheme, const IconData(1)));
    await tester.pumpAndSettle();

    textStyle = tester.widget(find.widgetWithText(DefaultTextStyle, 'Child 1'));
    iconTheme = tester.widget(find.widgetWithIcon(IconTheme, const IconData(1)));

    expect(textStyle.style.color, CupertinoColors.activeGreen.color);
    expect(iconTheme.data.color, CupertinoColors.lightBackgroundGray);
    expect(getBackgroundColor(tester, 0), CupertinoColors.lightBackgroundGray);
    expect(getBackgroundColor(tester, 1), CupertinoColors.activeGreen.color);

    final Offset center = tester.getCenter(find.text('Child 1'));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    expect(getBackgroundColor(tester, 0), const Color(0x638CFC7B));
    expect(getBackgroundColor(tester, 1), CupertinoColors.activeGreen.color);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Widgets are centered within segments', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 200.0,
            height: 200.0,
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {},
            ),
          ),
        ),
      ),
    );

    // Widgets are centered taking into account 16px of horizontal padding
    expect(tester.getCenter(find.text('Child 1')), const Offset(58.0, 100.0));
    expect(tester.getCenter(find.text('Child 2')), const Offset(142.0, 100.0));
  });

  testWidgets('Tap calls onValueChanged', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    bool value = false;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                value = true;
              },
            ),
          );
        },
      ),
    );

    expect(value, isFalse);

    await tester.tap(find.text('Child 2'));

    expect(value, isTrue);
  });

  testWidgets('State does not change if onValueChanged does not call setState()', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    const int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {},
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    await tester.tap(find.text('Child 2'));
    await tester.pump();

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Background color of child should change on selection, '
      'and should not change when tapped again', (WidgetTester tester) async {
    await tester.pumpWidget(setupSimpleSegmentedControl());

    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    await tester.tap(find.text('Child 2'));
    await tester.pumpAndSettle(const Duration(milliseconds: 200));

    expect(getBackgroundColor(tester, 1), CupertinoColors.activeBlue);

    await tester.tap(find.text('Child 2'));
    await tester.pump();

    expect(getBackgroundColor(tester, 1), CupertinoColors.activeBlue);
  });

  testWidgets('Children can be non-Text or Icon widgets (in this case, '
      'a Container or Placeholder widget)', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = Container(constraints: const BoxConstraints.tightFor(width: 50.0, height: 50.0));
    children[2] = const Placeholder();

    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );
  });

  testWidgets('Passed in value is child initially selected', (WidgetTester tester) async {
    await tester.pumpWidget(setupSimpleSegmentedControl());

    expect(getSelectedIndex(tester), 0);

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Null input for value results in no child initially selected', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    int? sharedValue;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(getSelectedIndex(tester), null);

    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Long press changes background color of not-selected child', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(setupSimpleSegmentedControl());

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    final Offset center = tester.getCenter(find.text('Child 2'));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), const Color(0x33007aff));

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Long press does not change background color of currently-selected child', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(setupSimpleSegmentedControl());

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    final Offset center = tester.getCenter(find.text('Child 1'));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Height of segmented control is determined by tallest widget', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = Container(constraints: const BoxConstraints.tightFor(height: 100.0));
    children[1] = Container(constraints: const BoxConstraints.tightFor(height: 400.0));
    children[2] = Container(constraints: const BoxConstraints.tightFor(height: 200.0));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {},
            ),
          );
        },
      ),
    );

    final RenderBox buttonBox = tester.renderObject(
      find.byKey(const ValueKey<String>('Segmented Control')),
    );

    expect(buttonBox.size.height, 400.0);
  });

  testWidgets('Width of each segmented control segment is determined by widest widget', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = Container(constraints: const BoxConstraints.tightFor(width: 50.0));
    children[1] = Container(constraints: const BoxConstraints.tightFor(width: 100.0));
    children[2] = Container(constraints: const BoxConstraints.tightFor(width: 200.0));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {},
            ),
          );
        },
      ),
    );

    final RenderBox segmentedControl = tester.renderObject(
      find.byKey(const ValueKey<String>('Segmented Control')),
    );

    // Subtract the 16.0px from each side. Remaining width should be allocated
    // to each child equally.
    final double childWidth = (segmentedControl.size.width - 32.0) / 3;

    expect(childWidth, 200.0);

    expect(childWidth, getSurroundingRect(tester).width);
    expect(childWidth, getSurroundingRect(tester, child: 1).width);
    expect(childWidth, getSurroundingRect(tester, child: 2).width);
  });

  testWidgets('Width is finite in unbounded space', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: Row(
              children: <Widget>[
                CupertinoSegmentedControl<int>(
                  key: const ValueKey<String>('Segmented Control'),
                  children: children,
                  onValueChanged: (int newValue) {},
                ),
              ],
            ),
          );
        },
      ),
    );

    final RenderBox segmentedControl = tester.renderObject(
      find.byKey(const ValueKey<String>('Segmented Control')),
    );

    expect(segmentedControl.size.width.isFinite, isTrue);
  });

  testWidgets('Directionality test - RTL should reverse order of widgets', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.rtl,
        child: Center(
          child: CupertinoSegmentedControl<int>(
            children: children,
            onValueChanged: (int newValue) {},
          ),
        ),
      ),
    );

    expect(
      tester.getTopRight(find.text('Child 1')).dx > tester.getTopRight(find.text('Child 2')).dx,
      isTrue,
    );
  });

  testWidgets('Correct initial selection and toggling behavior - RTL', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return Directionality(
            textDirection: TextDirection.rtl,
            child: Center(
              child: CupertinoSegmentedControl<int>(
                children: children,
                onValueChanged: (int newValue) {
                  setState(() {
                    sharedValue = newValue;
                  });
                },
                groupValue: sharedValue,
              ),
            ),
          );
        },
      ),
    );

    expect(getBackgroundColor(tester, 0), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    await tester.tap(find.text('Child 2'));
    await tester.pumpAndSettle();

    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), CupertinoColors.activeBlue);

    await tester.tap(find.text('Child 2'));
    await tester.pump();

    expect(getBackgroundColor(tester, 1), CupertinoColors.activeBlue);
  });

  testWidgets('Segmented control semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: CupertinoSegmentedControl<int>(
                children: children,
                onValueChanged: (int newValue) {
                  setState(() {
                    sharedValue = newValue;
                  });
                },
                groupValue: sharedValue,
              ),
            ),
          );
        },
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              label: 'Child 1',
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.hasSelectedState,
                SemanticsFlag.isSelected,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap],
            ),
            TestSemantics.rootChild(
              label: 'Child 2',
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                // Declares that it is selectable, but not currently selected.
                SemanticsFlag.hasSelectedState,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.tap(find.text('Child 2'));
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              label: 'Child 1',
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                // Declares that it is selectable, but not currently selected.
                SemanticsFlag.hasSelectedState,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap],
            ),
            TestSemantics.rootChild(
              label: 'Child 2',
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.hasSelectedState,
                SemanticsFlag.isSelected,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Non-centered taps work on smaller widgets', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');

    int sharedValue = 1;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(sharedValue, 1);

    final double childWidth = getChildSize(tester).width;
    final Offset centerOfSegmentedControl = tester.getCenter(find.text('Child 1'));

    // Tap just inside segment bounds
    await tester.tapAt(
      Offset(centerOfSegmentedControl.dx + (childWidth / 2) - 10.0, centerOfSegmentedControl.dy),
    );

    expect(sharedValue, 0);
  });

  testWidgets('Hit-tests report accurate local position in segments', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    late TapDownDetails tapDownDetails;
    children[0] = GestureDetector(
      behavior: HitTestBehavior.opaque,
      onTapDown: (TapDownDetails details) {
        tapDownDetails = details;
      },
      child: const SizedBox(width: 200, height: 200),
    );
    children[1] = const Text('Child 2');

    int sharedValue = 1;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(sharedValue, 1);

    final Offset segment0GlobalOffset = tester.getTopLeft(find.byWidget(children[0]!));
    await tester.tapAt(segment0GlobalOffset + const Offset(7, 11));

    expect(tapDownDetails.localPosition, const Offset(7, 11));
    expect(tapDownDetails.globalPosition, segment0GlobalOffset + const Offset(7, 11));
  });

  testWidgets('Segment still hittable with a child that has no hitbox', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/57326.
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const SizedBox();
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(sharedValue, 0);

    final Offset centerOfTwo = tester.getCenter(find.byWidget(children[1]!));
    // Tap within the bounds of children[1], but not at the center.
    // children[1] is a SizedBox thus not hittable by itself.
    await tester.tapAt(centerOfTwo + const Offset(10, 0));

    expect(sharedValue, 1);
  });

  testWidgets('Animation is correct when the selected segment changes', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(setupSimpleSegmentedControl());

    await tester.tap(find.text('Child 2'));

    await tester.pump();
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.activeBlue));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x33007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff3d9aff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x64007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff7bbaff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x95007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xffb9daff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0xc7007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xfff7faff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0xf8007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.activeBlue));
  });

  testWidgets('Animation is correct when widget is rebuilt', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('Child 1');
    children[1] = const Text('Child 2');
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('Child 2'));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.activeBlue));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x33007aff)));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
      duration: const Duration(milliseconds: 40),
    );
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff3d9aff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x64007aff)));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
      duration: const Duration(milliseconds: 40),
    );
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff7bbaff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x95007aff)));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
      duration: const Duration(milliseconds: 40),
    );
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xffb9daff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0xc7007aff)));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
      duration: const Duration(milliseconds: 40),
    );
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xfff7faff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0xf8007aff)));

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
      duration: const Duration(milliseconds: 40),
    );
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.activeBlue));
  });

  testWidgets('Multiple segments are pressed', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('A');
    children[1] = const Text('B');
    children[2] = const Text('C');
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    final TestGesture gesture1 = await tester.startGesture(tester.getCenter(find.text('B')));
    await tester.pumpAndSettle(const Duration(milliseconds: 200));

    expect(getBackgroundColor(tester, 1), const Color(0x33007aff));
    expect(getBackgroundColor(tester, 2), isSameColorAs(CupertinoColors.white));

    final TestGesture gesture2 = await tester.startGesture(tester.getCenter(find.text('C')));
    await tester.pumpAndSettle(const Duration(milliseconds: 200));

    // Press on C has no effect while B is held down.
    expect(getBackgroundColor(tester, 1), const Color(0x33007aff));
    expect(getBackgroundColor(tester, 2), isSameColorAs(CupertinoColors.white));

    // Finish gesture to release resources.
    await gesture1.up();
    await gesture2.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Transition is triggered while a transition is already occurring', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('A');
    children[1] = const Text('B');
    children[2] = const Text('C');
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('B'));

    await tester.pump();
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.activeBlue));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x33007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff3d9aff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x64007aff)));

    // While A to B transition is occurring, press on C.
    await tester.tap(find.text('C'));

    await tester.pump();

    // A and B are now both transitioning to white.
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff3d9aff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0xffc1deff)));
    expect(getBackgroundColor(tester, 2), isSameColorAs(const Color(0x33007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    // B background color has reached unselected state.
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff7bbaff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 2), isSameColorAs(const Color(0x64007aff)));

    await tester.pump(const Duration(milliseconds: 100));
    // A background color has reached unselected state.
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 2), isSameColorAs(const Color(0xe0007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    // C background color has reached selected state.
    expect(getBackgroundColor(tester, 2), isSameColorAs(CupertinoColors.activeBlue));
  });

  testWidgets('Segment is selected while it is transitioning to unselected state', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(setupSimpleSegmentedControl());

    await tester.tap(find.text('Child 2'));

    await tester.pump();
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.activeBlue));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x33007aff)));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff3d9aff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x64007aff)));

    // While A to B transition is occurring, press on A again.
    await tester.tap(find.text('Child 1'));

    await tester.pump();

    // Both transitions start to reverse.
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xcd007aff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0xffc1deff)));

    await tester.pump(const Duration(milliseconds: 40));
    // A and B finish transitioning.
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.activeBlue));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Add segment while animation is running', (WidgetTester tester) async {
    Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('A');
    children[1] = const Text('B');
    children[2] = const Text('C');
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
                if (sharedValue == 1) {
                  children = Map<int, Widget>.from(children);
                  children[3] = const Text('D');
                }
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('B'));

    await tester.pump();
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 3), isSameColorAs(CupertinoColors.white));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 3), isSameColorAs(CupertinoColors.white));

    await tester.pump(const Duration(milliseconds: 150));
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), CupertinoColors.activeBlue);
    expect(getBackgroundColor(tester, 3), isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Remove segment while animation is running', (WidgetTester tester) async {
    Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('A');
    children[1] = const Text('B');
    children[2] = const Text('C');
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
                if (sharedValue == 1) {
                  children.remove(2);
                  children = Map<int, Widget>.from(children);
                }
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(getChildCount(tester), 3);

    await tester.tap(find.text('B'));

    await tester.pump();
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x33007aff)));
    expect(getChildCount(tester), 2);

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 1), isSameColorAs(const Color(0x64007aff)));

    await tester.pump(const Duration(milliseconds: 150));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.activeBlue));
  });

  testWidgets('Remove currently animating segment', (WidgetTester tester) async {
    Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('A');
    children[1] = const Text('B');
    children[2] = const Text('C');
    int? sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
                if (sharedValue == 1) {
                  children.remove(1);
                  children = Map<int, Widget>.from(children);
                  sharedValue = null;
                }
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(getChildCount(tester), 3);

    await tester.tap(find.text('B'));

    await tester.pump();
    expect(getChildCount(tester), 2);

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff3d9aff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    await tester.pump(const Duration(milliseconds: 40));
    expect(getBackgroundColor(tester, 0), isSameColorAs(const Color(0xff7bbaff)));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    await tester.pump(const Duration(milliseconds: 100));
    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));
  });

  // Regression test: https://github.com/flutter/flutter/issues/43414.
  testWidgets("Quick double tap doesn't break the internal state", (WidgetTester tester) async {
    const Map<int, Widget> children = <int, Widget>{0: Text('A'), 1: Text('B'), 2: Text('C')};
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('B'));
    // sharedValue has been updated but widget.groupValue is not.
    expect(sharedValue, 1);

    // Land the second tap before the widget gets a chance to rebuild.
    final TestGesture secondTap = await tester.startGesture(tester.getCenter(find.text('B')));
    await tester.pump();

    await secondTap.up();
    expect(sharedValue, 1);

    await tester.tap(find.text('C'));
    expect(sharedValue, 2);
  });

  testWidgets('Golden Test Placeholder Widget', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = Container();
    children[1] = const Placeholder();
    children[2] = Container();

    const int currentValue = 0;

    await tester.pumpWidget(
      RepaintBoundary(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return boilerplate(
              child: SizedBox(
                width: 800.0,
                child: CupertinoSegmentedControl<int>(
                  key: const ValueKey<String>('Segmented Control'),
                  children: children,
                  onValueChanged: (int newValue) {},
                  groupValue: currentValue,
                ),
              ),
            );
          },
        ),
      ),
    );

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('segmented_control_test.0.png'),
    );
  });

  testWidgets('Golden Test Pressed State', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('A');
    children[1] = const Text('B');
    children[2] = const Text('C');

    const int currentValue = 0;

    await tester.pumpWidget(
      RepaintBoundary(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return boilerplate(
              child: SizedBox(
                width: 800.0,
                child: CupertinoSegmentedControl<int>(
                  key: const ValueKey<String>('Segmented Control'),
                  children: children,
                  onValueChanged: (int newValue) {},
                  groupValue: currentValue,
                ),
              ),
            );
          },
        ),
      ),
    );

    final Offset center = tester.getCenter(find.text('B'));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    await expectLater(
      find.byType(RepaintBoundary),
      matchesGoldenFile('segmented_control_test.1.png'),
    );

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Hovering over Cupertino segmented control updates cursor to clickable on Web', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{};
    children[0] = const Text('A');
    children[1] = const Text('B');
    children[2] = const Text('C');

    const int currentValue = 0;

    await tester.pumpWidget(
      RepaintBoundary(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return boilerplate(
              child: SizedBox(
                width: 800.0,
                child: CupertinoSegmentedControl<int>(
                  key: const ValueKey<String>('Segmented Control'),
                  children: children,
                  onValueChanged: (int newValue) {},
                  groupValue: currentValue,
                ),
              ),
            );
          },
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: const Offset(10, 10));
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    final Offset firstChild = tester.getCenter(find.text('A'));
    await gesture.moveTo(firstChild);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      kIsWeb ? SystemMouseCursors.click : SystemMouseCursors.basic,
    );
  });

  testWidgets('Tap on disabled segment should not change its state', (WidgetTester tester) async {
    final Map<int, Widget> children = <int, Widget>{
      0: const Text('Child 1'),
      1: const Text('Child 2'),
      2: const Text('Child 3'),
    };

    final Set<int> disabledChildren = <int>{1};

    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              key: const ValueKey<String>('Segmented Control'),
              children: children,
              disabledChildren: disabledChildren,
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    expect(sharedValue, 0);

    await tester.tap(find.text('Child 2'));
    await tester.pumpAndSettle();

    expect(sharedValue, 0);
  });

  testWidgets('Background color of disabled segment should be different than enabled segment', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{
      0: const Text('Child 1'),
      1: const Text('Child 2'),
    };
    int sharedValue = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              disabledChildren: const <int>{0},
              onValueChanged: (int newValue) {
                setState(() {
                  sharedValue = newValue;
                });
              },
              groupValue: sharedValue,
            ),
          );
        },
      ),
    );

    // Colors are different for disabled and enabled segments in initial state.
    // By default, the first segment is selected (and also is disabled in this test),
    // it should have a blue background (selected color) with 50% opacity
    expect(
      getBackgroundColor(tester, 0),
      isSameColorAs(CupertinoColors.systemBlue.withOpacity(0.5)),
    );
    expect(getBackgroundColor(tester, 1), isSameColorAs(CupertinoColors.white));

    // Tap on disabled segment should not change its color
    await tester.tap(find.text('Child 1'));
    await tester.pumpAndSettle();

    expect(
      getBackgroundColor(tester, 0),
      isSameColorAs(CupertinoColors.systemBlue.withOpacity(0.5)),
    );

    // When tapping on another enabled segment, the first disabled segment is not selected anymore,
    // it should have a white background (same to unselected color).
    await tester.tap(find.text('Child 2'));
    await tester.pumpAndSettle();

    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.white));
  });

  testWidgets('Custom disabled color of disabled segment is showing as desired', (
    WidgetTester tester,
  ) async {
    final Map<int, Widget> children = <int, Widget>{
      0: const Text('Child 1'),
      1: const Text('Child 2'),
      2: const Text('Child 3'),
    };

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return boilerplate(
            child: CupertinoSegmentedControl<int>(
              children: children,
              disabledChildren: const <int>{0},
              onValueChanged: (int newValue) {},
              disabledColor: CupertinoColors.systemGrey2,
            ),
          );
        },
      ),
    );

    expect(getBackgroundColor(tester, 0), isSameColorAs(CupertinoColors.systemGrey2));
  });
}
