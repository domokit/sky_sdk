// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

const List<String> menuItems = <String>['one', 'two', 'three', 'four'];
void onChanged<T>(T _) {}
Finder _iconRichText(Key iconKey) {
  return find.descendant(of: find.byKey(iconKey), matching: find.byType(RichText));
}

Widget buildFormFrame({
  Key? buttonKey,
  AutovalidateMode autovalidateMode = AutovalidateMode.disabled,
  int elevation = 8,
  String? value = 'two',
  ValueChanged<String?>? onChanged,
  VoidCallback? onTap,
  Widget? icon,
  Color? iconDisabledColor,
  Color? iconEnabledColor,
  double iconSize = 24.0,
  bool isDense = true,
  bool isExpanded = false,
  Widget? hint,
  Widget? disabledHint,
  Widget? underline,
  List<String>? items = menuItems,
  Alignment alignment = Alignment.center,
  TextDirection textDirection = TextDirection.ltr,
  AlignmentGeometry buttonAlignment = AlignmentDirectional.centerStart,
}) {
  return TestApp(
    textDirection: textDirection,
    child: Material(
      child: Align(
        alignment: alignment,
        child: RepaintBoundary(
          child: DropdownButtonFormField<String>(
            key: buttonKey,
            autovalidateMode: autovalidateMode,
            elevation: elevation,
            value: value,
            hint: hint,
            disabledHint: disabledHint,
            onChanged: onChanged,
            onTap: onTap,
            icon: icon,
            iconSize: iconSize,
            iconDisabledColor: iconDisabledColor,
            iconEnabledColor: iconEnabledColor,
            isDense: isDense,
            isExpanded: isExpanded,
            items:
                items?.map<DropdownMenuItem<String>>((String item) {
                  return DropdownMenuItem<String>(
                    key: ValueKey<String>(item),
                    value: item,
                    child: Text(item, key: ValueKey<String>('${item}Text')),
                  );
                }).toList(),
            alignment: buttonAlignment,
          ),
        ),
      ),
    ),
  );
}

class _TestAppState extends State<TestApp> {
  @override
  Widget build(BuildContext context) {
    return Localizations(
      locale: const Locale('en', 'US'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultWidgetsLocalizations.delegate,
        DefaultMaterialLocalizations.delegate,
      ],
      child: MediaQuery(
        data: const MediaQueryData().copyWith(size: widget.mediaSize),
        child: Directionality(
          textDirection: widget.textDirection,
          child: Navigator(
            onGenerateRoute: (RouteSettings settings) {
              assert(settings.name == '/');
              return MaterialPageRoute<void>(
                settings: settings,
                builder: (BuildContext context) => widget.child,
              );
            },
          ),
        ),
      ),
    );
  }
}

class TestApp extends StatefulWidget {
  const TestApp({super.key, required this.textDirection, required this.child, this.mediaSize});

  final TextDirection textDirection;
  final Widget child;
  final Size? mediaSize;

  @override
  State<TestApp> createState() => _TestAppState();
}

void verifyPaintedShadow(Finder customPaint, int elevation) {
  const Rect originalRectangle = Rect.fromLTRB(0.0, 0.0, 800, 208.0);

  final List<BoxShadow> boxShadows = List<BoxShadow>.generate(
    3,
    (int index) => kElevationToShadow[elevation]![index],
  );
  final List<RRect> rrects = List<RRect>.generate(3, (int index) {
    return RRect.fromRectAndRadius(
      originalRectangle.shift(boxShadows[index].offset).inflate(boxShadows[index].spreadRadius),
      const Radius.circular(2.0),
    );
  });

  expect(
    customPaint,
    paints
      ..save()
      ..rrect(rrect: rrects[0], color: boxShadows[0].color, hasMaskFilter: true)
      ..rrect(rrect: rrects[1], color: boxShadows[1].color, hasMaskFilter: true)
      ..rrect(rrect: rrects[2], color: boxShadows[2].color, hasMaskFilter: true),
  );
}

void main() {
  // Regression test for https://github.com/flutter/flutter/issues/87102
  testWidgets('label position test - show hint', (WidgetTester tester) async {
    int? value;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            hint: const Text('Hint'),
            onChanged: (int? newValue) {
              value = newValue;
            },
            items: const <DropdownMenuItem<int?>>[
              DropdownMenuItem<int?>(value: 1, child: Text('One')),
              DropdownMenuItem<int?>(value: 2, child: Text('Two')),
              DropdownMenuItem<int?>(value: 3, child: Text('Three')),
            ],
          ),
        ),
      ),
    );

    expect(value, null);
    final Offset hintEmptyLabel = tester.getTopLeft(find.text('labelText'));

    // Select a item.
    await tester.tap(find.text('Hint'), warnIfMissed: false);
    await tester.pumpAndSettle();
    await tester.tap(find.text('One').last);
    await tester.pumpAndSettle();

    expect(value, 1);
    final Offset oneValueLabel = tester.getTopLeft(find.text('labelText'));

    // The position of the label does not change.
    expect(hintEmptyLabel, oneValueLabel);
  });

  testWidgets('label position test - show disabledHint: disable', (WidgetTester tester) async {
    int? value;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            onChanged: null, // this disables the menu and shows the disabledHint.
            disabledHint: const Text('disabledHint'),
            items: const <DropdownMenuItem<int?>>[
              DropdownMenuItem<int?>(value: 1, child: Text('One')),
              DropdownMenuItem<int?>(value: 2, child: Text('Two')),
              DropdownMenuItem<int?>(value: 3, child: Text('Three')),
            ],
          ),
        ),
      ),
    );

    expect(value, null); // disabledHint shown.
    final Offset hintEmptyLabel = tester.getTopLeft(find.text('labelText'));
    expect(hintEmptyLabel, const Offset(0.0, 8.0));
  });

  testWidgets('label position test - show disabledHint: enable + null item', (
    WidgetTester tester,
  ) async {
    int? value;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            disabledHint: const Text('disabledHint'),
            onChanged: (_) {},
            items: null,
          ),
        ),
      ),
    );

    expect(value, null); // disabledHint shown.
    final Offset hintEmptyLabel = tester.getTopLeft(find.text('labelText'));
    expect(hintEmptyLabel, const Offset(0.0, 8.0));
  });

  testWidgets('label position test - show disabledHint: enable + empty item', (
    WidgetTester tester,
  ) async {
    int? value;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            disabledHint: const Text('disabledHint'),
            onChanged: (_) {},
            items: const <DropdownMenuItem<int?>>[],
          ),
        ),
      ),
    );

    expect(value, null); // disabledHint shown.
    final Offset hintEmptyLabel = tester.getTopLeft(find.text('labelText'));
    expect(hintEmptyLabel, const Offset(0.0, 8.0));
  });

  testWidgets('label position test - show hint: enable + empty item', (WidgetTester tester) async {
    int? value;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            hint: const Text('hint'),
            onChanged: (_) {},
            items: const <DropdownMenuItem<int?>>[],
          ),
        ),
      ),
    );

    expect(value, null); // hint shown.
    final Offset hintEmptyLabel = tester.getTopLeft(find.text('labelText'));
    expect(hintEmptyLabel, const Offset(0.0, 8.0));
  });

  testWidgets('label position test - no hint shown: enable + no selected + disabledHint', (
    WidgetTester tester,
  ) async {
    int? value;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            disabledHint: const Text('disabledHint'),
            onChanged: (_) {},
            items: const <DropdownMenuItem<int?>>[
              DropdownMenuItem<int?>(value: 1, child: Text('One')),
              DropdownMenuItem<int?>(value: 2, child: Text('Two')),
              DropdownMenuItem<int?>(value: 3, child: Text('Three')),
            ],
          ),
        ),
      ),
    );

    expect(value, null);
    final Offset hintEmptyLabel = tester.getTopLeft(find.text('labelText'));
    expect(hintEmptyLabel, const Offset(0.0, 20.0));
  });

  testWidgets('label position test - show selected item: disabled + hint + disabledHint', (
    WidgetTester tester,
  ) async {
    const int value = 1;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            hint: const Text('hint'),
            onChanged: null, // disabled
            disabledHint: const Text('disabledHint'),
            items: const <DropdownMenuItem<int?>>[
              DropdownMenuItem<int?>(value: 1, child: Text('One')),
              DropdownMenuItem<int?>(value: 2, child: Text('Two')),
              DropdownMenuItem<int?>(value: 3, child: Text('Three')),
            ],
          ),
        ),
      ),
    );

    expect(value, 1);
    final Offset hintEmptyLabel = tester.getTopLeft(find.text('labelText'));
    expect(hintEmptyLabel, const Offset(0.0, 8.0));
  });

  // Regression test for https://github.com/flutter/flutter/issues/82910
  testWidgets('null value test', (WidgetTester tester) async {
    int? value = 1;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<int?>(
            decoration: const InputDecoration(labelText: 'labelText'),
            value: value,
            onChanged: (int? newValue) {
              value = newValue;
            },
            items: const <DropdownMenuItem<int?>>[
              DropdownMenuItem<int?>(child: Text('None')),
              DropdownMenuItem<int?>(value: 1, child: Text('One')),
              DropdownMenuItem<int?>(value: 2, child: Text('Two')),
              DropdownMenuItem<int?>(value: 3, child: Text('Three')),
            ],
          ),
        ),
      ),
    );

    expect(value, 1);
    final Offset nonEmptyLabel = tester.getTopLeft(find.text('labelText'));

    // Switch to `null` value item from value 1.
    await tester.tap(find.text('One'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('None').last);
    await tester.pump();

    expect(value, null);
    final Offset nullValueLabel = tester.getTopLeft(find.text('labelText'));
    // The position of the label does not change.
    expect(nonEmptyLabel, nullValueLabel);
  });

  testWidgets('DropdownButtonFormField with autovalidation test', (WidgetTester tester) async {
    String? value = 'one';
    int validateCalled = 0;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Material(
              child: DropdownButtonFormField<String>(
                value: value,
                hint: const Text('Select Value'),
                decoration: const InputDecoration(prefixIcon: Icon(Icons.fastfood)),
                items:
                    menuItems.map((String value) {
                      return DropdownMenuItem<String>(value: value, child: Text(value));
                    }).toList(),
                onChanged: (String? newValue) {
                  setState(() {
                    value = newValue;
                  });
                },
                validator: (String? currentValue) {
                  validateCalled++;
                  return currentValue == null ? 'Must select value' : null;
                },
                autovalidateMode: AutovalidateMode.always,
              ),
            ),
          );
        },
      ),
    );

    expect(validateCalled, 1);
    expect(value, equals('one'));
    await tester.tap(find.text('one'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('three').last);
    await tester.pump();
    expect(validateCalled, 2);
    await tester.pumpAndSettle();
    expect(value, equals('three'));
  });

  testWidgets('DropdownButtonFormField arrow icon aligns with the edge of button when expanded', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    // There shouldn't be overflow when expanded although list contains longer items.
    final List<String> items = <String>[
      '1234567890',
      'abcdefghijklmnopqrstuvwxyz1234567890abcdefghijklmnopqrstuvwxyz1234567890',
    ];

    await tester.pumpWidget(
      buildFormFrame(
        buttonKey: buttonKey,
        value: '1234567890',
        isExpanded: true,
        onChanged: onChanged,
        items: items,
      ),
    );
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    expect(buttonBox.attached, isTrue);

    final RenderBox arrowIcon = tester.renderObject<RenderBox>(find.byIcon(Icons.arrow_drop_down));
    expect(arrowIcon.attached, isTrue);

    // Arrow icon should be aligned with far right of button when expanded
    expect(
      arrowIcon.localToGlobal(Offset.zero).dx,
      buttonBox.size.centerRight(Offset(-arrowIcon.size.width, 0.0)).dx,
    );
  });

  testWidgets('DropdownButtonFormField with isDense:true aligns selected menu item', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    await tester.pumpWidget(buildFormFrame(buttonKey: buttonKey, onChanged: onChanged));
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    expect(buttonBox.attached, isTrue);

    await tester.tap(find.text('two'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    // The selected dropdown item is both in menu we just popped up, and in
    // the IndexedStack contained by the dropdown button. Both of them should
    // have the same vertical center as the button.
    final List<RenderBox> itemBoxes =
        tester.renderObjectList<RenderBox>(find.byKey(const ValueKey<String>('two'))).toList();
    expect(itemBoxes.length, equals(2));

    // When isDense is true, the button's height is reduced. The menu items'
    // heights are not.
    final List<double> itemBoxesHeight =
        itemBoxes.map<double>((RenderBox box) => box.size.height).toList();
    final double menuItemHeight = itemBoxesHeight.reduce(math.max);
    expect(menuItemHeight, greaterThanOrEqualTo(buttonBox.size.height));

    for (final RenderBox itemBox in itemBoxes) {
      expect(itemBox.attached, isTrue);
      final Offset buttonBoxCenter = buttonBox.size.center(buttonBox.localToGlobal(Offset.zero));
      final Offset itemBoxCenter = itemBox.size.center(itemBox.localToGlobal(Offset.zero));
      expect(buttonBoxCenter.dy, equals(itemBoxCenter.dy));
    }
  });

  testWidgets('DropdownButtonFormField with isDense:true does not clip large scale text', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();
    const String value = 'two';

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Builder(
          builder:
              (BuildContext context) => MediaQuery.withClampedTextScaling(
                minScaleFactor: 3.0,
                maxScaleFactor: 3.0,
                child: Material(
                  child: Center(
                    child: DropdownButtonFormField<String>(
                      key: buttonKey,
                      value: value,
                      onChanged: onChanged,
                      items:
                          menuItems.map<DropdownMenuItem<String>>((String item) {
                            return DropdownMenuItem<String>(
                              key: ValueKey<String>(item),
                              value: item,
                              child: Text(
                                item,
                                key: ValueKey<String>('${item}Text'),
                                style: const TextStyle(fontSize: 20.0),
                              ),
                            );
                          }).toList(),
                    ),
                  ),
                ),
              ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject<RenderBox>(find.byType(DropdownButton<String>));
    expect(box.size.height, 64.0);
  });

  testWidgets('DropdownButtonFormField.isDense is true by default', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/46844
    final Key buttonKey = UniqueKey();
    const String value = 'two';

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Center(
            child: DropdownButtonFormField<String>(
              key: buttonKey,
              value: value,
              onChanged: onChanged,
              items:
                  menuItems.map<DropdownMenuItem<String>>((String item) {
                    return DropdownMenuItem<String>(
                      key: ValueKey<String>(item),
                      value: item,
                      child: Text(item, key: ValueKey<String>('${item}Text')),
                    );
                  }).toList(),
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject<RenderBox>(find.byType(DropdownButton<String>));
    expect(box.size.height, 48.0);
  });

  testWidgets('DropdownButtonFormField - custom text style', (WidgetTester tester) async {
    const String value = 'foo';
    final UniqueKey itemKey = UniqueKey();

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButtonFormField<String>(
            value: value,
            items: <DropdownMenuItem<String>>[
              DropdownMenuItem<String>(key: itemKey, value: 'foo', child: const Text(value)),
            ],
            onChanged: (_) {},
            style: const TextStyle(color: Colors.amber, fontSize: 20.0),
          ),
        ),
      ),
    );

    final RichText richText = tester.widget<RichText>(
      find.descendant(of: find.byKey(itemKey), matching: find.byType(RichText)),
    );

    expect(richText.text.style!.color, Colors.amber);
    expect(richText.text.style!.fontSize, 20.0);
  });

  testWidgets(
    'DropdownButtonFormField - disabledHint displays when the items list is empty, when items is null',
    (WidgetTester tester) async {
      final Key buttonKey = UniqueKey();

      Widget build({List<String>? items}) {
        return buildFormFrame(
          items: items,
          buttonKey: buttonKey,
          value: null,
          hint: const Text('enabled'),
          disabledHint: const Text('disabled'),
        );
      }

      // [disabledHint] should display when [items] is null
      await tester.pumpWidget(build());
      expect(find.text('enabled'), findsNothing);
      expect(find.text('disabled'), findsOneWidget);

      // [disabledHint] should display when [items] is an empty list.
      await tester.pumpWidget(build(items: <String>[]));
      expect(find.text('enabled'), findsNothing);
      expect(find.text('disabled'), findsOneWidget);
    },
  );

  testWidgets('DropdownButtonFormField - hint displays when the items list is '
      'empty, items is null, and disabledHint is null', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items}) {
      return buildFormFrame(
        items: items,
        buttonKey: buttonKey,
        value: null,
        hint: const Text('hint used when disabled'),
      );
    }

    // [hint] should display when [items] is null and [disabledHint] is not defined
    await tester.pumpWidget(build());
    expect(find.text('hint used when disabled'), findsOneWidget);

    // [hint] should display when [items] is an empty list and [disabledHint] is not defined.
    await tester.pumpWidget(build(items: <String>[]));
    expect(find.text('hint used when disabled'), findsOneWidget);
  });

  testWidgets('DropdownButtonFormField - disabledHint is null by default', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items}) {
      return buildFormFrame(
        items: items,
        buttonKey: buttonKey,
        value: null,
        hint: const Text('hint used when disabled'),
      );
    }

    // [hint] should display when [items] is null and [disabledHint] is not defined
    await tester.pumpWidget(build());
    expect(find.text('hint used when disabled'), findsOneWidget);

    // [hint] should display when [items] is an empty list and [disabledHint] is not defined.
    await tester.pumpWidget(build(items: <String>[]));
    expect(find.text('hint used when disabled'), findsOneWidget);
  });

  testWidgets('DropdownButtonFormField - disabledHint is null by default', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items}) {
      return buildFormFrame(
        items: items,
        buttonKey: buttonKey,
        value: null,
        hint: const Text('hint used when disabled'),
      );
    }

    // [hint] should display when [items] is null and [disabledHint] is not defined
    await tester.pumpWidget(build());
    expect(find.text('hint used when disabled'), findsOneWidget);

    // [hint] should display when [items] is an empty list and [disabledHint] is not defined.
    await tester.pumpWidget(build(items: <String>[]));
    expect(find.text('hint used when disabled'), findsOneWidget);
  });

  testWidgets('DropdownButtonFormField - disabledHint displays when onChanged is null', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items, ValueChanged<String?>? onChanged}) {
      return buildFormFrame(
        items: items,
        buttonKey: buttonKey,
        value: null,
        onChanged: onChanged,
        hint: const Text('enabled'),
        disabledHint: const Text('disabled'),
      );
    }

    await tester.pumpWidget(build(items: menuItems));
    expect(find.text('enabled'), findsNothing);
    expect(find.text('disabled'), findsOneWidget);
  });

  testWidgets('DropdownButtonFormField - disabled hint should be of same size as enabled hint', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items}) {
      return buildFormFrame(
        items: items,
        buttonKey: buttonKey,
        value: null,
        hint: const Text('enabled'),
        disabledHint: const Text('disabled'),
      );
    }

    await tester.pumpWidget(build());
    final RenderBox disabledHintBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));

    await tester.pumpWidget(build(items: menuItems));
    final RenderBox enabledHintBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    expect(
      enabledHintBox.localToGlobal(Offset.zero),
      equals(disabledHintBox.localToGlobal(Offset.zero)),
    );
    expect(enabledHintBox.size, equals(disabledHintBox.size));
  });

  testWidgets('DropdownButtonFormField - Custom icon size and colors', (WidgetTester tester) async {
    final Key iconKey = UniqueKey();
    final Icon customIcon = Icon(Icons.assessment, key: iconKey);

    await tester.pumpWidget(
      buildFormFrame(
        icon: customIcon,
        iconSize: 30.0,
        iconEnabledColor: Colors.pink,
        iconDisabledColor: Colors.orange,
        onChanged: onChanged,
      ),
    );

    // test for size
    final RenderBox icon = tester.renderObject(find.byKey(iconKey));
    expect(icon.size, const Size(30.0, 30.0));

    // test for enabled color
    final RichText enabledRichText = tester.widget<RichText>(_iconRichText(iconKey));
    expect(enabledRichText.text.style!.color, Colors.pink);

    // test for disabled color
    await tester.pumpWidget(
      buildFormFrame(
        icon: customIcon,
        iconSize: 30.0,
        iconEnabledColor: Colors.pink,
        iconDisabledColor: Colors.orange,
        items: null,
      ),
    );

    final RichText disabledRichText = tester.widget<RichText>(_iconRichText(iconKey));
    expect(disabledRichText.text.style!.color, Colors.orange);
  });

  testWidgets('DropdownButtonFormField - default elevation', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    debugDisableShadows = false;
    await tester.pumpWidget(buildFormFrame(buttonKey: buttonKey, onChanged: onChanged));
    await tester.tap(find.byKey(buttonKey));
    await tester.pumpAndSettle();

    final Finder customPaint =
        find.ancestor(of: find.text('one').last, matching: find.byType(CustomPaint)).last;

    // Verifying whether or not default elevation(i.e. 8) paints desired shadow
    verifyPaintedShadow(customPaint, 8);
    debugDisableShadows = true;
  });

  testWidgets('DropdownButtonFormField - custom elevation', (WidgetTester tester) async {
    debugDisableShadows = false;
    final Key buttonKeyOne = UniqueKey();
    final Key buttonKeyTwo = UniqueKey();

    await tester.pumpWidget(
      buildFormFrame(buttonKey: buttonKeyOne, elevation: 16, onChanged: onChanged),
    );
    await tester.tap(find.byKey(buttonKeyOne));
    await tester.pumpAndSettle();

    final Finder customPaintOne =
        find.ancestor(of: find.text('one').last, matching: find.byType(CustomPaint)).last;

    verifyPaintedShadow(customPaintOne, 16);
    await tester.tap(find.text('one').last);
    await tester.pumpWidget(
      buildFormFrame(buttonKey: buttonKeyTwo, elevation: 24, onChanged: onChanged),
    );
    await tester.tap(find.byKey(buttonKeyTwo));
    await tester.pumpAndSettle();

    final Finder customPaintTwo =
        find.ancestor(of: find.text('one').last, matching: find.byType(CustomPaint)).last;

    verifyPaintedShadow(customPaintTwo, 24);
    debugDisableShadows = true;
  });

  testWidgets('DropdownButtonFormField does not allow duplicate item values', (
    WidgetTester tester,
  ) async {
    final List<DropdownMenuItem<String>> itemsWithDuplicateValues =
        <String>['a', 'b', 'c', 'c'].map<DropdownMenuItem<String>>((String value) {
          return DropdownMenuItem<String>(value: value, child: Text(value));
        }).toList();

    await expectLater(
      () => tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: DropdownButtonFormField<String>(
              value: 'c',
              onChanged: (String? newValue) {},
              items: itemsWithDuplicateValues,
            ),
          ),
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains("There should be exactly one item with [DropdownButton]'s value"),
        ),
      ),
    );
  });

  testWidgets('DropdownButtonFormField value should only appear in one menu item', (
    WidgetTester tester,
  ) async {
    final List<DropdownMenuItem<String>> itemsWithDuplicateValues =
        <String>['a', 'b', 'c', 'd'].map<DropdownMenuItem<String>>((String value) {
          return DropdownMenuItem<String>(value: value, child: Text(value));
        }).toList();

    await expectLater(
      () => tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: DropdownButton<String>(
              value: 'e',
              onChanged: (String? newValue) {},
              items: itemsWithDuplicateValues,
            ),
          ),
        ),
      ),
      throwsA(
        isAssertionError.having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains("There should be exactly one item with [DropdownButton]'s value"),
        ),
      ),
    );
  });

  testWidgets('DropdownButtonFormField - selectedItemBuilder builds custom buttons', (
    WidgetTester tester,
  ) async {
    const List<String> items = <String>['One', 'Two', 'Three'];
    String? selectedItem = items[0];

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Scaffold(
              body: DropdownButtonFormField<String>(
                value: selectedItem,
                onChanged: (String? string) => setState(() => selectedItem = string),
                selectedItemBuilder: (BuildContext context) {
                  int index = 0;
                  return items.map((String string) {
                    index += 1;
                    return Text('$string as an Arabic numeral: $index');
                  }).toList();
                },
                items:
                    items.map((String string) {
                      return DropdownMenuItem<String>(value: string, child: Text(string));
                    }).toList(),
              ),
            ),
          );
        },
      ),
    );

    expect(find.text('One as an Arabic numeral: 1'), findsOneWidget);
    await tester.tap(find.text('One as an Arabic numeral: 1'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Two'));
    await tester.pumpAndSettle();
    expect(find.text('Two as an Arabic numeral: 2'), findsOneWidget);
  });

  testWidgets('DropdownButton onTap callback is called when defined', (WidgetTester tester) async {
    int dropdownButtonTapCounter = 0;
    String? value = 'one';
    void onChanged(String? newValue) {
      value = newValue;
    }

    void onTap() {
      dropdownButtonTapCounter += 1;
    }

    Widget build() => buildFormFrame(value: value, onChanged: onChanged, onTap: onTap);
    await tester.pumpWidget(build());

    expect(dropdownButtonTapCounter, 0);

    // Tap dropdown button.
    await tester.tap(find.text('one'));
    await tester.pumpAndSettle();

    expect(value, equals('one'));
    expect(dropdownButtonTapCounter, 1); // Should update counter.

    // Tap dropdown menu item.
    await tester.tap(find.text('three').last);
    await tester.pumpAndSettle();

    expect(value, equals('three'));
    expect(dropdownButtonTapCounter, 1); // Should not change.

    // Tap dropdown button again.
    await tester.tap(find.text('three'));
    await tester.pumpAndSettle();

    expect(value, equals('three'));
    expect(dropdownButtonTapCounter, 2); // Should update counter.

    // Tap dropdown menu item.
    await tester.tap(find.text('two').last);
    await tester.pumpAndSettle();

    expect(value, equals('two'));
    expect(dropdownButtonTapCounter, 2); // Should not change.
  });

  testWidgets('DropdownButtonFormField should re-render if value param changes', (
    WidgetTester tester,
  ) async {
    String currentValue = 'two';

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Material(
              child: DropdownButtonFormField<String>(
                value: currentValue,
                onChanged: onChanged,
                items:
                    menuItems.map((String value) {
                      return DropdownMenuItem<String>(
                        value: value,
                        child: Text(value),
                        onTap: () {
                          setState(() {
                            currentValue = value;
                          });
                        },
                      );
                    }).toList(),
              ),
            ),
          );
        },
      ),
    );

    // Make sure the rendered text value matches the initial state value.
    expect(currentValue, equals('two'));
    expect(find.text(currentValue), findsOneWidget);

    // Tap the DropdownButtonFormField widget
    await tester.tap(find.byType(DropdownButton<String>));
    await tester.pumpAndSettle();

    // Tap the first dropdown menu item.
    await tester.tap(find.text('one').last);
    await tester.pumpAndSettle();

    // Make sure the rendered text value matches the updated state value.
    expect(currentValue, equals('one'));
    expect(find.text(currentValue), findsOneWidget);
  });

  testWidgets('autovalidateMode is passed to super', (WidgetTester tester) async {
    int validateCalled = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: DropdownButtonFormField<String>(
              autovalidateMode: AutovalidateMode.always,
              items:
                  menuItems.map((String value) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
              onChanged: onChanged,
              validator: (String? value) {
                validateCalled++;
                return null;
              },
            ),
          ),
        ),
      ),
    );

    expect(validateCalled, 1);
  });

  testWidgets('DropdownButtonFormField - Custom button alignment', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildFormFrame(
        buttonAlignment: AlignmentDirectional.center,
        items: <String>['one'],
        value: 'one',
      ),
    );

    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byType(IndexedStack));
    final RenderBox selectedItemBox = tester.renderObject(find.text('one'));

    // Should be center-center aligned.
    expect(
      buttonBox.localToGlobal(Offset(buttonBox.size.width / 2.0, buttonBox.size.height / 2.0)),
      selectedItemBox.localToGlobal(
        Offset(selectedItemBox.size.width / 2.0, selectedItemBox.size.height / 2.0),
      ),
    );
  });

  testWidgets('DropdownButtonFormField onChanged is called when the form is reset', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/123009.
    final GlobalKey<FormFieldState<String>> stateKey = GlobalKey<FormFieldState<String>>();
    final GlobalKey<FormState> formKey = GlobalKey<FormState>();
    String? value;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Form(
            key: formKey,
            child: DropdownButtonFormField<String>(
              key: stateKey,
              value: 'One',
              items:
                  <String>['One', 'Two', 'Free', 'Four'].map<DropdownMenuItem<String>>((
                    String value,
                  ) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
              onChanged: (String? newValue) {
                value = newValue;
              },
            ),
          ),
        ),
      ),
    );

    // Initial value is 'One'.
    expect(value, isNull);
    expect(stateKey.currentState!.value, equals('One'));

    // Select 'Two'.
    await tester.tap(find.text('One'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Two').last);
    await tester.pumpAndSettle();
    expect(value, equals('Two'));
    expect(stateKey.currentState!.value, equals('Two'));

    // Should be back to 'One' when the form is reset.
    formKey.currentState!.reset();
    expect(value, equals('One'));
    expect(stateKey.currentState!.value, equals('One'));
  });

  testWidgets('DropdownButtonFormField with onChanged set to null does not throw on form reset', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/146335.
    final GlobalKey<FormFieldState<String>> stateKey = GlobalKey<FormFieldState<String>>();
    final GlobalKey<FormState> formKey = GlobalKey<FormState>();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Form(
            key: formKey,
            child: DropdownButtonFormField<String>(
              key: stateKey,
              value: 'One',
              items:
                  <String>['One', 'Two', 'Free', 'Four'].map<DropdownMenuItem<String>>((
                    String value,
                  ) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
              onChanged: null,
            ),
          ),
        ),
      ),
    );

    // Reset the form.
    formKey.currentState!.reset();

    expect(tester.takeException(), isNull);
  });

  // Regression test for https://github.com/flutter/flutter/issues/106659.
  testWidgets('Error visual logic is delegated to InputDecorator', (WidgetTester tester) async {
    final GlobalKey<FormState> formKey = GlobalKey<FormState>();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Form(
            key: formKey,
            child: DropdownButtonFormField<String>(
              items:
                  menuItems.map((String value) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
              onChanged: onChanged,
              validator: (String? v) => 'Required',
              onTap: () {
                formKey.currentState!.validate();
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(InputDecorator));
    await tester.pump();

    // Check InputDecorator state because DropdownButtonFormField delegates
    // visual logic to the InputDecorator.
    final InputDecorator inputDecorator = tester.widget(find.byType(InputDecorator));
    expect(inputDecorator.isFocused, true);
    expect(inputDecorator.decoration.errorText, 'Required');
  });
}
