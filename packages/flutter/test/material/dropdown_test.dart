// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// no-shuffle:
//   //TODO(gspencergoog): Remove this tag once this test's state leaks/test
//   dependencies have been fixed.
//   https://github.com/flutter/flutter/issues/85160
//   Fails with "flutter test --test-randomize-ordering-seed=456"
// reduced-test-set:
//   This file is run as part of a reduced test set in CI on Mac and Windows
//   machines.
@Tags(<String>['reduced-test-set', 'no-shuffle'])
library;

import 'dart:math' as math;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

const List<String> menuItems = <String>['one', 'two', 'three', 'four'];
void onChanged<T>(T _) {}

final Type dropdownButtonType =
    DropdownButton<String>(
      onChanged: (_) {},
      items: const <DropdownMenuItem<String>>[],
    ).runtimeType;

Finder _iconRichText(Key iconKey) {
  return find.descendant(of: find.byKey(iconKey), matching: find.byType(RichText));
}

Widget buildDropdown({
  required bool isFormField,
  Key? buttonKey,
  String? value = 'two',
  ValueChanged<String?>? onChanged,
  VoidCallback? onTap,
  Widget? icon,
  Color? iconDisabledColor,
  Color? iconEnabledColor,
  double iconSize = 24.0,
  bool isDense = false,
  bool isExpanded = false,
  Widget? hint,
  Widget? disabledHint,
  Widget? underline,
  List<String>? items = menuItems,
  List<Widget> Function(BuildContext)? selectedItemBuilder,
  double? itemHeight = kMinInteractiveDimension,
  double? menuWidth,
  AlignmentDirectional alignment = AlignmentDirectional.centerStart,
  TextDirection textDirection = TextDirection.ltr,
  Size? mediaSize,
  FocusNode? focusNode,
  bool autofocus = false,
  Color? focusColor,
  Color? dropdownColor,
  double? menuMaxHeight,
  EdgeInsetsGeometry? padding,
  InputDecoration? decoration,
}) {
  final List<DropdownMenuItem<String>>? listItems =
      items?.map<DropdownMenuItem<String>>((String item) {
        return DropdownMenuItem<String>(
          key: ValueKey<String>(item),
          value: item,
          child: Text(item, key: ValueKey<String>('${item}Text')),
        );
      }).toList();

  if (isFormField) {
    return Form(
      child: DropdownButtonFormField<String>(
        key: buttonKey,
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
        // No underline attribute
        focusNode: focusNode,
        autofocus: autofocus,
        focusColor: focusColor,
        dropdownColor: dropdownColor,
        items: listItems,
        selectedItemBuilder: selectedItemBuilder,
        itemHeight: itemHeight,
        alignment: alignment,
        menuMaxHeight: menuMaxHeight,
        padding: padding,
        decoration: decoration,
      ),
    );
  }
  return DropdownButton<String>(
    key: buttonKey,
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
    underline: underline,
    focusNode: focusNode,
    autofocus: autofocus,
    focusColor: focusColor,
    dropdownColor: dropdownColor,
    items: listItems,
    selectedItemBuilder: selectedItemBuilder,
    itemHeight: itemHeight,
    menuWidth: menuWidth,
    alignment: alignment,
    menuMaxHeight: menuMaxHeight,
    padding: padding,
  );
}

Widget buildFrame({
  Key? buttonKey,
  String? value = 'two',
  ValueChanged<String?>? onChanged,
  VoidCallback? onTap,
  Widget? icon,
  Color? iconDisabledColor,
  Color? iconEnabledColor,
  double iconSize = 24.0,
  bool isDense = false,
  bool isExpanded = false,
  Widget? hint,
  Widget? disabledHint,
  Widget? underline,
  List<String>? items = menuItems,
  List<Widget> Function(BuildContext)? selectedItemBuilder,
  double? itemHeight = kMinInteractiveDimension,
  double? menuWidth,
  AlignmentDirectional alignment = AlignmentDirectional.centerStart,
  TextDirection textDirection = TextDirection.ltr,
  Size? mediaSize,
  FocusNode? focusNode,
  bool autofocus = false,
  Color? focusColor,
  Color? dropdownColor,
  bool isFormField = false,
  double? menuMaxHeight,
  EdgeInsetsGeometry? padding,
  Alignment dropdownAlignment = Alignment.center,
  bool? useMaterial3,
  InputDecoration? decoration,
}) {
  return Theme(
    data: ThemeData(useMaterial3: useMaterial3),
    child: TestApp(
      textDirection: textDirection,
      mediaSize: mediaSize,
      child: Material(
        child: Align(
          alignment: dropdownAlignment,
          child: RepaintBoundary(
            child: buildDropdown(
              isFormField: isFormField,
              buttonKey: buttonKey,
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
              underline: underline,
              focusNode: focusNode,
              autofocus: autofocus,
              focusColor: focusColor,
              dropdownColor: dropdownColor,
              items: items,
              selectedItemBuilder: selectedItemBuilder,
              itemHeight: itemHeight,
              menuWidth: menuWidth,
              alignment: alignment,
              menuMaxHeight: menuMaxHeight,
              padding: padding,
              decoration: decoration,
            ),
          ),
        ),
      ),
    ),
  );
}

Widget buildDropdownWithHint({
  required AlignmentDirectional alignment,
  required bool isExpanded,
  bool enableSelectedItemBuilder = false,
}) {
  return buildFrame(
    useMaterial3: false,
    mediaSize: const Size(800, 600),
    itemHeight: 100.0,
    alignment: alignment,
    isExpanded: isExpanded,
    selectedItemBuilder:
        enableSelectedItemBuilder
            ? (BuildContext context) {
              return menuItems.map<Widget>((String item) {
                return ColoredBox(color: const Color(0xff00ff00), child: Text(item));
              }).toList();
            }
            : null,
    hint: const Text('hint'),
  );
}

class TestApp extends StatefulWidget {
  const TestApp({super.key, required this.textDirection, required this.child, this.mediaSize});

  final TextDirection textDirection;
  final Widget child;
  final Size? mediaSize;

  @override
  State<TestApp> createState() => _TestAppState();
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
        data: MediaQueryData.fromView(View.of(context)).copyWith(size: widget.mediaSize),
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

// When the dropdown's menu is popped up, a RenderParagraph for the selected
// menu's text item will appear both in the dropdown button and in the menu.
// The RenderParagraphs should be aligned, i.e. they should have the same
// size and location.
void checkSelectedItemTextGeometry(WidgetTester tester, String value) {
  final List<RenderBox> boxes =
      tester.renderObjectList<RenderBox>(find.byKey(ValueKey<String>('${value}Text'))).toList();
  expect(boxes.length, equals(2));
  final RenderBox box0 = boxes[0];
  final RenderBox box1 = boxes[1];
  expect(box0.localToGlobal(Offset.zero), equals(box1.localToGlobal(Offset.zero)));
  expect(box0.size, equals(box1.size));
}

// The dropdown menu isn't readily accessible. To find it we're assuming that it
// contains a ListView and that it's an instance of _DropdownMenu.
Rect getMenuRect(WidgetTester tester) {
  late Rect menuRect;
  tester.element(find.byType(ListView)).visitAncestorElements((Element element) {
    if (element.toString().startsWith('_DropdownMenu')) {
      final RenderBox box = element.findRenderObject()! as RenderBox;
      menuRect = box.localToGlobal(Offset.zero) & box.size;
      return false;
    }
    return true;
  });
  return menuRect;
}

Future<void> checkDropdownColor(
  WidgetTester tester, {
  Color? color,
  bool isFormField = false,
}) async {
  const String text = 'foo';
  await tester.pumpWidget(
    MaterialApp(
      theme: ThemeData(useMaterial3: false),
      home: Material(
        child:
            isFormField
                ? Form(
                  child: DropdownButtonFormField<String>(
                    dropdownColor: color,
                    value: text,
                    items: const <DropdownMenuItem<String>>[
                      DropdownMenuItem<String>(value: text, child: Text(text)),
                    ],
                    onChanged: (_) {},
                  ),
                )
                : DropdownButton<String>(
                  dropdownColor: color,
                  value: text,
                  items: const <DropdownMenuItem<String>>[
                    DropdownMenuItem<String>(value: text, child: Text(text)),
                  ],
                  onChanged: (_) {},
                ),
      ),
    ),
  );
  await tester.tap(find.text(text));
  await tester.pump();

  expect(
    find.ancestor(of: find.text(text).last, matching: find.byType(CustomPaint)).at(2),
    paints
      ..save()
      ..rrect()
      ..rrect()
      ..rrect()
      ..rrect(color: color ?? Colors.grey[50], hasMaskFilter: false),
  );
}

void main() {
  testWidgets('Default dropdown golden', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    Widget build() => buildFrame(buttonKey: buttonKey, onChanged: onChanged, useMaterial3: false);
    await tester.pumpWidget(build());
    final Finder buttonFinder = find.byKey(buttonKey);
    assert(tester.renderObject(buttonFinder).attached);
    await expectLater(
      find.ancestor(of: buttonFinder, matching: find.byType(RepaintBoundary)).first,
      matchesGoldenFile('dropdown_test.default.png'),
    );
  });

  testWidgets('Expanded dropdown golden', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    Widget build() => buildFrame(
      buttonKey: buttonKey,
      isExpanded: true,
      onChanged: onChanged,
      useMaterial3: false,
    );
    await tester.pumpWidget(build());
    final Finder buttonFinder = find.byKey(buttonKey);
    assert(tester.renderObject(buttonFinder).attached);
    await expectLater(
      find.ancestor(of: buttonFinder, matching: find.byType(RepaintBoundary)).first,
      matchesGoldenFile('dropdown_test.expanded.png'),
    );
  });

  testWidgets('Dropdown button control test', (WidgetTester tester) async {
    String? value = 'one';
    void didChangeValue(String? newValue) {
      value = newValue;
    }

    Widget build() => buildFrame(value: value, onChanged: didChangeValue);

    await tester.pumpWidget(build());

    await tester.tap(find.text('one'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('one'));

    await tester.tap(find.text('three').last);

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('three'));

    await tester.tap(find.text('three', skipOffstage: false), warnIfMissed: false);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('three'));

    await tester.pumpWidget(build());

    await tester.tap(find.text('two').last);

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('two'));
  });

  testWidgets('Dropdown button with no app', (WidgetTester tester) async {
    String? value = 'one';
    void didChangeValue(String? newValue) {
      value = newValue;
    }

    Widget build() {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: MediaQueryData.fromView(tester.view),
          child: Navigator(
            initialRoute: '/',
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<void>(
                settings: settings,
                builder: (BuildContext context) {
                  return Material(child: buildFrame(value: 'one', onChanged: didChangeValue));
                },
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(build());

    await tester.tap(find.text('one'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('one'));

    await tester.tap(find.text('three').last);

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('three'));

    await tester.tap(find.text('three', skipOffstage: false), warnIfMissed: false);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('three'));

    await tester.pumpWidget(build());

    await tester.tap(find.text('two').last);

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('two'));
  });

  testWidgets('DropdownButton does not allow duplicate item values', (WidgetTester tester) async {
    final List<DropdownMenuItem<String>> itemsWithDuplicateValues =
        <String>['a', 'b', 'c', 'c'].map<DropdownMenuItem<String>>((String value) {
          return DropdownMenuItem<String>(value: value, child: Text(value));
        }).toList();

    await expectLater(
      () => tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: DropdownButton<String>(
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

  testWidgets('DropdownButton value should only appear in one menu item', (
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

  testWidgets('Dropdown form field uses form field state', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    final GlobalKey<FormState> formKey = GlobalKey<FormState>();
    String? value;
    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Material(
              child: Form(
                key: formKey,
                child: DropdownButtonFormField<String>(
                  key: buttonKey,
                  value: value,
                  hint: const Text('Select Value'),
                  decoration: const InputDecoration(prefixIcon: Icon(Icons.fastfood)),
                  items:
                      menuItems.map((String val) {
                        return DropdownMenuItem<String>(value: val, child: Text(val));
                      }).toList(),
                  validator: (String? v) => v == null ? 'Must select value' : null,
                  onChanged: (String? newValue) {},
                  onSaved: (String? v) {
                    setState(() {
                      value = v;
                    });
                  },
                ),
              ),
            ),
          );
        },
      ),
    );
    int getIndex() {
      final IndexedStack stack = tester.element(find.byType(IndexedStack)).widget as IndexedStack;
      return stack.index!;
    }

    // Initial value of null displays hint
    expect(value, equals(null));
    expect(getIndex(), 4);
    await tester.tap(find.text('Select Value', skipOffstage: false), warnIfMissed: false);
    await tester.pumpAndSettle();
    await tester.tap(find.text('three').last);
    await tester.pumpAndSettle();
    expect(getIndex(), 2);
    // Changes only made to FormField state until form saved
    expect(value, equals(null));
    final FormState form = formKey.currentState!;
    form.save();
    expect(value, equals('three'));
  });

  testWidgets('Dropdown in ListView', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/12053
    // Positions a DropdownButton at the left and right edges of the screen,
    // forcing it to be sized down to the viewport width
    const String value = 'foo';
    final UniqueKey itemKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              DropdownButton<String>(
                value: value,
                items: <DropdownMenuItem<String>>[
                  DropdownMenuItem<String>(key: itemKey, value: value, child: const Text(value)),
                ],
                onChanged: (_) {},
              ),
            ],
          ),
        ),
      ),
    );
    await tester.tap(find.text(value));
    await tester.pump();
    final List<RenderBox> itemBoxes =
        tester.renderObjectList<RenderBox>(find.byKey(itemKey)).toList();
    expect(itemBoxes[0].localToGlobal(Offset.zero).dx, equals(0.0));
    expect(itemBoxes[1].localToGlobal(Offset.zero).dx, equals(16.0));
    expect(itemBoxes[1].size.width, equals(800.0 - 16.0 * 2));
  });

  testWidgets('Dropdown menu can position correctly inside a nested navigator', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/66870
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          appBar: AppBar(),
          body: Column(
            children: <Widget>[
              ConstrainedBox(
                constraints: const BoxConstraints(maxWidth: 500, maxHeight: 200),
                child: Navigator(
                  onGenerateRoute: (RouteSettings s) {
                    return MaterialPageRoute<void>(
                      builder: (BuildContext context) {
                        return Center(
                          child: DropdownButton<int>(
                            value: 1,
                            items: const <DropdownMenuItem<int>>[
                              DropdownMenuItem<int>(value: 1, child: Text('First Item')),
                              DropdownMenuItem<int>(value: 2, child: Text('Second Item')),
                            ],
                            onChanged: (_) {},
                          ),
                        );
                      },
                    );
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
    await tester.tap(find.text('First Item'));
    await tester.pump();
    final RenderBox secondItem =
        tester
            .renderObjectList<RenderBox>(find.text('Second Item', skipOffstage: false))
            .toList()[1];
    expect(secondItem.localToGlobal(Offset.zero).dx, equals(150.0));
    expect(secondItem.localToGlobal(Offset.zero).dy, equals(176.0));
  });

  testWidgets('Dropdown screen edges', (WidgetTester tester) async {
    int? value = 4;
    final List<DropdownMenuItem<int>> items = <DropdownMenuItem<int>>[
      for (int i = 0; i < 20; ++i) DropdownMenuItem<int>(value: i, child: Text('$i')),
    ];

    void handleChanged(int? newValue) {
      value = newValue;
    }

    final DropdownButton<int> button = DropdownButton<int>(
      value: value,
      onChanged: handleChanged,
      items: items,
    );

    await tester.pumpWidget(
      MaterialApp(home: Material(child: Align(alignment: Alignment.topCenter, child: button))),
    );

    await tester.tap(find.text('4'));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    // We should have two copies of item 5, one in the menu and one in the
    // button itself.
    expect(tester.elementList(find.text('5', skipOffstage: false)), hasLength(2));

    expect(value, 4);
    await tester.tap(find.byWidget(button, skipOffstage: false), warnIfMissed: false);
    expect(value, 4);
    // this waits for the route's completer to complete, which calls handleChanged
    await tester.idle();
    expect(value, 4);
  });

  for (final TextDirection textDirection in TextDirection.values) {
    testWidgets('Dropdown button aligns selected menu item ($textDirection)', (
      WidgetTester tester,
    ) async {
      final Key buttonKey = UniqueKey();

      Widget build() => buildFrame(
        buttonKey: buttonKey,
        textDirection: textDirection,
        onChanged: onChanged,
        useMaterial3: false,
      );

      await tester.pumpWidget(build());
      final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
      assert(buttonBox.attached);
      final Offset buttonOriginBeforeTap = buttonBox.localToGlobal(Offset.zero);

      await tester.tap(find.text('two'));
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the menu animation

      // Tapping the dropdown button should not cause it to move.
      expect(buttonBox.localToGlobal(Offset.zero), equals(buttonOriginBeforeTap));

      // The selected dropdown item is both in menu we just popped up, and in
      // the IndexedStack contained by the dropdown button. Both of them should
      // have the same origin and height as the dropdown button.
      final List<RenderBox> itemBoxes =
          tester.renderObjectList<RenderBox>(find.byKey(const ValueKey<String>('two'))).toList();
      expect(itemBoxes.length, equals(2));
      for (final RenderBox itemBox in itemBoxes) {
        assert(itemBox.attached);
        switch (textDirection) {
          case TextDirection.rtl:
            expect(
              buttonBox.localToGlobal(buttonBox.size.bottomRight(Offset.zero)),
              equals(itemBox.localToGlobal(itemBox.size.bottomRight(Offset.zero))),
            );
          case TextDirection.ltr:
            expect(
              buttonBox.localToGlobal(Offset.zero),
              equals(itemBox.localToGlobal(Offset.zero)),
            );
        }
        expect(buttonBox.size.height, equals(itemBox.size.height));
      }

      // The two RenderParagraph objects, for the 'two' items' Text children,
      // should have the same size and location.
      checkSelectedItemTextGeometry(tester, 'two');

      await tester.pumpWidget(Container()); // reset test
    });
  }

  testWidgets('Arrow icon aligns with the edge of button when expanded', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    Widget build() => buildFrame(buttonKey: buttonKey, isExpanded: true, onChanged: onChanged);

    await tester.pumpWidget(build());
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBox.attached);

    final RenderBox arrowIcon = tester.renderObject<RenderBox>(find.byIcon(Icons.arrow_drop_down));
    assert(arrowIcon.attached);

    // Arrow icon should be aligned with far right of button when expanded
    expect(
      arrowIcon.localToGlobal(Offset.zero).dx,
      buttonBox.size.centerRight(Offset(-arrowIcon.size.width, 0.0)).dx,
    );
  });

  testWidgets('Dropdown button icon will accept widgets as icons', (WidgetTester tester) async {
    final Widget customWidget = Container(
      decoration: ShapeDecoration(
        shape: CircleBorder(side: BorderSide(width: 5.0, color: Colors.grey.shade700)),
      ),
    );

    await tester.pumpWidget(buildFrame(icon: customWidget, onChanged: onChanged));

    expect(find.byWidget(customWidget), findsOneWidget);
    expect(find.byIcon(Icons.arrow_drop_down), findsNothing);

    await tester.pumpWidget(buildFrame(icon: const Icon(Icons.assessment), onChanged: onChanged));

    expect(find.byIcon(Icons.assessment), findsOneWidget);
    expect(find.byIcon(Icons.arrow_drop_down), findsNothing);
  });

  testWidgets('Dropdown button icon should have default size and colors when not defined', (
    WidgetTester tester,
  ) async {
    final Key iconKey = UniqueKey();
    final Icon customIcon = Icon(Icons.assessment, key: iconKey);

    await tester.pumpWidget(buildFrame(icon: customIcon, onChanged: onChanged));

    // test for size
    final RenderBox icon = tester.renderObject(find.byKey(iconKey));
    expect(icon.size, const Size(24.0, 24.0));

    // test for enabled color
    final RichText enabledRichText = tester.widget<RichText>(_iconRichText(iconKey));
    expect(enabledRichText.text.style!.color, Colors.grey.shade700);

    // test for disabled color
    await tester.pumpWidget(buildFrame(icon: customIcon));

    final RichText disabledRichText = tester.widget<RichText>(_iconRichText(iconKey));
    expect(disabledRichText.text.style!.color, Colors.grey.shade400);
  });

  testWidgets('Dropdown button icon should have the passed in size and color instead of defaults', (
    WidgetTester tester,
  ) async {
    final Key iconKey = UniqueKey();
    final Icon customIcon = Icon(Icons.assessment, key: iconKey);

    await tester.pumpWidget(
      buildFrame(
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
      buildFrame(
        icon: customIcon,
        iconSize: 30.0,
        iconEnabledColor: Colors.pink,
        iconDisabledColor: Colors.orange,
      ),
    );

    final RichText disabledRichText = tester.widget<RichText>(_iconRichText(iconKey));
    expect(disabledRichText.text.style!.color, Colors.orange);
  });

  testWidgets(
    'Dropdown button should use its own size and color properties over those defined by the theme',
    (WidgetTester tester) async {
      final Key iconKey = UniqueKey();

      final Icon customIcon = Icon(
        Icons.assessment,
        key: iconKey,
        size: 40.0,
        color: Colors.yellow,
      );

      await tester.pumpWidget(
        buildFrame(
          icon: customIcon,
          iconSize: 30.0,
          iconEnabledColor: Colors.pink,
          iconDisabledColor: Colors.orange,
          onChanged: onChanged,
        ),
      );

      // test for size
      final RenderBox icon = tester.renderObject(find.byKey(iconKey));
      expect(icon.size, const Size(40.0, 40.0));

      // test for enabled color
      final RichText enabledRichText = tester.widget<RichText>(_iconRichText(iconKey));
      expect(enabledRichText.text.style!.color, Colors.yellow);

      // test for disabled color
      await tester.pumpWidget(
        buildFrame(
          icon: customIcon,
          iconSize: 30.0,
          iconEnabledColor: Colors.pink,
          iconDisabledColor: Colors.orange,
        ),
      );

      final RichText disabledRichText = tester.widget<RichText>(_iconRichText(iconKey));
      expect(disabledRichText.text.style!.color, Colors.yellow);
    },
  );

  testWidgets('Dropdown button with isDense:true aligns selected menu item', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();

    Widget build() => buildFrame(buttonKey: buttonKey, isDense: true, onChanged: onChanged);

    await tester.pumpWidget(build());
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBox.attached);

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
    final double menuItemHeight = itemBoxes
        .map<double>((RenderBox box) => box.size.height)
        .reduce(math.max);
    expect(menuItemHeight, greaterThan(buttonBox.size.height));

    for (final RenderBox itemBox in itemBoxes) {
      assert(itemBox.attached);
      final Offset buttonBoxCenter = buttonBox.size.center(buttonBox.localToGlobal(Offset.zero));
      final Offset itemBoxCenter = itemBox.size.center(itemBox.localToGlobal(Offset.zero));
      expect(buttonBoxCenter.dy, equals(itemBoxCenter.dy));
    }

    // The two RenderParagraph objects, for the 'two' items' Text children,
    // should have the same size and location.
    checkSelectedItemTextGeometry(tester, 'two');
  });

  testWidgets('Dropdown button can have a text style with no fontSize specified', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/33425
    const String value = 'foo';
    final UniqueKey itemKey = UniqueKey();

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: DropdownButton<String>(
            value: value,
            items: <DropdownMenuItem<String>>[
              DropdownMenuItem<String>(key: itemKey, value: 'foo', child: const Text(value)),
            ],
            isDense: true,
            onChanged: (_) {},
            style: const TextStyle(color: Colors.blue),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('Dropdown menu scrolls to first item in long lists', (WidgetTester tester) async {
    // Open the dropdown menu
    final Key buttonKey = UniqueKey();
    await tester.pumpWidget(
      buildFrame(
        buttonKey: buttonKey,
        value: null, // nothing selected
        items: List<String>.generate(/*length=*/ 100, (int index) => index.toString()),
        onChanged: onChanged,
      ),
    );
    await tester.tap(find.byKey(buttonKey));
    await tester.pump();
    await tester.pumpAndSettle(); // finish the menu animation

    // Find the first item in the scrollable dropdown list
    final Finder menuItemFinder = find.byType(Scrollable);
    final RenderBox menuItemContainer = tester.renderObject<RenderBox>(menuItemFinder);
    final RenderBox firstItem = tester.renderObject<RenderBox>(
      find.descendant(of: menuItemFinder, matching: find.byKey(const ValueKey<String>('0'))),
    );

    // List should be scrolled so that the first item is at the top. Menu items
    // are offset 8.0 from the top edge of the scrollable menu.
    const Offset selectedItemOffset = Offset(0.0, -8.0);
    expect(
      firstItem.size.topCenter(firstItem.localToGlobal(selectedItemOffset)).dy,
      equals(menuItemContainer.size.topCenter(menuItemContainer.localToGlobal(Offset.zero)).dy),
    );
  });

  testWidgets('Dropdown menu aligns selected item with button in long lists', (
    WidgetTester tester,
  ) async {
    // Open the dropdown menu
    final Key buttonKey = UniqueKey();
    await tester.pumpWidget(
      buildFrame(
        buttonKey: buttonKey,
        value: '50',
        items: List<String>.generate(/*length=*/ 100, (int index) => index.toString()),
        onChanged: onChanged,
      ),
    );
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    await tester.tap(find.byKey(buttonKey));
    await tester.pumpAndSettle(); // finish the menu animation

    // Find the selected item in the scrollable dropdown list
    final RenderBox selectedItem = tester.renderObject<RenderBox>(
      find.descendant(
        of: find.byType(Scrollable),
        matching: find.byKey(const ValueKey<String>('50')),
      ),
    );

    // List should be scrolled so that the selected item is in line with the button
    expect(
      selectedItem.size.center(selectedItem.localToGlobal(Offset.zero)).dy,
      equals(buttonBox.size.center(buttonBox.localToGlobal(Offset.zero)).dy),
    );
  });

  testWidgets('Dropdown menu scrolls to last item in long lists', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    await tester.pumpWidget(
      buildFrame(
        buttonKey: buttonKey,
        value: '99',
        items: List<String>.generate(/*length=*/ 100, (int index) => index.toString()),
        onChanged: onChanged,
      ),
    );
    await tester.tap(find.byKey(buttonKey));
    await tester.pump();

    final ScrollController scrollController = PrimaryScrollController.of(
      tester.element(find.byType(ListView)),
    );
    // Make sure there is no overscroll
    expect(scrollController.offset, scrollController.position.maxScrollExtent);

    // Find the selected item in the scrollable dropdown list
    final Finder menuItemFinder = find.byType(Scrollable);
    final RenderBox menuItemContainer = tester.renderObject<RenderBox>(menuItemFinder);
    final RenderBox selectedItem = tester.renderObject<RenderBox>(
      find.descendant(of: menuItemFinder, matching: find.byKey(const ValueKey<String>('99'))),
    );

    // kMaterialListPadding.vertical is 8.
    const Offset menuPaddingOffset = Offset(0.0, -8.0);
    final Offset selectedItemOffset = selectedItem.localToGlobal(Offset.zero);
    final Offset menuItemContainerOffset = menuItemContainer.localToGlobal(menuPaddingOffset);
    // Selected item should be aligned to the bottom of the dropdown menu.
    expect(
      selectedItem.size.bottomCenter(selectedItemOffset).dy,
      menuItemContainer.size.bottomCenter(menuItemContainerOffset).dy,
    );
  });

  testWidgets('Size of DropdownButton with null value', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    String? value;

    Widget build() => buildFrame(buttonKey: buttonKey, value: value, onChanged: onChanged);

    await tester.pumpWidget(build());
    final RenderBox buttonBoxNullValue = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBoxNullValue.attached);

    value = 'three';
    await tester.pumpWidget(build());
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBox.attached);

    // A Dropdown button with a null value should be the same size as a
    // one with a non-null value.
    expect(
      buttonBox.localToGlobal(Offset.zero),
      equals(buttonBoxNullValue.localToGlobal(Offset.zero)),
    );
    expect(buttonBox.size, equals(buttonBoxNullValue.size));
  });

  testWidgets('Size of DropdownButton with no items', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/26419
    final Key buttonKey = UniqueKey();
    List<String>? items;

    Widget build() => buildFrame(buttonKey: buttonKey, items: items, onChanged: onChanged);

    await tester.pumpWidget(build());
    final RenderBox buttonBoxNullItems = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBoxNullItems.attached);

    items = <String>[];
    await tester.pumpWidget(build());
    final RenderBox buttonBoxEmptyItems = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBoxEmptyItems.attached);

    items = <String>['one', 'two', 'three', 'four'];
    await tester.pumpWidget(build());
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBox.attached);

    // A Dropdown button with a null value should be the same size as a
    // one with a non-null value.
    expect(
      buttonBox.localToGlobal(Offset.zero),
      equals(buttonBoxNullItems.localToGlobal(Offset.zero)),
    );
    expect(buttonBox.size, equals(buttonBoxNullItems.size));
  });

  testWidgets('Layout of a DropdownButton with null value', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    String? value;

    void onChanged(String? newValue) {
      value = newValue;
    }

    Widget build() => buildFrame(buttonKey: buttonKey, value: value, onChanged: onChanged);

    await tester.pumpWidget(build());
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBox.attached);

    // Show the menu.
    await tester.tap(find.byKey(buttonKey));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    // Tap on item 'one', which must appear over the button.
    await tester.tap(find.byKey(buttonKey, skipOffstage: false), warnIfMissed: false);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    await tester.pumpWidget(build());
    expect(value, equals('one'));
  });

  testWidgets('Size of DropdownButton with null value and a hint', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    String? value;

    // The hint will define the dropdown's width
    Widget build() =>
        buildFrame(buttonKey: buttonKey, value: value, hint: const Text('onetwothree'));

    await tester.pumpWidget(build());
    expect(find.text('onetwothree'), findsOneWidget);
    final RenderBox buttonBoxHintValue = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBoxHintValue.attached);

    value = 'three';
    await tester.pumpWidget(build());
    final RenderBox buttonBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBox.attached);

    // A Dropdown button with a null value and a hint should be the same size as a
    // one with a non-null value.
    expect(
      buttonBox.localToGlobal(Offset.zero),
      equals(buttonBoxHintValue.localToGlobal(Offset.zero)),
    );
    expect(buttonBox.size, equals(buttonBoxHintValue.size));
  });

  testWidgets('Dropdown menus must fit within the screen', (WidgetTester tester) async {
    // In all of the tests that follow we're assuming that the dropdown menu
    // is horizontally aligned with the center of the dropdown button and padded
    // on the top, left, and right.
    const EdgeInsets buttonPadding = EdgeInsets.only(top: 8.0, left: 16.0, right: 24.0);

    Rect getExpandedButtonRect() {
      final RenderBox box = tester.renderObject<RenderBox>(find.byType(dropdownButtonType));
      final Rect buttonRect = box.localToGlobal(Offset.zero) & box.size;
      return buttonPadding.inflateRect(buttonRect);
    }

    late Rect buttonRect;
    late Rect menuRect;

    Future<void> popUpAndDown(Widget frame) async {
      await tester.pumpWidget(frame);
      await tester.tap(find.byType(dropdownButtonType));
      await tester.pumpAndSettle();
      menuRect = getMenuRect(tester);
      buttonRect = getExpandedButtonRect();
      await tester.tap(find.byType(dropdownButtonType, skipOffstage: false), warnIfMissed: false);
    }

    // Dropdown button is along the top of the app. The top of the menu is
    // aligned with the top of the expanded button and shifted horizontally
    // so that it fits within the frame.

    await popUpAndDown(
      buildFrame(dropdownAlignment: Alignment.topLeft, value: menuItems.last, onChanged: onChanged),
    );
    expect(menuRect.topLeft, Offset.zero);
    expect(menuRect.topRight, Offset(menuRect.width, 0.0));

    await popUpAndDown(
      buildFrame(
        dropdownAlignment: Alignment.topCenter,
        value: menuItems.last,
        onChanged: onChanged,
      ),
    );
    expect(menuRect.topLeft, Offset(buttonRect.left, 0.0));
    expect(menuRect.topRight, Offset(buttonRect.right, 0.0));

    await popUpAndDown(
      buildFrame(
        dropdownAlignment: Alignment.topRight,
        value: menuItems.last,
        onChanged: onChanged,
      ),
    );
    expect(menuRect.topLeft, Offset(800.0 - menuRect.width, 0.0));
    expect(menuRect.topRight, const Offset(800.0, 0.0));

    // Dropdown button is along the middle of the app. The top of the menu is
    // aligned with the top of the expanded button (because the 1st item
    // is selected) and shifted horizontally so that it fits within the frame.

    await popUpAndDown(
      buildFrame(
        dropdownAlignment: Alignment.centerLeft,
        value: menuItems.first,
        onChanged: onChanged,
      ),
    );
    expect(menuRect.topLeft, Offset(0.0, buttonRect.top));
    expect(menuRect.topRight, Offset(menuRect.width, buttonRect.top));

    await popUpAndDown(buildFrame(value: menuItems.first, onChanged: onChanged));
    expect(menuRect.topLeft, buttonRect.topLeft);
    expect(menuRect.topRight, buttonRect.topRight);

    await popUpAndDown(
      buildFrame(
        dropdownAlignment: Alignment.centerRight,
        value: menuItems.first,
        onChanged: onChanged,
      ),
    );
    expect(menuRect.topLeft, Offset(800.0 - menuRect.width, buttonRect.top));
    expect(menuRect.topRight, Offset(800.0, buttonRect.top));

    // Dropdown button is along the bottom of the app. The bottom of the menu is
    // aligned with the bottom of the expanded button and shifted horizontally
    // so that it fits within the frame.

    await popUpAndDown(
      buildFrame(
        dropdownAlignment: Alignment.bottomLeft,
        value: menuItems.first,
        onChanged: onChanged,
      ),
    );
    expect(menuRect.bottomLeft, const Offset(0.0, 600.0));
    expect(menuRect.bottomRight, Offset(menuRect.width, 600.0));

    await popUpAndDown(
      buildFrame(
        dropdownAlignment: Alignment.bottomCenter,
        value: menuItems.first,
        onChanged: onChanged,
      ),
    );
    expect(menuRect.bottomLeft, Offset(buttonRect.left, 600.0));
    expect(menuRect.bottomRight, Offset(buttonRect.right, 600.0));

    await popUpAndDown(
      buildFrame(
        dropdownAlignment: Alignment.bottomRight,
        value: menuItems.first,
        onChanged: onChanged,
      ),
    );
    expect(menuRect.bottomLeft, Offset(800.0 - menuRect.width, 600.0));
    expect(menuRect.bottomRight, const Offset(800.0, 600.0));
  });

  testWidgets(
    'Dropdown menus are dismissed on screen orientation changes, but not on keyboard hide',
    (WidgetTester tester) async {
      await tester.pumpWidget(buildFrame(onChanged: onChanged, mediaSize: const Size(800, 600)));
      await tester.tap(find.byType(dropdownButtonType));
      await tester.pumpAndSettle();
      expect(find.byType(ListView), findsOneWidget);

      // Show a keyboard (simulate by shortening the height).
      await tester.pumpWidget(buildFrame(onChanged: onChanged, mediaSize: const Size(800, 300)));
      await tester.pump();
      expect(find.byType(ListView, skipOffstage: false), findsOneWidget);

      // Hide a keyboard again (simulate by increasing the height).
      await tester.pumpWidget(buildFrame(onChanged: onChanged, mediaSize: const Size(800, 600)));
      await tester.pump();
      expect(find.byType(ListView, skipOffstage: false), findsOneWidget);

      // Rotate the device (simulate by changing the aspect ratio).
      await tester.pumpWidget(buildFrame(onChanged: onChanged, mediaSize: const Size(600, 800)));
      await tester.pump();
      expect(find.byType(ListView, skipOffstage: false), findsNothing);
    },
  );

  testWidgets('Semantics Tree contains only selected element', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(buildFrame(onChanged: onChanged));

    expect(semantics, isNot(includesNodeWith(label: menuItems[0])));
    expect(semantics, includesNodeWith(label: menuItems[1]));
    expect(semantics, isNot(includesNodeWith(label: menuItems[2])));
    expect(semantics, isNot(includesNodeWith(label: menuItems[3])));

    semantics.dispose();
  });

  testWidgets('Dropdown button includes semantics', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    const Key key = Key('test');
    await tester.pumpWidget(
      buildFrame(buttonKey: key, value: null, onChanged: (String? _) {}, hint: const Text('test')),
    );

    // By default the hint contributes the label.
    expect(
      tester.getSemantics(find.text('test')),
      matchesSemantics(
        isButton: true,
        label: 'test',
        hasTapAction: true,
        hasFocusAction: true,
        isFocusable: true,
      ),
    );

    await tester.pumpWidget(
      buildFrame(buttonKey: key, value: 'three', onChanged: onChanged, hint: const Text('test')),
    );

    // Displays label of select item.
    expect(
      tester.getSemantics(find.text('three')),
      matchesSemantics(
        isButton: true,
        label: 'three',
        hasTapAction: true,
        hasFocusAction: true,
        isFocusable: true,
      ),
    );
    handle.dispose();
  });

  testWidgets('Dropdown menu includes semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const Key key = Key('test');
    await tester.pumpWidget(buildFrame(buttonKey: key, value: null, onChanged: onChanged));
    await tester.tap(find.byKey(key));
    await tester.pumpAndSettle();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
              label: 'Dismiss',
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                  label: 'Popup menu',
                  children: <TestSemantics>[
                    TestSemantics(
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          children: <TestSemantics>[
                            TestSemantics(
                              label: 'one',
                              textDirection: TextDirection.ltr,
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.isFocused,
                                SemanticsFlag.isFocusable,
                              ],
                              tags: <SemanticsTag>[const SemanticsTag('RenderViewport.twoPane')],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                            ),
                            TestSemantics(
                              label: 'two',
                              textDirection: TextDirection.ltr,
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.isFocusable,
                              ],
                              tags: <SemanticsTag>[const SemanticsTag('RenderViewport.twoPane')],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                            ),
                            TestSemantics(
                              label: 'three',
                              textDirection: TextDirection.ltr,
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.isFocusable,
                              ],
                              tags: <SemanticsTag>[const SemanticsTag('RenderViewport.twoPane')],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                            ),
                            TestSemantics(
                              label: 'four',
                              textDirection: TextDirection.ltr,
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.isFocusable,
                              ],
                              tags: <SemanticsTag>[const SemanticsTag('RenderViewport.twoPane')],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
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

  testWidgets('disabledHint displays on empty items or onChanged', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items, ValueChanged<String?>? onChanged}) => buildFrame(
      items: items,
      onChanged: onChanged,
      buttonKey: buttonKey,
      value: null,
      hint: const Text('enabled'),
      disabledHint: const Text('disabled'),
    );

    // [disabledHint] should display when [items] is null
    await tester.pumpWidget(build(onChanged: onChanged));
    expect(find.text('enabled'), findsNothing);
    expect(find.text('disabled'), findsOneWidget);

    // [disabledHint] should display when [items] is an empty list.
    await tester.pumpWidget(build(items: <String>[], onChanged: onChanged));
    expect(find.text('enabled'), findsNothing);
    expect(find.text('disabled'), findsOneWidget);

    // [disabledHint] should display when [onChanged] is null
    await tester.pumpWidget(build(items: menuItems));
    expect(find.text('enabled'), findsNothing);
    expect(find.text('disabled'), findsOneWidget);
    final RenderBox disabledHintBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));

    // A Dropdown button with a disabled hint should be the same size as a
    // one with a regular enabled hint.
    await tester.pumpWidget(build(items: menuItems, onChanged: onChanged));
    expect(find.text('disabled'), findsNothing);
    expect(find.text('enabled'), findsOneWidget);
    final RenderBox enabledHintBox = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    expect(
      enabledHintBox.localToGlobal(Offset.zero),
      equals(disabledHintBox.localToGlobal(Offset.zero)),
    );
    expect(enabledHintBox.size, equals(disabledHintBox.size));
  });

  // Regression test for https://github.com/flutter/flutter/issues/70177
  testWidgets('disabledHint behavior test', (WidgetTester tester) async {
    Widget build({
      List<String>? items,
      ValueChanged<String?>? onChanged,
      String? value,
      Widget? hint,
      Widget? disabledHint,
    }) => buildFrame(
      items: items,
      onChanged: onChanged,
      value: value,
      hint: hint,
      disabledHint: disabledHint,
    );

    // The selected value should be displayed when the button is disabled.
    await tester.pumpWidget(build(items: menuItems, value: 'two'));
    // The dropdown icon and the selected menu item are vertically aligned.
    expect(tester.getCenter(find.text('two')).dy, tester.getCenter(find.byType(Icon)).dy);

    // If [value] is null, the button is enabled, hint is displayed.
    await tester.pumpWidget(
      build(
        items: menuItems,
        onChanged: onChanged,
        hint: const Text('hint'),
        disabledHint: const Text('disabledHint'),
      ),
    );
    expect(tester.getCenter(find.text('hint')).dy, tester.getCenter(find.byType(Icon)).dy);

    // If [value] is null, the button is disabled, [disabledHint] is displayed when [disabledHint] is non-null.
    await tester.pumpWidget(
      build(items: menuItems, hint: const Text('hint'), disabledHint: const Text('disabledHint')),
    );
    expect(tester.getCenter(find.text('disabledHint')).dy, tester.getCenter(find.byType(Icon)).dy);

    // If [value] is null, the button is disabled, [hint] is displayed when [disabledHint] is null.
    await tester.pumpWidget(build(items: menuItems, hint: const Text('hint')));
    expect(tester.getCenter(find.text('hint')).dy, tester.getCenter(find.byType(Icon)).dy);

    int? getIndex() {
      final IndexedStack stack = tester.element(find.byType(IndexedStack)).widget as IndexedStack;
      return stack.index;
    }

    // If [value], [hint] and [disabledHint] are null, the button is disabled, nothing displayed.
    await tester.pumpWidget(build(items: menuItems));
    expect(getIndex(), null);

    // If [value], [hint] and [disabledHint] are null, the button is enabled, nothing displayed.
    await tester.pumpWidget(build(items: menuItems, onChanged: onChanged));
    expect(getIndex(), null);
  });

  testWidgets('DropdownButton selected item color test', (WidgetTester tester) async {
    Widget build({
      ValueChanged<String?>? onChanged,
      String? value,
      Widget? hint,
      Widget? disabledHint,
    }) {
      return MaterialApp(
        theme: ThemeData(disabledColor: Colors.pink),
        home: Scaffold(
          body: Center(
            child: Column(
              children: <Widget>[
                DropdownButtonFormField<String>(
                  style: const TextStyle(color: Colors.yellow),
                  disabledHint: disabledHint,
                  hint: hint,
                  items: const <DropdownMenuItem<String>>[
                    DropdownMenuItem<String>(value: 'one', child: Text('one')),
                    DropdownMenuItem<String>(value: 'two', child: Text('two')),
                  ],
                  value: value,
                  onChanged: onChanged,
                ),
              ],
            ),
          ),
        ),
      );
    }

    Color textColor(String text) {
      return tester.renderObject<RenderParagraph>(find.text(text)).text.style!.color!;
    }

    // The selected value should be displayed when the button is enabled.
    await tester.pumpWidget(build(onChanged: onChanged, value: 'two'));
    // The dropdown icon and the selected menu item are vertically aligned.
    expect(tester.getCenter(find.text('two')).dy, tester.getCenter(find.byType(Icon)).dy);
    // Selected item has a normal color from [DropdownButtonFormField.style]
    // when the button is enabled.
    expect(textColor('two'), Colors.yellow);

    // The selected value should be displayed when the button is disabled.
    await tester.pumpWidget(build(value: 'two'));
    expect(tester.getCenter(find.text('two')).dy, tester.getCenter(find.byType(Icon)).dy);
    // Selected item has a disabled color from [theme.disabledColor]
    // when the button is disable.
    expect(textColor('two'), Colors.pink);
  });

  testWidgets('DropdownButton hint displays when the items list is empty, '
      'items is null, and disabledHint is null', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items}) {
      return buildFrame(
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

  testWidgets('DropdownButton disabledHint is null by default', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();

    Widget build({List<String>? items}) {
      return buildFrame(
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

  testWidgets(
    'Size of largest widget is used DropdownButton when selectedItemBuilder is non-null',
    (WidgetTester tester) async {
      final List<String> items = <String>['25', '50', '100'];
      const String selectedItem = '25';

      await tester.pumpWidget(
        buildFrame(
          // To test the size constraints, the selected item should not be the
          // largest item. This validates that the button sizes itself according
          // to the largest item regardless of which one is selected.
          value: selectedItem,
          items: items,
          itemHeight: null,
          selectedItemBuilder: (BuildContext context) {
            return items.map<Widget>((String item) {
              return SizedBox(
                height: double.parse(item),
                width: double.parse(item),
                child: Center(child: Text(item)),
              );
            }).toList();
          },
          onChanged: (String? newValue) {},
        ),
      );

      final RenderBox dropdownButtonRenderBox = tester.renderObject<RenderBox>(
        find.widgetWithText(Row, '25'),
      );
      // DropdownButton should be the height of the largest item
      expect(dropdownButtonRenderBox.size.height, 100);
      // DropdownButton should be width of largest item added to the icon size
      expect(dropdownButtonRenderBox.size.width, 100 + 24.0);
    },
  );

  testWidgets(
    'Enabled button - Size of largest widget is used DropdownButton when selectedItemBuilder '
    'is non-null and hint is defined, but smaller than largest selected item widget',
    (WidgetTester tester) async {
      final List<String> items = <String>['25', '50', '100'];

      await tester.pumpWidget(
        buildFrame(
          value: null,
          // [hint] widget is smaller than largest selected item widget
          hint: const SizedBox(height: 50, width: 50, child: Text('hint')),
          items: items,
          itemHeight: null,
          selectedItemBuilder: (BuildContext context) {
            return items.map<Widget>((String item) {
              return SizedBox(
                height: double.parse(item),
                width: double.parse(item),
                child: Center(child: Text(item)),
              );
            }).toList();
          },
          onChanged: (String? newValue) {},
        ),
      );

      final RenderBox dropdownButtonRenderBox = tester.renderObject<RenderBox>(
        find.widgetWithText(Row, 'hint'),
      );
      // DropdownButton should be the height of the largest item
      expect(dropdownButtonRenderBox.size.height, 100);
      // DropdownButton should be width of largest item added to the icon size
      expect(dropdownButtonRenderBox.size.width, 100 + 24.0);
    },
  );

  testWidgets(
    'Enabled button - Size of largest widget is used DropdownButton when selectedItemBuilder '
    'is non-null and hint is defined, but larger than largest selected item widget',
    (WidgetTester tester) async {
      final List<String> items = <String>['25', '50', '100'];
      const String selectedItem = '25';

      await tester.pumpWidget(
        buildFrame(
          // To test the size constraints, the selected item should not be the
          // largest item. This validates that the button sizes itself according
          // to the largest item regardless of which one is selected.
          value: selectedItem,
          // [hint] widget is larger than largest selected item widget
          hint: const SizedBox(height: 125, width: 125, child: Text('hint')),
          items: items,
          itemHeight: null,
          selectedItemBuilder: (BuildContext context) {
            return items.map<Widget>((String item) {
              return SizedBox(
                height: double.parse(item),
                width: double.parse(item),
                child: Center(child: Text(item)),
              );
            }).toList();
          },
          onChanged: (String? newValue) {},
        ),
      );

      final RenderBox dropdownButtonRenderBox = tester.renderObject<RenderBox>(
        find.widgetWithText(Row, '25'),
      );
      // DropdownButton should be the height of the largest item (hint inclusive)
      expect(dropdownButtonRenderBox.size.height, 125);
      // DropdownButton should be width of largest item (hint inclusive) added to the icon size
      expect(dropdownButtonRenderBox.size.width, 125 + 24.0);
    },
  );

  testWidgets(
    'Disabled button - Size of largest widget is used DropdownButton when selectedItemBuilder '
    'is non-null, and hint is defined, but smaller than largest selected item widget',
    (WidgetTester tester) async {
      final List<String> items = <String>['25', '50', '100'];

      await tester.pumpWidget(
        buildFrame(
          value: null,
          // [hint] widget is smaller than largest selected item widget
          hint: const SizedBox(height: 50, width: 50, child: Text('hint')),
          items: items,
          itemHeight: null,
          selectedItemBuilder: (BuildContext context) {
            return items.map<Widget>((String item) {
              return SizedBox(
                height: double.parse(item),
                width: double.parse(item),
                child: Center(child: Text(item)),
              );
            }).toList();
          },
        ),
      );

      final RenderBox dropdownButtonRenderBox = tester.renderObject<RenderBox>(
        find.widgetWithText(Row, 'hint'),
      );
      // DropdownButton should be the height of the largest item
      expect(dropdownButtonRenderBox.size.height, 100);
      // DropdownButton should be width of largest item added to the icon size
      expect(dropdownButtonRenderBox.size.width, 100 + 24.0);
    },
  );

  testWidgets(
    'Disabled button - Size of largest widget is used DropdownButton when selectedItemBuilder '
    'is non-null and hint is defined, but larger than largest selected item widget',
    (WidgetTester tester) async {
      final List<String> items = <String>['25', '50', '100'];

      await tester.pumpWidget(
        buildFrame(
          value: null,
          // [hint] widget is larger than largest selected item widget
          hint: const SizedBox(height: 125, width: 125, child: Text('hint')),
          items: items,
          itemHeight: null,
          selectedItemBuilder: (BuildContext context) {
            return items.map<Widget>((String item) {
              return SizedBox(
                height: double.parse(item),
                width: double.parse(item),
                child: Center(child: Text(item)),
              );
            }).toList();
          },
        ),
      );

      final RenderBox dropdownButtonRenderBox = tester.renderObject<RenderBox>(
        find.widgetWithText(Row, '25', skipOffstage: false),
      );
      // DropdownButton should be the height of the largest item (hint inclusive)
      expect(dropdownButtonRenderBox.size.height, 125);
      // DropdownButton should be width of largest item (hint inclusive) added to the icon size
      expect(dropdownButtonRenderBox.size.width, 125 + 24.0);
    },
  );

  testWidgets(
    'Disabled button - Size of largest widget is used DropdownButton when selectedItemBuilder '
    'is non-null, and disabledHint is defined, but smaller than largest selected item widget',
    (WidgetTester tester) async {
      final List<String> items = <String>['25', '50', '100'];

      await tester.pumpWidget(
        buildFrame(
          value: null,
          // [hint] widget is smaller than largest selected item widget
          disabledHint: const SizedBox(height: 50, width: 50, child: Text('hint')),
          items: items,
          itemHeight: null,
          selectedItemBuilder: (BuildContext context) {
            return items.map<Widget>((String item) {
              return SizedBox(
                height: double.parse(item),
                width: double.parse(item),
                child: Center(child: Text(item)),
              );
            }).toList();
          },
        ),
      );

      final RenderBox dropdownButtonRenderBox = tester.renderObject<RenderBox>(
        find.widgetWithText(Row, 'hint'),
      );
      // DropdownButton should be the height of the largest item
      expect(dropdownButtonRenderBox.size.height, 100);
      // DropdownButton should be width of largest item added to the icon size
      expect(dropdownButtonRenderBox.size.width, 100 + 24.0);
    },
  );

  testWidgets(
    'Disabled button - Size of largest widget is used DropdownButton when selectedItemBuilder '
    'is non-null and disabledHint is defined, but larger than largest selected item widget',
    (WidgetTester tester) async {
      final List<String> items = <String>['25', '50', '100'];

      await tester.pumpWidget(
        buildFrame(
          value: null,
          // [hint] widget is larger than largest selected item widget
          disabledHint: const SizedBox(height: 125, width: 125, child: Text('hint')),
          items: items,
          itemHeight: null,
          selectedItemBuilder: (BuildContext context) {
            return items.map<Widget>((String item) {
              return SizedBox(
                height: double.parse(item),
                width: double.parse(item),
                child: Center(child: Text(item)),
              );
            }).toList();
          },
        ),
      );

      final RenderBox dropdownButtonRenderBox = tester.renderObject<RenderBox>(
        find.widgetWithText(Row, '25', skipOffstage: false),
      );
      // DropdownButton should be the height of the largest item (hint inclusive)
      expect(dropdownButtonRenderBox.size.height, 125);
      // DropdownButton should be width of largest item (hint inclusive) added to the icon size
      expect(dropdownButtonRenderBox.size.width, 125 + 24.0);
    },
  );

  testWidgets('Menu width is correct when set', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/133267.
    final List<String> items = <String>['25', '50', '100'];
    const String selectedItem = '25';

    await tester.pumpWidget(
      buildFrame(
        value: selectedItem,
        items: items,
        menuWidth: 200,
        selectedItemBuilder: (BuildContext context) {
          return items.map<Widget>((String item) {
            return SizedBox(
              height: double.parse(item),
              width: double.parse(item),
              child: Center(child: Text(item)),
            );
          }).toList();
        },
        onChanged: (String? newValue) {},
      ),
    );

    await tester.tap(find.text('25'));
    await tester.pumpAndSettle();

    expect(getMenuRect(tester).width, 200);
  });

  testWidgets('Dropdown in middle showing middle item', (WidgetTester tester) async {
    final List<DropdownMenuItem<int>> items = List<DropdownMenuItem<int>>.generate(
      100,
      (int i) => DropdownMenuItem<int>(value: i, child: Text('$i')),
    );

    final DropdownButton<int> button = DropdownButton<int>(
      value: 50,
      onChanged: (int? newValue) {},
      items: items,
    );

    double getMenuScroll() {
      double scrollPosition;
      final ScrollController scrollController = PrimaryScrollController.of(
        tester.element(find.byType(ListView)),
      );
      scrollPosition = scrollController.position.pixels;
      return scrollPosition;
    }

    await tester.pumpWidget(MaterialApp(home: Material(child: Align(child: button))));

    await tester.tap(find.text('50'));
    await tester.pumpAndSettle();
    expect(getMenuScroll(), 2180.0);
  });

  testWidgets('Dropdown in top showing bottom item', (WidgetTester tester) async {
    final List<DropdownMenuItem<int>> items = List<DropdownMenuItem<int>>.generate(
      100,
      (int i) => DropdownMenuItem<int>(value: i, child: Text('$i')),
    );

    final DropdownButton<int> button = DropdownButton<int>(
      value: 99,
      onChanged: (int? newValue) {},
      items: items,
    );

    double getMenuScroll() {
      double scrollPosition;
      final ScrollController scrollController = PrimaryScrollController.of(
        tester.element(find.byType(ListView)),
      );
      scrollPosition = scrollController.position.pixels;
      return scrollPosition;
    }

    await tester.pumpWidget(
      MaterialApp(home: Material(child: Align(alignment: Alignment.topCenter, child: button))),
    );

    await tester.tap(find.text('99'));
    await tester.pumpAndSettle();
    expect(getMenuScroll(), 4312.0);
  });

  testWidgets('Dropdown in bottom showing top item', (WidgetTester tester) async {
    final List<DropdownMenuItem<int>> items = List<DropdownMenuItem<int>>.generate(
      100,
      (int i) => DropdownMenuItem<int>(value: i, child: Text('$i')),
    );

    final DropdownButton<int> button = DropdownButton<int>(
      value: 0,
      onChanged: (int? newValue) {},
      items: items,
    );

    double getMenuScroll() {
      double scrollPosition;
      final ScrollController scrollController = PrimaryScrollController.of(
        tester.element(find.byType(ListView)),
      );
      scrollPosition = scrollController.position.pixels;
      return scrollPosition;
    }

    await tester.pumpWidget(
      MaterialApp(home: Material(child: Align(alignment: Alignment.bottomCenter, child: button))),
    );

    await tester.tap(find.text('0'));
    await tester.pumpAndSettle();
    expect(getMenuScroll(), 0.0);
  });

  testWidgets('Dropdown in center showing bottom item', (WidgetTester tester) async {
    final List<DropdownMenuItem<int>> items = List<DropdownMenuItem<int>>.generate(
      100,
      (int i) => DropdownMenuItem<int>(value: i, child: Text('$i')),
    );

    final DropdownButton<int> button = DropdownButton<int>(
      value: 99,
      onChanged: (int? newValue) {},
      items: items,
    );

    double getMenuScroll() {
      double scrollPosition;
      final ScrollController scrollController = PrimaryScrollController.of(
        tester.element(find.byType(ListView)),
      );
      scrollPosition = scrollController.position.pixels;
      return scrollPosition;
    }

    await tester.pumpWidget(MaterialApp(home: Material(child: Align(child: button))));

    await tester.tap(find.text('99'));
    await tester.pumpAndSettle();
    expect(getMenuScroll(), 4312.0);
  });

  testWidgets('Dropdown menu respects parent size limits', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/24417
    int? selectedIndex;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          bottomNavigationBar: const SizedBox(height: 200),
          body: Navigator(
            onGenerateRoute: (RouteSettings settings) {
              return MaterialPageRoute<void>(
                builder: (BuildContext context) {
                  return SafeArea(
                    child: Container(
                      alignment: Alignment.topLeft,
                      // From material/dropdown.dart (menus are unaligned by default):
                      //  _kUnalignedMenuMargin = EdgeInsetsDirectional.only(start: 16.0, end: 24.0)
                      // This padding ensures that the entire menu will be visible
                      padding: const EdgeInsetsDirectional.only(start: 16.0, end: 24.0),
                      child: DropdownButton<int>(
                        value: 12,
                        onChanged: (int? i) {
                          selectedIndex = i;
                        },
                        items: List<DropdownMenuItem<int>>.generate(100, (int i) {
                          return DropdownMenuItem<int>(value: i, child: Text('$i'));
                        }),
                      ),
                    ),
                  );
                },
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.text('12'));
    await tester.pumpAndSettle();
    expect(selectedIndex, null);

    await tester.tap(find.text('13').last);
    await tester.pumpAndSettle();
    expect(selectedIndex, 13);
  });

  testWidgets('Dropdown button will accept widgets as its underline', (WidgetTester tester) async {
    const BoxDecoration decoration = BoxDecoration(
      border: Border(bottom: BorderSide(color: Color(0xFFCCBB00), width: 4.0)),
    );
    const BoxDecoration defaultDecoration = BoxDecoration(
      border: Border(bottom: BorderSide(color: Color(0xFFBDBDBD), width: 0.0)),
    );

    final Widget customUnderline = Container(height: 4.0, decoration: decoration);
    final Key buttonKey = UniqueKey();

    final Finder decoratedBox = find.descendant(
      of: find.byKey(buttonKey),
      matching: find.byType(DecoratedBox),
    );

    await tester.pumpWidget(
      buildFrame(buttonKey: buttonKey, underline: customUnderline, onChanged: onChanged),
    );
    expect(tester.widgetList<DecoratedBox>(decoratedBox).last.decoration, decoration);

    await tester.pumpWidget(buildFrame(buttonKey: buttonKey, onChanged: onChanged));
    expect(tester.widgetList<DecoratedBox>(decoratedBox).last.decoration, defaultDecoration);
  });

  testWidgets('DropdownButton selectedItemBuilder builds custom buttons', (
    WidgetTester tester,
  ) async {
    const List<String> items = <String>['One', 'Two', 'Three'];
    String? selectedItem = items[0];

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Scaffold(
              body: DropdownButton<String>(
                value: selectedItem,
                onChanged: (String? string) {
                  setState(() => selectedItem = string);
                },
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

  testWidgets('DropdownButton uses default color when expanded', (WidgetTester tester) async {
    await checkDropdownColor(tester);
  });

  testWidgets('DropdownButton uses dropdownColor when expanded', (WidgetTester tester) async {
    await checkDropdownColor(tester, color: const Color.fromRGBO(120, 220, 70, 0.8));
  });

  testWidgets('DropdownButtonFormField uses dropdownColor when expanded', (
    WidgetTester tester,
  ) async {
    await checkDropdownColor(
      tester,
      color: const Color.fromRGBO(120, 220, 70, 0.8),
      isFormField: true,
    );
  });

  testWidgets('DropdownButton hint displays properly when selectedItemBuilder is defined', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/42340
    final List<String> items = <String>['1', '2', '3'];
    String? selectedItem;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Scaffold(
              body: DropdownButton<String>(
                hint: const Text('Please select an item'),
                value: selectedItem,
                onChanged: (String? string) {
                  setState(() {
                    selectedItem = string;
                  });
                },
                selectedItemBuilder: (BuildContext context) {
                  return items.map((String item) {
                    return Text('You have selected: $item');
                  }).toList();
                },
                items:
                    items.map((String item) {
                      return DropdownMenuItem<String>(value: item, child: Text(item));
                    }).toList(),
              ),
            ),
          );
        },
      ),
    );

    // Initially shows the hint text
    expect(find.text('Please select an item'), findsOneWidget);
    await tester.tap(find.text('Please select an item', skipOffstage: false), warnIfMissed: false);
    await tester.pumpAndSettle();
    await tester.tap(find.text('1'));
    await tester.pumpAndSettle();
    // Selecting an item should display its corresponding item builder
    expect(find.text('You have selected: 1'), findsOneWidget);
  });

  testWidgets('Variable size and oversized menu items', (WidgetTester tester) async {
    final List<double> itemHeights = <double>[30, 40, 50, 60];
    double? dropdownValue = itemHeights[0];

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return DropdownButton<double>(
                  onChanged: (double? value) {
                    setState(() {
                      dropdownValue = value;
                    });
                  },
                  value: dropdownValue,
                  itemHeight: null,
                  items:
                      itemHeights.map<DropdownMenuItem<double>>((double value) {
                        return DropdownMenuItem<double>(
                          key: ValueKey<double>(value),
                          value: value,
                          child: Center(
                            child: Container(width: 100, height: value, color: Colors.blue),
                          ),
                        );
                      }).toList(),
                );
              },
            ),
          ),
        ),
      );
    }

    final Finder dropdownIcon = find.byType(Icon);
    final Finder item30 = find.byKey(const ValueKey<double>(30), skipOffstage: false);
    final Finder item40 = find.byKey(const ValueKey<double>(40), skipOffstage: false);
    final Finder item50 = find.byKey(const ValueKey<double>(50), skipOffstage: false);
    final Finder item60 = find.byKey(const ValueKey<double>(60), skipOffstage: false);

    // Only the DropdownButton is visible. It contains the selected item
    // and a dropdown arrow icon.
    await tester.pumpWidget(buildFrame());
    expect(dropdownIcon, findsOneWidget);
    expect(item30, findsOneWidget);

    // All menu items have a minimum height of 48. The centers of the
    // dropdown icon and the selected menu item are vertically aligned
    // and horizontally adjacent.
    expect(tester.getSize(item30), const Size(100, 48));
    expect(tester.getCenter(item30).dy, tester.getCenter(dropdownIcon).dy);
    expect(tester.getTopRight(item30).dx, tester.getTopLeft(dropdownIcon).dx);

    // Show the popup menu.
    await tester.tap(item30);
    await tester.pumpAndSettle();

    // Each item appears twice, once in the menu and once
    // in the dropdown button's IndexedStack.
    expect(item30.evaluate().length, 2);
    expect(item40.evaluate().length, 2);
    expect(item50.evaluate().length, 2);
    expect(item60.evaluate().length, 2);

    // Verify that the items have the expected sizes. The width of the items
    // that appear in the menu is padded by 16 on the left and right.
    expect(tester.getSize(item30.first), const Size(100, 48));
    expect(tester.getSize(item40.first), const Size(100, 48));
    expect(tester.getSize(item50.first), const Size(100, 50));
    expect(tester.getSize(item60.first), const Size(100, 60));
    expect(tester.getSize(item30.last), const Size(132, 48));
    expect(tester.getSize(item40.last), const Size(132, 48));
    expect(tester.getSize(item50.last), const Size(132, 50));
    expect(tester.getSize(item60.last), const Size(132, 60));

    // The vertical center of the selectedItem (item30) should
    // line up with its button counterpart.
    expect(tester.getCenter(item30.first).dy, tester.getCenter(item30.last).dy);

    // The menu items should be arranged in a column.
    expect(tester.getBottomLeft(item30.last), tester.getTopLeft(item40.last));
    expect(tester.getBottomLeft(item40.last), tester.getTopLeft(item50.last));
    expect(tester.getBottomLeft(item50.last), tester.getTopLeft(item60.last));

    // Dismiss the menu by selecting item40 and then show the menu again.
    await tester.tap(item40.last);
    await tester.pumpAndSettle();
    expect(dropdownValue, 40);
    await tester.tap(item40.first);
    await tester.pumpAndSettle();

    // The vertical center of the selectedItem (item40) should
    // line up with its button counterpart.
    expect(tester.getCenter(item40.first).dy, tester.getCenter(item40.last).dy);
  });

  testWidgets('DropdownButton menu items do not resize when its route is popped', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/44877.
    const List<String> items = <String>['one', 'two', 'three'];
    String? item = items[0];
    late double textScale;

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            builder: (BuildContext context, Widget? child) {
              textScale = MediaQuery.of(context).textScaler.scale(14) / 14;
              return MediaQuery(
                data: MediaQueryData(textScaler: TextScaler.linear(textScale)),
                child: child!,
              );
            },
            home: Scaffold(
              body: DropdownButton<String>(
                value: item,
                items:
                    items
                        .map(
                          (String item) => DropdownMenuItem<String>(value: item, child: Text(item)),
                        )
                        .toList(),
                onChanged: (String? newItem) {
                  setState(() {
                    item = newItem;
                    textScale += 0.1;
                  });
                },
              ),
            ),
          );
        },
      ),
    );

    // Verify that the first item is showing.
    expect(find.text('one'), findsOneWidget);

    // Select a different item to trigger setState, which updates mediaQuery
    // and forces a performLayout on the popped _DropdownRoute. This operation
    // should not cause an exception.
    await tester.tap(find.text('one'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('two').last);
    await tester.pumpAndSettle();
    expect(find.text('two'), findsOneWidget);
  });

  testWidgets('DropdownButton hint is selected item', (WidgetTester tester) async {
    const double hintPaddingOffset = 8;
    const List<String> itemValues = <String>['item0', 'item1', 'item2', 'item3'];
    String? selectedItem = 'item0';

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          body: ButtonTheme(
            alignedDropdown: true,
            child: DropdownButtonHideUnderline(
              child: Center(
                child: StatefulBuilder(
                  builder: (BuildContext context, StateSetter setState) {
                    // The pretzel below is from an actual app. The price
                    // of limited configurability is keeping this working.
                    return DropdownButton<String>(
                      isExpanded: true,
                      elevation: 2,
                      hint: LayoutBuilder(
                        builder: (BuildContext context, BoxConstraints constraints) {
                          // Stack with a positioned widget is used to override the
                          // hard coded 16px margin in the dropdown code, so that
                          // this hint aligns "properly" with the menu.
                          return Stack(
                            clipBehavior: Clip.none,
                            alignment: Alignment.topCenter,
                            children: <Widget>[
                              PositionedDirectional(
                                width: constraints.maxWidth + hintPaddingOffset,
                                start: -hintPaddingOffset,
                                top: 4.0,
                                child: Text('-$selectedItem-'),
                              ),
                            ],
                          );
                        },
                      ),
                      onChanged: (String? value) {
                        setState(() {
                          selectedItem = value;
                        });
                      },
                      icon: Container(),
                      items:
                          itemValues.map<DropdownMenuItem<String>>((String value) {
                            return DropdownMenuItem<String>(value: value, child: Text(value));
                          }).toList(),
                    );
                  },
                ),
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    expect(tester.getTopLeft(find.text('-item0-')).dx, 8);

    // Show the popup menu.
    await tester.tap(find.text('-item0-', skipOffstage: false), warnIfMissed: false);
    await tester.pumpAndSettle();

    expect(tester.getTopLeft(find.text('-item0-')).dx, 8);
  });

  Finder findInputDecoratorBorderPainter() {
    return find.descendant(
      of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_BorderContainer'),
      matching: find.byWidgetPredicate((Widget w) => w is CustomPaint),
    );
  }

  testWidgets('DropdownButton can be focused, and has focusColor', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final UniqueKey buttonKey = UniqueKey();
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      buildFrame(
        buttonKey: buttonKey,
        onChanged: onChanged,
        focusNode: focusNode,
        autofocus: true,
        useMaterial3: false,
      ),
    );
    await tester.pumpAndSettle(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      find.byType(Material),
      paints..rect(
        rect: const Rect.fromLTRB(348.0, 276.0, 452.0, 324.0),
        color: const Color(0x1f000000),
      ),
    );

    await tester.pumpWidget(
      buildFrame(
        buttonKey: buttonKey,
        onChanged: onChanged,
        focusNode: focusNode,
        focusColor: const Color(0xff00ff00),
        useMaterial3: false,
      ),
    );
    await tester.pumpAndSettle(); // Pump a frame for autofocus to take effect.
    expect(
      find.byType(Material),
      paints..rect(
        rect: const Rect.fromLTRB(348.0, 276.0, 452.0, 324.0),
        color: const Color(0x1f00ff00),
      ),
    );
  });

  testWidgets('DropdownButtonFormField can be focused, and has focusColor', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final UniqueKey buttonKey = UniqueKey();
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButtonFormField');
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      buildFrame(
        isFormField: true,
        buttonKey: buttonKey,
        onChanged: onChanged,
        focusNode: focusNode,
        autofocus: true,
        decoration: const InputDecoration(filled: true),
      ),
    );

    await tester.pump(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isTrue);

    // Default focus Color from InputDecorator defaults.
    final ThemeData theme = Theme.of(tester.element(find.byType(InputDecorator)));
    expect(
      findInputDecoratorBorderPainter(),
      paints..path(style: PaintingStyle.fill, color: theme.colorScheme.surfaceContainerHighest),
    );

    // Focus color from Decoration.
    await tester.pumpWidget(
      buildFrame(
        isFormField: true,
        buttonKey: buttonKey,
        onChanged: onChanged,
        focusNode: focusNode,
        decoration: const InputDecoration(filled: true, focusColor: Color(0xff00ffff)),
      ),
    );

    expect(
      findInputDecoratorBorderPainter(),
      paints..path(style: PaintingStyle.fill, color: const Color(0xff00ffff)),
    );

    // Focus color from focusColor property.
    await tester.pumpWidget(
      buildFrame(
        isFormField: true,
        buttonKey: buttonKey,
        onChanged: onChanged,
        focusNode: focusNode,
        decoration: const InputDecoration(filled: true, focusColor: Color(0xff00ffff)),
        focusColor: const Color(0xff00ff00),
      ),
    );

    expect(
      findInputDecoratorBorderPainter(),
      paints..path(style: PaintingStyle.fill, color: const Color(0xff00ff00)),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/147069.
  testWidgets('DropdownButtonFormField can be hovered', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final UniqueKey buttonKey = UniqueKey();

    await tester.pumpWidget(
      buildFrame(isFormField: true, buttonKey: buttonKey, onChanged: onChanged),
    );
    await tester.pump();

    // Check inputDecorator.isHovering value because DropdownButtonFormField
    // delegates to the InputDecorator which manages hover overlay.
    InputDecorator inputDecorator = tester.widget(find.byType(InputDecorator));
    expect(inputDecorator.isHovering, false);

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byKey(buttonKey)));
    await tester.pump();

    inputDecorator = tester.widget(find.byType(InputDecorator));
    expect(inputDecorator.isHovering, true);
  });

  // Regression test for https://github.com/flutter/flutter/issues/151460.
  testWidgets('DropdownButtonFormField has hover color', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final UniqueKey buttonKey = UniqueKey();

    await tester.pumpWidget(
      buildFrame(
        isFormField: true,
        buttonKey: buttonKey,
        onChanged: onChanged,
        // Setting InputDecoration.filled to true is required to get overlay showing.
        decoration: const InputDecoration(filled: true),
      ),
    );
    await tester.pump();

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byKey(buttonKey)));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 15)); // Hover animation.

    // Default hover color.
    final ThemeData theme = Theme.of(tester.element(find.byType(InputDecorator)));
    expect(
      findInputDecoratorBorderPainter(),
      paints..path(
        style: PaintingStyle.fill,
        color: Color.alphaBlend(theme.hoverColor, theme.colorScheme.surfaceContainerHighest),
      ),
    );

    // Custom hover color.
    const Color hoverColor = Color(0xaa00ff00);
    await tester.pumpWidget(
      buildFrame(
        isFormField: true,
        buttonKey: buttonKey,
        onChanged: onChanged,
        decoration: const InputDecoration(filled: true, hoverColor: hoverColor),
      ),
    );
    expect(
      findInputDecoratorBorderPainter(),
      paints..path(
        style: PaintingStyle.fill,
        color: Color.alphaBlend(hoverColor, theme.colorScheme.surfaceContainerHighest),
      ),
    );
  });

  testWidgets("DropdownButton won't be focused if not enabled", (WidgetTester tester) async {
    final UniqueKey buttonKey = UniqueKey();
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      buildFrame(
        buttonKey: buttonKey,
        focusNode: focusNode,
        autofocus: true,
        focusColor: const Color(0xff00ff00),
      ),
    );
    await tester.pump(); // Pump a frame for autofocus to take effect (although it shouldn't).
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      find.byKey(buttonKey),
      isNot(
        paints..rrect(
          rrect: const RRect.fromLTRBXY(0.0, 0.0, 104.0, 48.0, 4.0, 4.0),
          color: const Color(0xff00ff00),
        ),
      ),
    );
  });

  testWidgets('DropdownButton is activated with the enter key', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
    addTearDown(focusNode.dispose);
    String? value = 'one';

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return DropdownButton<String>(
                  focusNode: focusNode,
                  autofocus: true,
                  onChanged: (String? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  value: value,
                  itemHeight: null,
                  items:
                      menuItems.map<DropdownMenuItem<String>>((String item) {
                        return DropdownMenuItem<String>(
                          key: ValueKey<String>(item),
                          value: item,
                          child: Text(item, key: ValueKey<String>('${item}Text')),
                        );
                      }).toList(),
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    await tester.pump(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation
    expect(value, equals('one'));

    await tester.sendKeyEvent(LogicalKeyboardKey.tab); // Focus 'two'
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.enter); // Select 'two'.
    await tester.pump();

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('two'));
  });

  // Regression test for https://github.com/flutter/flutter/issues/77655.
  testWidgets('DropdownButton selecting a null valued item should be selected', (
    WidgetTester tester,
  ) async {
    final List<MapEntry<String?, String>> items = <MapEntry<String?, String>>[
      const MapEntry<String?, String>(null, 'None'),
      const MapEntry<String?, String>('one', 'One'),
      const MapEntry<String?, String>('two', 'Two'),
    ];
    String? selectedItem = 'one';

    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setState) {
          return MaterialApp(
            home: Scaffold(
              body: DropdownButton<String>(
                value: selectedItem,
                onChanged: (String? string) {
                  setState(() {
                    selectedItem = string;
                  });
                },
                items:
                    items.map((MapEntry<String?, String> item) {
                      return DropdownMenuItem<String>(value: item.key, child: Text(item.value));
                    }).toList(),
              ),
            ),
          );
        },
      ),
    );

    await tester.tap(find.text('One'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('None').last);
    await tester.pumpAndSettle();
    expect(find.text('None'), findsOneWidget);
  });

  testWidgets('DropdownButton is activated with the space key', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
    addTearDown(focusNode.dispose);
    String? value = 'one';

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return DropdownButton<String>(
                  focusNode: focusNode,
                  autofocus: true,
                  onChanged: (String? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  value: value,
                  itemHeight: null,
                  items:
                      menuItems.map<DropdownMenuItem<String>>((String item) {
                        return DropdownMenuItem<String>(
                          key: ValueKey<String>(item),
                          value: item,
                          child: Text(item, key: ValueKey<String>('${item}Text')),
                        );
                      }).toList(),
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    await tester.pump(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.space);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation
    expect(value, equals('one'));

    await tester.sendKeyEvent(LogicalKeyboardKey.tab); // Focus 'two'
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.space); // Select 'two'.
    await tester.pump();

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('two'));
  });

  testWidgets('Selected element is focused when dropdown is opened', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
    addTearDown(focusNode.dispose);
    String? value = 'one';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return DropdownButton<String>(
                  focusNode: focusNode,
                  autofocus: true,
                  onChanged: (String? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  value: value,
                  itemHeight: null,
                  items:
                      menuItems.map<DropdownMenuItem<String>>((String item) {
                        return DropdownMenuItem<String>(
                          key: ValueKey<String>(item),
                          value: item,
                          child: Text(item, key: ValueKey<String>('Text $item')),
                        );
                      }).toList(),
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.pump(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu open animation
    expect(value, equals('one'));
    expect(
      Focus.of(tester.element(find.byKey(const ValueKey<String>('one')).last)).hasPrimaryFocus,
      isTrue,
    );

    await tester.sendKeyEvent(LogicalKeyboardKey.tab); // Focus 'two'
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.enter); // Select 'two' and close the dropdown.

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu close animation

    expect(value, equals('two'));

    // Now make sure that "two" is focused when we re-open the dropdown.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu open animation
    expect(value, equals('two'));
    final Element element = tester.element(find.byKey(const ValueKey<String>('two')).last);
    final FocusNode node = Focus.of(element);
    expect(node.hasFocus, isTrue);
  });

  testWidgets(
    'Selected element is correctly focused with dropdown that more items than fit on the screen',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
      addTearDown(focusNode.dispose);
      int? value = 1;
      final List<int> hugeMenuItems = List<int>.generate(50, (int index) => index);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return DropdownButton<int>(
                    focusNode: focusNode,
                    autofocus: true,
                    onChanged: (int? newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    value: value,
                    itemHeight: null,
                    items:
                        hugeMenuItems.map<DropdownMenuItem<int>>((int item) {
                          return DropdownMenuItem<int>(
                            key: ValueKey<int>(item),
                            value: item,
                            child: Text(item.toString(), key: ValueKey<String>('Text $item')),
                          );
                        }).toList(),
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.pump(); // Pump a frame for autofocus to take effect.
      expect(focusNode.hasPrimaryFocus, isTrue);

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the menu open animation
      expect(value, equals(1));
      expect(
        Focus.of(tester.element(find.byKey(const ValueKey<int>(1)).last)).hasPrimaryFocus,
        isTrue,
      );

      for (int i = 0; i < 41; ++i) {
        await tester.sendKeyEvent(LogicalKeyboardKey.tab); // Move to the next one.
        await tester.pumpAndSettle(
          const Duration(milliseconds: 200),
        ); // Wait for it to animate the menu.
      }
      await tester.sendKeyEvent(LogicalKeyboardKey.enter); // Select '42' and close the dropdown.
      await tester.pumpAndSettle(const Duration(seconds: 1)); // Finish the menu close animation
      expect(value, equals(42));

      // Now make sure that "42" is focused when we re-open the dropdown.
      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();
      await tester.pump(const Duration(seconds: 1)); // finish the menu open animation
      expect(value, equals(42));
      final Element element = tester.element(find.byKey(const ValueKey<int>(42)).last);
      final FocusNode node = Focus.of(element);
      expect(node.hasFocus, isTrue);
    },
  );

  testWidgets("Having a focused element doesn't interrupt scroll when flung by touch", (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
    addTearDown(focusNode.dispose);
    int? value = 1;
    final List<int> hugeMenuItems = List<int>.generate(100, (int index) => index);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return DropdownButton<int>(
                  focusNode: focusNode,
                  autofocus: true,
                  onChanged: (int? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  value: value,
                  itemHeight: null,
                  items:
                      hugeMenuItems.map<DropdownMenuItem<int>>((int item) {
                        return DropdownMenuItem<int>(
                          key: ValueKey<int>(item),
                          value: item,
                          child: Text(item.toString(), key: ValueKey<String>('Text $item')),
                        );
                      }).toList(),
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.pump(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    expect(value, equals(1));
    expect(
      Focus.of(tester.element(find.byKey(const ValueKey<int>(1)).last)).hasPrimaryFocus,
      isTrue,
    );

    // Move to an item very far down the menu.
    for (int i = 0; i < 90; ++i) {
      await tester.sendKeyEvent(LogicalKeyboardKey.tab); // Move to the next one.
      await tester.pumpAndSettle(); // Wait for it to animate the menu.
    }
    expect(
      Focus.of(tester.element(find.byKey(const ValueKey<int>(91)).last)).hasPrimaryFocus,
      isTrue,
    );

    // Scroll back to the top using touch, and make sure we end up there.
    final Finder menu = find.byWidgetPredicate((Widget widget) {
      return widget.runtimeType.toString().startsWith('_DropdownMenu<');
    });
    final Rect menuRect = tester.getRect(menu).shift(tester.getTopLeft(menu));
    for (int i = 0; i < 10; ++i) {
      await tester.fling(menu, Offset(0.0, menuRect.height), 10.0);
    }
    await tester.pumpAndSettle();

    // Make sure that we made it to the top and something didn't stop the
    // scroll.
    expect(find.byKey(const ValueKey<int>(1)), findsNWidgets(2));
    expect(
      tester.getRect(find.byKey(const ValueKey<int>(1)).last),
      equals(const Rect.fromLTRB(372.0, 104.0, 436.0, 152.0)),
    );

    // Scrolling to the top again has removed the one the focus was on from the
    // tree, causing it to lose focus.
    expect(
      Focus.of(
        tester.element(find.byKey(const ValueKey<int>(91), skipOffstage: false).last),
      ).hasPrimaryFocus,
      isFalse,
    );
  });

  testWidgets('DropdownButton onTap callback can request focus', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton')..addListener(() {});
    addTearDown(focusNode.dispose);
    int? value = 1;
    final List<int> hugeMenuItems = List<int>.generate(100, (int index) => index);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return DropdownButton<int>(
                  focusNode: focusNode,
                  onChanged: (int? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  value: value,
                  itemHeight: null,
                  items:
                      hugeMenuItems.map<DropdownMenuItem<int>>((int item) {
                        return DropdownMenuItem<int>(
                          key: ValueKey<int>(item),
                          value: item,
                          child: Text(item.toString()),
                        );
                      }).toList(),
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.pump(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isFalse);

    await tester.tap(find.text('1'));
    await tester.pumpAndSettle();

    // Close the dropdown menu.
    await tester.tapAt(const Offset(1.0, 1.0));
    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
  });

  testWidgets('DropdownButton changes selected item with arrow keys', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'DropdownButton');
    addTearDown(focusNode.dispose);
    String? value = 'one';

    Widget buildFrame() {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return DropdownButton<String>(
                  focusNode: focusNode,
                  autofocus: true,
                  onChanged: (String? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                  value: value,
                  itemHeight: null,
                  items:
                      menuItems.map<DropdownMenuItem<String>>((String item) {
                        return DropdownMenuItem<String>(
                          key: ValueKey<String>(item),
                          value: item,
                          child: Text(item, key: ValueKey<String>('${item}Text')),
                        );
                      }).toList(),
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    await tester.pump(); // Pump a frame for autofocus to take effect.
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation
    expect(value, equals('one'));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Focus 'two'.
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Focus 'three'.
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp); // Back to 'two'.
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.enter); // Select 'two'.
    await tester.pump();

    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(value, equals('two'));
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

    Widget build() => buildFrame(value: value, onChanged: onChanged, onTap: onTap);
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
    await tester.tap(find.text('three', skipOffstage: false), warnIfMissed: false);
    await tester.pumpAndSettle();

    expect(value, equals('three'));
    expect(dropdownButtonTapCounter, 2); // Should update counter.

    // Tap dropdown menu item.
    await tester.tap(find.text('two').last);
    await tester.pumpAndSettle();

    expect(value, equals('two'));
    expect(dropdownButtonTapCounter, 2); // Should not change.
  });

  testWidgets('DropdownMenuItem onTap callback is called when defined', (
    WidgetTester tester,
  ) async {
    String? value = 'one';
    final List<int> menuItemTapCounters = <int>[0, 0, 0, 0];
    void onChanged(String? newValue) {
      value = newValue;
    }

    final List<VoidCallback> onTapCallbacks = <VoidCallback>[
      () {
        menuItemTapCounters[0] += 1;
      },
      () {
        menuItemTapCounters[1] += 1;
      },
      () {
        menuItemTapCounters[2] += 1;
      },
      () {
        menuItemTapCounters[3] += 1;
      },
    ];

    int currentIndex = -1;
    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: RepaintBoundary(
            child: DropdownButton<String>(
              value: value,
              onChanged: onChanged,
              items:
                  menuItems.map<DropdownMenuItem<String>>((String item) {
                    currentIndex += 1;
                    return DropdownMenuItem<String>(
                      value: item,
                      onTap: onTapCallbacks[currentIndex],
                      child: Text(item),
                    );
                  }).toList(),
            ),
          ),
        ),
      ),
    );

    // Tap dropdown button.
    await tester.tap(find.text('one'));
    await tester.pumpAndSettle();

    expect(value, equals('one'));
    // Counters should still be zero.
    expect(menuItemTapCounters, <int>[0, 0, 0, 0]);

    // Tap dropdown menu item.
    await tester.tap(find.text('three').last);
    await tester.pumpAndSettle();

    // Should update the counter for the third item (second index).
    expect(value, equals('three'));
    expect(menuItemTapCounters, <int>[0, 0, 1, 0]);

    // Tap dropdown button again.
    await tester.tap(find.text('three', skipOffstage: false), warnIfMissed: false);
    await tester.pumpAndSettle();

    // Should not change.
    expect(value, equals('three'));
    expect(menuItemTapCounters, <int>[0, 0, 1, 0]);

    // Tap dropdown menu item.
    await tester.tap(find.text('two').last);
    await tester.pumpAndSettle();

    // Should update the counter for the second item (first index).
    expect(value, equals('two'));
    expect(menuItemTapCounters, <int>[0, 1, 1, 0]);

    // Tap dropdown button again.
    await tester.tap(find.text('two', skipOffstage: false), warnIfMissed: false);
    await tester.pumpAndSettle();

    // Should not change.
    expect(value, equals('two'));
    expect(menuItemTapCounters, <int>[0, 1, 1, 0]);

    // Tap the already selected menu item
    await tester.tap(find.text('two').last);
    await tester.pumpAndSettle();

    // Should update the counter for the second item (first index), even
    // though it was already selected.
    expect(value, equals('two'));
    expect(menuItemTapCounters, <int>[0, 2, 1, 0]);
  });

  testWidgets(
    'Does not crash when option is selected without waiting for opening animation to complete',
    (WidgetTester tester) async {
      // Regression test for b/171846624.

      final List<String> options = <String>['first', 'second', 'third'];
      String? value = options.first;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: StatefulBuilder(
              builder:
                  (BuildContext context, StateSetter setState) => DropdownButton<String>(
                    value: value,
                    items:
                        options
                            .map((String s) => DropdownMenuItem<String>(value: s, child: Text(s)))
                            .toList(),
                    onChanged: (String? v) {
                      setState(() {
                        value = v;
                      });
                    },
                  ),
            ),
          ),
        ),
      );
      expect(find.text('first').hitTestable(), findsOneWidget);
      expect(find.text('second').hitTestable(), findsNothing);
      expect(find.text('third').hitTestable(), findsNothing);

      // Open dropdown.
      await tester.tap(find.text('first').hitTestable());
      await tester.pump();

      expect(find.text('third').hitTestable(), findsOneWidget);
      expect(find.text('first').hitTestable(), findsOneWidget);
      expect(find.text('second').hitTestable(), findsOneWidget);

      // Deliberately not waiting for opening animation to complete!

      // Select an option in dropdown.
      await tester.tap(find.text('third').hitTestable());
      await tester.pump();
      expect(find.text('third').hitTestable(), findsOneWidget);
      expect(find.text('first').hitTestable(), findsNothing);
      expect(find.text('second').hitTestable(), findsNothing);
    },
  );

  testWidgets('Dropdown menu should persistently show a scrollbar if it is scrollable', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      buildFrame(
        value: '0',
        // menu is short enough to fit onto the screen.
        items: List<String>.generate(/*length=*/ 10, (int index) => index.toString()),
        onChanged: onChanged,
      ),
    );
    await tester.tap(find.text('0'));
    await tester.pumpAndSettle();

    ScrollController scrollController = PrimaryScrollController.of(
      tester.element(find.byType(ListView)),
    );
    // The scrollbar shouldn't show if the list fits into the screen.
    expect(scrollController.position.maxScrollExtent, 0);
    expect(find.byType(Scrollbar), isNot(paints..rect()));

    await tester.tap(find.text('0').last);
    await tester.pumpAndSettle();
    await tester.pumpWidget(
      buildFrame(
        value: '0',
        // menu is too long to fit onto the screen.
        items: List<String>.generate(/*length=*/ 100, (int index) => index.toString()),
        onChanged: onChanged,
      ),
    );
    await tester.tap(find.text('0'));
    await tester.pumpAndSettle();

    scrollController = PrimaryScrollController.of(tester.element(find.byType(ListView)));
    // The scrollbar is shown when the list is longer than the height of the screen.
    expect(scrollController.position.maxScrollExtent > 0, isTrue);
    expect(find.byType(Scrollbar), paints..rect());
  });

  testWidgets(
    "Dropdown menu's maximum height should be influenced by DropdownButton.menuMaxHeight.",
    (WidgetTester tester) async {
      await tester.pumpWidget(
        buildFrame(
          value: '0',
          items: List<String>.generate(/*length=*/ 64, (int index) => index.toString()),
          onChanged: onChanged,
        ),
      );
      await tester.tap(find.text('0'));
      await tester.pumpAndSettle();

      final Element element = tester.element(find.byType(ListView));
      double menuHeight = element.size!.height;
      // The default maximum height should be one item height from the edge.
      // https://material.io/design/components/menus.html#usage
      final double mediaHeight = MediaQuery.of(element).size.height;
      final double defaultMenuHeight = mediaHeight - (2 * kMinInteractiveDimension);
      expect(menuHeight, defaultMenuHeight);

      await tester.tap(find.text('0').last);
      await tester.pumpAndSettle();

      // Set menuMaxHeight which is less than defaultMenuHeight
      await tester.pumpWidget(
        buildFrame(
          value: '0',
          items: List<String>.generate(/*length=*/ 64, (int index) => index.toString()),
          onChanged: onChanged,
          menuMaxHeight: 7 * kMinInteractiveDimension,
        ),
      );
      await tester.tap(find.text('0'));
      await tester.pumpAndSettle();

      menuHeight = tester.element(find.byType(ListView)).size!.height;

      expect(menuHeight == defaultMenuHeight, isFalse);
      expect(menuHeight, kMinInteractiveDimension * 7);

      await tester.tap(find.text('0').last);
      await tester.pumpAndSettle();

      // Set menuMaxHeight which is greater than defaultMenuHeight
      await tester.pumpWidget(
        buildFrame(
          value: '0',
          items: List<String>.generate(/*length=*/ 64, (int index) => index.toString()),
          onChanged: onChanged,
          menuMaxHeight: mediaHeight,
        ),
      );

      await tester.tap(find.text('0'));
      await tester.pumpAndSettle();

      menuHeight = tester.element(find.byType(ListView)).size!.height;
      expect(menuHeight, defaultMenuHeight);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/89029
  testWidgets('menu position test with `menuMaxHeight`', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    await tester.pumpWidget(
      buildFrame(
        buttonKey: buttonKey,
        value: '6',
        items: List<String>.generate(/*length=*/ 64, (int index) => index.toString()),
        onChanged: onChanged,
        menuMaxHeight: 2 * kMinInteractiveDimension,
      ),
    );

    await tester.tap(find.text('6'));
    await tester.pumpAndSettle();

    final RenderBox menuBox = tester.renderObject(find.byType(ListView));
    final RenderBox buttonBox = tester.renderObject(find.byKey(buttonKey));
    // The menu's bottom should align with the drop-button's bottom.
    expect(
      menuBox.localToGlobal(menuBox.paintBounds.bottomCenter).dy,
      buttonBox.localToGlobal(buttonBox.paintBounds.bottomCenter).dy,
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/76614
  testWidgets('Do not crash if used in very short screen', (WidgetTester tester) async {
    // The default item height is 48.0 pixels and needs two items padding since
    // the menu requires empty space surrounding the menu. Finally, the constraint height
    // is 47.0 pixels for the menu rendering.
    tester.view.physicalSize = const Size(800.0, 48.0 * 3 - 1.0);
    tester.view.devicePixelRatio = 1;
    addTearDown(tester.view.reset);

    const String value = 'foo';
    final UniqueKey itemKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: DropdownButton<String>(
              value: value,
              items: <DropdownMenuItem<String>>[
                DropdownMenuItem<String>(key: itemKey, value: value, child: const Text(value)),
              ],
              onChanged: (_) {},
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text(value));
    await tester.pumpAndSettle();

    final List<RenderBox> itemBoxes =
        tester.renderObjectList<RenderBox>(find.byKey(itemKey)).toList();
    expect(itemBoxes[0].localToGlobal(Offset.zero).dx, 364.0);
    expect(itemBoxes[0].localToGlobal(Offset.zero).dy, 47.5);

    expect(itemBoxes[1].localToGlobal(Offset.zero).dx, 364.0);
    expect(itemBoxes[1].localToGlobal(Offset.zero).dy, 47.5);

    expect(
      find.ancestor(of: find.text(value).last, matching: find.byType(CustomPaint)).at(2),
      paints
        ..save()
        ..rrect()
        ..rrect()
        ..rrect()
        // The height of menu is 47.0.
        ..rrect(
          rrect: const RRect.fromLTRBXY(0.0, 0.0, 112.0, 47.0, 2.0, 2.0),
          color: Colors.grey[50],
          hasMaskFilter: false,
        ),
    );
  });

  testWidgets('Tapping a disabled item should not close DropdownButton', (
    WidgetTester tester,
  ) async {
    String? value = 'first';

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder:
                (BuildContext context, StateSetter setState) => DropdownButton<String>(
                  value: value,
                  items: const <DropdownMenuItem<String>>[
                    DropdownMenuItem<String>(enabled: false, child: Text('disabled')),
                    DropdownMenuItem<String>(value: 'first', child: Text('first')),
                    DropdownMenuItem<String>(value: 'second', child: Text('second')),
                  ],
                  onChanged: (String? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                ),
          ),
        ),
      ),
    );

    // Open dropdown.
    await tester.tap(find.text('first').hitTestable());
    await tester.pumpAndSettle();

    // Tap on a disabled item.
    await tester.tap(find.text('disabled').hitTestable());
    await tester.pumpAndSettle();

    // The dropdown should still be open, i.e., there should be one widget with 'second' text.
    expect(find.text('second').hitTestable(), findsOneWidget);
  });

  testWidgets('Disabled item should not be focusable', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownButton<String>(
            value: 'enabled',
            onChanged: onChanged,
            items: const <DropdownMenuItem<String>>[
              DropdownMenuItem<String>(enabled: false, child: Text('disabled')),
              DropdownMenuItem<String>(value: 'enabled', child: Text('enabled')),
            ],
          ),
        ),
      ),
    );

    // Open dropdown.
    await tester.tap(find.text('enabled').hitTestable());
    await tester.pumpAndSettle();

    // The `FocusNode` of [disabledItem] should be `null` as enabled is false.
    final Element disabledItem = tester.element(find.text('disabled').hitTestable());
    expect(
      Focus.maybeOf(disabledItem),
      null,
      reason: 'Disabled menu item should not be able to request focus',
    );
  });

  testWidgets('alignment test', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    Widget buildFrame({AlignmentGeometry? buttonAlignment, AlignmentGeometry? menuAlignment}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownButton<String>(
              key: buttonKey,
              alignment: buttonAlignment ?? AlignmentDirectional.centerStart,
              value: 'enabled',
              onChanged: onChanged,
              items: <DropdownMenuItem<String>>[
                DropdownMenuItem<String>(
                  alignment: buttonAlignment ?? AlignmentDirectional.centerStart,
                  enabled: false,
                  child: const Text('disabled'),
                ),
                DropdownMenuItem<String>(
                  alignment: buttonAlignment ?? AlignmentDirectional.centerStart,
                  value: 'enabled',
                  child: const Text('enabled'),
                ),
              ],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());

    final RenderBox buttonBox = tester.renderObject(find.byKey(buttonKey));
    RenderBox selectedItemBox = tester.renderObject(find.text('enabled'));
    // Default to center-start aligned.
    expect(
      buttonBox.localToGlobal(Offset(0.0, buttonBox.size.height / 2.0)),
      selectedItemBox.localToGlobal(Offset(0.0, selectedItemBox.size.height / 2.0)),
    );

    await tester.pumpWidget(
      buildFrame(
        buttonAlignment: AlignmentDirectional.center,
        menuAlignment: AlignmentDirectional.center,
      ),
    );

    selectedItemBox = tester.renderObject(find.text('enabled'));
    // Should be center-center aligned, the icon size is 24.0 pixels.
    expect(
      buttonBox.localToGlobal(
        Offset((buttonBox.size.width - 24.0) / 2.0, buttonBox.size.height / 2.0),
      ),
      offsetMoreOrLessEquals(
        selectedItemBox.localToGlobal(
          Offset(selectedItemBox.size.width / 2.0, selectedItemBox.size.height / 2.0),
        ),
      ),
    );

    // Open dropdown.
    await tester.tap(find.text('enabled').hitTestable());
    await tester.pumpAndSettle();

    final RenderBox selectedItemBoxInMenu =
        tester.renderObjectList<RenderBox>(find.text('enabled')).toList()[1];
    final Finder menu = find.byWidgetPredicate((Widget widget) {
      return widget.runtimeType.toString().startsWith('_DropdownMenu<');
    });
    final Rect menuRect = tester.getRect(menu);
    final Offset center = selectedItemBoxInMenu.localToGlobal(
      Offset(selectedItemBoxInMenu.size.width / 2.0, selectedItemBoxInMenu.size.height / 2.0),
    );

    expect(center.dx, moreOrLessEquals(menuRect.topCenter.dx));
    expect(
      center.dy,
      moreOrLessEquals(
        selectedItemBox
            .localToGlobal(
              Offset(selectedItemBox.size.width / 2.0, selectedItemBox.size.height / 2.0),
            )
            .dy,
      ),
    );
  });

  group('feedback', () {
    late FeedbackTester feedback;

    setUp(() {
      feedback = FeedbackTester();
    });

    tearDown(() {
      feedback.dispose();
    });

    Widget feedbackBoilerplate({bool? enableFeedback}) {
      return MaterialApp(
        home: Material(
          child: DropdownButton<String>(
            value: 'One',
            enableFeedback: enableFeedback,
            underline: Container(height: 2, color: Colors.deepPurpleAccent),
            onChanged: (String? value) {},
            items:
                <String>['One', 'Two'].map<DropdownMenuItem<String>>((String value) {
                  return DropdownMenuItem<String>(value: value, child: Text(value));
                }).toList(),
          ),
        ),
      );
    }

    testWidgets('Dropdown with enabled feedback', (WidgetTester tester) async {
      const bool enableFeedback = true;

      await tester.pumpWidget(feedbackBoilerplate(enableFeedback: enableFeedback));

      await tester.tap(find.text('One'));
      await tester.pumpAndSettle();
      await tester.tap(find.widgetWithText(InkWell, 'One').last);
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });

    testWidgets('Dropdown with disabled feedback', (WidgetTester tester) async {
      const bool enableFeedback = false;

      await tester.pumpWidget(feedbackBoilerplate(enableFeedback: enableFeedback));

      await tester.tap(find.text('One'));
      await tester.pumpAndSettle();
      await tester.tap(find.widgetWithText(InkWell, 'One').last);
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);
    });

    testWidgets('Dropdown with enabled feedback by default', (WidgetTester tester) async {
      await tester.pumpWidget(feedbackBoilerplate());

      await tester.tap(find.text('One'));
      await tester.pumpAndSettle();
      await tester.tap(find.widgetWithText(InkWell, 'Two').last);
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });
  });

  testWidgets('DropdownButton changes mouse cursor when hovered', (WidgetTester tester) async {
    const Key key = Key('testDropdownButton');
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DropdownButton<String>(
            key: key,
            onChanged: (String? newValue) {},
            items:
                <String>['One', 'Two', 'Three', 'Four'].map<DropdownMenuItem<String>>((
                  String value,
                ) {
                  return DropdownMenuItem<String>(value: value, child: Text(value));
                }).toList(),
          ),
        ),
      ),
    );

    final Finder dropdownButtonFinder = find.byKey(key);
    final Offset onDropdownButton = tester.getCenter(dropdownButtonFinder);
    final Offset offDropdownButton =
        tester.getBottomRight(dropdownButtonFinder) + const Offset(1, 1);
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );

    await gesture.addPointer(location: onDropdownButton);

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
    await gesture.moveTo(offDropdownButton);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Test that mouse cursor doesn't change when button is disabled
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: DropdownButton<String>(
            key: key,
            onChanged: null,
            items:
                <String>['One', 'Two', 'Three', 'Four'].map<DropdownMenuItem<String>>((
                  String value,
                ) {
                  return DropdownMenuItem<String>(value: value, child: Text(value));
                }).toList(),
          ),
        ),
      ),
    );

    await gesture.moveTo(onDropdownButton);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
    await gesture.moveTo(offDropdownButton);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets(
    'Conflicting scrollbars are not applied by ScrollBehavior to Dropdown',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/83819
      // Open the dropdown menu
      final Key buttonKey = UniqueKey();
      await tester.pumpWidget(
        buildFrame(
          buttonKey: buttonKey,
          value: null, // nothing selected
          items: List<String>.generate(100, (int index) => index.toString()),
          onChanged: onChanged,
        ),
      );
      await tester.tap(find.byKey(buttonKey));
      await tester.pump();
      await tester.pumpAndSettle(); // finish the menu animation

      // The inherited ScrollBehavior should not apply Scrollbars since they are
      // already built in to the widget. For iOS platform, ScrollBar directly returns
      // CupertinoScrollbar
      expect(
        find.byType(CupertinoScrollbar),
        debugDefaultTargetPlatformOverride == TargetPlatform.iOS ? findsOneWidget : findsNothing,
      );
      expect(find.byType(Scrollbar), findsOneWidget);
      expect(find.byType(RawScrollbar), findsNothing);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('borderRadius property works properly', (WidgetTester tester) async {
    const double radius = 20.0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: DropdownButton<String>(
              borderRadius: BorderRadius.circular(radius),
              value: 'One',
              items:
                  <String>['One', 'Two', 'Three', 'Four'].map<DropdownMenuItem<String>>((
                    String value,
                  ) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
              onChanged: (_) {},
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('One'));
    await tester.pumpAndSettle();

    expect(
      find.ancestor(of: find.text('One').last, matching: find.byType(CustomPaint)).at(2),
      paints
        ..save()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(rrect: const RRect.fromLTRBXY(0.0, 0.0, 144.0, 208.0, radius, radius)),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/88574
  testWidgets("specifying itemHeight affects popup menu items' height", (
    WidgetTester tester,
  ) async {
    const String value = 'One';
    const double itemHeight = 80;
    final List<DropdownMenuItem<String>> menuItems =
        <String>[value, 'Two', 'Free', 'Four'].map<DropdownMenuItem<String>>((String value) {
          return DropdownMenuItem<String>(value: value, child: Text(value));
        }).toList();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownButton<String>(
              value: value,
              itemHeight: itemHeight,
              onChanged: (_) {},
              items: menuItems,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text(value));
    await tester.pumpAndSettle();

    for (final DropdownMenuItem<String> item in menuItems) {
      final Iterable<Element> elements = tester.elementList(find.byWidget(item));
      for (final Element element in elements) {
        expect(element.size!.height, itemHeight);
      }
    }
  });

  // Regression test for https://github.com/flutter/flutter/issues/92438
  testWidgets('Do not throw due to the double precision', (WidgetTester tester) async {
    const String value = 'One';
    const double itemHeight = 77.701;
    final List<DropdownMenuItem<String>> menuItems =
        <String>[value, 'Two', 'Free'].map<DropdownMenuItem<String>>((String value) {
          return DropdownMenuItem<String>(value: value, child: Text(value));
        }).toList();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownButton<String>(
              value: value,
              itemHeight: itemHeight,
              onChanged: (_) {},
              items: menuItems,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text(value));
    await tester.pumpAndSettle();

    expect(tester.takeException(), null);
  });

  testWidgets('BorderRadius property works properly for DropdownButtonFormField', (
    WidgetTester tester,
  ) async {
    const double radius = 20.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownButtonFormField<String>(
              borderRadius: BorderRadius.circular(radius),
              value: 'One',
              items:
                  <String>['One', 'Two', 'Three', 'Four'].map<DropdownMenuItem<String>>((
                    String value,
                  ) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
              onChanged: (_) {},
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('One'));
    await tester.pumpAndSettle();

    expect(
      find.ancestor(of: find.text('One').last, matching: find.byType(CustomPaint)).at(2),
      paints
        ..save()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(rrect: const RRect.fromLTRBXY(0.0, 0.0, 800.0, 208.0, radius, radius)),
    );
  });

  testWidgets('DropdownButton hint alignment', (WidgetTester tester) async {
    const String hintText = 'hint';

    // AlignmentDirectional.centerStart (default)
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.centerStart, isExpanded: false),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 348.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.topStart
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.topStart, isExpanded: false),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 348.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.bottomStart
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.bottomStart, isExpanded: false),
    );
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dx, 348.0);
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dy, 350.0);
    // AlignmentDirectional.center
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.center, isExpanded: false),
    );
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dx, 388.0);
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dy, 300.0);
    // AlignmentDirectional.topEnd
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.topEnd, isExpanded: false),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 428.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.centerEnd
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.centerEnd, isExpanded: false),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 428.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.bottomEnd
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.bottomEnd, isExpanded: false),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 428.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 334.0);

    // DropdownButton with `isExpanded: true`
    // AlignmentDirectional.centerStart (default)
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.centerStart, isExpanded: true),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 0.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.topStart
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.topStart, isExpanded: true),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 0.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.bottomStart
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.bottomStart, isExpanded: true),
    );
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dx, 0.0);
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dy, 350.0);
    // AlignmentDirectional.center
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.center, isExpanded: true),
    );
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dx, 388.0);
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dy, 300.0);
    // AlignmentDirectional.topEnd
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.topEnd, isExpanded: true),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 776.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.centerEnd
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.centerEnd, isExpanded: true),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 776.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.bottomEnd
    await tester.pumpWidget(
      buildDropdownWithHint(alignment: AlignmentDirectional.bottomEnd, isExpanded: true),
    );
    expect(tester.getBottomRight(find.text(hintText, skipOffstage: false)).dx, 776.0);
    expect(tester.getBottomRight(find.text(hintText, skipOffstage: false)).dy, 350.0);
  });

  testWidgets('DropdownButton hint alignment with selectedItemBuilder', (
    WidgetTester tester,
  ) async {
    const String hintText = 'hint';

    // AlignmentDirectional.centerStart (default)
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.centerStart,
        isExpanded: false,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 348.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.topStart
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.topStart,
        isExpanded: false,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 348.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.bottomStart
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.bottomStart,
        isExpanded: false,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dx, 348.0);
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dy, 350.0);
    // AlignmentDirectional.center
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.center,
        isExpanded: false,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dx, 388.0);
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dy, 300.0);
    // AlignmentDirectional.topEnd
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.topEnd,
        isExpanded: false,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 428.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.centerEnd
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.centerEnd,
        isExpanded: false,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 428.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.bottomEnd
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.bottomEnd,
        isExpanded: false,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 428.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 334.0);

    // DropdownButton with `isExpanded: true`
    // AlignmentDirectional.centerStart (default)
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.centerStart,
        isExpanded: true,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 0.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.topStart
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.topStart,
        isExpanded: true,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dx, 0.0);
    expect(tester.getTopLeft(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.bottomStart
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.bottomStart,
        isExpanded: true,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dx, 0.0);
    expect(tester.getBottomLeft(find.text(hintText, skipOffstage: false)).dy, 350.0);
    // AlignmentDirectional.center
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.center,
        isExpanded: true,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dx, 388.0);
    expect(tester.getCenter(find.text(hintText, skipOffstage: false)).dy, 300.0);
    // AlignmentDirectional.topEnd
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.topEnd,
        isExpanded: true,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 776.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 250.0);
    // AlignmentDirectional.centerEnd
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.centerEnd,
        isExpanded: true,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dx, 776.0);
    expect(tester.getTopRight(find.text(hintText, skipOffstage: false)).dy, 292.0);
    // AlignmentDirectional.bottomEnd
    await tester.pumpWidget(
      buildDropdownWithHint(
        alignment: AlignmentDirectional.bottomEnd,
        isExpanded: true,
        enableSelectedItemBuilder: true,
      ),
    );
    expect(tester.getBottomRight(find.text(hintText, skipOffstage: false)).dx, 776.0);
    expect(tester.getBottomRight(find.text(hintText, skipOffstage: false)).dy, 350.0);
  });

  group('DropdownButtonFormField decoration hintText', () {
    const String decorationHintText = 'Decoration Hint text';
    const String hintText = 'Hint text';
    const String disabledHintText = 'Disabled Hint text';

    testWidgets('is the fallback value for DropdownButtonFormField.hint', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildFrame(
          isFormField: true,
          onChanged: (String? newValue) {},
          decoration: const InputDecoration(hintText: decorationHintText),
        ),
      );

      expect(find.text(decorationHintText, skipOffstage: false), findsOne);
    });

    testWidgets('does not override DropdownButtonFormField.hint', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildFrame(
          hint: const Text(hintText),
          isFormField: true,
          onChanged: (String? newValue) {},
          decoration: const InputDecoration(hintText: decorationHintText),
        ),
      );

      expect(find.text(hintText, skipOffstage: false), findsOne);
      expect(find.text(decorationHintText, skipOffstage: false), findsNothing);
    });

    testWidgets('is the fallback value for DropdownButtonFormField.disabledHint', (
      WidgetTester tester,
    ) async {
      // The Dropdown is disabled because onChanged is not defined.
      await tester.pumpWidget(
        buildFrame(
          isFormField: true,
          decoration: const InputDecoration(hintText: decorationHintText),
        ),
      );

      expect(find.text(decorationHintText, skipOffstage: false), findsOne);
    });

    testWidgets('does not override DropdownButtonFormField.disabledHint', (
      WidgetTester tester,
    ) async {
      // The Dropdown is disabled because onChanged is not defined.
      await tester.pumpWidget(
        buildFrame(
          disabledHint: const Text(disabledHintText),
          isFormField: true,
          decoration: const InputDecoration(hintText: decorationHintText),
        ),
      );

      expect(find.text(disabledHintText, skipOffstage: false), findsOne);
      expect(find.text(decorationHintText, skipOffstage: false), findsNothing);
    });

    testWidgets('is not used for disabledHint if DropdownButtonFormField.hint is provided', (
      WidgetTester tester,
    ) async {
      // The Dropdown is disabled because onChanged is not defined.
      await tester.pumpWidget(
        buildFrame(
          hint: const Text(hintText),
          isFormField: true,
          decoration: const InputDecoration(hintText: decorationHintText),
        ),
      );

      expect(find.text(hintText, skipOffstage: false), findsOne);
      expect(find.text(decorationHintText, skipOffstage: false), findsNothing);
    });
  });

  testWidgets('BorderRadius property clips dropdown menu', (WidgetTester tester) async {
    const double radius = 20.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownButtonFormField<String>(
              borderRadius: BorderRadius.circular(radius),
              value: 'One',
              items:
                  <String>['One', 'Two', 'Three', 'Four'].map<DropdownMenuItem<String>>((
                    String value,
                  ) {
                    return DropdownMenuItem<String>(value: value, child: Text(value));
                  }).toList(),
              onChanged: (_) {},
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('One'));
    await tester.pumpAndSettle();

    final RenderClipRRect renderClip = tester.allRenderObjects.whereType<RenderClipRRect>().first;
    expect(renderClip.borderRadius, BorderRadius.circular(radius));
  });

  testWidgets('Size of DropdownButton with padding', (WidgetTester tester) async {
    const double padVertical = 5;
    const double padHorizontal = 10;
    final Key buttonKey = UniqueKey();
    EdgeInsets? padding;

    Widget build() => buildFrame(buttonKey: buttonKey, onChanged: onChanged, padding: padding);

    await tester.pumpWidget(build());
    final RenderBox buttonBoxNoPadding = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBoxNoPadding.attached);
    final Size noPaddingSize = Size.copy(buttonBoxNoPadding.size);

    padding = const EdgeInsets.symmetric(vertical: padVertical, horizontal: padHorizontal);
    await tester.pumpWidget(build());
    final RenderBox buttonBoxPadded = tester.renderObject<RenderBox>(find.byKey(buttonKey));
    assert(buttonBoxPadded.attached);
    final Size paddedSize = Size.copy(buttonBoxPadded.size);

    // dropdowns with padding should be that much larger than with no padding
    expect(noPaddingSize.height, equals(paddedSize.height - padVertical * 2));
    expect(noPaddingSize.width, equals(paddedSize.width - padHorizontal * 2));
  });
}
