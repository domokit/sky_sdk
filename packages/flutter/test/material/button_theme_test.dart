// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('ButtonThemeData defaults', () {
    const ButtonThemeData theme = ButtonThemeData();
    expect(theme.textTheme, ButtonTextTheme.normal);
    expect(theme.constraints, const BoxConstraints(minWidth: 88.0, minHeight: 36.0));
    expect(theme.padding, const EdgeInsets.symmetric(horizontal: 16.0));
    expect(
      theme.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(2.0))),
    );
    expect(theme.alignedDropdown, false);
    expect(theme.layoutBehavior, ButtonBarLayoutBehavior.padded);
  });

  test('ButtonThemeData default overrides', () {
    const ButtonThemeData theme = ButtonThemeData(
      textTheme: ButtonTextTheme.primary,
      minWidth: 100.0,
      height: 200.0,
      padding: EdgeInsets.zero,
      shape: RoundedRectangleBorder(),
      alignedDropdown: true,
    );
    expect(theme.textTheme, ButtonTextTheme.primary);
    expect(theme.constraints, const BoxConstraints(minWidth: 100.0, minHeight: 200.0));
    expect(theme.padding, EdgeInsets.zero);
    expect(theme.shape, const RoundedRectangleBorder());
    expect(theme.alignedDropdown, true);
  });

  test('ButtonThemeData.copyWith', () {
    ButtonThemeData theme = const ButtonThemeData().copyWith();
    expect(theme.textTheme, ButtonTextTheme.normal);
    expect(theme.layoutBehavior, ButtonBarLayoutBehavior.padded);
    expect(theme.constraints, const BoxConstraints(minWidth: 88.0, minHeight: 36.0));
    expect(theme.padding, const EdgeInsets.symmetric(horizontal: 16.0));
    expect(
      theme.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(2.0))),
    );
    expect(theme.alignedDropdown, false);
    expect(theme.colorScheme, null);

    theme = const ButtonThemeData().copyWith(
      textTheme: ButtonTextTheme.primary,
      layoutBehavior: ButtonBarLayoutBehavior.constrained,
      minWidth: 100.0,
      height: 200.0,
      padding: EdgeInsets.zero,
      shape: const StadiumBorder(),
      alignedDropdown: true,
      colorScheme: const ColorScheme.dark(),
      materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
    );
    expect(theme.textTheme, ButtonTextTheme.primary);
    expect(theme.layoutBehavior, ButtonBarLayoutBehavior.constrained);
    expect(theme.constraints, const BoxConstraints(minWidth: 100.0, minHeight: 200.0));
    expect(theme.padding, EdgeInsets.zero);
    expect(theme.shape, const StadiumBorder());
    expect(theme.alignedDropdown, true);
    expect(theme.colorScheme, const ColorScheme.dark());
  });

  testWidgets('ButtonTheme alignedDropdown', (WidgetTester tester) async {
    final Key dropdownKey = UniqueKey();

    Widget buildFrame({required bool alignedDropdown, required TextDirection textDirection}) {
      return MaterialApp(
        builder: (BuildContext context, Widget? child) {
          return Directionality(textDirection: textDirection, child: child!);
        },
        home: ButtonTheme(
          alignedDropdown: alignedDropdown,
          child: Material(
            child: Builder(
              builder: (BuildContext context) {
                return Container(
                  alignment: Alignment.center,
                  child: DropdownButtonHideUnderline(
                    child: SizedBox(
                      width: 200.0,
                      child: DropdownButton<String>(
                        key: dropdownKey,
                        onChanged: (String? value) {},
                        value: 'foo',
                        items: const <DropdownMenuItem<String>>[
                          DropdownMenuItem<String>(value: 'foo', child: Text('foo')),
                          DropdownMenuItem<String>(value: 'bar', child: Text('bar')),
                        ],
                      ),
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );
    }

    final Finder button = find.byKey(dropdownKey);
    final Finder menu = find.byWidgetPredicate(
      (Widget w) => '${w.runtimeType}' == '_DropdownMenu<String>',
    );

    await tester.pumpWidget(buildFrame(alignedDropdown: false, textDirection: TextDirection.ltr));
    await tester.tap(button);
    await tester.pumpAndSettle();

    // 240 = 200.0 (button width) + _kUnalignedMenuMargin (20.0 left and right)
    expect(tester.getSize(button).width, 200.0);
    expect(tester.getSize(menu).width, 240.0);

    // Dismiss the menu.
    await tester.tapAt(Offset.zero);
    await tester.pumpAndSettle();
    expect(menu, findsNothing);

    await tester.pumpWidget(buildFrame(alignedDropdown: true, textDirection: TextDirection.ltr));
    await tester.tap(button);
    await tester.pumpAndSettle();

    // Aligneddropdown: true means the button and menu widths match
    expect(tester.getSize(button).width, 200.0);
    expect(tester.getSize(menu).width, 200.0);

    // There are two 'foo' widgets: the selected menu item's label and the drop
    // down button's label. The should both appear at the same location.
    final Finder fooText = find.text('foo');
    expect(fooText, findsNWidgets(2));
    expect(tester.getRect(fooText.at(0)), tester.getRect(fooText.at(1)));

    // Dismiss the menu.
    await tester.tapAt(Offset.zero);
    await tester.pumpAndSettle();
    expect(menu, findsNothing);

    // Same test as above except RTL
    await tester.pumpWidget(buildFrame(alignedDropdown: true, textDirection: TextDirection.rtl));
    await tester.tap(button);
    await tester.pumpAndSettle();

    expect(fooText, findsNWidgets(2));
    expect(tester.getRect(fooText.at(0)), tester.getRect(fooText.at(1)));
  });
}
