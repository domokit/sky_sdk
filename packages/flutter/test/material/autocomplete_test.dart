// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

class User {
  const User({required this.email, required this.name});

  final String email;
  final String name;

  @override
  String toString() {
    return '$name, $email';
  }
}

void main() {
  const List<String> kOptions = <String>[
    'aardvark',
    'bobcat',
    'chameleon',
    'dingo',
    'elephant',
    'flamingo',
    'goose',
    'hippopotamus',
    'iguana',
    'jaguar',
    'koala',
    'lemur',
    'mouse',
    'northern white rhinoceros',
  ];

  const List<User> kOptionsUsers = <User>[
    User(name: 'Alice', email: 'alice@example.com'),
    User(name: 'Bob', email: 'bob@example.com'),
    User(name: 'Charlie', email: 'charlie123@gmail.com'),
  ];

  testWidgets('can filter and select a list of string options', (WidgetTester tester) async {
    late String lastSelection;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<String>(
            onSelected: (String selection) {
              lastSelection = selection;
            },
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);

    // Focus the empty field. All the options are displayed.
    await tester.tap(find.byType(TextFormField));
    await tester.pump();
    expect(find.byType(ListView), findsOneWidget);
    ListView list = find.byType(ListView).evaluate().first.widget as ListView;
    expect(list.semanticChildCount, kOptions.length);

    // Enter text. The options are filtered by the text.
    await tester.enterText(find.byType(TextFormField), 'ele');
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsOneWidget);
    list = find.byType(ListView).evaluate().first.widget as ListView;
    // 'chameleon' and 'elephant' are displayed.
    expect(list.semanticChildCount, 2);

    // Select a option. The options hide and the field updates to show the
    // selection.
    await tester.tap(find.byType(InkWell).first);
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);
    final TextFormField field = find.byType(TextFormField).evaluate().first.widget as TextFormField;
    expect(field.controller!.text, 'chameleon');
    expect(lastSelection, 'chameleon');

    // Modify the field text. The options appear again and are filtered.
    await tester.enterText(find.byType(TextFormField), 'e');
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsOneWidget);
    list = find.byType(ListView).evaluate().first.widget as ListView;
    // 'chameleon', 'elephant', 'goose', 'lemur', 'mouse', and
    // 'northern white rhinoceros' are displayed.
    expect(list.semanticChildCount, 6);
  });

  testWidgets('can filter and select a list of custom User options', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<User>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptionsUsers.where((User option) {
                return option.toString().contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);

    // Focus the empty field. All the options are displayed.
    await tester.tap(find.byType(TextFormField));
    await tester.pump();
    expect(find.byType(ListView), findsOneWidget);
    ListView list = find.byType(ListView).evaluate().first.widget as ListView;
    expect(list.semanticChildCount, kOptionsUsers.length);

    // Enter text. The options are filtered by the text.
    await tester.enterText(find.byType(TextFormField), 'example');
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsOneWidget);
    list = find.byType(ListView).evaluate().first.widget as ListView;
    // 'Alice' and 'Bob' are displayed because they have "example.com" emails.
    expect(list.semanticChildCount, 2);

    // Select a option. The options hide and the field updates to show the
    // selection.
    await tester.tap(find.byType(InkWell).first);
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);
    final TextFormField field = find.byType(TextFormField).evaluate().first.widget as TextFormField;
    expect(field.controller!.text, 'Alice, alice@example.com');

    // Modify the field text. The options appear again and are filtered.
    await tester.enterText(find.byType(TextFormField), 'B');
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsOneWidget);
    list = find.byType(ListView).evaluate().first.widget as ListView;
    // 'Bob' is displayed.
    expect(list.semanticChildCount, 1);
  });

  testWidgets('displayStringForOption is displayed in the options', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<User>(
            displayStringForOption: (User option) {
              return option.name;
            },
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptionsUsers.where((User option) {
                return option.toString().contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);

    // Focus the empty field. All the options are displayed, and the string that
    // is used comes from displayStringForOption.
    await tester.tap(find.byType(TextFormField));
    await tester.pump();
    expect(find.byType(ListView), findsOneWidget);
    final ListView list = find.byType(ListView).evaluate().first.widget as ListView;
    expect(list.semanticChildCount, kOptionsUsers.length);
    for (int i = 0; i < kOptionsUsers.length; i++) {
      expect(find.text(kOptionsUsers[i].name), findsOneWidget);
    }

    // Select a option. The options hide and the field updates to show the
    // selection. The text in the field is given by displayStringForOption.
    await tester.tap(find.byType(InkWell).first);
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);
    final TextFormField field = find.byType(TextFormField).evaluate().first.widget as TextFormField;
    expect(field.controller!.text, kOptionsUsers.first.name);
  });

  testWidgets('can build a custom field', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController textEditingController,
              FocusNode focusNode,
              VoidCallback onFieldSubmitted,
            ) {
              return Container(key: fieldKey);
            },
          ),
        ),
      ),
    );

    // The custom field is rendered and not the default TextFormField.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byType(TextFormField), findsNothing);
  });

  testWidgets('can build custom options', (WidgetTester tester) async {
    final GlobalKey optionsKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // The default field is rendered but not the options, yet.
    expect(find.byKey(optionsKey), findsNothing);
    expect(find.byType(TextFormField), findsOneWidget);

    // Focus the empty field. The custom options is displayed.
    await tester.tap(find.byType(TextFormField));
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
  });

  testWidgets('the default Autocomplete options widget has a maximum height of 200', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    final Finder listFinder = find.byType(ListView);
    final Finder inputFinder = find.byType(TextFormField);
    await tester.tap(inputFinder);
    await tester.enterText(inputFinder, '');
    await tester.pump();
    final Size baseSize = tester.getSize(listFinder);
    final double resultingHeight = baseSize.height;
    expect(resultingHeight, equals(200));
  });

  testWidgets('the options height restricts to max desired height', (WidgetTester tester) async {
    const double desiredHeight = 150.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<String>(
            optionsMaxHeight: desiredHeight,
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    /// entering "a" returns 9 items from kOptions so basically the
    /// height of 9 options would be beyond `desiredHeight=150`,
    /// so height gets restricted to desiredHeight.
    final Finder listFinder = find.byType(ListView);
    final Finder inputFinder = find.byType(TextFormField);
    await tester.tap(inputFinder);
    await tester.enterText(inputFinder, 'a');
    await tester.pump();
    final Size baseSize = tester.getSize(listFinder);
    final double resultingHeight = baseSize.height;

    /// expected desired Height =150.0
    expect(resultingHeight, equals(desiredHeight));
  });

  testWidgets(
    'The height of options shrinks to height of resulting items, if less than maxHeight',
    (WidgetTester tester) async {
      // Returns a Future with the height of the default [Autocomplete] options widget
      // after the provided text had been entered into the [Autocomplete] field.
      Future<double> getDefaultOptionsHeight(WidgetTester tester, String enteredText) async {
        final Finder listFinder = find.byType(ListView);
        final Finder inputFinder = find.byType(TextFormField);
        final TextFormField field = inputFinder.evaluate().first.widget as TextFormField;
        field.controller!.clear();
        await tester.tap(inputFinder);
        await tester.enterText(inputFinder, enteredText);
        await tester.pump();
        final Size baseSize = tester.getSize(listFinder);
        return baseSize.height;
      }

      const double maxOptionsHeight = 250.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Autocomplete<String>(
              optionsMaxHeight: maxOptionsHeight,
              optionsBuilder: (TextEditingValue textEditingValue) {
                return kOptions.where((String option) {
                  return option.contains(textEditingValue.text.toLowerCase());
                });
              },
            ),
          ),
        ),
      );

      final Finder listFinder = find.byType(ListView);
      expect(listFinder, findsNothing);

      // Entering `a` returns 9 items(height > `maxOptionsHeight`) from the kOptions
      // so height gets restricted to `maxOptionsHeight =250`.
      final double nineItemsHeight = await getDefaultOptionsHeight(tester, 'a');
      expect(nineItemsHeight, equals(maxOptionsHeight));

      // Returns 2 Items (height < `maxOptionsHeight`)
      // so options height shrinks to 2 Items combined height.
      final double twoItemsHeight = await getDefaultOptionsHeight(tester, 'el');
      expect(twoItemsHeight, lessThan(maxOptionsHeight));

      // Returns 1 item (height < `maxOptionsHeight`) from `kOptions`
      // so options height shrinks to 1 items height.
      final double oneItemsHeight = await getDefaultOptionsHeight(tester, 'elep');
      expect(oneItemsHeight, lessThan(twoItemsHeight));
    },
  );

  testWidgets('initialValue sets initial text field value', (WidgetTester tester) async {
    late String lastSelection;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Autocomplete<String>(
            initialValue: const TextEditingValue(text: 'lem'),
            onSelected: (String selection) {
              lastSelection = selection;
            },
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);
    expect(tester.widget<TextFormField>(find.byType(TextFormField)).controller!.text, 'lem');

    // Focus the empty field. All the options are displayed.
    await tester.tap(find.byType(TextFormField));
    await tester.pump();
    expect(find.byType(ListView), findsOneWidget);
    final ListView list = find.byType(ListView).evaluate().first.widget as ListView;
    // Displays just one option ('lemur').
    expect(list.semanticChildCount, 1);

    // Select a option. The options hide and the field updates to show the
    // selection.
    await tester.tap(find.byType(InkWell).first);
    await tester.pump();
    expect(find.byType(TextFormField), findsOneWidget);
    expect(find.byType(ListView), findsNothing);
    final TextFormField field = find.byType(TextFormField).evaluate().first.widget as TextFormField;
    expect(field.controller!.text, 'lemur');
    expect(lastSelection, 'lemur');
  });

  // Ensures that the option with the given label has a given background color
  // if given, or no background if color is null.
  void checkOptionHighlight(WidgetTester tester, String label, Color? color) {
    final RenderBox renderBox = tester.renderObject<RenderBox>(
      find.ancestor(matching: find.byType(Container), of: find.text(label)),
    );
    if (color != null) {
      // Check to see that the container is painted with the highlighted background color.
      expect(renderBox, paints..rect(color: color));
    } else {
      // There should only be a paragraph painted.
      expect(renderBox, paintsExactlyCountTimes(const Symbol('drawRect'), 0));
      expect(renderBox, paints..paragraph());
    }
  }

  testWidgets('keyboard navigation of the options properly highlights the option', (
    WidgetTester tester,
  ) async {
    const Color highlightColor = Color(0xFF112233);
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light().copyWith(focusColor: highlightColor),
        home: Scaffold(
          body: Autocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byType(TextFormField));
    await tester.enterText(find.byType(TextFormField), 'el');
    await tester.pump();
    expect(find.byType(ListView), findsOneWidget);
    final ListView list = find.byType(ListView).evaluate().first.widget as ListView;
    expect(list.semanticChildCount, 2);

    // Initially the first option should be highlighted
    checkOptionHighlight(tester, 'chameleon', highlightColor);
    checkOptionHighlight(tester, 'elephant', null);

    // Move the selection down
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    // Highlight should be moved to the second item
    checkOptionHighlight(tester, 'chameleon', null);
    checkOptionHighlight(tester, 'elephant', highlightColor);
  });

  testWidgets('keyboard navigation keeps the highlighted option scrolled into view', (
    WidgetTester tester,
  ) async {
    const Color highlightColor = Color(0xFF112233);
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light().copyWith(focusColor: highlightColor),
        home: Scaffold(
          body: Autocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byType(TextFormField));
    await tester.enterText(find.byType(TextFormField), 'e');
    await tester.pump();
    expect(find.byType(ListView), findsOneWidget);
    final ListView list = find.byType(ListView).evaluate().first.widget as ListView;
    expect(list.semanticChildCount, 6);

    final Rect optionsGroupRect = tester.getRect(find.byType(ListView));
    const double optionsGroupPadding = 16.0;

    // Highlighted item should be at the top.
    checkOptionHighlight(tester, 'chameleon', highlightColor);
    expect(
      tester.getTopLeft(find.text('chameleon')).dy,
      equals(optionsGroupRect.top + optionsGroupPadding),
    );

    // Move down the list of options.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Select 'elephant'.
    await tester.pumpAndSettle();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Select 'goose'.
    await tester.pumpAndSettle();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Select 'lemur'.
    await tester.pumpAndSettle();

    // Highlighted item 'lemur' should be centered in the options popup.
    checkOptionHighlight(tester, 'lemur', highlightColor);
    expect(tester.getCenter(find.text('lemur')).dy, equals(optionsGroupRect.center.dy));

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown); // Select 'mouse'.
    await tester.pumpAndSettle();

    checkOptionHighlight(tester, 'mouse', highlightColor);

    // First item should have scrolled off the top, and not be selected.
    expect(find.text('chameleon'), findsNothing);

    // The other items on screen should not be selected.
    checkOptionHighlight(tester, 'goose', null);
    checkOptionHighlight(tester, 'lemur', null);
    checkOptionHighlight(tester, 'northern white rhinoceros', null);
  });

  group('optionsViewOpenDirection', () {
    testWidgets('default (down)', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Autocomplete<String>(
              optionsBuilder: (TextEditingValue textEditingValue) => <String>['a'],
            ),
          ),
        ),
      );
      final OptionsViewOpenDirection actual =
          tester
              .widget<RawAutocomplete<String>>(find.byType(RawAutocomplete<String>))
              .optionsViewOpenDirection;
      expect(actual, equals(OptionsViewOpenDirection.down));
    });

    testWidgets('down', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Autocomplete<String>(
              optionsViewOpenDirection:
                  OptionsViewOpenDirection.down, // ignore: avoid_redundant_argument_values
              optionsBuilder: (TextEditingValue textEditingValue) => <String>['a'],
            ),
          ),
        ),
      );
      final OptionsViewOpenDirection actual =
          tester
              .widget<RawAutocomplete<String>>(find.byType(RawAutocomplete<String>))
              .optionsViewOpenDirection;
      expect(actual, equals(OptionsViewOpenDirection.down));
    });

    testWidgets('up', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: Autocomplete<String>(
                optionsViewOpenDirection: OptionsViewOpenDirection.up,
                optionsBuilder: (TextEditingValue textEditingValue) => <String>['aa'],
              ),
            ),
          ),
        ),
      );
      final OptionsViewOpenDirection actual =
          tester
              .widget<RawAutocomplete<String>>(find.byType(RawAutocomplete<String>))
              .optionsViewOpenDirection;
      expect(actual, equals(OptionsViewOpenDirection.up));

      await tester.tap(find.byType(RawAutocomplete<String>));
      await tester.enterText(find.byType(RawAutocomplete<String>), 'a');
      expect(find.text('aa').hitTestable(), findsOneWidget);
    });
  });
}
