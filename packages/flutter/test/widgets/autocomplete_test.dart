// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
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
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<String> lastOptions;
    late AutocompleteOnSelected<String> lastOnSelected;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              lastOnSelected = onSelected;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Focus the empty field. All the options are displayed.
    focusNode.requestFocus();
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, kOptions.length);

    // Enter text. The options are filtered by the text.
    textEditingController.value = const TextEditingValue(
      text: 'ele',
      selection: TextSelection(baseOffset: 3, extentOffset: 3),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');

    // Select an option. The options hide and the field updates to show the
    // selection.
    final String selection = lastOptions.elementAt(1);
    lastOnSelected(selection);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(textEditingController.text, selection);

    // Modify the field text. The options appear again and are filtered.
    textEditingController.value = const TextEditingValue(
      text: 'e',
      selection: TextSelection(baseOffset: 1, extentOffset: 1),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 6);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');
    expect(lastOptions.elementAt(2), 'goose');
    expect(lastOptions.elementAt(3), 'lemur');
    expect(lastOptions.elementAt(4), 'mouse');
    expect(lastOptions.elementAt(5), 'northern white rhinoceros');
  });

  testWidgets('can split the field and options', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<String> lastOptions;
    late AutocompleteOnSelected<String> lastOnSelected;

    final GlobalKey autocompleteKey = GlobalKey();
    final TextEditingController textEditingController = TextEditingController();
    final FocusNode focusNode = FocusNode();
    addTearDown(textEditingController.dispose);
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            // The field is in the AppBar, not actually a child of RawAutocomplete.
            title: TextFormField(
              key: fieldKey,
              controller: textEditingController,
              focusNode: focusNode,
              decoration: const InputDecoration(hintText: 'Split RawAutocomplete App'),
              onFieldSubmitted: (String value) {
                RawAutocomplete.onFieldSubmitted<String>(autocompleteKey);
              },
            ),
          ),
          body: Align(
            alignment: Alignment.topLeft,
            child: RawAutocomplete<String>(
              key: autocompleteKey,
              focusNode: focusNode,
              textEditingController: textEditingController,
              optionsBuilder: (TextEditingValue textEditingValue) {
                return kOptions.where((String option) {
                  return option.contains(textEditingValue.text.toLowerCase());
                }).toList();
              },
              optionsViewBuilder: (
                BuildContext context,
                AutocompleteOnSelected<String> onSelected,
                Iterable<String> options,
              ) {
                lastOptions = options;
                lastOnSelected = onSelected;
                return Material(
                  key: optionsKey,
                  elevation: 4.0,
                  child: ListView(
                    children:
                        options
                            .map(
                              (String option) => GestureDetector(
                                onTap: () {
                                  onSelected(option);
                                },
                                child: ListTile(title: Text(option)),
                              ),
                            )
                            .toList(),
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Focus the empty field. All the options are displayed.
    focusNode.requestFocus();
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, kOptions.length);
    expect(tester.getSize(find.byKey(optionsKey)).width, greaterThan(0.0));

    // Enter text. The options are filtered by the text.
    textEditingController.value = const TextEditingValue(
      text: 'ele',
      selection: TextSelection(baseOffset: 3, extentOffset: 3),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');

    // Select an option. The options hide and the field updates to show the
    // selection.
    final String selection = lastOptions.elementAt(1);
    lastOnSelected(selection);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(textEditingController.text, selection);

    // Modify the field text. The options appear again and are filtered.
    textEditingController.value = const TextEditingValue(
      text: 'e',
      selection: TextSelection(baseOffset: 1, extentOffset: 1),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 6);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');
    expect(lastOptions.elementAt(2), 'goose');
    expect(lastOptions.elementAt(3), 'lemur');
    expect(lastOptions.elementAt(4), 'mouse');
    expect(lastOptions.elementAt(5), 'northern white rhinoceros');
  });

  for (final OptionsViewOpenDirection openDirection in OptionsViewOpenDirection.values) {
    testWidgets('tapping on an option selects it ($openDirection)', (WidgetTester tester) async {
      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();
      late Iterable<String> lastOptions;
      late FocusNode focusNode;
      late TextEditingController textEditingController;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Column(
              children: <Widget>[
                const SizedBox(height: 200),
                RawAutocomplete<String>(
                  optionsViewOpenDirection: openDirection,
                  optionsBuilder: (TextEditingValue textEditingValue) {
                    return kOptions.where((String option) {
                      return option.contains(textEditingValue.text.toLowerCase());
                    });
                  },
                  fieldViewBuilder: (
                    BuildContext context,
                    TextEditingController fieldTextEditingController,
                    FocusNode fieldFocusNode,
                    VoidCallback onFieldSubmitted,
                  ) {
                    focusNode = fieldFocusNode;
                    textEditingController = fieldTextEditingController;
                    return TextField(
                      key: fieldKey,
                      focusNode: focusNode,
                      controller: textEditingController,
                    );
                  },
                  optionsViewBuilder: (
                    BuildContext context,
                    AutocompleteOnSelected<String> onSelected,
                    Iterable<String> options,
                  ) {
                    lastOptions = options;
                    return Material(
                      elevation: 4.0,
                      child: ListView.builder(
                        key: optionsKey,
                        padding: const EdgeInsets.all(8.0),
                        itemCount: options.length,
                        itemBuilder: (BuildContext context, int index) {
                          final String option = options.elementAt(index);
                          return GestureDetector(
                            onTap: () {
                              onSelected(option);
                            },
                            child: ListTile(title: Text(option)),
                          );
                        },
                      ),
                    );
                  },
                ),
              ],
            ),
          ),
        ),
      );

      // The field is always rendered, but the options are not unless needed.
      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      // Tap on the text field to open the options.
      await tester.tap(find.byKey(fieldKey));
      // Two pumps required due to post frame callback.
      await tester.pump();
      await tester.pump();
      expect(find.byKey(optionsKey), findsOneWidget);
      expect(lastOptions.length, kOptions.length);

      await tester.tap(find.text(kOptions[2]));
      await tester.pump();

      expect(find.byKey(optionsKey), findsNothing);

      expect(textEditingController.text, equals(kOptions[2]));
    });

    testWidgets('when not enough room for options, options cover field ($openDirection)', (
      WidgetTester tester,
    ) async {
      const double padding = 32.0;
      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();
      late StateSetter setState;
      Alignment alignment = Alignment.bottomCenter;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: StatefulBuilder(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                return Padding(
                  padding: const EdgeInsets.symmetric(horizontal: padding),
                  child: Align(
                    alignment: alignment,
                    child: RawAutocomplete<String>(
                      optionsViewOpenDirection: openDirection,
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
                        return ListView.builder(
                          key: optionsKey,
                          padding: EdgeInsets.zero,
                          shrinkWrap: true,
                          itemCount: options.length,
                          itemBuilder: (BuildContext context, int index) {
                            final String option = options.elementAt(index);
                            return InkWell(
                              onTap: () {
                                onSelected(option);
                              },
                              child: Padding(
                                padding: const EdgeInsets.all(16.0),
                                child: Text(option),
                              ),
                            );
                          },
                        );
                      },
                      fieldViewBuilder: (
                        BuildContext context,
                        TextEditingController textEditingController,
                        FocusNode focusNode,
                        VoidCallback onSubmitted,
                      ) {
                        return TextField(
                          key: fieldKey,
                          focusNode: focusNode,
                          controller: textEditingController,
                        );
                      },
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byKey(fieldKey));
      await tester.pump();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      await tester.enterText(find.byKey(fieldKey), 'go'); // 3 results.
      await tester.pump();

      switch (openDirection) {
        case OptionsViewOpenDirection.up:
          // Options are positioned and sized like normal.
          expect(find.byType(InkWell), findsNWidgets(3));
          final double optionHeight = tester.getSize(find.byType(InkWell).first).height;
          final double topOfField = tester.getTopLeft(find.byKey(fieldKey)).dy;
          expect(
            tester.getTopLeft(find.byType(InkWell).first),
            Offset(padding, topOfField - 3 * optionHeight),
          );
          expect(tester.getBottomLeft(find.byType(InkWell).at(2)), Offset(padding, topOfField));
        case OptionsViewOpenDirection.down:
          expect(find.byType(InkWell), findsNWidgets(1));
          final Size optionsSize = tester.getSize(find.byKey(optionsKey));
          expect(optionsSize.height, kMinInteractiveDimension);
          // Options are positioned as low as possible while still fitting on screen.
          final double bottomOfField = tester.getBottomLeft(find.byKey(optionsKey)).dy;
          expect(
            tester.getTopLeft(find.byKey(optionsKey)),
            Offset(padding, bottomOfField - optionsSize.height),
          );
      }

      // Add an extra pump to account for any potential frame delays introduced
      // by the post frame callback in the _RawAutocompleteOptions
      // implementation.
      await tester.pump();
      setState(() {
        alignment = Alignment.topCenter;
      });

      // One frame for the field to move and one frame for the options to
      // follow.
      await tester.pump();
      await tester.pump();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      switch (openDirection) {
        case OptionsViewOpenDirection.up:
          // Options are positioned as high as possible while still fitting on
          // the screen.
          expect(find.byType(InkWell), findsNWidgets(1));
          final Size optionsSize = tester.getSize(find.byKey(optionsKey));
          expect(optionsSize.height, kMinInteractiveDimension);
          expect(tester.getTopLeft(find.byKey(optionsKey)), const Offset(padding, 0.0));
          expect(tester.getBottomLeft(find.byKey(optionsKey)), Offset(padding, optionsSize.height));
        case OptionsViewOpenDirection.down:
          // Options are positioned and sized like normal.
          expect(find.byType(InkWell), findsNWidgets(3));
          final double optionHeight = tester.getSize(find.byType(InkWell).first).height;
          final double bottomOfField = tester.getBottomLeft(find.byKey(fieldKey)).dy;
          expect(tester.getTopLeft(find.byType(InkWell).first), Offset(padding, bottomOfField));
          expect(
            tester.getBottomLeft(find.byType(InkWell).at(2)),
            Offset(padding, bottomOfField + 3 * optionHeight),
          );
      }
    });

    testWidgets('correct options alignment for RTL in direction $openDirection', (
      WidgetTester tester,
    ) async {
      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();
      const double kOptionsWidth = 100.0;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Directionality(
              textDirection: TextDirection.rtl,
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 32.0),
                child: RawAutocomplete<String>(
                  optionsViewOpenDirection: openDirection,
                  optionsBuilder: (TextEditingValue textEditingValue) {
                    return kOptions.where((String option) {
                      return option.contains(textEditingValue.text.toLowerCase());
                    });
                  },
                  fieldViewBuilder: (
                    BuildContext context,
                    TextEditingController textEditingController,
                    FocusNode focusNode,
                    VoidCallback onSubmitted,
                  ) {
                    return TextField(
                      key: fieldKey,
                      focusNode: focusNode,
                      controller: textEditingController,
                    );
                  },
                  optionsViewBuilder: (
                    BuildContext context,
                    AutocompleteOnSelected<String> onSelected,
                    Iterable<String> options,
                  ) {
                    return SizedBox(width: kOptionsWidth, key: optionsKey);
                  },
                ),
              ),
            ),
          ),
        ),
      );

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byType(TextField));
      await tester.pump();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      final RenderBox optionsBox = tester.renderObject(find.byKey(optionsKey));
      expect(optionsBox.size.width, kOptionsWidth);
      expect(
        tester.getTopRight(find.byKey(optionsKey)).dx,
        tester.getTopRight(find.byKey(fieldKey)).dx,
      );
    });

    testWidgets('options width matches field width with open direction $openDirection', (
      WidgetTester tester,
    ) async {
      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 32.0),
              child: Center(
                child: RawAutocomplete<String>(
                  optionsViewOpenDirection: openDirection,
                  optionsBuilder: (TextEditingValue textEditingValue) {
                    return kOptions.where((String option) {
                      return option.contains(textEditingValue.text.toLowerCase());
                    });
                  },
                  fieldViewBuilder: (
                    BuildContext context,
                    TextEditingController textEditingController,
                    FocusNode focusNode,
                    VoidCallback onSubmitted,
                  ) {
                    return TextField(
                      key: fieldKey,
                      focusNode: focusNode,
                      controller: textEditingController,
                    );
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
          ),
        ),
      );

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byType(TextField));
      // Two pumps required due to post frame callback.
      await tester.pump();
      await tester.pump();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      final RenderBox fieldBox = tester.renderObject(find.byKey(fieldKey));
      final RenderBox optionsBox = tester.renderObject(find.byKey(optionsKey));
      expect(optionsBox.size.width, equals(fieldBox.size.width));
      expect(tester.getTopLeft(find.byKey(optionsKey)).dy, switch (openDirection) {
        OptionsViewOpenDirection.down =>
          tester.getTopLeft(find.byKey(fieldKey)).dy + fieldBox.size.height,
        OptionsViewOpenDirection.up =>
          tester.getTopLeft(find.byKey(fieldKey)).dy - optionsBox.size.height,
      });
    });
  }

  testWidgets('can filter and select a list of custom User options', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<User> lastOptions;
    late AutocompleteOnSelected<User> lastOnSelected;
    late User lastUserSelected;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<User>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptionsUsers.where((User option) {
                return option.toString().contains(textEditingValue.text.toLowerCase());
              });
            },
            onSelected: (User selected) {
              lastUserSelected = selected;
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: fieldTextEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<User> onSelected,
              Iterable<User> options,
            ) {
              lastOptions = options;
              lastOnSelected = onSelected;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Enter text. The options are filtered by the text.
    focusNode.requestFocus();
    textEditingController.value = const TextEditingValue(
      text: 'example',
      selection: TextSelection(baseOffset: 7, extentOffset: 7),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), kOptionsUsers[0]);
    expect(lastOptions.elementAt(1), kOptionsUsers[1]);

    // Select an option. The options hide and onSelected is called.
    final User selection = lastOptions.elementAt(1);
    lastOnSelected(selection);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(lastUserSelected, selection);
    expect(textEditingController.text, selection.toString());

    // Modify the field text. The options appear again and are filtered, this
    // time by name instead of email.
    textEditingController.value = const TextEditingValue(
      text: 'B',
      selection: TextSelection(baseOffset: 1, extentOffset: 1),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 1);
    expect(lastOptions.elementAt(0), kOptionsUsers[1]);
  });

  testWidgets('can specify a custom display string for a list of custom User options', (
    WidgetTester tester,
  ) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<User> lastOptions;
    late AutocompleteOnSelected<User> lastOnSelected;
    late User lastUserSelected;
    String displayStringForOption(User option) => option.name;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<User>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptionsUsers.where((User option) {
                return option.toString().contains(textEditingValue.text.toLowerCase());
              });
            },
            displayStringForOption: displayStringForOption,
            onSelected: (User selected) {
              lastUserSelected = selected;
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              textEditingController = fieldTextEditingController;
              focusNode = fieldFocusNode;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: fieldTextEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<User> onSelected,
              Iterable<User> options,
            ) {
              lastOptions = options;
              lastOnSelected = onSelected;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Enter text. The options are filtered by the text.
    focusNode.requestFocus();
    textEditingController.value = const TextEditingValue(
      text: 'example',
      selection: TextSelection(baseOffset: 7, extentOffset: 7),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), kOptionsUsers[0]);
    expect(lastOptions.elementAt(1), kOptionsUsers[1]);

    // Select an option. The options hide and onSelected is called. The field
    // has its text set to the selection's display string.
    final User selection = lastOptions.elementAt(1);
    lastOnSelected(selection);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(lastUserSelected, selection);
    expect(textEditingController.text, selection.name);

    // Modify the field text. The options appear again and are filtered, this
    // time by name instead of email.
    textEditingController.value = const TextEditingValue(
      text: 'B',
      selection: TextSelection(baseOffset: 1, extentOffset: 1),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 1);
    expect(lastOptions.elementAt(0), kOptionsUsers[1]);
  });

  testWidgets('onFieldSubmitted selects the first option', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<String> lastOptions;
    late VoidCallback lastOnFieldSubmitted;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              textEditingController = fieldTextEditingController;
              focusNode = fieldFocusNode;
              lastOnFieldSubmitted = onFieldSubmitted;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: fieldTextEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Enter text. The options are filtered by the text.
    focusNode.requestFocus();
    textEditingController.value = const TextEditingValue(
      text: 'ele',
      selection: TextSelection(baseOffset: 3, extentOffset: 3),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');

    // Select the current string, as if the field was submitted. The options
    // hide and the field updates to show the selection.
    lastOnFieldSubmitted();
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(textEditingController.text, lastOptions.elementAt(0));
  });

  group('optionsViewOpenDirection', () {
    testWidgets('unset (default behavior): open downward', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: RawAutocomplete<String>(
                optionsBuilder: (TextEditingValue textEditingValue) => <String>['a'],
                fieldViewBuilder: (
                  BuildContext context,
                  TextEditingController controller,
                  FocusNode focusNode,
                  VoidCallback onFieldSubmitted,
                ) {
                  return TextField(controller: controller, focusNode: focusNode);
                },
                optionsViewBuilder: (
                  BuildContext context,
                  AutocompleteOnSelected<String> onSelected,
                  Iterable<String> options,
                ) {
                  return const Text('a');
                },
              ),
            ),
          ),
        ),
      );
      await tester.showKeyboard(find.byType(TextField));
      await tester.pump();
      expect(
        tester.getBottomLeft(find.byType(TextField)),
        offsetMoreOrLessEquals(tester.getTopLeft(find.text('a'))),
      );
    });

    testWidgets('down: open downward', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: RawAutocomplete<String>(
                optionsViewOpenDirection:
                    OptionsViewOpenDirection.down, // ignore: avoid_redundant_argument_values
                optionsBuilder: (TextEditingValue textEditingValue) => <String>['a'],
                fieldViewBuilder: (
                  BuildContext context,
                  TextEditingController controller,
                  FocusNode focusNode,
                  VoidCallback onFieldSubmitted,
                ) {
                  return TextField(controller: controller, focusNode: focusNode);
                },
                optionsViewBuilder: (
                  BuildContext context,
                  AutocompleteOnSelected<String> onSelected,
                  Iterable<String> options,
                ) {
                  return const Text('a');
                },
              ),
            ),
          ),
        ),
      );
      await tester.showKeyboard(find.byType(TextField));
      await tester.pump();
      expect(
        tester.getBottomLeft(find.byType(TextField)),
        offsetMoreOrLessEquals(tester.getTopLeft(find.text('a'))),
      );
    });

    testWidgets('up: open upward', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: RawAutocomplete<String>(
                optionsViewOpenDirection: OptionsViewOpenDirection.up,
                optionsBuilder: (TextEditingValue textEditingValue) => <String>['a'],
                fieldViewBuilder: (
                  BuildContext context,
                  TextEditingController controller,
                  FocusNode focusNode,
                  VoidCallback onFieldSubmitted,
                ) {
                  return TextField(controller: controller, focusNode: focusNode);
                },
                optionsViewBuilder: (
                  BuildContext context,
                  AutocompleteOnSelected<String> onSelected,
                  Iterable<String> options,
                ) {
                  return const Text('a');
                },
              ),
            ),
          ),
        ),
      );
      await tester.showKeyboard(find.byType(TextField));
      await tester.pump();
      expect(
        tester.getTopLeft(find.byType(TextField)),
        offsetMoreOrLessEquals(tester.getBottomLeft(find.text('a'))),
      );
    });

    group('fieldViewBuilder not passed', () {
      testWidgets('down', (WidgetTester tester) async {
        final GlobalKey autocompleteKey = GlobalKey();
        final TextEditingController controller = TextEditingController();
        addTearDown(controller.dispose);
        final FocusNode focusNode = FocusNode();
        addTearDown(focusNode.dispose);
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  TextField(controller: controller, focusNode: focusNode),
                  RawAutocomplete<String>(
                    key: autocompleteKey,
                    textEditingController: controller,
                    focusNode: focusNode,
                    optionsViewOpenDirection:
                        OptionsViewOpenDirection.down, // ignore: avoid_redundant_argument_values
                    optionsBuilder: (TextEditingValue textEditingValue) => <String>['a'],
                    optionsViewBuilder: (
                      BuildContext context,
                      AutocompleteOnSelected<String> onSelected,
                      Iterable<String> options,
                    ) {
                      return const Text('a');
                    },
                  ),
                ],
              ),
            ),
          ),
        );
        await tester.showKeyboard(find.byType(TextField));
        await tester.pump();
        expect(
          tester.getBottomLeft(find.byKey(autocompleteKey)),
          offsetMoreOrLessEquals(tester.getTopLeft(find.text('a'))),
        );
      });

      testWidgets('up', (WidgetTester tester) async {
        final GlobalKey autocompleteKey = GlobalKey();
        final TextEditingController controller = TextEditingController();
        addTearDown(controller.dispose);
        final FocusNode focusNode = FocusNode();
        addTearDown(focusNode.dispose);
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Column(
                mainAxisAlignment: MainAxisAlignment.end,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  RawAutocomplete<String>(
                    key: autocompleteKey,
                    textEditingController: controller,
                    focusNode: focusNode,
                    optionsViewOpenDirection: OptionsViewOpenDirection.up,
                    optionsBuilder: (TextEditingValue textEditingValue) => <String>['a'],
                    optionsViewBuilder: (
                      BuildContext context,
                      AutocompleteOnSelected<String> onSelected,
                      Iterable<String> options,
                    ) {
                      return const Text('a');
                    },
                  ),
                  TextField(controller: controller, focusNode: focusNode),
                ],
              ),
            ),
          ),
        );
        await tester.showKeyboard(find.byType(TextField));
        await tester.pump();
        expect(
          tester.getTopLeft(find.byKey(autocompleteKey)),
          offsetMoreOrLessEquals(tester.getBottomLeft(find.text('a'))),
        );
      });
    });
  });

  testWidgets('options follow field when it moves', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late StateSetter setState;
    Alignment alignment = Alignment.center;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return Align(
                alignment: alignment,
                child: RawAutocomplete<String>(
                  optionsBuilder: (TextEditingValue textEditingValue) {
                    return kOptions.where((String option) {
                      return option.contains(textEditingValue.text.toLowerCase());
                    });
                  },
                  fieldViewBuilder: (
                    BuildContext context,
                    TextEditingController fieldTextEditingController,
                    FocusNode fieldFocusNode,
                    VoidCallback onFieldSubmitted,
                  ) {
                    focusNode = fieldFocusNode;
                    textEditingController = fieldTextEditingController;
                    return TextFormField(
                      controller: fieldTextEditingController,
                      focusNode: focusNode,
                      key: fieldKey,
                    );
                  },
                  optionsViewBuilder: (
                    BuildContext context,
                    AutocompleteOnSelected<String> onSelected,
                    Iterable<String> options,
                  ) {
                    return Container(key: optionsKey);
                  },
                ),
              );
            },
          ),
        ),
      ),
    );

    // Field is shown but not options.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Enter text to show the options.
    focusNode.requestFocus();
    textEditingController.value = const TextEditingValue(
      text: 'ele',
      selection: TextSelection(baseOffset: 3, extentOffset: 3),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);

    // Options are just below the field.
    final Offset optionsTopLeft = tester.getTopLeft(find.byKey(optionsKey));
    final Offset fieldOffset = tester.getTopLeft(find.byKey(fieldKey));
    final Size fieldSize = tester.getSize(find.byKey(fieldKey));
    expect(optionsTopLeft.dy, fieldOffset.dy + fieldSize.height);

    // Add an extra pump to account for any potential frame delays introduced by
    // the post frame callback in the _RawAutocompleteOptions implementation.
    await tester.pump();

    // Move the field (similar to as if the keyboard opened). The options move
    // to follow the field.
    setState(() {
      alignment = Alignment.topCenter;
    });
    await tester.pump();
    final Offset fieldOffsetFrame1 = tester.getTopLeft(find.byKey(fieldKey));
    final Offset optionsTopLeftOpenFrame1 = tester.getTopLeft(find.byKey(optionsKey));

    expect(fieldOffsetFrame1.dy, lessThan(fieldOffset.dy));
    expect(optionsTopLeftOpenFrame1.dy, isNot(equals(optionsTopLeft.dy)));
    expect(optionsTopLeftOpenFrame1.dy, fieldOffsetFrame1.dy + fieldSize.height);
  });

  testWidgets('options are shown one frame after tapping in field', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.topCenter,
            child: RawAutocomplete<String>(
              optionsBuilder: (TextEditingValue textEditingValue) {
                return kOptions.where((String option) {
                  return option.contains(textEditingValue.text.toLowerCase());
                });
              },
              fieldViewBuilder: (
                BuildContext context,
                TextEditingController fieldTextEditingController,
                FocusNode fieldFocusNode,
                VoidCallback onFieldSubmitted,
              ) {
                return TextFormField(
                  controller: fieldTextEditingController,
                  focusNode: fieldFocusNode,
                  key: fieldKey,
                );
              },
              optionsViewBuilder: (
                BuildContext context,
                AutocompleteOnSelected<String> onSelected,
                Iterable<String> options,
              ) {
                return ListView(
                  key: optionsKey,
                  children: options.map((String option) => Text(option)).toList(),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Field is shown but not options.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(find.text('aardvark'), findsNothing);

    // Tap to show the options.
    await tester.tap(find.byKey(fieldKey));
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(find.text('aardvark'), findsOneWidget);
  });

  testWidgets('can prevent options from showing by returning an empty iterable', (
    WidgetTester tester,
  ) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<String> lastOptions;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              if (textEditingValue.text == '') {
                return const Iterable<String>.empty();
              }
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: fieldTextEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Focus the empty field. The options are not displayed because
    // optionsBuilder returns nothing for an empty field query.
    focusNode.requestFocus();
    textEditingController.value = const TextEditingValue(
      selection: TextSelection(baseOffset: 0, extentOffset: 0),
    );
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);

    // Enter text. Now the options appear, filtered by the text.
    textEditingController.value = const TextEditingValue(
      text: 'ele',
      selection: TextSelection(baseOffset: 3, extentOffset: 3),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');
  });

  testWidgets('can create a field outside of fieldViewBuilder', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    final GlobalKey autocompleteKey = GlobalKey();
    late Iterable<String> lastOptions;
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final TextEditingController textEditingController = TextEditingController();
    addTearDown(textEditingController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            // This is where the real field is being built.
            title: TextFormField(
              key: fieldKey,
              controller: textEditingController,
              focusNode: focusNode,
              onFieldSubmitted: (String value) {
                RawAutocomplete.onFieldSubmitted(autocompleteKey);
              },
            ),
          ),
          body: RawAutocomplete<String>(
            key: autocompleteKey,
            focusNode: focusNode,
            textEditingController: textEditingController,
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
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Enter text. The options are filtered by the text.
    focusNode.requestFocus();
    textEditingController.value = const TextEditingValue(
      text: 'ele',
      selection: TextSelection(baseOffset: 3, extentOffset: 3),
    );
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');

    // Submit the field. The options hide and the field updates to show the
    // selection.
    await tester.showKeyboard(find.byType(TextFormField));
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(textEditingController.text, lastOptions.elementAt(0));
  });

  testWidgets('initialValue sets initial text field value', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<String> lastOptions;
    late AutocompleteOnSelected<String> lastOnSelected;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            // Should initialize text field with 'lem'.
            initialValue: const TextEditingValue(text: 'lem'),
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              lastOnSelected = onSelected;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    // The text editing controller value starts off with initialized value.
    expect(textEditingController.text, 'lem');

    // Focus the empty field. All the options are displayed.
    focusNode.requestFocus();
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.elementAt(0), 'lemur');

    // Select an option. The options hide and the field updates to show the
    // selection.
    final String selection = lastOptions.elementAt(0);
    lastOnSelected(selection);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(textEditingController.text, selection);
  });

  testWidgets('initialValue cannot be defined if TextEditingController is defined', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final TextEditingController textEditingController = TextEditingController();
    addTearDown(textEditingController.dispose);

    expect(() {
      RawAutocomplete<String>(
        focusNode: focusNode,
        // Both [initialValue] and [textEditingController] cannot be
        // simultaneously defined.
        initialValue: const TextEditingValue(text: 'lemur'),
        textEditingController: textEditingController,
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
          return Container();
        },
        fieldViewBuilder: (
          BuildContext context,
          TextEditingController fieldTextEditingController,
          FocusNode fieldFocusNode,
          VoidCallback onFieldSubmitted,
        ) {
          return TextField(focusNode: focusNode, controller: textEditingController);
        },
      );
    }, throwsAssertionError);
  });

  testWidgets('support asynchronous options builder', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late FocusNode focusNode;
    late TextEditingController textEditingController;
    Iterable<String>? lastOptions;
    Duration? delay;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) async {
              final Iterable<String> options = kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
              if (delay == null) {
                return options;
              }
              return Future<Iterable<String>>.delayed(delay, () => options);
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // Enter text to build the options with delay.
    focusNode.requestFocus();
    delay = const Duration(milliseconds: 500);
    await tester.enterText(find.byKey(fieldKey), 'go');
    await tester.pumpAndSettle();

    // The options have not yet been built.
    expect(find.byKey(optionsKey), findsNothing);
    expect(lastOptions, isNull);

    // Await asynchronous options builder.
    await tester.pumpAndSettle(delay);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions, <String>['dingo', 'flamingo', 'goose']);

    // Enter text to rebuild the options without delay.
    delay = null;
    await tester.enterText(find.byKey(fieldKey), 'ngo');
    await tester.pump();
    expect(lastOptions, <String>['dingo', 'flamingo']);
  });

  testWidgets('can navigate options with the keyboard', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<String> lastOptions;
    late FocusNode focusNode;
    late TextEditingController textEditingController;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextFormField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
                onFieldSubmitted: (String value) {
                  onFieldSubmitted();
                },
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // Enter text. The options are filtered by the text.
    focusNode.requestFocus();
    await tester.enterText(find.byKey(fieldKey), 'ele');
    await tester.pumpAndSettle();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');

    // Move the highlighted option to the second item 'elephant' and select it
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    // Can't use the key event for enter to submit to the text field using
    // the test framework, so this appears to be the equivalent.
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(textEditingController.text, 'elephant');

    // Modify the field text. The options appear again and are filtered.
    focusNode.requestFocus();
    textEditingController.clear();
    await tester.enterText(find.byKey(fieldKey), 'e');
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 6);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');
    expect(lastOptions.elementAt(2), 'goose');
    expect(lastOptions.elementAt(3), 'lemur');
    expect(lastOptions.elementAt(4), 'mouse');
    expect(lastOptions.elementAt(5), 'northern white rhinoceros');

    // The selection should wrap at the top and bottom. Move up to 'mouse'
    // and then back down to 'goose' and select it.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(textEditingController.text, 'goose');
  });

  testWidgets('can hide and show options with the keyboard', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late Iterable<String> lastOptions;
    late FocusNode focusNode;
    late TextEditingController textEditingController;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextFormField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
                onFieldSubmitted: (String value) {
                  onFieldSubmitted();
                },
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // Enter text. The options are filtered by the text.
    focusNode.requestFocus();
    await tester.enterText(find.byKey(fieldKey), 'ele');
    await tester.pumpAndSettle();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions.length, 2);
    expect(lastOptions.elementAt(0), 'chameleon');
    expect(lastOptions.elementAt(1), 'elephant');

    // Hide the options.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    // Show the options again by pressing arrow keys
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);

    // Show the options again by re-focusing the field.
    focusNode.unfocus();
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);
    focusNode.requestFocus();
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);

    // Show the options again by editing the text (but not when selecting text
    // or moving the caret).
    await tester.enterText(find.byKey(fieldKey), 'elep');
    await tester.pump();
    expect(find.byKey(optionsKey), findsOneWidget);
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);
    textEditingController.selection = TextSelection.fromPosition(const TextPosition(offset: 3));
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);
  });

  testWidgets('re-invokes DismissIntent if options not shown', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late FocusNode focusNode;
    bool wrappingActionInvoked = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Actions(
            actions: <Type, Action<Intent>>{
              DismissIntent: CallbackAction<DismissIntent>(
                onInvoke: (_) => wrappingActionInvoked = true,
              ),
            },
            child: RawAutocomplete<String>(
              optionsBuilder: (TextEditingValue textEditingValue) {
                return kOptions.where((String option) {
                  return option.contains(textEditingValue.text.toLowerCase());
                });
              },
              fieldViewBuilder: (
                BuildContext context,
                TextEditingController fieldTextEditingController,
                FocusNode fieldFocusNode,
                VoidCallback onFieldSubmitted,
              ) {
                focusNode = fieldFocusNode;
                return TextFormField(
                  key: fieldKey,
                  focusNode: focusNode,
                  controller: fieldTextEditingController,
                  onFieldSubmitted: (String value) {
                    onFieldSubmitted();
                  },
                );
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
      ),
    );

    // Enter text to show options.
    focusNode.requestFocus();
    await tester.enterText(find.byKey(fieldKey), 'ele');
    await tester.pumpAndSettle();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);

    // Hide the options.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);
    expect(wrappingActionInvoked, false);

    // Ensure the wrapping Actions can receive the DismissIntent.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    expect(wrappingActionInvoked, true);
  });

  testWidgets(
    'optionsViewBuilders can use AutocompleteHighlightedOption to highlight selected option',
    (WidgetTester tester) async {
      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();
      late Iterable<String> lastOptions;
      late int lastHighlighted;
      late FocusNode focusNode;
      late TextEditingController textEditingController;
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: RawAutocomplete<String>(
              optionsBuilder: (TextEditingValue textEditingValue) {
                return kOptions.where((String option) {
                  return option.contains(textEditingValue.text.toLowerCase());
                });
              },
              fieldViewBuilder: (
                BuildContext context,
                TextEditingController fieldTextEditingController,
                FocusNode fieldFocusNode,
                VoidCallback onFieldSubmitted,
              ) {
                focusNode = fieldFocusNode;
                textEditingController = fieldTextEditingController;
                return TextFormField(
                  key: fieldKey,
                  focusNode: focusNode,
                  controller: textEditingController,
                  onFieldSubmitted: (String value) {
                    onFieldSubmitted();
                  },
                );
              },
              optionsViewBuilder: (
                BuildContext context,
                AutocompleteOnSelected<String> onSelected,
                Iterable<String> options,
              ) {
                lastOptions = options;
                lastHighlighted = AutocompleteHighlightedOption.of(context);
                return Container(key: optionsKey);
              },
            ),
          ),
        ),
      );

      // Enter text. The options are filtered by the text.
      focusNode.requestFocus();
      await tester.enterText(find.byKey(fieldKey), 'e');
      await tester.pump();
      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);
      expect(lastOptions.length, 6);
      expect(lastOptions.elementAt(0), 'chameleon');
      expect(lastOptions.elementAt(1), 'elephant');
      expect(lastOptions.elementAt(2), 'goose');
      expect(lastOptions.elementAt(3), 'lemur');
      expect(lastOptions.elementAt(4), 'mouse');
      expect(lastOptions.elementAt(5), 'northern white rhinoceros');

      // Move the highlighted option down and check the highlighted index
      expect(lastHighlighted, 0);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(lastHighlighted, 1);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(lastHighlighted, 2);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(lastHighlighted, 3);

      // And move it back up
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(lastHighlighted, 2);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(lastHighlighted, 1);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(lastHighlighted, 0);

      // Going back up should wrap around
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(lastHighlighted, 5);
    },
  );

  testWidgets('floating menu goes away on select', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/99749.
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late AutocompleteOnSelected<String> lastOnSelected;
    late FocusNode focusNode;
    late TextEditingController textEditingController;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) {
              return kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOnSelected = onSelected;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // The field is always rendered, but the options are not unless needed.
    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    await tester.enterText(find.byKey(fieldKey), kOptions[0]);
    await tester.pumpAndSettle();
    expect(find.byKey(optionsKey), findsOneWidget);

    // Pretend that the only option is selected. This does not change the
    // text in the text field.
    lastOnSelected(kOptions[0]);
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);
  });

  testWidgets('options width matches field width after rebuilding', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late StateSetter setState;
    double width = 100.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 32.0),
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter localStateSetter) {
                setState = localStateSetter;
                return SizedBox(
                  width: width,
                  child: RawAutocomplete<String>(
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
                    fieldViewBuilder: (
                      BuildContext context,
                      TextEditingController textEditingController,
                      FocusNode focusNode,
                      VoidCallback onSubmitted,
                    ) {
                      return TextField(
                        key: fieldKey,
                        focusNode: focusNode,
                        controller: textEditingController,
                      );
                    },
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    final RenderBox fieldBox = tester.renderObject(find.byKey(fieldKey));
    expect(fieldBox.size.width, 100.0);

    await tester.tap(find.byType(TextField));
    await tester.pump();

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);

    final RenderBox optionsBox = tester.renderObject(find.byKey(optionsKey));
    expect(optionsBox.size.width, 100.0);

    // Add an extra pump to account for any potential frame delays introduced by
    // the post frame callback in the _RawAutocompleteOptions implementation.
    await tester.pump();
    setState(() {
      width = 200.0;
    });
    await tester.pump();

    // The options width changes to match the field width.
    expect(fieldBox.size.width, 200.0);
    expect(optionsBox.size.width, 200.0);
  });

  testWidgets('options width matches field width after changing', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late StateSetter setState;
    double width = 100.0;

    final RawAutocomplete<String> autocomplete = RawAutocomplete<String>(
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
      fieldViewBuilder: (
        BuildContext context,
        TextEditingController textEditingController,
        FocusNode focusNode,
        VoidCallback onSubmitted,
      ) {
        return TextField(key: fieldKey, focusNode: focusNode, controller: textEditingController);
      },
    );
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Padding(
            padding: const EdgeInsets.symmetric(horizontal: 32.0),
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter localStateSetter) {
                setState = localStateSetter;
                return SizedBox(width: width, child: autocomplete);
              },
            ),
          ),
        ),
      ),
    );

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsNothing);

    final RenderBox fieldBox = tester.renderObject(find.byKey(fieldKey));
    expect(fieldBox.size.width, 100.0);

    await tester.tap(find.byType(TextField));
    await tester.pump();

    expect(find.byKey(fieldKey), findsOneWidget);
    expect(find.byKey(optionsKey), findsOneWidget);

    final RenderBox optionsBox = tester.renderObject(find.byKey(optionsKey));
    expect(fieldBox.size.width, 100.0);
    expect(optionsBox.size.width, 100.0);

    // Add an extra pump to account for any potential frame delays introduced by
    // the post frame callback in the _RawAutocompleteOptions implementation.
    await tester.pump();
    setState(() {
      width = 200.0;
    });
    await tester.pump();

    // The options width changes to match the field width.
    expect(fieldBox.size.width, 200.0);
    expect(optionsBox.size.width, 200.0);
  });

  group('screen size', () {
    Future<void> pumpRawAutocomplete(
      WidgetTester tester, {
      GlobalKey? fieldKey,
      GlobalKey? optionsKey,
      OptionsViewOpenDirection optionsViewOpenDirection = OptionsViewOpenDirection.down,
      Alignment alignment = Alignment.topLeft,
    }) {
      return tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 32.0),
              child: Align(
                alignment: alignment,
                child: RawAutocomplete<String>(
                  optionsViewOpenDirection: optionsViewOpenDirection,
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
                    return ListView.builder(
                      key: optionsKey,
                      padding: EdgeInsets.zero,
                      shrinkWrap: true,
                      itemCount: options.length,
                      itemBuilder: (BuildContext context, int index) {
                        final String option = options.elementAt(index);
                        return InkWell(
                          onTap: () {
                            onSelected(option);
                          },
                          child: Padding(padding: const EdgeInsets.all(16.0), child: Text(option)),
                        );
                      },
                    );
                  },
                  fieldViewBuilder: (
                    BuildContext context,
                    TextEditingController textEditingController,
                    FocusNode focusNode,
                    VoidCallback onSubmitted,
                  ) {
                    return TextField(
                      key: fieldKey,
                      focusNode: focusNode,
                      controller: textEditingController,
                    );
                  },
                ),
              ),
            ),
          ),
        ),
      );
    }

    testWidgets('options when screen changes landscape to portrait', (WidgetTester tester) async {
      // Start with a portrait-sized window, with enough space for all of the
      // options.
      const Size wideWindowSize = Size(1920.0, 1080.0);
      tester.view.physicalSize = wideWindowSize;
      tester.view.devicePixelRatio = 1.0;
      addTearDown(tester.view.reset);

      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();

      await pumpRawAutocomplete(tester, fieldKey: fieldKey, optionsKey: optionsKey);

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);
      expect(find.byType(InkWell), findsNWidgets(kOptions.length));

      final Size fieldSize1 = tester.getSize(find.byKey(fieldKey));
      final Offset optionsTopLeft1 = tester.getTopLeft(find.byKey(optionsKey));
      expect(
        optionsTopLeft1,
        Offset(
          tester.getTopLeft(find.byKey(fieldKey)).dx,
          tester.getTopLeft(find.byKey(fieldKey)).dy + fieldSize1.height,
        ),
      );
      final Offset optionsBottomRight1 = tester.getBottomRight(find.byKey(optionsKey));
      final double optionHeight = tester.getSize(find.byType(InkWell).first).height;
      expect(
        optionsBottomRight1,
        Offset(
          tester.getTopLeft(find.byKey(fieldKey)).dx + fieldSize1.width,
          tester.getTopLeft(find.byKey(fieldKey)).dy +
              fieldSize1.height +
              optionHeight * kOptions.length,
        ),
      );

      // Change the screen size to portrait.
      const Size narrowWindowSize = Size(1070.0, 1770.0);
      tester.view.physicalSize = narrowWindowSize;
      tester.view.devicePixelRatio = 1.0;
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byType(InkWell), findsNWidgets(kOptions.length));
      expect(tester.getTopLeft(find.byKey(optionsKey)), optionsTopLeft1);
      final Size fieldSize2 = tester.getSize(find.byKey(fieldKey));
      expect(fieldSize1.width, greaterThan(fieldSize2.width));
      expect(fieldSize1.height, fieldSize2.height);
      final Offset optionsBottomRight2 = tester.getBottomRight(find.byKey(optionsKey));
      final Offset fieldTopLeft2 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsBottomRight2.dx, lessThan(optionsBottomRight1.dx));
      expect(optionsBottomRight2.dy, optionsBottomRight1.dy);
      expect(
        optionsBottomRight2,
        Offset(
          fieldTopLeft2.dx + fieldSize2.width,
          fieldTopLeft2.dy + fieldSize2.height + optionHeight * kOptions.length,
        ),
      );
    });

    testWidgets('options when screen changes portrait to landscape and overflows', (
      WidgetTester tester,
    ) async {
      // Start with a portrait-sized window, with enough space for all of the
      // options.
      const Size narrowWindowSize = Size(1070.0, 1770.0);
      tester.view.physicalSize = narrowWindowSize;
      tester.view.devicePixelRatio = 1.0;
      addTearDown(tester.view.reset);

      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();

      await pumpRawAutocomplete(tester, fieldKey: fieldKey, optionsKey: optionsKey);

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);
      expect(find.byType(InkWell), findsNWidgets(kOptions.length));

      final Size fieldSize1 = tester.getSize(find.byKey(fieldKey));
      final Offset optionsTopLeft1 = tester.getTopLeft(find.byKey(optionsKey));
      expect(
        optionsTopLeft1,
        Offset(
          tester.getTopLeft(find.byKey(fieldKey)).dx,
          tester.getTopLeft(find.byKey(fieldKey)).dy + fieldSize1.height,
        ),
      );
      final Offset optionsBottomRight1 = tester.getBottomRight(find.byKey(optionsKey));
      final double optionHeight = tester.getSize(find.byType(InkWell).first).height;
      expect(
        optionsBottomRight1,
        Offset(
          tester.getTopLeft(find.byKey(fieldKey)).dx + fieldSize1.width,
          tester.getTopLeft(find.byKey(fieldKey)).dy +
              fieldSize1.height +
              optionHeight * kOptions.length,
        ),
      );

      // Change the screen size to landscape where the options can't all fit on
      // the screen.
      const Size wideWindowSize = Size(1920.0, 580.0);
      tester.view.physicalSize = wideWindowSize;
      tester.view.devicePixelRatio = 1.0;
      await tester.pumpAndSettle();
      expect(find.byKey(fieldKey), findsOneWidget);

      final int visibleOptions = (wideWindowSize.height / optionHeight).floor();
      expect(visibleOptions, lessThan(kOptions.length));
      expect(find.byType(InkWell), findsNWidgets(visibleOptions));
      expect(tester.getTopLeft(find.byKey(optionsKey)), optionsTopLeft1);
      final Size fieldSize2 = tester.getSize(find.byKey(fieldKey));
      expect(fieldSize1.width, lessThan(fieldSize2.width));
      expect(fieldSize1.height, fieldSize2.height);
      final Offset optionsBottomRight2 = tester.getBottomRight(find.byKey(optionsKey));
      final Offset fieldTopLeft2 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsBottomRight2.dx, greaterThan(optionsBottomRight1.dx));
      expect(optionsBottomRight2.dy, lessThan(optionsBottomRight1.dy));
      expect(
        optionsBottomRight2,
        Offset(
          fieldTopLeft2.dx + fieldSize2.width,
          // Options are taking all available space below the field.
          wideWindowSize.height,
        ),
      );
    });

    testWidgets('screen changes portrait to landscape and overflows', (WidgetTester tester) async {
      // Start with a portrait-sized window, with enough space for all of the
      // options.
      const Size narrowWindowSize = Size(1070.0, 1770.0);
      tester.view.physicalSize = narrowWindowSize;
      tester.view.devicePixelRatio = 1.0;
      addTearDown(tester.view.reset);

      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();

      await pumpRawAutocomplete(tester, fieldKey: fieldKey, optionsKey: optionsKey);

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();
      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      final double optionHeight = tester.getSize(find.byType(InkWell).first).height;
      final double optionsHeight1 = tester.getSize(find.byKey(optionsKey)).height;
      final int visibleOptions1 = (optionsHeight1 / optionHeight).ceil();
      expect(find.byType(InkWell), findsNWidgets(visibleOptions1));
      final Size fieldSize1 = tester.getSize(find.byKey(fieldKey));
      final Offset optionsTopLeft1 = tester.getTopLeft(find.byKey(optionsKey));
      final Offset fieldTopLeft1 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsTopLeft1, Offset(fieldTopLeft1.dx, fieldTopLeft1.dy + fieldSize1.height));
      final Offset optionsBottomRight1 = tester.getBottomRight(find.byKey(optionsKey));
      expect(
        optionsBottomRight1,
        Offset(
          fieldTopLeft1.dx + fieldSize1.width,
          fieldTopLeft1.dy + fieldSize1.height + optionsHeight1,
        ),
      );

      // Change the screen size to landscape where the options can't all fit on
      // the screen.
      const Size wideWindowSize = Size(1920.0, 580.0);
      tester.view.physicalSize = wideWindowSize;
      tester.view.devicePixelRatio = 1.0;
      await tester.pumpAndSettle();
      expect(find.byKey(fieldKey), findsOneWidget);

      final double optionsHeight2 = tester.getSize(find.byKey(optionsKey)).height;
      final int visibleOptions2 = (optionsHeight2 / optionHeight).ceil();
      expect(visibleOptions2, lessThan(kOptions.length));
      expect(find.byType(InkWell), findsNWidgets(visibleOptions2));
      final Offset optionsTopLeft2 = tester.getTopLeft(find.byKey(optionsKey));
      expect(optionsTopLeft2, optionsTopLeft1);
      final Size fieldSize2 = tester.getSize(find.byKey(fieldKey));
      expect(fieldSize1.width, lessThan(fieldSize2.width));
      expect(fieldSize1.height, fieldSize2.height);
      final Offset optionsBottomRight2 = tester.getBottomRight(find.byKey(optionsKey));
      final Offset fieldTopLeft2 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsBottomRight2.dx, greaterThan(optionsBottomRight1.dx));
      expect(
        optionsBottomRight2,
        Offset(
          fieldTopLeft2.dx + fieldSize2.width,
          // Options are taking all available space below the field.
          wideWindowSize.height,
        ),
      );

      // Shrink the screen further so that the options become smaller than
      // kMinInteractiveDimension and move to overlap the field.
      const Size shortWindowSize = Size(1920.0, 90.0);
      tester.view.physicalSize = shortWindowSize;
      tester.view.devicePixelRatio = 1.0;
      await tester.pumpAndSettle();
      expect(find.byKey(fieldKey), findsOneWidget);

      const int visibleOptions3 = 1;
      expect(find.byType(InkWell), findsNWidgets(visibleOptions3));
      final Offset optionsTopLeft3 = tester.getTopLeft(find.byKey(optionsKey));
      expect(optionsTopLeft3.dx, optionsTopLeft1.dx);
      // The options have moved up, overlapping the field, to still be able to
      // show kMinInteractiveDimension.
      expect(optionsTopLeft3.dy, lessThan(optionsTopLeft1.dy));
      final Size fieldSize3 = tester.getSize(find.byKey(fieldKey));
      final Offset fieldTopLeft3 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsTopLeft3.dy, lessThan(fieldTopLeft3.dy + fieldSize3.height));
      expect(fieldSize3.width, fieldSize2.width);
      expect(fieldSize1.height, fieldSize3.height);
      final Offset optionsBottomRight3 = tester.getBottomRight(find.byKey(optionsKey));
      expect(optionsBottomRight3.dx, greaterThan(optionsBottomRight1.dx));
      expect(
        optionsBottomRight3,
        Offset(fieldTopLeft3.dx + fieldSize3.width, shortWindowSize.height),
      );
    });

    testWidgets('when opening up screen changes portrait to landscape and overflows', (
      WidgetTester tester,
    ) async {
      // Start with a portrait-sized window, with enough space for all of the
      // options.
      const Size narrowWindowSize = Size(1070.0, 1770.0);
      tester.view.physicalSize = narrowWindowSize;
      tester.view.devicePixelRatio = 1.0;
      addTearDown(tester.view.reset);

      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();

      await pumpRawAutocomplete(
        tester,
        fieldKey: fieldKey,
        optionsKey: optionsKey,
        optionsViewOpenDirection: OptionsViewOpenDirection.up,
        alignment: Alignment.center,
      );

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byType(TextField));
      await tester.pumpAndSettle();
      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      final double optionHeight = tester.getSize(find.byType(InkWell).first).height;
      final double optionsHeight1 = tester.getSize(find.byKey(optionsKey)).height;
      final int visibleOptions1 = (optionsHeight1 / optionHeight).ceil();
      expect(find.byType(InkWell), findsNWidgets(visibleOptions1));
      final Size fieldSize1 = tester.getSize(find.byKey(fieldKey));
      final Offset optionsTopLeft1 = tester.getTopLeft(find.byKey(optionsKey));
      final Offset fieldTopLeft1 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsTopLeft1, Offset(fieldTopLeft1.dx, fieldTopLeft1.dy - optionsHeight1));
      expect(optionsTopLeft1.dy, greaterThan(0.0));
      final Offset optionsBottomRight1 = tester.getBottomRight(find.byKey(optionsKey));
      expect(optionsBottomRight1, Offset(fieldTopLeft1.dx + fieldSize1.width, fieldTopLeft1.dy));

      // Change the screen size to landscape where the options can't all fit on
      // the screen.
      const Size wideWindowSize = Size(1920.0, 580.0);
      tester.view.physicalSize = wideWindowSize;
      tester.view.devicePixelRatio = 1.0;
      await tester.pumpAndSettle();
      expect(find.byKey(fieldKey), findsOneWidget);

      final double optionsHeight2 = tester.getSize(find.byKey(optionsKey)).height;
      expect(optionsHeight2, lessThan(optionsHeight1));
      final int visibleOptions2 = (optionsHeight2 / optionHeight).ceil();
      expect(visibleOptions2, lessThan(visibleOptions1));
      expect(find.byType(InkWell), findsNWidgets(visibleOptions2));
      final Offset optionsTopLeft2 = tester.getTopLeft(find.byKey(optionsKey));
      final Offset fieldTopLeft2 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsTopLeft2, Offset(optionsTopLeft1.dx, fieldTopLeft2.dy - optionsHeight2));
      final Size fieldSize2 = tester.getSize(find.byKey(fieldKey));
      expect(fieldSize1.width, lessThan(fieldSize2.width));
      expect(fieldSize1.height, fieldSize2.height);
      final Offset optionsBottomRight2 = tester.getBottomRight(find.byKey(optionsKey));
      expect(optionsBottomRight2.dx, greaterThan(optionsBottomRight1.dx));
      expect(optionsBottomRight2, Offset(fieldTopLeft2.dx + fieldSize2.width, fieldTopLeft2.dy));

      // Shrink the screen further so that the options become smaller than
      // kMinInteractiveDimension and move to overlap the field.
      const Size shortWindowSize = Size(1920.0, 90.0);
      tester.view.physicalSize = shortWindowSize;
      tester.view.devicePixelRatio = 1.0;
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsOneWidget);
      const int visibleOptions3 = 1;
      expect(find.byType(InkWell), findsNWidgets(visibleOptions3));
      final Offset optionsTopLeft3 = tester.getTopLeft(find.byKey(optionsKey));
      expect(optionsTopLeft3.dx, optionsTopLeft1.dx);
      // The options have moved down, overlapping the field, to still be able to
      // show kMinInteractiveDimension.
      expect(optionsTopLeft3.dy, lessThan(optionsTopLeft1.dy));
      final Size fieldSize3 = tester.getSize(find.byKey(fieldKey));
      final Offset fieldTopLeft3 = tester.getTopLeft(find.byKey(fieldKey));
      expect(optionsTopLeft3.dy, lessThan(fieldTopLeft3.dy + fieldSize3.height));
      expect(fieldSize3.width, fieldSize2.width);
      expect(fieldSize1.height, fieldSize3.height);
      final Offset optionsBottomRight3 = tester.getBottomRight(find.byKey(optionsKey));
      expect(optionsBottomRight3.dx, greaterThan(optionsBottomRight1.dx));
      expect(optionsBottomRight3.dy, greaterThan(fieldTopLeft3.dy));
      expect(optionsBottomRight3.dx, fieldTopLeft3.dx + fieldSize3.width);
    });
  });

  testWidgets(
    'when field scrolled offscreen, options are hidden and not reshown when scrolled back on desktop and web',
    (WidgetTester tester) async {
      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ListView(
              controller: scrollController,
              children: <Widget>[
                const SizedBox(height: 1000.0),
                RawAutocomplete<String>(
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
                    return ListView.builder(
                      key: optionsKey,
                      padding: EdgeInsets.zero,
                      shrinkWrap: true,
                      itemCount: options.length,
                      itemBuilder: (BuildContext context, int index) {
                        final String option = options.elementAt(index);
                        return InkWell(
                          onTap: () {
                            onSelected(option);
                          },
                          child: Padding(padding: const EdgeInsets.all(16.0), child: Text(option)),
                        );
                      },
                    );
                  },
                  fieldViewBuilder: (
                    BuildContext context,
                    TextEditingController textEditingController,
                    FocusNode focusNode,
                    VoidCallback onSubmitted,
                  ) {
                    return TextField(
                      key: fieldKey,
                      focusNode: focusNode,
                      controller: textEditingController,
                    );
                  },
                ),
                const SizedBox(height: 1000.0),
              ],
            ),
          ),
        ),
      );

      expect(find.byKey(fieldKey), findsNothing);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.scrollUntilVisible(find.byKey(fieldKey), 500.0);
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byKey(fieldKey));
      await tester.pump();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      // Jump to the beginning. The field is off screen and the options are not
      // showing either.
      scrollController.jumpTo(0.0);
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsNothing);
      expect(find.byKey(optionsKey), findsNothing);

      // Scroll back to the field and ensure it is visible.
      await tester.scrollUntilVisible(find.byKey(fieldKey), 500.0);
      await tester.pumpAndSettle();

      // The options are no longer visible on desktop and web.
      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      // Jump to the end. The field is hidden again.
      scrollController.jumpTo(2000.0);
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsNothing);
      expect(find.byKey(optionsKey), findsNothing);
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets(
    'when field scrolled offscreen, options are hidden and reshown when scrolled back on mobile',
    (WidgetTester tester) async {
      final GlobalKey fieldKey = GlobalKey();
      final GlobalKey optionsKey = GlobalKey();
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ListView(
              controller: scrollController,
              children: <Widget>[
                const SizedBox(height: 1000.0),
                RawAutocomplete<String>(
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
                    return ListView.builder(
                      key: optionsKey,
                      padding: EdgeInsets.zero,
                      shrinkWrap: true,
                      itemCount: options.length,
                      itemBuilder: (BuildContext context, int index) {
                        final String option = options.elementAt(index);
                        return InkWell(
                          onTap: () {
                            onSelected(option);
                          },
                          child: Padding(padding: const EdgeInsets.all(16.0), child: Text(option)),
                        );
                      },
                    );
                  },
                  fieldViewBuilder: (
                    BuildContext context,
                    TextEditingController textEditingController,
                    FocusNode focusNode,
                    VoidCallback onSubmitted,
                  ) {
                    return TextField(
                      key: fieldKey,
                      focusNode: focusNode,
                      controller: textEditingController,
                    );
                  },
                ),
                const SizedBox(height: 1000.0),
              ],
            ),
          ),
        ),
      );

      expect(find.byKey(fieldKey), findsNothing);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.scrollUntilVisible(find.byKey(fieldKey), 500.0);
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsNothing);

      await tester.tap(find.byKey(fieldKey));
      await tester.pump();

      expect(find.byKey(fieldKey), findsOneWidget);
      expect(find.byKey(optionsKey), findsOneWidget);

      // Jump to the beginning. The field is off screen and the options are not
      // showing either.
      scrollController.jumpTo(0.0);
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsNothing);
      expect(find.byKey(optionsKey), findsNothing);

      // Scroll back to the field and ensure it is visible.
      await tester.scrollUntilVisible(find.byKey(fieldKey), 500.0);
      await tester.pumpAndSettle();

      // The options remain visible on mobile, but not on web.
      expect(find.byKey(fieldKey), findsOneWidget);
      kIsWeb
          ? expect(find.byKey(optionsKey), findsNothing)
          : expect(find.byKey(optionsKey), findsOneWidget);

      // Jump to the end. The field is hidden again.
      scrollController.jumpTo(2000.0);
      await tester.pumpAndSettle();

      expect(find.byKey(fieldKey), findsNothing);
      expect(find.byKey(optionsKey), findsNothing);
    },
    variant: TargetPlatformVariant.mobile(),
  );

  testWidgets('can prevent older optionsBuilder results from replacing the new ones', (
    WidgetTester tester,
  ) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late FocusNode focusNode;
    late TextEditingController textEditingController;
    Iterable<String>? lastOptions;
    Duration? delay;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) async {
              final Iterable<String> options = kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
              if (delay == null) {
                return options;
              }
              return Future<Iterable<String>>.delayed(delay, () => options);
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    const Duration longRequestDelay = Duration(milliseconds: 5000);
    const Duration shortRequestDelay = Duration(milliseconds: 1000);
    focusNode.requestFocus();

    // Enter the first letter.
    delay = longRequestDelay;
    await tester.enterText(find.byKey(fieldKey), 'c');
    await tester.pump();
    expect(lastOptions, null);

    // Enter the second letter which resolves faster.
    delay = shortRequestDelay;
    await tester.enterText(find.byKey(fieldKey), 'ch');
    await tester.pump();
    expect(lastOptions, null);

    // Wait for the short request to resolve.
    await tester.pump(shortRequestDelay);

    // lastOptions must contain results from the last request.
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions, <String>['chameleon']);

    // Wait for the last timer to finish.
    await tester.pump(longRequestDelay - shortRequestDelay);
    expect(lastOptions, <String>['chameleon']);
  });

  testWidgets('updates result only from the last call made to optionsBuilder', (
    WidgetTester tester,
  ) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late FocusNode focusNode;
    late TextEditingController textEditingController;
    Iterable<String>? lastOptions;
    Duration? delay;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) async {
              final Iterable<String> options = kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
              if (delay == null) {
                return options;
              }
              return Future<Iterable<String>>.delayed(delay, () => options);
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    focusNode.requestFocus();
    const Duration firstRequestDelay = Duration(milliseconds: 1000);
    const Duration secondRequestDelay = Duration(milliseconds: 2000);
    const Duration thirdRequestDelay = Duration(milliseconds: 3000);

    // Enter the first letter.
    delay = firstRequestDelay;
    await tester.enterText(find.byKey(fieldKey), 'l');
    await tester.pump();
    expect(lastOptions, null);

    // Enter the second letter which resolves slower.
    delay = secondRequestDelay;
    await tester.enterText(find.byKey(fieldKey), 'le');
    await tester.pump();
    expect(lastOptions, null);

    // Enter the third letter which resolves the slowest.
    delay = thirdRequestDelay;
    await tester.enterText(find.byKey(fieldKey), 'lem');
    await tester.pump();
    expect(lastOptions, null);

    // lastOptions should get updated only from the last request.
    await tester.pump(firstRequestDelay);
    expect(find.byKey(optionsKey), findsNothing);
    expect(lastOptions, null);

    await tester.pump(secondRequestDelay - firstRequestDelay);
    expect(find.byKey(optionsKey), findsNothing);
    expect(lastOptions, null);

    await tester.pump(thirdRequestDelay - secondRequestDelay);
    expect(find.byKey(optionsKey), findsOneWidget);
    expect(lastOptions, <String>['lemur']);
  });

  testWidgets('update options view when field input changes return to the starting keyword', (
    WidgetTester tester,
  ) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late FocusNode focusNode;
    late TextEditingController textEditingController;
    Iterable<String>? lastOptions;
    const Duration delay = Duration(milliseconds: 100);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) async {
              final Iterable<String> options = kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
              return Future<Iterable<String>>.delayed(delay, () => options);
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    // Setup starting point.
    await tester.enterText(find.byKey(fieldKey), 'ele');
    await tester.pump(delay);
    expect(lastOptions, <String>['chameleon', 'elephant']);
    expect(find.byKey(optionsKey), findsOneWidget);

    // Hide options.
    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);

    // Enter another letter and then immediately erase it.
    await tester.enterText(find.byKey(fieldKey), 'eleo');
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);

    await tester.enterText(find.byKey(fieldKey), 'ele');
    await tester.pump(delay);
    expect(lastOptions, <String>['chameleon', 'elephant']);

    // Options dropdown should be visible after the last optionsBuilder
    // call is resolved.
    expect(find.byKey(optionsKey), findsOneWidget);
  });

  testWidgets('optionsBuilder does not have to be a pure function', (WidgetTester tester) async {
    final GlobalKey fieldKey = GlobalKey();
    final GlobalKey optionsKey = GlobalKey();
    late FocusNode focusNode;
    late TextEditingController textEditingController;
    Iterable<String>? lastOptions;
    const Duration delay = Duration(milliseconds: 100);

    // This is used to tell optionsBuilder to return something different after
    // being called with "ele" the second time. I.e. it is not a pure function.
    int timesOptionsBuilderCalledWithEle = 0;
    final Iterable<String> altEleOptions = <String>['something new and crazy for ele!'];

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: RawAutocomplete<String>(
            optionsBuilder: (TextEditingValue textEditingValue) async {
              if (textEditingValue.text == 'ele') {
                timesOptionsBuilderCalledWithEle += 1;
                if (timesOptionsBuilderCalledWithEle > 1) {
                  return Future<Iterable<String>>.delayed(delay, () => altEleOptions);
                }
              }
              final Iterable<String> options = kOptions.where((String option) {
                return option.contains(textEditingValue.text.toLowerCase());
              });
              return Future<Iterable<String>>.delayed(delay, () => options);
            },
            fieldViewBuilder: (
              BuildContext context,
              TextEditingController fieldTextEditingController,
              FocusNode fieldFocusNode,
              VoidCallback onFieldSubmitted,
            ) {
              focusNode = fieldFocusNode;
              textEditingController = fieldTextEditingController;
              return TextField(
                key: fieldKey,
                focusNode: focusNode,
                controller: textEditingController,
              );
            },
            optionsViewBuilder: (
              BuildContext context,
              AutocompleteOnSelected<String> onSelected,
              Iterable<String> options,
            ) {
              lastOptions = options;
              return Container(key: optionsKey);
            },
          ),
        ),
      ),
    );

    await tester.enterText(find.byKey(fieldKey), 'ele');
    await tester.pump(delay);
    expect(lastOptions, <String>['chameleon', 'elephant']);
    expect(find.byKey(optionsKey), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.escape);
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);

    await tester.enterText(find.byKey(fieldKey), 'eleo');
    await tester.pump();
    expect(find.byKey(optionsKey), findsNothing);

    await tester.enterText(find.byKey(fieldKey), 'ele');
    await tester.pump(delay);
    expect(lastOptions, altEleOptions);
    expect(find.byKey(optionsKey), findsOneWidget);
  });
}
