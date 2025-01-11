// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  // Returns the RenderEditable at the given index, or the first if not given.
  RenderEditable findRenderEditable(WidgetTester tester, {int index = 0}) {
    final RenderObject root = tester.renderObject(find.byType(EditableText).at(index));
    expect(root, isNotNull);

    late RenderEditable renderEditable;
    void recursiveFinder(RenderObject child) {
      if (child is RenderEditable) {
        renderEditable = child;
        return;
      }
      child.visitChildren(recursiveFinder);
    }

    root.visitChildren(recursiveFinder);
    expect(renderEditable, isNotNull);
    return renderEditable;
  }

  List<TextSelectionPoint> globalize(Iterable<TextSelectionPoint> points, RenderBox box) {
    return points.map<TextSelectionPoint>((TextSelectionPoint point) {
      return TextSelectionPoint(box.localToGlobal(point.point), point.direction);
    }).toList();
  }

  Offset textOffsetToPosition(WidgetTester tester, int offset, {int index = 0}) {
    final RenderEditable renderEditable = findRenderEditable(tester, index: index);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(TextSelection.collapsed(offset: offset)),
      renderEditable,
    );
    expect(endpoints.length, 1);
    return endpoints[0].point + const Offset(kIsWeb ? 1.0 : 0.0, -2.0);
  }

  testWidgets('SearchBar defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    final ColorScheme colorScheme = theme.colorScheme;

    await tester.pumpWidget(
      MaterialApp(theme: theme, home: const Material(child: SearchBar(hintText: 'hint text'))),
    );

    final Finder searchBarMaterial = find.descendant(
      of: find.byType(SearchBar),
      matching: find.byType(Material),
    );

    final Material material = tester.widget<Material>(searchBarMaterial);
    checkSearchBarDefaults(tester, colorScheme, material);
  });

  testWidgets('SearchBar respects controller property', (WidgetTester tester) async {
    const String defaultText = 'default text';
    final TextEditingController controller = TextEditingController(text: defaultText);
    addTearDown(controller.dispose);

    await tester.pumpWidget(MaterialApp(home: Material(child: SearchBar(controller: controller))));

    expect(controller.value.text, defaultText);
    expect(find.text(defaultText), findsOneWidget);

    const String updatedText = 'updated text';
    await tester.enterText(find.byType(SearchBar), updatedText);
    expect(controller.value.text, updatedText);
    expect(find.text(defaultText), findsNothing);
    expect(find.text(updatedText), findsOneWidget);
  });

  testWidgets('SearchBar respects focusNode property', (WidgetTester tester) async {
    final FocusNode node = FocusNode();
    addTearDown(node.dispose);

    await tester.pumpWidget(MaterialApp(home: Material(child: SearchBar(focusNode: node))));

    expect(node.hasFocus, false);

    node.requestFocus();
    await tester.pump();
    expect(node.hasFocus, true);

    node.unfocus();
    await tester.pump();
    expect(node.hasFocus, false);
  });

  testWidgets('SearchBar focusNode is hot swappable', (WidgetTester tester) async {
    final FocusNode node1 = FocusNode();
    addTearDown(node1.dispose);

    await tester.pumpWidget(MaterialApp(home: Material(child: SearchBar(focusNode: node1))));

    expect(node1.hasFocus, isFalse);

    node1.requestFocus();
    await tester.pump();
    expect(node1.hasFocus, isTrue);

    node1.unfocus();
    await tester.pump();
    expect(node1.hasFocus, isFalse);

    final FocusNode node2 = FocusNode();
    addTearDown(node2.dispose);

    await tester.pumpWidget(MaterialApp(home: Material(child: SearchBar(focusNode: node2))));

    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isFalse);

    node2.requestFocus();
    await tester.pump();
    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isTrue);

    node2.unfocus();
    await tester.pump();
    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isFalse);

    await tester.pumpWidget(const MaterialApp(home: Material(child: SearchBar())));

    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isFalse);

    await tester.tap(find.byType(SearchBar));
    await tester.pump();
    expect(node1.hasFocus, isFalse);
    expect(node2.hasFocus, isFalse);
  });

  testWidgets('SearchBar has correct default layout and padding LTR', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: SearchBar(
            leading: IconButton(icon: const Icon(Icons.search), onPressed: () {}),
            trailing: <Widget>[IconButton(icon: const Icon(Icons.menu), onPressed: () {})],
          ),
        ),
      ),
    );

    final Rect barRect = tester.getRect(find.byType(SearchBar));
    expect(barRect.size, const Size(800.0, 56.0));
    expect(barRect, equals(const Rect.fromLTRB(0.0, 272.0, 800.0, 328.0)));

    final Rect leadingIcon = tester.getRect(find.widgetWithIcon(IconButton, Icons.search));
    // Default left padding is 8.0, and icon button has 8.0 padding, so in total the padding between
    // the edge of the bar and the icon of the button is 16.0, which matches the spec.
    expect(leadingIcon.left, equals(barRect.left + 8.0));

    final Rect textField = tester.getRect(find.byType(TextField));
    expect(textField.left, equals(leadingIcon.right + 8.0));

    final Rect trailingIcon = tester.getRect(find.widgetWithIcon(IconButton, Icons.menu));
    expect(trailingIcon.left, equals(textField.right + 8.0));
    expect(trailingIcon.right, equals(barRect.right - 8.0));
  });

  testWidgets('SearchBar has correct default layout and padding - RTL', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Center(
            child: SearchBar(
              leading: IconButton(icon: const Icon(Icons.search), onPressed: () {}),
              trailing: <Widget>[IconButton(icon: const Icon(Icons.menu), onPressed: () {})],
            ),
          ),
        ),
      ),
    );

    final Rect barRect = tester.getRect(find.byType(SearchBar));
    expect(barRect.size, const Size(800.0, 56.0));
    expect(barRect, equals(const Rect.fromLTRB(0.0, 272.0, 800.0, 328.0)));

    // The default padding is set to 8.0 so the distance between the icon of the button
    // and the edge of the bar is 16.0, which matches the spec.
    final Rect leadingIcon = tester.getRect(find.widgetWithIcon(IconButton, Icons.search));
    expect(leadingIcon.right, equals(barRect.right - 8.0));

    final Rect textField = tester.getRect(find.byType(TextField));
    expect(textField.right, equals(leadingIcon.left - 8.0));

    final Rect trailingIcon = tester.getRect(find.widgetWithIcon(IconButton, Icons.menu));
    expect(trailingIcon.right, equals(textField.left - 8.0));
    expect(trailingIcon.left, equals(barRect.left + 8.0));
  });

  testWidgets('SearchBar respects hintText property', (WidgetTester tester) async {
    const String hintText = 'hint text';
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SearchBar(hintText: hintText))),
    );

    expect(find.text(hintText), findsOneWidget);
  });

  testWidgets('SearchBar respects leading property', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    final ColorScheme colorScheme = theme.colorScheme;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchBar(leading: IconButton(icon: const Icon(Icons.search), onPressed: () {})),
        ),
      ),
    );

    expect(find.widgetWithIcon(IconButton, Icons.search), findsOneWidget);
    final Color? iconColor = _iconStyle(tester, Icons.search)?.color;
    expect(iconColor, colorScheme.onSurface); // Default icon color.
  });

  testWidgets('SearchBar respects trailing property', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    final ColorScheme colorScheme = theme.colorScheme;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchBar(
            trailing: <Widget>[IconButton(icon: const Icon(Icons.menu), onPressed: () {})],
          ),
        ),
      ),
    );

    expect(find.widgetWithIcon(IconButton, Icons.menu), findsOneWidget);
    final Color? iconColor = _iconStyle(tester, Icons.menu)?.color;
    expect(iconColor, colorScheme.onSurfaceVariant); // Default icon color.
  });

  testWidgets('SearchBar respects onTap property', (WidgetTester tester) async {
    int tapCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: SearchBar(
                onTap: () {
                  setState(() {
                    tapCount++;
                  });
                },
              ),
            );
          },
        ),
      ),
    );
    expect(tapCount, 0);
    await tester.tap(find.byType(SearchBar));
    expect(tapCount, 1);
    await tester.tap(find.byType(SearchBar));
    expect(tapCount, 2);
  });

  testWidgets('SearchBar respects onChanged property', (WidgetTester tester) async {
    int changeCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: SearchBar(
                onChanged: (_) {
                  setState(() {
                    changeCount++;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    expect(changeCount, 0);
    await tester.enterText(find.byType(SearchBar), 'a');
    expect(changeCount, 1);
    await tester.enterText(find.byType(SearchBar), 'b');
    expect(changeCount, 2);
  });

  testWidgets('SearchBar respects onSubmitted property', (WidgetTester tester) async {
    String submittedQuery = '';
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchBar(
            onSubmitted: (String text) {
              submittedQuery = text;
            },
          ),
        ),
      ),
    );

    await tester.enterText(find.byType(SearchBar), 'query');
    await tester.testTextInput.receiveAction(TextInputAction.done);

    expect(submittedQuery, equals('query'));
  });

  testWidgets('SearchBar respects constraints property', (WidgetTester tester) async {
    const BoxConstraints constraints = BoxConstraints(maxWidth: 350.0, minHeight: 80);
    await tester.pumpWidget(
      const MaterialApp(home: Center(child: Material(child: SearchBar(constraints: constraints)))),
    );

    final Rect barRect = tester.getRect(find.byType(SearchBar));
    expect(barRect.size, const Size(350.0, 80.0));
  });

  testWidgets('SearchBar respects elevation property', (WidgetTester tester) async {
    const double pressedElevation = 0.0;
    const double hoveredElevation = 1.0;
    const double focusedElevation = 2.0;
    const double defaultElevation = 3.0;
    double getElevation(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedElevation;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoveredElevation;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedElevation;
      }
      return defaultElevation;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(elevation: MaterialStateProperty.resolveWith<double>(getElevation)),
          ),
        ),
      ),
    );

    final Finder searchBarMaterial = find.descendant(
      of: find.byType(SearchBar),
      matching: find.byType(Material),
    );
    Material material = tester.widget<Material>(searchBarMaterial);

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.elevation, hoveredElevation);

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();

    material = tester.widget<Material>(searchBarMaterial);
    expect(material.elevation, pressedElevation);

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.elevation, focusedElevation);
  });

  testWidgets('SearchBar respects backgroundColor property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(backgroundColor: MaterialStateProperty.resolveWith<Color>(_getColor)),
          ),
        ),
      ),
    );

    final Finder searchBarMaterial = find.descendant(
      of: find.byType(SearchBar),
      matching: find.byType(Material),
    );
    Material material = tester.widget<Material>(searchBarMaterial);

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.color, hoveredColor);

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();

    material = tester.widget<Material>(searchBarMaterial);
    expect(material.color, pressedColor);

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.color, focusedColor);
  });

  testWidgets('SearchBar respects shadowColor property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(shadowColor: MaterialStateProperty.resolveWith<Color>(_getColor)),
          ),
        ),
      ),
    );

    final Finder searchBarMaterial = find.descendant(
      of: find.byType(SearchBar),
      matching: find.byType(Material),
    );
    Material material = tester.widget<Material>(searchBarMaterial);

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.shadowColor, hoveredColor);

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();

    material = tester.widget<Material>(searchBarMaterial);
    expect(material.shadowColor, pressedColor);

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.shadowColor, focusedColor);
  });

  testWidgets('SearchBar respects surfaceTintColor property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(surfaceTintColor: MaterialStateProperty.resolveWith<Color>(_getColor)),
          ),
        ),
      ),
    );

    final Finder searchBarMaterial = find.descendant(
      of: find.byType(SearchBar),
      matching: find.byType(Material),
    );
    Material material = tester.widget<Material>(searchBarMaterial);

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.surfaceTintColor, hoveredColor);

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();

    material = tester.widget<Material>(searchBarMaterial);
    expect(material.surfaceTintColor, pressedColor);

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.surfaceTintColor, focusedColor);
  });

  testWidgets('SearchBar respects overlayColor property', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(
              focusNode: focusNode,
              overlayColor: MaterialStateProperty.resolveWith<Color>(_getColor),
            ),
          ),
        ),
      ),
    );

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pumpAndSettle();
    BuildContext context = tester.element(find.byType(SearchBar));
    expect(Material.of(context), paints..rect(color: hoveredColor.withOpacity(1.0)));

    // On pressed.
    await tester.pumpAndSettle();
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();
    context = tester.element(find.byType(SearchBar));
    expect(
      Material.of(context),
      paints
        ..rect()
        ..rect(color: pressedColor.withOpacity(1.0)),
    );

    // On focused.
    await tester.pumpAndSettle();
    await gesture.up();
    await tester.pumpAndSettle();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    context = tester.element(find.byType(SearchBar));
    expect(
      Material.of(context),
      paints
        ..rect()
        ..rect(color: focusedColor.withOpacity(1.0)),
    );
  });

  testWidgets('SearchBar respects side and shape properties', (WidgetTester tester) async {
    const BorderSide pressedSide = BorderSide(width: 2.0);
    const BorderSide hoveredSide = BorderSide(width: 3.0);
    const BorderSide focusedSide = BorderSide(width: 4.0);
    const BorderSide defaultSide = BorderSide(width: 5.0);

    const OutlinedBorder pressedShape = RoundedRectangleBorder();
    const OutlinedBorder hoveredShape = ContinuousRectangleBorder();
    const OutlinedBorder focusedShape = CircleBorder();
    const OutlinedBorder defaultShape = StadiumBorder();
    BorderSide getSide(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedSide;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoveredSide;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedSide;
      }
      return defaultSide;
    }

    OutlinedBorder getShape(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedShape;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoveredShape;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedShape;
      }
      return defaultShape;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(
              side: MaterialStateProperty.resolveWith<BorderSide>(getSide),
              shape: MaterialStateProperty.resolveWith<OutlinedBorder>(getShape),
            ),
          ),
        ),
      ),
    );

    final Finder searchBarMaterial = find.descendant(
      of: find.byType(SearchBar),
      matching: find.byType(Material),
    );
    Material material = tester.widget<Material>(searchBarMaterial);

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.shape, hoveredShape.copyWith(side: hoveredSide));

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();

    material = tester.widget<Material>(searchBarMaterial);
    expect(material.shape, pressedShape.copyWith(side: pressedSide));

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    material = tester.widget<Material>(searchBarMaterial);
    expect(material.shape, focusedShape.copyWith(side: focusedSide));
  });

  testWidgets('SearchBar respects padding property', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(
              leading: Icon(Icons.search),
              padding: MaterialStatePropertyAll<EdgeInsets>(EdgeInsets.all(16.0)),
              trailing: <Widget>[Icon(Icons.menu)],
            ),
          ),
        ),
      ),
    );

    final Rect barRect = tester.getRect(find.byType(SearchBar));
    final Rect leadingRect = tester.getRect(find.byIcon(Icons.search));
    final Rect textFieldRect = tester.getRect(find.byType(TextField));
    final Rect trailingRect = tester.getRect(find.byIcon(Icons.menu));

    expect(barRect.left, leadingRect.left - 16.0);
    expect(leadingRect.right, textFieldRect.left - 16.0);
    expect(textFieldRect.right, trailingRect.left - 16.0);
    expect(trailingRect.right, barRect.right - 16.0);
  });

  testWidgets('SearchBar respects hintStyle property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(
              hintText: 'hint text',
              hintStyle: MaterialStateProperty.resolveWith<TextStyle?>(_getTextStyle),
            ),
          ),
        ),
      ),
    );

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    Text helperText = tester.widget(find.text('hint text'));
    expect(helperText.style?.color, hoveredColor);

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();
    helperText = tester.widget(find.text('hint text'));
    expect(helperText.style?.color, pressedColor);

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    helperText = tester.widget(find.text('hint text'));
    expect(helperText.style?.color, focusedColor);
  });

  testWidgets('SearchBar respects textStyle property', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'input text');
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(
              controller: controller,
              textStyle: MaterialStateProperty.resolveWith<TextStyle?>(_getTextStyle),
            ),
          ),
        ),
      ),
    );

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    EditableText inputText = tester.widget(find.text('input text'));
    expect(inputText.style.color, hoveredColor);

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();
    inputText = tester.widget(find.text('input text'));
    expect(inputText.style.color, pressedColor);

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    inputText = tester.widget(find.text('input text'));
    expect(inputText.style.color, focusedColor);
  });

  testWidgets('SearchBar respects textCapitalization property', (WidgetTester tester) async {
    Widget buildSearchBar(TextCapitalization textCapitalization) {
      return MaterialApp(
        home: Center(child: Material(child: SearchBar(textCapitalization: textCapitalization))),
      );
    }

    await tester.pumpWidget(buildSearchBar(TextCapitalization.characters));
    await tester.pump();
    TextField textField = tester.widget(find.byType(TextField));
    expect(textField.textCapitalization, TextCapitalization.characters);

    await tester.pumpWidget(buildSearchBar(TextCapitalization.sentences));
    await tester.pump();
    textField = tester.widget(find.byType(TextField));
    expect(textField.textCapitalization, TextCapitalization.sentences);

    await tester.pumpWidget(buildSearchBar(TextCapitalization.words));
    await tester.pump();
    textField = tester.widget(find.byType(TextField));
    expect(textField.textCapitalization, TextCapitalization.words);

    await tester.pumpWidget(buildSearchBar(TextCapitalization.none));
    await tester.pump();
    textField = tester.widget(find.byType(TextField));
    expect(textField.textCapitalization, TextCapitalization.none);
  });

  testWidgets('SearchAnchor respects textCapitalization property', (WidgetTester tester) async {
    Widget buildSearchAnchor(TextCapitalization textCapitalization) {
      return MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor(
              textCapitalization: textCapitalization,
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.ac_unit),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSearchAnchor(TextCapitalization.characters));
    await tester.pump();
    await tester.tap(find.widgetWithIcon(IconButton, Icons.ac_unit));
    await tester.pumpAndSettle();
    TextField textField = tester.widget(find.byType(TextField));
    expect(textField.textCapitalization, TextCapitalization.characters);
    await tester.tap(find.backButton());
    await tester.pump();

    await tester.pumpWidget(buildSearchAnchor(TextCapitalization.none));
    await tester.pump();
    await tester.tap(find.widgetWithIcon(IconButton, Icons.ac_unit));
    await tester.pumpAndSettle();
    textField = tester.widget(find.byType(TextField));
    expect(textField.textCapitalization, TextCapitalization.none);
  });

  testWidgets('SearchAnchor respects viewOnChanged and viewOnSubmitted properties', (
    WidgetTester tester,
  ) async {
    final SearchController controller = SearchController();
    addTearDown(controller.dispose);
    int onChangedCalled = 0;
    int onSubmittedCalled = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: Material(
                child: SearchAnchor(
                  searchController: controller,
                  viewOnChanged: (String value) {
                    setState(() {
                      onChangedCalled = onChangedCalled + 1;
                    });
                  },
                  viewOnSubmitted: (String value) {
                    setState(() {
                      onSubmittedCalled = onSubmittedCalled + 1;
                    });
                    controller.closeView(value);
                  },
                  builder: (BuildContext context, SearchController controller) {
                    return SearchBar(
                      onTap: () {
                        if (!controller.isOpen) {
                          controller.openView();
                        }
                      },
                    );
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) {
                    return <Widget>[];
                  },
                ),
              ),
            );
          },
        ),
      ),
    );
    await tester.tap(find.byType(SearchBar)); // Open search view.
    await tester.pumpAndSettle();
    expect(controller.isOpen, true);

    final Finder barOnView = find.descendant(
      of: findViewContent(),
      matching: find.byType(TextField),
    );
    await tester.enterText(barOnView, 'a');
    expect(onChangedCalled, 1);
    await tester.enterText(barOnView, 'abc');
    expect(onChangedCalled, 2);

    await tester.testTextInput.receiveAction(TextInputAction.done);
    expect(onSubmittedCalled, 1);
    expect(controller.isOpen, false);
  });

  testWidgets('SearchAnchor.bar respects textCapitalization property', (WidgetTester tester) async {
    Widget buildSearchAnchor(TextCapitalization textCapitalization) {
      return MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor.bar(
              textCapitalization: textCapitalization,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSearchAnchor(TextCapitalization.characters));
    await tester.pump();
    await tester.tap(find.byType(SearchBar)); // Open search view.
    await tester.pumpAndSettle();
    final Finder textFieldFinder = find.descendant(
      of: findViewContent(),
      matching: find.byType(TextField),
    );
    final TextField textFieldInView = tester.widget<TextField>(textFieldFinder);
    expect(textFieldInView.textCapitalization, TextCapitalization.characters);
    // Close search view.
    await tester.tap(find.backButton());
    await tester.pumpAndSettle();
    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.textCapitalization, TextCapitalization.characters);
  });

  testWidgets('SearchAnchor.bar respects onChanged and onSubmitted properties', (
    WidgetTester tester,
  ) async {
    final SearchController controller = SearchController();
    addTearDown(controller.dispose);
    int onChangedCalled = 0;
    int onSubmittedCalled = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Center(
              child: Material(
                child: SearchAnchor.bar(
                  searchController: controller,
                  onSubmitted: (String value) {
                    setState(() {
                      onSubmittedCalled = onSubmittedCalled + 1;
                    });
                    controller.closeView(value);
                  },
                  onChanged: (String value) {
                    setState(() {
                      onChangedCalled = onChangedCalled + 1;
                    });
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) {
                    return <Widget>[];
                  },
                ),
              ),
            );
          },
        ),
      ),
    );
    await tester.tap(find.byType(SearchBar)); // Open search view.
    await tester.pumpAndSettle();
    expect(controller.isOpen, true);

    final Finder barOnView = find.descendant(
      of: findViewContent(),
      matching: find.byType(TextField),
    );
    await tester.enterText(barOnView, 'a');
    expect(onChangedCalled, 1);
    await tester.enterText(barOnView, 'abc');
    expect(onChangedCalled, 2);

    await tester.testTextInput.receiveAction(TextInputAction.done);
    expect(onSubmittedCalled, 1);
    expect(controller.isOpen, false);

    await tester.testTextInput.receiveAction(TextInputAction.done);
    expect(onSubmittedCalled, 2);
  });

  testWidgets('hintStyle can override textStyle for hintText', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchBar(
              hintText: 'hint text',
              hintStyle: MaterialStateProperty.resolveWith<TextStyle?>(_getTextStyle),
              textStyle: const MaterialStatePropertyAll<TextStyle>(TextStyle(color: Colors.pink)),
            ),
          ),
        ),
      ),
    );

    // On hovered.
    final TestGesture gesture = await _pointGestureToSearchBar(tester);
    await tester.pump();
    Text helperText = tester.widget(find.text('hint text'));
    expect(helperText.style?.color, hoveredColor);

    // On pressed.
    await gesture.down(tester.getCenter(find.byType(SearchBar)));
    await tester.pumpAndSettle();
    helperText = tester.widget(find.text('hint text'));
    expect(helperText.style?.color, pressedColor);

    // On focused.
    await gesture.up();
    await tester.pump();
    // Remove the pointer so we are no longer hovering.
    await gesture.removePointer();
    await tester.pump();
    helperText = tester.widget(find.text('hint text'));
    expect(helperText.style?.color, focusedColor);
  });

  // Regression test for https://github.com/flutter/flutter/issues/127092.
  testWidgets('The text is still centered when SearchBar text field is smaller than 48', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: const Center(
          child: Material(child: SearchBar(constraints: BoxConstraints.tightFor(height: 35.0))),
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'input text');
    final Finder textContent = find.text('input text');
    final double textCenterY = tester.getCenter(textContent).dy;
    final Finder searchBar = find.byType(SearchBar);
    final double searchBarCenterY = tester.getCenter(searchBar).dy;
    expect(textCenterY, searchBarCenterY);
  });

  testWidgets('The search view defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    final ColorScheme colorScheme = theme.colorScheme;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Material(
            child: Align(
              alignment: Alignment.topLeft,
              child: SearchAnchor(
                viewHintText: 'hint text',
                builder: (BuildContext context, SearchController controller) {
                  return const Icon(Icons.search);
                },
                suggestionsBuilder: (BuildContext context, SearchController controller) {
                  return <Widget>[];
                },
              ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    final Material material = getSearchViewMaterial(tester);
    expect(material.elevation, 6.0);
    expect(material.color, colorScheme.surfaceContainerHigh);
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.clipBehavior, Clip.antiAlias);

    final Finder findDivider = find.byType(Divider);
    final Container dividerContainer = tester.widget<Container>(
      find.descendant(of: findDivider, matching: find.byType(Container)).first,
    );
    final BoxDecoration decoration = dividerContainer.decoration! as BoxDecoration;
    expect(decoration.border!.bottom.color, colorScheme.outline);

    // Default search view has a leading back button on the start of the header.
    expect(find.backButton(), findsOneWidget);

    final Text helperText = tester.widget(find.text('hint text'));
    expect(helperText.style?.color, colorScheme.onSurfaceVariant);
    expect(helperText.style?.fontSize, 16.0);
    expect(helperText.style?.fontFamily, 'Roboto');
    expect(helperText.style?.fontWeight, FontWeight.w400);

    const String input = 'entered text';
    await tester.enterText(find.byType(SearchBar), input);
    final EditableText inputText = tester.widget(find.text(input));
    expect(inputText.style.color, colorScheme.onSurface);
    expect(inputText.style.fontSize, 16.0);
    expect(inputText.style.fontFamily, 'Roboto');
    expect(inputText.style.fontWeight, FontWeight.w400);
  });

  testWidgets('The search view default size on different platforms', (WidgetTester tester) async {
    // The search view should be is full-screen on mobile platforms,
    // and have a size of (360, 2/3 screen height) on other platforms
    Widget buildSearchAnchor(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Scaffold(
          body: SafeArea(
            child: Material(
              child: Align(
                alignment: Alignment.topLeft,
                child: SearchAnchor(
                  builder: (BuildContext context, SearchController controller) {
                    return const Icon(Icons.search);
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) {
                    return <Widget>[];
                  },
                ),
              ),
            ),
          ),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    ]) {
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildSearchAnchor(platform));
      await tester.tap(find.byIcon(Icons.search));
      await tester.pumpAndSettle();
      final Size size = tester.getSize(
        find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
      );
      expect(size.width, 800.0);
      expect(size.height, 600.0);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildSearchAnchor(platform));
      await tester.tap(find.byIcon(Icons.search));
      await tester.pumpAndSettle();
      final Size size = tester.getSize(
        find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
      );
      expect(size.width, 360.0);
      expect(size.height, 400.0);
    }
  });

  testWidgets('SearchAnchor respects isFullScreen property', (WidgetTester tester) async {
    Widget buildSearchAnchor(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Scaffold(
          body: SafeArea(
            child: Material(
              child: Align(
                alignment: Alignment.topLeft,
                child: SearchAnchor(
                  isFullScreen: true,
                  builder: (BuildContext context, SearchController controller) {
                    return const Icon(Icons.search);
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) {
                    return <Widget>[];
                  },
                ),
              ),
            ),
          ),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildSearchAnchor(platform));
      await tester.tap(find.byIcon(Icons.search));
      await tester.pumpAndSettle();
      final Size size = tester.getSize(
        find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
      );
      expect(size.width, 800.0);
      expect(size.height, 600.0);
    }
  });

  testWidgets('SearchAnchor respects controller property', (WidgetTester tester) async {
    const String defaultText = 'initial text';
    final SearchController controller = SearchController();
    addTearDown(controller.dispose);
    controller.text = defaultText;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            searchController: controller,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(controller.value.text, defaultText);
    expect(find.text(defaultText), findsOneWidget);

    const String updatedText = 'updated text';
    await tester.enterText(find.byType(SearchBar), updatedText);
    expect(controller.value.text, updatedText);
    expect(find.text(defaultText), findsNothing);
    expect(find.text(updatedText), findsOneWidget);
  });

  testWidgets('SearchAnchor attaches and detaches controllers property', (
    WidgetTester tester,
  ) async {
    Widget builder(BuildContext context, SearchController controller) {
      return const Icon(Icons.search);
    }

    List<Widget> suggestionsBuilder(BuildContext context, SearchController controller) {
      return const <Widget>[];
    }

    final SearchController controller1 = SearchController();
    addTearDown(controller1.dispose);

    expect(controller1.isAttached, isFalse);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            searchController: controller1,
            builder: builder,
            suggestionsBuilder: suggestionsBuilder,
          ),
        ),
      ),
    );

    expect(controller1.isAttached, isTrue);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(builder: builder, suggestionsBuilder: suggestionsBuilder),
        ),
      ),
    );

    expect(controller1.isAttached, isFalse);

    final SearchController controller2 = SearchController();
    addTearDown(controller2.dispose);

    expect(controller2.isAttached, isFalse);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            searchController: controller2,
            builder: builder,
            suggestionsBuilder: suggestionsBuilder,
          ),
        ),
      ),
    );

    expect(controller1.isAttached, isFalse);
    expect(controller2.isAttached, isTrue);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(builder: builder, suggestionsBuilder: suggestionsBuilder),
        ),
      ),
    );

    expect(controller1.isAttached, isFalse);
    expect(controller2.isAttached, isFalse);
  });

  testWidgets('SearchAnchor respects viewBuilder property', (WidgetTester tester) async {
    Widget buildAnchor({ViewBuilder? viewBuilder}) {
      return MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewBuilder: viewBuilder,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAnchor());
    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    // Default is a ListView.
    expect(find.byType(ListView), findsOneWidget);

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildAnchor(
        viewBuilder:
            (Iterable<Widget> suggestions) =>
                GridView.count(crossAxisCount: 5, children: suggestions.toList()),
      ),
    );
    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(find.byType(ListView), findsNothing);
    expect(find.byType(GridView), findsOneWidget);
  });

  testWidgets('SearchAnchor.bar respects viewBuilder property', (WidgetTester tester) async {
    Widget buildAnchor({ViewBuilder? viewBuilder}) {
      return MaterialApp(
        home: Material(
          child: SearchAnchor.bar(
            viewBuilder: viewBuilder,
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAnchor());
    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    // Default is a ListView.
    expect(find.byType(ListView), findsOneWidget);

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildAnchor(
        viewBuilder:
            (Iterable<Widget> suggestions) =>
                GridView.count(crossAxisCount: 5, children: suggestions.toList()),
      ),
    );
    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    expect(find.byType(ListView), findsNothing);
    expect(find.byType(GridView), findsOneWidget);
  });

  testWidgets('SearchAnchor respects viewLeading property', (WidgetTester tester) async {
    Widget buildAnchor({Widget? viewLeading}) {
      return MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewLeading: viewLeading,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAnchor());
    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    // Default is a icon button with arrow_back.
    expect(find.backButton(), findsOneWidget);

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildAnchor(viewLeading: const Icon(Icons.history)));
    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(find.backButton(), findsNothing);
    expect(find.byIcon(Icons.history), findsOneWidget);
  });

  testWidgets('SearchAnchor respects viewTrailing property', (WidgetTester tester) async {
    Widget buildAnchor({Iterable<Widget>? viewTrailing}) {
      return MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewTrailing: viewTrailing,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildAnchor());
    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    // Default is a icon button with close icon when input is not empty.
    await tester.enterText(findTextField(), 'a');
    await tester.pump();
    expect(find.widgetWithIcon(IconButton, Icons.close), findsOneWidget);

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildAnchor(viewTrailing: <Widget>[const Icon(Icons.history)]));
    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(find.byIcon(Icons.close), findsNothing);
    expect(find.byIcon(Icons.history), findsOneWidget);
  });

  testWidgets('SearchAnchor respects viewHintText property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewHintText: 'hint text',
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );
    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(find.text('hint text'), findsOneWidget);
  });

  testWidgets('SearchAnchor respects viewBackgroundColor property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewBackgroundColor: Colors.purple,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(getSearchViewMaterial(tester).color, Colors.purple);
  });

  testWidgets('SearchAnchor respects viewElevation property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewElevation: 3.0,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(getSearchViewMaterial(tester).elevation, 3.0);
  });

  testWidgets('SearchAnchor respects viewSurfaceTint property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewSurfaceTintColor: Colors.purple,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(getSearchViewMaterial(tester).surfaceTintColor, Colors.purple);
  });

  testWidgets('SearchAnchor respects viewSide property', (WidgetTester tester) async {
    const BorderSide side = BorderSide(color: Colors.purple, width: 5.0);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            isFullScreen: false,
            viewSide: side,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(
      getSearchViewMaterial(tester).shape,
      RoundedRectangleBorder(side: side, borderRadius: BorderRadius.circular(28.0)),
    );
  });

  testWidgets('SearchAnchor respects viewShape property', (WidgetTester tester) async {
    const BorderSide side = BorderSide(color: Colors.purple, width: 5.0);
    const OutlinedBorder shape = StadiumBorder(side: side);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            isFullScreen: false,
            viewShape: shape,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    expect(getSearchViewMaterial(tester).shape, shape);
  });

  testWidgets('SearchAnchor respects headerTextStyle property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            headerTextStyle: theme.textTheme.bodyLarge?.copyWith(color: Colors.red),
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();
    await tester.enterText(find.byType(SearchBar), 'input text');
    await tester.pumpAndSettle();

    final EditableText inputText = tester.widget(find.text('input text'));
    expect(inputText.style.color, Colors.red);
  });

  testWidgets('SearchAnchor respects headerHintStyle property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            viewHintText: 'hint text',
            headerHintStyle: theme.textTheme.bodyLarge?.copyWith(color: Colors.orange),
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();

    final Text inputText = tester.widget(find.text('hint text'));
    expect(inputText.style?.color, Colors.orange);
  });

  testWidgets('SearchAnchor respects viewPadding property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            isFullScreen: false,
            viewPadding: const EdgeInsets.all(16.0),
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();

    final Padding padding = tester.widget<Padding>(
      find.descendant(of: findViewContent(), matching: find.byType(Padding)).first,
    );
    expect(padding.padding, const EdgeInsets.all(16.0));
  });

  testWidgets('SearchAnchor ignores viewPadding property if full screen', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            isFullScreen: true,
            viewPadding: const EdgeInsets.all(16.0),
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();

    final Padding padding = tester.widget<Padding>(
      find.descendant(of: findViewContent(), matching: find.byType(Padding)).first,
    );
    expect(padding.padding, EdgeInsets.zero);
  });

  testWidgets('SearchAnchor respects shrinkWrap property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            isFullScreen: false,
            shrinkWrap: true,
            viewConstraints: const BoxConstraints(),
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return List<Widget>.generate(
                controller.text.length,
                (int index) => ListTile(title: Text('Item $index')),
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();

    final Finder findDivider = find.descendant(
      of: findViewContent(),
      matching: find.byType(Divider),
    );

    // Divider should not be shown if there are no suggestions
    expect(findDivider, findsNothing);

    final Finder findMaterial =
        find.descendant(of: findViewContent(), matching: find.byType(Material)).first;
    final Rect materialRectWithoutSuggestions = tester.getRect(findMaterial);
    expect(materialRectWithoutSuggestions, equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0)));

    await tester.enterText(find.byType(SearchBar), 'a');
    await tester.pumpAndSettle();

    expect(findDivider, findsOneWidget);

    final Rect materialRectWithSuggestions = tester.getRect(findMaterial);
    expect(materialRectWithSuggestions, equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 113.0)));
  });

  testWidgets('SearchAnchor respects dividerColor property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            dividerColor: Colors.red,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();

    final Finder findDivider = find.byType(Divider);
    final Container dividerContainer = tester.widget<Container>(
      find.descendant(of: findDivider, matching: find.byType(Container)).first,
    );
    final BoxDecoration decoration = dividerContainer.decoration! as BoxDecoration;
    expect(decoration.border!.bottom.color, Colors.red);
  });

  testWidgets('SearchAnchor respects viewConstraints property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SearchAnchor(
              isFullScreen: false,
              viewConstraints: BoxConstraints.tight(const Size(280.0, 390.0)),
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();

    final Size size = tester.getSize(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(size.width, 280.0);
    expect(size.height, 390.0);
  });

  testWidgets('SearchAnchor respects viewBarPadding property', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SearchAnchor(
              viewBarPadding: const EdgeInsets.symmetric(horizontal: 16.0),
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.widgetWithIcon(IconButton, Icons.search));
    await tester.pumpAndSettle();

    final Finder findSearchBar =
        find.descendant(of: findViewContent(), matching: find.byType(SearchBar)).first;
    final Padding padding = tester.widget<Padding>(
      find.descendant(of: findSearchBar, matching: find.byType(Padding)).first,
    );
    expect(padding.padding, const EdgeInsets.symmetric(horizontal: 16.0));
  });

  testWidgets('SearchAnchor respects builder property - LTR', (WidgetTester tester) async {
    Widget buildAnchor({required SearchAnchorChildBuilder builder}) {
      return MaterialApp(
        home: Material(
          child: Align(
            alignment: Alignment.topCenter,
            child: SearchAnchor(
              isFullScreen: false,
              builder: builder,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(
      buildAnchor(
        builder: (BuildContext context, SearchController controller) => const Icon(Icons.search),
      ),
    );
    final Rect anchorRect = tester.getRect(find.byIcon(Icons.search));
    expect(anchorRect.size, const Size(24.0, 24.0));
    expect(anchorRect, equals(const Rect.fromLTRB(388.0, 0.0, 412.0, 24.0)));

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect, equals(const Rect.fromLTRB(388.0, 0.0, 748.0, 400.0)));

    // Search view top left should be the same as the anchor top left
    expect(searchViewRect.topLeft, anchorRect.topLeft);
  });

  testWidgets('SearchAnchor respects builder property - RTL', (WidgetTester tester) async {
    Widget buildAnchor({required SearchAnchorChildBuilder builder}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Align(
              alignment: Alignment.topCenter,
              child: SearchAnchor(
                isFullScreen: false,
                builder: builder,
                suggestionsBuilder: (BuildContext context, SearchController controller) {
                  return <Widget>[];
                },
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(
      buildAnchor(
        builder: (BuildContext context, SearchController controller) => const Icon(Icons.search),
      ),
    );
    final Rect anchorRect = tester.getRect(find.byIcon(Icons.search));
    expect(anchorRect.size, const Size(24.0, 24.0));
    expect(anchorRect, equals(const Rect.fromLTRB(388.0, 0.0, 412.0, 24.0)));

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect, equals(const Rect.fromLTRB(52.0, 0.0, 412.0, 400.0)));

    // Search view top right should be the same as the anchor top right
    expect(searchViewRect.topRight, anchorRect.topRight);
  });

  testWidgets('SearchAnchor respects suggestionsBuilder property', (WidgetTester tester) async {
    final SearchController controller = SearchController();
    addTearDown(controller.dispose);
    const String suggestion = 'suggestion text';

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Align(
                alignment: Alignment.topCenter,
                child: SearchAnchor(
                  searchController: controller,
                  builder: (BuildContext context, SearchController controller) {
                    return const Icon(Icons.search);
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) {
                    return <Widget>[
                      ListTile(
                        title: const Text(suggestion),
                        onTap: () {
                          setState(() {
                            controller.closeView(suggestion);
                          });
                        },
                      ),
                    ];
                  },
                ),
              ),
            );
          },
        ),
      ),
    );
    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Finder listTile = find.widgetWithText(ListTile, suggestion);
    expect(listTile, findsOneWidget);
    await tester.tap(listTile);
    await tester.pumpAndSettle();

    expect(controller.isOpen, false);
    expect(controller.value.text, suggestion);
  });

  testWidgets('SearchAnchor should update suggestions on changes to search controller', (
    WidgetTester tester,
  ) async {
    final SearchController controller = SearchController();
    const List<String> suggestions = <String>['foo', 'far', 'bim'];
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Align(
                alignment: Alignment.topCenter,
                child: SearchAnchor(
                  searchController: controller,
                  builder: (BuildContext context, SearchController controller) {
                    return const Icon(Icons.search);
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) {
                    final String searchText = controller.text.toLowerCase();
                    if (searchText.isEmpty) {
                      return const <Widget>[Center(child: Text('No Search'))];
                    }
                    final Iterable<String> filterSuggestions = suggestions.where(
                      (String suggestion) => suggestion.toLowerCase().contains(searchText),
                    );
                    return filterSuggestions.map((String suggestion) {
                      return ListTile(
                        title: Text(suggestion),
                        trailing: IconButton(
                          icon: const Icon(Icons.call_missed),
                          onPressed: () {
                            controller.text = suggestion;
                          },
                        ),
                        onTap: () {
                          controller.closeView(suggestion);
                        },
                      );
                    }).toList();
                  },
                ),
              ),
            );
          },
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Finder listTile1 = find.widgetWithText(ListTile, 'foo');
    final Finder listTile2 = find.widgetWithText(ListTile, 'far');
    final Finder listTile3 = find.widgetWithText(ListTile, 'bim');
    final Finder textWidget = find.widgetWithText(Center, 'No Search');
    final Finder iconInListTile1 = find.descendant(
      of: listTile1,
      matching: find.byIcon(Icons.call_missed),
    );

    expect(textWidget, findsOneWidget);
    expect(listTile1, findsNothing);
    expect(listTile2, findsNothing);
    expect(listTile3, findsNothing);

    await tester.enterText(find.byType(SearchBar), 'f');
    await tester.pumpAndSettle();
    expect(textWidget, findsNothing);
    expect(listTile1, findsOneWidget);
    expect(listTile2, findsOneWidget);
    expect(listTile3, findsNothing);

    await tester.tap(iconInListTile1);
    await tester.pumpAndSettle();
    expect(controller.value.text, 'foo');
    expect(textWidget, findsNothing);
    expect(listTile1, findsOneWidget);
    expect(listTile2, findsNothing);
    expect(listTile3, findsNothing);

    await tester.tap(listTile1);
    await tester.pumpAndSettle();
    expect(controller.isOpen, false);
    expect(controller.value.text, 'foo');
    expect(textWidget, findsNothing);
    expect(listTile1, findsNothing);
    expect(listTile2, findsNothing);
    expect(listTile3, findsNothing);
  });

  testWidgets('SearchAnchor suggestionsBuilder property could be async', (
    WidgetTester tester,
  ) async {
    final SearchController controller = SearchController();
    addTearDown(controller.dispose);
    const String suggestion = 'suggestion text';

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Align(
                alignment: Alignment.topCenter,
                child: SearchAnchor(
                  searchController: controller,
                  builder: (BuildContext context, SearchController controller) {
                    return const Icon(Icons.search);
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) async {
                    return <Widget>[
                      ListTile(
                        title: const Text(suggestion),
                        onTap: () {
                          setState(() {
                            controller.closeView(suggestion);
                          });
                        },
                      ),
                    ];
                  },
                ),
              ),
            );
          },
        ),
      ),
    );
    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Finder text = find.text(suggestion);
    expect(text, findsOneWidget);
    await tester.tap(text);
    await tester.pumpAndSettle();

    expect(controller.isOpen, false);
    expect(controller.value.text, suggestion);
  });

  testWidgets('SearchAnchor.bar has a default search bar as the anchor', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Align(
            alignment: Alignment.topLeft,
            child: SearchAnchor.bar(
              isFullScreen: false,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );

    expect(find.byType(SearchBar), findsOneWidget);
    final Rect anchorRect = tester.getRect(find.byType(SearchBar));
    expect(anchorRect.size, const Size(800.0, 56.0));
    expect(anchorRect, equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 56.0)));

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect, equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 400.0)));

    // Search view has same width with the default anchor(search bar).
    expect(searchViewRect.width, anchorRect.width);
  });

  testWidgets('SearchController can open/close view', (WidgetTester tester) async {
    final SearchController controller = SearchController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor.bar(
            searchController: controller,
            isFullScreen: false,
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[
                ListTile(
                  title: const Text('item 0'),
                  onTap: () {
                    controller.closeView('item 0');
                  },
                ),
              ];
            },
          ),
        ),
      ),
    );

    expect(controller.isOpen, false);
    await tester.tap(find.byType(SearchBar));
    await tester.pumpAndSettle();

    expect(controller.isOpen, true);
    await tester.tap(find.widgetWithText(ListTile, 'item 0'));
    await tester.pumpAndSettle();
    expect(controller.isOpen, false);
    controller.openView();
    expect(controller.isOpen, true);
  });

  testWidgets('Search view does not go off the screen - LTR', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Align(
            // Put the search anchor on the bottom-right corner of the screen to test
            // if the search view goes off the window.
            alignment: Alignment.bottomRight,
            child: SearchAnchor(
              isFullScreen: false,
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );

    final Finder findIconButton = find.widgetWithIcon(IconButton, Icons.search);
    final Rect iconButton = tester.getRect(findIconButton);
    // Icon button has a size of (48.0, 48.0) and the screen size is (800.0, 600.0).
    expect(iconButton, equals(const Rect.fromLTRB(752.0, 552.0, 800.0, 600.0)));

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect, equals(const Rect.fromLTRB(440.0, 200.0, 800.0, 600.0)));
  });

  testWidgets('Search view does not go off the screen - RTL', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Align(
              // Put the search anchor on the bottom-left corner of the screen to test
              // if the search view goes off the window when the text direction is right-to-left.
              alignment: Alignment.bottomLeft,
              child: SearchAnchor(
                isFullScreen: false,
                builder: (BuildContext context, SearchController controller) {
                  return IconButton(
                    icon: const Icon(Icons.search),
                    onPressed: () {
                      controller.openView();
                    },
                  );
                },
                suggestionsBuilder: (BuildContext context, SearchController controller) {
                  return <Widget>[];
                },
              ),
            ),
          ),
        ),
      ),
    );

    final Finder findIconButton = find.widgetWithIcon(IconButton, Icons.search);
    final Rect iconButton = tester.getRect(findIconButton);
    expect(iconButton, equals(const Rect.fromLTRB(0.0, 552.0, 48.0, 600.0)));

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect, equals(const Rect.fromLTRB(0.0, 200.0, 360.0, 600.0)));
  });

  testWidgets('Search view becomes smaller if the window size is smaller than the view size', (
    WidgetTester tester,
  ) async {
    addTearDown(tester.view.reset);
    tester.view.physicalSize = const Size(200.0, 200.0);
    tester.view.devicePixelRatio = 1.0;

    Widget buildSearchAnchor({TextDirection textDirection = TextDirection.ltr}) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Material(
            child: SearchAnchor(
              isFullScreen: false,
              builder: (BuildContext context, SearchController controller) {
                return Align(
                  alignment: Alignment.bottomRight,
                  child: IconButton(
                    icon: const Icon(Icons.search),
                    onPressed: () {
                      controller.openView();
                    },
                  ),
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    // Test LTR text direction.
    await tester.pumpWidget(buildSearchAnchor());

    final Finder findIconButton = find.widgetWithIcon(IconButton, Icons.search);
    final Rect iconButton = tester.getRect(findIconButton);
    // The icon button size is (48.0, 48.0), and the screen size is (200.0, 200.0)
    expect(iconButton, equals(const Rect.fromLTRB(152.0, 152.0, 200.0, 200.0)));

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect, equals(const Rect.fromLTRB(0.0, 0.0, 200.0, 200.0)));

    // Test RTL text direction.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildSearchAnchor(textDirection: TextDirection.rtl));

    final Finder findIconButtonRTL = find.widgetWithIcon(IconButton, Icons.search);
    final Rect iconButtonRTL = tester.getRect(findIconButtonRTL);
    // The icon button size is (48.0, 48.0), and the screen size is (200.0, 200.0)
    expect(iconButtonRTL, equals(const Rect.fromLTRB(152.0, 152.0, 200.0, 200.0)));

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRectRTL = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRectRTL, equals(const Rect.fromLTRB(0.0, 0.0, 200.0, 200.0)));
  });

  testWidgets('Docked search view route is popped if the window size changes', (
    WidgetTester tester,
  ) async {
    addTearDown(tester.view.reset);
    tester.view.physicalSize = const Size(500.0, 600.0);
    tester.view.devicePixelRatio = 1.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            isFullScreen: false,
            builder: (BuildContext context, SearchController controller) {
              return Align(
                alignment: Alignment.bottomRight,
                child: IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                ),
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    // Open the search view
    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    expect(find.backButton(), findsOneWidget);

    // Change window size
    tester.view.physicalSize = const Size(250.0, 200.0);
    tester.view.devicePixelRatio = 1.0;
    await tester.pumpAndSettle();
    expect(find.backButton(), findsNothing);
  });

  testWidgets('Full-screen search view route should stay if the window size changes', (
    WidgetTester tester,
  ) async {
    addTearDown(tester.view.reset);
    tester.view.physicalSize = const Size(500.0, 600.0);
    tester.view.devicePixelRatio = 1.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            isFullScreen: true,
            builder: (BuildContext context, SearchController controller) {
              return Align(
                alignment: Alignment.bottomRight,
                child: IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                ),
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    // Open a full-screen search view
    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    expect(find.backButton(), findsOneWidget);

    // Change window size
    tester.view.physicalSize = const Size(250.0, 200.0);
    tester.view.devicePixelRatio = 1.0;
    await tester.pumpAndSettle();
    expect(find.backButton(), findsOneWidget);
  });

  testWidgets('Search view route does not throw exception during pop animation', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/126590.
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SearchAnchor(
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return List<Widget>.generate(5, (int index) {
                  final String item = 'item $index';
                  return ListTile(
                    leading: const Icon(Icons.history),
                    title: Text(item),
                    trailing: const Icon(Icons.chevron_right),
                    onTap: () {},
                  );
                });
              },
            ),
          ),
        ),
      ),
    );

    // Open search view
    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    // Pop search view route
    await tester.tap(find.backButton());
    await tester.pumpAndSettle();

    // No exception.
  });

  testWidgets('Docked search should position itself correctly based on closest navigator', (
    WidgetTester tester,
  ) async {
    const double rootSpacing = 100.0;

    await tester.pumpWidget(
      MaterialApp(
        builder: (BuildContext context, Widget? child) {
          return Scaffold(body: Padding(padding: const EdgeInsets.all(rootSpacing), child: child));
        },
        home: Material(
          child: SearchAnchor(
            isFullScreen: false,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                icon: const Icon(Icons.search),
                onPressed: () {
                  controller.openView();
                },
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect.topLeft, equals(const Offset(rootSpacing, rootSpacing)));
  });

  testWidgets('Docked search view with nested navigator does not go off the screen', (
    WidgetTester tester,
  ) async {
    addTearDown(tester.view.reset);
    tester.view.physicalSize = const Size(400.0, 400.0);
    tester.view.devicePixelRatio = 1.0;

    const double rootSpacing = 100.0;

    await tester.pumpWidget(
      MaterialApp(
        builder: (BuildContext context, Widget? child) {
          return Scaffold(body: Padding(padding: const EdgeInsets.all(rootSpacing), child: child));
        },
        home: Material(
          child: Align(
            alignment: Alignment.bottomRight,
            child: SearchAnchor(
              isFullScreen: false,
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.search),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();

    final Rect searchViewRect = tester.getRect(
      find.descendant(of: findViewContent(), matching: find.byType(ConstrainedBox)).first,
    );
    expect(searchViewRect.bottomRight, equals(const Offset(300.0, 300.0)));
  });

  // Regression tests for https://github.com/flutter/flutter/issues/128332
  group('SearchAnchor text selection', () {
    testWidgets('can right-click to select word', (WidgetTester tester) async {
      const String defaultText = 'initial text';
      final SearchController controller = SearchController();
      addTearDown(controller.dispose);
      controller.text = defaultText;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SearchAnchor.bar(
              searchController: controller,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );

      expect(controller.value.text, defaultText);
      expect(find.text(defaultText), findsOneWidget);

      final TestGesture gesture = await tester.startGesture(
        textOffsetToPosition(tester, 4) + const Offset(0.0, -9.0),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(controller.value.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      await gesture.removePointer();
    }, variant: TargetPlatformVariant.only(TargetPlatform.macOS));

    testWidgets('can click to set position', (WidgetTester tester) async {
      const String defaultText = 'initial text';
      final SearchController controller = SearchController();
      addTearDown(controller.dispose);
      controller.text = defaultText;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SearchAnchor.bar(
              searchController: controller,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );

      expect(controller.value.text, defaultText);
      expect(find.text(defaultText), findsOneWidget);

      final TestGesture gesture = await _pointGestureToSearchBar(tester);
      await gesture.down(textOffsetToPosition(tester, 2) + const Offset(0.0, -9.0));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);
      expect(controller.value.selection, const TextSelection.collapsed(offset: 2));

      await gesture.down(textOffsetToPosition(tester, 9, index: 1) + const Offset(0.0, -9.0));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(controller.value.selection, const TextSelection.collapsed(offset: 9));
      await gesture.removePointer();
    }, variant: TargetPlatformVariant.desktop());

    testWidgets('can double-click to select word', (WidgetTester tester) async {
      const String defaultText = 'initial text';
      final SearchController controller = SearchController();
      addTearDown(controller.dispose);
      controller.text = defaultText;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SearchAnchor.bar(
              searchController: controller,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );

      expect(controller.value.text, defaultText);
      expect(find.text(defaultText), findsOneWidget);

      final TestGesture gesture = await _pointGestureToSearchBar(tester);
      final Offset targetPosition = textOffsetToPosition(tester, 4) + const Offset(0.0, -9.0);
      await gesture.down(targetPosition);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      final Offset targetPositionAfterViewOpened =
          textOffsetToPosition(tester, 4, index: 1) + const Offset(0.0, -9.0);
      await gesture.down(targetPositionAfterViewOpened);
      await tester.pumpAndSettle();
      await gesture.up();
      await tester.pump();

      await gesture.down(targetPositionAfterViewOpened);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      expect(controller.value.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      await gesture.removePointer();
    }, variant: TargetPlatformVariant.desktop());

    testWidgets('can triple-click to select field', (WidgetTester tester) async {
      const String defaultText = 'initial text';
      final SearchController controller = SearchController();
      addTearDown(controller.dispose);
      controller.text = defaultText;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SearchAnchor.bar(
              searchController: controller,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );

      expect(controller.value.text, defaultText);
      expect(find.text(defaultText), findsOneWidget);

      final TestGesture gesture = await _pointGestureToSearchBar(tester);
      final Offset targetPosition = textOffsetToPosition(tester, 4) + const Offset(0.0, -9.0);
      await gesture.down(targetPosition);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      final Offset targetPositionAfterViewOpened =
          textOffsetToPosition(tester, 4, index: 1) + const Offset(0.0, -9.0);
      await gesture.down(targetPositionAfterViewOpened);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(targetPositionAfterViewOpened);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      await gesture.down(targetPositionAfterViewOpened);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(controller.value.selection, const TextSelection(baseOffset: 0, extentOffset: 12));
      await gesture.removePointer();
    }, variant: TargetPlatformVariant.desktop());
  });

  // Regression tests for https://github.com/flutter/flutter/issues/126623
  group('Overall InputDecorationTheme does not impact SearchBar and SearchView', () {
    const InputDecorationTheme inputDecorationTheme = InputDecorationTheme(
      focusColor: Colors.green,
      hoverColor: Colors.blue,
      outlineBorder: BorderSide(color: Colors.pink, width: 10),
      isDense: true,
      contentPadding: EdgeInsets.symmetric(horizontal: 20),
      hintStyle: TextStyle(color: Colors.purpleAccent),
      fillColor: Colors.tealAccent,
      filled: true,
      isCollapsed: true,
      border: OutlineInputBorder(),
      focusedBorder: UnderlineInputBorder(),
      enabledBorder: UnderlineInputBorder(),
      errorBorder: UnderlineInputBorder(),
      focusedErrorBorder: UnderlineInputBorder(),
      disabledBorder: UnderlineInputBorder(),
      constraints: BoxConstraints(maxWidth: 300),
    );
    final ThemeData theme = ThemeData(
      useMaterial3: true,
      inputDecorationTheme: inputDecorationTheme,
    );

    void checkDecorationInSearchBar(WidgetTester tester) {
      final Finder textField = findTextField();
      final InputDecoration? decoration = tester.widget<TextField>(textField).decoration;

      expect(decoration?.border, InputBorder.none);
      expect(decoration?.focusedBorder, InputBorder.none);
      expect(decoration?.enabledBorder, InputBorder.none);
      expect(decoration?.errorBorder, null);
      expect(decoration?.focusedErrorBorder, null);
      expect(decoration?.disabledBorder, null);
      expect(decoration?.constraints, null);
      expect(decoration?.isCollapsed, false);
      expect(decoration?.filled, false);
      expect(decoration?.fillColor, null);
      expect(decoration?.focusColor, null);
      expect(decoration?.hoverColor, null);
      expect(decoration?.contentPadding, EdgeInsets.zero);
      expect(decoration?.hintStyle?.color, theme.colorScheme.onSurfaceVariant);
    }

    testWidgets('Overall InputDecorationTheme does not override text field style'
        ' in SearchBar', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: const Center(child: Material(child: SearchBar(hintText: 'hint text'))),
        ),
      );

      // Check input decoration in `SearchBar`
      checkDecorationInSearchBar(tester);

      // Check search bar defaults.
      final Finder searchBarMaterial = find.descendant(
        of: find.byType(SearchBar),
        matching: find.byType(Material),
      );

      final Material material = tester.widget<Material>(searchBarMaterial);
      checkSearchBarDefaults(tester, theme.colorScheme, material);
    });

    testWidgets('Overall InputDecorationTheme does not override text field style'
        ' in the search view route', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Scaffold(
            body: Material(
              child: Align(
                alignment: Alignment.topLeft,
                child: SearchAnchor(
                  viewHintText: 'hint text',
                  builder: (BuildContext context, SearchController controller) {
                    return const Icon(Icons.search);
                  },
                  suggestionsBuilder: (BuildContext context, SearchController controller) {
                    return <Widget>[];
                  },
                ),
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byIcon(Icons.search));
      await tester.pumpAndSettle();

      // Check input decoration in `SearchBar`
      checkDecorationInSearchBar(tester);

      // Check search bar defaults in search view route.
      final Finder searchBarMaterial =
          find
              .descendant(
                of: find.descendant(of: findViewContent(), matching: find.byType(SearchBar)),
                matching: find.byType(Material),
              )
              .first;

      final Material material = tester.widget<Material>(searchBarMaterial);
      expect(material.color, Colors.transparent);
      expect(material.elevation, 0.0);
      final Text hintText = tester.widget(find.text('hint text'));
      expect(hintText.style?.color, theme.colorScheme.onSurfaceVariant);

      const String input = 'entered text';
      await tester.enterText(find.byType(SearchBar), input);
      final EditableText inputText = tester.widget(find.text(input));
      expect(inputText.style.color, theme.colorScheme.onSurface);
    });
  });

  testWidgets('SearchAnchor view respects theme brightness', (WidgetTester tester) async {
    Widget buildSearchAnchor(ThemeData theme) {
      return MaterialApp(
        theme: theme,
        home: Center(
          child: Material(
            child: SearchAnchor(
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.ac_unit),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    ThemeData theme = ThemeData(brightness: Brightness.light);
    await tester.pumpWidget(buildSearchAnchor(theme));

    // Open the search view.
    await tester.tap(find.widgetWithIcon(IconButton, Icons.ac_unit));
    await tester.pumpAndSettle();

    // Test the search view background color.
    Material material = getSearchViewMaterial(tester);
    expect(material.color, theme.colorScheme.surfaceContainerHigh);

    // Change the theme brightness.
    theme = ThemeData(brightness: Brightness.dark);
    await tester.pumpWidget(buildSearchAnchor(theme));
    await tester.pumpAndSettle();

    // Test the search view background color.
    material = getSearchViewMaterial(tester);
    expect(material.color, theme.colorScheme.surfaceContainerHigh);
  });

  testWidgets('Search view widgets can inherit local themes', (WidgetTester tester) async {
    final ThemeData globalTheme = ThemeData(colorSchemeSeed: Colors.red);
    final ThemeData localTheme = ThemeData(
      colorSchemeSeed: Colors.green,
      iconButtonTheme: IconButtonThemeData(
        style: IconButton.styleFrom(backgroundColor: const Color(0xffffff00)),
      ),
      cardTheme: const CardThemeData(color: Color(0xff00ffff)),
    );
    Widget buildSearchAnchor() {
      return MaterialApp(
        theme: globalTheme,
        home: Center(
          child: Builder(
            builder: (BuildContext context) {
              return Theme(
                data: localTheme,
                child: Material(
                  child: SearchAnchor.bar(
                    suggestionsBuilder: (BuildContext context, SearchController controller) {
                      return <Widget>[
                        Card(child: ListTile(onTap: () {}, title: const Text('Item 1'))),
                      ];
                    },
                  ),
                ),
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSearchAnchor());

    // Open the search view.
    await tester.tap(find.byType(SearchBar));
    await tester.pumpAndSettle();

    // Test the search view background color.
    final Material searchViewMaterial = getSearchViewMaterial(tester);
    expect(searchViewMaterial.color, localTheme.colorScheme.surfaceContainerHigh);

    // Test the search view icons background color.
    final Material iconButtonMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(IconButton), matching: find.byType(Material)).first,
    );
    expect(find.byWidget(iconButtonMaterial), findsOneWidget);
    expect(
      iconButtonMaterial.color,
      localTheme.iconButtonTheme.style?.backgroundColor?.resolve(<MaterialState>{}),
    );

    // Test the suggestion card color.
    final Material suggestionMaterial = tester.widget<Material>(
      find.descendant(of: find.byType(Card), matching: find.byType(Material)).first,
    );
    expect(suggestionMaterial.color, localTheme.cardTheme.color);
  });

  testWidgets('SearchBar respects keyboardType property', (WidgetTester tester) async {
    Widget buildSearchBar(TextInputType keyboardType) {
      return MaterialApp(
        home: Center(child: Material(child: SearchBar(keyboardType: keyboardType))),
      );
    }

    await tester.pumpWidget(buildSearchBar(TextInputType.number));
    await tester.pump();
    TextField textField = tester.widget(find.byType(TextField));
    expect(textField.keyboardType, TextInputType.number);

    await tester.pumpWidget(buildSearchBar(TextInputType.phone));
    await tester.pump();
    textField = tester.widget(find.byType(TextField));
    expect(textField.keyboardType, TextInputType.phone);
  });

  testWidgets('SearchAnchor respects keyboardType property', (WidgetTester tester) async {
    Widget buildSearchAnchor(TextInputType keyboardType) {
      return MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor(
              keyboardType: keyboardType,
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.ac_unit),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSearchAnchor(TextInputType.number));
    await tester.pump();
    await tester.tap(find.widgetWithIcon(IconButton, Icons.ac_unit));
    await tester.pumpAndSettle();
    TextField textField = tester.widget(find.byType(TextField));
    expect(textField.keyboardType, TextInputType.number);
    await tester.tap(find.backButton());
    await tester.pump();

    await tester.pumpWidget(buildSearchAnchor(TextInputType.phone));
    await tester.pump();
    await tester.tap(find.widgetWithIcon(IconButton, Icons.ac_unit));
    await tester.pumpAndSettle();
    textField = tester.widget(find.byType(TextField));
    expect(textField.keyboardType, TextInputType.phone);
  });

  testWidgets('SearchAnchor.bar respects keyboardType property', (WidgetTester tester) async {
    Widget buildSearchAnchor(TextInputType keyboardType) {
      return MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor.bar(
              keyboardType: keyboardType,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSearchAnchor(TextInputType.number));
    await tester.pump();
    await tester.tap(find.byType(SearchBar)); // Open search view.
    await tester.pumpAndSettle();
    final Finder textFieldFinder = find.descendant(
      of: findViewContent(),
      matching: find.byType(TextField),
    );
    final TextField textFieldInView = tester.widget<TextField>(textFieldFinder);
    expect(textFieldInView.keyboardType, TextInputType.number);
    // Close search view.
    await tester.tap(find.backButton());
    await tester.pumpAndSettle();
    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.keyboardType, TextInputType.number);
  });

  testWidgets('SearchBar respects textInputAction property', (WidgetTester tester) async {
    Widget buildSearchBar(TextInputAction textInputAction) {
      return MaterialApp(
        home: Center(child: Material(child: SearchBar(textInputAction: textInputAction))),
      );
    }

    await tester.pumpWidget(buildSearchBar(TextInputAction.previous));
    await tester.pump();
    TextField textField = tester.widget(find.byType(TextField));
    expect(textField.textInputAction, TextInputAction.previous);

    await tester.pumpWidget(buildSearchBar(TextInputAction.send));
    await tester.pump();
    textField = tester.widget(find.byType(TextField));
    expect(textField.textInputAction, TextInputAction.send);
  });

  testWidgets('SearchAnchor respects textInputAction property', (WidgetTester tester) async {
    Widget buildSearchAnchor(TextInputAction textInputAction) {
      return MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor(
              textInputAction: textInputAction,
              builder: (BuildContext context, SearchController controller) {
                return IconButton(
                  icon: const Icon(Icons.ac_unit),
                  onPressed: () {
                    controller.openView();
                  },
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSearchAnchor(TextInputAction.previous));
    await tester.pump();
    await tester.tap(find.widgetWithIcon(IconButton, Icons.ac_unit));
    await tester.pumpAndSettle();
    TextField textField = tester.widget(find.byType(TextField));
    expect(textField.textInputAction, TextInputAction.previous);
    await tester.tap(find.backButton());
    await tester.pump();

    await tester.pumpWidget(buildSearchAnchor(TextInputAction.send));
    await tester.pump();
    await tester.tap(find.widgetWithIcon(IconButton, Icons.ac_unit));
    await tester.pumpAndSettle();
    textField = tester.widget(find.byType(TextField));
    expect(textField.textInputAction, TextInputAction.send);
  });

  testWidgets('SearchAnchor.bar respects textInputAction property', (WidgetTester tester) async {
    Widget buildSearchAnchor(TextInputAction textInputAction) {
      return MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor.bar(
              textInputAction: textInputAction,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSearchAnchor(TextInputAction.previous));
    await tester.pump();
    await tester.tap(find.byType(SearchBar)); // Open search view.
    await tester.pumpAndSettle();
    final Finder textFieldFinder = find.descendant(
      of: findViewContent(),
      matching: find.byType(TextField),
    );
    final TextField textFieldInView = tester.widget<TextField>(textFieldFinder);
    expect(textFieldInView.textInputAction, TextInputAction.previous);
    // Close search view.
    await tester.tap(find.backButton());
    await tester.pumpAndSettle();
    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.textInputAction, TextInputAction.previous);
  });

  testWidgets('Block entering text on disabled widget', (WidgetTester tester) async {
    const String initValue = 'init';
    final TextEditingController controller = TextEditingController(text: initValue);
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: SearchBar(controller: controller, enabled: false))),
      ),
    );

    const String testValue = 'abcdefghi';
    await tester.enterText(find.byType(SearchBar), testValue);
    expect(controller.value.text, initValue);
  });

  testWidgets('Disabled SearchBar semantics node still contains value', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TextEditingController controller = TextEditingController(text: 'text');
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: SearchBar(controller: controller, enabled: false))),
      ),
    );

    expect(semantics, includesNodeWith(actions: <SemanticsAction>[], value: 'text'));
    semantics.dispose();
  });

  testWidgets('Check SearchBar opacity when disabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: SearchBar(enabled: false)))),
    );

    final Finder searchBarFinder = find.byType(SearchBar);
    expect(searchBarFinder, findsOneWidget);
    final Finder opacityFinder = find.descendant(
      of: searchBarFinder,
      matching: find.byType(Opacity),
    );
    expect(opacityFinder, findsOneWidget);
    final Opacity opacityWidget = tester.widget<Opacity>(opacityFinder);
    expect(opacityWidget.opacity, 0.38);
  });

  testWidgets('Check SearchAnchor opacity when disabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor(
              enabled: false,
              builder: (BuildContext context, SearchController controller) {
                return const Icon(Icons.search);
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );

    final Finder searchBarFinder = find.byType(SearchAnchor);
    expect(searchBarFinder, findsOneWidget);
    final Finder opacityFinder = find.descendant(
      of: searchBarFinder,
      matching: find.byType(AnimatedOpacity),
    );
    expect(opacityFinder, findsOneWidget);
    final AnimatedOpacity opacityWidget = tester.widget<AnimatedOpacity>(opacityFinder);
    expect(opacityWidget.opacity, 0.38);
  });

  testWidgets('SearchAnchor tap failed when disabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor(
              enabled: false,
              builder: (BuildContext context, SearchController controller) {
                return const Icon(Icons.search);
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );

    final Finder searchBarFinder = find.byType(SearchAnchor);
    expect(searchBarFinder, findsOneWidget);
    expect(searchBarFinder.hitTestable().tryEvaluate(), false);
  });

  testWidgets('SearchAnchor respects headerHeight', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor(
              isFullScreen: true,
              builder: (BuildContext context, SearchController controller) {
                return const Icon(Icons.search);
              },
              headerHeight: 32,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );
    await tester.pump();
    await tester.tap(find.byIcon(Icons.search)); // Open search view.
    await tester.pumpAndSettle();
    final Finder findHeader = find.descendant(
      of: findViewContent(),
      matching: find.byType(SearchBar),
    );
    expect(tester.getSize(findHeader).height, 32);
  });

  testWidgets('SearchAnchor.bar respects viewHeaderHeight', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor.bar(
              isFullScreen: true,
              viewHeaderHeight: 32,
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );
    await tester.pump();
    await tester.tap(find.byType(SearchBar)); // Open search view.
    await tester.pumpAndSettle();
    final Finder findHeader = find.descendant(
      of: findViewContent(),
      matching: find.byType(SearchBar),
    );
    final RenderBox box = tester.renderObject(findHeader);
    expect(box.size.height, 32);
  });

  testWidgets(
    'Tapping outside searchbar should unfocus the searchbar on mobile',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode(debugLabel: 'Test Node');
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SearchAnchor(
              builder: (BuildContext context, SearchController controller) {
                return SearchBar(
                  controller: controller,
                  onTap: () {
                    controller.openView();
                  },
                  onTapOutside: (PointerDownEvent event) {
                    focusNode.unfocus();
                  },
                  onChanged: (_) {
                    controller.openView();
                  },
                  autoFocus: true,
                  focusNode: focusNode,
                );
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return List<ListTile>.generate(5, (int index) {
                  final String item = 'item $index';
                  return ListTile(title: Text(item));
                });
              },
            ),
          ),
        ),
      );
      await tester.pump();
      expect(focusNode.hasPrimaryFocus, isTrue);

      await tester.tapAt(const Offset(50, 50));
      await tester.pump();

      expect(focusNode.hasPrimaryFocus, isFalse);
    },
    variant: TargetPlatformVariant.mobile(),
  );

  testWidgets('The default clear button only shows when text input is not empty '
      'on the search view', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: Material(
            child: SearchAnchor(
              builder: (BuildContext context, SearchController controller) {
                return const Icon(Icons.search);
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );
    await tester.pump();
    await tester.tap(find.byIcon(Icons.search)); // Open search view.
    await tester.pumpAndSettle();

    expect(find.widgetWithIcon(IconButton, Icons.close), findsNothing);
    await tester.enterText(findTextField(), 'a');
    await tester.pump();
    expect(find.widgetWithIcon(IconButton, Icons.close), findsOneWidget);
    await tester.enterText(findTextField(), '');
    await tester.pump();
    expect(find.widgetWithIcon(IconButton, Icons.close), findsNothing);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/139880.
  testWidgets('suggestionsBuilder with Future is not called twice on layout resize', (
    WidgetTester tester,
  ) async {
    int suggestionsLoadingCount = 0;

    Future<List<String>> createListData() async {
      return List<String>.generate(1000, (int index) {
        return 'Hello World - $index';
      });
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SearchAnchor(
              builder: (BuildContext context, SearchController controller) {
                return const Icon(Icons.search);
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[
                  FutureBuilder<List<String>>(
                    future: createListData(),
                    builder: (BuildContext context, AsyncSnapshot<List<String>> snapshot) {
                      if (snapshot.connectionState != ConnectionState.done) {
                        return const LinearProgressIndicator();
                      }
                      final List<String>? result = snapshot.data;
                      if (result == null) {
                        return const LinearProgressIndicator();
                      }
                      suggestionsLoadingCount++;
                      return SingleChildScrollView(
                        child: Column(
                          children:
                              result.map((String text) {
                                return ListTile(title: Text(text));
                              }).toList(),
                        ),
                      );
                    },
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );
    await tester.pump();
    await tester.tap(find.byIcon(Icons.search)); // Open search view.
    await tester.pumpAndSettle();

    // Simulate the keyboard opening resizing the view.
    tester.view.viewInsets = const FakeViewPadding(bottom: 500.0);
    addTearDown(tester.view.reset);
    await tester.pumpAndSettle();

    expect(suggestionsLoadingCount, 1);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/139880.
  testWidgets('suggestionsBuilder is not called when the search value does not change', (
    WidgetTester tester,
  ) async {
    int suggestionsBuilderCalledCount = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SearchAnchor(
              builder: (BuildContext context, SearchController controller) {
                return const Icon(Icons.search);
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                suggestionsBuilderCalledCount++;
                return <Widget>[];
              },
            ),
          ),
        ),
      ),
    );
    await tester.pump();
    await tester.tap(find.byIcon(Icons.search)); // Open search view.
    await tester.pumpAndSettle();

    // Simulate the keyboard opening resizing the view.
    tester.view.viewInsets = const FakeViewPadding(bottom: 500.0);
    addTearDown(tester.view.reset);
    // Show the keyboard.
    await tester.showKeyboard(find.byType(TextField));
    await tester.pumpAndSettle();

    expect(suggestionsBuilderCalledCount, 2);

    // Remove the viewInset, as if the keyboard were hidden.
    tester.view.resetViewInsets();
    // Hide the keyboard.
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pumpAndSettle();

    expect(suggestionsBuilderCalledCount, 2);
  });

  testWidgets('Suggestions gets refreshed after long API call', (WidgetTester tester) async {
    Timer? debounceTimer;
    const Duration apiCallDuration = Duration(seconds: 1);
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SearchAnchor(
              builder: (BuildContext context, SearchController controller) {
                return const Icon(Icons.search);
              },
              suggestionsBuilder: (BuildContext context, SearchController controller) async {
                final Completer<List<String>> completer = Completer<List<String>>();
                debounceTimer?.cancel();
                debounceTimer = Timer(apiCallDuration, () {
                  completer.complete(List<String>.generate(10, (int index) => 'Item - $index'));
                });
                final List<String> options = await completer.future;

                final List<Widget> suggestions = List<Widget>.generate(options.length, (int index) {
                  final String item = options[index];
                  return ListTile(title: Text(item));
                });
                return suggestions;
              },
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.byIcon(Icons.search)); // Open search view.
    await tester.pumpAndSettle();

    // Simulate the keyboard opening resizing the view.
    tester.view.viewInsets = const FakeViewPadding(bottom: 500.0);
    addTearDown(tester.view.reset);

    // Show the keyboard.
    await tester.showKeyboard(find.byType(TextField));
    await tester.pumpAndSettle(apiCallDuration);

    expect(find.text('Item - 1'), findsOneWidget);
  });

  testWidgets('SearchBar.scrollPadding is passed through to EditableText', (
    WidgetTester tester,
  ) async {
    const EdgeInsets scrollPadding = EdgeInsets.zero;
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SearchBar(scrollPadding: scrollPadding))),
    );

    expect(find.byType(EditableText), findsOneWidget);
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.scrollPadding, scrollPadding);
  });

  testWidgets('SearchAnchor.bar.scrollPadding is passed through to EditableText', (
    WidgetTester tester,
  ) async {
    const EdgeInsets scrollPadding = EdgeInsets.zero;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor.bar(
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
            scrollPadding: scrollPadding,
          ),
        ),
      ),
    );

    expect(find.byType(EditableText), findsOneWidget);
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.scrollPadding, scrollPadding);
  });

  group('contextMenuBuilder', () {
    setUp(() async {
      if (!kIsWeb) {
        return;
      }
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.contextMenu,
        (MethodCall call) {
          // Just complete successfully, so that BrowserContextMenu thinks that
          // the engine successfully received its call.
          return Future<void>.value();
        },
      );
      await BrowserContextMenu.disableContextMenu();
    });

    tearDown(() async {
      if (!kIsWeb) {
        return;
      }
      await BrowserContextMenu.enableContextMenu();
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.contextMenu,
        null,
      );
    });

    testWidgets('SearchAnchor.bar.contextMenuBuilder is passed through to EditableText', (
      WidgetTester tester,
    ) async {
      Widget contextMenuBuilder(BuildContext context, EditableTextState editableTextState) {
        return const Placeholder();
      }

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SearchAnchor.bar(
              suggestionsBuilder: (BuildContext context, SearchController controller) {
                return <Widget>[];
              },
              contextMenuBuilder: contextMenuBuilder,
            ),
          ),
        ),
      );

      expect(find.byType(EditableText), findsOneWidget);
      final EditableText editableText = tester.widget(find.byType(EditableText));
      expect(editableText.contextMenuBuilder, contextMenuBuilder);

      expect(find.byType(Placeholder), findsNothing);

      await tester.tap(find.byType(SearchBar), buttons: kSecondaryButton);
      await tester.pumpAndSettle();

      expect(find.byType(Placeholder), findsOneWidget);
    });
  });

  testWidgets('SearchAnchor does not dispose external SearchController', (
    WidgetTester tester,
  ) async {
    final SearchController controller = SearchController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SearchAnchor(
            searchController: controller,
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                onPressed: () async {
                  controller.openView();
                },
                icon: const Icon(Icons.search),
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    await tester.pumpWidget(const MaterialApp(home: Material(child: Text('disposed'))));
    expect(tester.takeException(), isNull);
    ChangeNotifier.debugAssertNotDisposed(controller);
  });

  testWidgets('SearchAnchor gracefully closes its search view when disposed', (
    WidgetTester tester,
  ) async {
    bool disposed = false;
    late StateSetter setState;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter stateSetter) {
              setState = stateSetter;
              if (disposed) {
                return const Text('disposed');
              }
              return SearchAnchor(
                builder: (BuildContext context, SearchController controller) {
                  return IconButton(
                    onPressed: () async {
                      controller.openView();
                    },
                    icon: const Icon(Icons.search),
                  );
                },
                suggestionsBuilder: (BuildContext context, SearchController controller) {
                  return <Widget>[const Text('suggestion')];
                },
              );
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    setState(() {
      disposed = true;
    });
    await tester.pump();
    // The search menu starts to close but is not disposed yet.
    final EditableText editableText = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableText.controller;
    ChangeNotifier.debugAssertNotDisposed(controller);

    await tester.pumpAndSettle();
    // The search menu and the internal search controller are now disposed.
    expect(tester.takeException(), isNull);
    expect(find.byType(TextField), findsNothing);
    FlutterError? error;
    try {
      ChangeNotifier.debugAssertNotDisposed(controller);
    } on FlutterError catch (e) {
      error = e;
    }
    expect(error, isNotNull);
    expect(error, isFlutterError);
    expect(
      error!.toStringDeep(),
      equalsIgnoringHashCodes(
        'FlutterError\n'
        '   A SearchController was used after being disposed.\n'
        '   Once you have called dispose() on a SearchController, it can no\n'
        '   longer be used.\n',
      ),
    );
  });

  // Regression test for https://github.com/flutter/flutter/issues/155180.
  testWidgets('disposing SearchAnchor during search view exit animation does not crash', (
    WidgetTester tester,
  ) async {
    final GlobalKey<NavigatorState> key = GlobalKey<NavigatorState>();
    await tester.pumpWidget(
      MaterialApp(
        navigatorKey: key,
        home: Material(
          child: SearchAnchor(
            builder: (BuildContext context, SearchController controller) {
              return IconButton(
                onPressed: () async {
                  controller.openView();
                },
                icon: const Icon(Icons.search),
              );
            },
            suggestionsBuilder: (BuildContext context, SearchController controller) {
              return <Widget>[const Text('suggestion')];
            },
          ),
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.search));
    await tester.pumpAndSettle();
    key.currentState!.pop();
    await tester.pump();
    await tester.pumpWidget(
      MaterialApp(navigatorKey: key, home: const Material(child: Text('disposed'))),
    );
    await tester.pump();
    expect(tester.takeException(), isNull);
  });
}

Future<void> checkSearchBarDefaults(
  WidgetTester tester,
  ColorScheme colorScheme,
  Material material,
) async {
  expect(material.animationDuration, const Duration(milliseconds: 200));
  expect(material.borderOnForeground, true);
  expect(material.borderRadius, null);
  expect(material.clipBehavior, Clip.none);
  expect(material.color, colorScheme.surfaceContainerHigh);
  expect(material.elevation, 6.0);
  expect(material.shadowColor, colorScheme.shadow);
  expect(material.surfaceTintColor, Colors.transparent);
  expect(material.shape, const StadiumBorder());

  final Text helperText = tester.widget(find.text('hint text'));
  expect(helperText.style?.color, colorScheme.onSurfaceVariant);
  expect(helperText.style?.fontSize, 16.0);
  expect(helperText.style?.fontFamily, 'Roboto');
  expect(helperText.style?.fontWeight, FontWeight.w400);

  const String input = 'entered text';
  await tester.enterText(find.byType(SearchBar), input);
  final EditableText inputText = tester.widget(find.text(input));
  expect(inputText.style.color, colorScheme.onSurface);
  expect(inputText.style.fontSize, 16.0);
  expect(helperText.style?.fontFamily, 'Roboto');
  expect(inputText.style.fontWeight, FontWeight.w400);
}

Finder findTextField() {
  return find.descendant(of: find.byType(SearchBar), matching: find.byType(TextField));
}

TextStyle? _iconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
  );
  return iconRichText.text.style;
}

const Color pressedColor = Colors.red;
const Color hoveredColor = Colors.orange;
const Color focusedColor = Colors.yellow;
const Color defaultColor = Colors.green;

Color _getColor(Set<MaterialState> states) {
  if (states.contains(MaterialState.pressed)) {
    return pressedColor;
  }
  if (states.contains(MaterialState.hovered)) {
    return hoveredColor;
  }
  if (states.contains(MaterialState.focused)) {
    return focusedColor;
  }
  return defaultColor;
}

final ThemeData theme = ThemeData();
final TextStyle? pressedStyle = theme.textTheme.bodyLarge?.copyWith(color: pressedColor);
final TextStyle? hoveredStyle = theme.textTheme.bodyLarge?.copyWith(color: hoveredColor);
final TextStyle? focusedStyle = theme.textTheme.bodyLarge?.copyWith(color: focusedColor);

TextStyle? _getTextStyle(Set<MaterialState> states) {
  if (states.contains(MaterialState.pressed)) {
    return pressedStyle;
  }
  if (states.contains(MaterialState.hovered)) {
    return hoveredStyle;
  }
  if (states.contains(MaterialState.focused)) {
    return focusedStyle;
  }
  return null;
}

Future<TestGesture> _pointGestureToSearchBar(WidgetTester tester) async {
  final Offset center = tester.getCenter(find.byType(SearchBar));
  final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);

  // On hovered.
  await gesture.addPointer();
  await gesture.moveTo(center);
  return gesture;
}

Finder findViewContent() {
  return find.byWidgetPredicate((Widget widget) {
    return widget.runtimeType.toString() == '_ViewContent';
  });
}

Material getSearchViewMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: findViewContent(), matching: find.byType(Material)).first,
  );
}
