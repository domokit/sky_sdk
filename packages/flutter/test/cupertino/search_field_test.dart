// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('default search field has a border radius', (WidgetTester tester) async {
    await tester.pumpWidget(const CupertinoApp(home: Center(child: CupertinoSearchTextField())));

    final BoxDecoration decoration =
        tester
                .widget<DecoratedBox>(
                  find.descendant(
                    of: find.byType(CupertinoSearchTextField),
                    matching: find.byType(DecoratedBox),
                  ),
                )
                .decoration
            as BoxDecoration;

    expect(decoration.borderRadius, const BorderRadius.all(Radius.circular(9)));
  });

  testWidgets('decoration overrides default background color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            decoration: BoxDecoration(color: Color.fromARGB(1, 1, 1, 1)),
          ),
        ),
      ),
    );

    final BoxDecoration decoration =
        tester
                .widget<DecoratedBox>(
                  find.descendant(
                    of: find.byType(CupertinoSearchTextField),
                    matching: find.byType(DecoratedBox),
                  ),
                )
                .decoration
            as BoxDecoration;

    expect(decoration.color, const Color.fromARGB(1, 1, 1, 1));
  });

  testWidgets('decoration overrides default border radius', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            decoration: BoxDecoration(borderRadius: BorderRadius.zero),
          ),
        ),
      ),
    );

    final BoxDecoration decoration =
        tester
                .widget<DecoratedBox>(
                  find.descendant(
                    of: find.byType(CupertinoSearchTextField),
                    matching: find.byType(DecoratedBox),
                  ),
                )
                .decoration
            as BoxDecoration;

    expect(decoration.borderRadius, BorderRadius.zero);
  });

  testWidgets('text entries are padded by default', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'initial');
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoSearchTextField(controller: controller))),
    );

    expect(
      tester.getTopLeft(find.text('initial')) -
          tester.getTopLeft(find.byType(CupertinoSearchTextField)),
      const Offset(31.5, 8.0),
    );
  });

  testWidgets('can change keyboard type', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(keyboardType: TextInputType.number)),
      ),
    );
    await tester.tap(find.byType(CupertinoSearchTextField));
    await tester.showKeyboard(find.byType(CupertinoSearchTextField));
    expect(
      (tester.testTextInput.setClientArgs!['inputType'] as Map<String, dynamic>)['name'],
      equals('TextInputType.number'),
    );
  });

  testWidgets('can control text content via controller', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoSearchTextField(controller: controller))),
    );

    controller.text = 'controller text';
    await tester.pump();

    expect(find.text('controller text'), findsOneWidget);

    controller.text = '';
    await tester.pump();

    expect(find.text('controller text'), findsNothing);
  });

  testWidgets('placeholder color', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        theme: CupertinoThemeData(brightness: Brightness.dark),
        home: Center(child: CupertinoSearchTextField()),
      ),
    );

    Text placeholder = tester.widget(find.text('Search'));
    expect(placeholder.style!.color!.value, CupertinoColors.systemGrey.darkColor.value);

    await tester.pumpAndSettle();

    await tester.pumpWidget(
      const CupertinoApp(
        theme: CupertinoThemeData(brightness: Brightness.light),
        home: Center(child: CupertinoSearchTextField()),
      ),
    );

    placeholder = tester.widget(find.text('Search'));
    expect(placeholder.style!.color!.value, CupertinoColors.systemGrey.color.value);
  });

  testWidgets("placeholderStyle modifies placeholder's style and doesn't affect text's style", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            placeholder: 'placeholder',
            style: TextStyle(color: Color(0x00FFFFFF), fontWeight: FontWeight.w300),
            placeholderStyle: TextStyle(color: Color(0xAAFFFFFF), fontWeight: FontWeight.w600),
          ),
        ),
      ),
    );

    final Text placeholder = tester.widget(find.text('placeholder'));
    expect(placeholder.style!.color, const Color(0xAAFFFFFF));
    expect(placeholder.style!.fontWeight, FontWeight.w600);

    await tester.enterText(find.byType(CupertinoSearchTextField), 'input');
    await tester.pump();

    final EditableText inputText = tester.widget(find.text('input'));
    expect(inputText.style.color, const Color(0x00FFFFFF));
    expect(inputText.style.fontWeight, FontWeight.w300);
  });

  testWidgets('prefix widget is in front of the text', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'input');
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoSearchTextField(controller: controller))),
    );

    expect(
      tester.getTopRight(find.byIcon(CupertinoIcons.search)).dx + 5.5,
      tester.getTopLeft(find.byType(EditableText)).dx,
    );

    expect(
      tester.getTopLeft(find.byType(EditableText)).dx,
      tester.getTopLeft(find.byType(CupertinoSearchTextField)).dx +
          tester.getSize(find.byIcon(CupertinoIcons.search)).width +
          11.5,
    );
  });

  testWidgets('suffix widget is after the text', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'Hi');
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoSearchTextField(controller: controller))),
    );

    expect(
      tester.getTopRight(find.byType(EditableText)).dx + 5.5,
      tester.getTopLeft(find.byIcon(CupertinoIcons.xmark_circle_fill)).dx,
    );

    expect(
      tester.getTopRight(find.byType(EditableText)).dx,
      tester.getTopRight(find.byType(CupertinoSearchTextField)).dx -
          tester.getSize(find.byIcon(CupertinoIcons.xmark_circle_fill)).width -
          10.5,
    );
  });

  testWidgets('prefix widget visibility', (WidgetTester tester) async {
    const Key prefixIcon = Key('prefix');

    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            prefixIcon: SizedBox(key: prefixIcon, width: 50, height: 50),
          ),
        ),
      ),
    );

    expect(find.byIcon(CupertinoIcons.search), findsNothing);
    expect(find.byKey(prefixIcon), findsOneWidget);

    await tester.enterText(find.byType(CupertinoSearchTextField), 'text input');
    await tester.pump();

    expect(find.text('text input'), findsOneWidget);
    expect(find.byIcon(CupertinoIcons.search), findsNothing);
    expect(find.byKey(prefixIcon), findsOneWidget);
  });

  testWidgets('suffix widget respects visibility mode', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(suffixMode: OverlayVisibilityMode.notEditing)),
      ),
    );

    expect(find.byIcon(CupertinoIcons.xmark_circle_fill), findsOneWidget);

    await tester.enterText(find.byType(CupertinoSearchTextField), 'text input');
    await tester.pump();

    expect(find.text('text input'), findsOneWidget);
    expect(find.byIcon(CupertinoIcons.xmark_circle_fill), findsNothing);
  });

  testWidgets('clear button shows with right visibility mode', (WidgetTester tester) async {
    TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            controller: controller,
            placeholder: 'placeholder does not affect clear button',
          ),
        ),
      ),
    );

    expect(find.byIcon(CupertinoIcons.xmark_circle_fill), findsNothing);

    await tester.enterText(find.byType(CupertinoSearchTextField), 'text input');
    await tester.pump();

    expect(find.byIcon(CupertinoIcons.xmark_circle_fill), findsOneWidget);
    expect(find.text('text input'), findsOneWidget);

    controller = TextEditingController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            controller: controller,
            placeholder: 'placeholder does not affect clear button',
            suffixMode: OverlayVisibilityMode.notEditing,
          ),
        ),
      ),
    );
    expect(find.byIcon(CupertinoIcons.xmark_circle_fill), findsOneWidget);

    controller.text = 'input';
    await tester.pump();

    expect(find.byIcon(CupertinoIcons.xmark_circle_fill), findsNothing);
  });

  testWidgets('clear button removes text', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(home: Center(child: CupertinoSearchTextField(controller: controller))),
    );

    controller.text = 'text entry';
    await tester.pump();

    await tester.tap(find.byIcon(CupertinoIcons.xmark_circle_fill));
    await tester.pump();

    expect(controller.text, '');
    expect(find.text('Search'), findsOneWidget);
    expect(find.text('text entry'), findsNothing);
    expect(find.byIcon(CupertinoIcons.xmark_circle_fill), findsNothing);
  });

  testWidgets('tapping clear button also calls onChanged when text not empty', (
    WidgetTester tester,
  ) async {
    String value = 'text entry';
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            controller: controller,
            placeholder: 'placeholder',
            onChanged: (String newValue) => value = newValue,
          ),
        ),
      ),
    );

    controller.text = value;
    await tester.pump();

    await tester.tap(find.byIcon(CupertinoIcons.xmark_circle_fill));
    await tester.pump();

    expect(controller.text, isEmpty);
    expect(find.text('text entry'), findsNothing);
    expect(value, isEmpty);
  });

  testWidgets('RTL puts attachments to the right places', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Center(child: CupertinoSearchTextField(suffixMode: OverlayVisibilityMode.always)),
        ),
      ),
    );

    expect(tester.getTopLeft(find.byIcon(CupertinoIcons.search)).dx, 800.0 - 26.0);

    expect(tester.getTopRight(find.byIcon(CupertinoIcons.xmark_circle_fill)).dx, 25.0);
  });

  testWidgets('Can modify prefix and suffix insets', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            suffixMode: OverlayVisibilityMode.always,
            prefixInsets: EdgeInsets.zero,
            suffixInsets: EdgeInsets.zero,
          ),
        ),
      ),
    );

    expect(tester.getTopLeft(find.byIcon(CupertinoIcons.search)).dx, 0.0);

    expect(tester.getTopRight(find.byIcon(CupertinoIcons.xmark_circle_fill)).dx, 800.0);
  });

  testWidgets('custom suffix onTap overrides default clearing behavior', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = TextEditingController(text: 'Text');
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoSearchTextField(controller: controller, onSuffixTap: () {})),
      ),
    );

    await tester.pump();

    await tester.tap(find.byIcon(CupertinoIcons.xmark_circle_fill));
    await tester.pump();

    expect(controller.text, isNotEmpty);
    expect(find.text('Text'), findsOneWidget);
  });

  testWidgets('onTap is properly forwarded to the inner text field', (WidgetTester tester) async {
    int onTapCallCount = 0;

    // onTap can be null.
    await tester.pumpWidget(const CupertinoApp(home: Center(child: CupertinoSearchTextField())));

    // onTap callback is called if not null.
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(
          child: CupertinoSearchTextField(
            onTap: () {
              onTapCallCount++;
            },
          ),
        ),
      ),
    );

    expect(onTapCallCount, 0);
    await tester.tap(find.byType(CupertinoTextField));
    expect(onTapCallCount, 1);
  });

  testWidgets('autocorrect is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(home: Center(child: CupertinoSearchTextField(autocorrect: false))),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.autocorrect, false);
  });

  testWidgets('enabled is properly forwarded to the inner text field', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(home: Center(child: CupertinoSearchTextField(enabled: false))),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.enabled, false);
  });

  testWidgets('textInputAction is set to TextInputAction.search by default', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const CupertinoApp(home: Center(child: CupertinoSearchTextField())));

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.textInputAction, TextInputAction.search);
  });

  testWidgets('autofocus:true gives focus to the widget', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    await tester.pumpWidget(
      CupertinoApp(
        home: Center(child: CupertinoSearchTextField(focusNode: focusNode, autofocus: true)),
      ),
    );

    expect(focusNode.hasFocus, isTrue);
  });

  testWidgets('smartQuotesType is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(smartQuotesType: SmartQuotesType.disabled)),
      ),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.smartQuotesType, SmartQuotesType.disabled);
  });

  testWidgets('smartDashesType is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(smartDashesType: SmartDashesType.disabled)),
      ),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.smartDashesType, SmartDashesType.disabled);
  });

  testWidgets('enableIMEPersonalizedLearning is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(enableIMEPersonalizedLearning: false)),
      ),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.enableIMEPersonalizedLearning, false);
  });

  testWidgets('cursorWidth is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(home: Center(child: CupertinoSearchTextField(cursorWidth: 1))),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.cursorWidth, 1);
  });

  testWidgets('cursorHeight is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(home: Center(child: CupertinoSearchTextField(cursorHeight: 10))),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.cursorHeight, 10);
  });

  testWidgets('cursorRadius is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(cursorRadius: Radius.circular(1.0))),
      ),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.cursorRadius, const Radius.circular(1.0));
  });

  testWidgets('cursorOpacityAnimates is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(cursorOpacityAnimates: false)),
      ),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.cursorOpacityAnimates, false);
  });

  testWidgets('cursorColor is properly forwarded to the inner text field', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: Center(child: CupertinoSearchTextField(cursorColor: Color.fromARGB(255, 255, 0, 0))),
      ),
    );

    final CupertinoTextField textField = tester.widget(find.byType(CupertinoTextField));
    expect(textField.cursorColor, const Color.fromARGB(255, 255, 0, 0));
  });

  testWidgets('Icons and placeholder fade while resizing on scroll', (WidgetTester tester) async {
    await tester.pumpWidget(
      const CupertinoApp(
        home: CupertinoPageScaffold(
          child: CustomScrollView(
            slivers: <Widget>[
              SliverResizingHeader(
                child: CupertinoSearchTextField(suffixMode: OverlayVisibilityMode.always),
              ),
              SliverFillRemaining(),
            ],
          ),
        ),
      ),
    );

    final Finder searchTextFieldFinder = find.byType(CupertinoSearchTextField);
    expect(searchTextFieldFinder, findsOneWidget);

    final Finder prefixIconFinder = find.descendant(
      of: searchTextFieldFinder,
      matching: find.byIcon(CupertinoIcons.search),
    );
    final Finder suffixIconFinder = find.descendant(
      of: searchTextFieldFinder,
      matching: find.byIcon(CupertinoIcons.xmark_circle_fill),
    );
    final Finder placeholderFinder = find.descendant(
      of: searchTextFieldFinder,
      matching: find.text('Search'),
    );
    expect(prefixIconFinder, findsOneWidget);
    expect(suffixIconFinder, findsOneWidget);
    expect(placeholderFinder, findsOneWidget);

    // Initially, the icons and placeholder text are fully opaque.
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: prefixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      equals(1.0),
    );
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: suffixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      equals(1.0),
    );
    expect(tester.widget<Text>(placeholderFinder).style?.color?.a, equals(1.0));

    final double searchTextFieldHeight = tester.getSize(searchTextFieldFinder).height;

    final TestGesture scrollGesture1 = await tester.startGesture(
      tester.getCenter(find.byType(CustomScrollView)),
    );
    await scrollGesture1.moveBy(Offset(0, -searchTextFieldHeight / 5));
    await scrollGesture1.up();
    await tester.pumpAndSettle();

    // The icons and placeholder text start to fade.
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: prefixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      greaterThan(0.0),
    );
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: prefixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      lessThan(1.0),
    );
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: suffixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      greaterThan(0.0),
    );
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: suffixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      lessThan(1.0),
    );
    expect(tester.widget<Text>(placeholderFinder).style?.color?.a, greaterThan(0.0));
    expect(tester.widget<Text>(placeholderFinder).style?.color?.a, lessThan(1.0));

    final TestGesture scrollGesture2 = await tester.startGesture(
      tester.getCenter(find.byType(CustomScrollView)),
    );
    await scrollGesture2.moveBy(Offset(0, -4 * searchTextFieldHeight / 5));
    await scrollGesture2.up();
    await tester.pumpAndSettle();

    // The icons and placeholder text have faded completely.
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: prefixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      equals(0.0),
    );
    expect(
      tester
          .widget<Opacity>(find.ancestor(of: suffixIconFinder, matching: find.byType(Opacity)))
          .opacity,
      equals(0.0),
    );
    expect(tester.widget<Text>(placeholderFinder).style?.color?.a, equals(0.0));
  });

  testWidgets('Top padding animates while resizing on scroll', (WidgetTester tester) async {
    const TextDirection direction = TextDirection.ltr;
    await tester.pumpWidget(
      const Directionality(
        textDirection: direction,
        child: CupertinoApp(
          home: CupertinoPageScaffold(
            child: CustomScrollView(
              slivers: <Widget>[
                SliverResizingHeader(child: CupertinoSearchTextField()),
                SliverFillRemaining(),
              ],
            ),
          ),
        ),
      ),
    );

    final Finder searchTextFieldFinder = find.byType(CupertinoSearchTextField);
    expect(searchTextFieldFinder, findsOneWidget);

    final double initialPadding =
        tester
            .widget<CupertinoTextField>(
              find.descendant(of: searchTextFieldFinder, matching: find.byType(CupertinoTextField)),
            )
            .padding
            .resolve(direction)
            .top;
    expect(initialPadding, equals(8.0));

    final double searchTextFieldHeight = tester.getSize(searchTextFieldFinder).height;

    final TestGesture scrollGesture = await tester.startGesture(
      tester.getCenter(find.byType(CustomScrollView)),
    );
    await scrollGesture.moveBy(Offset(0, -searchTextFieldHeight / 5));
    await scrollGesture.up();
    await tester.pumpAndSettle();

    expect(
      tester
          .widget<CupertinoTextField>(
            find.descendant(of: searchTextFieldFinder, matching: find.byType(CupertinoTextField)),
          )
          .padding
          .resolve(direction)
          .top,
      lessThan(initialPadding),
    );
  });
}
