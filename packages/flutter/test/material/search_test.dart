// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/clipboard_utils.dart';
import '../widgets/semantics_tester.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();

  setUp(() async {
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
  });

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  testWidgets('Changing query moves cursor to the end of query', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 300));

    delegate.query = 'Foo';

    final TextField textField = tester.widget<TextField>(find.byType(TextField));

    expect(
      textField.controller!.selection,
      TextSelection(baseOffset: delegate.query.length, extentOffset: delegate.query.length),
    );
  });

  testWidgets('Can open and close search', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());
    final List<String> selectedResults = <String>[];

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));

    // We are on the homepage
    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('HomeTitle'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);

    // Open search
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('HomeTitle'), findsNothing);
    expect(find.text('Suggestions'), findsOneWidget);
    expect(selectedResults, hasLength(0));

    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.focusNode!.hasFocus, isTrue);

    // Close search
    await tester.tap(find.byTooltip('Back'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('HomeTitle'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);
    expect(selectedResults, <String>['Result']);
  });

  testWidgets('Can close search with system back button to return null', (
    WidgetTester tester,
  ) async {
    // regression test for https://github.com/flutter/flutter/issues/18145

    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());
    final List<String?> selectedResults = <String?>[];

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));

    // We are on the homepage
    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('HomeTitle'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);

    // Open search
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('HomeTitle'), findsNothing);
    expect(find.text('Suggestions'), findsOneWidget);
    expect(find.text('Bottom'), findsOneWidget);

    // Simulate system back button
    final ByteData message = const JSONMethodCodec().encodeMethodCall(const MethodCall('popRoute'));
    await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/navigation',
      message,
      (_) {},
    );
    await tester.pumpAndSettle();

    expect(selectedResults, <String?>[null]);

    // We are on the homepage again
    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('HomeTitle'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);

    // Open search again
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('HomeTitle'), findsNothing);
    expect(find.text('Suggestions'), findsOneWidget);
  });

  testWidgets('Hint text color overridden', (WidgetTester tester) async {
    const String searchHintText = 'Enter search terms';
    final _TestSearchDelegate delegate = _TestSearchDelegate(searchHint: searchHintText);
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final Text hintText = tester.widget(find.text(searchHintText));
    expect(hintText.style!.color, _TestSearchDelegate.hintTextColor);
  });

  testWidgets('Requests suggestions', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(delegate.query, '');
    expect(delegate.queriesForSuggestions.last, '');
    expect(delegate.queriesForResults, hasLength(0));

    // Type W o w into search field
    delegate.queriesForSuggestions.clear();
    await tester.enterText(find.byType(TextField), 'W');
    await tester.pumpAndSettle();
    expect(delegate.query, 'W');
    await tester.enterText(find.byType(TextField), 'Wo');
    await tester.pumpAndSettle();
    expect(delegate.query, 'Wo');
    await tester.enterText(find.byType(TextField), 'Wow');
    await tester.pumpAndSettle();
    expect(delegate.query, 'Wow');

    expect(delegate.queriesForSuggestions, <String>['W', 'Wo', 'Wow']);
    expect(delegate.queriesForResults, hasLength(0));
  });

  testWidgets('Shows Results and closes search', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());
    final List<String> selectedResults = <String>[];

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();
    await tester.enterText(find.byType(TextField), 'Wow');
    await tester.pumpAndSettle();
    await tester.tap(find.text('Suggestions'));
    await tester.pumpAndSettle();

    // We are on the results page for Wow
    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('HomeTitle'), findsNothing);
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Results'), findsOneWidget);

    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.focusNode!.hasFocus, isFalse);
    expect(delegate.queriesForResults, <String>['Wow']);

    // Close search
    await tester.tap(find.byTooltip('Back'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('HomeTitle'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Results'), findsNothing);
    expect(selectedResults, <String>['Result']);
  });

  testWidgets('Can switch between results and suggestions', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    // Showing suggestions
    expect(find.text('Suggestions'), findsOneWidget);
    expect(find.text('Results'), findsNothing);

    // Typing query Wow
    delegate.queriesForSuggestions.clear();
    await tester.enterText(find.byType(TextField), 'Wow');
    await tester.pumpAndSettle();

    expect(delegate.query, 'Wow');
    expect(delegate.queriesForSuggestions, <String>['Wow']);
    expect(delegate.queriesForResults, hasLength(0));

    await tester.tap(find.text('Suggestions'));
    await tester.pumpAndSettle();

    // Showing Results
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Results'), findsOneWidget);

    expect(delegate.query, 'Wow');
    expect(delegate.queriesForSuggestions, <String>['Wow']);
    expect(delegate.queriesForResults, <String>['Wow']);

    TextField textField = tester.widget(find.byType(TextField));
    expect(textField.focusNode!.hasFocus, isFalse);

    // Tapping search field to go back to suggestions
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    textField = tester.widget(find.byType(TextField));
    expect(textField.focusNode!.hasFocus, isTrue);

    expect(find.text('Suggestions'), findsOneWidget);
    expect(find.text('Results'), findsNothing);
    expect(delegate.queriesForSuggestions, <String>['Wow', 'Wow']);
    expect(delegate.queriesForResults, <String>['Wow']);

    await tester.enterText(find.byType(TextField), 'Foo');
    await tester.pumpAndSettle();

    expect(delegate.query, 'Foo');
    expect(delegate.queriesForSuggestions, <String>['Wow', 'Wow', 'Foo']);
    expect(delegate.queriesForResults, <String>['Wow']);

    // Go to results again
    await tester.tap(find.text('Suggestions'));
    await tester.pumpAndSettle();

    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Results'), findsOneWidget);

    expect(delegate.query, 'Foo');
    expect(delegate.queriesForSuggestions, <String>['Wow', 'Wow', 'Foo']);
    expect(delegate.queriesForResults, <String>['Wow', 'Foo']);

    textField = tester.widget(find.byType(TextField));
    expect(textField.focusNode!.hasFocus, isFalse);
  });

  testWidgets('Fresh search always starts with empty query', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(delegate.query, '');

    delegate.query = 'Foo';
    await tester.tap(find.byTooltip('Back'));
    await tester.pumpAndSettle();
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(delegate.query, '');
  });

  testWidgets('Initial queries are honored', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    expect(delegate.query, '');

    await tester.pumpWidget(
      TestHomePage(delegate: delegate, passInInitialQuery: true, initialQuery: 'Foo'),
    );
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(delegate.query, 'Foo');
  });

  testWidgets('Initial query null re-used previous query', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    delegate.query = 'Foo';

    await tester.pumpWidget(TestHomePage(delegate: delegate, passInInitialQuery: true));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(delegate.query, 'Foo');
  });

  testWidgets('Changing query shows up in search field', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate, passInInitialQuery: true));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    delegate.query = 'Foo';

    expect(find.text('Foo'), findsOneWidget);
    expect(find.text('Bar'), findsNothing);

    delegate.query = 'Bar';

    expect(find.text('Foo'), findsNothing);
    expect(find.text('Bar'), findsOneWidget);
  });

  testWidgets('transitionAnimation runs while search fades in/out', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate, passInInitialQuery: true));

    // runs while search fades in
    expect(delegate.transitionAnimation.status, AnimationStatus.dismissed);
    await tester.tap(find.byTooltip('Search'));
    expect(delegate.transitionAnimation.status, AnimationStatus.forward);
    await tester.pumpAndSettle();
    expect(delegate.transitionAnimation.status, AnimationStatus.completed);

    // does not run while switching to results
    await tester.tap(find.text('Suggestions'));
    expect(delegate.transitionAnimation.status, AnimationStatus.completed);
    await tester.pumpAndSettle();
    expect(delegate.transitionAnimation.status, AnimationStatus.completed);

    // runs while search fades out
    await tester.tap(find.byTooltip('Back'));
    expect(delegate.transitionAnimation.status, AnimationStatus.reverse);
    await tester.pumpAndSettle();
    expect(delegate.transitionAnimation.status, AnimationStatus.dismissed);
  });

  testWidgets('Closing nested search returns to search', (WidgetTester tester) async {
    final List<String?> nestedSearchResults = <String?>[];
    final _TestSearchDelegate nestedSearchDelegate = _TestSearchDelegate(
      suggestions: 'Nested Suggestions',
      result: 'Nested Result',
    );
    addTearDown(nestedSearchDelegate.dispose);

    final List<String> selectedResults = <String>[];
    final _TestSearchDelegate delegate = _TestSearchDelegate(
      actions: <Widget>[
        Builder(
          builder: (BuildContext context) {
            return IconButton(
              tooltip: 'Nested Search',
              icon: const Icon(Icons.search),
              onPressed: () async {
                final String? result = await showSearch(
                  context: context,
                  delegate: nestedSearchDelegate,
                );
                nestedSearchResults.add(result);
              },
            );
          },
        ),
      ],
    );
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));
    expect(find.text('HomeBody'), findsOneWidget);
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('Suggestions'), findsOneWidget);
    expect(find.text('Nested Suggestions'), findsNothing);

    await tester.tap(find.byTooltip('Nested Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Nested Suggestions'), findsOneWidget);

    await tester.tap(find.byTooltip('Back'));
    await tester.pumpAndSettle();
    expect(nestedSearchResults, <String>['Nested Result']);

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('Suggestions'), findsOneWidget);
    expect(find.text('Nested Suggestions'), findsNothing);

    await tester.tap(find.byTooltip('Back'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Nested Suggestions'), findsNothing);
    expect(selectedResults, <String>['Result']);
  });

  testWidgets('Closing search with nested search shown goes back to underlying route', (
    WidgetTester tester,
  ) async {
    late _TestSearchDelegate delegate;
    addTearDown(() => delegate.dispose());
    final List<String?> nestedSearchResults = <String?>[];
    final _TestSearchDelegate nestedSearchDelegate = _TestSearchDelegate(
      suggestions: 'Nested Suggestions',
      result: 'Nested Result',
      actions: <Widget>[
        Builder(
          builder: (BuildContext context) {
            return IconButton(
              tooltip: 'Close Search',
              icon: const Icon(Icons.close),
              onPressed: () async {
                delegate.close(context, 'Result Foo');
              },
            );
          },
        ),
      ],
    );
    addTearDown(nestedSearchDelegate.dispose);

    final List<String> selectedResults = <String>[];
    delegate = _TestSearchDelegate(
      actions: <Widget>[
        Builder(
          builder: (BuildContext context) {
            return IconButton(
              tooltip: 'Nested Search',
              icon: const Icon(Icons.search),
              onPressed: () async {
                final String? result = await showSearch(
                  context: context,
                  delegate: nestedSearchDelegate,
                );
                nestedSearchResults.add(result);
              },
            );
          },
        ),
      ],
    );

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));

    expect(find.text('HomeBody'), findsOneWidget);
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('Suggestions'), findsOneWidget);
    expect(find.text('Nested Suggestions'), findsNothing);

    await tester.tap(find.byTooltip('Nested Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Nested Suggestions'), findsOneWidget);

    await tester.tap(find.byTooltip('Close Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('Nested Suggestions'), findsNothing);
    expect(nestedSearchResults, <String?>[null]);
    expect(selectedResults, <String>['Result Foo']);
  });

  testWidgets('Custom searchFieldLabel value', (WidgetTester tester) async {
    const String searchHint = 'custom search hint';
    final String defaultSearchHint = const DefaultMaterialLocalizations().searchFieldLabel;

    final _TestSearchDelegate delegate = _TestSearchDelegate(searchHint: searchHint);
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text(searchHint), findsOneWidget);
    expect(find.text(defaultSearchHint), findsNothing);
  });

  testWidgets('Default searchFieldLabel is used when it is set to null', (
    WidgetTester tester,
  ) async {
    final String searchHint = const DefaultMaterialLocalizations().searchFieldLabel;

    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text(searchHint), findsOneWidget);
  });

  testWidgets('Custom searchFieldStyle value', (WidgetTester tester) async {
    const String searchHintText = 'Enter search terms';
    const TextStyle searchFieldStyle = TextStyle(color: Colors.red, fontSize: 3);

    final _TestSearchDelegate delegate = _TestSearchDelegate(
      searchHint: searchHintText,
      searchFieldStyle: searchFieldStyle,
    );
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final Text hintText = tester.widget(find.text(searchHintText));
    final TextField textField = tester.widget<TextField>(find.byType(TextField));

    expect(hintText.style?.color, delegate.searchFieldStyle?.color);
    expect(hintText.style?.fontSize, delegate.searchFieldStyle?.fontSize);
    expect(textField.style?.color, delegate.searchFieldStyle?.color);
    expect(textField.style?.fontSize, delegate.searchFieldStyle?.fontSize);
  });

  testWidgets('Default autocorrect and enableSuggestions value', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final TextField textField = tester.widget<TextField>(find.byType(TextField));

    expect(textField.autocorrect, isTrue);
    expect(textField.enableSuggestions, isTrue);
  });

  testWidgets('Custom autocorrect value', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate(autocorrect: false);
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final TextField textField = tester.widget<TextField>(find.byType(TextField));

    expect(textField.autocorrect, isFalse);
  });

  testWidgets('Custom enableSuggestions value', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate(enableSuggestions: false);
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final TextField textField = tester.widget<TextField>(find.byType(TextField));

    expect(textField.enableSuggestions, isFalse);
  });

  testWidgets('keyboard show search button by default', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    await tester.showKeyboard(find.byType(TextField));

    expect(tester.testTextInput.setClientArgs!['inputAction'], TextInputAction.search.toString());
  });

  testWidgets('Custom textInputAction results in keyboard with corresponding button', (
    WidgetTester tester,
  ) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate(textInputAction: TextInputAction.done);
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();
    await tester.showKeyboard(find.byType(TextField));
    expect(tester.testTextInput.setClientArgs!['inputAction'], TextInputAction.done.toString());
  });

  testWidgets('Custom flexibleSpace value', (WidgetTester tester) async {
    const Widget flexibleSpace = Text('custom flexibleSpace');
    final _TestSearchDelegate delegate = _TestSearchDelegate(flexibleSpace: flexibleSpace);
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.byWidget(flexibleSpace), findsOneWidget);
  });

  group('contributes semantics with custom flexibleSpace', () {
    const Widget flexibleSpace = Text('FlexibleSpace');

    TestSemantics buildExpected({required String routeName}) {
      final bool isDesktop =
          debugDefaultTargetPlatformOverride == TargetPlatform.macOS ||
          debugDefaultTargetPlatformOverride == TargetPlatform.windows ||
          debugDefaultTargetPlatformOverride == TargetPlatform.linux;
      final bool isCupertino =
          debugDefaultTargetPlatformOverride == TargetPlatform.iOS ||
          debugDefaultTargetPlatformOverride == TargetPlatform.macOS;
      return TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics(
            id: 1,
            textDirection: TextDirection.ltr,
            children: <TestSemantics>[
              TestSemantics(
                id: 2,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 3,
                    flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                    label: routeName,
                    textDirection: TextDirection.ltr,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 4,
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 6,
                            children: <TestSemantics>[
                              TestSemantics(
                                id: 8,
                                flags: <SemanticsFlag>[
                                  SemanticsFlag.hasEnabledState,
                                  SemanticsFlag.isButton,
                                  SemanticsFlag.isEnabled,
                                  SemanticsFlag.isFocusable,
                                ],
                                actions: <SemanticsAction>[
                                  SemanticsAction.tap,
                                  if (defaultTargetPlatform != TargetPlatform.iOS)
                                    SemanticsAction.focus,
                                ],
                                tooltip: 'Back',
                                textDirection: TextDirection.ltr,
                              ),
                              TestSemantics(
                                id: 9,
                                flags: <SemanticsFlag>[
                                  SemanticsFlag.isTextField,
                                  SemanticsFlag.hasEnabledState,
                                  SemanticsFlag.isEnabled,
                                  SemanticsFlag.isFocused,
                                  SemanticsFlag.isHeader,
                                  if (!isCupertino) SemanticsFlag.namesRoute,
                                ],
                                actions: <SemanticsAction>[
                                  if (isDesktop) SemanticsAction.didGainAccessibilityFocus,
                                  if (isDesktop) SemanticsAction.didLoseAccessibilityFocus,
                                  SemanticsAction.tap,
                                  SemanticsAction.focus,
                                  SemanticsAction.setSelection,
                                  SemanticsAction.setText,
                                  SemanticsAction.paste,
                                ],
                                label: 'Search',
                                textDirection: TextDirection.ltr,
                                textSelection: const TextSelection(baseOffset: 0, extentOffset: 0),
                              ),
                              TestSemantics(
                                id: 10,
                                label: 'Bottom',
                                textDirection: TextDirection.ltr,
                              ),
                            ],
                          ),
                          TestSemantics(
                            id: 7,
                            children: <TestSemantics>[
                              TestSemantics(
                                id: 11,
                                label: 'FlexibleSpace',
                                textDirection: TextDirection.ltr,
                              ),
                            ],
                          ),
                        ],
                      ),
                      TestSemantics(
                        id: 5,
                        flags: <SemanticsFlag>[
                          SemanticsFlag.hasEnabledState,
                          SemanticsFlag.isButton,
                          SemanticsFlag.isEnabled,
                          SemanticsFlag.isFocusable,
                        ],
                        actions: <SemanticsAction>[
                          SemanticsAction.tap,
                          if (defaultTargetPlatform != TargetPlatform.iOS) SemanticsAction.focus,
                        ],
                        label: 'Suggestions',
                        textDirection: TextDirection.ltr,
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
        ],
      );
    }

    testWidgets('includes routeName on Android', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final _TestSearchDelegate delegate = _TestSearchDelegate(flexibleSpace: flexibleSpace);
      addTearDown(() => delegate.dispose());

      await tester.pumpWidget(TestHomePage(delegate: delegate));

      await tester.tap(find.byTooltip('Search'));
      await tester.pumpAndSettle();

      expect(
        semantics,
        hasSemantics(
          buildExpected(routeName: 'Search'),
          ignoreId: true,
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      semantics.dispose();
    });

    testWidgets(
      'does not include routeName',
      (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final _TestSearchDelegate delegate = _TestSearchDelegate(flexibleSpace: flexibleSpace);
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(TestHomePage(delegate: delegate));

        await tester.tap(find.byTooltip('Search'));
        await tester.pumpAndSettle();

        expect(
          semantics,
          hasSemantics(
            buildExpected(routeName: ''),
            ignoreId: true,
            ignoreRect: true,
            ignoreTransform: true,
          ),
        );

        semantics.dispose();
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );
  });

  group('contributes semantics', () {
    TestSemantics buildExpected({required String routeName}) {
      final bool isDesktop =
          debugDefaultTargetPlatformOverride == TargetPlatform.macOS ||
          debugDefaultTargetPlatformOverride == TargetPlatform.windows ||
          debugDefaultTargetPlatformOverride == TargetPlatform.linux;
      final bool isCupertino =
          debugDefaultTargetPlatformOverride == TargetPlatform.iOS ||
          debugDefaultTargetPlatformOverride == TargetPlatform.macOS;
      return TestSemantics.root(
        children: <TestSemantics>[
          TestSemantics(
            id: 1,
            textDirection: TextDirection.ltr,
            children: <TestSemantics>[
              TestSemantics(
                id: 2,
                children: <TestSemantics>[
                  TestSemantics(
                    id: 7,
                    flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                    label: routeName,
                    textDirection: TextDirection.ltr,
                    children: <TestSemantics>[
                      TestSemantics(
                        id: 9,
                        children: <TestSemantics>[
                          TestSemantics(
                            id: 10,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                            ],
                            actions: <SemanticsAction>[
                              SemanticsAction.tap,
                              if (defaultTargetPlatform != TargetPlatform.iOS)
                                SemanticsAction.focus,
                            ],
                            tooltip: 'Back',
                            textDirection: TextDirection.ltr,
                          ),
                          TestSemantics(
                            id: 11,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.isTextField,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocused,
                              SemanticsFlag.isHeader,
                              if (!isCupertino) SemanticsFlag.namesRoute,
                            ],
                            actions: <SemanticsAction>[
                              if (isDesktop) SemanticsAction.didGainAccessibilityFocus,
                              if (isDesktop) SemanticsAction.didLoseAccessibilityFocus,
                              SemanticsAction.tap,
                              SemanticsAction.focus,
                              SemanticsAction.setSelection,
                              SemanticsAction.setText,
                              SemanticsAction.paste,
                            ],
                            label: 'Search',
                            textDirection: TextDirection.ltr,
                            textSelection: const TextSelection(baseOffset: 0, extentOffset: 0),
                          ),
                          TestSemantics(id: 14, label: 'Bottom', textDirection: TextDirection.ltr),
                        ],
                      ),
                      TestSemantics(
                        id: 8,
                        flags: <SemanticsFlag>[
                          SemanticsFlag.hasEnabledState,
                          SemanticsFlag.isButton,
                          SemanticsFlag.isEnabled,
                          SemanticsFlag.isFocusable,
                        ],
                        actions: <SemanticsAction>[
                          SemanticsAction.tap,
                          if (defaultTargetPlatform != TargetPlatform.iOS) SemanticsAction.focus,
                        ],
                        label: 'Suggestions',
                        textDirection: TextDirection.ltr,
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
        ],
      );
    }

    testWidgets('includes routeName on Android', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final _TestSearchDelegate delegate = _TestSearchDelegate();
      addTearDown(() => delegate.dispose());

      await tester.pumpWidget(TestHomePage(delegate: delegate));

      await tester.tap(find.byTooltip('Search'));
      await tester.pumpAndSettle();

      expect(
        semantics,
        hasSemantics(
          buildExpected(routeName: 'Search'),
          ignoreId: true,
          ignoreRect: true,
          ignoreTransform: true,
        ),
      );

      semantics.dispose();
    });

    testWidgets(
      'does not include routeName',
      (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final _TestSearchDelegate delegate = _TestSearchDelegate();
        addTearDown(() => delegate.dispose());

        await tester.pumpWidget(TestHomePage(delegate: delegate));

        await tester.tap(find.byTooltip('Search'));
        await tester.pumpAndSettle();

        expect(
          semantics,
          hasSemantics(
            buildExpected(routeName: ''),
            ignoreId: true,
            ignoreRect: true,
            ignoreTransform: true,
          ),
        );

        semantics.dispose();
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );
  });

  testWidgets('Custom searchFieldDecorationTheme value', (WidgetTester tester) async {
    const InputDecorationTheme searchFieldDecorationTheme = InputDecorationTheme(
      hintStyle: TextStyle(color: _TestSearchDelegate.hintTextColor),
    );
    final _TestSearchDelegate delegate = _TestSearchDelegate(
      searchFieldDecorationTheme: searchFieldDecorationTheme,
    );
    addTearDown(() => delegate.dispose());

    await tester.pumpWidget(TestHomePage(delegate: delegate));
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final ThemeData textFieldTheme = Theme.of(tester.element(find.byType(TextField)));
    expect(textFieldTheme.inputDecorationTheme, searchFieldDecorationTheme);
  });

  // Regression test for: https://github.com/flutter/flutter/issues/66781
  testWidgets('text in search bar contrasts background (light mode)', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData(useMaterial3: false);
    final _TestSearchDelegate delegate = _TestSearchDelegate(defaultAppBarTheme: true);
    addTearDown(() => delegate.dispose());
    const String query = 'search query';

    await tester.pumpWidget(
      TestHomePage(
        delegate: delegate,
        passInInitialQuery: true,
        initialQuery: query,
        themeData: themeData,
      ),
    );

    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final Material appBarBackground =
        tester
            .widgetList<Material>(
              find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
            )
            .first;
    expect(appBarBackground.color, Colors.white);

    final TextField textField = tester.widget<TextField>(find.byType(TextField));
    expect(textField.style!.color, themeData.textTheme.bodyLarge!.color);
    expect(textField.style!.color, isNot(equals(Colors.white)));
  });

  // Regression test for: https://github.com/flutter/flutter/issues/66781
  testWidgets('text in search bar contrasts background (dark mode)', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData.dark(useMaterial3: false);
    final _TestSearchDelegate delegate = _TestSearchDelegate(defaultAppBarTheme: true);
    addTearDown(() => delegate.dispose());
    const String query = 'search query';

    await tester.pumpWidget(
      TestHomePage(
        delegate: delegate,
        passInInitialQuery: true,
        initialQuery: query,
        themeData: themeData,
      ),
    );

    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final Material appBarBackground =
        tester
            .widgetList<Material>(
              find.descendant(of: find.byType(AppBar), matching: find.byType(Material)),
            )
            .first;
    expect(appBarBackground.color, themeData.primaryColor);

    final TextField textField = tester.widget<TextField>(find.byType(TextField));
    expect(textField.style!.color, themeData.textTheme.bodyLarge!.color);
    expect(textField.style!.color, isNot(equals(themeData.primaryColor)));
  });

  // Regression test for: https://github.com/flutter/flutter/issues/78144
  testWidgets('`Leading`, `Actions` and `FlexibleSpace` nullable test', (
    WidgetTester tester,
  ) async {
    // The search delegate page is displayed with no issues
    // even with a null return values for [buildLeading], [buildActions] and [flexibleSpace].
    final _TestEmptySearchDelegate delegate = _TestEmptySearchDelegate();
    addTearDown(delegate.dispose);
    final List<String> selectedResults = <String>[];

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));

    // We are on the homepage.
    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('HomeTitle'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);

    // Open the search page.
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsNothing);
    expect(find.text('HomeTitle'), findsNothing);
    expect(find.text('Suggestions'), findsOneWidget);
    expect(selectedResults, hasLength(0));

    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.focusNode!.hasFocus, isTrue);

    // Close the search page.
    await tester.tap(find.byTooltip('Close'));
    await tester.pumpAndSettle();

    expect(find.text('HomeBody'), findsOneWidget);
    expect(find.text('HomeTitle'), findsOneWidget);
    expect(find.text('Suggestions'), findsNothing);
    expect(selectedResults, <String>['Result']);
  });

  testWidgets('Leading width size is 16', (WidgetTester tester) async {
    final _TestSearchDelegate delegate = _TestSearchDelegate();
    final List<String> selectedResults = <String>[];
    delegate.leadingWidth = 16;

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));

    // Open the search page with check leading width smaller than 16.
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();
    await tester.tapAt(const Offset(16, 16));
    expect(find.text('Suggestions'), findsOneWidget);
    final Finder appBarFinder = find.byType(AppBar);
    final AppBar appBar = tester.widget<AppBar>(appBarFinder);
    expect(appBar.leadingWidth, 16);
    await tester.tapAt(const Offset(8, 16));
    await tester.pumpAndSettle();
    expect(find.text('Suggestions'), findsNothing);
    expect(find.text('HomeBody'), findsOneWidget);
  });

  testWidgets('showSearch with useRootNavigator', (WidgetTester tester) async {
    final _MyNavigatorObserver rootObserver = _MyNavigatorObserver();
    final _MyNavigatorObserver localObserver = _MyNavigatorObserver();

    final _TestEmptySearchDelegate delegate = _TestEmptySearchDelegate();
    addTearDown(delegate.dispose);

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[localObserver],
          onGenerateRoute: (RouteSettings settings) {
            if (settings.name == 'nested') {
              return MaterialPageRoute<dynamic>(
                builder:
                    (BuildContext context) => Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: <Widget>[
                        TextButton(
                          onPressed: () async {
                            await showSearch(
                              context: context,
                              delegate: delegate,
                              useRootNavigator: true,
                            );
                          },
                          child: const Text('showSearchRootNavigator'),
                        ),
                        TextButton(
                          onPressed: () async {
                            await showSearch(context: context, delegate: delegate);
                          },
                          child: const Text('showSearchLocalNavigator'),
                        ),
                      ],
                    ),
                settings: settings,
              );
            }
            throw UnimplementedError();
          },
          initialRoute: 'nested',
        ),
      ),
    );

    expect(rootObserver.pushCount, 0);
    expect(localObserver.pushCount, 0);

    // showSearch normal and back.
    await tester.tap(find.text('showSearchLocalNavigator'));
    await tester.pumpAndSettle();
    final Finder backButtonFinder = find.byType(BackButton);
    expect(backButtonFinder, findsWidgets);
    await tester.tap(find.byTooltip('Close'));
    await tester.pumpAndSettle();
    expect(rootObserver.pushCount, 0);
    expect(localObserver.pushCount, 1);

    // showSearch with rootNavigator.
    await tester.tap(find.text('showSearchRootNavigator'));
    await tester.pumpAndSettle();
    await tester.tap(find.byTooltip('Close'));
    await tester.pumpAndSettle();

    // showSearch without back button.
    delegate.automaticallyImplyLeading = false;
    await tester.tap(find.text('showSearchRootNavigator'));
    await tester.pumpAndSettle();
    final Finder appBarFinder = find.byType(AppBar);
    final AppBar appBar = tester.widget<AppBar>(appBarFinder);
    expect(appBar.automaticallyImplyLeading, false);
    expect(find.byTooltip('Back'), findsNothing);
    await tester.tap(find.byTooltip('Close'));
    await tester.pumpAndSettle();
    expect(rootObserver.pushCount, 2);
    expect(localObserver.pushCount, 1);
  });

  testWidgets('Query text field shows toolbar initially', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/95588

    final _TestSearchDelegate delegate = _TestSearchDelegate();
    addTearDown(() => delegate.dispose());
    final List<String> selectedResults = <String>[];

    await tester.pumpWidget(TestHomePage(delegate: delegate, results: selectedResults));

    // Open search.
    await tester.tap(find.byTooltip('Search'));
    await tester.pumpAndSettle();

    final Finder textFieldFinder = find.byType(TextField);
    final TextField textField = tester.widget<TextField>(textFieldFinder);
    expect(textField.controller!.text.length, 0);

    mockClipboard.handleMethodCall(
      const MethodCall('Clipboard.setData', <String, dynamic>{'text': 'pasteablestring'}),
    );

    // Long press shows toolbar.
    await tester.longPress(textFieldFinder);
    await tester.pump();
    expect(find.text('Paste'), findsOneWidget);

    await tester.tap(find.text('Paste'));
    await tester.pump();
    expect(textField.controller!.text.length, 15);
  }, skip: kIsWeb); // [intended] We do not use Flutter-rendered context menu on the Web.

  testWidgets('showSearch with maintainState on the route', (WidgetTester tester) async {
    final _MyNavigatorObserver navigationObserver = _MyNavigatorObserver();

    final _TestEmptySearchDelegate delegate = _TestEmptySearchDelegate();
    addTearDown(delegate.dispose);

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[navigationObserver],
        home: Builder(
          builder:
              (BuildContext context) => Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  TextButton(
                    onPressed: () async {
                      await showSearch(context: context, delegate: delegate);
                    },
                    child: const Text('showSearch'),
                  ),
                  TextButton(
                    onPressed: () async {
                      await showSearch(context: context, delegate: delegate, maintainState: true);
                    },
                    child: const Text('showSearchWithMaintainState'),
                  ),
                ],
              ),
        ),
      ),
    );

    expect(navigationObserver.pushCount, 0);
    expect(navigationObserver.maintainState, false);

    // showSearch normal and back.
    await tester.tap(find.text('showSearch'));
    await tester.pumpAndSettle();
    final Finder backButtonFinder = find.byType(BackButton);
    expect(backButtonFinder, findsWidgets);
    await tester.tap(find.byTooltip('Close'));
    await tester.pumpAndSettle();
    expect(navigationObserver.pushCount, 1);
    expect(navigationObserver.maintainState, false);

    // showSearch with maintainState.
    await tester.tap(find.text('showSearchWithMaintainState'));
    await tester.pumpAndSettle();
    expect(backButtonFinder, findsWidgets);
    await tester.tap(find.byTooltip('Close'));
    await tester.pumpAndSettle();
    expect(navigationObserver.pushCount, 2);
    expect(navigationObserver.maintainState, true);
  });
}

class TestHomePage extends StatelessWidget {
  const TestHomePage({
    super.key,
    this.results,
    required this.delegate,
    this.passInInitialQuery = false,
    this.initialQuery,
    this.themeData,
  });

  final List<String?>? results;
  final SearchDelegate<String> delegate;
  final bool passInInitialQuery;
  final ThemeData? themeData;
  final String? initialQuery;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: themeData,
      home: Builder(
        builder: (BuildContext context) {
          return Scaffold(
            appBar: AppBar(
              title: const Text('HomeTitle'),
              actions: <Widget>[
                IconButton(
                  tooltip: 'Search',
                  icon: const Icon(Icons.search),
                  onPressed: () async {
                    String? selectedResult;
                    if (passInInitialQuery) {
                      selectedResult = await showSearch<String>(
                        context: context,
                        delegate: delegate,
                        query: initialQuery,
                      );
                    } else {
                      selectedResult = await showSearch<String>(
                        context: context,
                        delegate: delegate,
                      );
                    }
                    results?.add(selectedResult);
                  },
                ),
              ],
            ),
            body: const Text('HomeBody'),
          );
        },
      ),
    );
  }
}

class _TestSearchDelegate extends SearchDelegate<String> {
  _TestSearchDelegate({
    this.suggestions = 'Suggestions',
    this.result = 'Result',
    this.actions = const <Widget>[],
    this.flexibleSpace,
    this.defaultAppBarTheme = false,
    super.searchFieldDecorationTheme,
    super.searchFieldStyle,
    String? searchHint,
    super.textInputAction,
    super.autocorrect,
    super.enableSuggestions,
  }) : super(searchFieldLabel: searchHint);

  final bool defaultAppBarTheme;
  final String suggestions;
  final String result;
  final List<Widget> actions;
  final Widget? flexibleSpace;
  static const Color hintTextColor = Colors.green;

  @override
  ThemeData appBarTheme(BuildContext context) {
    if (defaultAppBarTheme) {
      return super.appBarTheme(context);
    }
    final ThemeData theme = Theme.of(context);
    return theme.copyWith(
      inputDecorationTheme:
          searchFieldDecorationTheme ??
          InputDecorationTheme(
            hintStyle: searchFieldStyle ?? const TextStyle(color: hintTextColor),
          ),
    );
  }

  @override
  Widget buildLeading(BuildContext context) {
    return IconButton(
      tooltip: 'Back',
      icon: const Icon(Icons.arrow_back),
      onPressed: () {
        close(context, result);
      },
    );
  }

  final List<String> queriesForSuggestions = <String>[];
  final List<String> queriesForResults = <String>[];

  @override
  Widget buildSuggestions(BuildContext context) {
    queriesForSuggestions.add(query);
    return ElevatedButton(
      onPressed: () {
        showResults(context);
      },
      child: Text(suggestions),
    );
  }

  @override
  Widget buildResults(BuildContext context) {
    queriesForResults.add(query);
    return const Text('Results');
  }

  @override
  List<Widget> buildActions(BuildContext context) {
    return actions;
  }

  @override
  Widget? buildFlexibleSpace(BuildContext context) {
    return flexibleSpace;
  }

  @override
  PreferredSizeWidget buildBottom(BuildContext context) {
    return const PreferredSize(preferredSize: Size.fromHeight(56.0), child: Text('Bottom'));
  }
}

class _TestEmptySearchDelegate extends SearchDelegate<String> {
  @override
  Widget? buildLeading(BuildContext context) => null;

  @override
  List<Widget>? buildActions(BuildContext context) => null;

  @override
  Widget buildSuggestions(BuildContext context) {
    return ElevatedButton(
      onPressed: () {
        showResults(context);
      },
      child: const Text('Suggestions'),
    );
  }

  @override
  Widget buildResults(BuildContext context) {
    return const Text('Results');
  }

  @override
  PreferredSizeWidget buildBottom(BuildContext context) {
    return PreferredSize(
      preferredSize: const Size.fromHeight(56.0),
      child: IconButton(
        tooltip: 'Close',
        icon: const Icon(Icons.arrow_back),
        onPressed: () {
          close(context, 'Result');
        },
      ),
    );
  }
}

class _MyNavigatorObserver extends NavigatorObserver {
  bool maintainState = false;
  int pushCount = 0;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    // don't count the root route
    if (<String>['nested', '/'].contains(route.settings.name)) {
      return;
    }
    if (route is PageRoute) {
      maintainState = route.maintainState;
    }
    pushCount++;
  }
}
