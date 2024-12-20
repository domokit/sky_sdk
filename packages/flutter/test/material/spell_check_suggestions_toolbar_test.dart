// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

// Vertical position at which to anchor the toolbar for testing.
const double _kAnchor = 200;
// Amount for toolbar to overlap bottom padding for testing.
const double _kTestToolbarOverlap = 10;

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  /// Builds test button items for each of the suggestions provided.
  List<ContextMenuButtonItem> buildSuggestionButtons(List<String> suggestions) {
    return <ContextMenuButtonItem>[
      for (final String suggestion in suggestions)
        ContextMenuButtonItem(onPressed: () {}, label: suggestion),
      ContextMenuButtonItem(onPressed: () {}, type: ContextMenuButtonType.delete, label: 'DELETE'),
    ];
  }

  /// Finds the container of the [SpellCheckSuggestionsToolbar] so that
  /// the position of the toolbar itself may be determined.
  Finder findSpellCheckSuggestionsToolbar() {
    return find.descendant(
      of: find.byType(MaterialApp),
      matching: find.byWidgetPredicate(
        (Widget w) => '${w.runtimeType}' == '_SpellCheckSuggestionsToolbarContainer',
      ),
    );
  }

  testWidgets('positions toolbar below anchor when it fits above bottom view padding', (
    WidgetTester tester,
  ) async {
    // We expect the toolbar to be positioned right below the anchor with padding accounted for.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SpellCheckSuggestionsToolbar(
            anchor: const Offset(0.0, _kAnchor),
            buttonItems: buildSuggestionButtons(<String>['hello', 'yellow', 'yell']),
          ),
        ),
      ),
    );

    final double toolbarY = tester.getTopLeft(findSpellCheckSuggestionsToolbar()).dy;
    expect(toolbarY, equals(_kAnchor));
  });

  testWidgets(
    're-positions toolbar higher below anchor when it does not fit above bottom view padding',
    (WidgetTester tester) async {
      // We expect the toolbar to be positioned _kTestToolbarOverlap pixels above the anchor.
      const double expectedToolbarY = _kAnchor - _kTestToolbarOverlap;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SpellCheckSuggestionsToolbar(
              anchor: const Offset(0.0, _kAnchor - _kTestToolbarOverlap),
              buttonItems: buildSuggestionButtons(<String>['hello', 'yellow', 'yell']),
            ),
          ),
        ),
      );

      final double toolbarY = tester.getTopLeft(findSpellCheckSuggestionsToolbar()).dy;
      expect(toolbarY, equals(expectedToolbarY));
    },
  );

  testWidgets(
    'more than three suggestions throws an error',
    (WidgetTester tester) async {
      Future<void> pumpToolbar(List<String> suggestions) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: SpellCheckSuggestionsToolbar(
                anchor: const Offset(0.0, _kAnchor - _kTestToolbarOverlap),
                buttonItems: buildSuggestionButtons(suggestions),
              ),
            ),
          ),
        );
      }

      await pumpToolbar(<String>['hello', 'yellow', 'yell']);
      expect(() async {
        await pumpToolbar(<String>['hello', 'yellow', 'yell', 'yeller']);
      }, throwsAssertionError);
    },
    skip: kIsWeb, // [intended]
  );

  test('buildSuggestionButtons only considers the first three suggestions', () {
    final _FakeEditableTextState editableTextState = _FakeEditableTextState(
      suggestions: <String>['hello', 'yellow', 'yell', 'yeller'],
    );
    final List<ContextMenuButtonItem>? buttonItems = SpellCheckSuggestionsToolbar.buildButtonItems(
      editableTextState,
    );
    expect(buttonItems, isNotNull);
    final Iterable<String?> labels = buttonItems!.map((ContextMenuButtonItem buttonItem) {
      return buttonItem.label;
    });
    expect(labels, hasLength(4));
    expect(labels, contains('hello'));
    expect(labels, contains('yellow'));
    expect(labels, contains('yell'));
    expect(labels, contains(null)); // For the delete button.
    expect(labels, isNot(contains('yeller')));
  });

  test('buildButtonItems builds only a delete button when no suggestions', () {
    final _FakeEditableTextState editableTextState = _FakeEditableTextState();
    final List<ContextMenuButtonItem>? buttonItems = SpellCheckSuggestionsToolbar.buildButtonItems(
      editableTextState,
    );

    expect(buttonItems, hasLength(1));
    expect(buttonItems!.first.type, ContextMenuButtonType.delete);
  });
}

class _FakeEditableTextState extends EditableTextState {
  _FakeEditableTextState({this.suggestions});

  final List<String>? suggestions;

  @override
  TextEditingValue get currentTextEditingValue => TextEditingValue.empty;

  @override
  SuggestionSpan? findSuggestionSpanAtCursorIndex(int cursorIndex) {
    return SuggestionSpan(const TextRange(start: 0, end: 0), suggestions ?? <String>[]);
  }
}
