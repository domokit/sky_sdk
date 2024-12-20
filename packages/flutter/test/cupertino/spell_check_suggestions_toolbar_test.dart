// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets(
    'more than three suggestions throws an error',
    (WidgetTester tester) async {
      Future<void> pumpToolbar(List<String> suggestions) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: Center(
              child: CupertinoSpellCheckSuggestionsToolbar(
                anchors: const TextSelectionToolbarAnchors(primaryAnchor: Offset.zero),
                buttonItems:
                    suggestions.map((String string) {
                      return ContextMenuButtonItem(onPressed: () {}, label: string);
                    }).toList(),
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
    final List<ContextMenuButtonItem>? buttonItems =
        CupertinoSpellCheckSuggestionsToolbar.buildButtonItems(editableTextState);
    expect(buttonItems, isNotNull);
    final Iterable<String?> labels = buttonItems!.map((ContextMenuButtonItem buttonItem) {
      return buttonItem.label;
    });
    expect(labels, hasLength(3));
    expect(labels, contains('hello'));
    expect(labels, contains('yellow'));
    expect(labels, contains('yell'));
    expect(labels, isNot(contains('yeller')));
  });

  testWidgets(
    'buildButtonItems builds a disabled "No Replacements Found" button when no suggestions',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController();
      addTearDown(controller.dispose);
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(CupertinoApp(home: _FakeEditableText(focusNode, controller)));
      final _FakeEditableTextState editableTextState = tester.state(find.byType(_FakeEditableText));
      final List<ContextMenuButtonItem>? buttonItems =
          CupertinoSpellCheckSuggestionsToolbar.buildButtonItems(editableTextState);

      expect(buttonItems, isNotNull);
      expect(buttonItems, hasLength(1));
      expect(buttonItems!.first.label, 'No Replacements Found');
      expect(buttonItems.first.onPressed, isNull);
    },
  );
}

class _FakeEditableText extends EditableText {
  /// The parameters focusNode and controller are needed here so the can be
  /// safely disposed after the test is completed.
  _FakeEditableText(FocusNode focusNode, TextEditingController controller)
    : super(
        controller: controller,
        focusNode: focusNode,
        backgroundCursorColor: CupertinoColors.white,
        cursorColor: CupertinoColors.white,
        style: const TextStyle(),
      );

  @override
  _FakeEditableTextState createState() => _FakeEditableTextState();
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
