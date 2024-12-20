// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

const List<Widget> fooBarTexts = <Text>[
  Text('foo', textDirection: TextDirection.ltr),
  Text('bar', textDirection: TextDirection.ltr),
];

void main() {
  group('image', () {
    testWidgets('finds Image widgets', (WidgetTester tester) async {
      await tester.pumpWidget(_boilerplate(Image(image: FileImage(File('test')))));
      expect(find.image(FileImage(File('test'))), findsOneWidget);
    });

    testWidgets('finds Button widgets with Image', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(ElevatedButton(onPressed: null, child: Image(image: FileImage(File('test'))))),
      );
      expect(find.widgetWithImage(ElevatedButton, FileImage(File('test'))), findsOneWidget);
    });
  });

  group('text', () {
    testWidgets('finds Text widgets', (WidgetTester tester) async {
      await tester.pumpWidget(_boilerplate(const Text('test')));
      expect(find.text('test'), findsOneWidget);
    });

    testWidgets('finds Text.rich widgets', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          const Text.rich(
            TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'e'), TextSpan(text: 'st')]),
          ),
        ),
      );

      expect(find.text('test'), findsOneWidget);
    });

    group('findRichText', () {
      testWidgets('finds RichText widgets when enabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            RichText(text: const TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est')])),
          ),
        );

        expect(find.text('test', findRichText: true), findsOneWidget);
      });

      testWidgets('finds Text widgets once when enabled', (WidgetTester tester) async {
        await tester.pumpWidget(_boilerplate(const Text('test2')));

        expect(find.text('test2', findRichText: true), findsOneWidget);
      });

      testWidgets('does not find RichText widgets when disabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            RichText(text: const TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est')])),
          ),
        );

        expect(find.text('test'), findsNothing);
      });

      testWidgets('does not find Text and RichText separated by semantics widgets twice', (
        WidgetTester tester,
      ) async {
        // If rich: true found both Text and RichText, this would find two widgets.
        await tester.pumpWidget(_boilerplate(const Text('test', semanticsLabel: 'foo')));

        expect(find.text('test'), findsOneWidget);
      });

      testWidgets('finds Text.rich widgets when enabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            const Text.rich(
              TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est'), TextSpan(text: '3')]),
            ),
          ),
        );

        expect(find.text('test3', findRichText: true), findsOneWidget);
      });

      testWidgets('finds Text.rich widgets when disabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            const Text.rich(
              TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est'), TextSpan(text: '3')]),
            ),
          ),
        );

        expect(find.text('test3'), findsOneWidget);
      });
    });
  });

  group('textContaining', () {
    testWidgets('finds Text widgets', (WidgetTester tester) async {
      await tester.pumpWidget(_boilerplate(const Text('this is a test')));
      expect(find.textContaining(RegExp(r'test')), findsOneWidget);
      expect(find.textContaining('test'), findsOneWidget);
      expect(find.textContaining('a'), findsOneWidget);
      expect(find.textContaining('s'), findsOneWidget);
    });

    testWidgets('finds Text.rich widgets', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          const Text.rich(
            TextSpan(
              text: 'this',
              children: <TextSpan>[
                TextSpan(text: 'is'),
                TextSpan(text: 'a'),
                TextSpan(text: 'test'),
              ],
            ),
          ),
        ),
      );

      expect(find.textContaining(RegExp(r'isatest')), findsOneWidget);
      expect(find.textContaining('isatest'), findsOneWidget);
    });

    testWidgets('finds EditableText widgets', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: _boilerplate(
              TextField(controller: TextEditingController()..text = 'this is test'),
            ),
          ),
        ),
      );

      expect(find.textContaining(RegExp(r'test')), findsOneWidget);
      expect(find.textContaining('test'), findsOneWidget);
    });

    group('findRichText', () {
      testWidgets('finds RichText widgets when enabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            RichText(text: const TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est')])),
          ),
        );

        expect(find.textContaining('te', findRichText: true), findsOneWidget);
      });

      testWidgets('finds Text widgets once when enabled', (WidgetTester tester) async {
        await tester.pumpWidget(_boilerplate(const Text('test2')));

        expect(find.textContaining('tes', findRichText: true), findsOneWidget);
      });

      testWidgets('does not find RichText widgets when disabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            RichText(text: const TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est')])),
          ),
        );

        expect(find.textContaining('te'), findsNothing);
      });

      testWidgets('does not find Text and RichText separated by semantics widgets twice', (
        WidgetTester tester,
      ) async {
        // If rich: true found both Text and RichText, this would find two widgets.
        await tester.pumpWidget(_boilerplate(const Text('test', semanticsLabel: 'foo')));

        expect(find.textContaining('tes'), findsOneWidget);
      });

      testWidgets('finds Text.rich widgets when enabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            const Text.rich(
              TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est'), TextSpan(text: '3')]),
            ),
          ),
        );

        expect(find.textContaining('t3', findRichText: true), findsOneWidget);
      });

      testWidgets('finds Text.rich widgets when disabled', (WidgetTester tester) async {
        await tester.pumpWidget(
          _boilerplate(
            const Text.rich(
              TextSpan(text: 't', children: <TextSpan>[TextSpan(text: 'est'), TextSpan(text: '3')]),
            ),
          ),
        );

        expect(find.textContaining('t3'), findsOneWidget);
      });
    });
  });

  group('semantics', () {
    testWidgets('Throws StateError if semantics are not enabled', (WidgetTester tester) async {
      expect(() => find.bySemanticsLabel('Add'), throwsStateError);
    }, semanticsEnabled: false);

    testWidgets('finds Semantically labeled widgets', (WidgetTester tester) async {
      final SemanticsHandle semanticsHandle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            label: 'Add',
            button: true,
            child: const TextButton(onPressed: null, child: Text('+')),
          ),
        ),
      );
      expect(find.bySemanticsLabel('Add'), findsOneWidget);
      semanticsHandle.dispose();
    });

    testWidgets('finds Semantically labeled widgets by RegExp', (WidgetTester tester) async {
      final SemanticsHandle semanticsHandle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            container: true,
            child: const Row(children: <Widget>[Text('Hello'), Text('World')]),
          ),
        ),
      );
      expect(find.bySemanticsLabel('Hello'), findsNothing);
      expect(find.bySemanticsLabel(RegExp(r'^Hello')), findsOneWidget);
      semanticsHandle.dispose();
    });

    testWidgets('finds Semantically labeled widgets without explicit Semantics', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle semanticsHandle = tester.ensureSemantics();
      await tester.pumpWidget(_boilerplate(const SimpleCustomSemanticsWidget('Foo')));
      expect(find.bySemanticsLabel('Foo'), findsOneWidget);
      semanticsHandle.dispose();
    });

    testWidgets(
      'Throws StateError if semantics are not enabled (bySemanticsIdentifier)',
      (WidgetTester tester) async {
        expect(
          () => find.bySemanticsIdentifier('Add'),
          throwsA(
            isA<StateError>().having(
              (StateError e) => e.message,
              'message',
              contains('Semantics are not enabled'),
            ),
          ),
        );
      },
      semanticsEnabled: false,
    );

    testWidgets('finds Semantically labeled widgets by identifier', (WidgetTester tester) async {
      final SemanticsHandle semanticsHandle = tester.ensureSemantics();
      await tester.pumpWidget(
        _boilerplate(
          Semantics(
            identifier: 'Add',
            button: true,
            child: const TextButton(onPressed: null, child: Text('+')),
          ),
        ),
      );
      expect(find.bySemanticsIdentifier('Add'), findsOneWidget);
      semanticsHandle.dispose();
    });

    testWidgets('finds Semantically labeled widgets by identifier RegExp', (
      WidgetTester tester,
    ) async {
      final SemanticsHandle semanticsHandle = tester.ensureSemantics();
      // list of elements with a prefixed identifier
      await tester.pumpWidget(
        _boilerplate(
          Row(
            children: <Widget>[
              Semantics(identifier: 'item-1', child: const Text('Item 1')),
              Semantics(identifier: 'item-2', child: const Text('Item 2')),
            ],
          ),
        ),
      );
      expect(find.bySemanticsIdentifier('item'), findsNothing);
      expect(find.bySemanticsIdentifier(RegExp(r'^item-')), findsNWidgets(2));
      semanticsHandle.dispose();
    });
  });

  group('byTooltip', () {
    testWidgets('finds widgets by tooltip', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(const Tooltip(message: 'Tooltip Message', child: Text('+'))),
      );
      expect(find.byTooltip('Tooltip Message'), findsOneWidget);
    });

    testWidgets('finds widgets with tooltip by RegExp', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(const Tooltip(message: 'Tooltip Message', child: Text('+'))),
      );
      expect(find.byTooltip('Tooltip'), findsNothing);
      expect(find.byTooltip(RegExp(r'^Tooltip')), findsOneWidget);
    });

    testWidgets('finds widgets by rich text tooltip', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          const Tooltip(
            richMessage: TextSpan(
              children: <InlineSpan>[TextSpan(text: 'Tooltip '), TextSpan(text: 'Message')],
            ),
            child: Text('+'),
          ),
        ),
      );
      expect(find.byTooltip('Tooltip Message'), findsOneWidget);
    });

    testWidgets('finds widgets with rich text tooltip by RegExp', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          const Tooltip(
            richMessage: TextSpan(
              children: <InlineSpan>[TextSpan(text: 'Tooltip '), TextSpan(text: 'Message')],
            ),
            child: Text('+'),
          ),
        ),
      );
      expect(find.byTooltip('Tooltip M'), findsNothing);
      expect(find.byTooltip(RegExp(r'^Tooltip M')), findsOneWidget);
    });

    testWidgets('finds empty string with tooltip', (WidgetTester tester) async {
      await tester.pumpWidget(_boilerplate(const Tooltip(message: '', child: Text('+'))));
      expect(find.byTooltip(''), findsOneWidget);

      await tester.pumpWidget(
        _boilerplate(
          const Tooltip(
            richMessage: TextSpan(children: <InlineSpan>[TextSpan(text: '')]),
            child: Text('+'),
          ),
        ),
      );
      expect(find.byTooltip(''), findsOneWidget);

      await tester.pumpWidget(_boilerplate(const Tooltip(message: '', child: Text('+'))));
      expect(find.byTooltip(RegExp(r'^$')), findsOneWidget);

      await tester.pumpWidget(
        _boilerplate(
          const Tooltip(
            richMessage: TextSpan(children: <InlineSpan>[TextSpan(text: '')]),
            child: Text('+'),
          ),
        ),
      );
      expect(find.byTooltip(RegExp(r'^$')), findsOneWidget);
    });
  });

  group('hitTestable', () {
    testWidgets('excludes non-hit-testable widgets', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          IndexedStack(
            sizing: StackFit.expand,
            children: <Widget>[
              GestureDetector(
                key: const ValueKey<int>(0),
                behavior: HitTestBehavior.opaque,
                onTap: () {},
                child: const SizedBox.expand(),
              ),
              GestureDetector(
                key: const ValueKey<int>(1),
                behavior: HitTestBehavior.opaque,
                onTap: () {},
                child: const SizedBox.expand(),
              ),
            ],
          ),
        ),
      );
      expect(find.byType(GestureDetector), findsOneWidget);
      expect(find.byType(GestureDetector, skipOffstage: false), findsNWidgets(2));
      final Finder hitTestable = find.byType(GestureDetector, skipOffstage: false).hitTestable();
      expect(hitTestable, findsOneWidget);
      expect(tester.widget(hitTestable).key, const ValueKey<int>(0));
    });
  });

  group('text range finders', () {
    testWidgets('basic text span test', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          const IndexedStack(
            sizing: StackFit.expand,
            children: <Widget>[
              Text.rich(
                TextSpan(
                  text: 'sub',
                  children: <InlineSpan>[
                    TextSpan(text: 'stringsub'),
                    TextSpan(text: 'stringsub'),
                    TextSpan(text: 'stringsub'),
                  ],
                ),
              ),
              Text('substringsub'),
            ],
          ),
        ),
      );

      expect(
        find.textRange.ofSubstring('substringsub'),
        findsExactly(2),
      ); // Pattern skips overlapping matches.
      expect(
        find.textRange.ofSubstring('substringsub').first.evaluate().single.textRange,
        const TextRange(start: 0, end: 12),
      );
      expect(
        find.textRange.ofSubstring('substringsub').last.evaluate().single.textRange,
        const TextRange(start: 18, end: 30),
      );

      expect(
        find.textRange.ofSubstring('substringsub').first.evaluate().single.renderObject,
        find.textRange.ofSubstring('substringsub').last.evaluate().single.renderObject,
      );

      expect(find.textRange.ofSubstring('substringsub', skipOffstage: false), findsExactly(3));
    });

    testWidgets('basic text span test', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          const IndexedStack(
            sizing: StackFit.expand,
            children: <Widget>[
              Text.rich(
                TextSpan(
                  text: 'sub',
                  children: <InlineSpan>[
                    TextSpan(text: 'stringsub'),
                    TextSpan(text: 'stringsub'),
                    TextSpan(text: 'stringsub'),
                  ],
                ),
              ),
              Text('substringsub'),
            ],
          ),
        ),
      );

      expect(
        find.textRange.ofSubstring('substringsub'),
        findsExactly(2),
      ); // Pattern skips overlapping matches.
      expect(
        find.textRange.ofSubstring('substringsub').first.evaluate().single.textRange,
        const TextRange(start: 0, end: 12),
      );
      expect(
        find.textRange.ofSubstring('substringsub').last.evaluate().single.textRange,
        const TextRange(start: 18, end: 30),
      );

      expect(
        find.textRange.ofSubstring('substringsub').first.evaluate().single.renderObject,
        find.textRange.ofSubstring('substringsub').last.evaluate().single.renderObject,
      );

      expect(find.textRange.ofSubstring('substringsub', skipOffstage: false), findsExactly(3));
    });

    testWidgets('descendentOf', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          const Column(
            children: <Widget>[
              Text.rich(TextSpan(text: 'text')),
              Text.rich(TextSpan(text: 'text')),
            ],
          ),
        ),
      );

      expect(find.textRange.ofSubstring('text'), findsExactly(2));
      expect(find.textRange.ofSubstring('text', descendentOf: find.text('text').first), findsOne);
    });

    testWidgets('finds only static text for now', (WidgetTester tester) async {
      await tester.pumpWidget(
        _boilerplate(
          EditableText(
            controller: TextEditingController(text: 'text'),
            focusNode: FocusNode(),
            style: const TextStyle(),
            cursorColor: const Color(0x00000000),
            backgroundCursorColor: const Color(0x00000000),
          ),
        ),
      );

      expect(find.textRange.ofSubstring('text'), findsNothing);
    });
  });

  testWidgets('ChainedFinders chain properly', (WidgetTester tester) async {
    final GlobalKey key1 = GlobalKey();
    await tester.pumpWidget(
      _boilerplate(
        Column(children: <Widget>[Container(key: key1, child: const Text('1')), const Text('2')]),
      ),
    );

    // Get the text back. By correctly chaining the descendant finder's
    // candidates, it should find 1 instead of 2. If the _LastFinder wasn't
    // correctly chained after the descendant's candidates, the last element
    // with a Text widget would have been 2.
    final Text text =
        find
                .descendant(of: find.byKey(key1), matching: find.byType(Text))
                .last
                .evaluate()
                .single
                .widget
            as Text;

    expect(text.data, '1');
  });

  testWidgets('finds multiple subtypes', (WidgetTester tester) async {
    await tester.pumpWidget(
      _boilerplate(
        Row(
          children: <Widget>[
            const Column(children: <Widget>[Text('Hello'), Text('World')]),
            Column(children: <Widget>[Image(image: FileImage(File('test')))]),
            const Column(
              children: <Widget>[
                SimpleGenericWidget<int>(child: Text('one')),
                SimpleGenericWidget<double>(child: Text('pi')),
                SimpleGenericWidget<String>(child: Text('two')),
              ],
            ),
          ],
        ),
      ),
    );

    expect(find.bySubtype<Row>(), findsOneWidget);
    expect(find.bySubtype<Column>(), findsNWidgets(3));
    // Finds both rows and columns.
    expect(find.bySubtype<Flex>(), findsNWidgets(4));

    // Finds only the requested generic subtypes.
    expect(find.bySubtype<SimpleGenericWidget<int>>(), findsOneWidget);
    expect(find.bySubtype<SimpleGenericWidget<num>>(), findsNWidgets(2));
    expect(find.bySubtype<SimpleGenericWidget<Object>>(), findsNWidgets(3));

    // Finds all widgets.
    final int totalWidgetCount = find.byWidgetPredicate((_) => true).evaluate().length;
    expect(find.bySubtype<Widget>(), findsNWidgets(totalWidgetCount));
  });

  group('find.byElementPredicate', () {
    testWidgets('fails with a custom description in the message', (WidgetTester tester) async {
      await tester.pumpWidget(const Text('foo', textDirection: TextDirection.ltr));

      const String customDescription = 'custom description';
      late TestFailure failure;
      try {
        expect(
          find.byElementPredicate((_) => false, description: customDescription),
          findsOneWidget,
        );
      } on TestFailure catch (e) {
        failure = e;
      }

      expect(failure, isNotNull);
      expect(
        failure.message,
        contains('Actual: _ElementPredicateWidgetFinder:<Found 0 widgets with $customDescription'),
      );
    });
  });

  group('find.byWidgetPredicate', () {
    testWidgets('fails with a custom description in the message', (WidgetTester tester) async {
      await tester.pumpWidget(const Text('foo', textDirection: TextDirection.ltr));

      const String customDescription = 'custom description';
      late TestFailure failure;
      try {
        expect(
          find.byWidgetPredicate((_) => false, description: customDescription),
          findsOneWidget,
        );
      } on TestFailure catch (e) {
        failure = e;
      }

      expect(failure, isNotNull);
      expect(
        failure.message,
        contains('Actual: _WidgetPredicateWidgetFinder:<Found 0 widgets with $customDescription'),
      );
    });
  });

  group('find.descendant', () {
    testWidgets('finds one descendant', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[Column(children: fooBarTexts)],
        ),
      );

      expect(
        find.descendant(of: find.widgetWithText(Row, 'foo'), matching: find.text('bar')),
        findsOneWidget,
      );
    });

    testWidgets('finds two descendants with different ancestors', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[Column(children: fooBarTexts), Column(children: fooBarTexts)],
        ),
      );

      expect(
        find.descendant(of: find.widgetWithText(Column, 'foo'), matching: find.text('bar')),
        findsNWidgets(2),
      );
    });

    testWidgets('fails with a descriptive message', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[
            Column(children: <Text>[Text('foo', textDirection: TextDirection.ltr)]),
            Text('bar', textDirection: TextDirection.ltr),
          ],
        ),
      );

      late TestFailure failure;
      try {
        expect(
          find.descendant(of: find.widgetWithText(Column, 'foo'), matching: find.text('bar')),
          findsOneWidget,
        );
      } on TestFailure catch (e) {
        failure = e;
      }

      expect(failure, isNotNull);
      expect(
        failure.message,
        contains(
          'Actual: _DescendantWidgetFinder:<Found 0 widgets with text "bar" descending from widgets with type "Column" that are ancestors of widgets with text "foo"',
        ),
      );
    });
  });

  group('find.ancestor', () {
    testWidgets('finds one ancestor', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[Column(children: fooBarTexts)],
        ),
      );

      expect(
        find.ancestor(of: find.text('bar'), matching: find.widgetWithText(Row, 'foo')),
        findsOneWidget,
      );
    });

    testWidgets('finds two matching ancestors, one descendant', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: Row(children: <Widget>[Row(children: fooBarTexts)]),
        ),
      );

      expect(find.ancestor(of: find.text('bar'), matching: find.byType(Row)), findsNWidgets(2));
    });

    testWidgets('fails with a descriptive message', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[
            Column(children: <Text>[Text('foo', textDirection: TextDirection.ltr)]),
            Text('bar', textDirection: TextDirection.ltr),
          ],
        ),
      );

      late TestFailure failure;
      try {
        expect(
          find.ancestor(of: find.text('bar'), matching: find.widgetWithText(Column, 'foo')),
          findsOneWidget,
        );
      } on TestFailure catch (e) {
        failure = e;
      }

      expect(failure, isNotNull);
      expect(
        failure.message,
        contains(
          'Actual: _AncestorWidgetFinder:<Found 0 widgets with type "Column" that are ancestors of widgets with text "foo" that are ancestors of widgets with text "bar"',
        ),
      );
    });

    testWidgets('Root not matched by default', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[Column(children: fooBarTexts)],
        ),
      );

      expect(
        find.ancestor(of: find.byType(Column), matching: find.widgetWithText(Column, 'foo')),
        findsNothing,
      );
    });

    testWidgets('Match the root', (WidgetTester tester) async {
      await tester.pumpWidget(
        const Row(
          textDirection: TextDirection.ltr,
          children: <Widget>[Column(children: fooBarTexts)],
        ),
      );

      expect(
        find.descendant(
          of: find.byType(Column),
          matching: find.widgetWithText(Column, 'foo'),
          matchRoot: true,
        ),
        findsOneWidget,
      );
    });

    testWidgets('is fast in deep tree', (WidgetTester tester) async {
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: _deepWidgetTree(
            depth: 500,
            child: Row(
              children: <Widget>[
                _deepWidgetTree(depth: 500, child: const Column(children: fooBarTexts)),
              ],
            ),
          ),
        ),
      );

      expect(find.ancestor(of: find.text('bar'), matching: find.byType(Row)), findsOneWidget);
    });
  });

  group('CommonSemanticsFinders', () {
    final Widget semanticsTree = _boilerplate(
      Semantics(
        container: true,
        header: true,
        readOnly: true,
        onCopy: () {},
        onLongPress: () {},
        value: 'value1',
        hint: 'hint1',
        label: 'label1',
        child: Semantics(
          container: true,
          textField: true,
          onSetText: (_) {},
          onPaste: () {},
          onLongPress: () {},
          value: 'value2',
          hint: 'hint2',
          label: 'label2',
          child: Semantics(
            container: true,
            readOnly: true,
            onCopy: () {},
            value: 'value3',
            hint: 'hint3',
            label: 'label3',
            child: Semantics(
              container: true,
              readOnly: true,
              onLongPress: () {},
              value: 'value4',
              hint: 'hint4',
              label: 'label4',
              child: Semantics(
                container: true,
                onLongPress: () {},
                onCopy: () {},
                value: 'value5',
                hint: 'hint5',
                label: 'label5',
              ),
            ),
          ),
        ),
      ),
    );

    group('ancestor', () {
      testWidgets('finds matching ancestor nodes', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final FinderBase<SemanticsNode> finder = find.semantics.ancestor(
          of: find.semantics.byLabel('label4'),
          matching: find.semantics.byAction(SemanticsAction.copy),
        );

        expect(finder, findsExactly(2));
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final FinderBase<SemanticsNode> finder = find.semantics.ancestor(
          of: find.semantics.byLabel('label4'),
          matching: find.semantics.byAction(SemanticsAction.copy),
        );

        try {
          expect(finder, findsExactly(3));
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains(
            'Actual: _AncestorSemanticsFinder:<Found 2 SemanticsNodes with action "SemanticsAction.copy" that are ancestors of SemanticsNodes with label "label4"',
          ),
        );
      });
    });

    group('descendant', () {
      testWidgets('finds matching descendant nodes', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final FinderBase<SemanticsNode> finder = find.semantics.descendant(
          of: find.semantics.byLabel('label4'),
          matching: find.semantics.byAction(SemanticsAction.copy),
        );

        expect(finder, findsOne);
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final FinderBase<SemanticsNode> finder = find.semantics.descendant(
          of: find.semantics.byLabel('label4'),
          matching: find.semantics.byAction(SemanticsAction.copy),
        );

        try {
          expect(finder, findsNothing);
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains(
            'Actual: _DescendantSemanticsFinder:<Found 1 SemanticsNode with action "SemanticsAction.copy" descending from SemanticsNode with label "label4"',
          ),
        );
      });
    });

    group('byPredicate', () {
      testWidgets('finds nodes matching given predicate', (WidgetTester tester) async {
        final RegExp replaceRegExp = RegExp(r'^[^\d]+');
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byPredicate((SemanticsNode node) {
          final int labelNum = int.tryParse(node.label.replaceAll(replaceRegExp, '')) ?? -1;
          return labelNum > 1;
        });

        expect(finder, findsExactly(4));
      });

      testWidgets('fails with default message', (WidgetTester tester) async {
        late TestFailure failure;
        final RegExp replaceRegExp = RegExp(r'^[^\d]+');
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byPredicate((SemanticsNode node) {
          final int labelNum = int.tryParse(node.label.replaceAll(replaceRegExp, '')) ?? -1;
          return labelNum > 1;
        });
        try {
          expect(finder, findsExactly(5));
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains('Actual: _PredicateSemanticsFinder:<Found 4 matching semantics predicate'),
        );
      });

      testWidgets('fails with given message', (WidgetTester tester) async {
        late TestFailure failure;
        const String expected = 'custom error message';
        final RegExp replaceRegExp = RegExp(r'^[^\d]+');
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byPredicate((SemanticsNode node) {
          final int labelNum = int.tryParse(node.label.replaceAll(replaceRegExp, '')) ?? -1;
          return labelNum > 1;
        }, describeMatch: (_) => expected);
        try {
          expect(finder, findsExactly(5));
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(failure.message, contains(expected));
      });
    });

    group('byLabel', () {
      testWidgets('finds nodes with matching label using String', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byLabel('label3');

        expect(finder, findsOne);
        expect(finder.found.first.label, 'label3');
      });

      testWidgets('finds nodes with matching label using RegEx', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byLabel(RegExp('^label.*'));

        expect(finder, findsExactly(5));
        expect(finder.found.every((SemanticsNode node) => node.label.startsWith('label')), isTrue);
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byLabel('label3');

        try {
          expect(finder, findsNothing);
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains('Actual: _PredicateSemanticsFinder:<Found 1 SemanticsNode with label "label3"'),
        );
      });
    });

    group('byValue', () {
      testWidgets('finds nodes with matching value using String', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byValue('value3');

        expect(finder, findsOne);
        expect(finder.found.first.value, 'value3');
      });

      testWidgets('finds nodes with matching value using RegEx', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byValue(RegExp('^value.*'));

        expect(finder, findsExactly(5));
        expect(finder.found.every((SemanticsNode node) => node.value.startsWith('value')), isTrue);
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byValue('value3');

        try {
          expect(finder, findsNothing);
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains('Actual: _PredicateSemanticsFinder:<Found 1 SemanticsNode with value "value3"'),
        );
      });
    });

    group('byHint', () {
      testWidgets('finds nodes with matching hint using String', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byHint('hint3');

        expect(finder, findsOne);
        expect(finder.found.first.hint, 'hint3');
      });

      testWidgets('finds nodes with matching hint using RegEx', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byHint(RegExp('^hint.*'));

        expect(finder, findsExactly(5));
        expect(finder.found.every((SemanticsNode node) => node.hint.startsWith('hint')), isTrue);
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byHint('hint3');

        try {
          expect(finder, findsNothing);
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains('Actual: _PredicateSemanticsFinder:<Found 1 SemanticsNode with hint "hint3"'),
        );
      });
    });

    group('byAction', () {
      testWidgets('finds nodes with matching action', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byAction(SemanticsAction.copy);

        expect(finder, findsExactly(3));
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byAction(SemanticsAction.copy);

        try {
          expect(finder, findsExactly(4));
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains(
            'Actual: _PredicateSemanticsFinder:<Found 3 SemanticsNodes with action "SemanticsAction.copy"',
          ),
        );
      });
    });

    group('byAnyAction', () {
      testWidgets('finds nodes with any matching actions', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byAnyAction(<SemanticsAction>[
          SemanticsAction.paste,
          SemanticsAction.longPress,
        ]);

        expect(finder, findsExactly(4));
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byAnyAction(<SemanticsAction>[
          SemanticsAction.paste,
          SemanticsAction.longPress,
        ]);

        try {
          expect(finder, findsExactly(5));
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains(
            'Actual: _PredicateSemanticsFinder:<Found 4 SemanticsNodes with any of the following actions: [SemanticsAction.paste, SemanticsAction.longPress]:',
          ),
        );
      });
    });

    group('byFlag', () {
      testWidgets('finds nodes with matching flag', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byFlag(SemanticsFlag.isReadOnly);

        expect(finder, findsExactly(3));
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byFlag(SemanticsFlag.isReadOnly);

        try {
          expect(finder, findsExactly(4));
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains(
            '_PredicateSemanticsFinder:<Found 3 SemanticsNodes with flag "SemanticsFlag.isReadOnly":',
          ),
        );
      });
    });

    group('byAnyFlag', () {
      testWidgets('finds nodes with any matching flag', (WidgetTester tester) async {
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byAnyFlag(<SemanticsFlag>[
          SemanticsFlag.isHeader,
          SemanticsFlag.isTextField,
        ]);

        expect(finder, findsExactly(2));
      });

      testWidgets('fails with descriptive message', (WidgetTester tester) async {
        late TestFailure failure;
        await tester.pumpWidget(semanticsTree);

        final SemanticsFinder finder = find.semantics.byAnyFlag(<SemanticsFlag>[
          SemanticsFlag.isHeader,
          SemanticsFlag.isTextField,
        ]);

        try {
          expect(finder, findsExactly(3));
        } on TestFailure catch (e) {
          failure = e;
        }

        expect(
          failure.message,
          contains(
            'Actual: _PredicateSemanticsFinder:<Found 2 SemanticsNodes with any of the following flags: [SemanticsFlag.isHeader, SemanticsFlag.isTextField]:',
          ),
        );
      });
    });

    group('scrollable', () {
      testWidgets('can find node that can scroll up', (WidgetTester tester) async {
        final ScrollController controller = ScrollController();
        await tester.pumpWidget(
          MaterialApp(
            home: SingleChildScrollView(
              controller: controller,
              child: const SizedBox(width: 100, height: 1000),
            ),
          ),
        );

        expect(
          find.semantics.scrollable(),
          containsSemantics(hasScrollUpAction: true, hasScrollDownAction: false),
        );
      });

      testWidgets('can find node that can scroll down', (WidgetTester tester) async {
        final ScrollController controller = ScrollController(initialScrollOffset: 400);
        await tester.pumpWidget(
          MaterialApp(
            home: SingleChildScrollView(
              controller: controller,
              child: const SizedBox(width: 100, height: 1000),
            ),
          ),
        );

        expect(
          find.semantics.scrollable(),
          containsSemantics(hasScrollUpAction: false, hasScrollDownAction: true),
        );
      });

      testWidgets('can find node that can scroll left', (WidgetTester tester) async {
        final ScrollController controller = ScrollController();
        await tester.pumpWidget(
          MaterialApp(
            home: SingleChildScrollView(
              scrollDirection: Axis.horizontal,
              controller: controller,
              child: const SizedBox(width: 1000, height: 100),
            ),
          ),
        );

        expect(
          find.semantics.scrollable(),
          containsSemantics(hasScrollLeftAction: true, hasScrollRightAction: false),
        );
      });

      testWidgets('can find node that can scroll right', (WidgetTester tester) async {
        final ScrollController controller = ScrollController(initialScrollOffset: 200);
        await tester.pumpWidget(
          MaterialApp(
            home: SingleChildScrollView(
              scrollDirection: Axis.horizontal,
              controller: controller,
              child: const SizedBox(width: 1000, height: 100),
            ),
          ),
        );

        expect(
          find.semantics.scrollable(),
          containsSemantics(hasScrollLeftAction: false, hasScrollRightAction: true),
        );
      });

      testWidgets('can exclusively find node that scrolls horizontally', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(
          const MaterialApp(
            home: Column(
              children: <Widget>[
                SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: SizedBox(width: 1000, height: 100),
                ),
                Expanded(child: SingleChildScrollView(child: SizedBox(width: 100, height: 1000))),
              ],
            ),
          ),
        );

        expect(find.semantics.scrollable(axis: Axis.horizontal), findsOne);
      });

      testWidgets('can exclusively find node that scrolls vertically', (WidgetTester tester) async {
        await tester.pumpWidget(
          const MaterialApp(
            home: Column(
              children: <Widget>[
                SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: SizedBox(width: 1000, height: 100),
                ),
                Expanded(child: SingleChildScrollView(child: SizedBox(width: 100, height: 1000))),
              ],
            ),
          ),
        );

        expect(find.semantics.scrollable(axis: Axis.vertical), findsOne);
      });
    });
  });

  group('FinderBase', () {
    group('describeMatch', () {
      test('is used for Finder and results', () {
        const String expected = 'Fake finder describe match';
        final _FakeFinder finder = _FakeFinder(
          describeMatchCallback: (_) {
            return expected;
          },
        );

        expect(finder.evaluate().toString(), contains(expected));
        expect(finder.toString(describeSelf: true), contains(expected));
      });

      for (int i = 0; i < 4; i++) {
        test('gets expected plurality for $i when reporting results from find', () {
          final Plurality expected = switch (i) {
            0 => Plurality.zero,
            1 => Plurality.one,
            _ => Plurality.many,
          };
          late final Plurality actual;
          final _FakeFinder finder = _FakeFinder(
            describeMatchCallback: (Plurality plurality) {
              actual = plurality;
              return 'Fake description';
            },
            findInCandidatesCallback:
                (_) => Iterable<String>.generate(i, (int index) => index.toString()),
          );
          finder.evaluate().toString();

          expect(actual, expected);
        });

        test('gets expected plurality for $i when reporting results from toString', () {
          final Plurality expected = switch (i) {
            0 => Plurality.zero,
            1 => Plurality.one,
            _ => Plurality.many,
          };
          late final Plurality actual;
          final _FakeFinder finder = _FakeFinder(
            describeMatchCallback: (Plurality plurality) {
              actual = plurality;
              return 'Fake description';
            },
            findInCandidatesCallback:
                (_) => Iterable<String>.generate(i, (int index) => index.toString()),
          );
          finder.toString();

          expect(actual, expected);
        });

        test('always gets many when describing finder', () {
          const Plurality expected = Plurality.many;
          late final Plurality actual;
          final _FakeFinder finder = _FakeFinder(
            describeMatchCallback: (Plurality plurality) {
              actual = plurality;
              return 'Fake description';
            },
            findInCandidatesCallback:
                (_) => Iterable<String>.generate(i, (int index) => index.toString()),
          );
          finder.toString(describeSelf: true);

          expect(actual, expected);
        });
      }
    });

    test('findInCandidates gets allCandidates', () {
      final List<String> expected = <String>['Test1', 'Test2', 'Test3', 'Test4'];
      late final List<String> actual;
      final _FakeFinder finder = _FakeFinder(
        allCandidatesCallback: () => expected,
        findInCandidatesCallback: (Iterable<String> candidates) {
          actual = candidates.toList();
          return candidates;
        },
      );
      finder.evaluate();

      expect(actual, expected);
    });

    test('allCandidates calculated for each find', () {
      const int expectedCallCount = 3;
      int actualCallCount = 0;
      final _FakeFinder finder = _FakeFinder(
        allCandidatesCallback: () {
          actualCallCount++;
          return <String>['test'];
        },
      );
      for (int i = 0; i < expectedCallCount; i++) {
        finder.evaluate();
      }

      expect(actualCallCount, expectedCallCount);
    });

    test('allCandidates only called once while caching', () {
      int actualCallCount = 0;
      final _FakeFinder finder = _FakeFinder(
        allCandidatesCallback: () {
          actualCallCount++;
          return <String>['test'];
        },
      );
      finder.runCached(() {
        for (int i = 0; i < 5; i++) {
          finder.evaluate();
          finder.tryEvaluate();
          final FinderResult<String> _ = finder.found;
        }
      });

      expect(actualCallCount, 1);
    });

    group('tryFind', () {
      test('returns false if no results', () {
        final _FakeFinder finder = _FakeFinder(findInCandidatesCallback: (_) => <String>[]);

        expect(finder.tryEvaluate(), false);
      });

      test('returns true if results are available', () {
        final _FakeFinder finder = _FakeFinder(
          findInCandidatesCallback: (_) => <String>['Results'],
        );

        expect(finder.tryEvaluate(), true);
      });
    });

    group('found', () {
      test('throws before any calls to evaluate or tryEvaluate', () {
        final _FakeFinder finder = _FakeFinder();

        expect(finder.hasFound, false);
        expect(() => finder.found, throwsAssertionError);
      });

      test('has same results as evaluate after call to evaluate', () {
        final _FakeFinder finder = _FakeFinder();
        final FinderResult<String> expected = finder.evaluate();

        expect(finder.hasFound, true);
        expect(finder.found, expected);
      });

      test('has expected results after call to tryFind', () {
        final Iterable<String> expected = Iterable<String>.generate(10, (int i) => i.toString());
        final _FakeFinder finder = _FakeFinder(findInCandidatesCallback: (_) => expected);
        finder.tryEvaluate();

        expect(finder.hasFound, true);
        expect(finder.found, orderedEquals(expected));
      });
    });
  });
}

Widget _boilerplate(Widget child) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Navigator(
      onGenerateRoute: (RouteSettings settings) {
        return MaterialPageRoute<void>(builder: (BuildContext context) => child);
      },
    ),
  );
}

class SimpleCustomSemanticsWidget extends LeafRenderObjectWidget {
  const SimpleCustomSemanticsWidget(this.label, {super.key});

  final String label;

  @override
  RenderObject createRenderObject(BuildContext context) => SimpleCustomSemanticsRenderObject(label);
}

class SimpleCustomSemanticsRenderObject extends RenderBox {
  SimpleCustomSemanticsRenderObject(this.label);

  final String label;

  @override
  bool get sizedByParent => true;

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return constraints.smallest;
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    super.describeSemanticsConfiguration(config);
    config
      ..label = label
      ..textDirection = TextDirection.ltr;
  }
}

class SimpleGenericWidget<T> extends StatelessWidget {
  const SimpleGenericWidget({required Widget child, super.key}) : _child = child;

  final Widget _child;

  @override
  Widget build(BuildContext context) {
    return _child;
  }
}

/// Wraps [child] in [depth] layers of [SizedBox]
Widget _deepWidgetTree({required int depth, required Widget child}) {
  Widget tree = child;
  for (int i = 0; i < depth; i += 1) {
    tree = SizedBox(child: tree);
  }
  return tree;
}

class _FakeFinder extends FinderBase<String> {
  _FakeFinder({
    this.allCandidatesCallback,
    this.describeMatchCallback,
    this.findInCandidatesCallback,
  });

  final Iterable<String> Function()? allCandidatesCallback;
  final DescribeMatchCallback? describeMatchCallback;
  final Iterable<String> Function(Iterable<String> candidates)? findInCandidatesCallback;

  @override
  Iterable<String> get allCandidates {
    return allCandidatesCallback?.call() ?? <String>['String 1', 'String 2', 'String 3'];
  }

  @override
  String describeMatch(Plurality plurality) {
    return describeMatchCallback?.call(plurality) ??
        switch (plurality) {
          Plurality.one => 'String',
          Plurality.many || Plurality.zero => 'Strings',
        };
  }

  @override
  Iterable<String> findInCandidates(Iterable<String> candidates) {
    return findInCandidatesCallback?.call(candidates) ?? candidates;
  }
}
