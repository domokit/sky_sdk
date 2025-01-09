// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  Future<void> pumpContainer(WidgetTester tester, Widget child) async {
    await tester.pumpWidget(MaterialApp(home: child));
  }

  testWidgets('updates its registrar and delegate based on the number of selectables', (
    WidgetTester tester,
  ) async {
    final TestSelectionRegistrar registrar = TestSelectionRegistrar();
    final TestContainerDelegate delegate = TestContainerDelegate();
    addTearDown(delegate.dispose);

    await pumpContainer(
      tester,
      SelectionContainer(
        registrar: registrar,
        delegate: delegate,
        child: const Column(
          children: <Widget>[
            Text('column1', textDirection: TextDirection.ltr),
            Text('column2', textDirection: TextDirection.ltr),
            Text('column3', textDirection: TextDirection.ltr),
          ],
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(registrar.selectables.length, 1);
    expect(delegate.selectables.length, 3);
  });

  testWidgets('disabled container', (WidgetTester tester) async {
    final TestSelectionRegistrar registrar = TestSelectionRegistrar();
    final TestContainerDelegate delegate = TestContainerDelegate();
    addTearDown(delegate.dispose);

    await pumpContainer(
      tester,
      SelectionContainer(
        registrar: registrar,
        delegate: delegate,
        child: const SelectionContainer.disabled(
          child: Column(
            children: <Widget>[
              Text('column1', textDirection: TextDirection.ltr),
              Text('column2', textDirection: TextDirection.ltr),
              Text('column3', textDirection: TextDirection.ltr),
            ],
          ),
        ),
      ),
    );
    expect(registrar.selectables.length, 0);
    expect(delegate.selectables.length, 0);
  });

  testWidgets('Swapping out container delegate does not crash', (WidgetTester tester) async {
    final TestSelectionRegistrar registrar = TestSelectionRegistrar();
    final TestContainerDelegate delegate = TestContainerDelegate();
    addTearDown(delegate.dispose);
    final TestContainerDelegate childDelegate = TestContainerDelegate();
    addTearDown(childDelegate.dispose);

    await pumpContainer(
      tester,
      SelectionContainer(
        registrar: registrar,
        delegate: delegate,
        child: Builder(
          builder: (BuildContext context) {
            return SelectionContainer(
              registrar: SelectionContainer.maybeOf(context),
              delegate: childDelegate,
              child: const Text('dummy'),
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(registrar.selectables.length, 1);
    expect(delegate.value.hasContent, isTrue);

    final TestContainerDelegate newDelegate = TestContainerDelegate();
    addTearDown(newDelegate.dispose);

    await pumpContainer(
      tester,
      SelectionContainer(
        registrar: registrar,
        delegate: delegate,
        child: Builder(
          builder: (BuildContext context) {
            return SelectionContainer(
              registrar: SelectionContainer.maybeOf(context),
              delegate: newDelegate,
              child: const Text('dummy'),
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(registrar.selectables.length, 1);
    expect(delegate.value.hasContent, isTrue);
    expect(tester.takeException(), isNull);
  });

  testWidgets('Can update within one frame', (WidgetTester tester) async {
    final TestSelectionRegistrar registrar = TestSelectionRegistrar();
    final TestContainerDelegate delegate = TestContainerDelegate();
    addTearDown(delegate.dispose);
    final TestContainerDelegate childDelegate = TestContainerDelegate();
    addTearDown(childDelegate.dispose);

    await pumpContainer(
      tester,
      SelectionContainer(
        registrar: registrar,
        delegate: delegate,
        child: Builder(
          builder: (BuildContext context) {
            return SelectionContainer(
              registrar: SelectionContainer.maybeOf(context),
              delegate: childDelegate,
              child: const Text('dummy'), // The [Text] widget has an internal [SelectionContainer].
            );
          },
        ),
      ),
    );
    await tester.pump();
    // Should finish update after flushing the micro tasks.
    await tester.idle();
    expect(registrar.selectables.length, 1);
    expect(delegate.value.hasContent, isTrue);
  });

  testWidgets('selection container registers itself if there is a selectable child', (
    WidgetTester tester,
  ) async {
    final TestSelectionRegistrar registrar = TestSelectionRegistrar();
    final TestContainerDelegate delegate = TestContainerDelegate();
    addTearDown(delegate.dispose);

    await pumpContainer(
      tester,
      SelectionContainer(registrar: registrar, delegate: delegate, child: const Column()),
    );
    expect(registrar.selectables.length, 0);

    await pumpContainer(
      tester,
      SelectionContainer(
        registrar: registrar,
        delegate: delegate,
        child: const Column(children: <Widget>[Text('column1', textDirection: TextDirection.ltr)]),
      ),
    );
    await tester.pumpAndSettle();
    expect(registrar.selectables.length, 1);

    await pumpContainer(
      tester,
      SelectionContainer(registrar: registrar, delegate: delegate, child: const Column()),
    );
    await tester.pumpAndSettle();
    expect(registrar.selectables.length, 0);
  });

  testWidgets('selection container gets registrar from context if not provided', (
    WidgetTester tester,
  ) async {
    final TestSelectionRegistrar registrar = TestSelectionRegistrar();
    final TestContainerDelegate delegate = TestContainerDelegate();
    addTearDown(delegate.dispose);

    await pumpContainer(
      tester,
      SelectionRegistrarScope(
        registrar: registrar,
        child: SelectionContainer(
          delegate: delegate,
          child: const Column(
            children: <Widget>[Text('column1', textDirection: TextDirection.ltr)],
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(registrar.selectables.length, 1);
  });
}

class TestContainerDelegate extends MultiSelectableSelectionContainerDelegate {
  @override
  SelectionResult dispatchSelectionEventToChild(Selectable selectable, SelectionEvent event) {
    throw UnimplementedError();
  }

  @override
  void ensureChildUpdated(Selectable selectable) {
    throw UnimplementedError();
  }
}

class TestSelectionRegistrar extends SelectionRegistrar {
  final Set<Selectable> selectables = <Selectable>{};

  @override
  void add(Selectable selectable) => selectables.add(selectable);

  @override
  void remove(Selectable selectable) => selectables.remove(selectable);
}
