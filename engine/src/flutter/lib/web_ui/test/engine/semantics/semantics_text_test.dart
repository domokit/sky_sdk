// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';
import 'package:ui/ui.dart' as ui;

import '../../common/test_initialization.dart';
import 'semantics_tester.dart';

DateTime _testTime = DateTime(2023, 2, 17);
EngineSemantics semantics() => EngineSemantics.instance;
EngineSemanticsOwner owner() => EnginePlatformDispatcher.instance.implicitView!.semantics;

void main() {
  internalBootstrapBrowserTest(() {
    return testMain;
  });
}

Future<void> testMain() async {
  setUpImplicitView();

  test('renders label text as DOM', () async {
    semantics()
      ..debugOverrideTimestampFunction(() => _testTime)
      ..semanticsEnabled = true;

    // Add a node with a label - expect a <span>
    {
      final SemanticsTester tester = SemanticsTester(owner());
      tester.updateNode(
        id: 0,
        label: 'Hello',
        transform: Matrix4.identity().toFloat64(),
        rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
      );
      tester.apply();

      expectSemanticsTree(owner(), '''
        <sem><span>Hello</span></sem>''');

      final SemanticsObject node = owner().debugSemanticsTree![0]!;
      expect(node.semanticRole?.kind, SemanticRoleKind.generic);
      expect(
        reason: 'A node with a label should get a LabelAndValue role',
        node.semanticRole!.debugSemanticBehaviorTypes,
        contains(LabelAndValue),
      );
    }

    // Change label - expect the <span> to be updated.
    {
      final SemanticsTester tester = SemanticsTester(owner());
      tester.updateNode(
        id: 0,
        label: 'World',
        transform: Matrix4.identity().toFloat64(),
        rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
      );
      tester.apply();

      expectSemanticsTree(owner(), '''
        <sem><span>World</span></sem>''');
    }

    // Empty the label - expect the <span> to be removed.
    {
      final SemanticsTester tester = SemanticsTester(owner());
      tester.updateNode(
        id: 0,
        label: '',
        transform: Matrix4.identity().toFloat64(),
        rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
      );
      tester.apply();

      expectSemanticsTree(owner(), '<sem></sem>');
    }

    semantics().semanticsEnabled = false;
  });

  test('does not add a span in container nodes', () async {
    semantics()
      ..debugOverrideTimestampFunction(() => _testTime)
      ..semanticsEnabled = true;

    final SemanticsTester tester = SemanticsTester(owner());
    tester.updateNode(
      id: 0,
      label: 'I am a parent',
      transform: Matrix4.identity().toFloat64(),
      rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
      children: <SemanticsNodeUpdate>[
        tester.updateNode(
          id: 1,
          label: 'I am a child',
          transform: Matrix4.identity().toFloat64(),
          rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
        ),
      ],
    );
    tester.apply();

    expectSemanticsTree(owner(), '''
      <sem aria-label="I am a parent" role="group">
        <sem-c>
          <sem><span>I am a child</span></sem>
        </sem-c>
      </sem>''');

    semantics().semanticsEnabled = false;
  });

  test('adds a span when a leaf becomes a parent, and vice versa', () async {
    semantics()
      ..debugOverrideTimestampFunction(() => _testTime)
      ..semanticsEnabled = true;

    // A leaf node with a label - expect <span>
    {
      final SemanticsTester tester = SemanticsTester(owner());
      tester.updateNode(
        id: 0,
        label: 'I am a leaf',
        transform: Matrix4.identity().toFloat64(),
        rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
      );
      tester.apply();

      expectSemanticsTree(owner(), '''
        <sem><span>I am a leaf</span></sem>''');
    }

    // Add a child - expect <span> to be removed from the parent.
    {
      final SemanticsTester tester = SemanticsTester(owner());
      tester.updateNode(
        id: 0,
        label: 'I am a parent',
        transform: Matrix4.identity().toFloat64(),
        rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
        children: <SemanticsNodeUpdate>[
          tester.updateNode(
            id: 1,
            label: 'I am a child',
            transform: Matrix4.identity().toFloat64(),
            rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
          ),
        ],
      );
      tester.apply();

      expectSemanticsTree(owner(), '''
        <sem aria-label="I am a parent" role="group">
          <sem-c>
            <sem><span>I am a child</span></sem>
          </sem-c>
        </sem>''');
    }

    // Remove the child - expect the <span> to be readded to the former parent.
    {
      final SemanticsTester tester = SemanticsTester(owner());
      tester.updateNode(
        id: 0,
        label: 'I am a leaf again',
        transform: Matrix4.identity().toFloat64(),
        rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
      );
      tester.apply();

      expectSemanticsTree(owner(), '''
        <sem><span>I am a leaf again</span></sem>''');
    }

    semantics().semanticsEnabled = false;
  });

  test('focusAsRouteDefault focuses on <span> when sized span is used', () async {
    semantics()
      ..debugOverrideTimestampFunction(() => _testTime)
      ..semanticsEnabled = true;

    final SemanticsTester tester = SemanticsTester(owner());
    tester.updateNode(
      id: 0,
      label: 'Hello',
      transform: Matrix4.identity().toFloat64(),
      rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
    );
    tester.apply();

    expectSemanticsTree(owner(), '''
      <sem><span>Hello</span></sem>''');

    final SemanticsObject node = owner().debugSemanticsTree![0]!;
    final DomElement span = node.element.querySelector('span')!;

    expect(span.getAttribute('tabindex'), isNull);
    node.semanticRole!.focusAsRouteDefault();
    expect(span.getAttribute('tabindex'), '-1');
    expect(domDocument.activeElement, span);

    semantics().semanticsEnabled = false;
  });

  test('focusAsRouteDefault focuses on <flt-semantics> when DOM text is used', () async {
    semantics()
      ..debugOverrideTimestampFunction(() => _testTime)
      ..semanticsEnabled = true;

    final SemanticsTester tester = SemanticsTester(owner());
    tester.updateNode(
      id: 0,
      label: 'Hello',
      transform: Matrix4.identity().toFloat64(),
      rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
    );
    tester.apply();

    final SemanticsObject node = owner().debugSemanticsTree![0]!;

    // Set DOM text as preferred representation
    final LabelAndValue lav = node.semanticRole!.labelAndValue!;
    lav.preferredRepresentation = LabelRepresentation.domText;
    lav.update();

    expectSemanticsTree(owner(), '''
      <sem>Hello</sem>''');

    expect(node.element.getAttribute('tabindex'), isNull);
    node.semanticRole!.focusAsRouteDefault();
    expect(node.element.getAttribute('tabindex'), '-1');
    expect(domDocument.activeElement, node.element);

    semantics().semanticsEnabled = false;
  });

  test('focusAsRouteDefault focuses on <flt-semantics> when aria-label is used', () async {
    semantics()
      ..debugOverrideTimestampFunction(() => _testTime)
      ..semanticsEnabled = true;

    final SemanticsTester tester = SemanticsTester(owner());
    tester.updateNode(
      id: 0,
      label: 'Hello',
      transform: Matrix4.identity().toFloat64(),
      rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
    );
    tester.apply();

    final SemanticsObject node = owner().debugSemanticsTree![0]!;

    // Set DOM text as preferred representation
    final LabelAndValue lav = node.semanticRole!.labelAndValue!;
    lav.preferredRepresentation = LabelRepresentation.ariaLabel;
    lav.update();

    expectSemanticsTree(owner(), '''
      <sem aria-label="Hello"></sem>''');

    expect(node.element.getAttribute('tabindex'), isNull);
    node.semanticRole!.focusAsRouteDefault();
    expect(node.element.getAttribute('tabindex'), '-1');
    expect(domDocument.activeElement, node.element);

    semantics().semanticsEnabled = false;
  });

  test('The <span> ignores pointer events', () async {
    semantics()
      ..debugOverrideTimestampFunction(() => _testTime)
      ..semanticsEnabled = true;

    final SemanticsTester tester = SemanticsTester(owner());
    tester.updateNode(
      id: 0,
      label: 'Ignore pointer events',
      transform: Matrix4.identity().toFloat64(),
      rect: const ui.Rect.fromLTRB(0, 0, 100, 50),
    );
    tester.apply();

    expectSemanticsTree(owner(), '''
      <sem>
        <span style="pointer-events: none">Ignore pointer events</span>
      </sem>''');

    semantics().semanticsEnabled = false;
  });
}
