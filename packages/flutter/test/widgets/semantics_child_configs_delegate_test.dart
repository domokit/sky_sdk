// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Semantics can merge sibling group', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const SemanticsTag first = SemanticsTag('1');
    const SemanticsTag second = SemanticsTag('2');
    const SemanticsTag third = SemanticsTag('3');
    ChildSemanticsConfigurationsResult delegate(List<SemanticsConfiguration> configs) {
      expect(configs.length, 3);
      final ChildSemanticsConfigurationsResultBuilder builder =
          ChildSemanticsConfigurationsResultBuilder();
      final List<SemanticsConfiguration> sibling = <SemanticsConfiguration>[];
      // Merge first and third
      for (final SemanticsConfiguration config in configs) {
        if (config.tagsChildrenWith(first) || config.tagsChildrenWith(third)) {
          sibling.add(config);
        } else {
          builder.markAsMergeUp(config);
        }
      }
      builder.markAsSiblingMergeGroup(sibling);
      return builder.build();
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Semantics(
          label: 'parent',
          child: TestConfigDelegate(
            delegate: delegate,
            child: Column(
              children: <Widget>[
                Semantics(
                  label: '1',
                  tagForChildren: first,
                  child: const SizedBox(width: 100, height: 100),
                  // this tests that empty nodes disappear
                ),
                Semantics(
                  label: '2',
                  tagForChildren: second,
                  child: const SizedBox(width: 100, height: 100),
                ),
                Semantics(
                  label: '3',
                  tagForChildren: third,
                  child: const SizedBox(width: 100, height: 100),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(label: 'parent\n2'),
            TestSemantics.rootChild(label: '1\n3'),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Semantics can drop semantics config', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    const SemanticsTag first = SemanticsTag('1');
    const SemanticsTag second = SemanticsTag('2');
    const SemanticsTag third = SemanticsTag('3');
    ChildSemanticsConfigurationsResult delegate(List<SemanticsConfiguration> configs) {
      final ChildSemanticsConfigurationsResultBuilder builder =
          ChildSemanticsConfigurationsResultBuilder();
      // Merge first and third
      for (final SemanticsConfiguration config in configs) {
        if (config.tagsChildrenWith(first) || config.tagsChildrenWith(third)) {
          continue;
        }
        builder.markAsMergeUp(config);
      }
      return builder.build();
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Semantics(
          label: 'parent',
          child: TestConfigDelegate(
            delegate: delegate,
            child: Column(
              children: <Widget>[
                Semantics(
                  label: '1',
                  tagForChildren: first,
                  child: const SizedBox(width: 100, height: 100),
                  // this tests that empty nodes disappear
                ),
                Semantics(
                  label: '2',
                  tagForChildren: second,
                  child: const SizedBox(width: 100, height: 100),
                ),
                Semantics(
                  label: '3',
                  tagForChildren: third,
                  child: const SizedBox(width: 100, height: 100),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(children: <TestSemantics>[TestSemantics.rootChild(label: 'parent\n2')]),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('Semantics throws when mark the same config twice case 1', (
    WidgetTester tester,
  ) async {
    const SemanticsTag first = SemanticsTag('1');
    const SemanticsTag second = SemanticsTag('2');
    const SemanticsTag third = SemanticsTag('3');
    ChildSemanticsConfigurationsResult delegate(List<SemanticsConfiguration> configs) {
      final ChildSemanticsConfigurationsResultBuilder builder =
          ChildSemanticsConfigurationsResultBuilder();
      // Marks the same one twice.
      builder.markAsMergeUp(configs.first);
      builder.markAsMergeUp(configs.first);
      return builder.build();
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Semantics(
          label: 'parent',
          child: TestConfigDelegate(
            delegate: delegate,
            child: Column(
              children: <Widget>[
                Semantics(
                  label: '1',
                  tagForChildren: first,
                  child: const SizedBox(width: 100, height: 100),
                  // this tests that empty nodes disappear
                ),
                Semantics(
                  label: '2',
                  tagForChildren: second,
                  child: const SizedBox(width: 100, height: 100),
                ),
                Semantics(
                  label: '3',
                  tagForChildren: third,
                  child: const SizedBox(width: 100, height: 100),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isAssertionError);
  });

  testWidgets('Semantics throws when mark the same config twice case 2', (
    WidgetTester tester,
  ) async {
    const SemanticsTag first = SemanticsTag('1');
    const SemanticsTag second = SemanticsTag('2');
    const SemanticsTag third = SemanticsTag('3');
    ChildSemanticsConfigurationsResult delegate(List<SemanticsConfiguration> configs) {
      final ChildSemanticsConfigurationsResultBuilder builder =
          ChildSemanticsConfigurationsResultBuilder();
      // Marks the same one twice.
      builder.markAsMergeUp(configs.first);
      builder.markAsSiblingMergeGroup(<SemanticsConfiguration>[configs.first]);
      return builder.build();
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Semantics(
          label: 'parent',
          child: TestConfigDelegate(
            delegate: delegate,
            child: Column(
              children: <Widget>[
                Semantics(
                  label: '1',
                  tagForChildren: first,
                  child: const SizedBox(width: 100, height: 100),
                  // this tests that empty nodes disappear
                ),
                Semantics(
                  label: '2',
                  tagForChildren: second,
                  child: const SizedBox(width: 100, height: 100),
                ),
                Semantics(
                  label: '3',
                  tagForChildren: third,
                  child: const SizedBox(width: 100, height: 100),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isAssertionError);
  });

  testWidgets('RenderObject with semantics child delegate will mark correct boundary dirty', (
    WidgetTester tester,
  ) async {
    final UniqueKey inner = UniqueKey();
    final UniqueKey boundaryParent = UniqueKey();
    final UniqueKey grandBoundaryParent = UniqueKey();
    ChildSemanticsConfigurationsResult delegate(List<SemanticsConfiguration> configs) {
      final ChildSemanticsConfigurationsResultBuilder builder =
          ChildSemanticsConfigurationsResultBuilder();
      configs.forEach(builder.markAsMergeUp);
      return builder.build();
    }

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MarkSemanticsDirtySpy(
          key: grandBoundaryParent,
          child: MarkSemanticsDirtySpy(
            key: boundaryParent,
            child: TestConfigDelegate(
              delegate: delegate,
              child: Column(
                children: <Widget>[
                  Semantics(
                    label: 'label',
                    child: MarkSemanticsDirtySpy(key: inner, child: const Text('inner')),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    final RenderMarkSemanticsDirtySpy innerObject = tester
        .renderObject<RenderMarkSemanticsDirtySpy>(find.byKey(inner));
    final RenderTestConfigDelegate objectWithDelegate = tester
        .renderObject<RenderTestConfigDelegate>(find.byType(TestConfigDelegate));
    final RenderMarkSemanticsDirtySpy boundaryParentObject = tester
        .renderObject<RenderMarkSemanticsDirtySpy>(find.byKey(boundaryParent));
    final RenderMarkSemanticsDirtySpy grandBoundaryParentObject = tester
        .renderObject<RenderMarkSemanticsDirtySpy>(find.byKey(grandBoundaryParent));
    void resetBuildState() {
      innerObject.hasRebuildSemantics = false;
      boundaryParentObject.hasRebuildSemantics = false;
      grandBoundaryParentObject.hasRebuildSemantics = false;
    }

    // Sanity check
    expect(innerObject.hasRebuildSemantics, isTrue);
    expect(boundaryParentObject.hasRebuildSemantics, isTrue);
    expect(grandBoundaryParentObject.hasRebuildSemantics, isTrue);
    resetBuildState();

    innerObject.markNeedsSemanticsUpdate();
    await tester.pump();
    // Inner boundary should not trigger rebuild above it.
    expect(innerObject.hasRebuildSemantics, isTrue);
    expect(boundaryParentObject.hasRebuildSemantics, isFalse);
    expect(grandBoundaryParentObject.hasRebuildSemantics, isFalse);
    resetBuildState();

    objectWithDelegate.markNeedsSemanticsUpdate();
    await tester.pump();
    // object with delegate rebuilds up to grand parent boundary;
    expect(innerObject.hasRebuildSemantics, isTrue);
    expect(boundaryParentObject.hasRebuildSemantics, isTrue);
    expect(grandBoundaryParentObject.hasRebuildSemantics, isTrue);
    resetBuildState();

    boundaryParentObject.markNeedsSemanticsUpdate();
    await tester.pump();
    // Render objects in between child delegate and grand boundary parent does
    // not mark the grand boundary parent dirty because it should not change the
    // generated sibling nodes.
    expect(innerObject.hasRebuildSemantics, isTrue);
    expect(boundaryParentObject.hasRebuildSemantics, isTrue);
    expect(grandBoundaryParentObject.hasRebuildSemantics, isFalse);
  });
}

class MarkSemanticsDirtySpy extends SingleChildRenderObjectWidget {
  const MarkSemanticsDirtySpy({super.key, super.child});
  @override
  RenderMarkSemanticsDirtySpy createRenderObject(BuildContext context) =>
      RenderMarkSemanticsDirtySpy();
}

class RenderMarkSemanticsDirtySpy extends RenderProxyBox {
  RenderMarkSemanticsDirtySpy();
  bool hasRebuildSemantics = false;

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    config.isSemanticBoundary = true;
  }

  @override
  void assembleSemanticsNode(
    SemanticsNode node,
    SemanticsConfiguration config,
    Iterable<SemanticsNode> children,
  ) {
    hasRebuildSemantics = true;
  }
}

class TestConfigDelegate extends SingleChildRenderObjectWidget {
  const TestConfigDelegate({super.key, required this.delegate, super.child});
  final ChildSemanticsConfigurationsDelegate delegate;

  @override
  RenderTestConfigDelegate createRenderObject(BuildContext context) =>
      RenderTestConfigDelegate(delegate: delegate);

  @override
  void updateRenderObject(BuildContext context, RenderTestConfigDelegate renderObject) {
    renderObject.delegate = delegate;
  }
}

class RenderTestConfigDelegate extends RenderProxyBox {
  RenderTestConfigDelegate({ChildSemanticsConfigurationsDelegate? delegate}) : _delegate = delegate;

  ChildSemanticsConfigurationsDelegate? get delegate => _delegate;
  ChildSemanticsConfigurationsDelegate? _delegate;
  set delegate(ChildSemanticsConfigurationsDelegate? value) {
    if (value != _delegate) {
      markNeedsSemanticsUpdate();
    }
    _delegate = value;
  }

  @override
  void describeSemanticsConfiguration(SemanticsConfiguration config) {
    config.childConfigurationsDelegate = delegate;
  }
}
