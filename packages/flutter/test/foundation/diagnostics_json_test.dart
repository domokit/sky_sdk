// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Element diagnostics json includes widgetRuntimeType', () async {
    final Element element = _TestElement();

    final Map<String, Object?> json = element.toDiagnosticsNode().toJsonMap(
      const DiagnosticsSerializationDelegate(),
    );
    expect(json['widgetRuntimeType'], 'Placeholder');
    expect(json['stateful'], isFalse);
  });

  test('StatefulElement diagnostics are stateful', () {
    final Element element = StatefulElement(const Tooltip(message: 'foo'));

    final Map<String, Object?> json = element.toDiagnosticsNode().toJsonMap(
      const DiagnosticsSerializationDelegate(),
    );
    expect(json['widgetRuntimeType'], 'Tooltip');
    expect(json['stateful'], isTrue);
  });

  group('Serialization', () {
    // These are always included.
    const List<String> defaultDiagnosticKeys = <String>['description'];
    // These are only included when fullDetails = false.
    const List<String> essentialDiagnosticKeys = <String>['shouldIndent'];
    // These are only included with fullDetails = true.
    const List<String> detailedDiagnosticKeys = <String>['type', 'hasChildren', 'allowWrap'];

    final TestTree testTree = TestTree(
      properties: <DiagnosticsNode>[
        StringProperty('stringProperty1', 'value1', quoted: false),
        DoubleProperty('doubleProperty1', 42.5),
        DoubleProperty('roundedProperty', 1.0 / 3.0),
        StringProperty('DO_NOT_SHOW', 'DO_NOT_SHOW', level: DiagnosticLevel.hidden, quoted: false),
        DiagnosticsProperty<Object>('DO_NOT_SHOW_NULL', null, defaultValue: null),
        DiagnosticsProperty<Object>('nullProperty', null),
        StringProperty('node_type', '<root node>', showName: false, quoted: false),
      ],
      children: <TestTree>[
        TestTree(name: 'node A'),
        TestTree(
          name: 'node B',
          properties: <DiagnosticsNode>[
            StringProperty('p1', 'v1', quoted: false),
            StringProperty('p2', 'v2', quoted: false),
          ],
          children: <TestTree>[
            TestTree(name: 'node B1'),
            TestTree(
              name: 'node B2',
              properties: <DiagnosticsNode>[StringProperty('property1', 'value1', quoted: false)],
            ),
            TestTree(
              name: 'node B3',
              properties: <DiagnosticsNode>[
                StringProperty('node_type', '<leaf node>', showName: false, quoted: false),
                IntProperty('foo', 42),
              ],
            ),
          ],
        ),
        TestTree(
          name: 'node C',
          properties: <DiagnosticsNode>[
            StringProperty('foo', 'multi\nline\nvalue!', quoted: false),
          ],
        ),
      ],
    );

    test('default', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        const DiagnosticsSerializationDelegate(),
      );
      expect(result.containsKey('properties'), isFalse);
      expect(result.containsKey('children'), isFalse);

      for (final String keyName in defaultDiagnosticKeys) {
        expect(result.containsKey(keyName), isTrue, reason: '$keyName is included.');
      }
      for (final String keyName in detailedDiagnosticKeys) {
        expect(result.containsKey(keyName), isTrue, reason: '$keyName is included.');
      }
    });

    test('iterative implementation (without full details)', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMapIterative(
        const DiagnosticsSerializationDelegate(),
      );
      expect(result.containsKey('properties'), isFalse);
      expect(result.containsKey('children'), isFalse);
      for (final String keyName in defaultDiagnosticKeys) {
        expect(result.containsKey(keyName), isTrue, reason: '$keyName is included.');
      }
      for (final String keyName in essentialDiagnosticKeys) {
        expect(result.containsKey(keyName), isTrue, reason: '$keyName is included.');
      }
      for (final String keyName in detailedDiagnosticKeys) {
        expect(result.containsKey(keyName), isFalse, reason: '$keyName is  not included.');
      }

      // The truncated value should not be included if it is false.
      expect(result['truncated'] == null || result['truncated'] == true, isTrue);
    });

    test('subtreeDepth 1', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        const DiagnosticsSerializationDelegate(subtreeDepth: 1),
      );
      expect(result.containsKey('properties'), isFalse);
      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children[0].containsKey('children'), isFalse);
      expect(children[1].containsKey('children'), isFalse);
      expect(children[2].containsKey('children'), isFalse);
    });

    test('subtreeDepth 5', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        const DiagnosticsSerializationDelegate(subtreeDepth: 5),
      );
      expect(result.containsKey('properties'), isFalse);
      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children[0]['children'], hasLength(0));
      expect(children[1]['children'], hasLength(3));
      expect(children[2]['children'], hasLength(0));
    });

    test('includeProperties', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        const DiagnosticsSerializationDelegate(includeProperties: true),
      );
      expect(result.containsKey('children'), isFalse);
      expect(result['properties'], hasLength(7));
    });

    test('includeProperties with subtreedepth 1', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        const DiagnosticsSerializationDelegate(includeProperties: true, subtreeDepth: 1),
      );
      expect(result['properties'], hasLength(7));
      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children, hasLength(3));
      expect(children[0]['properties'], hasLength(0));
      expect(children[1]['properties'], hasLength(2));
      expect(children[2]['properties'], hasLength(1));
    });

    test('additionalNodeProperties', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        const TestDiagnosticsSerializationDelegate(
          includeProperties: true,
          subtreeDepth: 1,
          additionalNodePropertiesMap: <String, Object>{'foo': true},
        ),
      );
      expect(result['foo'], isTrue);
      final List<Map<String, Object?>> properties =
          result['properties']! as List<Map<String, Object?>>;
      expect(properties, hasLength(7));
      expect(properties.every((Map<String, Object?> property) => property['foo'] == true), isTrue);

      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children, hasLength(3));
      expect(children.every((Map<String, Object?> child) => child['foo'] == true), isTrue);
    });

    test('filterProperties - sublist', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        TestDiagnosticsSerializationDelegate(
          includeProperties: true,
          propertyFilter: (List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
            return nodes.whereType<StringProperty>().toList();
          },
        ),
      );
      final List<Map<String, Object?>> properties =
          result['properties']! as List<Map<String, Object?>>;
      expect(properties, hasLength(3));
      expect(
        properties.every((Map<String, Object?> property) => property['type'] == 'StringProperty'),
        isTrue,
      );
    });

    test('filterProperties - replace', () {
      bool replaced = false;
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        TestDiagnosticsSerializationDelegate(
          includeProperties: true,
          propertyFilter: (List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
            if (replaced) {
              return nodes;
            }
            replaced = true;
            return <DiagnosticsNode>[StringProperty('foo', 'bar')];
          },
        ),
      );
      final List<Map<String, Object?>> properties =
          result['properties']! as List<Map<String, Object?>>;
      expect(properties, hasLength(1));
      expect(properties.single['name'], 'foo');
    });

    test('filterChildren - sublist', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        TestDiagnosticsSerializationDelegate(
          subtreeDepth: 1,
          childFilter: (List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
            return nodes.where((DiagnosticsNode node) => node.getProperties().isEmpty).toList();
          },
        ),
      );
      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children, hasLength(1));
    });

    test('filterChildren - replace', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        TestDiagnosticsSerializationDelegate(
          subtreeDepth: 1,
          childFilter: (List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
            return nodes.expand((DiagnosticsNode node) => node.getChildren()).toList();
          },
        ),
      );
      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children, hasLength(3));
      expect(children.first['name'], 'child node B1');
    });

    test('nodeTruncator', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        TestDiagnosticsSerializationDelegate(
          subtreeDepth: 5,
          includeProperties: true,
          nodeTruncator: (List<DiagnosticsNode> nodes, DiagnosticsNode? owner) {
            return nodes.take(2).toList();
          },
        ),
      );
      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children, hasLength(3));
      expect(children.last['truncated'], isTrue);

      final List<Map<String, Object?>> properties =
          result['properties']! as List<Map<String, Object?>>;
      expect(properties, hasLength(3));
      expect(properties.last['truncated'], isTrue);
    });

    test('delegateForAddingNodes', () {
      final Map<String, Object?> result = testTree.toDiagnosticsNode().toJsonMap(
        TestDiagnosticsSerializationDelegate(
          subtreeDepth: 5,
          includeProperties: true,
          nodeDelegator: (DiagnosticsNode node, DiagnosticsSerializationDelegate delegate) {
            return delegate.copyWith(includeProperties: false);
          },
        ),
      );
      final List<Map<String, Object?>> properties =
          result['properties']! as List<Map<String, Object?>>;
      expect(properties, hasLength(7));
      expect(
        properties.every((Map<String, Object?> property) => !property.containsKey('properties')),
        isTrue,
      );

      final List<Map<String, Object?>> children = result['children']! as List<Map<String, Object?>>;
      expect(children, hasLength(3));
      expect(
        children.every((Map<String, Object?> child) => !child.containsKey('properties')),
        isTrue,
      );
    });
  });
}

class _TestElement extends Element {
  _TestElement() : super(const Placeholder());

  @override
  bool get debugDoingBuild => throw UnimplementedError();
}

class TestTree extends Object with DiagnosticableTreeMixin {
  TestTree({
    this.name = '',
    this.style,
    this.children = const <TestTree>[],
    this.properties = const <DiagnosticsNode>[],
  });

  final String name;
  final List<TestTree> children;
  final List<DiagnosticsNode> properties;
  final DiagnosticsTreeStyle? style;

  @override
  List<DiagnosticsNode> debugDescribeChildren() => <DiagnosticsNode>[
    for (final TestTree child in children)
      child.toDiagnosticsNode(name: 'child ${child.name}', style: child.style),
  ];

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    if (style != null) {
      properties.defaultDiagnosticsTreeStyle = style!;
    }

    this.properties.forEach(properties.add);
  }
}

typedef NodeDelegator =
    DiagnosticsSerializationDelegate Function(
      DiagnosticsNode node,
      TestDiagnosticsSerializationDelegate delegate,
    );
typedef NodeTruncator =
    List<DiagnosticsNode> Function(List<DiagnosticsNode> nodes, DiagnosticsNode? owner);
typedef NodeFilter =
    List<DiagnosticsNode> Function(List<DiagnosticsNode> nodes, DiagnosticsNode owner);

class TestDiagnosticsSerializationDelegate implements DiagnosticsSerializationDelegate {
  const TestDiagnosticsSerializationDelegate({
    this.includeProperties = false,
    this.subtreeDepth = 0,
    this.additionalNodePropertiesMap = const <String, Object>{},
    this.childFilter,
    this.propertyFilter,
    this.nodeTruncator,
    this.nodeDelegator,
  });

  final Map<String, Object> additionalNodePropertiesMap;
  final NodeFilter? childFilter;
  final NodeFilter? propertyFilter;
  final NodeTruncator? nodeTruncator;
  final NodeDelegator? nodeDelegator;

  @override
  Map<String, Object> additionalNodeProperties(DiagnosticsNode node, {bool fullDetails = true}) {
    return additionalNodePropertiesMap;
  }

  @override
  DiagnosticsSerializationDelegate delegateForNode(DiagnosticsNode node) {
    if (nodeDelegator != null) {
      return nodeDelegator!(node, this);
    }
    return subtreeDepth > 0 ? copyWith(subtreeDepth: subtreeDepth - 1) : this;
  }

  @override
  bool get expandPropertyValues => false;

  @override
  List<DiagnosticsNode> filterChildren(List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
    return childFilter?.call(nodes, owner) ?? nodes;
  }

  @override
  List<DiagnosticsNode> filterProperties(List<DiagnosticsNode> nodes, DiagnosticsNode owner) {
    return propertyFilter?.call(nodes, owner) ?? nodes;
  }

  @override
  final bool includeProperties;

  @override
  final int subtreeDepth;

  @override
  List<DiagnosticsNode> truncateNodesList(List<DiagnosticsNode> nodes, DiagnosticsNode? owner) {
    return nodeTruncator?.call(nodes, owner) ?? nodes;
  }

  @override
  DiagnosticsSerializationDelegate copyWith({int? subtreeDepth, bool? includeProperties}) {
    return TestDiagnosticsSerializationDelegate(
      includeProperties: includeProperties ?? this.includeProperties,
      subtreeDepth: subtreeDepth ?? this.subtreeDepth,
      additionalNodePropertiesMap: additionalNodePropertiesMap,
      childFilter: childFilter,
      propertyFilter: propertyFilter,
      nodeTruncator: nodeTruncator,
      nodeDelegator: nodeDelegator,
    );
  }
}
