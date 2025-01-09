// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/dart/ast/visitor.dart';
import 'package:apicheck/apicheck.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;
import 'package:test/test.dart';

void main() {
  final String flutterRoot = Engine.findWithin().flutterDir.path;

  checkApiConsistency(flutterRoot);
  checkNativeApi(flutterRoot);
}

/// Verify that duplicate Flutter API is consistent between implementations.
///
/// Flutter contains API that is required to match between implementations.
/// Notably, some enums such as those used by Flutter accessibility, appear in:
/// * dart:ui (native)
/// * dart:ui (web)
/// * embedder.h
/// * Internal enumerations used by iOS/Android
///
/// WARNING: The embedder API makes strong API/ABI stability guarantees. Care
/// must be taken when adding new enums, or values to existing enums. These
/// CANNOT be removed without breaking backward compatibility, which we have
/// never done. See the note at the top of `shell/platform/embedder/embedder.h`
/// for further details.
void checkApiConsistency(String flutterRoot) {
  test('AccessibilityFeatures enums match', () {
    // Dart values: _kFooBarIndex = 1 << N
    final List<String> uiFields = getDartClassFields(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'window.dart'),
      className: 'AccessibilityFeatures',
    );
    // C values: kFlutterAccessibilityFeatureFooBar = 1 << N,
    final List<String> embedderEnumValues = getCppEnumValues(
      sourcePath: path.join(flutterRoot, 'shell', 'platform', 'embedder', 'embedder.h'),
      enumName: 'FlutterAccessibilityFeature',
    );
    // C++ values: kFooBar = 1 << N,
    final List<String> internalEnumValues = getCppEnumClassValues(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'window', 'platform_configuration.h'),
      enumName: 'AccessibilityFeatureFlag',
    );
    // Java values: FOO_BAR(1 << N).
    final List<String> javaEnumValues =
        getJavaEnumValues(
          sourcePath: path.join(
            flutterRoot,
            'shell',
            'platform',
            'android',
            'io',
            'flutter',
            'view',
            'AccessibilityBridge.java',
          ),
          enumName: 'AccessibilityFeature',
        ).map(allCapsToCamelCase).toList();

    expect(embedderEnumValues, uiFields);
    expect(internalEnumValues, uiFields);
    expect(javaEnumValues, uiFields);
  });

  test('SemanticsAction enums match', () {
    // Dart values: _kFooBarIndex = 1 << N.
    final List<String> uiFields = getDartClassFields(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'semantics.dart'),
      className: 'SemanticsAction',
    );
    final List<String> webuiFields = getDartClassFields(
      sourcePath: path.join(flutterRoot, 'lib', 'web_ui', 'lib', 'semantics.dart'),
      className: 'SemanticsAction',
    );
    // C values: kFlutterSemanticsActionFooBar = 1 << N.
    final List<String> embedderEnumValues = getCppEnumValues(
      sourcePath: path.join(flutterRoot, 'shell', 'platform', 'embedder', 'embedder.h'),
      enumName: 'FlutterSemanticsAction',
    );
    // C++ values: kFooBar = 1 << N.
    final List<String> internalEnumValues = getCppEnumClassValues(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'semantics', 'semantics_node.h'),
      enumName: 'SemanticsAction',
    );
    // Java values: FOO_BAR(1 << N).
    final List<String> javaEnumValues =
        getJavaEnumValues(
          sourcePath: path.join(
            flutterRoot,
            'shell',
            'platform',
            'android',
            'io',
            'flutter',
            'view',
            'AccessibilityBridge.java',
          ),
          enumName: 'Action',
        ).map(allCapsToCamelCase).toList();

    expect(webuiFields, uiFields);
    expect(embedderEnumValues, uiFields);
    expect(internalEnumValues, uiFields);
    expect(javaEnumValues, uiFields);
  });

  test('AppLifecycleState enums match', () {
    // Dart values: _kFooBarIndex = 1 << N.
    final List<String> uiFields = getDartClassFields(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'platform_dispatcher.dart'),
      className: 'AppLifecycleState',
    );
    final List<String> webuiFields = getDartClassFields(
      sourcePath: path.join(flutterRoot, 'lib', 'web_ui', 'lib', 'platform_dispatcher.dart'),
      className: 'AppLifecycleState',
    );
    // C++ values: kFooBar = 1 << N.
    final List<String> internalEnumValues = getCppEnumClassValues(
      sourcePath: path.join(flutterRoot, 'shell', 'platform', 'common', 'app_lifecycle_state.h'),
      enumName: 'AppLifecycleState',
    );
    // Java values: FOO_BAR(1 << N).
    final List<String> javaEnumValues =
        getJavaEnumValues(
          sourcePath: path.join(
            flutterRoot,
            'shell',
            'platform',
            'android',
            'io',
            'flutter',
            'embedding',
            'engine',
            'systemchannels',
            'LifecycleChannel.java',
          ),
          enumName: 'AppLifecycleState',
        ).map(allCapsToCamelCase).toList();

    expect(webuiFields, uiFields);
    expect(internalEnumValues, uiFields);
    expect(javaEnumValues, uiFields);
  });

  test('SemanticsFlag enums match', () {
    // Dart values: _kFooBarIndex = 1 << N.
    final List<String> uiFields = getDartClassFields(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'semantics.dart'),
      className: 'SemanticsFlag',
    );
    final List<String> webuiFields = getDartClassFields(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'semantics.dart'),
      className: 'SemanticsFlag',
    );
    // C values: kFlutterSemanticsFlagFooBar = 1 << N.
    final List<String> embedderEnumValues = getCppEnumValues(
      sourcePath: path.join(flutterRoot, 'shell', 'platform', 'embedder', 'embedder.h'),
      enumName: 'FlutterSemanticsFlag',
    );
    // C++ values: kFooBar = 1 << N.
    final List<String> internalEnumValues = getCppEnumClassValues(
      sourcePath: path.join(flutterRoot, 'lib', 'ui', 'semantics', 'semantics_node.h'),
      enumName: 'SemanticsFlags',
    );
    // Java values: FOO_BAR(1 << N).
    final List<String> javaEnumValues =
        getJavaEnumValues(
          sourcePath: path.join(
            flutterRoot,
            'shell',
            'platform',
            'android',
            'io',
            'flutter',
            'view',
            'AccessibilityBridge.java',
          ),
          enumName: 'Flag',
        ).map(allCapsToCamelCase).toList();

    expect(webuiFields, uiFields);
    expect(embedderEnumValues, uiFields);
    expect(internalEnumValues, uiFields);
    expect(javaEnumValues, uiFields);
  });
}

/// Returns the CamelCase equivalent of an ALL_CAPS identifier.
String allCapsToCamelCase(String identifier) {
  final StringBuffer buffer = StringBuffer();
  for (final String word in identifier.split('_')) {
    if (word.isNotEmpty) {
      buffer.write(word[0]);
    }
    if (word.length > 1) {
      buffer.write(word.substring(1).toLowerCase());
    }
  }
  return buffer.toString();
}

/// Verify that the native functions in the dart:ui package do not use nullable
/// parameters that map to simple types such as numbers and strings.
///
/// The Tonic argument converters used by the native function implementations
/// expect that values of these types will not be null.
class NativeFunctionVisitor extends RecursiveAstVisitor<void> {
  final Set<String> simpleTypes = <String>{'int', 'double', 'bool', 'String'};

  List<String> errors = <String>[];

  @override
  void visitNativeFunctionBody(NativeFunctionBody node) {
    final MethodDeclaration? method = node.thisOrAncestorOfType<MethodDeclaration>();
    if (method != null) {
      if (method.parameters != null) {
        check(method.toString(), method.parameters!);
      }
      return;
    }

    final FunctionDeclaration? func = node.thisOrAncestorOfType<FunctionDeclaration>();
    if (func != null) {
      final FunctionExpression funcExpr = func.functionExpression;
      if (funcExpr.parameters != null) {
        check(func.toString(), funcExpr.parameters!);
      }
      return;
    }

    throw Exception('unreachable');
  }

  void check(String description, FormalParameterList parameters) {
    for (final FormalParameter parameter in parameters.parameters) {
      TypeAnnotation? type;
      if (parameter is SimpleFormalParameter) {
        type = parameter.type;
      } else if (parameter is DefaultFormalParameter) {
        type = (parameter.parameter as SimpleFormalParameter).type;
      }
      if (type! is NamedType) {
        final String name = (type as NamedType).name2.lexeme;
        if (type.question != null && simpleTypes.contains(name)) {
          errors.add(description);
          return;
        }
      }
    }
  }
}

void checkNativeApi(String flutterRoot) {
  test('Native API does not pass nullable parameters of simple types', () {
    final NativeFunctionVisitor visitor = NativeFunctionVisitor();
    visitUIUnits(flutterRoot, visitor);
    expect(visitor.errors, isEmpty);
  });
}
