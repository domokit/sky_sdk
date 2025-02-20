// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/dart/ast/visitor.dart';
import 'package:analyzer/dart/element/element.dart';
import 'package:analyzer/dart/element/type.dart';

import '../utils.dart';
import 'analyze.dart';

/// Verify that no RenderBox subclasses call compute* instead of get* for
/// computing the intrinsic dimensions. The [candidates] variable contains the
/// full list of RenderBox intrinsic method invocations checked by this rule.
final AnalyzeRule renderBoxIntrinsicCalculation = _RenderBoxIntrinsicCalculationRule();

const Map<String, String> candidates = <String, String>{
  'computeDryBaseline': 'getDryBaseline',
  'computeDryLayout': 'getDryLayout',
  'computeDistanceToActualBaseline': 'getDistanceToBaseline, or getDistanceToActualBaseline',
  'computeMaxIntrinsicHeight': 'getMaxIntrinsicHeight',
  'computeMinIntrinsicHeight': 'getMinIntrinsicHeight',
  'computeMaxIntrinsicWidth': 'getMaxIntrinsicWidth',
  'computeMinIntrinsicWidth': 'getMinIntrinsicWidth',
};

class _RenderBoxIntrinsicCalculationRule implements AnalyzeRule {
  final Map<ResolvedUnitResult, List<(AstNode, String)>> _errors =
      <ResolvedUnitResult, List<(AstNode, String)>>{};

  @override
  void applyTo(ResolvedUnitResult unit) {
    final _RenderBoxSubclassVisitor visitor = _RenderBoxSubclassVisitor();
    unit.unit.visitChildren(visitor);
    final List<(AstNode, String)> violationsInUnit = visitor.violationNodes;
    if (violationsInUnit.isNotEmpty) {
      _errors.putIfAbsent(unit, () => <(AstNode, String)>[]).addAll(violationsInUnit);
    }
  }

  @override
  void reportViolations(String workingDirectory) {
    if (_errors.isEmpty) {
      return;
    }

    foundError(<String>[
      for (final MapEntry<ResolvedUnitResult, List<(AstNode, String)>> entry in _errors.entries)
        for (final (AstNode node, String suggestion) in entry.value)
          '${locationInFile(entry.key, node, workingDirectory)}: ${node.parent}. Consider calling $suggestion instead.',
      '\n${bold}Typically the get* methods should be used to obtain the intrinsics of a RenderBox.$reset',
    ]);
  }

  @override
  String toString() => 'RenderBox subclass intrinsic calculation best practices';
}

class _RenderBoxSubclassVisitor extends RecursiveAstVisitor<void> {
  final List<(AstNode, String)> violationNodes = <(AstNode, String)>[];

  static final Map<InterfaceElement, bool> _isRenderBoxClassElementCache =
      <InterfaceElement, bool>{};
  // The cached version, call this method instead of _checkIfImplementsRenderBox.
  static bool _implementsRenderBox(InterfaceElement interfaceElement) {
    // Framework naming convention: a RenderObject subclass names have "Render" in its name.
    if (!interfaceElement.name.contains('Render')) {
      return false;
    }
    return interfaceElement.name == 'RenderBox' ||
        _isRenderBoxClassElementCache.putIfAbsent(
          interfaceElement,
          () => _checkIfImplementsRenderBox(interfaceElement),
        );
  }

  static bool _checkIfImplementsRenderBox(InterfaceElement element) {
    return element.allSupertypes.any(
      (InterfaceType interface) => _implementsRenderBox(interface.element),
    );
  }

  // We don't care about directives, comments, or asserts.
  @override
  void visitImportDirective(ImportDirective node) {}
  @override
  void visitExportDirective(ExportDirective node) {}
  @override
  void visitComment(Comment node) {}
  @override
  void visitAssertStatement(AssertStatement node) {}

  @override
  void visitClassDeclaration(ClassDeclaration node) {
    // Ignore the RenderBox class implementation: that's the only place the
    // compute* methods are supposed to be called.
    if (node.name.lexeme != 'RenderBox') {
      super.visitClassDeclaration(node);
    }
  }

  @override
  void visitSimpleIdentifier(SimpleIdentifier node) {
    final String? correctMethodName = candidates[node.name];
    if (correctMethodName == null) {
      return;
    }
    final bool isCallingSuperImplementation = switch (node.parent) {
      PropertyAccess(target: SuperExpression()) ||
      MethodInvocation(target: SuperExpression()) => true,
      _ => false,
    };
    if (isCallingSuperImplementation) {
      return;
    }
    final Element? declaredInClassElement = node.staticElement?.declaration?.enclosingElement3;
    if (declaredInClassElement is InterfaceElement &&
        _implementsRenderBox(declaredInClassElement)) {
      violationNodes.add((node, correctMethodName));
    }
  }
}
