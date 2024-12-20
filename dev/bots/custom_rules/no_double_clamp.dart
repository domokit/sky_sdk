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

/// Verify that we use clampDouble instead of double.clamp for performance
/// reasons.
///
/// See also:
///   * https://github.com/flutter/flutter/pull/103559
///   * https://github.com/flutter/flutter/issues/103917
final AnalyzeRule noDoubleClamp = _NoDoubleClamp();

class _NoDoubleClamp implements AnalyzeRule {
  final Map<ResolvedUnitResult, List<AstNode>> _errors = <ResolvedUnitResult, List<AstNode>>{};

  @override
  void applyTo(ResolvedUnitResult unit) {
    final _DoubleClampVisitor visitor = _DoubleClampVisitor();
    unit.unit.visitChildren(visitor);
    final List<AstNode> violationsInUnit = visitor.clampAccessNodes;
    if (violationsInUnit.isNotEmpty) {
      _errors.putIfAbsent(unit, () => <AstNode>[]).addAll(violationsInUnit);
    }
  }

  @override
  void reportViolations(String workingDirectory) {
    if (_errors.isEmpty) {
      return;
    }

    foundError(<String>[
      for (final MapEntry<ResolvedUnitResult, List<AstNode>> entry in _errors.entries)
        for (final AstNode node in entry.value)
          '${locationInFile(entry.key, node, workingDirectory)}: ${node.parent}',
      '\n${bold}For performance reasons, we use a custom "clampDouble" function instead of using "double.clamp".$reset',
    ]);
  }

  @override
  String toString() => 'No "double.clamp"';
}

class _DoubleClampVisitor extends RecursiveAstVisitor<void> {
  final List<AstNode> clampAccessNodes = <AstNode>[];

  // We don't care about directives or comments.
  @override
  void visitImportDirective(ImportDirective node) {}

  @override
  void visitExportDirective(ExportDirective node) {}

  @override
  void visitComment(Comment node) {}

  @override
  void visitSimpleIdentifier(SimpleIdentifier node) {
    if (node.name != 'clamp' || node.staticElement is! MethodElement) {
      return;
    }
    final bool isAllowed = switch (node.parent) {
      // PropertyAccess matches num.clamp in tear-off form. Always prefer
      // doubleClamp over tear-offs: even when all 3 operands are int literals,
      // the return type doesn't get promoted to int:
      // final x = 1.clamp(0, 2); // The inferred return type is int, where as:
      // final f = 1.clamp;
      // final y = f(0, 2)       // The inferred return type is num.
      PropertyAccess(
        target: Expression(
          staticType: DartType(isDartCoreDouble: true) ||
              DartType(isDartCoreNum: true) ||
              DartType(isDartCoreInt: true),
        ),
      ) =>
        false,

      // Expressions like `final int x = 1.clamp(0, 2);` should be allowed.
      MethodInvocation(
        target: Expression(staticType: DartType(isDartCoreInt: true)),
        argumentList: ArgumentList(
          arguments: [
            Expression(staticType: DartType(isDartCoreInt: true)),
            Expression(staticType: DartType(isDartCoreInt: true)),
          ],
        ),
      ) =>
        true,

      // Otherwise, disallow num.clamp() invocations.
      MethodInvocation(
        target: Expression(
          staticType: DartType(isDartCoreDouble: true) ||
              DartType(isDartCoreNum: true) ||
              DartType(isDartCoreInt: true),
        ),
      ) =>
        false,

      _ => true,
    };
    if (!isAllowed) {
      clampAccessNodes.add(node);
    }
  }
}
