// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// TODO(nate-thegrate): remove this file if @protected changes, or add a test if it doesn't.
// https://github.com/dart-lang/sdk/issues/57094

import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/dart/ast/visitor.dart';
import 'package:analyzer/dart/element/element.dart';
import 'package:analyzer/dart/element/type.dart';

import '../utils.dart';
import 'analyze.dart';

final AnalyzeRule protectPublicStateSubtypes = _ProtectPublicStateSubtypes();

class _ProtectPublicStateSubtypes implements AnalyzeRule {
  final Map<ResolvedUnitResult, List<MethodDeclaration>> _errors =
      <ResolvedUnitResult, List<MethodDeclaration>>{};

  @override
  void applyTo(ResolvedUnitResult unit) {
    final _StateSubclassVisitor visitor = _StateSubclassVisitor();
    unit.unit.visitChildren(visitor);
    final List<MethodDeclaration> unprotected = visitor.unprotectedMethods;
    if (unprotected.isNotEmpty) {
      _errors.putIfAbsent(unit, () => <MethodDeclaration>[]).addAll(unprotected);
    }
  }

  @override
  void reportViolations(String workingDirectory) {
    if (_errors.isEmpty) {
      return;
    }

    foundError(<String>[
      for (final MapEntry<ResolvedUnitResult, List<MethodDeclaration>> entry in _errors.entries)
        for (final MethodDeclaration method in entry.value)
          '${locationInFile(entry.key, method, workingDirectory)}: $method - missing "@protected" annotation.',
      '\nPublic State subtypes should add @protected when overriding methods,',
      'to avoid exposing internal logic to developers.',
    ]);
  }

  @override
  String toString() => 'Add "@protected" to public State subtypes';
}

class _StateSubclassVisitor extends SimpleAstVisitor<void> {
  final List<MethodDeclaration> unprotectedMethods = <MethodDeclaration>[];

  /// Holds the `State` class [DartType].
  static DartType? stateType;

  static bool isPublicStateSubtype(InterfaceElement element) {
    if (!element.isPublic) {
      return false;
    }
    if (stateType != null) {
      return element.allSupertypes.contains(stateType);
    }
    for (final InterfaceType superType in element.allSupertypes) {
      if (superType.element.name == 'State') {
        stateType = superType;
        return true;
      }
    }
    return false;
  }

  @override
  void visitClassDeclaration(ClassDeclaration node) {
    if (isPublicStateSubtype(node.declaredElement!)) {
      node.visitChildren(this);
    }
  }

  /// Checks whether overridden `State` methods have the `@protected` annotation,
  /// and adds the declaration to [unprotectedMethods] if not.
  @override
  void visitMethodDeclaration(MethodDeclaration node) {
    switch (node.name.lexeme) {
      case 'initState':
      case 'didUpdateWidget':
      case 'didChangeDependencies':
      case 'reassemble':
      case 'deactivate':
      case 'activate':
      case 'dispose':
      case 'build':
      case 'debugFillProperties':
        if (!node.declaredElement!.hasProtected) {
          unprotectedMethods.add(node);
        }
    }
  }
}
