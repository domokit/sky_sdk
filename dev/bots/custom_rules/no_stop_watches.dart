// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/dart/ast/visitor.dart';
import 'package:analyzer/dart/element/element.dart';
import 'package:analyzer/dart/element/type.dart';
import 'package:path/path.dart' as path;

import '../utils.dart';
import 'analyze.dart';

// The comment pattern representing the "flutter_ignore" inline directive that
// indicates the line should be exempt from the stopwatch check.
final Pattern _ignoreStopwatch = RegExp(r'// flutter_ignore: .*stopwatch .*\(see analyze\.dart\)');

/// Use of Stopwatches can introduce test flakes as the logical time of a
/// stopwatch can fall out of sync with the mocked time of FakeAsync in testing.
/// The Clock object provides a safe stopwatch instead, which is paired with
/// FakeAsync as part of the test binding.
final AnalyzeRule noStopwatches = _NoStopwatches();

class _NoStopwatches implements AnalyzeRule {
  final Map<ResolvedUnitResult, List<AstNode>> _errors = <ResolvedUnitResult, List<AstNode>>{};

  @override
  void applyTo(ResolvedUnitResult unit) {
    final _StopwatchVisitor visitor = _StopwatchVisitor(unit);
    unit.unit.visitChildren(visitor);
    final List<AstNode> violationsInUnit = visitor.stopwatchAccessNodes;
    if (violationsInUnit.isNotEmpty) {
      _errors.putIfAbsent(unit, () => <AstNode>[]).addAll(violationsInUnit);
    }
  }

  @override
  void reportViolations(String workingDirectory) {
    if (_errors.isEmpty) {
      return;
    }

    String locationInFile(ResolvedUnitResult unit, AstNode node) {
      return '${path.relative(path.relative(unit.path, from: workingDirectory))}:${unit.lineInfo.getLocation(node.offset).lineNumber}';
    }

    foundError(<String>[
      for (final MapEntry<ResolvedUnitResult, List<AstNode>> entry in _errors.entries)
        for (final AstNode node in entry.value)
          '${locationInFile(entry.key, node)}: ${node.parent}',
      '\n${bold}Stopwatches introduce flakes by falling out of sync with the FakeAsync used in testing.$reset',
      'A Stopwatch that stays in sync with FakeAsync is available through the Gesture or Test bindings, through samplingClock.',
    ]);
  }

  @override
  String toString() => 'No "Stopwatch"';
}

// This visitor finds invocation sites of Stopwatch (and subclasses) constructors
// and references to "external" functions that return a Stopwatch (and subclasses),
// including constructors, and put them in the stopwatchAccessNodes list.
class _StopwatchVisitor extends RecursiveAstVisitor<void> {
  _StopwatchVisitor(this.compilationUnit);

  final ResolvedUnitResult compilationUnit;

  final List<AstNode> stopwatchAccessNodes = <AstNode>[];

  final Map<ClassElement, bool> _isStopwatchClassElementCache = <ClassElement, bool>{};

  bool _checkIfImplementsStopwatchRecursively(ClassElement classElement) {
    if (classElement.library.isDartCore) {
      return classElement.name == 'Stopwatch';
    }
    return classElement.allSupertypes.any((InterfaceType interface) {
      final InterfaceElement interfaceElement = interface.element;
      return interfaceElement is ClassElement && _implementsStopwatch(interfaceElement);
    });
  }

  // The cached version, call this method instead of _checkIfImplementsStopwatchRecursively.
  bool _implementsStopwatch(ClassElement classElement) {
    return classElement.library.isDartCore
        ? classElement.name == 'Stopwatch'
        : _isStopwatchClassElementCache.putIfAbsent(
          classElement,
          () => _checkIfImplementsStopwatchRecursively(classElement),
        );
  }

  bool _isInternal(LibraryElement libraryElement) {
    return path.isWithin(
      compilationUnit.session.analysisContext.contextRoot.root.path,
      libraryElement.source.fullName,
    );
  }

  bool _hasTrailingFlutterIgnore(AstNode node) {
    return compilationUnit.content
        .substring(
          node.offset + node.length,
          compilationUnit.lineInfo.getOffsetOfLineAfter(node.offset + node.length),
        )
        .contains(_ignoreStopwatch);
  }

  // We don't care about directives or comments, skip them.
  @override
  void visitImportDirective(ImportDirective node) {}

  @override
  void visitExportDirective(ExportDirective node) {}

  @override
  void visitComment(Comment node) {}

  @override
  void visitConstructorName(ConstructorName node) {
    final Element? element = node.staticElement;
    if (element is! ConstructorElement) {
      assert(false, '$element of $node is not a ConstructorElement.');
      return;
    }
    final bool isAllowed = switch (element.returnType) {
      InterfaceType(element: final ClassElement classElement) =>
        !_implementsStopwatch(classElement),
      InterfaceType(element: InterfaceElement()) => true,
    };
    if (isAllowed || _hasTrailingFlutterIgnore(node)) {
      return;
    }
    stopwatchAccessNodes.add(node);
  }

  @override
  void visitSimpleIdentifier(SimpleIdentifier node) {
    final bool isAllowed = switch (node.staticElement) {
      ExecutableElement(
        returnType: DartType(element: final ClassElement classElement),
        library: final LibraryElement libraryElement,
      ) =>
        _isInternal(libraryElement) || !_implementsStopwatch(classElement),
      Element() || null => true,
    };
    if (isAllowed || _hasTrailingFlutterIgnore(node)) {
      return;
    }
    stopwatchAccessNodes.add(node);
  }
}
