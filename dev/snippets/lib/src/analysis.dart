// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:analyzer/dart/analysis/features.dart';
import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/analysis/utilities.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/dart/ast/token.dart';
import 'package:analyzer/dart/ast/visitor.dart';
import 'package:analyzer/file_system/file_system.dart' as afs;
import 'package:analyzer/file_system/physical_file_system.dart' as afs;
import 'package:analyzer/source/line_info.dart';
import 'package:file/file.dart';

import 'data_types.dart';
import 'util.dart';

/// Gets an iterable over all of the blocks of documentation comments in a file
/// using the analyzer.
///
/// Each entry in the list is a list of source lines corresponding to the
/// documentation comment block.
Iterable<List<SourceLine>> getFileDocumentationComments(File file) {
  return getDocumentationComments(getFileElements(file));
}

/// Gets an iterable over all of the blocks of documentation comments from an
/// iterable over the [SourceElement]s involved.
Iterable<List<SourceLine>> getDocumentationComments(Iterable<SourceElement> elements) {
  return elements
      .where((SourceElement element) => element.comment.isNotEmpty)
      .map<List<SourceLine>>((SourceElement element) => element.comment);
}

/// Gets an iterable over the comment [SourceElement]s in a file.
Iterable<SourceElement> getFileCommentElements(File file) {
  return getCommentElements(getFileElements(file));
}

/// Filters the source `elements` to only return the comment elements.
Iterable<SourceElement> getCommentElements(Iterable<SourceElement> elements) {
  return elements.where((SourceElement element) => element.comment.isNotEmpty);
}

/// Reads the file content from a string, to avoid having to read the file more
/// than once if the caller already has the content in memory.
///
/// The `file` argument is used to tag the lines with a filename that they came from.
Iterable<SourceElement> getElementsFromString(String content, File file) {
  final ParseStringResult parseResult = parseString(
    featureSet: FeatureSet.fromEnableFlags2(
      sdkLanguageVersion: FlutterInformation.instance.getDartSdkVersion(),
      flags: <String>[],
    ),
    content: content,
  );
  final _SourceVisitor<CompilationUnit> visitor = _SourceVisitor<CompilationUnit>(file);
  visitor.visitCompilationUnit(parseResult.unit);
  visitor.assignLineNumbers();
  return visitor.elements;
}

/// Gets an iterable over the [SourceElement]s in the given `file`.
///
/// Takes an optional [ResourceProvider] to allow reading from a memory
/// filesystem.
Iterable<SourceElement> getFileElements(File file, {afs.ResourceProvider? resourceProvider}) {
  resourceProvider ??= afs.PhysicalResourceProvider.INSTANCE;
  final ParseStringResult parseResult = parseFile(
    featureSet: FeatureSet.fromEnableFlags2(
      sdkLanguageVersion: FlutterInformation.instance.getDartSdkVersion(),
      flags: <String>[],
    ),
    path: file.absolute.path,
    resourceProvider: resourceProvider,
  );
  final _SourceVisitor<CompilationUnit> visitor = _SourceVisitor<CompilationUnit>(file);
  visitor.visitCompilationUnit(parseResult.unit);
  visitor.assignLineNumbers();
  return visitor.elements;
}

class _SourceVisitor<T> extends RecursiveAstVisitor<T> {
  _SourceVisitor(this.file) : elements = <SourceElement>{};

  final Set<SourceElement> elements;
  String enclosingClass = '';

  File file;

  void assignLineNumbers() {
    final String contents = file.readAsStringSync();
    final LineInfo lineInfo = LineInfo.fromContent(contents);

    final Set<SourceElement> removedElements = <SourceElement>{};
    final Set<SourceElement> replacedElements = <SourceElement>{};
    for (final SourceElement element in elements) {
      final List<SourceLine> newLines = <SourceLine>[];
      for (final SourceLine line in element.comment) {
        final CharacterLocation intervalLine = lineInfo.getLocation(line.startChar);
        newLines.add(line.copyWith(line: intervalLine.lineNumber));
      }
      final int elementLine = lineInfo.getLocation(element.startPos).lineNumber;
      replacedElements.add(element.copyWith(comment: newLines, startLine: elementLine));
      removedElements.add(element);
    }
    elements.removeAll(removedElements);
    elements.addAll(replacedElements);
  }

  List<SourceLine> _processComment(String element, Comment comment) {
    final List<SourceLine> result = <SourceLine>[];
    if (comment.tokens.isNotEmpty) {
      for (final Token token in comment.tokens) {
        result.add(
          SourceLine(
            token.toString(),
            element: element,
            file: file,
            startChar: token.charOffset,
            endChar: token.charEnd,
          ),
        );
      }
    }
    return result;
  }

  @override
  T? visitCompilationUnit(CompilationUnit node) {
    elements.clear();
    return super.visitCompilationUnit(node);
  }

  static bool isPublic(String name) {
    return !name.startsWith('_');
  }

  static bool isInsideMethod(AstNode startNode) {
    AstNode? node = startNode.parent;
    while (node != null) {
      if (node is MethodDeclaration) {
        return true;
      }
      node = node.parent;
    }
    return false;
  }

  @override
  T? visitTopLevelVariableDeclaration(TopLevelVariableDeclaration node) {
    for (final VariableDeclaration declaration in node.variables.variables) {
      if (!isPublic(declaration.name.lexeme)) {
        continue;
      }
      List<SourceLine> comment = <SourceLine>[];
      if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
        comment = _processComment(declaration.name.lexeme, node.documentationComment!);
      }
      elements.add(
        SourceElement(
          SourceElementType.topLevelVariableType,
          declaration.name.lexeme,
          node.beginToken.charOffset,
          file: file,
          className: enclosingClass,
          comment: comment,
        ),
      );
    }
    return super.visitTopLevelVariableDeclaration(node);
  }

  @override
  T? visitGenericTypeAlias(GenericTypeAlias node) {
    if (isPublic(node.name.lexeme)) {
      List<SourceLine> comment = <SourceLine>[];
      if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
        comment = _processComment(node.name.lexeme, node.documentationComment!);
      }
      elements.add(
        SourceElement(
          SourceElementType.typedefType,
          node.name.lexeme,
          node.beginToken.charOffset,
          file: file,
          comment: comment,
        ),
      );
    }
    return super.visitGenericTypeAlias(node);
  }

  @override
  T? visitFieldDeclaration(FieldDeclaration node) {
    for (final VariableDeclaration declaration in node.fields.variables) {
      if (!isPublic(declaration.name.lexeme) || !isPublic(enclosingClass)) {
        continue;
      }
      List<SourceLine> comment = <SourceLine>[];
      if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
        assert(enclosingClass.isNotEmpty);
        comment = _processComment(
          '$enclosingClass.${declaration.name.lexeme}',
          node.documentationComment!,
        );
      }
      elements.add(
        SourceElement(
          SourceElementType.fieldType,
          declaration.name.lexeme,
          node.beginToken.charOffset,
          file: file,
          className: enclosingClass,
          comment: comment,
          override: _isOverridden(node),
        ),
      );
      return super.visitFieldDeclaration(node);
    }
    return null;
  }

  @override
  T? visitConstructorDeclaration(ConstructorDeclaration node) {
    final String fullName = '$enclosingClass${node.name == null ? '' : '.${node.name}'}';
    if (isPublic(enclosingClass) && (node.name == null || isPublic(node.name!.lexeme))) {
      List<SourceLine> comment = <SourceLine>[];
      if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
        comment = _processComment('$enclosingClass.$fullName', node.documentationComment!);
      }
      elements.add(
        SourceElement(
          SourceElementType.constructorType,
          fullName,
          node.beginToken.charOffset,
          file: file,
          className: enclosingClass,
          comment: comment,
        ),
      );
    }
    return super.visitConstructorDeclaration(node);
  }

  @override
  T? visitFunctionDeclaration(FunctionDeclaration node) {
    if (isPublic(node.name.lexeme)) {
      List<SourceLine> comment = <SourceLine>[];
      // Skip functions that are defined inside of methods.
      if (!isInsideMethod(node)) {
        if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
          comment = _processComment(node.name.lexeme, node.documentationComment!);
        }
        elements.add(
          SourceElement(
            SourceElementType.functionType,
            node.name.lexeme,
            node.beginToken.charOffset,
            file: file,
            comment: comment,
            override: _isOverridden(node),
          ),
        );
      }
    }
    return super.visitFunctionDeclaration(node);
  }

  @override
  T? visitMethodDeclaration(MethodDeclaration node) {
    if (isPublic(node.name.lexeme) && isPublic(enclosingClass)) {
      List<SourceLine> comment = <SourceLine>[];
      if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
        assert(enclosingClass.isNotEmpty);
        comment = _processComment(
          '$enclosingClass.${node.name.lexeme}',
          node.documentationComment!,
        );
      }
      elements.add(
        SourceElement(
          SourceElementType.methodType,
          node.name.lexeme,
          node.beginToken.charOffset,
          file: file,
          className: enclosingClass,
          comment: comment,
          override: _isOverridden(node),
        ),
      );
    }
    return super.visitMethodDeclaration(node);
  }

  bool _isOverridden(AnnotatedNode node) {
    return node.metadata.where((Annotation annotation) {
      return annotation.name.name == 'override';
    }).isNotEmpty;
  }

  @override
  T? visitMixinDeclaration(MixinDeclaration node) {
    enclosingClass = node.name.lexeme;
    if (!node.name.lexeme.startsWith('_')) {
      enclosingClass = node.name.lexeme;
      List<SourceLine> comment = <SourceLine>[];
      if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
        comment = _processComment(node.name.lexeme, node.documentationComment!);
      }
      elements.add(
        SourceElement(
          SourceElementType.classType,
          node.name.lexeme,
          node.beginToken.charOffset,
          file: file,
          comment: comment,
        ),
      );
    }
    final T? result = super.visitMixinDeclaration(node);
    enclosingClass = '';
    return result;
  }

  @override
  T? visitClassDeclaration(ClassDeclaration node) {
    enclosingClass = node.name.lexeme;
    if (!node.name.lexeme.startsWith('_')) {
      enclosingClass = node.name.lexeme;
      List<SourceLine> comment = <SourceLine>[];
      if (node.documentationComment != null && node.documentationComment!.tokens.isNotEmpty) {
        comment = _processComment(node.name.lexeme, node.documentationComment!);
      }
      elements.add(
        SourceElement(
          SourceElementType.classType,
          node.name.lexeme,
          node.beginToken.charOffset,
          file: file,
          comment: comment,
        ),
      );
    }
    final T? result = super.visitClassDeclaration(node);
    enclosingClass = '';
    return result;
  }
}
