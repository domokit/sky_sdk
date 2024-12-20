// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:analyzer/dart/analysis/features.dart';
import 'package:analyzer/dart/analysis/results.dart';
import 'package:analyzer/dart/analysis/utilities.dart';
import 'package:analyzer/dart/ast/ast.dart';
import 'package:analyzer/dart/ast/token.dart';
import 'package:analyzer/error/error.dart';
import 'package:analyzer/source/line_info.dart';
import 'package:meta/meta.dart';

import 'util.dart';

/// Read the given source code, and return the new contents after sorting the
/// imports.
String sortImports(String contents) {
  final ParseStringResult parseResult = parseString(
    content: contents,
    featureSet: FeatureSet.fromEnableFlags2(
      sdkLanguageVersion: FlutterInformation.instance.getDartSdkVersion(),
      flags: <String>[],
    ),
  );
  final List<AnalysisError> errors = <AnalysisError>[];
  final _ImportOrganizer organizer = _ImportOrganizer(contents, parseResult.unit, errors);
  final List<_SourceEdit> edits = organizer.organize();
  // Sort edits in reverse order
  edits.sort((_SourceEdit a, _SourceEdit b) {
    return b.offset.compareTo(a.offset);
  });
  // Apply edits
  for (final _SourceEdit edit in edits) {
    contents = contents.replaceRange(edit.offset, edit.end, edit.replacement);
  }
  return contents;
}

/// Organizer of imports (and other directives) in the [unit].
// Adapted from the analysis_server package.
// This code is largely copied from:
// https://github.com/dart-lang/sdk/blob/c7405b9d86b4b47cf7610667491f1db72723b0dd/pkg/analysis_server/lib/src/services/correction/organize_imports.dart#L15
// TODO(gspencergoog): If ImportOrganizer ever becomes part of the public API,
// this class should probably be replaced.
// https://github.com/flutter/flutter/issues/86197
class _ImportOrganizer {
  _ImportOrganizer(this.initialCode, this.unit, this.errors) : code = initialCode {
    endOfLine = getEOL(code);
    hasUnresolvedIdentifierError = errors.any((AnalysisError error) {
      return error.errorCode.isUnresolvedIdentifier;
    });
  }

  final String initialCode;

  final CompilationUnit unit;

  final List<AnalysisError> errors;

  String code;

  String endOfLine = '\n';

  bool hasUnresolvedIdentifierError = false;

  /// Returns the number of characters common to the end of [a] and [b].
  int findCommonSuffix(String a, String b) {
    final int aLength = a.length;
    final int bLength = b.length;
    final int n = min(aLength, bLength);
    for (int i = 1; i <= n; i++) {
      if (a.codeUnitAt(aLength - i) != b.codeUnitAt(bLength - i)) {
        return i - 1;
      }
    }
    return n;
  }

  /// Return the [_SourceEdit]s that organize imports in the [unit].
  List<_SourceEdit> organize() {
    _organizeDirectives();
    // prepare edits
    final List<_SourceEdit> edits = <_SourceEdit>[];
    if (code != initialCode) {
      final int suffixLength = findCommonSuffix(initialCode, code);
      final _SourceEdit edit = _SourceEdit(
        0,
        initialCode.length - suffixLength,
        code.substring(0, code.length - suffixLength),
      );
      edits.add(edit);
    }
    return edits;
  }

  /// Organize all [Directive]s.
  void _organizeDirectives() {
    final LineInfo lineInfo = unit.lineInfo;
    bool hasLibraryDirective = false;
    final List<_DirectiveInfo> directives = <_DirectiveInfo>[];
    for (final Directive directive in unit.directives) {
      if (directive is LibraryDirective) {
        hasLibraryDirective = true;
      }
      if (directive is UriBasedDirective) {
        final _DirectivePriority? priority = getDirectivePriority(directive);
        if (priority != null) {
          int offset = directive.offset;
          int end = directive.end;

          final Token? leadingComment = getLeadingComment(unit, directive, lineInfo);
          final Token? trailingComment = getTrailingComment(unit, directive, lineInfo, end);

          String? leadingCommentText;
          if (leadingComment != null) {
            leadingCommentText = code.substring(leadingComment.offset, directive.offset);
            offset = leadingComment.offset;
          }
          String? trailingCommentText;
          if (trailingComment != null) {
            trailingCommentText = code.substring(directive.end, trailingComment.end);
            end = trailingComment.end;
          }
          String? documentationText;
          final Comment? documentationComment = directive.documentationComment;
          if (documentationComment != null) {
            documentationText = code.substring(
              documentationComment.offset,
              documentationComment.end,
            );
          }
          String? annotationText;
          final Token? beginToken = directive.metadata.beginToken;
          final Token? endToken = directive.metadata.endToken;
          if (beginToken != null && endToken != null) {
            annotationText = code.substring(beginToken.offset, endToken.end);
          }
          final String text = code.substring(
            directive.firstTokenAfterCommentAndMetadata.offset,
            directive.end,
          );
          final String uriContent = directive.uri.stringValue ?? '';
          directives.add(
            _DirectiveInfo(
              directive,
              priority,
              leadingCommentText,
              documentationText,
              annotationText,
              uriContent,
              trailingCommentText,
              offset,
              end,
              text,
            ),
          );
        }
      }
    }
    // nothing to do
    if (directives.isEmpty) {
      return;
    }
    final int firstDirectiveOffset = directives.first.offset;
    final int lastDirectiveEnd = directives.last.end;

    // Without a library directive, the library comment is the comment of the
    // first directive.
    _DirectiveInfo? libraryDocumentationDirective;
    if (!hasLibraryDirective && directives.isNotEmpty) {
      libraryDocumentationDirective = directives.first;
    }

    // sort
    directives.sort();
    // append directives with grouping
    String directivesCode;
    {
      final StringBuffer sb = StringBuffer();
      if (libraryDocumentationDirective != null &&
          libraryDocumentationDirective.documentationText != null) {
        sb.write(libraryDocumentationDirective.documentationText);
        sb.write(endOfLine);
      }
      _DirectivePriority currentPriority = directives.first.priority;
      for (final _DirectiveInfo directiveInfo in directives) {
        if (currentPriority != directiveInfo.priority) {
          sb.write(endOfLine);
          currentPriority = directiveInfo.priority;
        }
        if (directiveInfo.leadingCommentText != null) {
          sb.write(directiveInfo.leadingCommentText);
        }
        if (directiveInfo != libraryDocumentationDirective &&
            directiveInfo.documentationText != null) {
          sb.write(directiveInfo.documentationText);
          sb.write(endOfLine);
        }
        if (directiveInfo.annotationText != null) {
          sb.write(directiveInfo.annotationText);
          sb.write(endOfLine);
        }
        sb.write(directiveInfo.text);
        if (directiveInfo.trailingCommentText != null) {
          sb.write(directiveInfo.trailingCommentText);
        }
        sb.write(endOfLine);
      }
      directivesCode = sb.toString();
      directivesCode = directivesCode.trimRight();
    }
    // prepare code
    final String beforeDirectives = code.substring(0, firstDirectiveOffset);
    final String afterDirectives = code.substring(lastDirectiveEnd);
    code = beforeDirectives + directivesCode + afterDirectives;
  }

  static _DirectivePriority? getDirectivePriority(UriBasedDirective directive) {
    final String uriContent = directive.uri.stringValue ?? '';
    if (directive is ImportDirective) {
      if (uriContent.startsWith('dart:')) {
        return _DirectivePriority.IMPORT_SDK;
      } else if (uriContent.startsWith('package:')) {
        return _DirectivePriority.IMPORT_PKG;
      } else if (uriContent.contains('://')) {
        return _DirectivePriority.IMPORT_OTHER;
      } else {
        return _DirectivePriority.IMPORT_REL;
      }
    }
    if (directive is ExportDirective) {
      if (uriContent.startsWith('dart:')) {
        return _DirectivePriority.EXPORT_SDK;
      } else if (uriContent.startsWith('package:')) {
        return _DirectivePriority.EXPORT_PKG;
      } else if (uriContent.contains('://')) {
        return _DirectivePriority.EXPORT_OTHER;
      } else {
        return _DirectivePriority.EXPORT_REL;
      }
    }
    if (directive is PartDirective) {
      return _DirectivePriority.PART;
    }
    return null;
  }

  /// Return the EOL to use for [code].
  static String getEOL(String code) {
    if (code.contains('\r\n')) {
      return '\r\n';
    } else {
      return '\n';
    }
  }

  /// Gets the first comment token considered to be the leading comment for this
  /// directive.
  ///
  /// Leading comments for the first directive in a file are considered library
  /// comments and not returned unless they contain blank lines, in which case
  /// only the last part of the comment will be returned.
  static Token? getLeadingComment(
    CompilationUnit unit,
    UriBasedDirective directive,
    LineInfo lineInfo,
  ) {
    if (directive.beginToken.precedingComments == null) {
      return null;
    }

    Token? firstComment = directive.beginToken.precedingComments;
    Token? comment = firstComment;
    Token? nextComment = comment?.next;
    // Don't connect comments that have a blank line between them
    while (comment != null && nextComment != null) {
      final int currentLine = lineInfo.getLocation(comment.offset).lineNumber;
      final int nextLine = lineInfo.getLocation(nextComment.offset).lineNumber;
      if (nextLine - currentLine > 1) {
        firstComment = nextComment;
      }
      comment = nextComment;
      nextComment = comment.next;
    }

    // Check if the comment is the first comment in the document
    if (firstComment != unit.beginToken.precedingComments) {
      final int previousDirectiveLine =
          lineInfo.getLocation(directive.beginToken.previous!.end).lineNumber;

      // Skip over any comments on the same line as the previous directive
      // as they will be attached to the end of it.
      Token? comment = firstComment;
      while (comment != null &&
          previousDirectiveLine == lineInfo.getLocation(comment.offset).lineNumber) {
        comment = comment.next;
      }
      return comment;
    }
    return null;
  }

  /// Gets the last comment token considered to be the trailing comment for this
  /// directive.
  ///
  /// To be considered a trailing comment, the comment must be on the same line
  /// as the directive.
  static Token? getTrailingComment(
    CompilationUnit unit,
    UriBasedDirective directive,
    LineInfo lineInfo,
    int end,
  ) {
    final int line = lineInfo.getLocation(end).lineNumber;
    Token? comment = directive.endToken.next!.precedingComments;
    while (comment != null) {
      if (lineInfo.getLocation(comment.offset).lineNumber == line) {
        return comment;
      }
      comment = comment.next;
    }
    return null;
  }
}

class _DirectiveInfo implements Comparable<_DirectiveInfo> {
  _DirectiveInfo(
    this.directive,
    this.priority,
    this.leadingCommentText,
    this.documentationText,
    this.annotationText,
    this.uri,
    this.trailingCommentText,
    this.offset,
    this.end,
    this.text,
  );

  final UriBasedDirective directive;
  final _DirectivePriority priority;
  final String? leadingCommentText;
  final String? documentationText;
  final String? annotationText;
  final String uri;
  final String? trailingCommentText;

  /// The offset of the first token, usually the keyword but may include leading comments.
  final int offset;

  /// The offset after the last token, including the end-of-line comment.
  final int end;

  /// The text excluding comments, documentation and annotations.
  final String text;

  @override
  int compareTo(_DirectiveInfo other) {
    if (priority == other.priority) {
      return _compareUri(uri, other.uri);
    }
    return priority.index - other.priority.index;
  }

  @override
  String toString() => '(priority=$priority; text=$text)';

  static int _compareUri(String a, String b) {
    final List<String> aList = _splitUri(a);
    final List<String> bList = _splitUri(b);
    int result;
    if ((result = aList[0].compareTo(bList[0])) != 0) {
      return result;
    }
    if ((result = aList[1].compareTo(bList[1])) != 0) {
      return result;
    }
    return 0;
  }

  /// Split the given [uri] like `package:some.name/and/path.dart` into a list
  /// like `[package:some.name, and/path.dart]`.
  static List<String> _splitUri(String uri) {
    final int index = uri.indexOf('/');
    if (index == -1) {
      return <String>[uri, ''];
    }
    return <String>[uri.substring(0, index), uri.substring(index + 1)];
  }
}

enum _DirectivePriority {
  IMPORT_SDK,
  IMPORT_PKG,
  IMPORT_OTHER,
  IMPORT_REL,
  EXPORT_SDK,
  EXPORT_PKG,
  EXPORT_OTHER,
  EXPORT_REL,
  PART,
}

/// SourceEdit
///
/// {
///   "offset": int
///   "length": int
///   "replacement": String
///   "id": optional String
/// }
///
/// Clients may not extend, implement or mix-in this class.
@immutable
class _SourceEdit {
  const _SourceEdit(this.offset, this.length, this.replacement);

  /// The offset of the region to be modified.
  final int offset;

  /// The length of the region to be modified.
  final int length;

  /// The end of the region to be modified.
  int get end => offset + length;

  /// The code that is to replace the specified region in the original code.
  final String replacement;
}
