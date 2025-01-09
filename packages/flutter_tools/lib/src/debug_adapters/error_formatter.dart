// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

typedef _OutputSender =
    void Function(
      String category,
      String message, {
      bool? parseStackFrames,
      int? variablesReference,
    });

/// A formatter for improving the display of Flutter structured errors over DAP.
///
/// The formatter deserializes a `Flutter.Error` event and produces output
/// similar to the `renderedErrorText` field, but may include ansi color codes
/// to provide improved formatting (such as making stack frames from non-user
/// code faint) if the client indicated support.
///
/// Lines that look like stack frames will be marked so they can be parsed by
/// the base adapter and attached as [Source]s to allow them to be clickable
/// in the client.
class FlutterErrorFormatter {
  final List<_BatchedOutput> batchedOutput = <_BatchedOutput>[];

  /// Formats a Flutter error.
  ///
  /// If this is not the first error since the reload, only a summary will be
  /// included.
  void formatError(Map<String, Object?> errorData) {
    final _ErrorData data = _ErrorData(errorData);

    const int assumedTerminalSize = 80;
    const String barChar = '═';
    final String headerPrefix = barChar * 8;
    final String headerSuffix =
        barChar *
        math.max(
          assumedTerminalSize - (data.description?.length ?? 0) - 2 - headerPrefix.length,
          0,
        );
    final String header = '$headerPrefix ${data.description} $headerSuffix';
    _write('');
    _write(header, isError: true);

    if (data.errorsSinceReload == 0) {
      data.properties.forEach(_writeNode);
      data.children.forEach(_writeNode);
    } else {
      data.properties.forEach(_writeSummary);
    }

    _write(barChar * header.length, isError: true);
  }

  /// Sends all collected output through [sendOutput].
  void sendOutput(_OutputSender sendOutput) {
    for (final _BatchedOutput output in batchedOutput) {
      sendOutput(
        output.isError ? 'stderr' : 'stdout',
        output.output,
        parseStackFrames: output.parseStackFrames,
      );
    }
  }

  /// Writes [text] to the output.
  ///
  /// If the last item in the batch has the same settings as this item, it will
  /// be appended to the same item, otherwise a new item will be added to the
  /// batch.
  void _write(String? text, {int indent = 0, bool isError = false, bool parseStackFrames = false}) {
    if (text != null) {
      final String indentString = '    ' * indent;
      final String message = '$indentString${text.trim()}';

      _BatchedOutput? output = batchedOutput.lastOrNull;
      if (output == null ||
          output.isError != isError ||
          output.parseStackFrames != parseStackFrames) {
        batchedOutput.add(output = _BatchedOutput(isError, parseStackFrames: parseStackFrames));
      }
      output.writeln(message);
    }
  }

  /// Writes [node] to the output using [indent], recursing unless [recursive]
  /// is `false`.
  void _writeNode(_ErrorNode node, {int indent = 0, bool recursive = true}) {
    // Errors, summaries and lines starting "Exception:" are marked as errors so
    // they go to stderr instead of stdout (this may cause the client to colour
    // them like errors).
    final bool showAsError =
        node.level == _DiagnosticsNodeLevel.error ||
        node.level == _DiagnosticsNodeLevel.summary ||
        (node.description?.startsWith('Exception: ') ?? false);

    if (node.showName && node.name != null) {
      _write('${node.name}: ${node.description}', indent: indent, isError: showAsError);
    } else if (node.description?.startsWith('#') ?? false) {
      // Possible stack frame.
      _write(node.description, indent: indent, isError: showAsError, parseStackFrames: true);
    } else {
      _write(node.description, indent: indent, isError: showAsError);
    }

    if (recursive) {
      if (node.style != _DiagnosticsNodeStyle.flat) {
        indent++;
      }
      _writeNodes(node.properties, indent: indent);
      _writeNodes(node.children, indent: indent);
    }
  }

  /// Writes [nodes] to the output.
  void _writeNodes(List<_ErrorNode> nodes, {int indent = 0, bool recursive = true}) {
    for (final _ErrorNode child in nodes) {
      _writeNode(child, indent: indent, recursive: recursive);
    }
  }

  /// Writes a simple summary of [node] to the output.
  void _writeSummary(_ErrorNode node) {
    final bool allChildrenAreLeaf =
        node.children.isNotEmpty &&
        !node.children.any((_ErrorNode child) => child.children.isNotEmpty);
    if (node.level == _DiagnosticsNodeLevel.summary || allChildrenAreLeaf) {
      // DiagnosticsBlock is a container, so recurse into its children if
      // there's only a single level. The container may be
      // "The relevant error-causing widget was" and the child may be
      // the specific widget details.
      _writeNode(
        node,
        recursive: node.type == _DiagnosticsNodeType.DiagnosticsBlock && allChildrenAreLeaf,
      );
    }
  }
}

/// A container for output to be sent to the client.
///
/// When multiple lines are being sent, they may be written to the same batch
/// if the output options (error/stackFrame) are the same.
class _BatchedOutput {
  _BatchedOutput(this.isError, {this.parseStackFrames = false});

  final bool isError;
  final bool parseStackFrames;
  final StringBuffer _buffer = StringBuffer();

  String get output => _buffer.toString();

  void writeln(String output) => _buffer.writeln(output);
}

enum _DiagnosticsNodeLevel { error, summary }

enum _DiagnosticsNodeStyle { flat }

enum _DiagnosticsNodeType { DiagnosticsBlock }

class _ErrorData extends _ErrorNode {
  _ErrorData(super.data);

  int get errorsSinceReload => data['errorsSinceReload'] as int? ?? 0;
  String get renderedErrorText => data['renderedErrorText'] as String? ?? '';
}

class _ErrorNode {
  _ErrorNode(this.data);

  final Map<Object, Object?> data;

  List<_ErrorNode> get children => asList('children', _ErrorNode.new);
  String? get description => asString('description');
  _DiagnosticsNodeLevel? get level => asEnum('level', _DiagnosticsNodeLevel.values);
  String? get name => asString('name');
  List<_ErrorNode> get properties => asList('properties', _ErrorNode.new);
  bool get showName => data['showName'] != false;
  _DiagnosticsNodeStyle? get style => asEnum('style', _DiagnosticsNodeStyle.values);
  _DiagnosticsNodeType? get type => asEnum('type', _DiagnosticsNodeType.values);

  String? asString(String field) {
    final Object? value = data[field];
    return value is String ? value : null;
  }

  T? asEnum<T extends Enum>(String field, Iterable<T> enumValues) {
    final String? value = asString(field);
    return value != null ? enumValues.asNameMap()[value] : null;
  }

  List<T> asList<T>(String field, T Function(Map<Object, Object?>) constructor) {
    final Object? objects = data[field];
    return objects is List && objects.every((Object? element) => element is Map<String, Object?>)
        ? objects.cast<Map<Object, Object?>>().map(constructor).toList()
        : <T>[];
  }
}
