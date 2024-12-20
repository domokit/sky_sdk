// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import 'constants.dart';
import 'object.dart';

/// A object representation of a frame from a stack trace.
///
/// {@tool snippet}
///
/// This example creates a traversable list of parsed [StackFrame] objects from
/// the current [StackTrace].
///
/// ```dart
/// final List<StackFrame> currentFrames = StackFrame.fromStackTrace(StackTrace.current);
/// ```
/// {@end-tool}
@immutable
class StackFrame {
  /// Creates a new StackFrame instance.
  ///
  /// The [className] may be the empty string if there is no class (e.g. for a
  /// top level library method).
  const StackFrame({
    required this.number,
    required this.column,
    required this.line,
    required this.packageScheme,
    required this.package,
    required this.packagePath,
    this.className = '',
    required this.method,
    this.isConstructor = false,
    required this.source,
  });

  /// A stack frame representing an asynchronous suspension.
  static const StackFrame asynchronousSuspension = StackFrame(
    number: -1,
    column: -1,
    line: -1,
    method: 'asynchronous suspension',
    packageScheme: '',
    package: '',
    packagePath: '',
    source: '<asynchronous suspension>',
  );

  /// A stack frame representing a Dart elided stack overflow frame.
  static const StackFrame stackOverFlowElision = StackFrame(
    number: -1,
    column: -1,
    line: -1,
    method: '...',
    packageScheme: '',
    package: '',
    packagePath: '',
    source: '...',
  );

  /// Parses a list of [StackFrame]s from a [StackTrace] object.
  ///
  /// This is normally useful with [StackTrace.current].
  static List<StackFrame> fromStackTrace(StackTrace stack) {
    return fromStackString(stack.toString());
  }

  /// Parses a list of [StackFrame]s from the [StackTrace.toString] method.
  static List<StackFrame> fromStackString(String stack) {
    return stack
        .trim()
        .split('\n')
        .where((String line) => line.isNotEmpty)
        .map(fromStackTraceLine)
        // On the Web in non-debug builds the stack trace includes the exception
        // message that precedes the stack trace itself. fromStackTraceLine will
        // return null in that case. We will skip it here.
        // TODO(polina-c): if one of lines was parsed to null, the entire stack trace
        // is in unexpected format and should be returned as is, without partial parsing.
        // https://github.com/flutter/flutter/issues/131877
        .whereType<StackFrame>()
        .toList();
  }

  /// Parses a single [StackFrame] from a line of a [StackTrace].
  ///
  /// Returns null if format is not as expected.
  static StackFrame? _tryParseWebFrame(String line) {
    // dart2wasm doesn't emit stack frames in the same way DDC does, so we need
    // to do the less clever non-debug path here when compiled to wasm.
    if (kDebugMode && !kIsWasm) {
      return _tryParseWebDebugFrame(line);
    } else {
      return _tryParseWebNonDebugFrame(line);
    }
  }

  /// Parses a single [StackFrame] from a line of a [StackTrace].
  ///
  /// Returns null if format is not as expected.
  static StackFrame? _tryParseWebDebugFrame(String line) {
    // This RegExp is only partially correct for flutter run/test differences.
    // https://github.com/flutter/flutter/issues/52685
    final bool hasPackage = line.startsWith('package');
    final RegExp parser =
        hasPackage
            ? RegExp(r'^(package.+) (\d+):(\d+)\s+(.+)$')
            : RegExp(r'^(.+) (\d+):(\d+)\s+(.+)$');

    final Match? match = parser.firstMatch(line);

    if (match == null) {
      return null;
    }

    String package = '<unknown>';
    String packageScheme = '<unknown>';
    String packagePath = '<unknown>';

    if (hasPackage) {
      packageScheme = 'package';
      final Uri packageUri = Uri.parse(match.group(1)!);
      package = packageUri.pathSegments[0];
      packagePath = packageUri.path.replaceFirst('${packageUri.pathSegments[0]}/', '');
    }

    return StackFrame(
      number: -1,
      packageScheme: packageScheme,
      package: package,
      packagePath: packagePath,
      line: int.parse(match.group(2)!),
      column: int.parse(match.group(3)!),
      className: '<unknown>',
      method: match.group(4)!,
      source: line,
    );
  }

  // Non-debug builds do not point to dart code but compiled JavaScript, so
  // line numbers are meaningless. We only attempt to parse the class and
  // method name, which is more or less readable in profile builds, and
  // minified in release builds.
  static final RegExp _webNonDebugFramePattern = RegExp(r'^\s*at ([^\s]+).*$');

  // Parses `line` as a stack frame in profile and release Web builds. If not
  // recognized as a stack frame, returns null.
  static StackFrame? _tryParseWebNonDebugFrame(String line) {
    final Match? match = _webNonDebugFramePattern.firstMatch(line);
    if (match == null) {
      // On the Web in non-debug builds the stack trace includes the exception
      // message that precedes the stack trace itself. Example:
      //
      // TypeError: Cannot read property 'hello$0' of null
      //    at _GalleryAppState.build$1 (http://localhost:8080/main.dart.js:149790:13)
      //    at StatefulElement.build$0 (http://localhost:8080/main.dart.js:129138:37)
      //    at StatefulElement.performRebuild$0 (http://localhost:8080/main.dart.js:129032:23)
      //
      // Instead of crashing when a line is not recognized as a stack frame, we
      // return null. The caller, such as fromStackString, can then just skip
      // this frame.
      return null;
    }

    final List<String> classAndMethod = match.group(1)!.split('.');
    final String className = classAndMethod.length > 1 ? classAndMethod.first : '<unknown>';
    final String method =
        classAndMethod.length > 1 ? classAndMethod.skip(1).join('.') : classAndMethod.single;

    return StackFrame(
      number: -1,
      packageScheme: '<unknown>',
      package: '<unknown>',
      packagePath: '<unknown>',
      line: -1,
      column: -1,
      className: className,
      method: method,
      source: line,
    );
  }

  /// Parses a single [StackFrame] from a single line of a [StackTrace].
  ///
  /// Returns null if format is not as expected.
  static StackFrame? fromStackTraceLine(String line) {
    if (line == '<asynchronous suspension>') {
      return asynchronousSuspension;
    } else if (line == '...') {
      return stackOverFlowElision;
    }

    assert(
      line != '===== asynchronous gap ===========================',
      'Got a stack frame from package:stack_trace, where a vm or web frame was expected. '
      'This can happen if FlutterError.demangleStackTrace was not set in an environment '
      'that propagates non-standard stack traces to the framework, such as during tests.',
    );

    // Web frames.
    if (!line.startsWith('#')) {
      return _tryParseWebFrame(line);
    }

    final RegExp parser = RegExp(r'^#(\d+) +(.+) \((.+?):?(\d+){0,1}:?(\d+){0,1}\)$');
    Match? match = parser.firstMatch(line);
    assert(match != null, 'Expected $line to match $parser.');
    match = match!;

    bool isConstructor = false;
    String className = '';
    String method = match.group(2)!.replaceAll('.<anonymous closure>', '');
    if (method.startsWith('new')) {
      final List<String> methodParts = method.split(' ');
      // Sometimes a web frame will only read "new" and have no class name.
      className = methodParts.length > 1 ? method.split(' ')[1] : '<unknown>';
      method = '';
      if (className.contains('.')) {
        final List<String> parts = className.split('.');
        className = parts[0];
        method = parts[1];
      }
      isConstructor = true;
    } else if (method.contains('.')) {
      final List<String> parts = method.split('.');
      className = parts[0];
      method = parts[1];
    }

    final Uri packageUri = Uri.parse(match.group(3)!);
    String package = '<unknown>';
    String packagePath = packageUri.path;
    if (packageUri.scheme == 'dart' || packageUri.scheme == 'package') {
      package = packageUri.pathSegments[0];
      packagePath = packageUri.path.replaceFirst('${packageUri.pathSegments[0]}/', '');
    }

    return StackFrame(
      number: int.parse(match.group(1)!),
      className: className,
      method: method,
      packageScheme: packageUri.scheme,
      package: package,
      packagePath: packagePath,
      line: match.group(4) == null ? -1 : int.parse(match.group(4)!),
      column: match.group(5) == null ? -1 : int.parse(match.group(5)!),
      isConstructor: isConstructor,
      source: line,
    );
  }

  /// The original source of this stack frame.
  final String source;

  /// The zero-indexed frame number.
  ///
  /// This value may be -1 to indicate an unknown frame number.
  final int number;

  /// The scheme of the package for this frame, e.g. "dart" for
  /// dart:core/errors_patch.dart or "package" for
  /// package:flutter/src/widgets/text.dart.
  ///
  /// The path property refers to the source file.
  final String packageScheme;

  /// The package for this frame, e.g. "core" for
  /// dart:core/errors_patch.dart or "flutter" for
  /// package:flutter/src/widgets/text.dart.
  final String package;

  /// The path of the file for this frame, e.g. "errors_patch.dart" for
  /// dart:core/errors_patch.dart or "src/widgets/text.dart" for
  /// package:flutter/src/widgets/text.dart.
  final String packagePath;

  /// The source line number.
  final int line;

  /// The source column number.
  final int column;

  /// The class name, if any, for this frame.
  ///
  /// This may be null for top level methods in a library or anonymous closure
  /// methods.
  final String className;

  /// The method name for this frame.
  ///
  /// This will be an empty string if the stack frame is from the default
  /// constructor.
  final String method;

  /// Whether or not this was thrown from a constructor.
  final bool isConstructor;

  @override
  int get hashCode => Object.hash(number, package, line, column, className, method, source);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is StackFrame &&
        other.number == number &&
        other.package == package &&
        other.line == line &&
        other.column == column &&
        other.className == className &&
        other.method == method &&
        other.source == source;
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'StackFrame')}(#$number, $packageScheme:$package/$packagePath:$line:$column, className: $className, method: $method)';
}
