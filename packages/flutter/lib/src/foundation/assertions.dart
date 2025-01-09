// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'package:meta/meta.dart';

import 'basic_types.dart';
import 'constants.dart';
import 'diagnostics.dart';
import 'print.dart';
import 'stack_frame.dart';

export 'basic_types.dart' show IterableFilter;
export 'diagnostics.dart'
    show DiagnosticLevel, DiagnosticPropertiesBuilder, DiagnosticsNode, DiagnosticsTreeStyle;
export 'stack_frame.dart' show StackFrame;

// Examples can assume:
// late String runtimeType;
// late bool draconisAlive;
// late bool draconisAmulet;
// late Diagnosticable draconis;
// void methodThatMayThrow() { }
// class Trace implements StackTrace { late StackTrace vmTrace; }
// class Chain implements StackTrace { Trace toTrace() => Trace(); }

/// Signature for [FlutterError.onError] handler.
typedef FlutterExceptionHandler = void Function(FlutterErrorDetails details);

/// Signature for [DiagnosticPropertiesBuilder] transformer.
typedef DiagnosticPropertiesTransformer =
    Iterable<DiagnosticsNode> Function(Iterable<DiagnosticsNode> properties);

/// Signature for [FlutterErrorDetails.informationCollector] callback
/// and other callbacks that collect information describing an error.
typedef InformationCollector = Iterable<DiagnosticsNode> Function();

/// Signature for a function that demangles [StackTrace] objects into a format
/// that can be parsed by [StackFrame].
///
/// See also:
///
///   * [FlutterError.demangleStackTrace], which shows an example implementation.
typedef StackTraceDemangler = StackTrace Function(StackTrace details);

/// Partial information from a stack frame for stack filtering purposes.
///
/// See also:
///
///  * [RepetitiveStackFrameFilter], which uses this class to compare against [StackFrame]s.
@immutable
class PartialStackFrame {
  /// Creates a new [PartialStackFrame] instance.
  const PartialStackFrame({required this.package, required this.className, required this.method});

  /// An `<asynchronous suspension>` line in a stack trace.
  static const PartialStackFrame asynchronousSuspension = PartialStackFrame(
    package: '',
    className: '',
    method: 'asynchronous suspension',
  );

  /// The package to match, e.g. `package:flutter/src/foundation/assertions.dart`,
  /// or `dart:ui/window.dart`.
  final Pattern package;

  /// The class name for the method.
  ///
  /// On web, this is ignored, since class names are not available.
  ///
  /// On all platforms, top level methods should use the empty string.
  final String className;

  /// The method name for this frame line.
  ///
  /// On web, private methods are wrapped with `[]`.
  final String method;

  /// Tests whether the [StackFrame] matches the information in this
  /// [PartialStackFrame].
  bool matches(StackFrame stackFrame) {
    final String stackFramePackage =
        '${stackFrame.packageScheme}:${stackFrame.package}/${stackFrame.packagePath}';
    // Ideally this wouldn't be necessary.
    // TODO(dnfield): https://github.com/dart-lang/sdk/issues/40117
    if (kIsWeb) {
      return package.allMatches(stackFramePackage).isNotEmpty &&
          stackFrame.method == (method.startsWith('_') ? '[$method]' : method);
    }
    return package.allMatches(stackFramePackage).isNotEmpty &&
        stackFrame.method == method &&
        stackFrame.className == className;
  }
}

/// A class that filters stack frames for additional filtering on
/// [FlutterError.defaultStackFilter].
abstract class StackFilter {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const StackFilter();

  /// Filters the list of [StackFrame]s by updating corresponding indices in
  /// `reasons`.
  ///
  /// To elide a frame or number of frames, set the string.
  void filter(List<StackFrame> stackFrames, List<String?> reasons);
}

/// A [StackFilter] that filters based on repeating lists of
/// [PartialStackFrame]s.
///
/// See also:
///
///   * [FlutterError.addDefaultStackFilter], a method to register additional
///     stack filters for [FlutterError.defaultStackFilter].
///   * [StackFrame], a class that can help with parsing stack frames.
///   * [PartialStackFrame], a class that helps match partial method information
///     to a stack frame.
class RepetitiveStackFrameFilter extends StackFilter {
  /// Creates a new RepetitiveStackFrameFilter. All parameters are required and must not be
  /// null.
  const RepetitiveStackFrameFilter({required this.frames, required this.replacement});

  /// The shape of this repetitive stack pattern.
  final List<PartialStackFrame> frames;

  /// The number of frames in this pattern.
  int get numFrames => frames.length;

  /// The string to replace the frames with.
  ///
  /// If the same replacement string is used multiple times in a row, the
  /// [FlutterError.defaultStackFilter] will insert a repeat count after this
  /// line rather than repeating it.
  final String replacement;

  List<String> get _replacements => List<String>.filled(numFrames, replacement);

  @override
  void filter(List<StackFrame> stackFrames, List<String?> reasons) {
    for (int index = 0; index < stackFrames.length - numFrames; index += 1) {
      if (_matchesFrames(stackFrames.skip(index).take(numFrames).toList())) {
        reasons.setRange(index, index + numFrames, _replacements);
        index += numFrames - 1;
      }
    }
  }

  bool _matchesFrames(List<StackFrame> stackFrames) {
    if (stackFrames.length < numFrames) {
      return false;
    }
    for (int index = 0; index < stackFrames.length; index++) {
      if (!frames[index].matches(stackFrames[index])) {
        return false;
      }
    }
    return true;
  }
}

abstract class _ErrorDiagnostic extends DiagnosticsProperty<List<Object>> {
  /// This constructor provides a reliable hook for a kernel transformer to find
  /// error messages that need to be rewritten to include object references for
  /// interactive display of errors.
  _ErrorDiagnostic(
    String message, {
    DiagnosticsTreeStyle style = DiagnosticsTreeStyle.flat,
    DiagnosticLevel level = DiagnosticLevel.info,
  }) : super(
         null,
         <Object>[message],
         showName: false,
         showSeparator: false,
         defaultValue: null,
         style: style,
         level: level,
       );

  /// In debug builds, a kernel transformer rewrites calls to the default
  /// constructors for [ErrorSummary], [ErrorDescription], and [ErrorHint] to use
  /// this constructor.
  //
  // ```dart
  // _ErrorDiagnostic('Element $element must be $color')
  // ```
  // Desugars to:
  // ```dart
  // _ErrorDiagnostic.fromParts(<Object>['Element ', element, ' must be ', color])
  // ```
  //
  // Slightly more complex case:
  // ```dart
  // _ErrorDiagnostic('Element ${element.runtimeType} must be $color')
  // ```
  // Desugars to:
  //```dart
  // _ErrorDiagnostic.fromParts(<Object>[
  //   'Element ',
  //   DiagnosticsProperty(null, element, description: element.runtimeType?.toString()),
  //   ' must be ',
  //   color,
  // ])
  // ```
  _ErrorDiagnostic._fromParts(
    List<Object> messageParts, {
    DiagnosticsTreeStyle style = DiagnosticsTreeStyle.flat,
    DiagnosticLevel level = DiagnosticLevel.info,
  }) : super(
         null,
         messageParts,
         showName: false,
         showSeparator: false,
         defaultValue: null,
         style: style,
         level: level,
       );

  @override
  String toString({
    TextTreeConfiguration? parentConfiguration,
    DiagnosticLevel minLevel = DiagnosticLevel.info,
  }) {
    return valueToString(parentConfiguration: parentConfiguration);
  }

  @override
  List<Object> get value => super.value!;

  @override
  String valueToString({TextTreeConfiguration? parentConfiguration}) {
    return value.join();
  }
}

/// An explanation of the problem and its cause, any information that may help
/// track down the problem, background information, etc.
///
/// Use [ErrorDescription] for any part of an error message where neither
/// [ErrorSummary] or [ErrorHint] is appropriate.
///
/// In debug builds, values interpolated into the `message` are
/// expanded and placed into [value], which is of type [List<Object>].
/// This allows IDEs to examine values interpolated into error messages.
///
/// See also:
///
///  * [ErrorSummary], which provides a short (one line) description of the
///    problem that was detected.
///  * [ErrorHint], which provides specific, non-obvious advice that may be
///    applicable.
///  * [ErrorSpacer], which renders as a blank line.
///  * [FlutterError], which is the most common place to use an
///    [ErrorDescription].
class ErrorDescription extends _ErrorDiagnostic {
  /// A lint enforces that this constructor can only be called with a string
  /// literal to match the limitations of the Dart Kernel transformer that
  /// optionally extracts out objects referenced using string interpolation in
  /// the message passed in.
  ///
  /// The message will display with the same text regardless of whether the
  /// kernel transformer is used. The kernel transformer is required so that
  /// debugging tools can provide interactive displays of objects described by
  /// the error.
  ErrorDescription(super.message) : super(level: DiagnosticLevel.info);

  /// Calls to the default constructor may be rewritten to use this constructor
  /// in debug mode using a kernel transformer.
  // ignore: unused_element
  ErrorDescription._fromParts(super.messageParts) : super._fromParts(level: DiagnosticLevel.info);
}

/// A short (one line) description of the problem that was detected.
///
/// Error summaries from the same source location should have little variance,
/// so that they can be recognized as related. For example, they shouldn't
/// include hash codes.
///
/// A [FlutterError] must start with an [ErrorSummary] and may not contain
/// multiple summaries.
///
/// In debug builds, values interpolated into the `message` are
/// expanded and placed into [value], which is of type [List<Object>].
/// This allows IDEs to examine values interpolated into error messages.
///
/// See also:
///
///  * [ErrorDescription], which provides an explanation of the problem and its
///    cause, any information that may help track down the problem, background
///    information, etc.
///  * [ErrorHint], which provides specific, non-obvious advice that may be
///    applicable.
///  * [FlutterError], which is the most common place to use an [ErrorSummary].
class ErrorSummary extends _ErrorDiagnostic {
  /// A lint enforces that this constructor can only be called with a string
  /// literal to match the limitations of the Dart Kernel transformer that
  /// optionally extracts out objects referenced using string interpolation in
  /// the message passed in.
  ///
  /// The message will display with the same text regardless of whether the
  /// kernel transformer is used. The kernel transformer is required so that
  /// debugging tools can provide interactive displays of objects described by
  /// the error.
  ErrorSummary(super.message) : super(level: DiagnosticLevel.summary);

  /// Calls to the default constructor may be rewritten to use this constructor
  /// in debug mode using a kernel transformer.
  // ignore: unused_element
  ErrorSummary._fromParts(super.messageParts) : super._fromParts(level: DiagnosticLevel.summary);
}

/// An [ErrorHint] provides specific, non-obvious advice that may be applicable.
///
/// If your message provides obvious advice that is always applicable, it is an
/// [ErrorDescription] not a hint.
///
/// In debug builds, values interpolated into the `message` are
/// expanded and placed into [value], which is of type [List<Object>].
/// This allows IDEs to examine values interpolated into error messages.
///
/// See also:
///
///  * [ErrorSummary], which provides a short (one line) description of the
///    problem that was detected.
///  * [ErrorDescription], which provides an explanation of the problem and its
///    cause, any information that may help track down the problem, background
///    information, etc.
///  * [ErrorSpacer], which renders as a blank line.
///  * [FlutterError], which is the most common place to use an [ErrorHint].
class ErrorHint extends _ErrorDiagnostic {
  /// A lint enforces that this constructor can only be called with a string
  /// literal to match the limitations of the Dart Kernel transformer that
  /// optionally extracts out objects referenced using string interpolation in
  /// the message passed in.
  ///
  /// The message will display with the same text regardless of whether the
  /// kernel transformer is used. The kernel transformer is required so that
  /// debugging tools can provide interactive displays of objects described by
  /// the error.
  ErrorHint(super.message) : super(level: DiagnosticLevel.hint);

  /// Calls to the default constructor may be rewritten to use this constructor
  /// in debug mode using a kernel transformer.
  // ignore: unused_element
  ErrorHint._fromParts(super.messageParts) : super._fromParts(level: DiagnosticLevel.hint);
}

/// An [ErrorSpacer] creates an empty [DiagnosticsNode], that can be used to
/// tune the spacing between other [DiagnosticsNode] objects.
class ErrorSpacer extends DiagnosticsProperty<void> {
  /// Creates an empty space to insert into a list of [DiagnosticsNode] objects
  /// typically within a [FlutterError] object.
  ErrorSpacer() : super('', null, description: '', showName: false);
}

/// Class for information provided to [FlutterExceptionHandler] callbacks.
///
/// {@tool snippet}
/// This is an example of using [FlutterErrorDetails] when calling
/// [FlutterError.reportError].
///
/// ```dart
/// void main() {
///   try {
///     // Try to do something!
///   } catch (error) {
///     // Catch & report error.
///     FlutterError.reportError(FlutterErrorDetails(
///       exception: error,
///       library: 'Flutter test framework',
///       context: ErrorSummary('while running async test code'),
///     ));
///   }
/// }
/// ```
/// {@end-tool}
///
/// See also:
///
///   * [FlutterError.onError], which is called whenever the Flutter framework
///     catches an error.
class FlutterErrorDetails with Diagnosticable {
  /// Creates a [FlutterErrorDetails] object with the given arguments setting
  /// the object's properties.
  ///
  /// The framework calls this constructor when catching an exception that will
  /// subsequently be reported using [FlutterError.onError].
  const FlutterErrorDetails({
    required this.exception,
    this.stack,
    this.library = 'Flutter framework',
    this.context,
    this.stackFilter,
    this.informationCollector,
    this.silent = false,
  });

  /// Creates a copy of the error details but with the given fields replaced
  /// with new values.
  FlutterErrorDetails copyWith({
    DiagnosticsNode? context,
    Object? exception,
    InformationCollector? informationCollector,
    String? library,
    bool? silent,
    StackTrace? stack,
    IterableFilter<String>? stackFilter,
  }) {
    return FlutterErrorDetails(
      context: context ?? this.context,
      exception: exception ?? this.exception,
      informationCollector: informationCollector ?? this.informationCollector,
      library: library ?? this.library,
      silent: silent ?? this.silent,
      stack: stack ?? this.stack,
      stackFilter: stackFilter ?? this.stackFilter,
    );
  }

  /// Transformers to transform [DiagnosticsNode] in [DiagnosticPropertiesBuilder]
  /// into a more descriptive form.
  ///
  /// There are layers that attach certain [DiagnosticsNode] into
  /// [FlutterErrorDetails] that require knowledge from other layers to parse.
  /// To correctly interpret those [DiagnosticsNode], register transformers in
  /// the layers that possess the knowledge.
  ///
  /// See also:
  ///
  ///  * [WidgetsBinding.initInstances], which registers its transformer.
  static final List<DiagnosticPropertiesTransformer> propertiesTransformers =
      <DiagnosticPropertiesTransformer>[];

  /// The exception. Often this will be an [AssertionError], maybe specifically
  /// a [FlutterError]. However, this could be any value at all.
  final Object exception;

  /// The stack trace from where the [exception] was thrown (as opposed to where
  /// it was caught).
  ///
  /// StackTrace objects are opaque except for their [toString] function.
  ///
  /// If this field is not null, then the [stackFilter] callback, if any, will
  /// be called with the result of calling [toString] on this object and
  /// splitting that result on line breaks. If there's no [stackFilter]
  /// callback, then [FlutterError.defaultStackFilter] is used instead. That
  /// function expects the stack to be in the format used by
  /// [StackTrace.toString].
  final StackTrace? stack;

  /// A human-readable brief name describing the library that caught the error
  /// message. This is used by the default error handler in the header dumped to
  /// the console.
  final String? library;

  /// A [DiagnosticsNode] that provides a human-readable description of where
  /// the error was caught (as opposed to where it was thrown).
  ///
  /// The node, e.g. an [ErrorDescription], should be in a form that will make
  /// sense in English when following the word "thrown", as in "thrown while
  /// obtaining the image from the network" (for the context "while obtaining
  /// the image from the network").
  ///
  /// {@tool snippet}
  /// This is an example of using and [ErrorDescription] as the
  /// [FlutterErrorDetails.context] when calling [FlutterError.reportError].
  ///
  /// ```dart
  /// void maybeDoSomething() {
  ///   try {
  ///     // Try to do something!
  ///   } catch (error) {
  ///     // Catch & report error.
  ///     FlutterError.reportError(FlutterErrorDetails(
  ///       exception: error,
  ///       library: 'Flutter test framework',
  ///       context: ErrorDescription('while dispatching notifications for $runtimeType'),
  ///     ));
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [ErrorDescription], which provides an explanation of the problem and
  ///    its cause, any information that may help track down the problem,
  ///    background information, etc.
  ///  * [ErrorSummary], which provides a short (one line) description of the
  ///    problem that was detected.
  ///  * [ErrorHint], which provides specific, non-obvious advice that may be
  ///    applicable.
  ///  * [FlutterError], which is the most common place to use
  ///    [FlutterErrorDetails].
  final DiagnosticsNode? context;

  /// A callback which filters the [stack] trace. Receives an iterable of
  /// strings representing the frames encoded in the way that
  /// [StackTrace.toString()] provides. Should return an iterable of lines to
  /// output for the stack.
  ///
  /// If this is not provided, then [FlutterError.dumpErrorToConsole] will use
  /// [FlutterError.defaultStackFilter] instead.
  ///
  /// If the [FlutterError.defaultStackFilter] behavior is desired, then the
  /// callback should manually call that function. That function expects the
  /// incoming list to be in the [StackTrace.toString()] format. The output of
  /// that function, however, does not always follow this format.
  ///
  /// This won't be called if [stack] is null.
  final IterableFilter<String>? stackFilter;

  /// A callback which will provide information that could help with debugging
  /// the problem.
  ///
  /// Information collector callbacks can be expensive, so the generated
  /// information should be cached by the caller, rather than the callback being
  /// called multiple times.
  ///
  /// The callback is expected to return an iterable of [DiagnosticsNode] objects,
  /// typically implemented using `sync*` and `yield`.
  ///
  /// {@tool snippet}
  /// In this example, the information collector returns two pieces of information,
  /// one broadly-applicable statement regarding how the error happened, and one
  /// giving a specific piece of information that may be useful in some cases but
  /// may also be irrelevant most of the time (an argument to the method).
  ///
  /// ```dart
  /// void climbElevator(int pid) {
  ///   try {
  ///     // ...
  ///   } catch (error, stack) {
  ///     FlutterError.reportError(FlutterErrorDetails(
  ///       exception: error,
  ///       stack: stack,
  ///       informationCollector: () => <DiagnosticsNode>[
  ///         ErrorDescription('This happened while climbing the space elevator.'),
  ///         ErrorHint('The process ID is: $pid'),
  ///       ],
  ///     ));
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  ///
  /// The following classes may be of particular use:
  ///
  ///  * [ErrorDescription], for information that is broadly applicable to the
  ///    situation being described.
  ///  * [ErrorHint], for specific information that may not always be applicable
  ///    but can be helpful in certain situations.
  ///  * [DiagnosticsStackTrace], for reporting stack traces.
  ///  * [ErrorSpacer], for adding spaces (a blank line) between other items.
  ///
  /// For objects that implement [Diagnosticable] one may consider providing
  /// additional information by yielding the output of the object's
  /// [Diagnosticable.toDiagnosticsNode] method.
  final InformationCollector? informationCollector;

  /// Whether this error should be ignored by the default error reporting
  /// behavior in release mode.
  ///
  /// If this is false, the default, then the default error handler will always
  /// dump this error to the console.
  ///
  /// If this is true, then the default error handler would only dump this error
  /// to the console in debug mode. In release mode, the error is ignored.
  ///
  /// This is used by certain exception handlers that catch errors that could be
  /// triggered by environmental conditions (as opposed to logic errors). For
  /// example, the HTTP library sets this flag so as to not report every 404
  /// error to the console on end-user devices, while still allowing a custom
  /// error handler to see the errors even in release builds.
  final bool silent;

  /// Converts the [exception] to a string.
  ///
  /// This applies some additional logic to make [AssertionError] exceptions
  /// prettier, to handle exceptions that stringify to empty strings, to handle
  /// objects that don't inherit from [Exception] or [Error], and so forth.
  String exceptionAsString() {
    String? longMessage;
    if (exception is AssertionError) {
      // Regular _AssertionErrors thrown by assert() put the message last, after
      // some code snippets. This leads to ugly messages. To avoid this, we move
      // the assertion message up to before the code snippets, separated by a
      // newline, if we recognize that format is being used.
      final Object? message = (exception as AssertionError).message;
      final String fullMessage = exception.toString();
      if (message is String && message != fullMessage) {
        if (fullMessage.length > message.length) {
          final int position = fullMessage.lastIndexOf(message);
          if (position == fullMessage.length - message.length &&
              position > 2 &&
              fullMessage.substring(position - 2, position) == ': ') {
            // Add a linebreak so that the filename at the start of the
            // assertion message is always on its own line.
            String body = fullMessage.substring(0, position - 2);
            final int splitPoint = body.indexOf(' Failed assertion:');
            if (splitPoint >= 0) {
              body = '${body.substring(0, splitPoint)}\n${body.substring(splitPoint + 1)}';
            }
            longMessage = '${message.trimRight()}\n$body';
          }
        }
      }
      longMessage ??= fullMessage;
    } else if (exception is String) {
      longMessage = exception as String;
    } else if (exception is Error || exception is Exception) {
      longMessage = exception.toString();
    } else {
      longMessage = '  $exception';
    }
    longMessage = longMessage.trimRight();
    if (longMessage.isEmpty) {
      longMessage = '  <no message available>';
    }
    return longMessage;
  }

  Diagnosticable? _exceptionToDiagnosticable() {
    final Object exception = this.exception;
    if (exception is FlutterError) {
      return exception;
    }
    if (exception is AssertionError && exception.message is FlutterError) {
      return exception.message! as FlutterError;
    }
    return null;
  }

  /// Returns a short (one line) description of the problem that was detected.
  ///
  /// If the exception contains an [ErrorSummary] that summary is used,
  /// otherwise the summary is inferred from the string representation of the
  /// exception.
  ///
  /// In release mode, this always returns a [DiagnosticsNode.message] with a
  /// formatted version of the exception.
  DiagnosticsNode get summary {
    String formatException() => exceptionAsString().split('\n')[0].trimLeft();
    if (kReleaseMode) {
      return DiagnosticsNode.message(formatException());
    }
    final Diagnosticable? diagnosticable = _exceptionToDiagnosticable();
    DiagnosticsNode? summary;
    if (diagnosticable != null) {
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      debugFillProperties(builder);
      summary = builder.properties.cast<DiagnosticsNode?>().firstWhere(
        (DiagnosticsNode? node) => node!.level == DiagnosticLevel.summary,
        orElse: () => null,
      );
    }
    return summary ?? ErrorSummary(formatException());
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    final DiagnosticsNode verb = ErrorDescription(
      'thrown${context != null ? ErrorDescription(" $context") : ""}',
    );
    final Diagnosticable? diagnosticable = _exceptionToDiagnosticable();
    if (exception is num) {
      properties.add(ErrorDescription('The number $exception was $verb.'));
    } else {
      final DiagnosticsNode errorName = ErrorDescription(switch (exception) {
        AssertionError() => 'assertion',
        String() => 'message',
        Error() || Exception() => '${exception.runtimeType}',
        _ => '${exception.runtimeType} object',
      });
      properties.add(ErrorDescription('The following $errorName was $verb:'));
      if (diagnosticable != null) {
        diagnosticable.debugFillProperties(properties);
      } else {
        // Many exception classes put their type at the head of their message.
        // This is redundant with the way we display exceptions, so attempt to
        // strip out that header when we see it.
        final String prefix = '${exception.runtimeType}: ';
        String message = exceptionAsString();
        if (message.startsWith(prefix)) {
          message = message.substring(prefix.length);
        }
        properties.add(ErrorSummary(message));
      }
    }

    if (stack != null) {
      if (exception is AssertionError && diagnosticable == null) {
        // After popping off any dart: stack frames, are there at least two more
        // stack frames coming from package flutter?
        //
        // If not: Error is in user code (user violated assertion in framework).
        // If so:  Error is in Framework. We either need an assertion higher up
        //         in the stack, or we've violated our own assertions.
        final List<StackFrame> stackFrames =
            StackFrame.fromStackTrace(
              FlutterError.demangleStackTrace(stack!),
            ).skipWhile((StackFrame frame) => frame.packageScheme == 'dart').toList();
        final bool ourFault =
            stackFrames.length >= 2 &&
            stackFrames[0].package == 'flutter' &&
            stackFrames[1].package == 'flutter';
        if (ourFault) {
          properties.add(ErrorSpacer());
          properties.add(
            ErrorHint(
              'Either the assertion indicates an error in the framework itself, or we should '
              'provide substantially more information in this error message to help you determine '
              'and fix the underlying cause.\n'
              'In either case, please report this assertion by filing a bug on GitHub:\n'
              '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
            ),
          );
        }
      }
      properties.add(ErrorSpacer());
      properties.add(
        DiagnosticsStackTrace(
          'When the exception was thrown, this was the stack',
          stack,
          stackFilter: stackFilter,
        ),
      );
    }
    if (informationCollector != null) {
      properties.add(ErrorSpacer());
      informationCollector!().forEach(properties.add);
    }
  }

  @override
  String toStringShort() {
    return library != null ? 'Exception caught by $library' : 'Exception caught';
  }

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    return toDiagnosticsNode(style: DiagnosticsTreeStyle.error).toStringDeep(minLevel: minLevel);
  }

  @override
  DiagnosticsNode toDiagnosticsNode({String? name, DiagnosticsTreeStyle? style}) {
    return _FlutterErrorDetailsNode(name: name, value: this, style: style);
  }
}

/// Error class used to report Flutter-specific assertion failures and
/// contract violations.
///
/// See also:
///
///  * <https://docs.flutter.dev/testing/errors>, more information about error
///    handling in Flutter.
class FlutterError extends Error with DiagnosticableTreeMixin implements AssertionError {
  /// Create an error message from a string.
  ///
  /// The message may have newlines in it. The first line should be a terse
  /// description of the error, e.g. "Incorrect GlobalKey usage" or "setState()
  /// or markNeedsBuild() called during build". Subsequent lines should contain
  /// substantial additional information, ideally sufficient to develop a
  /// correct solution to the problem.
  ///
  /// In some cases, when a [FlutterError] is reported to the user, only the first
  /// line is included. For example, Flutter will typically only fully report
  /// the first exception at runtime, displaying only the first line of
  /// subsequent errors.
  ///
  /// All sentences in the error should be correctly punctuated (i.e.,
  /// do end the error message with a period).
  ///
  /// This constructor defers to the [FlutterError.fromParts] constructor.
  /// The first line is wrapped in an implied [ErrorSummary], and subsequent
  /// lines are wrapped in implied [ErrorDescription]s. Consider using the
  /// [FlutterError.fromParts] constructor to provide more detail, e.g.
  /// using [ErrorHint]s or other [DiagnosticsNode]s.
  factory FlutterError(String message) {
    final List<String> lines = message.split('\n');
    return FlutterError.fromParts(<DiagnosticsNode>[
      ErrorSummary(lines.first),
      ...lines.skip(1).map<DiagnosticsNode>((String line) => ErrorDescription(line)),
    ]);
  }

  /// Create an error message from a list of [DiagnosticsNode]s.
  ///
  /// By convention, there should be exactly one [ErrorSummary] in the list,
  /// and it should be the first entry.
  ///
  /// Other entries are typically [ErrorDescription]s (for material that is
  /// always applicable for this error) and [ErrorHint]s (for material that may
  /// be sometimes useful, but may not always apply). Other [DiagnosticsNode]
  /// subclasses, such as [DiagnosticsStackTrace], may
  /// also be used.
  ///
  /// When using an [ErrorSummary], [ErrorDescription]s, and [ErrorHint]s, in
  /// debug builds, values interpolated into the `message` arguments of those
  /// classes' constructors are expanded and placed into the
  /// [DiagnosticsProperty.value] property of those objects (which is of type
  /// [List<Object>]). This allows IDEs to examine values interpolated into
  /// error messages.
  ///
  /// Alternatively, to include a specific [Diagnosticable] object into the
  /// error message and have the object describe itself in detail (see
  /// [DiagnosticsNode.toStringDeep]), consider calling
  /// [Diagnosticable.toDiagnosticsNode] on that object and using that as one of
  /// the values passed to this constructor.
  ///
  /// {@tool snippet}
  /// In this example, an error is thrown in debug mode if certain conditions
  /// are not met. The error message includes a description of an object that
  /// implements the [Diagnosticable] interface, `draconis`.
  ///
  /// ```dart
  /// void controlDraconis() {
  ///   assert(() {
  ///     if (!draconisAlive || !draconisAmulet) {
  ///       throw FlutterError.fromParts(<DiagnosticsNode>[
  ///         ErrorSummary('Cannot control Draconis in current state.'),
  ///         ErrorDescription('Draconis can only be controlled while alive and while the amulet is wielded.'),
  ///         if (!draconisAlive)
  ///           ErrorHint('Draconis is currently not alive.'),
  ///         if (!draconisAmulet)
  ///           ErrorHint('The Amulet of Draconis is currently not wielded.'),
  ///         draconis.toDiagnosticsNode(name: 'Draconis'),
  ///       ]);
  ///     }
  ///     return true;
  ///   }());
  ///   // ...
  /// }
  /// ```
  /// {@end-tool}
  FlutterError.fromParts(this.diagnostics)
    : assert(
        diagnostics.isNotEmpty,
        FlutterError.fromParts(<DiagnosticsNode>[ErrorSummary('Empty FlutterError')]),
      ) {
    assert(
      diagnostics.first.level == DiagnosticLevel.summary,
      FlutterError.fromParts(<DiagnosticsNode>[
        ErrorSummary('FlutterError is missing a summary.'),
        ErrorDescription(
          'All FlutterError objects should start with a short (one line) '
          'summary description of the problem that was detected.',
        ),
        DiagnosticsProperty<FlutterError>(
          'Malformed',
          this,
          expandableValue: true,
          showSeparator: false,
          style: DiagnosticsTreeStyle.whitespace,
        ),
        ErrorDescription(
          '\nThis error should still help you solve your problem, '
          'however please also report this malformed error in the '
          'framework by filing a bug on GitHub:\n'
          '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
        ),
      ]),
    );
    assert(() {
      final Iterable<DiagnosticsNode> summaries = diagnostics.where(
        (DiagnosticsNode node) => node.level == DiagnosticLevel.summary,
      );
      if (summaries.length > 1) {
        final List<DiagnosticsNode> message = <DiagnosticsNode>[
          ErrorSummary('FlutterError contained multiple error summaries.'),
          ErrorDescription(
            'All FlutterError objects should have only a single short '
            '(one line) summary description of the problem that was '
            'detected.',
          ),
          DiagnosticsProperty<FlutterError>(
            'Malformed',
            this,
            expandableValue: true,
            showSeparator: false,
            style: DiagnosticsTreeStyle.whitespace,
          ),
          ErrorDescription('\nThe malformed error has ${summaries.length} summaries.'),
        ];
        int i = 1;
        for (final DiagnosticsNode summary in summaries) {
          message.add(
            DiagnosticsProperty<DiagnosticsNode>('Summary $i', summary, expandableValue: true),
          );
          i += 1;
        }
        message.add(
          ErrorDescription(
            '\nThis error should still help you solve your problem, '
            'however please also report this malformed error in the '
            'framework by filing a bug on GitHub:\n'
            '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
          ),
        );
        throw FlutterError.fromParts(message);
      }
      return true;
    }());
  }

  /// The information associated with this error, in structured form.
  ///
  /// The first node is typically an [ErrorSummary] giving a short description
  /// of the problem, suitable for an index of errors, a log, etc.
  ///
  /// Subsequent nodes should give information specific to this error. Typically
  /// these will be [ErrorDescription]s or [ErrorHint]s, but they could be other
  /// objects also. For instance, an error relating to a timer could include a
  /// stack trace of when the timer was scheduled using the
  /// [DiagnosticsStackTrace] class.
  final List<DiagnosticsNode> diagnostics;

  /// The message associated with this error.
  ///
  /// This is generated by serializing the [diagnostics].
  @override
  String get message => toString();

  /// Called whenever the Flutter framework catches an error.
  ///
  /// The default behavior is to call [presentError].
  ///
  /// You can set this to your own function to override this default behavior.
  /// For example, you could report all errors to your server. Consider calling
  /// [presentError] from your custom error handler in order to see the logs in
  /// the console as well.
  ///
  /// If the error handler throws an exception, it will not be caught by the
  /// Flutter framework.
  ///
  /// Set this to null to silently catch and ignore errors. This is not
  /// recommended.
  ///
  /// Do not call [onError] directly, instead, call [reportError], which
  /// forwards to [onError] if it is not null.
  ///
  /// See also:
  ///
  ///  * <https://docs.flutter.dev/testing/errors>, more information about error
  ///    handling in Flutter.
  static FlutterExceptionHandler? onError = presentError;

  /// Called by the Flutter framework before attempting to parse a [StackTrace].
  ///
  /// Some [StackTrace] implementations have a different [toString] format from
  /// what the framework expects, like ones from `package:stack_trace`. To make
  /// sure we can still parse and filter mangled [StackTrace]s, the framework
  /// first calls this function to demangle them.
  ///
  /// This should be set in any environment that could propagate an unusual
  /// stack trace to the framework. Otherwise, the default behavior is to assume
  /// all stack traces are in a format usually generated by Dart.
  ///
  /// The following example demangles `package:stack_trace` traces by converting
  /// them into VM traces, which the framework is able to parse:
  ///
  /// ```dart
  /// FlutterError.demangleStackTrace = (StackTrace stack) {
  ///   // Trace and Chain are classes in package:stack_trace
  ///   if (stack is Trace) {
  ///     return stack.vmTrace;
  ///   }
  ///   if (stack is Chain) {
  ///     return stack.toTrace().vmTrace;
  ///   }
  ///   return stack;
  /// };
  /// ```
  static StackTraceDemangler demangleStackTrace = _defaultStackTraceDemangler;

  static StackTrace _defaultStackTraceDemangler(StackTrace stackTrace) => stackTrace;

  /// Called whenever the Flutter framework wants to present an error to the
  /// users.
  ///
  /// The default behavior is to call [dumpErrorToConsole].
  ///
  /// Plugins can override how an error is to be presented to the user. For
  /// example, the structured errors service extension sets its own method when
  /// the extension is enabled. If you want to change how Flutter responds to an
  /// error, use [onError] instead.
  static FlutterExceptionHandler presentError = dumpErrorToConsole;

  static int _errorCount = 0;

  /// Resets the count of errors used by [dumpErrorToConsole] to decide whether
  /// to show a complete error message or an abbreviated one.
  ///
  /// After this is called, the next error message will be shown in full.
  static void resetErrorCount() {
    _errorCount = 0;
  }

  /// The width to which [dumpErrorToConsole] will wrap lines.
  ///
  /// This can be used to ensure strings will not exceed the length at which
  /// they will wrap, e.g. when placing ASCII art diagrams in messages.
  static const int wrapWidth = 100;

  /// Prints the given exception details to the console.
  ///
  /// The first time this is called, it dumps a very verbose message to the
  /// console using [debugPrint].
  ///
  /// Subsequent calls only dump the first line of the exception, unless
  /// `forceReport` is set to true (in which case it dumps the verbose message).
  ///
  /// Call [resetErrorCount] to cause this method to go back to acting as if it
  /// had not been called before (so the next message is verbose again).
  ///
  /// The default behavior for the [onError] handler is to call this function.
  static void dumpErrorToConsole(FlutterErrorDetails details, {bool forceReport = false}) {
    bool isInDebugMode = false;
    assert(() {
      // In debug mode, we ignore the "silent" flag.
      isInDebugMode = true;
      return true;
    }());
    final bool reportError = isInDebugMode || !details.silent;
    if (!reportError && !forceReport) {
      return;
    }
    if (_errorCount == 0 || forceReport) {
      // Diagnostics is only available in debug mode. In profile and release modes fallback to plain print.
      if (isInDebugMode) {
        debugPrint(
          TextTreeRenderer(
            wrapWidthProperties: wrapWidth,
            maxDescendentsTruncatableNode: 5,
          ).render(details.toDiagnosticsNode(style: DiagnosticsTreeStyle.error)).trimRight(),
        );
      } else {
        debugPrintStack(
          stackTrace: details.stack,
          label: details.exception.toString(),
          maxFrames: 100,
        );
      }
    } else {
      debugPrint('Another exception was thrown: ${details.summary}');
    }
    _errorCount += 1;
  }

  static final List<StackFilter> _stackFilters = <StackFilter>[];

  /// Adds a stack filtering function to [defaultStackFilter].
  ///
  /// For example, the framework adds common patterns of element building to
  /// elide tree-walking patterns in the stack trace.
  ///
  /// Added filters are checked in order of addition. The first matching filter
  /// wins, and subsequent filters will not be checked.
  static void addDefaultStackFilter(StackFilter filter) {
    _stackFilters.add(filter);
  }

  /// Converts a stack to a string that is more readable by omitting stack
  /// frames that correspond to Dart internals.
  ///
  /// This is the default filter used by [dumpErrorToConsole] if the
  /// [FlutterErrorDetails] object has no [FlutterErrorDetails.stackFilter]
  /// callback.
  ///
  /// This function expects its input to be in the format used by
  /// [StackTrace.toString()]. The output of this function is similar to that
  /// format but the frame numbers will not be consecutive (frames are elided)
  /// and the final line may be prose rather than a stack frame.
  static Iterable<String> defaultStackFilter(Iterable<String> frames) {
    final Map<String, int> removedPackagesAndClasses = <String, int>{
      'dart:async-patch': 0,
      'dart:async': 0,
      'package:stack_trace': 0,
      'class _AssertionError': 0,
      'class _FakeAsync': 0,
      'class _FrameCallbackEntry': 0,
      'class _Timer': 0,
      'class _RawReceivePortImpl': 0,
    };
    int skipped = 0;

    final List<StackFrame> parsedFrames = StackFrame.fromStackString(frames.join('\n'));

    for (int index = 0; index < parsedFrames.length; index += 1) {
      final StackFrame frame = parsedFrames[index];
      final String className = 'class ${frame.className}';
      final String package = '${frame.packageScheme}:${frame.package}';
      if (removedPackagesAndClasses.containsKey(className)) {
        skipped += 1;
        removedPackagesAndClasses.update(className, (int value) => value + 1);
        parsedFrames.removeAt(index);
        index -= 1;
      } else if (removedPackagesAndClasses.containsKey(package)) {
        skipped += 1;
        removedPackagesAndClasses.update(package, (int value) => value + 1);
        parsedFrames.removeAt(index);
        index -= 1;
      }
    }
    final List<String?> reasons = List<String?>.filled(parsedFrames.length, null);
    for (final StackFilter filter in _stackFilters) {
      filter.filter(parsedFrames, reasons);
    }

    final List<String> result = <String>[];

    // Collapse duplicated reasons.
    for (int index = 0; index < parsedFrames.length; index += 1) {
      final int start = index;
      while (index < reasons.length - 1 &&
          reasons[index] != null &&
          reasons[index + 1] == reasons[index]) {
        index++;
      }
      String suffix = '';
      if (reasons[index] != null) {
        if (index != start) {
          suffix = ' (${index - start + 2} frames)';
        } else {
          suffix = ' (1 frame)';
        }
      }
      final String resultLine = '${reasons[index] ?? parsedFrames[index].source}$suffix';
      result.add(resultLine);
    }

    // Only include packages we actually elided from.
    final List<String> where = <String>[
      for (final MapEntry<String, int> entry in removedPackagesAndClasses.entries)
        if (entry.value > 0) entry.key,
    ]..sort();
    if (skipped == 1) {
      result.add('(elided one frame from ${where.single})');
    } else if (skipped > 1) {
      if (where.length > 1) {
        where[where.length - 1] = 'and ${where.last}';
      }
      if (where.length > 2) {
        result.add('(elided $skipped frames from ${where.join(", ")})');
      } else {
        result.add('(elided $skipped frames from ${where.join(" ")})');
      }
    }
    return result;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    diagnostics.forEach(properties.add);
  }

  @override
  String toStringShort() => 'FlutterError';

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    if (kReleaseMode) {
      final Iterable<_ErrorDiagnostic> errors = diagnostics.whereType<_ErrorDiagnostic>();
      return errors.isNotEmpty ? errors.first.valueToString() : toStringShort();
    }
    // Avoid wrapping lines.
    final TextTreeRenderer renderer = TextTreeRenderer(wrapWidth: 4000000000);
    return diagnostics.map((DiagnosticsNode node) => renderer.render(node).trimRight()).join('\n');
  }

  /// Calls [onError] with the given details, unless it is null.
  ///
  /// {@tool snippet}
  /// When calling this from a `catch` block consider annotating the method
  /// containing the `catch` block with
  /// `@pragma('vm:notify-debugger-on-exception')` to allow an attached debugger
  /// to treat the exception as unhandled. This means instead of executing the
  /// `catch` block, the debugger can break at the original source location from
  /// which the exception was thrown.
  ///
  /// ```dart
  /// @pragma('vm:notify-debugger-on-exception')
  /// void doSomething() {
  ///   try {
  ///     methodThatMayThrow();
  ///   } catch (exception, stack) {
  ///     FlutterError.reportError(FlutterErrorDetails(
  ///       exception: exception,
  ///       stack: stack,
  ///       library: 'example library',
  ///       context: ErrorDescription('while doing something'),
  ///     ));
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  static void reportError(FlutterErrorDetails details) {
    onError?.call(details);
  }
}

/// Dump the stack to the console using [debugPrint] and
/// [FlutterError.defaultStackFilter].
///
/// If the `stackTrace` parameter is null, the [StackTrace.current] is used to
/// obtain the stack.
///
/// The `maxFrames` argument can be given to limit the stack to the given number
/// of lines before filtering is applied. By default, all stack lines are
/// included.
///
/// The `label` argument, if present, will be printed before the stack.
void debugPrintStack({StackTrace? stackTrace, String? label, int? maxFrames}) {
  if (label != null) {
    debugPrint(label);
  }
  if (stackTrace == null) {
    stackTrace = StackTrace.current;
  } else {
    stackTrace = FlutterError.demangleStackTrace(stackTrace);
  }
  Iterable<String> lines = stackTrace.toString().trimRight().split('\n');
  if (kIsWeb && lines.isNotEmpty) {
    // Remove extra call to StackTrace.current for web platform.
    // TODO(ferhat): remove when https://github.com/flutter/flutter/issues/37635
    // is addressed.
    lines = lines.skipWhile((String line) {
      return line.contains('StackTrace.current') ||
          line.contains('dart-sdk/lib/_internal') ||
          line.contains('dart:sdk_internal');
    });
  }
  if (maxFrames != null) {
    lines = lines.take(maxFrames);
  }
  debugPrint(FlutterError.defaultStackFilter(lines).join('\n'));
}

/// Diagnostic with a [StackTrace] [value] suitable for displaying stack traces
/// as part of a [FlutterError] object.
class DiagnosticsStackTrace extends DiagnosticsBlock {
  /// Creates a diagnostic for a stack trace.
  ///
  /// [name] describes a name the stack trace is given, e.g.
  /// `When the exception was thrown, this was the stack`.
  /// [stackFilter] provides an optional filter to use to filter which frames
  /// are included. If no filter is specified, [FlutterError.defaultStackFilter]
  /// is used.
  /// [showSeparator] indicates whether to include a ':' after the [name].
  DiagnosticsStackTrace(
    String name,
    StackTrace? stack, {
    IterableFilter<String>? stackFilter,
    super.showSeparator,
  }) : super(
         name: name,
         value: stack,
         properties: _applyStackFilter(stack, stackFilter),
         style: DiagnosticsTreeStyle.flat,
         allowTruncate: true,
       );

  /// Creates a diagnostic describing a single frame from a StackTrace.
  DiagnosticsStackTrace.singleFrame(String name, {required String frame, super.showSeparator})
    : super(
        name: name,
        properties: <DiagnosticsNode>[_createStackFrame(frame)],
        style: DiagnosticsTreeStyle.whitespace,
      );

  static List<DiagnosticsNode> _applyStackFilter(
    StackTrace? stack,
    IterableFilter<String>? stackFilter,
  ) {
    if (stack == null) {
      return <DiagnosticsNode>[];
    }
    final IterableFilter<String> filter = stackFilter ?? FlutterError.defaultStackFilter;
    final Iterable<String> frames = filter(
      '${FlutterError.demangleStackTrace(stack)}'.trimRight().split('\n'),
    );
    return frames.map<DiagnosticsNode>(_createStackFrame).toList();
  }

  static DiagnosticsNode _createStackFrame(String frame) {
    return DiagnosticsNode.message(frame, allowWrap: false);
  }

  @override
  bool get allowTruncate => false;
}

class _FlutterErrorDetailsNode extends DiagnosticableNode<FlutterErrorDetails> {
  _FlutterErrorDetailsNode({super.name, required super.value, required super.style});

  @override
  DiagnosticPropertiesBuilder? get builder {
    final DiagnosticPropertiesBuilder? builder = super.builder;
    if (builder == null) {
      return null;
    }
    Iterable<DiagnosticsNode> properties = builder.properties;
    for (final DiagnosticPropertiesTransformer transformer
        in FlutterErrorDetails.propertiesTransformers) {
      properties = transformer(properties);
    }
    return DiagnosticPropertiesBuilder.fromProperties(properties.toList());
  }
}
