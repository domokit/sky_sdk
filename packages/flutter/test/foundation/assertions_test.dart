// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

import 'capture_output.dart';

void main() {
  test('debugPrintStack', () {
    final List<String> log = captureOutput(() {
      debugPrintStack(label: 'Example label', maxFrames: 7);
    });
    expect(log[0], contains('Example label'));
    expect(log[1], contains('debugPrintStack'));
  });

  test('should show message of ErrorDescription', () {
    const String descriptionMessage = 'This is the message';
    final ErrorDescription errorDescription = ErrorDescription(descriptionMessage);

    expect(errorDescription.toString(), descriptionMessage);
  });

  test('debugPrintStack', () {
    final List<String> log = captureOutput(() {
      final FlutterErrorDetails details = FlutterErrorDetails(
        exception: 'Example exception',
        stack: StackTrace.current,
        library: 'Example library',
        context: ErrorDescription('Example context'),
        informationCollector: () sync* {
          yield ErrorDescription('Example information');
        },
      );

      FlutterError.dumpErrorToConsole(details);
    });

    expect(log[0], contains('EXAMPLE LIBRARY'));
    expect(log[1], contains('Example context'));
    expect(log[2], contains('Example exception'));

    final String joined = log.join('\n');

    expect(joined, contains('captureOutput'));
    expect(joined, contains('\nExample information\n'));
  });

  test('FlutterErrorDetails.toString', () {
    expect(
      FlutterErrorDetails(
        exception: 'MESSAGE',
        library: 'LIBRARY',
        context: ErrorDescription('CONTEXTING'),
        informationCollector: () sync* {
          yield ErrorDescription('INFO');
        },
      ).toString(),
      '══╡ EXCEPTION CAUGHT BY LIBRARY ╞════════════════════════════════\n'
      'The following message was thrown CONTEXTING:\n'
      'MESSAGE\n'
      '\n'
      'INFO\n'
      '═════════════════════════════════════════════════════════════════\n',
    );
    expect(
      FlutterErrorDetails(
        exception: 'MESSAGE',
        context: ErrorDescription('CONTEXTING'),
        informationCollector: () sync* {
          yield ErrorDescription('INFO');
        },
      ).toString(),
      '══╡ EXCEPTION CAUGHT BY FLUTTER FRAMEWORK ╞══════════════════════\n'
      'The following message was thrown CONTEXTING:\n'
      'MESSAGE\n'
      '\n'
      'INFO\n'
      '═════════════════════════════════════════════════════════════════\n',
    );
    expect(
      FlutterErrorDetails(
        exception: 'MESSAGE',
        context: ErrorDescription('CONTEXTING ${'SomeContext(BlaBla)'}'),
        informationCollector: () sync* {
          yield ErrorDescription('INFO');
        },
      ).toString(),
      '══╡ EXCEPTION CAUGHT BY FLUTTER FRAMEWORK ╞══════════════════════\n'
      'The following message was thrown CONTEXTING SomeContext(BlaBla):\n'
      'MESSAGE\n'
      '\n'
      'INFO\n'
      '═════════════════════════════════════════════════════════════════\n',
    );
    expect(
      const FlutterErrorDetails(exception: 'MESSAGE').toString(),
      '══╡ EXCEPTION CAUGHT BY FLUTTER FRAMEWORK ╞══════════════════════\n'
      'The following message was thrown:\n'
      'MESSAGE\n'
      '═════════════════════════════════════════════════════════════════\n',
    );
  });

  test('FlutterErrorDetails.toStringShort', () {
    expect(
      FlutterErrorDetails(
        exception: 'MESSAGE',
        library: 'library',
        context: ErrorDescription('CONTEXTING'),
        informationCollector: () sync* {
          yield ErrorDescription('INFO');
        },
      ).toStringShort(),
      'Exception caught by library',
    );
  });

  test('FlutterError default constructor', () {
    FlutterError error = FlutterError(
      'My Error Summary.\n'
      'My first description.\n'
      'My second description.',
    );
    expect(error.diagnostics.length, equals(3));
    expect(error.diagnostics[0].level, DiagnosticLevel.summary);
    expect(error.diagnostics[1].level, DiagnosticLevel.info);
    expect(error.diagnostics[2].level, DiagnosticLevel.info);
    expect(error.diagnostics[0].toString(), 'My Error Summary.');
    expect(error.diagnostics[1].toString(), 'My first description.');
    expect(error.diagnostics[2].toString(), 'My second description.');
    expect(
      error.toStringDeep(),
      'FlutterError\n'
      '   My Error Summary.\n'
      '   My first description.\n'
      '   My second description.\n',
    );

    error = FlutterError(
      'My Error Summary.\n'
      'My first description.\n'
      'My second description.\n'
      '\n',
    );

    expect(error.diagnostics.length, equals(5));
    expect(error.diagnostics[0].level, DiagnosticLevel.summary);
    expect(error.diagnostics[1].level, DiagnosticLevel.info);
    expect(error.diagnostics[2].level, DiagnosticLevel.info);
    expect(error.diagnostics[0].toString(), 'My Error Summary.');
    expect(error.diagnostics[1].toString(), 'My first description.');
    expect(error.diagnostics[2].toString(), 'My second description.');
    expect(error.diagnostics[3].toString(), '');
    expect(error.diagnostics[4].toString(), '');
    expect(
      error.toStringDeep(),
      'FlutterError\n'
      '   My Error Summary.\n'
      '   My first description.\n'
      '   My second description.\n'
      '\n'
      '\n',
    );

    error = FlutterError(
      'My Error Summary.\n'
      'My first description.\n'
      '\n'
      'My second description.',
    );
    expect(error.diagnostics.length, equals(4));
    expect(error.diagnostics[0].level, DiagnosticLevel.summary);
    expect(error.diagnostics[1].level, DiagnosticLevel.info);
    expect(error.diagnostics[2].level, DiagnosticLevel.info);
    expect(error.diagnostics[3].level, DiagnosticLevel.info);
    expect(error.diagnostics[0].toString(), 'My Error Summary.');
    expect(error.diagnostics[1].toString(), 'My first description.');
    expect(error.diagnostics[2].toString(), '');
    expect(error.diagnostics[3].toString(), 'My second description.');

    expect(
      error.toStringDeep(),
      'FlutterError\n'
      '   My Error Summary.\n'
      '   My first description.\n'
      '\n'
      '   My second description.\n',
    );
    error = FlutterError('My Error Summary.');
    expect(error.diagnostics.length, 1);
    expect(error.diagnostics.first.level, DiagnosticLevel.summary);
    expect(error.diagnostics.first.toString(), 'My Error Summary.');

    expect(
      error.toStringDeep(),
      'FlutterError\n'
      '   My Error Summary.\n',
    );
  });

  test('Malformed FlutterError objects', () {
    {
      final AssertionError error;
      try {
        throw FlutterError.fromParts(<DiagnosticsNode>[]);
      } on AssertionError catch (e) {
        error = e;
      }
      expect(
        FlutterErrorDetails(exception: error).toString(),
        '══╡ EXCEPTION CAUGHT BY FLUTTER FRAMEWORK ╞══════════════════════\n'
        'The following assertion was thrown:\n'
        'Empty FlutterError\n'
        '═════════════════════════════════════════════════════════════════\n',
      );
    }

    {
      final AssertionError error;
      try {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorDescription('Error description without a summary'),
        ]);
      } on AssertionError catch (e) {
        error = e;
      }
      expect(
        FlutterErrorDetails(exception: error).toString(),
        '══╡ EXCEPTION CAUGHT BY FLUTTER FRAMEWORK ╞══════════════════════\n'
        'The following assertion was thrown:\n'
        'FlutterError is missing a summary.\n'
        'All FlutterError objects should start with a short (one line)\n'
        'summary description of the problem that was detected.\n'
        'Malformed FlutterError:\n'
        '  Error description without a summary\n'
        '\n'
        'This error should still help you solve your problem, however\n'
        'please also report this malformed error in the framework by\n'
        'filing a bug on GitHub:\n'
        '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml\n'
        '═════════════════════════════════════════════════════════════════\n',
      );
    }

    {
      final AssertionError error;
      try {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorSummary('Error Summary A'),
          ErrorDescription('Some descriptionA'),
          ErrorSummary('Error Summary B'),
          ErrorDescription('Some descriptionB'),
        ]);
      } on AssertionError catch (e) {
        error = e;
      }
      expect(
        FlutterErrorDetails(exception: error).toString(),
        '══╡ EXCEPTION CAUGHT BY FLUTTER FRAMEWORK ╞══════════════════════\n'
        'The following assertion was thrown:\n'
        'FlutterError contained multiple error summaries.\n'
        'All FlutterError objects should have only a single short (one\n'
        'line) summary description of the problem that was detected.\n'
        'Malformed FlutterError:\n'
        '  Error Summary A\n'
        '  Some descriptionA\n'
        '  Error Summary B\n'
        '  Some descriptionB\n'
        '\n'
        'The malformed error has 2 summaries.\n'
        'Summary 1: Error Summary A\n'
        'Summary 2: Error Summary B\n'
        '\n'
        'This error should still help you solve your problem, however\n'
        'please also report this malformed error in the framework by\n'
        'filing a bug on GitHub:\n'
        '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml\n'
        '═════════════════════════════════════════════════════════════════\n',
      );
    }

    {
      final AssertionError error;
      try {
        throw FlutterError.fromParts(<DiagnosticsNode>[
          ErrorDescription('Some description'),
          ErrorSummary('Error summary'),
        ]);
      } on AssertionError catch (e) {
        error = e;
      }
      expect(
        FlutterErrorDetails(exception: error).toString(),
        '══╡ EXCEPTION CAUGHT BY FLUTTER FRAMEWORK ╞══════════════════════\n'
        'The following assertion was thrown:\n'
        'FlutterError is missing a summary.\n'
        'All FlutterError objects should start with a short (one line)\n'
        'summary description of the problem that was detected.\n'
        'Malformed FlutterError:\n'
        '  Some description\n'
        '  Error summary\n'
        '\n'
        'This error should still help you solve your problem, however\n'
        'please also report this malformed error in the framework by\n'
        'filing a bug on GitHub:\n'
        '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml\n'
        '═════════════════════════════════════════════════════════════════\n',
      );
    }
  });

  test('User-thrown exceptions have ErrorSummary properties', () {
    {
      final DiagnosticsNode node;
      try {
        throw 'User thrown string';
      } catch (e) {
        node = FlutterErrorDetails(exception: e).toDiagnosticsNode();
      }
      final ErrorSummary summary = node.getProperties().whereType<ErrorSummary>().single;
      expect(summary.value, equals(<String>['User thrown string']));
    }

    {
      final DiagnosticsNode node;
      try {
        throw ArgumentError.notNull('myArgument');
      } catch (e) {
        node = FlutterErrorDetails(exception: e).toDiagnosticsNode();
      }
      final ErrorSummary summary = node.getProperties().whereType<ErrorSummary>().single;
      expect(summary.value, equals(<String>['Invalid argument(s) (myArgument): Must not be null']));
    }
  });

  test('Identifies user fault', () {
    // User fault because they called `new Text(null)` from their own code.
    final StackTrace stack = StackTrace.fromString('''
#0      _AssertionError._doThrowNew (dart:core-patch/errors_patch.dart:42:39)
#1      _AssertionError._throwNew (dart:core-patch/errors_patch.dart:38:5)
#2      new Text (package:flutter/src/widgets/text.dart:287:10)
#3      _MyHomePageState.build (package:hello_flutter/main.dart:72:16)
#4      StatefulElement.build (package:flutter/src/widgets/framework.dart:4414:27)
#5      ComponentElement.performRebuild (package:flutter/src/widgets/framework.dart:4303:15)
#6      Element.rebuild (package:flutter/src/widgets/framework.dart:4027:5)
#7      ComponentElement._firstBuild (package:flutter/src/widgets/framework.dart:4286:5)
#8      StatefulElement._firstBuild (package:flutter/src/widgets/framework.dart:4461:11)
#9      ComponentElement.mount (package:flutter/src/widgets/framework.dart:4281:5)
#10      Element.inflateWidget (package:flutter/src/widgets/framework.dart:3276:14)
#11     Element.updateChild (package:flutter/src/widgets/framework.dart:3070:12)
#12     SingleChildRenderObjectElement.mount (package:flutter/blah.dart:999:9)''');

    final FlutterErrorDetails details = FlutterErrorDetails(
      exception: AssertionError('Test assertion'),
      stack: stack,
    );

    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    details.debugFillProperties(builder);

    expect(builder.properties.length, 4);
    expect(builder.properties[0].toString(), 'The following assertion was thrown:');
    expect(builder.properties[1].toString(), contains('Assertion failed'));
    expect(builder.properties[2] is ErrorSpacer, true);
    final DiagnosticsStackTrace trace = builder.properties[3] as DiagnosticsStackTrace;
    expect(trace, isNotNull);
    expect(trace.value, stack);
  });

  test('Identifies our fault', () {
    // Our fault because we should either have an assertion in `text_helper.dart`
    // or we should make sure not to pass bad values into new Text.
    final StackTrace stack = StackTrace.fromString('''
#0      _AssertionError._doThrowNew (dart:core-patch/errors_patch.dart:42:39)
#1      _AssertionError._throwNew (dart:core-patch/errors_patch.dart:38:5)
#2      new Text (package:flutter/src/widgets/text.dart:287:10)
#3      new SomeWidgetUsingText (package:flutter/src/widgets/text_helper.dart:287:10)
#4      _MyHomePageState.build (package:hello_flutter/main.dart:72:16)
#5      StatefulElement.build (package:flutter/src/widgets/framework.dart:4414:27)
#6      ComponentElement.performRebuild (package:flutter/src/widgets/framework.dart:4303:15)
#7      Element.rebuild (package:flutter/src/widgets/framework.dart:4027:5)
#8      ComponentElement._firstBuild (package:flutter/src/widgets/framework.dart:4286:5)
#9      StatefulElement._firstBuild (package:flutter/src/widgets/framework.dart:4461:11)
#10     ComponentElement.mount (package:flutter/src/widgets/framework.dart:4281:5)
#11     Element.inflateWidget (package:flutter/src/widgets/framework.dart:3276:14)
#12     Element.updateChild (package:flutter/src/widgets/framework.dart:3070:12)
#13     SingleChildRenderObjectElement.mount (package:flutter/blah.dart:999:9)''');

    final FlutterErrorDetails details = FlutterErrorDetails(
      exception: AssertionError('Test assertion'),
      stack: stack,
    );

    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    details.debugFillProperties(builder);
    expect(builder.properties.length, 6);
    expect(builder.properties[0].toString(), 'The following assertion was thrown:');
    expect(builder.properties[1].toString(), contains('Assertion failed'));
    expect(builder.properties[2] is ErrorSpacer, true);
    expect(
      builder.properties[3].toString(),
      'Either the assertion indicates an error in the framework itself, or we should '
      'provide substantially more information in this error message to help you determine '
      'and fix the underlying cause.\n'
      'In either case, please report this assertion by filing a bug on GitHub:\n'
      '  https://github.com/flutter/flutter/issues/new?template=2_bug.yml',
    );
    expect(builder.properties[4] is ErrorSpacer, true);
    final DiagnosticsStackTrace trace = builder.properties[5] as DiagnosticsStackTrace;
    expect(trace, isNotNull);
    expect(trace.value, stack);
  });

  test('RepetitiveStackFrameFilter does not go out of range', () {
    const RepetitiveStackFrameFilter filter = RepetitiveStackFrameFilter(
      frames: <PartialStackFrame>[
        PartialStackFrame(
          className: 'TestClass',
          method: 'test1',
          package: 'package:test/blah.dart',
        ),
        PartialStackFrame(
          className: 'TestClass',
          method: 'test2',
          package: 'package:test/blah.dart',
        ),
        PartialStackFrame(
          className: 'TestClass',
          method: 'test3',
          package: 'package:test/blah.dart',
        ),
      ],
      replacement: 'test',
    );
    final List<String?> reasons = List<String?>.filled(2, null);
    filter.filter(const <StackFrame>[
      StackFrame(
        className: 'TestClass',
        method: 'test1',
        packageScheme: 'package',
        package: 'test',
        packagePath: 'blah.dart',
        line: 1,
        column: 1,
        number: 0,
        source: '',
      ),
      StackFrame(
        className: 'TestClass',
        method: 'test2',
        packageScheme: 'package',
        package: 'test',
        packagePath: 'blah.dart',
        line: 1,
        column: 1,
        number: 0,
        source: '',
      ),
    ], reasons);
    expect(reasons, List<String?>.filled(2, null));
  });
}
