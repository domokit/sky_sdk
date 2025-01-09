// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Parses line', () {
    expect(StackFrame.fromStackTraceLine(stackString.split('\n')[0]), stackFrames[0]);
  });

  test('Parses string', () {
    expect(StackFrame.fromStackString(stackString), stackFrames);
  });

  test('Parses StackTrace', () {
    expect(StackFrame.fromStackTrace(StackTrace.fromString(stackString)), stackFrames);
  });

  test('Parses complex stack', () {
    expect(StackFrame.fromStackString(asyncStackString), asyncStackFrames);
  });

  test('Parses stack without cols', () {
    expect(StackFrame.fromStackString(stackFrameNoCols), stackFrameNoColsFrames);
  });

  test(
    'Parses web stack',
    // Wasm stacks are not reliable, even in debug mode, so the stack
    // parser doesn't do the standard debug stack parsing this test is
    // expecting here.
    skip: kIsWasm, // [intended] See comments above
    () {
      expect(StackFrame.fromStackString(webStackTrace), webStackTraceFrames);
    },
  );

  test('Parses ...', () {
    expect(StackFrame.fromStackTraceLine('...'), StackFrame.stackOverFlowElision);
  });

  test('Live handling of Stack Overflows', () {
    void overflow(int seed) {
      overflow(seed + 1);
    }

    bool overflowed = false;
    try {
      overflow(1);
    } on StackOverflowError catch (e, stack) {
      overflowed = true;
      final List<StackFrame> frames = StackFrame.fromStackTrace(stack);
      expect(frames.contains(StackFrame.stackOverFlowElision), true);
    }
    expect(overflowed, true);
  }, skip: isBrowser); // [intended] The VM test harness can handle a
  // stack overflow, but the browser cannot - running this test in a browser
  // will cause it to become unresponsive.

  test('Traces from package:stack_trace throw assertion', () {
    try {
      StackFrame.fromStackString(mangledStackString);
      assert(false, 'StackFrame.fromStackString did not throw on a mangled stack trace');
    } catch (e) {
      expect(e, isA<AssertionError>());
      expect('$e', contains('Got a stack frame from package:stack_trace'));
    }
  });

  test('Can parse web constructor invocation with unknown class name', () {
    const String stackTraceLine =
        '#32     new (http://localhost:42191/dart-sdk/lib/async/stream_controller.dart:880:9)';
    expect(
      StackFrame.fromStackTraceLine(stackTraceLine),
      const StackFrame(
        number: 32,
        className: '<unknown>',
        method: '',
        packageScheme: 'http',
        package: '<unknown>',
        packagePath: 'dart-sdk/lib/async/stream_controller.dart',
        line: 880,
        column: 9,
        source: stackTraceLine,
      ),
    );
  });

  test('Parses to null for wrong format.', () {
    expect(StackFrame.fromStackTraceLine('wrong stack trace format'), null);
  });
}

const String stackString = '''
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
#10     Element.inflateWidget (package:flutter/src/widgets/framework.dart:3276:14)
#11     Element.updateChild (package:flutter/src/widgets/framework.dart)
#12     SingleChildRenderObjectElement.mount (package:flutter/blah.dart:999:9)
#13     main (package:hello_flutter/main.dart:10:4)''';

const List<StackFrame> stackFrames = <StackFrame>[
  StackFrame(
    number: 0,
    className: '_AssertionError',
    method: '_doThrowNew',
    packageScheme: 'dart',
    package: 'core-patch',
    packagePath: 'errors_patch.dart',
    line: 42,
    column: 39,
    source: '#0      _AssertionError._doThrowNew (dart:core-patch/errors_patch.dart:42:39)',
  ),
  StackFrame(
    number: 1,
    className: '_AssertionError',
    method: '_throwNew',
    packageScheme: 'dart',
    package: 'core-patch',
    packagePath: 'errors_patch.dart',
    line: 38,
    column: 5,
    source: '#1      _AssertionError._throwNew (dart:core-patch/errors_patch.dart:38:5)',
  ),
  StackFrame(
    number: 2,
    className: 'Text',
    method: '',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/text.dart',
    line: 287,
    column: 10,
    isConstructor: true,
    source: '#2      new Text (package:flutter/src/widgets/text.dart:287:10)',
  ),
  StackFrame(
    number: 3,
    className: '_MyHomePageState',
    method: 'build',
    packageScheme: 'package',
    package: 'hello_flutter',
    packagePath: 'main.dart',
    line: 72,
    column: 16,
    source: '#3      _MyHomePageState.build (package:hello_flutter/main.dart:72:16)',
  ),
  StackFrame(
    number: 4,
    className: 'StatefulElement',
    method: 'build',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: 4414,
    column: 27,
    source: '#4      StatefulElement.build (package:flutter/src/widgets/framework.dart:4414:27)',
  ),
  StackFrame(
    number: 5,
    className: 'ComponentElement',
    method: 'performRebuild',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: 4303,
    column: 15,
    source:
        '#5      ComponentElement.performRebuild (package:flutter/src/widgets/framework.dart:4303:15)',
  ),
  StackFrame(
    number: 6,
    className: 'Element',
    method: 'rebuild',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: 4027,
    column: 5,
    source: '#6      Element.rebuild (package:flutter/src/widgets/framework.dart:4027:5)',
  ),
  StackFrame(
    number: 7,
    className: 'ComponentElement',
    method: '_firstBuild',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: 4286,
    column: 5,
    source:
        '#7      ComponentElement._firstBuild (package:flutter/src/widgets/framework.dart:4286:5)',
  ),
  StackFrame(
    number: 8,
    className: 'StatefulElement',
    method: '_firstBuild',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: 4461,
    column: 11,
    source:
        '#8      StatefulElement._firstBuild (package:flutter/src/widgets/framework.dart:4461:11)',
  ),
  StackFrame(
    number: 9,
    className: 'ComponentElement',
    method: 'mount',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: 4281,
    column: 5,
    source: '#9      ComponentElement.mount (package:flutter/src/widgets/framework.dart:4281:5)',
  ),
  StackFrame(
    number: 10,
    className: 'Element',
    method: 'inflateWidget',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: 3276,
    column: 14,
    source: '#10     Element.inflateWidget (package:flutter/src/widgets/framework.dart:3276:14)',
  ),
  StackFrame(
    number: 11,
    className: 'Element',
    method: 'updateChild',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'src/widgets/framework.dart',
    line: -1,
    column: -1,
    source: '#11     Element.updateChild (package:flutter/src/widgets/framework.dart)',
  ),
  StackFrame(
    number: 12,
    className: 'SingleChildRenderObjectElement',
    method: 'mount',
    packageScheme: 'package',
    package: 'flutter',
    packagePath: 'blah.dart',
    line: 999,
    column: 9,
    source: '#12     SingleChildRenderObjectElement.mount (package:flutter/blah.dart:999:9)',
  ),
  StackFrame(
    number: 13,
    method: 'main',
    packageScheme: 'package',
    package: 'hello_flutter',
    packagePath: 'main.dart',
    line: 10,
    column: 4,
    source: '#13     main (package:hello_flutter/main.dart:10:4)',
  ),
];

const String asyncStackString = '''
#0      getSampleStack.<anonymous closure> (file:///path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart:40:57)
#1      new Future.sync (dart:async/future.dart:224:31)
#2      getSampleStack (file:///path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart:40:10)
#3      main (file:///path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart:46:40)
#4      main (package:flutter_goldens/flutter_goldens.dart:43:17)
<asynchronous suspension>
#5      main.<anonymous closure>.<anonymous closure> (file:///temp/path.whatever/listener.dart:47:18)
#6      _rootRun (dart:async/zone.dart:1126:13)
#7      _CustomZone.run (dart:async/zone.dart:1023:19)
#8      _runZoned (dart:async/zone.dart:1518:10)
#9      runZoned (dart:async/zone.dart:1465:12)
#10     Declarer.declare (package:test_api/src/backend/declarer.dart:130:22)
#11     RemoteListener.start.<anonymous closure>.<anonymous closure>.<anonymous closure> (package:test_api/src/remote_listener.dart:124:26)
<asynchronous suspension>
#12     _rootRun (dart:async/zone.dart:1126:13)
#13     _CustomZone.run (dart:async/zone.dart:1023:19)
#14     _runZoned (dart:async/zone.dart:1518:10)
#15     runZoned (dart:async/zone.dart:1502:12)
#16     RemoteListener.start.<anonymous closure>.<anonymous closure> (package:test_api/src/remote_listener.dart:70:9)
#17     _rootRun (dart:async/zone.dart:1126:13)
#18     _CustomZone.run (dart:async/zone.dart:1023:19)
#19     _runZoned (dart:async/zone.dart:1518:10)
#20     runZoned (dart:async/zone.dart:1465:12)
#21     StackTraceFormatter.asCurrent (package:test_api/src/backend/stack_trace_formatter.dart:41:31)
#22     RemoteListener.start.<anonymous closure> (package:test_api/src/remote_listener.dart:69:29)
#23     _rootRun (dart:async/zone.dart:1126:13)
#24     _CustomZone.run (dart:async/zone.dart:1023:19)
#25     _runZoned (dart:async/zone.dart:1518:10)
#26     runZoned (dart:async/zone.dart:1465:12)
#27     SuiteChannelManager.asCurrent (package:test_api/src/suite_channel_manager.dart:34:31)
#28     RemoteListener.start (package:test_api/src/remote_listener.dart:68:27)
#29     serializeSuite (file:///temp/path.whatever/listener.dart:17:25)
#30     main (file:///temp/path.whatever/listener.dart:43:36)
#31     _runMainZoned.<anonymous closure>.<anonymous closure> (dart:ui/hooks.dart:239:25)
#32     _rootRun (dart:async/zone.dart:1126:13)
#33     _CustomZone.run (dart:async/zone.dart:1023:19)
#34     _runZoned (dart:async/zone.dart:1518:10)
#35     runZoned (dart:async/zone.dart:1502:12)
#36     _runMainZoned.<anonymous closure> (dart:ui/hooks.dart:231:5)
#37     _startIsolate.<anonymous closure> (dart:isolate-patch/isolate_patch.dart:307:19)
#38     _RawReceivePortImpl._handleMessage (dart:isolate-patch/isolate_patch.dart:174:12)''';

const String mangledStackString = r'''
dart:async/future_impl.dart 23:44                              _Completer.completeError
test\bindings_async_gap_test.dart 42:17                        main.<fn>.<fn>
package:flutter_test/src/binding.dart 744:19                   TestWidgetsFlutterBinding._runTestBody
===== asynchronous gap ===========================
dart:async/zone.dart 1121:19                                   _CustomZone.registerUnaryCallback
dart:async-patch/async_patch.dart 83:23                        _asyncThenWrapperHelper
dart:async/zone.dart 1222:13                                   _rootRunBinary
dart:async/zone.dart 1107:19                                   _CustomZone.runBinary
package:flutter_test/src/binding.dart 724:14                   TestWidgetsFlutterBinding._runTest
package:flutter_test/src/binding.dart 1124:24                  AutomatedTestWidgetsFlutterBinding.runTest.<fn>
package:fake_async/fake_async.dart 177:54                      FakeAsync.run.<fn>.<fn>
dart:async/zone.dart 1190:13                                   _rootRun
''';

const List<StackFrame> asyncStackFrames = <StackFrame>[
  StackFrame(
    number: 0,
    method: 'getSampleStack',
    packageScheme: 'file',
    package: '<unknown>',
    packagePath: '/path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart',
    line: 40,
    column: 57,
    source:
        '#0      getSampleStack.<anonymous closure> (file:///path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart:40:57)',
  ),
  StackFrame(
    number: 1,
    className: 'Future',
    method: 'sync',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'future.dart',
    line: 224,
    column: 31,
    isConstructor: true,
    source: '#1      new Future.sync (dart:async/future.dart:224:31)',
  ),
  StackFrame(
    number: 2,
    method: 'getSampleStack',
    packageScheme: 'file',
    package: '<unknown>',
    packagePath: '/path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart',
    line: 40,
    column: 10,
    source:
        '#2      getSampleStack (file:///path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart:40:10)',
  ),
  StackFrame(
    number: 3,
    method: 'main',
    packageScheme: 'file',
    package: '<unknown>',
    packagePath: '/path/to/flutter/packages/flutter/foundation/error_reporting_test.dart',
    line: 46,
    column: 40,
    source:
        '#3      main (file:///path/to/flutter/packages/flutter/test/foundation/error_reporting_test.dart:46:40)',
  ),
  StackFrame(
    number: 4,
    method: 'main',
    packageScheme: 'package',
    package: 'flutter_goldens',
    packagePath: 'flutter_goldens.dart',
    line: 43,
    column: 17,
    source: '#4      main (package:flutter_goldens/flutter_goldens.dart:43:17)',
  ),
  StackFrame.asynchronousSuspension,
  StackFrame(
    number: 5,
    method: 'main',
    packageScheme: 'file',
    package: '<unknown>',
    packagePath: '/temp/path.whatever/listener.dart',
    line: 47,
    column: 18,
    source:
        '#5      main.<anonymous closure>.<anonymous closure> (file:///temp/path.whatever/listener.dart:47:18)',
  ),
  StackFrame(
    number: 6,
    method: '_rootRun',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1126,
    column: 13,
    source: '#6      _rootRun (dart:async/zone.dart:1126:13)',
  ),
  StackFrame(
    number: 7,
    className: '_CustomZone',
    method: 'run',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1023,
    column: 19,
    source: '#7      _CustomZone.run (dart:async/zone.dart:1023:19)',
  ),
  StackFrame(
    number: 8,
    method: '_runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1518,
    column: 10,
    source: '#8      _runZoned (dart:async/zone.dart:1518:10)',
  ),
  StackFrame(
    number: 9,
    method: 'runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1465,
    column: 12,
    source: '#9      runZoned (dart:async/zone.dart:1465:12)',
  ),
  StackFrame(
    number: 10,
    className: 'Declarer',
    method: 'declare',
    packageScheme: 'package',
    package: 'test_api',
    packagePath: 'src/backend/declarer.dart',
    line: 130,
    column: 22,
    source: '#10     Declarer.declare (package:test_api/src/backend/declarer.dart:130:22)',
  ),
  StackFrame(
    number: 11,
    className: 'RemoteListener',
    method: 'start',
    packageScheme: 'package',
    package: 'test_api',
    packagePath: 'src/remote_listener.dart',
    line: 124,
    column: 26,
    source:
        '#11     RemoteListener.start.<anonymous closure>.<anonymous closure>.<anonymous closure> (package:test_api/src/remote_listener.dart:124:26)',
  ),
  StackFrame.asynchronousSuspension,
  StackFrame(
    number: 12,
    method: '_rootRun',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1126,
    column: 13,
    source: '#12     _rootRun (dart:async/zone.dart:1126:13)',
  ),
  StackFrame(
    number: 13,
    className: '_CustomZone',
    method: 'run',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1023,
    column: 19,
    source: '#13     _CustomZone.run (dart:async/zone.dart:1023:19)',
  ),
  StackFrame(
    number: 14,
    method: '_runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1518,
    column: 10,
    source: '#14     _runZoned (dart:async/zone.dart:1518:10)',
  ),
  StackFrame(
    number: 15,
    method: 'runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1502,
    column: 12,
    source: '#15     runZoned (dart:async/zone.dart:1502:12)',
  ),
  StackFrame(
    number: 16,
    className: 'RemoteListener',
    method: 'start',
    packageScheme: 'package',
    package: 'test_api',
    packagePath: 'src/remote_listener.dart',
    line: 70,
    column: 9,
    source:
        '#16     RemoteListener.start.<anonymous closure>.<anonymous closure> (package:test_api/src/remote_listener.dart:70:9)',
  ),
  StackFrame(
    number: 17,
    method: '_rootRun',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1126,
    column: 13,
    source: '#17     _rootRun (dart:async/zone.dart:1126:13)',
  ),
  StackFrame(
    number: 18,
    className: '_CustomZone',
    method: 'run',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1023,
    column: 19,
    source: '#18     _CustomZone.run (dart:async/zone.dart:1023:19)',
  ),
  StackFrame(
    number: 19,
    method: '_runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1518,
    column: 10,
    source: '#19     _runZoned (dart:async/zone.dart:1518:10)',
  ),
  StackFrame(
    number: 20,
    method: 'runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1465,
    column: 12,
    source: '#20     runZoned (dart:async/zone.dart:1465:12)',
  ),
  StackFrame(
    number: 21,
    className: 'StackTraceFormatter',
    method: 'asCurrent',
    packageScheme: 'package',
    package: 'test_api',
    packagePath: 'src/backend/stack_trace_formatter.dart',
    line: 41,
    column: 31,
    source:
        '#21     StackTraceFormatter.asCurrent (package:test_api/src/backend/stack_trace_formatter.dart:41:31)',
  ),
  StackFrame(
    number: 22,
    className: 'RemoteListener',
    method: 'start',
    packageScheme: 'package',
    package: 'test_api',
    packagePath: 'src/remote_listener.dart',
    line: 69,
    column: 29,
    source:
        '#22     RemoteListener.start.<anonymous closure> (package:test_api/src/remote_listener.dart:69:29)',
  ),
  StackFrame(
    number: 23,
    method: '_rootRun',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1126,
    column: 13,
    source: '#23     _rootRun (dart:async/zone.dart:1126:13)',
  ),
  StackFrame(
    number: 24,
    className: '_CustomZone',
    method: 'run',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1023,
    column: 19,
    source: '#24     _CustomZone.run (dart:async/zone.dart:1023:19)',
  ),
  StackFrame(
    number: 25,
    method: '_runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1518,
    column: 10,
    source: '#25     _runZoned (dart:async/zone.dart:1518:10)',
  ),
  StackFrame(
    number: 26,
    method: 'runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1465,
    column: 12,
    source: '#26     runZoned (dart:async/zone.dart:1465:12)',
  ),
  StackFrame(
    number: 27,
    className: 'SuiteChannelManager',
    method: 'asCurrent',
    packageScheme: 'package',
    package: 'test_api',
    packagePath: 'src/suite_channel_manager.dart',
    line: 34,
    column: 31,
    source:
        '#27     SuiteChannelManager.asCurrent (package:test_api/src/suite_channel_manager.dart:34:31)',
  ),
  StackFrame(
    number: 28,
    className: 'RemoteListener',
    method: 'start',
    packageScheme: 'package',
    package: 'test_api',
    packagePath: 'src/remote_listener.dart',
    line: 68,
    column: 27,
    source: '#28     RemoteListener.start (package:test_api/src/remote_listener.dart:68:27)',
  ),
  StackFrame(
    number: 29,
    method: 'serializeSuite',
    packageScheme: 'file',
    package: '<unknown>',
    packagePath: '/temp/path.whatever/listener.dart',
    line: 17,
    column: 25,
    source: '#29     serializeSuite (file:///temp/path.whatever/listener.dart:17:25)',
  ),
  StackFrame(
    number: 30,
    method: 'main',
    packageScheme: 'file',
    package: '<unknown>',
    packagePath: '/temp/path.whatever/listener.dart',
    line: 43,
    column: 36,
    source: '#30     main (file:///temp/path.whatever/listener.dart:43:36)',
  ),
  StackFrame(
    number: 31,
    method: '_runMainZoned',
    packageScheme: 'dart',
    package: 'ui',
    packagePath: 'hooks.dart',
    line: 239,
    column: 25,
    source:
        '#31     _runMainZoned.<anonymous closure>.<anonymous closure> (dart:ui/hooks.dart:239:25)',
  ),
  StackFrame(
    number: 32,
    method: '_rootRun',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1126,
    column: 13,
    source: '#32     _rootRun (dart:async/zone.dart:1126:13)',
  ),
  StackFrame(
    number: 33,
    className: '_CustomZone',
    method: 'run',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1023,
    column: 19,
    source: '#33     _CustomZone.run (dart:async/zone.dart:1023:19)',
  ),
  StackFrame(
    number: 34,
    method: '_runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1518,
    column: 10,
    source: '#34     _runZoned (dart:async/zone.dart:1518:10)',
  ),
  StackFrame(
    number: 35,
    method: 'runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1502,
    column: 12,
    source: '#35     runZoned (dart:async/zone.dart:1502:12)',
  ),
  StackFrame(
    number: 36,
    method: '_runMainZoned',
    packageScheme: 'dart',
    package: 'ui',
    packagePath: 'hooks.dart',
    line: 231,
    column: 5,
    source: '#36     _runMainZoned.<anonymous closure> (dart:ui/hooks.dart:231:5)',
  ),
  StackFrame(
    number: 37,
    method: '_startIsolate',
    packageScheme: 'dart',
    package: 'isolate-patch',
    packagePath: 'isolate_patch.dart',
    line: 307,
    column: 19,
    source:
        '#37     _startIsolate.<anonymous closure> (dart:isolate-patch/isolate_patch.dart:307:19)',
  ),
  StackFrame(
    number: 38,
    className: '_RawReceivePortImpl',
    method: '_handleMessage',
    packageScheme: 'dart',
    package: 'isolate-patch',
    packagePath: 'isolate_patch.dart',
    line: 174,
    column: 12,
    source:
        '#38     _RawReceivePortImpl._handleMessage (dart:isolate-patch/isolate_patch.dart:174:12)',
  ),
];

const String stackFrameNoCols = '''
#0      blah (package:assertions/main.dart:4)
#1      main (package:assertions/main.dart:8)
#2      _runMainZoned.<anonymous closure>.<anonymous closure> (dart:ui/hooks.dart:239)
#3      _rootRun (dart:async/zone.dart:1126)
#4      _CustomZone.run (dart:async/zone.dart:1023)
#5      _runZoned (dart:async/zone.dart:1518)
#6      runZoned (dart:async/zone.dart:1502)
#7      _runMainZoned.<anonymous closure> (dart:ui/hooks.dart:231)
#8      _startIsolate.<anonymous closure> (dart:isolate-patch/isolate_patch.dart:307)
#9      _RawReceivePortImpl._handleMessage (dart:isolate-patch/isolate_patch.dart:174)''';

const List<StackFrame> stackFrameNoColsFrames = <StackFrame>[
  StackFrame(
    number: 0,
    method: 'blah',
    packageScheme: 'package',
    package: 'assertions',
    packagePath: 'main.dart',
    line: 4,
    column: -1,
    source: '#0      blah (package:assertions/main.dart:4)',
  ),
  StackFrame(
    number: 1,
    method: 'main',
    packageScheme: 'package',
    package: 'assertions',
    packagePath: 'main.dart',
    line: 8,
    column: -1,
    source: '#1      main (package:assertions/main.dart:8)',
  ),
  StackFrame(
    number: 2,
    method: '_runMainZoned',
    packageScheme: 'dart',
    package: 'ui',
    packagePath: 'hooks.dart',
    line: 239,
    column: -1,
    source:
        '#2      _runMainZoned.<anonymous closure>.<anonymous closure> (dart:ui/hooks.dart:239)',
  ),
  StackFrame(
    number: 3,
    method: '_rootRun',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1126,
    column: -1,
    source: '#3      _rootRun (dart:async/zone.dart:1126)',
  ),
  StackFrame(
    number: 4,
    className: '_CustomZone',
    method: 'run',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1023,
    column: -1,
    source: '#4      _CustomZone.run (dart:async/zone.dart:1023)',
  ),
  StackFrame(
    number: 5,
    method: '_runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1518,
    column: -1,
    source: '#5      _runZoned (dart:async/zone.dart:1518)',
  ),
  StackFrame(
    number: 6,
    method: 'runZoned',
    packageScheme: 'dart',
    package: 'async',
    packagePath: 'zone.dart',
    line: 1502,
    column: -1,
    source: '#6      runZoned (dart:async/zone.dart:1502)',
  ),
  StackFrame(
    number: 7,
    method: '_runMainZoned',
    packageScheme: 'dart',
    package: 'ui',
    packagePath: 'hooks.dart',
    line: 231,
    column: -1,
    source: '#7      _runMainZoned.<anonymous closure> (dart:ui/hooks.dart:231)',
  ),
  StackFrame(
    number: 8,
    method: '_startIsolate',
    packageScheme: 'dart',
    package: 'isolate-patch',
    packagePath: 'isolate-patch.dart',
    line: 307,
    column: -1,
    source: '#8      _startIsolate.<anonymous closure> (dart:isolate-patch/isolate_patch.dart:307)',
  ),
  StackFrame(
    number: 9,
    className: '_RawReceivePortImpl',
    method: '_handleMessage',
    packageScheme: 'dart',
    package: 'isolate-patch',
    packagePath: 'isolate-patch.dart',
    line: 174,
    column: -1,
    source:
        '#9      _RawReceivePortImpl._handleMessage (dart:isolate-patch/isolate_patch.dart:174)',
  ),
];

const String webStackTrace = r'''
package:dart-sdk/lib/_internal/js_dev_runtime/private/ddc_runtime/errors.dart 196:49  throw_
package:assertions/main.dart 4:3                                                      blah
package:assertions/main.dart 8:5                                                      main$
package:assertions/main_web_entrypoint.dart 9:3                                       main$
package:dart-sdk/lib/_internal/js_dev_runtime/patch/async_patch.dart 47:50            onValue
package:dart-sdk/lib/async/zone.dart 1381:54                                          runUnary
object_test.dart 210:5                                                                performLayout
package:dart-sdk/lib/async/future_impl.dart 140:18                                    handleValue
package:dart-sdk/lib/async/future_impl.dart 682:44                                    handleValueCallback
package:dart-sdk/lib/async/future_impl.dart 711:32                                    _propagateToListeners
package:dart-sdk/lib/async/future_impl.dart 391:9                                     callback
package:dart-sdk/lib/async/schedule_microtask.dart 43:11                              _microtaskLoop
package:dart-sdk/lib/async/schedule_microtask.dart 52:5                               _startMicrotaskLoop
package:dart-sdk/lib/_internal/js_dev_runtime/patch/async_patch.dart 168:15           <fn>''';

const List<StackFrame> webStackTraceFrames = <StackFrame>[
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'throw_',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/_internal/js_dev_runtime/private/ddc_runtime/errors.dart',
    line: 196,
    column: 49,
    source:
        'package:dart-sdk/lib/_internal/js_dev_runtime/private/ddc_runtime/errors.dart 196:49  throw_',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'blah',
    packageScheme: 'package',
    package: 'assertions',
    packagePath: 'main.dart',
    line: 4,
    column: 3,
    source:
        'package:assertions/main.dart 4:3                                                      blah',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: r'main$',
    packageScheme: 'package',
    package: 'assertions',
    packagePath: 'main.dart',
    line: 8,
    column: 5,
    source:
        r'package:assertions/main.dart 8:5                                                      main$',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: r'main$',
    packageScheme: 'package',
    package: 'assertions',
    packagePath: 'main_web_entrypoint.dart',
    line: 9,
    column: 3,
    source:
        r'package:assertions/main_web_entrypoint.dart 9:3                                       main$',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'onValue',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/_internal/js_dev_runtime/patch/async_patch.dart',
    line: 47,
    column: 50,
    source:
        'package:dart-sdk/lib/_internal/js_dev_runtime/patch/async_patch.dart 47:50            onValue',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'runUnary',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/async/zone.dart',
    line: 1381,
    column: 54,
    source:
        'package:dart-sdk/lib/async/zone.dart 1381:54                                          runUnary',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'performLayout',
    packageScheme: '<unknown>',
    package: '<unknown>',
    packagePath: '<unknown>',
    line: 210,
    column: 5,
    source:
        'object_test.dart 210:5                                                                performLayout',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'handleValue',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/async/future_impl.dart',
    line: 140,
    column: 18,
    source:
        'package:dart-sdk/lib/async/future_impl.dart 140:18                                    handleValue',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'handleValueCallback',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/async/future_impl.dart',
    line: 682,
    column: 44,
    source:
        'package:dart-sdk/lib/async/future_impl.dart 682:44                                    handleValueCallback',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: '_propagateToListeners',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/async/future_impl.dart',
    line: 711,
    column: 32,
    source:
        'package:dart-sdk/lib/async/future_impl.dart 711:32                                    _propagateToListeners',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: 'callback',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/async/future_impl.dart',
    line: 391,
    column: 9,
    source:
        'package:dart-sdk/lib/async/future_impl.dart 391:9                                     callback',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: '_microtaskLoop',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/async/schedule_microtask.dart',
    line: 43,
    column: 11,
    source:
        'package:dart-sdk/lib/async/schedule_microtask.dart 43:11                              _microtaskLoop',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: '_startMicrotaskLoop',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/async/schedule_microtask.dart',
    line: 52,
    column: 5,
    source:
        'package:dart-sdk/lib/async/schedule_microtask.dart 52:5                               _startMicrotaskLoop',
  ),
  StackFrame(
    number: -1,
    className: '<unknown>',
    method: '<fn>',
    packageScheme: 'package',
    package: 'dart-sdk',
    packagePath: 'lib/_internal/js_dev_runtime/patch/async_patch.dart',
    line: 168,
    column: 15,
    source:
        'package:dart-sdk/lib/_internal/js_dev_runtime/patch/async_patch.dart 168:15           <fn>',
  ),
];
