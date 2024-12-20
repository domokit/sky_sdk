// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:fake_async/fake_async.dart';
import 'package:flutter_driver/src/common/error.dart';
import 'package:flutter_driver/src/common/health.dart';
import 'package:flutter_driver/src/common/layer_tree.dart';
import 'package:flutter_driver/src/common/text_input_action.dart';
import 'package:flutter_driver/src/common/wait.dart';
import 'package:flutter_driver/src/driver/driver.dart';
import 'package:flutter_driver/src/driver/timeline.dart';
import 'package:vm_service/vm_service.dart' as vms;

import '../../common.dart';

/// Magical timeout value that's different from the default.
const Duration _kTestTimeout = Duration(milliseconds: 1234);
const String _kSerializedTestTimeout = '1234';
const String _kWebScriptPrefix = r"window.$flutterDriver('";
const String _kWebScriptSuffix = "')";

void main() {
  final List<String> log = <String>[];

  driverLog = (String source, String message) {
    log.add('$source: $message');
  };

  group('VMServiceFlutterDriver with logCommunicationToFile', () {
    late FakeVmService fakeClient;
    late FakeVM fakeVM;
    late vms.Isolate fakeIsolate;
    late VMServiceFlutterDriver driver;
    late File logFile;

    setUp(() {
      fakeIsolate = createFakeIsolate();
      fakeVM = FakeVM(fakeIsolate);
      fakeClient = FakeVmService(fakeVM);
      fakeClient.responses['waitFor'] = makeFakeResponse(<String, dynamic>{'status': 'ok'});
    });

    tearDown(() {
      if (logFile.existsSync()) {
        logFile.deleteSync();
      }
    });

    group('logCommunicationToFile', () {
      test('logCommunicationToFile = true', () async {
        driver = VMServiceFlutterDriver.connectedTo(fakeClient, fakeIsolate);
        logFile = File(driver.logFilePathName);

        await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);

        final bool exists = logFile.existsSync();
        expect(exists, true, reason: 'Not found ${logFile.path}');

        final String commandLog = await logFile.readAsString();
        const String waitForCommandLog =
            '>>> {command: waitFor, timeout: $_kSerializedTestTimeout, finderType: ByTooltipMessage, text: foo}';
        const String responseLog = '<<< {isError: false, response: {status: ok}}';

        expect(
          commandLog.contains(waitForCommandLog),
          true,
          reason: '$commandLog not contains $waitForCommandLog',
        );
        expect(
          commandLog.contains(responseLog),
          true,
          reason: '$commandLog not contains $responseLog',
        );
      });

      test('logCommunicationToFile = false', () async {
        driver = VMServiceFlutterDriver.connectedTo(
          fakeClient,
          fakeIsolate,
          logCommunicationToFile: false,
        );
        logFile = File(driver.logFilePathName);
        // clear log file if left in filetree from previous run
        if (logFile.existsSync()) {
          logFile.deleteSync();
        }
        await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);

        final bool exists = logFile.existsSync();
        expect(exists, false, reason: 'because ${logFile.path} exists');
      });

      test('logFilePathName was set when a new driver was created', () {
        driver = VMServiceFlutterDriver.connectedTo(fakeClient, fakeIsolate);
        logFile = File(driver.logFilePathName);
        expect(logFile.path, endsWith('.log'));
      });
    });
  });

  group('VMServiceFlutterDriver with printCommunication', () {
    late FakeVmService fakeClient;
    late FakeVM fakeVM;
    late vms.Isolate fakeIsolate;
    late VMServiceFlutterDriver driver;

    setUp(() async {
      log.clear();
      fakeIsolate = createFakeIsolate();
      fakeVM = FakeVM(fakeIsolate);
      fakeClient = FakeVmService(fakeVM);
      fakeClient.responses['waitFor'] = makeFakeResponse(<String, dynamic>{'status': 'ok'});
    });

    test('printCommunication = true', () async {
      driver = VMServiceFlutterDriver.connectedTo(
        fakeClient,
        fakeIsolate,
        printCommunication: true,
      );
      await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);
      expect(log, <String>[
        'VMServiceFlutterDriver: >>> {command: waitFor, timeout: $_kSerializedTestTimeout, finderType: ByTooltipMessage, text: foo}',
        'VMServiceFlutterDriver: <<< {isError: false, response: {status: ok}}',
      ]);
    });

    test('printCommunication = false', () async {
      driver = VMServiceFlutterDriver.connectedTo(fakeClient, fakeIsolate);
      await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);
      expect(log, <String>[]);
    });
  });

  group('VMServiceFlutterDriver.connect', () {
    late FakeVmService fakeClient;
    late FakeVM fakeVM;
    late vms.Isolate fakeIsolate;

    void expectLogContains(String message) {
      expect(log, anyElement(contains(message)));
    }

    setUp(() {
      log.clear();
      fakeIsolate = createFakeIsolate();
      fakeVM = FakeVM(fakeIsolate);
      fakeClient = FakeVmService(fakeVM);
      vmServiceConnectFunction = (String url, Map<String, dynamic>? headers) async {
        return fakeClient;
      };
      fakeClient.responses['get_health'] = makeFakeResponse(<String, dynamic>{'status': 'ok'});
    });

    tearDown(() async {
      restoreVmServiceConnectFunction();
    });

    test('Retries while Dart VM service is not available', () async {
      // This test case will test the real implementation of `_waitAndConnect`.
      restoreVmServiceConnectFunction();

      // The actual behavior is to retry indefinitely until the Dart VM service
      // becomes available. `.timeout` is used here to exit the infinite loop,
      // expecting that no other types of error are thrown during the process.
      expect(
        vmServiceConnectFunction(
          'http://foo.bar',
          <String, dynamic>{},
        ).timeout(const Duration(seconds: 1)),
        throwsA(isA<TimeoutException>()),
      );
    });

    test('throws after retries if no isolate', () async {
      fakeVM.numberOfTriesBeforeResolvingIsolate = 10000;
      FakeAsync().run((FakeAsync time) {
        FlutterDriver.connect(dartVmServiceUrl: '');
        time.elapse(kUnusuallyLongTimeout);
      });
      expect(log, <String>[
        'VMServiceFlutterDriver: Connecting to Flutter application at ',
        'VMServiceFlutterDriver: The root isolate is taking an unusually long time to start.',
      ]);
    });

    test('Retries connections if isolate is not available', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kPauseStart, timestamp: 0);
      fakeVM.numberOfTriesBeforeResolvingIsolate = 5;
      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expect(fakeClient.connectionLog, <String>[
        'getIsolate',
        'setFlag pause_isolates_on_start false',
        'resume',
        'streamListen Isolate',
        'getIsolate',
        'onIsolateEvent',
        'streamCancel Isolate',
      ]);
    });

    test('Refreshes isolate if it is not started for long time', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kNone, timestamp: 0);
      fakeClient.onGetIsolate = changeIsolateEventAfter(
        5,
        vms.Event(kind: vms.EventKind.kPauseStart, timestamp: 1),
      );

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expect(fakeClient.connectionLog, <String>[
        'getIsolate',
        'getIsolate',
        'getIsolate',
        'getIsolate',
        'getIsolate',
        'setFlag pause_isolates_on_start false',
        'resume',
        'streamListen Isolate',
        'getIsolate',
        'onIsolateEvent',
        'streamCancel Isolate',
      ]);
    });

    test('Connects to isolate number', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kPauseStart, timestamp: 0);
      final FlutterDriver driver = await FlutterDriver.connect(
        dartVmServiceUrl: '',
        isolateNumber: int.parse(fakeIsolate.number!),
      );
      expect(driver, isNotNull);
      expect(fakeClient.connectionLog, <String>[
        'getIsolate',
        'setFlag pause_isolates_on_start false',
        'resume',
        'streamListen Isolate',
        'getIsolate',
        'onIsolateEvent',
        'streamCancel Isolate',
      ]);
    });

    test('connects to isolate paused at start', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kPauseStart, timestamp: 0);

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expectLogContains('Isolate is paused at start');
      expect(fakeClient.connectionLog, <String>[
        'getIsolate',
        'setFlag pause_isolates_on_start false',
        'resume',
        'streamListen Isolate',
        'getIsolate',
        'onIsolateEvent',
        'streamCancel Isolate',
      ]);
    });

    test('ignores setFlag failure', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kPauseStart, timestamp: 0);
      fakeClient.failOnSetFlag = true;

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expectLogContains(
        'Failed to set pause_isolates_on_start=false, proceeding. '
        'Error: Exception: setFlag failed',
      );
      expect(driver, isNotNull);
    });

    test('connects to isolate paused mid-flight', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kPauseBreakpoint, timestamp: 0);

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expectLogContains('Isolate is paused mid-flight');
    });

    test('connects to isolate paused mid-flight after request', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kPausePostRequest, timestamp: 0);

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expectLogContains('Isolate is paused mid-flight');
    });

    // This test simulates a situation when we believe that the isolate is
    // currently paused, but something else (e.g. a debugger) resumes it before
    // we do. There's no need to fail as we should be able to drive the app
    // just fine.
    test('connects despite losing the race to resume isolate', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kPauseBreakpoint, timestamp: 0);
      fakeClient.failOnResumeWith101 = true;

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expectLogContains('Attempted to resume an already resumed isolate');
    });

    test('connects to unpaused isolate', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kResume, timestamp: 0);

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expectLogContains('Isolate is not paused. Assuming application is ready.');
    });

    test('connects to unpaused when onExtensionAdded does not contain the '
        'driver extension', () async {
      fakeIsolate.pauseEvent = vms.Event(kind: vms.EventKind.kResume, timestamp: 0);
      fakeIsolate.extensionRPCs!.add('ext.flutter.driver');

      final FlutterDriver driver = await FlutterDriver.connect(dartVmServiceUrl: '');
      expect(driver, isNotNull);
      expectLogContains('Isolate is not paused. Assuming application is ready.');
    });
  });

  group('VMServiceFlutterDriver', () {
    late FakeVmService fakeClient;
    late FakeVM fakeVM;
    late vms.Isolate fakeIsolate;
    late VMServiceFlutterDriver driver;

    setUp(() {
      fakeIsolate = createFakeIsolate();
      fakeVM = FakeVM(fakeIsolate);
      fakeClient = FakeVmService(fakeVM);
      driver = VMServiceFlutterDriver.connectedTo(fakeClient, fakeIsolate);
      fakeClient.responses['tap'] = makeFakeResponse(<String, dynamic>{});
    });

    test('checks the health of the driver extension', () async {
      fakeClient.responses['get_health'] = makeFakeResponse(<String, dynamic>{'status': 'ok'});
      final Health result = await driver.checkHealth();
      expect(result.status, HealthStatus.ok);
    });

    test('closes connection', () async {
      await driver.close();
      expect(fakeClient.connectionLog.last, 'dispose');
    });

    group('ByValueKey', () {
      test('restricts value types', () async {
        expect(() => find.byValueKey(null), throwsDriverError);
      });

      test('finds by ValueKey', () async {
        await driver.tap(find.byValueKey('foo'), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: tap, timeout: $_kSerializedTestTimeout, finderType: ByValueKey, keyValueString: foo, keyValueType: String}',
        ]);
      });
    });

    group('BySemanticsLabel', () {
      test('finds by Semantic label using String', () async {
        await driver.tap(find.bySemanticsLabel('foo'), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: tap, timeout: $_kSerializedTestTimeout, finderType: BySemanticsLabel, label: foo}',
        ]);
      });

      test('finds by Semantic label using RegExp', () async {
        await driver.tap(find.bySemanticsLabel(RegExp('^foo')), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: tap, timeout: $_kSerializedTestTimeout, finderType: BySemanticsLabel, label: ^foo, isRegExp: true}',
        ]);
      });
    });

    group('tap', () {
      test('sends the tap command', () async {
        await driver.tap(find.text('foo'), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: tap, timeout: $_kSerializedTestTimeout, finderType: ByText, text: foo}',
        ]);
      });
    });

    group('getText', () {
      test('sends the getText command', () async {
        fakeClient.responses['get_text'] = makeFakeResponse(<String, dynamic>{'text': 'hello'});
        final String result = await driver.getText(find.byValueKey(123), timeout: _kTestTimeout);
        expect(result, 'hello');
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_text, timeout: $_kSerializedTestTimeout, finderType: ByValueKey, keyValueString: 123, keyValueType: int}',
        ]);
      });
    });

    group('sendTextInputAction', () {
      test('sends the SendTextInputAction command with action done', () async {
        fakeClient.responses['send_text_input_action'] = makeFakeResponse(<String, dynamic>{});
        await driver.sendTextInputAction(TextInputAction.done, timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: send_text_input_action, timeout: $_kSerializedTestTimeout, action: done}',
        ]);
      });
    });

    group('getLayerTree', () {
      test('sends the getLayerTree command', () async {
        fakeClient.responses['get_layer_tree'] = makeFakeResponse(<String, String>{
          'tree': 'hello',
        });
        final LayerTree result = await driver.getLayerTree(timeout: _kTestTimeout);
        final LayerTree referenceTree = LayerTree.fromJson(<String, String>{'tree': 'hello'});
        expect(result.tree, referenceTree.tree);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_layer_tree, timeout: $_kSerializedTestTimeout}',
        ]);
      });
    });

    group('waitFor', () {
      test('sends the waitFor command', () async {
        fakeClient.responses['waitFor'] = makeFakeResponse(<String, dynamic>{});
        await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: waitFor, timeout: $_kSerializedTestTimeout, finderType: ByTooltipMessage, text: foo}',
        ]);
      });
    });

    group('getWidgetDiagnostics', () {
      test('sends the getWidgetDiagnostics command', () async {
        fakeClient.responses['get_diagnostics_tree'] = makeFakeResponse(<String, dynamic>{});
        await driver.getWidgetDiagnostics(find.byTooltip('foo'), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_diagnostics_tree, timeout: $_kSerializedTestTimeout, finderType: ByTooltipMessage, text: foo, subtreeDepth: 0, includeProperties: true, diagnosticsType: widget}',
        ]);
      });
    });

    group('getRenderObjectDiagnostics', () {
      test('sends the getRenderObjectDiagnostics command', () async {
        fakeClient.responses['get_diagnostics_tree'] = makeFakeResponse(<String, dynamic>{});
        await driver.getRenderObjectDiagnostics(find.byTooltip('foo'), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_diagnostics_tree, timeout: $_kSerializedTestTimeout, finderType: ByTooltipMessage, text: foo, subtreeDepth: 0, includeProperties: true, diagnosticsType: renderObject}',
        ]);
      });
    });

    group('waitForCondition', () {
      test('sends the wait for NoPendingFrameCondition command', () async {
        fakeClient.responses['waitForCondition'] = makeFakeResponse(<String, dynamic>{});
        await driver.waitForCondition(const NoPendingFrame(), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: waitForCondition, timeout: $_kSerializedTestTimeout, conditionName: NoPendingFrameCondition}',
        ]);
      });

      test('sends the wait for NoPendingPlatformMessages command', () async {
        fakeClient.responses['waitForCondition'] = makeFakeResponse(<String, dynamic>{});
        await driver.waitForCondition(const NoPendingPlatformMessages(), timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: waitForCondition, timeout: $_kSerializedTestTimeout, conditionName: NoPendingPlatformMessagesCondition}',
        ]);
      });

      test('sends the waitForCondition of combined conditions command', () async {
        fakeClient.responses['waitForCondition'] = makeFakeResponse(<String, dynamic>{});
        const SerializableWaitCondition combinedCondition = CombinedCondition(
          <SerializableWaitCondition>[NoPendingFrame(), NoTransientCallbacks()],
        );
        await driver.waitForCondition(combinedCondition, timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: waitForCondition, timeout: $_kSerializedTestTimeout, conditionName: CombinedCondition, conditions: [{"conditionName":"NoPendingFrameCondition"},{"conditionName":"NoTransientCallbacksCondition"}]}',
        ]);
      });
    });

    group('waitUntilNoTransientCallbacks', () {
      test('sends the waitUntilNoTransientCallbacks command', () async {
        fakeClient.responses['waitForCondition'] = makeFakeResponse(<String, dynamic>{});
        await driver.waitUntilNoTransientCallbacks(timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: waitForCondition, timeout: $_kSerializedTestTimeout, conditionName: NoTransientCallbacksCondition}',
        ]);
      });
    });

    group('waitUntilFirstFrameRasterized', () {
      test('sends the waitUntilFirstFrameRasterized command', () async {
        fakeClient.responses['waitForCondition'] = makeFakeResponse(<String, dynamic>{});
        await driver.waitUntilFirstFrameRasterized();
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: waitForCondition, conditionName: FirstFrameRasterizedCondition}',
        ]);
      });
    });

    group('getOffset', () {
      setUp(() {
        fakeClient.responses['get_offset'] = makeFakeResponse(<String, double>{'dx': 11, 'dy': 12});
      });

      test('sends the getCenter command', () async {
        final DriverOffset result = await driver.getCenter(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_offset, timeout: 1234, finderType: ByValueKey, keyValueString: 123, keyValueType: int, offsetType: center}',
        ]);
      });

      test('sends the getTopLeft command', () async {
        final DriverOffset result = await driver.getTopLeft(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_offset, timeout: 1234, finderType: ByValueKey, keyValueString: 123, keyValueType: int, offsetType: topLeft}',
        ]);
      });

      test('sends the getTopRight command', () async {
        final DriverOffset result = await driver.getTopRight(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_offset, timeout: 1234, finderType: ByValueKey, keyValueString: 123, keyValueType: int, offsetType: topRight}',
        ]);
      });

      test('sends the getBottomLeft command', () async {
        final DriverOffset result = await driver.getBottomLeft(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_offset, timeout: 1234, finderType: ByValueKey, keyValueString: 123, keyValueType: int, offsetType: bottomLeft}',
        ]);
      });

      test('sends the getBottomRight command', () async {
        final DriverOffset result = await driver.getBottomRight(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: get_offset, timeout: 1234, finderType: ByValueKey, keyValueString: 123, keyValueType: int, offsetType: bottomRight}',
        ]);
      });
    });

    group('clearTimeline', () {
      test('clears timeline', () async {
        await driver.clearTimeline();
        expect(fakeClient.connectionLog, contains('clearVMTimeline'));
      });
    });

    group('traceAction', () {
      test('without clearing timeline', () async {
        final Timeline timeline = await driver.traceAction(() async {
          fakeClient.connectionLog.add('action');
        }, retainPriorEvents: true);

        expect(fakeClient.connectionLog, const <String>[
          'setVMTimelineFlags [all]',
          'action',
          'getFlagList',
          'setVMTimelineFlags []',
          'getVMTimeline null null',
        ]);
        expect(timeline.events!.single.name, 'test event');
      });

      test('with clearing timeline', () async {
        final Timeline timeline = await driver.traceAction(() async {
          fakeClient.connectionLog.add('action');
        });

        expect(fakeClient.connectionLog, const <String>[
          'clearVMTimeline',
          'getVMTimelineMicros',
          'setVMTimelineFlags [all]',
          'action',
          'getVMTimelineMicros',
          'getFlagList',
          'setVMTimelineFlags []',
          'getVMTimeline 1 999999',
        ]);
        expect(timeline.events!.single.name, 'test event');
      });

      test('with time interval', () async {
        fakeClient.incrementMicros = true;
        fakeClient.timelineResponses[1000001] = vms.Timeline.parse(<String, dynamic>{
          'traceEvents': <dynamic>[
            <String, dynamic>{'name': 'test event 2'},
          ],
          'timeOriginMicros': 1000000,
          'timeExtentMicros': 999999,
        });
        final Timeline timeline = await driver.traceAction(() async {
          fakeClient.connectionLog.add('action');
        });

        expect(fakeClient.connectionLog, const <String>[
          'clearVMTimeline',
          'getVMTimelineMicros',
          'setVMTimelineFlags [all]',
          'action',
          'getVMTimelineMicros',
          'getFlagList',
          'setVMTimelineFlags []',
          'getVMTimeline 1 999999',
          'getVMTimeline 1000001 999999',
        ]);
        expect(timeline.events!.map((TimelineEvent event) => event.name), <String>[
          'test event',
          'test event 2',
        ]);
      });
    });

    group('traceAction with timeline streams', () {
      test('specify non-default timeline streams', () async {
        bool actionCalled = false;

        final Timeline timeline = await driver.traceAction(
          () async {
            actionCalled = true;
          },
          streams: const <TimelineStream>[
            TimelineStream.dart,
            TimelineStream.gc,
            TimelineStream.compiler,
          ],
          retainPriorEvents: true,
        );

        expect(actionCalled, isTrue);
        expect(fakeClient.connectionLog, <String>[
          'setVMTimelineFlags [Dart, GC, Compiler]',
          'getFlagList',
          'setVMTimelineFlags []',
          'getVMTimeline null null',
        ]);

        expect(timeline.events!.single.name, 'test event');
      });
    });

    group('sendCommand error conditions', () {
      test('local default timeout', () async {
        log.clear();
        fakeClient.artificialExtensionDelay = Completer<void>().future;
        FakeAsync().run((FakeAsync time) {
          driver.waitFor(find.byTooltip('foo'));
          expect(log, <String>[]);
          time.elapse(kUnusuallyLongTimeout);
        });
        expect(log, <String>[
          'VMServiceFlutterDriver: waitFor message is taking a long time to complete...',
        ]);
      });

      test('local custom timeout', () async {
        log.clear();
        fakeClient.artificialExtensionDelay = Completer<void>().future;
        FakeAsync().run((FakeAsync time) {
          final Duration customTimeout = kUnusuallyLongTimeout - const Duration(seconds: 1);
          driver.waitFor(find.byTooltip('foo'), timeout: customTimeout);
          expect(log, <String>[]);
          time.elapse(customTimeout);
        });
        expect(log, <String>[
          'VMServiceFlutterDriver: waitFor message is taking a long time to complete...',
        ]);
      });

      test('remote error', () async {
        fakeClient.responses['waitFor'] = makeFakeResponse(<String, dynamic>{
          'message': 'This is a failure',
        }, isError: true);
        await expectLater(
          () => driver.waitFor(find.byTooltip('foo')),
          throwsA(
            isA<DriverError>().having(
              (DriverError error) => error.message,
              'message',
              'Error in Flutter application: {message: This is a failure}',
            ),
          ),
        );
      });

      test('uncaught remote error', () async {
        fakeClient.artificialExtensionDelay = Future<void>.error(
          vms.RPCError('callServiceExtension', 9999, 'test error'),
        );

        expect(driver.waitFor(find.byTooltip('foo')), throwsDriverError);
      });
    });

    group('setSemantics', () {
      test('can be enabled', () async {
        fakeClient.responses['set_semantics'] = makeFakeResponse(<String, Object>{
          'changedState': true,
        });
        await driver.setSemantics(true, timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: set_semantics, timeout: $_kSerializedTestTimeout, enabled: true}',
        ]);
      });

      test('can be disabled', () async {
        fakeClient.responses['set_semantics'] = makeFakeResponse(<String, Object>{
          'changedState': false,
        });
        await driver.setSemantics(false, timeout: _kTestTimeout);
        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: set_semantics, timeout: $_kSerializedTestTimeout, enabled: false}',
        ]);
      });
    });

    test('VMServiceFlutterDriver does not support webDriver', () async {
      expect(() => driver.webDriver, throwsUnsupportedError);
    });

    group('runUnsynchronized', () {
      test('wrap waitFor with runUnsynchronized', () async {
        fakeClient.responses['waitFor'] = makeFakeResponse(<String, dynamic>{});
        fakeClient.responses['set_frame_sync'] = makeFakeResponse(<String, dynamic>{});

        await driver.runUnsynchronized(() async {
          await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);
        });

        expect(fakeClient.commandLog, <String>[
          'ext.flutter.driver {command: set_frame_sync, enabled: false}',
          'ext.flutter.driver {command: waitFor, timeout: $_kSerializedTestTimeout, finderType: ByTooltipMessage, text: foo}',
          'ext.flutter.driver {command: set_frame_sync, enabled: true}',
        ]);
      });
    });
  });

  group('VMServiceFlutterDriver with custom timeout', () {
    late FakeVmService fakeClient;
    late FakeVM fakeVM;
    late vms.Isolate fakeIsolate;
    late VMServiceFlutterDriver driver;

    setUp(() {
      fakeIsolate = createFakeIsolate();
      fakeVM = FakeVM(fakeIsolate);
      fakeClient = FakeVmService(fakeVM);
      driver = VMServiceFlutterDriver.connectedTo(fakeClient, fakeIsolate);
      fakeClient.responses['get_health'] = makeFakeResponse(<String, dynamic>{'status': 'ok'});
    });

    test('GetHealth has no default timeout', () async {
      await driver.checkHealth();
      expect(fakeClient.commandLog, <String>['ext.flutter.driver {command: get_health}']);
    });

    test('does not interfere with explicit timeouts', () async {
      await driver.checkHealth(timeout: _kTestTimeout);
      expect(fakeClient.commandLog, <String>[
        'ext.flutter.driver {command: get_health, timeout: $_kSerializedTestTimeout}',
      ]);
    });
  });

  group('WebFlutterDriver with logCommunicationToFile', () {
    late FakeFlutterWebConnection fakeConnection;
    late WebFlutterDriver driver;
    late File logFile;

    setUp(() {
      fakeConnection = FakeFlutterWebConnection();
      fakeConnection.supportsTimelineAction = true;
      fakeConnection.responses['waitFor'] = jsonEncode(
        makeFakeResponse(<String, dynamic>{'status': 'ok'}),
      );
    });

    tearDown(() {
      if (logFile.existsSync()) {
        logFile.deleteSync();
      }
    });

    test('logCommunicationToFile = true', () async {
      driver = WebFlutterDriver.connectedTo(fakeConnection);
      logFile = File(driver.logFilePathName);
      await driver.waitFor(find.byTooltip('logCommunicationToFile test'), timeout: _kTestTimeout);

      final bool exists = logFile.existsSync();
      expect(exists, true, reason: 'Not found ${logFile.path}');

      final String commandLog = await logFile.readAsString();
      const String waitForCommandLog =
          '>>> {command: waitFor, timeout: 1234, finderType: ByTooltipMessage, text: logCommunicationToFile test}';
      const String responseLog = '<<< {isError: false, response: {status: ok}, type: Response}';

      expect(
        commandLog,
        contains(waitForCommandLog),
        reason: '$commandLog not contains $waitForCommandLog',
      );
      expect(commandLog, contains(responseLog), reason: '$commandLog not contains $responseLog');
    });

    test('logCommunicationToFile = false', () async {
      driver = WebFlutterDriver.connectedTo(fakeConnection, logCommunicationToFile: false);
      logFile = File(driver.logFilePathName);
      // clear log file if left in filetree from previous run
      if (logFile.existsSync()) {
        logFile.deleteSync();
      }
      await driver.waitFor(find.byTooltip('logCommunicationToFile test'), timeout: _kTestTimeout);
      final bool exists = logFile.existsSync();
      expect(exists, false, reason: 'because ${logFile.path} exists');
    });
  });

  group('WebFlutterDriver with printCommunication', () {
    late FakeFlutterWebConnection fakeConnection;
    late WebFlutterDriver driver;

    setUp(() {
      log.clear();
      fakeConnection = FakeFlutterWebConnection();
      fakeConnection.supportsTimelineAction = true;
      fakeConnection.responses['waitFor'] = jsonEncode(
        makeFakeResponse(<String, dynamic>{'status': 'ok'}),
      );
    });

    test('printCommunication = true', () async {
      driver = WebFlutterDriver.connectedTo(fakeConnection, printCommunication: true);
      await driver.waitFor(find.byTooltip('printCommunication test'), timeout: _kTestTimeout);
      expect(log, <String>[
        'WebFlutterDriver: >>> {command: waitFor, timeout: 1234, finderType: ByTooltipMessage, text: printCommunication test}',
        'WebFlutterDriver: <<< {isError: false, response: {status: ok}, type: Response}',
      ]);
    });

    test('printCommunication = false', () async {
      driver = WebFlutterDriver.connectedTo(fakeConnection);
      await driver.waitFor(find.byTooltip('printCommunication test'), timeout: _kTestTimeout);
      expect(log, <String>[]);
    });
  });

  group('WebFlutterDriver', () {
    late FakeFlutterWebConnection fakeConnection;
    late WebFlutterDriver driver;

    setUp(() {
      fakeConnection = FakeFlutterWebConnection();
      fakeConnection.supportsTimelineAction = true;
      driver = WebFlutterDriver.connectedTo(fakeConnection);
    });

    test('closes connection', () async {
      await driver.close();
    });

    group('ByValueKey', () {
      test('restricts value types', () async {
        expect(() => find.byValueKey(null), throwsDriverError);
      });

      test('finds by ValueKey', () async {
        fakeConnection.responses['tap'] = jsonEncode(makeFakeResponse(<String, dynamic>{}));
        await driver.tap(find.byValueKey('foo'), timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"tap","timeout":"1234","finderType":"ByValueKey","keyValueString":"foo","keyValueType":"String"}') 0:00:01.234000''',
        ]);
      });
    });

    group('BySemanticsLabel', () {
      test('finds by Semantic label using String', () async {
        fakeConnection.responses['tap'] = jsonEncode(makeFakeResponse(<String, dynamic>{}));
        await driver.tap(find.bySemanticsLabel('foo'), timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"tap","timeout":"1234","finderType":"BySemanticsLabel","label":"foo"}') 0:00:01.234000''',
        ]);
      });

      test('finds by Semantic label using RegExp', () async {
        fakeConnection.responses['tap'] = jsonEncode(makeFakeResponse(<String, dynamic>{}));
        await driver.tap(find.bySemanticsLabel(RegExp('^foo')), timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"tap","timeout":"1234","finderType":"BySemanticsLabel","label":"^foo","isRegExp":"true"}') 0:00:01.234000''',
        ]);
      });
    });

    group('tap', () {
      test('sends the tap command', () async {
        fakeConnection.responses['tap'] = jsonEncode(makeFakeResponse(<String, dynamic>{}));
        await driver.tap(find.text('foo'), timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"tap","timeout":"1234","finderType":"ByText","text":"foo"}') 0:00:01.234000''',
        ]);
      });
    });

    group('getText', () {
      test('sends the getText command', () async {
        fakeConnection.responses['get_text'] = jsonEncode(
          makeFakeResponse(<String, dynamic>{'text': 'hello'}),
        );
        final String result = await driver.getText(find.byValueKey(123), timeout: _kTestTimeout);
        expect(result, 'hello');
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"get_text","timeout":"1234","finderType":"ByValueKey","keyValueString":"123","keyValueType":"int"}') 0:00:01.234000''',
        ]);
      });
    });

    group('waitFor', () {
      test('sends the waitFor command', () async {
        fakeConnection.responses['waitFor'] = jsonEncode(
          makeFakeResponse(<String, dynamic>{'text': 'hello'}),
        );
        await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"waitFor","timeout":"1234","finderType":"ByTooltipMessage","text":"foo"}') 0:00:01.234000''',
        ]);
      });
    });

    group('waitForCondition', () {
      setUp(() {
        fakeConnection.responses['waitForCondition'] = jsonEncode(
          makeFakeResponse(<String, dynamic>{'text': 'hello'}),
        );
      });

      test('sends the wait for NoPendingFrameCondition command', () async {
        await driver.waitForCondition(const NoPendingFrame(), timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"waitForCondition","timeout":"1234","conditionName":"NoPendingFrameCondition"}') 0:00:01.234000''',
        ]);
      });

      test('sends the wait for NoPendingPlatformMessages command', () async {
        await driver.waitForCondition(const NoPendingPlatformMessages(), timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"waitForCondition","timeout":"1234","conditionName":"NoPendingPlatformMessagesCondition"}') 0:00:01.234000''',
        ]);
      });

      test('sends the waitForCondition of combined conditions command', () async {
        const SerializableWaitCondition combinedCondition = CombinedCondition(
          <SerializableWaitCondition>[NoPendingFrame(), NoTransientCallbacks()],
        );
        await driver.waitForCondition(combinedCondition, timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"waitForCondition","timeout":"1234","conditionName":"CombinedCondition","conditions":"[{\"conditionName\":\"NoPendingFrameCondition\"},{\"conditionName\":\"NoTransientCallbacksCondition\"}]"}') 0:00:01.234000''',
        ]);
      });
    });

    group('waitUntilNoTransientCallbacks', () {
      test('sends the waitUntilNoTransientCallbacks command', () async {
        fakeConnection.responses['waitForCondition'] = jsonEncode(
          makeFakeResponse(<String, dynamic>{}),
        );
        await driver.waitUntilNoTransientCallbacks(timeout: _kTestTimeout);
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"waitForCondition","timeout":"1234","conditionName":"NoTransientCallbacksCondition"}') 0:00:01.234000''',
        ]);
      });
    });

    group('getOffset', () {
      setUp(() {
        fakeConnection.responses['get_offset'] = jsonEncode(
          makeFakeResponse(<String, double>{'dx': 11, 'dy': 12}),
        );
      });

      test('sends the getCenter command', () async {
        final DriverOffset result = await driver.getCenter(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"get_offset","timeout":"1234","finderType":"ByValueKey","keyValueString":"123","keyValueType":"int","offsetType":"center"}') 0:00:01.234000''',
        ]);
      });

      test('sends the getTopLeft command', () async {
        final DriverOffset result = await driver.getTopLeft(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"get_offset","timeout":"1234","finderType":"ByValueKey","keyValueString":"123","keyValueType":"int","offsetType":"topLeft"}') 0:00:01.234000''',
        ]);
      });

      test('sends the getTopRight command', () async {
        final DriverOffset result = await driver.getTopRight(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"get_offset","timeout":"1234","finderType":"ByValueKey","keyValueString":"123","keyValueType":"int","offsetType":"topRight"}') 0:00:01.234000''',
        ]);
      });

      test('sends the getBottomLeft command', () async {
        final DriverOffset result = await driver.getBottomLeft(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"get_offset","timeout":"1234","finderType":"ByValueKey","keyValueString":"123","keyValueType":"int","offsetType":"bottomLeft"}') 0:00:01.234000''',
        ]);
      });

      test('sends the getBottomRight command', () async {
        final DriverOffset result = await driver.getBottomRight(
          find.byValueKey(123),
          timeout: _kTestTimeout,
        );
        expect(result, const DriverOffset(11, 12));
        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"get_offset","timeout":"1234","finderType":"ByValueKey","keyValueString":"123","keyValueType":"int","offsetType":"bottomRight"}') 0:00:01.234000''',
        ]);
      });
    });

    test('checks the health of the driver extension', () async {
      fakeConnection.responses['get_health'] = jsonEncode(
        makeFakeResponse(<String, dynamic>{'status': 'ok'}),
      );
      await driver.checkHealth();
      expect(fakeConnection.commandLog, <String>[
        r'''window.$flutterDriver('{"command":"get_health"}') null''',
      ]);
    });

    group('WebFlutterDriver Unimplemented/Unsupported error', () {
      test('forceGC', () async {
        expect(driver.forceGC(), throwsUnimplementedError);
      });

      test('getVmFlags', () async {
        expect(driver.getVmFlags(), throwsUnimplementedError);
      });

      test('waitUntilFirstFrameRasterized', () async {
        expect(driver.waitUntilFirstFrameRasterized(), throwsUnimplementedError);
      });

      test('appIsolate', () async {
        expect(() => driver.appIsolate.extensionRPCs, throwsUnsupportedError);
      });

      test('serviceClient', () async {
        expect(() => driver.serviceClient.getVM(), throwsUnsupportedError);
      });
    });

    group('runUnsynchronized', () {
      test('wrap waitFor with runUnsynchronized', () async {
        fakeConnection.responses['waitFor'] = jsonEncode(
          makeFakeResponse(<String, dynamic>{'text': 'hello'}),
        );
        fakeConnection.responses['set_frame_sync'] = jsonEncode(
          makeFakeResponse(<String, dynamic>{}),
        );

        await driver.runUnsynchronized(() async {
          await driver.waitFor(find.byTooltip('foo'), timeout: _kTestTimeout);
        });

        expect(fakeConnection.commandLog, <String>[
          r'''window.$flutterDriver('{"command":"set_frame_sync","enabled":"false"}') null''',
          r'''window.$flutterDriver('{"command":"waitFor","timeout":"1234","finderType":"ByTooltipMessage","text":"foo"}') 0:00:01.234000''',
          r'''window.$flutterDriver('{"command":"set_frame_sync","enabled":"true"}') null''',
        ]);
      });
    });
  });

  group('WebFlutterDriver with non-chrome browser', () {
    FakeFlutterWebConnection fakeConnection;
    late WebFlutterDriver driver;

    setUp(() {
      fakeConnection = FakeFlutterWebConnection();
      driver = WebFlutterDriver.connectedTo(fakeConnection);
    });

    test('tracing', () async {
      expect(
        driver.traceAction(() async {
          return Future<dynamic>.value();
        }),
        throwsUnsupportedError,
      );
      expect(driver.startTracing(), throwsUnsupportedError);
      expect(driver.stopTracingAndDownloadTimeline(), throwsUnsupportedError);
      expect(driver.clearTimeline(), throwsUnsupportedError);
    });
  });
}

// This function will verify the format of the script and return the actual
// script. The script will be in the following format:
//   window.flutterDriver('[actual script]')
String _checkAndEncode(dynamic script) {
  expect(script, isA<String>());
  final String scriptString = script as String;
  expect(scriptString.startsWith(_kWebScriptPrefix), isTrue);
  expect(scriptString.endsWith(_kWebScriptSuffix), isTrue);
  // Strip prefix and suffix
  return scriptString.substring(_kWebScriptPrefix.length, script.length - 2);
}

vms.Response? makeFakeResponse(Map<String, dynamic> response, {bool isError = false}) {
  return vms.Response.parse(<String, dynamic>{'isError': isError, 'response': response});
}

void Function(vms.Isolate) changeIsolateEventAfter(int gets, vms.Event nextEvent) {
  return (vms.Isolate i) {
    gets -= 1;
    if (gets == 0) {
      i.pauseEvent = nextEvent;
    }
  };
}

class FakeFlutterWebConnection extends Fake implements FlutterWebConnection {
  @override
  bool supportsTimelineAction = false;

  Map<String, dynamic> responses = <String, dynamic>{};
  List<String> commandLog = <String>[];
  @override
  Future<dynamic> sendCommand(String script, Duration? duration) async {
    commandLog.add('$script $duration');
    final Map<String, dynamic> decoded =
        jsonDecode(_checkAndEncode(script)) as Map<String, dynamic>;
    final dynamic response = responses[decoded['command']];
    assert(response != null, 'Missing ${decoded['command']} in responses.');
    return response;
  }

  @override
  Future<void> close() async {
    return;
  }
}

class FakeVmService extends Fake implements vms.VmService {
  FakeVmService(this.vm);

  FakeVM? vm;
  bool failOnSetFlag = false;
  bool failOnResumeWith101 = false;
  void Function(vms.Isolate)? onGetIsolate;

  final List<String> connectionLog = <String>[];

  @override
  Future<vms.VM> getVM() async => vm!;

  @override
  Future<vms.Isolate> getIsolate(String isolateId) async {
    connectionLog.add('getIsolate');
    if (isolateId == vm!.isolate!.id) {
      onGetIsolate?.call(vm!.isolate!);
      return vm!.isolate!;
    }
    throw UnimplementedError('getIsolate called with unrecognized $isolateId');
  }

  @override
  Future<vms.Success> resume(String isolateId, {String? step, int? frameIndex}) async {
    assert(isolateId == vm!.isolate!.id);
    connectionLog.add('resume');
    if (failOnResumeWith101) {
      throw vms.RPCError('resume', 101, '');
    }
    return vms.Success();
  }

  @override
  Future<vms.Success> streamListen(String streamId) async {
    connectionLog.add('streamListen $streamId');
    return vms.Success();
  }

  @override
  Future<vms.Success> streamCancel(String streamId) async {
    connectionLog.add('streamCancel $streamId');
    return vms.Success();
  }

  @override
  Future<vms.Response> setFlag(String name, String value) async {
    connectionLog.add('setFlag $name $value');
    if (failOnSetFlag) {
      throw Exception('setFlag failed');
    }
    return vms.Success();
  }

  @override
  Stream<vms.Event> get onIsolateEvent async* {
    connectionLog.add('onIsolateEvent');
    yield vms.Event(
      kind: vms.EventKind.kServiceExtensionAdded,
      extensionRPC: 'ext.flutter.driver',
      timestamp: 0,
    );
  }

  List<String> commandLog = <String>[];
  Map<String, vms.Response?> responses = <String, vms.Response?>{};
  Future<void>? artificialExtensionDelay;

  @override
  Future<vms.Response> callServiceExtension(
    String method, {
    Map<dynamic, dynamic>? args,
    String? isolateId,
  }) async {
    commandLog.add('$method $args');
    await artificialExtensionDelay;

    final vms.Response? response = responses[args!['command']];
    assert(response != null, 'Failed to create a response for ${args['command']}');
    return response!;
  }

  @override
  Future<vms.Success> clearVMTimeline() async {
    connectionLog.add('clearVMTimeline');
    return vms.Success();
  }

  @override
  Future<vms.FlagList> getFlagList() async {
    connectionLog.add('getFlagList');
    return vms.FlagList(flags: <vms.Flag>[]);
  }

  int vmTimelineMicros = -1000000;
  bool incrementMicros = false;

  @override
  Future<vms.Timestamp> getVMTimelineMicros() async {
    connectionLog.add('getVMTimelineMicros');
    if (incrementMicros || vmTimelineMicros < 0) {
      vmTimelineMicros = vmTimelineMicros + 1000001;
    }
    return vms.Timestamp(timestamp: vmTimelineMicros);
  }

  @override
  Future<vms.Success> setVMTimelineFlags(List<String> recordedStreams) async {
    connectionLog.add('setVMTimelineFlags $recordedStreams');
    return vms.Success();
  }

  final Map<int, vms.Timeline?> timelineResponses = <int, vms.Timeline?>{
    1: vms.Timeline.parse(<String, dynamic>{
      'traceEvents': <dynamic>[
        <String, dynamic>{'name': 'test event'},
      ],
      'timeOriginMicros': 0,
      'timeExtentMicros': 999999,
    }),
  };

  @override
  Future<vms.Timeline> getVMTimeline({int? timeOriginMicros, int? timeExtentMicros}) async {
    connectionLog.add('getVMTimeline $timeOriginMicros $timeExtentMicros');
    final vms.Timeline? timeline = timelineResponses[timeOriginMicros ?? 1];
    assert(timeline != null, 'Missing entry in timelineResponses[$timeOriginMicros]');
    return timeline!;
  }

  @override
  Future<void> dispose() async {
    connectionLog.add('dispose');
  }

  @override
  Future<void> get onDone async {}
}

class FakeVM extends Fake implements vms.VM {
  FakeVM(this.isolate);

  vms.Isolate? isolate;

  int numberOfTriesBeforeResolvingIsolate = 0;

  @override
  List<vms.IsolateRef> get isolates {
    numberOfTriesBeforeResolvingIsolate -= 1;
    return <vms.Isolate>[if (numberOfTriesBeforeResolvingIsolate <= 0) isolate!];
  }
}

vms.Isolate createFakeIsolate() => vms.Isolate(id: '123', number: '123', extensionRPCs: <String>[]);
