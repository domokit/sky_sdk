// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/signals.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/resident_devtools_handler.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../src/common.dart';
import '../src/fake_vm_services.dart';
import '../src/fakes.dart';

final vm_service.Isolate fakeUnpausedIsolate = vm_service.Isolate(
  id: '1',
  pauseEvent: vm_service.Event(kind: vm_service.EventKind.kResume, timestamp: 0),
  breakpoints: <vm_service.Breakpoint>[],
  extensionRPCs: <String>[],
  libraries: <vm_service.LibraryRef>[
    vm_service.LibraryRef(id: '1', uri: 'file:///hello_world/main.dart', name: ''),
  ],
  livePorts: 0,
  name: 'test',
  number: '1',
  pauseOnExit: false,
  runnable: true,
  startTime: 0,
  isSystemIsolate: false,
  isolateFlags: <vm_service.IsolateFlag>[],
);

final FlutterView fakeFlutterView = FlutterView(id: 'a', uiIsolate: fakeUnpausedIsolate);

final FakeVmServiceRequest listViews = FakeVmServiceRequest(
  method: kListViewsMethod,
  jsonResponse: <String, Object>{
    'views': <Object>[fakeFlutterView.toJson()],
  },
);

void main() {
  testWithoutContext('keyboard input handling single help character', () async {
    final TestRunner testRunner = TestRunner();
    final Logger logger = BufferLogger.test();
    final Signals signals = Signals.test();
    final Terminal terminal = Terminal.test();
    final MemoryFileSystem fs = MemoryFileSystem.test();
    final ProcessInfo processInfo = ProcessInfo.test(fs);
    final TerminalHandler terminalHandler = TerminalHandler(
      testRunner,
      logger: logger,
      signals: signals,
      terminal: terminal,
      processInfo: processInfo,
      reportReady: false,
    );

    expect(testRunner.hasHelpBeenPrinted, false);
    await terminalHandler.processTerminalInput('h');
    expect(testRunner.hasHelpBeenPrinted, true);
  });

  testWithoutContext('keyboard input handling help character surrounded with newlines', () async {
    final TestRunner testRunner = TestRunner();
    final Logger logger = BufferLogger.test();
    final Signals signals = Signals.test();
    final Terminal terminal = Terminal.test();
    final MemoryFileSystem fs = MemoryFileSystem.test();
    final ProcessInfo processInfo = ProcessInfo.test(fs);
    final TerminalHandler terminalHandler = TerminalHandler(
      testRunner,
      logger: logger,
      signals: signals,
      terminal: terminal,
      processInfo: processInfo,
      reportReady: false,
    );

    expect(testRunner.hasHelpBeenPrinted, false);
    await terminalHandler.processTerminalInput('\nh\n');
    expect(testRunner.hasHelpBeenPrinted, true);
  });

  group('keycode verification, brought to you by the letter', () {
    testWithoutContext('a, can handle trailing newlines', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsServiceProtocol: false,
      );
      await terminalHandler.processTerminalInput('a\n');

      expect(terminalHandler.lastReceivedCommand, 'a');
    });

    testWithoutContext('n, can handle trailing only newlines', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[]);
      await terminalHandler.processTerminalInput('\n\n');

      expect(terminalHandler.lastReceivedCommand, '');
    });

    testWithoutContext('a - debugToggleProfileWidgetBuilds', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.profileWidgetBuilds',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'enabled': 'false'},
        ),
        const FakeVmServiceRequest(
          method: 'ext.flutter.profileWidgetBuilds',
          args: <String, Object>{'isolateId': '1', 'enabled': 'true'},
          jsonResponse: <String, Object>{'enabled': 'true'},
        ),
      ]);

      await terminalHandler.processTerminalInput('a');
    });

    testWithoutContext('a - debugToggleProfileWidgetBuilds with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.profileWidgetBuilds',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'enabled': 'false'},
        ),
        const FakeVmServiceRequest(
          method: 'ext.flutter.profileWidgetBuilds',
          args: <String, Object>{'isolateId': '1', 'enabled': 'true'},
          jsonResponse: <String, Object>{'enabled': 'true'},
        ),
      ], web: true);

      await terminalHandler.processTerminalInput('a');
    });

    testWithoutContext(
      'a - debugToggleProfileWidgetBuilds without service protocol is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );

        await terminalHandler.processTerminalInput('a');
      },
    );

    testWithoutContext('b - debugToggleBrightness', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.brightnessOverride',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'value': 'Brightness.light'},
        ),
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.brightnessOverride',
          args: <String, Object>{'isolateId': '1', 'value': 'Brightness.dark'},
          jsonResponse: <String, Object>{'value': 'Brightness.dark'},
        ),
      ]);
      await terminalHandler.processTerminalInput('b');

      expect(terminalHandler.logger.statusText, contains('Changed brightness to Brightness.dark'));
    });

    testWithoutContext('b - debugToggleBrightness with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.brightnessOverride',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'value': 'Brightness.light'},
        ),
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.brightnessOverride',
          args: <String, Object>{'isolateId': '1', 'value': 'Brightness.dark'},
          jsonResponse: <String, Object>{'value': 'Brightness.dark'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('b');

      expect(terminalHandler.logger.statusText, contains('Changed brightness to Brightness.dark'));
    });

    testWithoutContext('b - debugToggleBrightness without service protocol is skipped', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsServiceProtocol: false,
      );

      await terminalHandler.processTerminalInput('b');
    });

    testWithoutContext('d,D - detach', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[]);
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;
      await terminalHandler.processTerminalInput('d');

      expect(runner.calledDetach, true);
      runner.calledDetach = false;

      await terminalHandler.processTerminalInput('D');

      expect(runner.calledDetach, true);
    });

    testWithoutContext('h,H,? - printHelp', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[]);
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;
      await terminalHandler.processTerminalInput('h');

      expect(runner.calledPrintWithDetails, true);
      runner.calledPrintWithDetails = false;

      await terminalHandler.processTerminalInput('H');

      expect(runner.calledPrintWithDetails, true);
      runner.calledPrintWithDetails = false;

      await terminalHandler.processTerminalInput('?');

      expect(runner.calledPrintWithDetails, true);
    });

    testWithoutContext('i - debugToggleWidgetInspector', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.inspector.show',
          args: <String, Object>{'isolateId': '1'},
        ),
      ]);

      await terminalHandler.processTerminalInput('i');
    });

    testWithoutContext('i - debugToggleWidgetInspector with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.inspector.show',
          args: <String, Object>{'isolateId': '1'},
        ),
      ], web: true);

      await terminalHandler.processTerminalInput('i');
    });

    testWithoutContext(
      'i - debugToggleWidgetInspector without service protocol is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );

        await terminalHandler.processTerminalInput('i');
      },
    );

    testWithoutContext('I - debugToggleInvertOversizedImages', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.invertOversizedImages',
          args: <String, Object>{'isolateId': '1'},
        ),
      ]);
      await terminalHandler.processTerminalInput('I');
    });

    testWithoutContext('I - debugToggleInvertOversizedImages with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.invertOversizedImages',
          args: <String, Object>{'isolateId': '1'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('I');
    });

    testWithoutContext(
      'I - debugToggleInvertOversizedImages without service protocol is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );
        await terminalHandler.processTerminalInput('I');
      },
    );

    testWithoutContext('I - debugToggleInvertOversizedImages in profile mode is skipped', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        buildMode: BuildMode.profile,
      );
      await terminalHandler.processTerminalInput('I');
    });

    testWithoutContext('L - debugDumpLayerTree', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpLayerTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'LAYER TREE'},
        ),
      ]);
      await terminalHandler.processTerminalInput('L');

      expect(terminalHandler.logger.statusText, contains('LAYER TREE'));
    });

    testWithoutContext('L - debugDumpLayerTree with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpLayerTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'LAYER TREE'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('L');

      expect(terminalHandler.logger.statusText, contains('LAYER TREE'));
    });

    testWithoutContext(
      'L - debugDumpLayerTree with service protocol and profile mode is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          buildMode: BuildMode.profile,
        );
        await terminalHandler.processTerminalInput('L');
      },
    );

    testWithoutContext('L - debugDumpLayerTree without service protocol is skipped', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsServiceProtocol: false,
      );
      await terminalHandler.processTerminalInput('L');
    });

    testWithoutContext('f - debugDumpFocusTree', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpFocusTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'FOCUS TREE'},
        ),
      ]);
      await terminalHandler.processTerminalInput('f');

      expect(terminalHandler.logger.statusText, contains('FOCUS TREE'));
    });

    testWithoutContext('f - debugDumpLayerTree with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpFocusTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'FOCUS TREE'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('f');

      expect(terminalHandler.logger.statusText, contains('FOCUS TREE'));
    });

    testWithoutContext(
      'f - debugDumpFocusTree with service protocol and profile mode is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          buildMode: BuildMode.profile,
        );
        await terminalHandler.processTerminalInput('f');
      },
    );

    testWithoutContext('f - debugDumpFocusTree without service protocol is skipped', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsServiceProtocol: false,
      );
      await terminalHandler.processTerminalInput('f');
    });

    testWithoutContext('o,O - debugTogglePlatform', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        // Request 1.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'value': 'iOS'},
        ),
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1', 'value': 'windows'},
          jsonResponse: <String, Object>{'value': 'windows'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'value': 'android'},
        ),
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1', 'value': 'iOS'},
          jsonResponse: <String, Object>{'value': 'iOS'},
        ),
      ]);
      await terminalHandler.processTerminalInput('o');
      await terminalHandler.processTerminalInput('O');

      expect(terminalHandler.logger.statusText, contains('Switched operating system to windows'));
      expect(terminalHandler.logger.statusText, contains('Switched operating system to iOS'));
    });

    testWithoutContext('o,O - debugTogglePlatform with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        // Request 1.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'value': 'iOS'},
        ),
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1', 'value': 'windows'},
          jsonResponse: <String, Object>{'value': 'windows'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'value': 'android'},
        ),
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.platformOverride',
          args: <String, Object>{'isolateId': '1', 'value': 'iOS'},
          jsonResponse: <String, Object>{'value': 'iOS'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('o');
      await terminalHandler.processTerminalInput('O');

      expect(terminalHandler.logger.statusText, contains('Switched operating system to windows'));
      expect(terminalHandler.logger.statusText, contains('Switched operating system to iOS'));
    });

    testWithoutContext('o,O - debugTogglePlatform without service protocol is skipped', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsServiceProtocol: false,
      );
      await terminalHandler.processTerminalInput('o');
      await terminalHandler.processTerminalInput('O');
    });

    testWithoutContext('p - debugToggleDebugPaintSizeEnabled', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugPaint',
          args: <String, Object>{'isolateId': '1'},
        ),
      ]);
      await terminalHandler.processTerminalInput('p');
    });

    testWithoutContext('p - debugToggleDebugPaintSizeEnabled with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugPaint',
          args: <String, Object>{'isolateId': '1'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('p');
    });

    testWithoutContext(
      'p - debugToggleDebugPaintSizeEnabled without service protocol is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );
        await terminalHandler.processTerminalInput('p');
      },
    );

    testWithoutContext('P - debugTogglePerformanceOverlayOverride', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.showPerformanceOverlay',
          args: <String, Object>{'isolateId': '1'},
        ),
      ]);
      await terminalHandler.processTerminalInput('P');
    });

    testWithoutContext(
      'P - debugTogglePerformanceOverlayOverride with web target is skipped ',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          web: true,
        );
        await terminalHandler.processTerminalInput('P');
      },
    );

    testWithoutContext(
      'P - debugTogglePerformanceOverlayOverride without service protocol is skipped ',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );
        await terminalHandler.processTerminalInput('P');
      },
    );

    testWithoutContext('S - debugDumpSemanticsTreeInTraversalOrder', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpSemanticsTreeInTraversalOrder',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'SEMANTICS DATA'},
        ),
      ]);
      await terminalHandler.processTerminalInput('S');

      expect(terminalHandler.logger.statusText, contains('SEMANTICS DATA'));
    });

    testWithoutContext('S - debugDumpSemanticsTreeInTraversalOrder with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpSemanticsTreeInTraversalOrder',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'SEMANTICS DATA'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('S');

      expect(terminalHandler.logger.statusText, contains('SEMANTICS DATA'));
    });

    testWithoutContext(
      'S - debugDumpSemanticsTreeInTraversalOrder without service protocol is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );
        await terminalHandler.processTerminalInput('S');
      },
    );

    testWithoutContext('U - debugDumpSemanticsTreeInInverseHitTestOrder', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpSemanticsTreeInInverseHitTestOrder',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'SEMANTICS DATA'},
        ),
      ]);
      await terminalHandler.processTerminalInput('U');

      expect(terminalHandler.logger.statusText, contains('SEMANTICS DATA'));
    });

    testWithoutContext('U - debugDumpSemanticsTreeInInverseHitTestOrder with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpSemanticsTreeInInverseHitTestOrder',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'SEMANTICS DATA'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('U');

      expect(terminalHandler.logger.statusText, contains('SEMANTICS DATA'));
    });

    testWithoutContext(
      'U - debugDumpSemanticsTreeInInverseHitTestOrder without service protocol is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );
        await terminalHandler.processTerminalInput('U');
      },
    );

    testWithoutContext('t,T - debugDumpRenderTree', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpRenderTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'RENDER DATA 1'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpRenderTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'RENDER DATA 2'},
        ),
      ]);
      await terminalHandler.processTerminalInput('t');
      await terminalHandler.processTerminalInput('T');

      expect(terminalHandler.logger.statusText, contains('RENDER DATA 1'));
      expect(terminalHandler.logger.statusText, contains('RENDER DATA 2'));
    });

    testWithoutContext('t,T - debugDumpRenderTree with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpRenderTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'RENDER DATA 1'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpRenderTree',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'RENDER DATA 2'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('t');
      await terminalHandler.processTerminalInput('T');

      expect(terminalHandler.logger.statusText, contains('RENDER DATA 1'));
      expect(terminalHandler.logger.statusText, contains('RENDER DATA 2'));
    });

    testWithoutContext('t,T - debugDumpRenderTree without service protocol is skipped', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsServiceProtocol: false,
      );
      await terminalHandler.processTerminalInput('t');
      await terminalHandler.processTerminalInput('T');
    });

    testWithoutContext('w,W - debugDumpApp', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpApp',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'WIDGET DATA 1'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpApp',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'WIDGET DATA 2'},
        ),
      ]);
      await terminalHandler.processTerminalInput('w');
      await terminalHandler.processTerminalInput('W');

      expect(terminalHandler.logger.statusText, contains('WIDGET DATA 1'));
      expect(terminalHandler.logger.statusText, contains('WIDGET DATA 2'));
    });

    testWithoutContext('w,W - debugDumpApp with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpApp',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'WIDGET DATA 1'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugDumpApp',
          args: <String, Object>{'isolateId': '1'},
          jsonResponse: <String, Object>{'data': 'WIDGET DATA 2'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('w');
      await terminalHandler.processTerminalInput('W');

      expect(terminalHandler.logger.statusText, contains('WIDGET DATA 1'));
      expect(terminalHandler.logger.statusText, contains('WIDGET DATA 2'));
    });

    testWithoutContext('v - launchDevToolsInBrowser', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[]);
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;
      final FakeResidentDevtoolsHandler devtoolsHandler =
          runner.residentDevtoolsHandler as FakeResidentDevtoolsHandler;

      expect(devtoolsHandler.calledLaunchDevToolsInBrowser, isFalse);
      await terminalHandler.processTerminalInput('v');
      expect(devtoolsHandler.calledLaunchDevToolsInBrowser, isTrue);
    });

    testWithoutContext('w,W - debugDumpApp without service protocol is skipped', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsServiceProtocol: false,
      );
      await terminalHandler.processTerminalInput('w');
      await terminalHandler.processTerminalInput('W');
    });

    testWithoutContext('z,Z - debugToggleDebugCheckElevationsEnabled', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugCheckElevationsEnabled',
          args: <String, Object>{'isolateId': '1'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugCheckElevationsEnabled',
          args: <String, Object>{'isolateId': '1'},
        ),
      ]);
      await terminalHandler.processTerminalInput('z');
      await terminalHandler.processTerminalInput('Z');
    });

    testWithoutContext('z,Z - debugToggleDebugCheckElevationsEnabled with web target', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugCheckElevationsEnabled',
          args: <String, Object>{'isolateId': '1'},
        ),
        // Request 2.
        listViews,
        const FakeVmServiceRequest(
          method: 'ext.flutter.debugCheckElevationsEnabled',
          args: <String, Object>{'isolateId': '1'},
        ),
      ], web: true);
      await terminalHandler.processTerminalInput('z');
      await terminalHandler.processTerminalInput('Z');
    });

    testWithoutContext(
      'z,Z - debugToggleDebugCheckElevationsEnabled without service protocol is skipped',
      () async {
        final TerminalHandler terminalHandler = setUpTerminalHandler(
          <FakeVmServiceRequest>[],
          supportsServiceProtocol: false,
        );
        await terminalHandler.processTerminalInput('z');
        await terminalHandler.processTerminalInput('Z');
      },
    );

    testWithoutContext('q,Q - exit', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[]);
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;
      await terminalHandler.processTerminalInput('q');

      expect(runner.calledExit, true);
      runner.calledExit = false;

      await terminalHandler.processTerminalInput('Q');

      expect(runner.calledExit, true);
    });

    testWithoutContext('r - hotReload unsupported', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsHotReload: false,
      );
      await terminalHandler.processTerminalInput('r');
    });

    testWithoutContext('R - hotRestart unsupported', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        supportsRestart: false,
      );
      await terminalHandler.processTerminalInput('R');
    });

    testWithoutContext('r - hotReload', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[]);
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;

      await terminalHandler.processTerminalInput('r');

      expect(runner.calledReload, true);
      expect(runner.calledRestart, false);
    });

    testWithoutContext('R - hotRestart', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(<FakeVmServiceRequest>[]);
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;

      await terminalHandler.processTerminalInput('R');

      expect(runner.calledReload, false);
      expect(runner.calledRestart, true);
    });

    testWithoutContext('r - hotReload with non-fatal error', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        reloadExitCode: 1,
      );
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;

      await terminalHandler.processTerminalInput('r');

      expect(runner.calledReload, true);
      expect(runner.calledRestart, false);
      expect(
        terminalHandler.logger.statusText,
        contains('Try again after fixing the above error(s).'),
      );
    });

    testWithoutContext('R - hotRestart with non-fatal error', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        reloadExitCode: 1,
      );
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;

      await terminalHandler.processTerminalInput('R');

      expect(runner.calledReload, false);
      expect(runner.calledRestart, true);
      expect(
        terminalHandler.logger.statusText,
        contains('Try again after fixing the above error(s).'),
      );
    });

    testWithoutContext('r - hotReload with fatal error', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        reloadExitCode: 1,
        fatalReloadError: true,
      );
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;

      await expectLater(() => terminalHandler.processTerminalInput('r'), throwsToolExit());

      expect(runner.calledReload, true);
      expect(runner.calledRestart, false);
    });

    testWithoutContext('R - hotRestart with fatal error', () async {
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        reloadExitCode: 1,
        fatalReloadError: true,
      );
      final FakeResidentRunner runner = terminalHandler.residentRunner as FakeResidentRunner;

      await expectLater(() => terminalHandler.processTerminalInput('R'), throwsToolExit());

      expect(runner.calledReload, false);
      expect(runner.calledRestart, true);
    });
  });

  testWithoutContext('ResidentRunner clears the screen when it should', () async {
    final TerminalHandler terminalHandler = setUpTerminalHandler(
      <FakeVmServiceRequest>[],
      reloadExitCode: 1,
      fatalReloadError: true,
    );
    const String message = 'This should be cleared';

    expect(terminalHandler.logger.statusText, equals(''));
    terminalHandler.logger.printStatus(message);
    expect(terminalHandler.logger.statusText, equals('$message\n')); // printStatus makes a newline

    await terminalHandler.processTerminalInput('c');
    expect(terminalHandler.logger.statusText, equals(''));
  });

  testWithoutContext('s, can take screenshot on debug device that supports screenshot', () async {
    final BufferLogger logger = BufferLogger.test();
    final TerminalHandler terminalHandler = setUpTerminalHandler(
      <FakeVmServiceRequest>[
        listViews,
        FakeVmServiceRequest(
          method: 'ext.flutter.debugAllowBanner',
          args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'false'},
        ),
        FakeVmServiceRequest(
          method: 'ext.flutter.debugAllowBanner',
          args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'true'},
        ),
      ],
      logger: logger,
      supportsScreenshot: true,
    );

    await terminalHandler.processTerminalInput('s');

    expect(logger.statusText, contains('Screenshot written to flutter_01.png (0kB)'));
  });

  testWithoutContext(
    's, will not take screenshot on non-web device without screenshot tooling support',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        logger: logger,
        fileSystem: fileSystem,
      );

      await terminalHandler.processTerminalInput('s');

      expect(logger.statusText, isNot(contains('Screenshot written to')));
    },
  );

  testWithoutContext(
    's, can take screenshot on debug web device that does not support screenshot',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[
          listViews,
          FakeVmServiceRequest(
            method: 'ext.flutter.debugAllowBanner',
            args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'false'},
          ),
          FakeVmServiceRequest(
            method: 'ext.dwds.screenshot',
            args: <String, Object>{},
            jsonResponse: <String, Object>{
              'data': base64.encode(<int>[1, 2, 3, 4]),
            },
          ),
          FakeVmServiceRequest(
            method: 'ext.flutter.debugAllowBanner',
            args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'true'},
          ),
        ],
        logger: logger,
        web: true,
        fileSystem: fileSystem,
      );

      await terminalHandler.processTerminalInput('s');

      expect(logger.statusText, contains('Screenshot written to flutter_01.png (0kB)'));
      expect(fileSystem.currentDirectory.childFile('flutter_01.png').readAsBytesSync(), <int>[
        1,
        2,
        3,
        4,
      ]);
    },
  );

  testWithoutContext(
    's, can take screenshot on device that does not support service protocol',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        logger: logger,
        supportsScreenshot: true,
        supportsServiceProtocol: false,
        fileSystem: fileSystem,
      );

      await terminalHandler.processTerminalInput('s');

      expect(logger.statusText, contains('Screenshot written to flutter_01.png (0kB)'));
      expect(fileSystem.currentDirectory.childFile('flutter_01.png').readAsBytesSync(), <int>[
        1,
        2,
        3,
        4,
      ]);
    },
  );

  testWithoutContext(
    's, does not take a screenshot on a device that does not support screenshot or the service protocol',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        logger: logger,
        supportsServiceProtocol: false,
        fileSystem: fileSystem,
      );

      await terminalHandler.processTerminalInput('s');

      expect(logger.statusText, '\n');
      expect(fileSystem.currentDirectory.childFile('flutter_01.png'), isNot(exists));
    },
  );

  testWithoutContext(
    's, does not take a screenshot on a web device that does not support screenshot or the service protocol',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[],
        logger: logger,
        supportsServiceProtocol: false,
        web: true,
        fileSystem: fileSystem,
      );

      await terminalHandler.processTerminalInput('s');

      expect(logger.statusText, '\n');
      expect(fileSystem.currentDirectory.childFile('flutter_01.png'), isNot(exists));
    },
  );

  testWithoutContext(
    's, bails taking screenshot on debug device if dwds.screenshot throws RpcError, restoring banner',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[
          listViews,
          FakeVmServiceRequest(
            method: 'ext.flutter.debugAllowBanner',
            args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'false'},
          ),
          FakeVmServiceRequest(
            method: 'ext.dwds.screenshot',
            // Failed response,
            error: FakeRPCError(code: vm_service.RPCErrorKind.kInternalError.code),
          ),
          FakeVmServiceRequest(
            method: 'ext.flutter.debugAllowBanner',
            args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'true'},
          ),
        ],
        logger: logger,
        web: true,
        fileSystem: fileSystem,
      );

      await terminalHandler.processTerminalInput('s');

      expect(logger.errorText, contains('Error'));
    },
  );

  testWithoutContext(
    's, bails taking screenshot on debug device if debugAllowBanner during second request',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final TerminalHandler terminalHandler = setUpTerminalHandler(
        <FakeVmServiceRequest>[
          listViews,
          FakeVmServiceRequest(
            method: 'ext.flutter.debugAllowBanner',
            args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'false'},
          ),
          FakeVmServiceRequest(
            method: 'ext.flutter.debugAllowBanner',
            args: <String, Object?>{'isolateId': fakeUnpausedIsolate.id, 'enabled': 'true'},
            // Failed response,
            error: FakeRPCError(code: vm_service.RPCErrorKind.kInternalError.code),
          ),
        ],
        logger: logger,
        supportsScreenshot: true,
        fileSystem: fileSystem,
      );

      await terminalHandler.processTerminalInput('s');

      expect(logger.errorText, contains('Error'));
    },
  );

  testWithoutContext('pidfile creation', () {
    final BufferLogger testLogger = BufferLogger.test();
    final Signals signals = _TestSignals(Signals.defaultExitSignals);
    final Terminal terminal = Terminal.test();
    final MemoryFileSystem fs = MemoryFileSystem.test();
    final ProcessInfo processInfo = ProcessInfo.test(fs);
    final FakeResidentRunner residentRunner = FakeResidentRunner(
      FlutterDevice(
        FakeDevice(),
        buildInfo: BuildInfo.debug,
        generator: FakeResidentCompiler(),
        developmentShaderCompiler: const FakeShaderCompiler(),
      ),
      testLogger,
      fs,
    );
    residentRunner
      ..supportsRestart = true
      ..supportsServiceProtocol = true
      ..stayResident = true;

    const String filename = 'test.pid';
    final TerminalHandler terminalHandler = TerminalHandler(
      residentRunner,
      logger: testLogger,
      signals: signals,
      terminal: terminal,
      processInfo: processInfo,
      reportReady: false,
      pidFile: filename,
    );

    expect(fs.file(filename), isNot(exists));
    terminalHandler.setupTerminal();
    terminalHandler.registerSignalHandlers();
    expect(fs.file(filename), exists);
    terminalHandler.stop();
    expect(fs.file(filename), isNot(exists));
  });
}

class FakeResidentRunner extends ResidentHandlers {
  FakeResidentRunner(FlutterDevice device, this.logger, this.fileSystem)
    : flutterDevices = <FlutterDevice>[device];

  bool calledDetach = false;
  bool calledPrint = false;
  bool calledExit = false;
  bool calledPrintWithDetails = false;
  bool calledReload = false;
  bool calledRestart = false;
  int reloadExitCode = 0;
  bool fatalReloadError = false;

  @override
  final Logger logger;

  @override
  final FileSystem fileSystem;

  @override
  final List<FlutterDevice> flutterDevices;

  @override
  bool canHotReload = true;

  @override
  bool hotMode = true;

  @override
  bool isRunningDebug = true;

  @override
  bool isRunningProfile = false;

  @override
  bool isRunningRelease = false;

  @override
  bool stayResident = true;

  @override
  bool supportsRestart = true;

  @override
  bool supportsServiceProtocol = true;

  @override
  bool supportsWriteSkSL = true;

  @override
  Future<void> cleanupAfterSignal() async {}

  @override
  Future<void> detach() async {
    calledDetach = true;
  }

  @override
  Future<void> exit() async {
    calledExit = true;
  }

  @override
  void printHelp({required bool details}) {
    if (details) {
      calledPrintWithDetails = true;
    } else {
      calledPrint = true;
    }
  }

  @override
  Future<void> runSourceGenerators() async {}

  @override
  Future<OperationResult> restart({
    bool fullRestart = false,
    bool pause = false,
    String? reason,
  }) async {
    if (fullRestart && !supportsRestart) {
      throw StateError('illegal restart');
    }
    if (!fullRestart && !canHotReload) {
      throw StateError('illegal reload');
    }
    if (fullRestart) {
      calledRestart = true;
    } else {
      calledReload = true;
    }
    return OperationResult(reloadExitCode, '', fatal: fatalReloadError);
  }

  // TODO(bkonyi): remove when ready to serve DevTools from DDS.
  @override
  ResidentDevtoolsHandler get residentDevtoolsHandler => _residentDevtoolsHandler;
  final ResidentDevtoolsHandler _residentDevtoolsHandler = FakeResidentDevtoolsHandler();
}

// TODO(bkonyi): remove when ready to serve DevTools from DDS.
class FakeResidentDevtoolsHandler extends Fake implements ResidentDevtoolsHandler {
  bool calledLaunchDevToolsInBrowser = false;

  @override
  bool launchDevToolsInBrowser({List<FlutterDevice?>? flutterDevices}) {
    return calledLaunchDevToolsInBrowser = true;
  }
}

class FakeDevice extends Fake implements Device {
  @override
  bool isSupported() => true;

  @override
  bool supportsScreenshot = false;

  @override
  String get name => 'Fake Device';

  @override
  DartDevelopmentService dds = DartDevelopmentService(logger: FakeLogger());

  @override
  Future<void> takeScreenshot(File file) async {
    if (!supportsScreenshot) {
      throw StateError('illegal screenshot attempt');
    }
    file.writeAsBytesSync(<int>[1, 2, 3, 4]);
  }
}

TerminalHandler setUpTerminalHandler(
  List<FakeVmServiceRequest> requests, {
  bool supportsRestart = true,
  bool supportsServiceProtocol = true,
  bool supportsHotReload = true,
  bool web = false,
  bool fatalReloadError = false,
  bool supportsScreenshot = false,
  int reloadExitCode = 0,
  BuildMode buildMode = BuildMode.debug,
  Logger? logger,
  FileSystem? fileSystem,
}) {
  final Logger testLogger = logger ?? BufferLogger.test();
  final Signals signals = Signals.test();
  final Terminal terminal = Terminal.test();
  final FileSystem localFileSystem = fileSystem ?? MemoryFileSystem.test();
  final ProcessInfo processInfo = ProcessInfo.test(MemoryFileSystem.test());
  final FlutterDevice device = FlutterDevice(
    FakeDevice()..supportsScreenshot = supportsScreenshot,
    buildInfo: BuildInfo(
      buildMode,
      '',
      treeShakeIcons: false,
      packageConfigPath: '.dart_tool/package_config.json',
    ),
    generator: FakeResidentCompiler(),
    developmentShaderCompiler: const FakeShaderCompiler(),
    targetPlatform: web ? TargetPlatform.web_javascript : TargetPlatform.android_arm,
  );
  device.vmService = FakeVmServiceHost(requests: requests).vmService;
  final FakeResidentRunner residentRunner =
      FakeResidentRunner(device, testLogger, localFileSystem)
        ..supportsServiceProtocol = supportsServiceProtocol
        ..supportsRestart = supportsRestart
        ..canHotReload = supportsHotReload
        ..fatalReloadError = fatalReloadError
        ..reloadExitCode = reloadExitCode;

  switch (buildMode) {
    case BuildMode.debug:
      residentRunner
        ..isRunningDebug = true
        ..isRunningProfile = false
        ..isRunningRelease = false;
    case BuildMode.profile:
      residentRunner
        ..isRunningDebug = false
        ..isRunningProfile = true
        ..isRunningRelease = false;
    case BuildMode.release:
      residentRunner
        ..isRunningDebug = false
        ..isRunningProfile = false
        ..isRunningRelease = true;
    case _:
    // NOOP
  }
  return TerminalHandler(
    residentRunner,
    logger: testLogger,
    signals: signals,
    terminal: terminal,
    processInfo: processInfo,
    reportReady: false,
  );
}

class FakeResidentCompiler extends Fake implements ResidentCompiler {}

class TestRunner extends Fake implements ResidentRunner {
  bool hasHelpBeenPrinted = false;

  @override
  Future<void> cleanupAfterSignal() async {}

  @override
  Future<void> cleanupAtFinish() async {}

  @override
  void printHelp({bool? details}) {
    hasHelpBeenPrinted = true;
  }

  @override
  Future<int?> run({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool enableDevTools = false,
    String? route,
  }) async => null;

  @override
  Future<int?> attach({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool allowExistingDdsInstance = false,
    bool enableDevTools = false,
    bool needsFullRestart = true,
  }) async => null;
}

class _TestSignals implements Signals {
  _TestSignals(this.exitSignals);

  final List<ProcessSignal> exitSignals;

  final Map<ProcessSignal, Map<Object, SignalHandler>> _handlersTable =
      <ProcessSignal, Map<Object, SignalHandler>>{};

  @override
  Object addHandler(ProcessSignal signal, SignalHandler handler) {
    final Object token = Object();
    _handlersTable.putIfAbsent(signal, () => <Object, SignalHandler>{})[token] = handler;
    return token;
  }

  @override
  Future<bool> removeHandler(ProcessSignal signal, Object token) async {
    if (!_handlersTable.containsKey(signal)) {
      return false;
    }
    if (!_handlersTable[signal]!.containsKey(token)) {
      return false;
    }
    _handlersTable[signal]!.remove(token);
    return true;
  }

  @override
  Stream<Object> get errors => _errors.stream;
  final StreamController<Object> _errors = StreamController<Object>();
}

class FakeShaderCompiler implements DevelopmentShaderCompiler {
  const FakeShaderCompiler();

  @override
  void configureCompiler(TargetPlatform? platform) {}

  @override
  Future<DevFSContent> recompileShader(DevFSContent inputShader) {
    throw UnimplementedError();
  }
}
