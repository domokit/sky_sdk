// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:fake_async/fake_async.dart';
import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/web/chrome.dart';
import 'package:test/fake.dart';
import 'package:webkit_inspection_protocol/webkit_inspection_protocol.dart';

import '../src/common.dart';
import '../src/fake_process_manager.dart';
import '../src/fakes.dart' hide FakeProcess;

const List<String> kChromeArgs = <String>[
  '--disable-background-timer-throttling',
  '--disable-extensions',
  '--disable-popup-blocking',
  '--bwsi',
  '--no-first-run',
  '--no-default-browser-check',
  '--disable-default-apps',
  '--disable-translate',
  '--disable-search-engine-choice-screen',
];

const List<String> kCodeCache = <String>['Cache', 'Code Cache', 'GPUCache'];

const String kDevtoolsStderr = '\n\nDevTools listening\n\n';

void main() {
  late FileExceptionHandler exceptionHandler;
  late ChromiumLauncher chromeLauncher;
  late FileSystem fileSystem;
  late Platform platform;
  late FakeProcessManager processManager;
  late OperatingSystemUtils operatingSystemUtils;
  late BufferLogger testLogger;

  setUp(() {
    exceptionHandler = FileExceptionHandler();
    operatingSystemUtils = FakeOperatingSystemUtils();
    platform = FakePlatform(
      operatingSystem: 'macos',
      environment: <String, String>{kChromeEnvironment: 'example_chrome'},
    );
    fileSystem = MemoryFileSystem.test(opHandle: exceptionHandler.opHandle);
    processManager = FakeProcessManager.empty();
    chromeLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: testLogger = BufferLogger.test(),
    );
  });

  Future<Chromium> testLaunchChrome(
    String userDataDir,
    FakeProcessManager processManager,
    ChromiumLauncher chromeLauncher,
  ) {
    if (testLogger.isVerbose) {
      processManager.addCommand(
        const FakeCommand(command: <String>['example_chrome', '--version'], stdout: 'Chromium 115'),
      );
    }

    processManager.addCommand(
      FakeCommand(
        command: <String>[
          'example_chrome',
          '--user-data-dir=$userDataDir',
          '--remote-debugging-port=12345',
          ...kChromeArgs,
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    );

    return chromeLauncher.launch('example_url', skipCheck: true);
  }

  testWithoutContext('can launch chrome and connect to the devtools', () async {
    await expectReturnsNormallyLater(
      testLaunchChrome(
        '/.tmp_rand0/flutter_tools_chrome_device.rand0',
        processManager,
        chromeLauncher,
      ),
    );
  });

  testWithoutContext('can launch chrome in verbose mode', () async {
    chromeLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: testLogger = BufferLogger.test(verbose: true),
    );

    await expectReturnsNormallyLater(
      testLaunchChrome(
        '/.tmp_rand0/flutter_tools_chrome_device.rand0',
        processManager,
        chromeLauncher,
      ),
    );

    expect(
      testLogger.traceText.trim(),
      'Launching Chromium (url = example_url, headless = false, skipCheck = true, debugPort = null)\n'
      'Will use Chromium executable at example_chrome\n'
      'Using Chromium 115\n'
      '[CHROME]: \n'
      '[CHROME]: \n'
      '[CHROME]: DevTools listening',
    );
  });

  testWithoutContext('cannot have two concurrent instances of chrome', () async {
    await testLaunchChrome(
      '/.tmp_rand0/flutter_tools_chrome_device.rand0',
      processManager,
      chromeLauncher,
    );

    await expectToolExitLater(
      testLaunchChrome(
        '/.tmp_rand0/flutter_tools_chrome_device.rand1',
        processManager,
        chromeLauncher,
      ),
      contains('Only one instance of chrome can be started'),
    );
  });

  testWithoutContext('can launch new chrome after stopping a previous chrome', () async {
    final Chromium chrome = await testLaunchChrome(
      '/.tmp_rand0/flutter_tools_chrome_device.rand0',
      processManager,
      chromeLauncher,
    );
    await chrome.close();

    await expectReturnsNormallyLater(
      testLaunchChrome(
        '/.tmp_rand0/flutter_tools_chrome_device.rand1',
        processManager,
        chromeLauncher,
      ),
    );
  });

  testWithoutContext('exits normally using SIGTERM', () async {
    final BufferLogger logger = BufferLogger.test();
    final FakeAsync fakeAsync = FakeAsync();

    fakeAsync.run((_) {
      () async {
        final FakeChromeConnection chromeConnection = FakeChromeConnection(maxRetries: 4);
        final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
          fileSystem: fileSystem,
          platform: platform,
          processManager: processManager,
          operatingSystemUtils: operatingSystemUtils,
          browserFinder: findChromeExecutable,
          logger: logger,
        );

        final FakeProcess process = FakeProcess(duration: const Duration(seconds: 3));

        final Chromium chrome = Chromium(
          0,
          chromeConnection,
          chromiumLauncher: chromiumLauncher,
          process: process,
          logger: logger,
        );

        final Future<void> closeFuture = chrome.close();
        fakeAsync.elapse(const Duration(seconds: 4));
        await closeFuture;

        expect(process.signals, <io.ProcessSignal>[io.ProcessSignal.sigterm]);
      }();
    });

    fakeAsync.flushTimers();
    expect(logger.warningText, isEmpty);
  });

  testWithoutContext('falls back to SIGKILL if SIGTERM did not work', () async {
    final BufferLogger logger = BufferLogger.test();
    final FakeAsync fakeAsync = FakeAsync();

    fakeAsync.run((_) {
      () async {
        final FakeChromeConnection chromeConnection = FakeChromeConnection(maxRetries: 4);
        final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
          fileSystem: fileSystem,
          platform: platform,
          processManager: processManager,
          operatingSystemUtils: operatingSystemUtils,
          browserFinder: findChromeExecutable,
          logger: logger,
        );

        final FakeProcess process = FakeProcess(duration: const Duration(seconds: 6));

        final Chromium chrome = Chromium(
          0,
          chromeConnection,
          chromiumLauncher: chromiumLauncher,
          process: process,
          logger: logger,
        );

        final Future<void> closeFuture = chrome.close();
        fakeAsync.elapse(const Duration(seconds: 7));
        await closeFuture;

        expect(process.signals, <io.ProcessSignal>[
          io.ProcessSignal.sigterm,
          io.ProcessSignal.sigkill,
        ]);
      }();
    });

    fakeAsync.flushTimers();
    expect(
      logger.warningText,
      'Failed to exit Chromium (pid: 1234) using SIGTERM. Will try sending SIGKILL instead.\n',
    );
  });

  testWithoutContext('falls back to a warning if SIGKILL did not work', () async {
    final BufferLogger logger = BufferLogger.test();
    final FakeAsync fakeAsync = FakeAsync();

    fakeAsync.run((_) {
      () async {
        final FakeChromeConnection chromeConnection = FakeChromeConnection(maxRetries: 4);
        final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
          fileSystem: fileSystem,
          platform: platform,
          processManager: processManager,
          operatingSystemUtils: operatingSystemUtils,
          browserFinder: findChromeExecutable,
          logger: logger,
        );

        final FakeProcess process = FakeProcess(duration: const Duration(seconds: 20));

        final Chromium chrome = Chromium(
          0,
          chromeConnection,
          chromiumLauncher: chromiumLauncher,
          process: process,
          logger: logger,
        );

        final Future<void> closeFuture = chrome.close();
        fakeAsync.elapse(const Duration(seconds: 30));
        await closeFuture;
        expect(process.signals, <io.ProcessSignal>[
          io.ProcessSignal.sigterm,
          io.ProcessSignal.sigkill,
        ]);
      }();
    });

    fakeAsync.flushTimers();
    expect(
      logger.warningText,
      'Failed to exit Chromium (pid: 1234) using SIGTERM. Will try sending SIGKILL instead.\n'
      'Failed to exit Chromium (pid: 1234) using SIGKILL. Giving up. Will continue, assuming '
      'Chromium has exited successfully, but it is possible that this left a dangling Chromium '
      'process running on the system.\n',
    );
  });

  testWithoutContext(
    'does not crash if saving profile information fails due to a file system exception.',
    () async {
      final BufferLogger logger = BufferLogger.test();
      chromeLauncher = ChromiumLauncher(
        fileSystem: fileSystem,
        platform: platform,
        processManager: processManager,
        operatingSystemUtils: operatingSystemUtils,
        browserFinder: findChromeExecutable,
        logger: logger,
      );
      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'example_chrome',
            '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
            '--remote-debugging-port=12345',
            ...kChromeArgs,
            'example_url',
          ],
          stderr: kDevtoolsStderr,
        ),
      );

      final Chromium chrome = await chromeLauncher.launch(
        'example_url',
        skipCheck: true,
        cacheDir: fileSystem.currentDirectory,
      );

      // Create cache dir that the Chrome launcher will attempt to persist, and a file
      // that will thrown an exception when it is read.
      const String directoryPrefix = '/.tmp_rand0/flutter_tools_chrome_device.rand0/Default';
      fileSystem.directory('$directoryPrefix/Local Storage').createSync(recursive: true);
      final File file = fileSystem.file('$directoryPrefix/Local Storage/foo')
        ..createSync(recursive: true);
      exceptionHandler.addError(file, FileSystemOp.read, const FileSystemException());

      await chrome.close(); // does not exit with error.
      expect(logger.errorText, contains('Failed to save Chrome preferences'));
    },
  );

  testWithoutContext(
    'does not crash if restoring profile information fails due to a file system exception.',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final File file = fileSystem.file('/Default/foo')..createSync(recursive: true);
      exceptionHandler.addError(file, FileSystemOp.read, const FileSystemException());
      chromeLauncher = ChromiumLauncher(
        fileSystem: fileSystem,
        platform: platform,
        processManager: processManager,
        operatingSystemUtils: operatingSystemUtils,
        browserFinder: findChromeExecutable,
        logger: logger,
      );

      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'example_chrome',
            '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
            '--remote-debugging-port=12345',
            ...kChromeArgs,
            'example_url',
          ],
          stderr: kDevtoolsStderr,
        ),
      );

      fileSystem.currentDirectory.childDirectory('Default').createSync();
      final Chromium chrome = await chromeLauncher.launch(
        'example_url',
        skipCheck: true,
        cacheDir: fileSystem.currentDirectory,
      );

      // Create cache dir that the Chrome launcher will attempt to persist.
      fileSystem
          .directory('/.tmp_rand0/flutter_tools_chrome_device.rand0/Default/Local Storage')
          .createSync(recursive: true);

      await chrome.close(); // does not exit with error.
      expect(logger.errorText, contains('Failed to restore Chrome preferences'));
    },
  );

  testWithoutContext('can launch Chrome on x86_64 macOS', () async {
    final OperatingSystemUtils macOSUtils = FakeOperatingSystemUtils(
      hostPlatform: HostPlatform.darwin_x64,
    );
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: macOSUtils,
      browserFinder: findChromeExecutable,
      logger: BufferLogger.test(),
    );

    processManager.addCommands(<FakeCommand>[
      const FakeCommand(
        command: <String>[
          'example_chrome',
          '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
          '--remote-debugging-port=12345',
          ...kChromeArgs,
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    ]);

    await expectReturnsNormallyLater(chromiumLauncher.launch('example_url', skipCheck: true));
  });

  testWithoutContext('can launch x86_64 Chrome on ARM macOS', () async {
    final OperatingSystemUtils macOSUtils = FakeOperatingSystemUtils(
      hostPlatform: HostPlatform.darwin_arm64,
    );
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: macOSUtils,
      browserFinder: findChromeExecutable,
      logger: BufferLogger.test(),
    );

    processManager.addCommands(<FakeCommand>[
      const FakeCommand(
        command: <String>['file', 'example_chrome'],
        stdout: 'Mach-O 64-bit executable x86_64',
      ),
      const FakeCommand(
        command: <String>[
          'example_chrome',
          '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
          '--remote-debugging-port=12345',
          ...kChromeArgs,
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    ]);

    await expectReturnsNormallyLater(chromiumLauncher.launch('example_url', skipCheck: true));
  });

  testWithoutContext('can launch ARM Chrome natively on ARM macOS when installed', () async {
    final OperatingSystemUtils macOSUtils = FakeOperatingSystemUtils(
      hostPlatform: HostPlatform.darwin_arm64,
    );
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: macOSUtils,
      browserFinder: findChromeExecutable,
      logger: BufferLogger.test(),
    );

    processManager.addCommands(<FakeCommand>[
      const FakeCommand(
        command: <String>['file', 'example_chrome'],
        stdout: 'Mach-O 64-bit executable arm64',
      ),
      const FakeCommand(
        command: <String>[
          '/usr/bin/arch',
          '-arm64',
          'example_chrome',
          '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
          '--remote-debugging-port=12345',
          ...kChromeArgs,
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    ]);

    await expectReturnsNormallyLater(chromiumLauncher.launch('example_url', skipCheck: true));
  });

  testWithoutContext('can launch chrome with a custom debug port', () async {
    processManager.addCommand(
      const FakeCommand(
        command: <String>[
          'example_chrome',
          '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
          '--remote-debugging-port=10000',
          ...kChromeArgs,
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    );

    await expectReturnsNormallyLater(
      chromeLauncher.launch('example_url', skipCheck: true, debugPort: 10000),
    );
  });

  testWithoutContext('can launch chrome with arbitrary flags', () async {
    processManager.addCommand(
      const FakeCommand(
        command: <String>[
          'example_chrome',
          '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
          '--remote-debugging-port=12345',
          ...kChromeArgs,
          '--autoplay-policy=no-user-gesture-required',
          '--incognito',
          '--auto-select-desktop-capture-source="Entire screen"',
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    );

    await expectReturnsNormallyLater(
      chromeLauncher.launch(
        'example_url',
        skipCheck: true,
        webBrowserFlags: <String>[
          '--autoplay-policy=no-user-gesture-required',
          '--incognito',
          '--auto-select-desktop-capture-source="Entire screen"',
        ],
      ),
    );
  });

  testWithoutContext('can launch chrome headless', () async {
    processManager.addCommand(
      const FakeCommand(
        command: <String>[
          'example_chrome',
          '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
          '--remote-debugging-port=12345',
          ...kChromeArgs,
          '--headless',
          '--disable-gpu',
          '--no-sandbox',
          '--window-size=2400,1800',
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    );

    await expectReturnsNormallyLater(
      chromeLauncher.launch('example_url', skipCheck: true, headless: true),
    );
  });

  testWithoutContext(
    'can seed chrome temp directory with existing session data, excluding Cache folder',
    () async {
      final Completer<void> exitCompleter = Completer<void>.sync();
      final Directory dataDir = fileSystem.directory('chrome-stuff');
      final File preferencesFile = dataDir.childDirectory('Default').childFile('preferences');
      preferencesFile
        ..createSync(recursive: true)
        ..writeAsStringSync('"exit_type":"Crashed"');

      final Directory defaultContentDirectory = dataDir
          .childDirectory('Default')
          .childDirectory('Foo');
      defaultContentDirectory.createSync(recursive: true);
      // Create Cache directories that should be skipped
      for (final String cache in kCodeCache) {
        dataDir.childDirectory('Default').childDirectory(cache).createSync(recursive: true);
      }

      processManager.addCommand(
        FakeCommand(
          command: const <String>[
            'example_chrome',
            '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
            '--remote-debugging-port=12345',
            ...kChromeArgs,
            'example_url',
          ],
          completer: exitCompleter,
          stderr: kDevtoolsStderr,
        ),
      );

      await chromeLauncher.launch('example_url', skipCheck: true, cacheDir: dataDir);

      // validate any Default content is copied
      final Directory defaultContentDir = fileSystem
          .directory('.tmp_rand0/flutter_tools_chrome_device.rand0')
          .childDirectory('Default')
          .childDirectory('Foo');

      expect(defaultContentDir, exists);

      exitCompleter.complete();
      await Future<void>.delayed(const Duration(milliseconds: 1));

      // writes non-crash back to dart_tool
      expect(preferencesFile.readAsStringSync(), '"exit_type":"Normal"');

      // Validate cache dirs are not copied.
      for (final String cache in kCodeCache) {
        expect(
          fileSystem
              .directory('.tmp_rand0/flutter_tools_chrome_device.rand0')
              .childDirectory('Default')
              .childDirectory(cache),
          isNot(exists),
        );
      }

      // validate defaultContentDir is deleted after exit, data is in cache
      expect(defaultContentDir, isNot(exists));
    },
  );

  testWithoutContext('can retry launch when glibc bug happens', () async {
    const List<String> args = <String>[
      'example_chrome',
      '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
      '--remote-debugging-port=12345',
      ...kChromeArgs,
      '--headless',
      '--disable-gpu',
      '--no-sandbox',
      '--window-size=2400,1800',
      'example_url',
    ];

    // Pretend to hit glibc bug 3 times.
    for (int i = 0; i < 3; i++) {
      processManager.addCommand(
        const FakeCommand(
          command: args,
          stderr:
              'Inconsistency detected by ld.so: ../elf/dl-tls.c: 493: '
              '_dl_allocate_tls_init: Assertion `listp->slotinfo[cnt].gen '
              "<= GL(dl_tls_generation)' failed!",
        ),
      );
    }

    // Succeed on the 4th try.
    processManager.addCommand(const FakeCommand(command: args, stderr: kDevtoolsStderr));

    await expectReturnsNormallyLater(
      chromeLauncher.launch('example_url', skipCheck: true, headless: true),
    );
  });

  testWithoutContext('can retry launch when chrome fails to start', () async {
    const List<String> args = <String>[
      'example_chrome',
      '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
      '--remote-debugging-port=12345',
      ...kChromeArgs,
      '--headless',
      '--disable-gpu',
      '--no-sandbox',
      '--window-size=2400,1800',
      'example_url',
    ];

    // Pretend to random error 3 times.
    for (int i = 0; i < 3; i++) {
      processManager.addCommand(const FakeCommand(command: args, stderr: 'BLAH BLAH'));
    }

    // Succeed on the 4th try.
    processManager.addCommand(const FakeCommand(command: args, stderr: kDevtoolsStderr));

    await expectReturnsNormallyLater(
      chromeLauncher.launch('example_url', skipCheck: true, headless: true),
    );
  });

  testWithoutContext('gives up retrying when an error happens more than 3 times', () async {
    final BufferLogger logger = BufferLogger.test();
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: logger,
    );
    for (int i = 0; i < 4; i++) {
      processManager.addCommand(
        const FakeCommand(
          command: <String>[
            'example_chrome',
            '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
            '--remote-debugging-port=12345',
            ...kChromeArgs,
            '--headless',
            '--disable-gpu',
            '--no-sandbox',
            '--window-size=2400,1800',
            'example_url',
          ],
          stderr: 'nothing in the std error indicating glibc error',
        ),
      );
    }

    await expectToolExitLater(
      chromiumLauncher.launch('example_url', skipCheck: true, headless: true),
      contains('Failed to launch browser.'),
    );
    expect(logger.errorText, contains('nothing in the std error indicating glibc error'));
  });

  testWithoutContext('Logs an error and exits if connection check fails.', () async {
    final BufferLogger logger = BufferLogger.test();
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: logger,
    );
    processManager.addCommand(
      const FakeCommand(
        command: <String>[
          'example_chrome',
          '--user-data-dir=/.tmp_rand0/flutter_tools_chrome_device.rand0',
          '--remote-debugging-port=12345',
          ...kChromeArgs,
          'example_url',
        ],
        stderr: kDevtoolsStderr,
      ),
    );

    await expectToolExitLater(
      chromiumLauncher.launch('example_url'),
      contains('Unable to connect to Chrome debug port:'),
    );
    expect(logger.errorText, contains('SocketException'));
  });

  testWithoutContext('can recover if getTabs throws a connection exception', () async {
    final BufferLogger logger = BufferLogger.test();
    final FakeChromeConnection chromeConnection = FakeChromeConnection(maxRetries: 4);
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: logger,
    );
    final FakeProcess process = FakeProcess();
    final Chromium chrome = Chromium(
      0,
      chromeConnection,
      chromiumLauncher: chromiumLauncher,
      process: process,
      logger: logger,
    );
    expect(await chromiumLauncher.connect(chrome, false), equals(chrome));
    expect(logger.errorText, isEmpty);
  });

  testWithoutContext('can recover if getTabs throws an HttpException', () async {
    final BufferLogger logger = BufferLogger.test();
    final FakeChromeConnection chromeConnection = FakeChromeConnection(
      maxRetries: 4,
      error: io.HttpException(
        'Connection closed before full header was received',
        uri: Uri.parse('http://localhost:52097/json'),
      ),
    );
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: logger,
    );
    final FakeProcess process = FakeProcess();
    final Chromium chrome = Chromium(
      0,
      chromeConnection,
      chromiumLauncher: chromiumLauncher,
      process: process,
      logger: logger,
    );
    expect(await chromiumLauncher.connect(chrome, false), equals(chrome));
    expect(logger.errorText, isEmpty);
  });

  testWithoutContext('chrome.close can recover if getTab throws a StateError', () async {
    final BufferLogger logger = BufferLogger.test();
    final FakeChromeConnectionWithTab chromeConnection = FakeChromeConnectionWithTab(
      onGetTab: () {
        throw StateError('Client is closed.');
      },
    );
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: logger,
    );
    final FakeProcess process = FakeProcess();
    final Chromium chrome = Chromium(
      0,
      chromeConnection,
      chromiumLauncher: chromiumLauncher,
      process: process,
      logger: logger,
    );
    await chromiumLauncher.connect(chrome, false);
    await chrome.close();
    expect(logger.errorText, isEmpty);
  });

  testWithoutContext('exits if getTabs throws a connection exception consistently', () async {
    final BufferLogger logger = BufferLogger.test();
    final FakeChromeConnection chromeConnection = FakeChromeConnection();
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: logger,
    );
    final FakeProcess process = FakeProcess();
    final Chromium chrome = Chromium(
      0,
      chromeConnection,
      chromiumLauncher: chromiumLauncher,
      process: process,
      logger: logger,
    );
    await expectToolExitLater(
      chromiumLauncher.connect(chrome, false),
      allOf(contains('Unable to connect to Chrome debug port'), contains('incorrect format')),
    );
    expect(
      logger.errorText,
      allOf(contains('incorrect format'), contains('OK'), contains('<html> ...')),
    );
  });

  testWithoutContext('Chromium close sends browser close command', () async {
    final BufferLogger logger = BufferLogger.test();
    final List<String> commands = <String>[];
    void onSendCommand(String cmd) {
      commands.add(cmd);
    }

    final FakeChromeConnectionWithTab chromeConnection = FakeChromeConnectionWithTab(
      onSendCommand: onSendCommand,
    );
    final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
      fileSystem: fileSystem,
      platform: platform,
      processManager: processManager,
      operatingSystemUtils: operatingSystemUtils,
      browserFinder: findChromeExecutable,
      logger: logger,
    );
    final FakeProcess process = FakeProcess();
    final Chromium chrome = Chromium(
      0,
      chromeConnection,
      chromiumLauncher: chromiumLauncher,
      process: process,
      logger: logger,
    );
    expect(await chromiumLauncher.connect(chrome, false), equals(chrome));
    await chrome.close();
    expect(commands, contains('Browser.close'));
  });

  testWithoutContext(
    'Chromium close handles a SocketException when connecting to Chrome',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FakeChromeConnectionWithTab chromeConnection = FakeChromeConnectionWithTab();
      final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
        fileSystem: fileSystem,
        platform: platform,
        processManager: processManager,
        operatingSystemUtils: operatingSystemUtils,
        browserFinder: findChromeExecutable,
        logger: logger,
      );
      final FakeProcess process = FakeProcess();
      final Chromium chrome = Chromium(
        0,
        chromeConnection,
        chromiumLauncher: chromiumLauncher,
        process: process,
        logger: logger,
      );
      expect(await chromiumLauncher.connect(chrome, false), equals(chrome));
      chromeConnection.throwSocketExceptions = true;
      await chrome.close();
    },
  );

  testWithoutContext(
    'Chromium close handles a WebSocketException when closing the WipConnection',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FakeChromeConnectionWithTab chromeConnection = FakeChromeConnectionWithTab(
        throwWebSocketException: true,
      );
      final ChromiumLauncher chromiumLauncher = ChromiumLauncher(
        fileSystem: fileSystem,
        platform: platform,
        processManager: processManager,
        operatingSystemUtils: operatingSystemUtils,
        browserFinder: findChromeExecutable,
        logger: logger,
      );
      final FakeProcess process = FakeProcess();
      final Chromium chrome = Chromium(
        0,
        chromeConnection,
        chromiumLauncher: chromiumLauncher,
        process: process,
        logger: logger,
      );
      expect(await chromiumLauncher.connect(chrome, false), equals(chrome));
      await chrome.close();
    },
  );
}

/// Fake chrome connection that fails to get tabs a few times.
class FakeChromeConnection extends Fake implements ChromeConnection {
  /// Create a connection that throws a connection exception on first
  /// [maxRetries] calls to [getTabs].
  /// If [maxRetries] is `null`, [getTabs] calls never succeed.
  FakeChromeConnection({this.maxRetries, Object? error}) : _retries = 0 {
    this.error =
        error ??
        ConnectionException(
          formatException: const FormatException('incorrect format'),
          responseStatus: 'OK,',
          responseBody: '<html> ...',
        );
  }

  final List<ChromeTab> tabs = <ChromeTab>[];
  final int? maxRetries;
  int _retries;
  late final Object error;

  @override
  Future<ChromeTab?> getTab(bool Function(ChromeTab tab) accept, {Duration? retryFor}) async {
    return tabs.firstWhere(accept);
  }

  @override
  Future<List<ChromeTab>> getTabs({Duration? retryFor}) async {
    _retries++;
    if (maxRetries == null || _retries < maxRetries!) {
      // ignore: only_throw_errors -- This is fine for an ad-hoc test.
      throw error;
    }
    return tabs;
  }

  @override
  void close() {}
}

typedef OnSendCommand = void Function(String);

/// Fake chrome connection that returns a tab.
class FakeChromeConnectionWithTab extends Fake implements ChromeConnection {
  FakeChromeConnectionWithTab({
    OnSendCommand? onSendCommand,
    this.onGetTab,
    bool throwWebSocketException = false,
  }) : _tab = FakeChromeTab(onSendCommand, throwWebSocketException);

  final FakeChromeTab _tab;
  void Function()? onGetTab;
  bool throwSocketExceptions = false;

  @override
  Future<ChromeTab?> getTab(bool Function(ChromeTab tab) accept, {Duration? retryFor}) async {
    onGetTab?.call();
    if (throwSocketExceptions) {
      throw const io.SocketException('test');
    }
    return _tab;
  }

  @override
  Future<List<ChromeTab>> getTabs({Duration? retryFor}) async {
    if (throwSocketExceptions) {
      throw const io.SocketException('test');
    }
    return <ChromeTab>[_tab];
  }

  @override
  void close() {}
}

class FakeChromeTab extends Fake implements ChromeTab {
  FakeChromeTab(this.onSendCommand, this.throwWebSocketException);

  final OnSendCommand? onSendCommand;
  final bool throwWebSocketException;

  @override
  Future<WipConnection> connect({Function? onError}) async {
    return FakeWipConnection(onSendCommand, throwWebSocketException);
  }
}

class FakeWipConnection extends Fake implements WipConnection {
  FakeWipConnection(this.onSendCommand, this.throwWebSocketException);

  final OnSendCommand? onSendCommand;
  final bool throwWebSocketException;

  @override
  Future<WipResponse> sendCommand(String method, [Map<String, dynamic>? params]) async {
    onSendCommand?.call(method);
    return WipResponse(<String, dynamic>{'id': 0, 'result': <String, dynamic>{}});
  }

  @override
  Future<void> close() async {
    if (throwWebSocketException) {
      throw const io.WebSocketException('test');
    }
  }
}
