// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/net.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/time.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/drive/web_driver_service.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/web/web_runner.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:webdriver/sync_io.dart' as sync_io;

import '../../src/common.dart';
import '../../src/context.dart';

const List<String> kChromeArgs = <String>[
  '--bwsi',
  '--disable-background-timer-throttling',
  '--disable-default-apps',
  '--disable-extensions',
  '--disable-popup-blocking',
  '--disable-translate',
  '--no-default-browser-check',
  '--no-sandbox',
  '--no-first-run',
];

void main() {
  testWithoutContext('getDesiredCapabilities Chrome with headless on', () {
    final Map<String, dynamic> expected = <String, dynamic>{
      'acceptInsecureCerts': true,
      'browserName': 'chrome',
      'goog:loggingPrefs': <String, String>{
        sync_io.LogType.browser: 'INFO',
        sync_io.LogType.performance: 'ALL',
      },
      'goog:chromeOptions': <String, dynamic>{
        'w3c': true,
        'args': <String>[...kChromeArgs, '--headless'],
        'perfLoggingPrefs': <String, String>{
          'traceCategories':
              'devtools.timeline,'
              'v8,blink.console,benchmark,blink,'
              'blink.user_timing',
        },
      },
    };

    expect(getDesiredCapabilities(Browser.chrome, true), expected);
  });

  testWithoutContext('getDesiredCapabilities Chrome with headless off', () {
    const String chromeBinary = 'random-binary';
    final Map<String, dynamic> expected = <String, dynamic>{
      'acceptInsecureCerts': true,
      'browserName': 'chrome',
      'goog:loggingPrefs': <String, String>{
        sync_io.LogType.browser: 'INFO',
        sync_io.LogType.performance: 'ALL',
      },
      'goog:chromeOptions': <String, dynamic>{
        'binary': chromeBinary,
        'w3c': true,
        'args': kChromeArgs,
        'perfLoggingPrefs': <String, String>{
          'traceCategories':
              'devtools.timeline,'
              'v8,blink.console,benchmark,blink,'
              'blink.user_timing',
        },
      },
    };

    expect(getDesiredCapabilities(Browser.chrome, false, chromeBinary: chromeBinary), expected);
  });

  testWithoutContext('getDesiredCapabilities Chrome with browser flags', () {
    const List<String> webBrowserFlags = <String>[
      '--autoplay-policy=no-user-gesture-required',
      '--incognito',
      '--auto-select-desktop-capture-source="Entire screen"',
    ];
    final Map<String, dynamic> expected = <String, dynamic>{
      'acceptInsecureCerts': true,
      'browserName': 'chrome',
      'goog:loggingPrefs': <String, String>{
        sync_io.LogType.browser: 'INFO',
        sync_io.LogType.performance: 'ALL',
      },
      'goog:chromeOptions': <String, dynamic>{
        'w3c': true,
        'args': <String>[
          ...kChromeArgs,
          '--autoplay-policy=no-user-gesture-required',
          '--incognito',
          '--auto-select-desktop-capture-source="Entire screen"',
        ],
        'perfLoggingPrefs': <String, String>{
          'traceCategories':
              'devtools.timeline,'
              'v8,blink.console,benchmark,blink,'
              'blink.user_timing',
        },
      },
    };

    expect(
      getDesiredCapabilities(Browser.chrome, false, webBrowserFlags: webBrowserFlags),
      expected,
    );
  });

  testWithoutContext('getDesiredCapabilities Firefox with headless on', () {
    final Map<String, dynamic> expected = <String, dynamic>{
      'acceptInsecureCerts': true,
      'browserName': 'firefox',
      'moz:firefoxOptions': <String, dynamic>{
        'args': <String>['-headless'],
        'prefs': <String, dynamic>{
          'dom.file.createInChild': true,
          'dom.timeout.background_throttling_max_budget': -1,
          'media.autoplay.default': 0,
          'media.gmp-manager.url': '',
          'media.gmp-provider.enabled': false,
          'network.captive-portal-service.enabled': false,
          'security.insecure_field_warning.contextual.enabled': false,
          'test.currentTimeOffsetSeconds': 11491200,
        },
        'log': <String, String>{'level': 'trace'},
      },
    };

    expect(getDesiredCapabilities(Browser.firefox, true), expected);
  });

  testWithoutContext('getDesiredCapabilities Firefox with headless off', () {
    final Map<String, dynamic> expected = <String, dynamic>{
      'acceptInsecureCerts': true,
      'browserName': 'firefox',
      'moz:firefoxOptions': <String, dynamic>{
        'args': <String>[],
        'prefs': <String, dynamic>{
          'dom.file.createInChild': true,
          'dom.timeout.background_throttling_max_budget': -1,
          'media.autoplay.default': 0,
          'media.gmp-manager.url': '',
          'media.gmp-provider.enabled': false,
          'network.captive-portal-service.enabled': false,
          'security.insecure_field_warning.contextual.enabled': false,
          'test.currentTimeOffsetSeconds': 11491200,
        },
        'log': <String, String>{'level': 'trace'},
      },
    };

    expect(getDesiredCapabilities(Browser.firefox, false), expected);
  });

  testWithoutContext('getDesiredCapabilities Firefox with browser flags', () {
    const List<String> webBrowserFlags = <String>['-url=https://example.com', '-private'];
    final Map<String, dynamic> expected = <String, dynamic>{
      'acceptInsecureCerts': true,
      'browserName': 'firefox',
      'moz:firefoxOptions': <String, dynamic>{
        'args': <String>['-url=https://example.com', '-private'],
        'prefs': <String, dynamic>{
          'dom.file.createInChild': true,
          'dom.timeout.background_throttling_max_budget': -1,
          'media.autoplay.default': 0,
          'media.gmp-manager.url': '',
          'media.gmp-provider.enabled': false,
          'network.captive-portal-service.enabled': false,
          'security.insecure_field_warning.contextual.enabled': false,
          'test.currentTimeOffsetSeconds': 11491200,
        },
        'log': <String, String>{'level': 'trace'},
      },
    };

    expect(
      getDesiredCapabilities(Browser.firefox, false, webBrowserFlags: webBrowserFlags),
      expected,
    );
  });

  testWithoutContext('getDesiredCapabilities Edge', () {
    final Map<String, dynamic> expected = <String, dynamic>{
      'acceptInsecureCerts': true,
      'browserName': 'edge',
    };

    expect(getDesiredCapabilities(Browser.edge, false), expected);
  });

  testWithoutContext('getDesiredCapabilities macOS Safari', () {
    final Map<String, dynamic> expected = <String, dynamic>{'browserName': 'safari'};

    expect(getDesiredCapabilities(Browser.safari, false), expected);
  });

  testWithoutContext('getDesiredCapabilities iOS Safari', () {
    final Map<String, dynamic> expected = <String, dynamic>{
      'platformName': 'ios',
      'browserName': 'safari',
      'safari:useSimulator': true,
    };

    expect(getDesiredCapabilities(Browser.iosSafari, false), expected);
  });

  testWithoutContext('getDesiredCapabilities android chrome', () {
    const List<String> webBrowserFlags = <String>[
      '--autoplay-policy=no-user-gesture-required',
      '--incognito',
    ];
    final Map<String, dynamic> expected = <String, dynamic>{
      'browserName': 'chrome',
      'platformName': 'android',
      'goog:chromeOptions': <String, dynamic>{
        'androidPackage': 'com.android.chrome',
        'args': <String>[
          '--disable-fullscreen',
          '--autoplay-policy=no-user-gesture-required',
          '--incognito',
        ],
      },
    };

    expect(
      getDesiredCapabilities(Browser.androidChrome, false, webBrowserFlags: webBrowserFlags),
      expected,
    );
  });

  testUsingContext(
    'WebDriverService starts and stops an app',
    () async {
      final WebDriverService service = setUpDriverService();
      final FakeDevice device = FakeDevice();
      await service.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
      );
      await service.stop();
      expect(FakeResidentRunner.instance.callLog, <String>['run', 'exitApp', 'cleanupAtFinish']);
    },
    overrides: <Type, Generator>{WebRunnerFactory: () => FakeWebRunnerFactory()},
  );

  testUsingContext(
    'WebDriverService can start an app with a launch url provided',
    () async {
      final WebDriverService service = setUpDriverService();
      final FakeDevice device = FakeDevice();
      const String testUrl = 'http://localhost:1234/test';
      await service.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, webLaunchUrl: testUrl, ipv6: true),
      );
      await service.stop();
      expect(service.webUri, Uri.parse(testUrl));
    },
    overrides: <Type, Generator>{WebRunnerFactory: () => FakeWebRunnerFactory()},
  );

  testUsingContext(
    'WebDriverService starts an app with provided web headers',
    () async {
      final WebDriverService service = setUpDriverService();
      final FakeDevice device = FakeDevice();
      final Map<String, String> webHeaders = <String, String>{'test-header': 'test-value'};
      await service.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, webHeaders: webHeaders, ipv6: true),
      );
      await service.stop();
      expect(FakeResidentRunner.instance.debuggingOptions.webHeaders, equals(webHeaders));
    },
    overrides: <Type, Generator>{WebRunnerFactory: () => FakeWebRunnerFactory()},
  );

  testUsingContext(
    'WebDriverService will throw when an invalid launch url is provided',
    () async {
      final WebDriverService service = setUpDriverService();
      final FakeDevice device = FakeDevice();
      const String invalidTestUrl = '::INVALID_URL::';
      await expectLater(
        service.start(
          BuildInfo.profile,
          device,
          DebuggingOptions.enabled(BuildInfo.profile, webLaunchUrl: invalidTestUrl, ipv6: true),
        ),
        throwsA(isA<FormatException>()),
      );
    },
    overrides: <Type, Generator>{WebRunnerFactory: () => FakeWebRunnerFactory()},
  );

  testUsingContext(
    'WebDriverService forwards exception when run future fails before app starts',
    () async {
      final WebDriverService service = setUpDriverService();
      final Device device = FakeDevice();
      await expectLater(
        service.start(
          BuildInfo.profile,
          device,
          DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
        ),
        throwsA('This is a test error'),
      );
    },
    overrides: <Type, Generator>{
      WebRunnerFactory: () => FakeWebRunnerFactory(doResolveToError: true),
    },
  );
}

class FakeWebRunnerFactory implements WebRunnerFactory {
  FakeWebRunnerFactory({this.doResolveToError = false});

  final bool doResolveToError;

  @override
  ResidentRunner createWebRunner(
    FlutterDevice device, {
    String? target,
    bool? stayResident,
    FlutterProject? flutterProject,
    bool? ipv6,
    required DebuggingOptions debuggingOptions,
    UrlTunneller? urlTunneller,
    Logger? logger,
    FileSystem? fileSystem,
    SystemClock? systemClock,
    Usage? usage,
    Analytics? analytics,
    bool machine = false,
  }) {
    expect(stayResident, isTrue);
    return FakeResidentRunner(
      doResolveToError: doResolveToError,
      debuggingOptions: debuggingOptions,
    );
  }
}

class FakeResidentRunner extends Fake implements ResidentRunner {
  FakeResidentRunner({required this.doResolveToError, required this.debuggingOptions}) {
    instance = this;
  }

  static late FakeResidentRunner instance;

  final bool doResolveToError;
  @override
  final DebuggingOptions debuggingOptions;
  final Completer<int> _exitCompleter = Completer<int>();
  final List<String> callLog = <String>[];

  @override
  Uri get uri => Uri();

  @override
  Future<int> run({
    Completer<DebugConnectionInfo>? connectionInfoCompleter,
    Completer<void>? appStartedCompleter,
    bool enableDevTools = false,
    String? route,
  }) async {
    callLog.add('run');

    if (doResolveToError) {
      return Future<int>.error('This is a test error');
    }

    appStartedCompleter?.complete();
    // Emulate stayResident by completing after exitApp is called.
    return _exitCompleter.future;
  }

  @override
  Future<void> exitApp() async {
    callLog.add('exitApp');
    _exitCompleter.complete(0);
  }

  @override
  Future<void> cleanupAtFinish() async {
    callLog.add('cleanupAtFinish');
  }
}

WebDriverService setUpDriverService() {
  final BufferLogger logger = BufferLogger.test();
  return WebDriverService(
    logger: logger,
    processUtils: ProcessUtils(logger: logger, processManager: FakeProcessManager.any()),
    dartSdkPath: 'dart',
  );
}

class FakeDevice extends Fake implements Device {
  @override
  final PlatformType platformType = PlatformType.web;

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.android_arm;
}
