// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:file/file.dart';
import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';
import 'package:webdriver/async_io.dart' as async_io;

import '../base/common.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../convert.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../resident_runner.dart';
import '../web/web_runner.dart';
import 'drive_service.dart';

/// An implementation of the driver service for web debug and release applications.
class WebDriverService extends DriverService {
  WebDriverService({
    required ProcessUtils processUtils,
    required String dartSdkPath,
    required Platform platform,
    required Logger logger,
    required Terminal terminal,
    required OutputPreferences outputPreferences,
  }) : _processUtils = processUtils,
       _dartSdkPath = dartSdkPath,
       _platform = platform,
       _logger = logger,
       _terminal = terminal,
       _outputPreferences = outputPreferences;

  final ProcessUtils _processUtils;
  final String _dartSdkPath;
  final Platform _platform;
  final Logger _logger;
  final Terminal _terminal;
  final OutputPreferences _outputPreferences;

  late ResidentRunner _residentRunner;
  Uri? _webUri;

  @visibleForTesting
  Uri? get webUri => _webUri;

  /// The result of [ResidentRunner.run].
  ///
  /// This is expected to stay `null` throughout the test, as the application
  /// must be running until [stop] is called. If it becomes non-null, it likely
  /// indicates a bug.
  int? _runResult;

  @override
  Future<void> start(
    BuildInfo buildInfo,
    Device device,
    DebuggingOptions debuggingOptions, {
    File? applicationBinary,
    String? route,
    String? userIdentifier,
    String? mainPath,
    Map<String, Object> platformArgs = const <String, Object>{},
  }) async {
    final FlutterDevice flutterDevice = await FlutterDevice.create(
      device,
      target: mainPath,
      buildInfo: buildInfo,
      platform: globals.platform,
    );
    _residentRunner = webRunnerFactory!.createWebRunner(
      flutterDevice,
      target: mainPath,
      debuggingOptions:
          buildInfo.isRelease
              ? DebuggingOptions.disabled(
                buildInfo,
                port: debuggingOptions.port,
                hostname: debuggingOptions.hostname,
                webRenderer: debuggingOptions.webRenderer,
                webUseWasm: debuggingOptions.webUseWasm,
                webHeaders: debuggingOptions.webHeaders,
              )
              : DebuggingOptions.enabled(
                buildInfo,
                port: debuggingOptions.port,
                hostname: debuggingOptions.hostname,
                disablePortPublication: debuggingOptions.disablePortPublication,
                webRenderer: debuggingOptions.webRenderer,
                webUseWasm: debuggingOptions.webUseWasm,
                webHeaders: debuggingOptions.webHeaders,
              ),
      stayResident: true,
      flutterProject: FlutterProject.current(),
      fileSystem: globals.fs,
      analytics: globals.analytics,
      logger: _logger,
      terminal: _terminal,
      platform: _platform,
      outputPreferences: _outputPreferences,
      systemClock: globals.systemClock,
    );
    final Completer<void> appStartedCompleter = Completer<void>.sync();
    final Future<int?> runFuture = _residentRunner.run(
      appStartedCompleter: appStartedCompleter,
      route: route,
    );

    bool isAppStarted = false;
    await Future.any(<Future<Object?>>[
      runFuture.then((int? result) {
        _runResult = result;
        return null;
      }),
      appStartedCompleter.future.then((_) {
        isAppStarted = true;
        return null;
      }),
    ]);

    if (_runResult != null) {
      throwToolExit(
        'Application exited before the test started. Check web driver logs '
        'for possible application-side errors.',
      );
    }

    if (!isAppStarted) {
      throwToolExit('Failed to start application');
    }

    if (_residentRunner.uri == null) {
      throwToolExit('Unable to connect to the app. URL not available.');
    }

    if (debuggingOptions.webLaunchUrl != null) {
      // It should throw an error if the provided url is invalid so no tryParse
      _webUri = Uri.parse(debuggingOptions.webLaunchUrl!);
    } else {
      _webUri = _residentRunner.uri;
    }
  }

  @override
  Future<int> startTest(
    String testFile,
    List<String> arguments,
    PackageConfig packageConfig, {
    bool? headless,
    String? chromeBinary,
    String? browserName,
    bool? androidEmulator,
    int? driverPort,
    List<String> webBrowserFlags = const <String>[],
    List<String>? browserDimension,
    String? profileMemory,
  }) async {
    late async_io.WebDriver webDriver;
    final Browser browser = Browser.fromCliName(browserName);
    try {
      webDriver = await async_io.createDriver(
        uri: Uri.parse('http://localhost:$driverPort/'),
        desired: getDesiredCapabilities(
          browser,
          headless,
          webBrowserFlags: webBrowserFlags,
          chromeBinary: chromeBinary,
        ),
      );
    } on SocketException catch (error) {
      _logger.printTrace('$error');
      throwToolExit(
        'Unable to start a WebDriver session for web testing.\n'
        'Make sure you have the correct WebDriver server (e.g. chromedriver) running at $driverPort.\n'
        'For instructions on how to obtain and run a WebDriver server, see:\n'
        'https://flutter.dev/to/integration-test-on-web\n',
      );
    }

    final bool isAndroidChrome = browser == Browser.androidChrome;
    // Do not set the window size for android chrome browser.
    if (!isAndroidChrome) {
      assert(browserDimension!.length == 2);
      late int x;
      late int y;
      try {
        x = int.parse(browserDimension![0]);
        y = int.parse(browserDimension[1]);
      } on FormatException catch (ex) {
        throwToolExit('Dimension provided to --browser-dimension is invalid: $ex');
      }
      final async_io.Window window = await webDriver.window;
      await window.setLocation(const math.Point<int>(0, 0));
      await window.setSize(math.Rectangle<int>(0, 0, x, y));
    }
    final int result = await _processUtils.stream(
      <String>[_dartSdkPath, ...arguments, testFile],
      environment: <String, String>{
        ..._platform.environment,
        'VM_SERVICE_URL': _webUri.toString(),
        ..._additionalDriverEnvironment(webDriver, browserName, androidEmulator),
      },
    );
    await webDriver.quit();
    return result;
  }

  @override
  Future<void> stop({String? userIdentifier}) async {
    final bool appDidFinishPrematurely = _runResult != null;
    await _residentRunner.exitApp();
    await _residentRunner.cleanupAtFinish();

    if (appDidFinishPrematurely) {
      throwToolExit(
        'Application exited before the test finished. Check web driver logs '
        'for possible application-side errors.',
      );
    }
  }

  Map<String, String> _additionalDriverEnvironment(
    async_io.WebDriver webDriver,
    String? browserName,
    bool? androidEmulator,
  ) {
    return <String, String>{
      'DRIVER_SESSION_ID': webDriver.id,
      'DRIVER_SESSION_URI': webDriver.uri.toString(),
      'DRIVER_SESSION_SPEC': webDriver.spec.toString(),
      'DRIVER_SESSION_CAPABILITIES': json.encode(webDriver.capabilities),
      'SUPPORT_TIMELINE_ACTION': (Browser.fromCliName(browserName) == Browser.chrome).toString(),
      'FLUTTER_WEB_TEST': 'true',
      'ANDROID_CHROME_ON_EMULATOR':
          (Browser.fromCliName(browserName) == Browser.androidChrome && androidEmulator!)
              .toString(),
    };
  }

  @override
  Future<void> reuseApplication(
    Uri vmServiceUri,
    Device device,
    DebuggingOptions debuggingOptions,
  ) async {
    throwToolExit('--use-existing-app is not supported with flutter web driver');
  }
}

/// A list of supported browsers.
enum Browser implements CliEnum {
  /// Chrome on Android: https://developer.chrome.com/docs/multidevice/android/
  androidChrome,

  /// Chrome: https://www.google.com/chrome/
  chrome,

  /// Edge: https://www.microsoft.com/edge
  edge,

  /// Firefox: https://www.mozilla.org/en-US/firefox/
  firefox,

  /// Safari in iOS: https://www.apple.com/safari/
  iosSafari,

  /// Safari in macOS: https://www.apple.com/safari/
  safari;

  @override
  String get helpText => switch (this) {
    Browser.androidChrome => 'Chrome on Android (see also "--android-emulator").',
    Browser.chrome => 'Google Chrome on this computer (see also "--chrome-binary").',
    Browser.edge => 'Microsoft Edge on this computer (Windows only).',
    Browser.firefox => 'Mozilla Firefox on this computer.',
    Browser.iosSafari => 'Apple Safari on an iOS device.',
    Browser.safari => 'Apple Safari on this computer (macOS only).',
  };

  @override
  String get cliName => kebabCase(name);

  static Browser fromCliName(String? value) => Browser.values.singleWhere(
    (Browser element) => element.cliName == value,
    orElse: () => throw UnsupportedError('Browser $value not supported'),
  );
}

/// Returns desired capabilities for given [browser], [headless], [chromeBinary]
/// and [webBrowserFlags].
@visibleForTesting
Map<String, dynamic> getDesiredCapabilities(
  Browser browser,
  bool? headless, {
  List<String> webBrowserFlags = const <String>[],
  String? chromeBinary,
}) => switch (browser) {
  Browser.chrome => <String, dynamic>{
    'acceptInsecureCerts': true,
    'browserName': 'chrome',
    'goog:loggingPrefs': <String, String>{
      async_io.LogType.browser: 'INFO',
      async_io.LogType.performance: 'ALL',
    },
    'goog:chromeOptions': <String, dynamic>{
      if (chromeBinary != null) 'binary': chromeBinary,
      'w3c': true,
      'args': <String>[
        '--bwsi',
        '--disable-background-timer-throttling',
        '--disable-default-apps',
        '--disable-extensions',
        '--disable-popup-blocking',
        '--disable-translate',
        '--no-default-browser-check',
        '--no-sandbox',
        '--no-first-run',
        if (headless!) '--headless',
        ...webBrowserFlags,
      ],
      'perfLoggingPrefs': <String, String>{
        'traceCategories':
            'devtools.timeline,'
            'v8,blink.console,benchmark,blink,'
            'blink.user_timing',
      },
    },
  },
  Browser.firefox => <String, dynamic>{
    'acceptInsecureCerts': true,
    'browserName': 'firefox',
    'moz:firefoxOptions': <String, dynamic>{
      'args': <String>[if (headless!) '-headless', ...webBrowserFlags],
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
  },
  Browser.edge => <String, dynamic>{'acceptInsecureCerts': true, 'browserName': 'edge'},
  Browser.safari => <String, dynamic>{'browserName': 'safari'},
  Browser.iosSafari => <String, dynamic>{
    'platformName': 'ios',
    'browserName': 'safari',
    'safari:useSimulator': true,
  },
  Browser.androidChrome => <String, dynamic>{
    'browserName': 'chrome',
    'platformName': 'android',
    'goog:chromeOptions': <String, dynamic>{
      'androidPackage': 'com.android.chrome',
      'args': <String>['--disable-fullscreen', ...webBrowserFlags],
    },
  },
};
