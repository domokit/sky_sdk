// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/args.dart';
import 'package:meta/meta.dart';
import 'package:package_config/package_config_types.dart';

import '../android/android_device.dart';
import '../application_package.dart';
import '../artifacts.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/signals.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../dart/package_map.dart';
import '../device.dart';
import '../drive/drive_service.dart';
import '../drive/web_driver_service.dart' show Browser;
import '../globals.dart' as globals;
import '../ios/devices.dart';
import '../resident_runner.dart';
import '../runner/flutter_command.dart'
    show FlutterCommandCategory, FlutterCommandResult, FlutterOptions;
import '../web/web_device.dart';
import 'run.dart';

/// Runs integration (a.k.a. end-to-end) tests.
///
/// An integration test is a program that runs in a separate process from your
/// Flutter application. It connects to the application and acts like a user,
/// performing taps, scrolls, reading out widget properties and verifying their
/// correctness.
///
/// This command takes a target Flutter application that you would like to test
/// as the `--target` option (defaults to `lib/main.dart`). It then looks for a
/// corresponding test file within the `test_driver` directory. The test file is
/// expected to have the same name but contain the `_test.dart` suffix. The
/// `_test.dart` file would generally be a Dart program that uses
/// `package:flutter_driver` and exercises your application. Most commonly it
/// is a test written using `package:test`, but you are free to use something
/// else.
///
/// The app and the test are launched simultaneously. Once the test completes
/// the application is stopped and the command exits. If all these steps are
/// successful the exit code will be `0`. Otherwise, you will see a non-zero
/// exit code.
class DriveCommand extends RunCommandBase {
  DriveCommand({
    bool verboseHelp = false,
    @visibleForTesting FlutterDriverFactory? flutterDriverFactory,
    @visibleForTesting
    this.signalsToHandle = const <ProcessSignal>{ProcessSignal.sigint, ProcessSignal.sigterm},
    required FileSystem fileSystem,
    required Logger logger,
    required Platform platform,
    required Terminal terminal,
    required OutputPreferences outputPreferences,
    required this.signals,
  }) : _flutterDriverFactory = flutterDriverFactory,
       _fileSystem = fileSystem,
       _logger = logger,
       _platform = platform,
       _terminal = terminal,
       _outputPreferences = outputPreferences,
       _fsUtils = FileSystemUtils(fileSystem: fileSystem, platform: platform),
       super(verboseHelp: verboseHelp) {
    requiresPubspecYaml();
    addEnableExperimentation(hide: !verboseHelp);

    // By default, the drive app should not publish the VM service port over mDNS
    // to prevent a local network permission dialog on iOS 14+,
    // which cannot be accepted or dismissed in a CI environment.
    addPublishPort(enabledByDefault: false, verboseHelp: verboseHelp);
    argParser
      ..addFlag(
        'keep-app-running',
        help:
            'Will keep the Flutter application running when done testing.\n'
            'By default, "flutter drive" stops the application after tests are finished, '
            'and "--keep-app-running" overrides this. On the other hand, if "--use-existing-app" '
            'is specified, then "flutter drive" instead defaults to leaving the application '
            'running, and "--no-keep-app-running" overrides it.',
      )
      ..addOption(
        'use-existing-app',
        help:
            'Connect to an already running instance via the given Dart VM Service URL. '
            'If this option is given, the application will not be automatically started, '
            'and it will only be stopped if "--no-keep-app-running" is explicitly set.',
        valueHelp: 'url',
      )
      ..addOption(
        'driver',
        help:
            'The test file to run on the host (as opposed to the target file to run on '
            'the device).\n'
            'By default, this file has the same base name as the target file, but in the '
            '"test_driver/" directory instead, and with "_test" inserted just before the '
            'extension, so e.g. if the target is "lib/main.dart", the driver will be '
            '"test_driver/main_test.dart".',
        valueHelp: 'path',
      )
      ..addFlag(
        'build',
        defaultsTo: true,
        help:
            '(deprecated) Build the app before running. To use an existing app, pass the "--${FlutterOptions.kUseApplicationBinary}" '
            'flag with an existing APK.',
      )
      ..addOption(
        'screenshot',
        valueHelp: 'path/to/directory',
        help: 'Directory location to write screenshots on test failure.',
      )
      ..addOption(
        'driver-port',
        defaultsTo: '4444',
        help: 'The port where Webdriver server is launched at.',
        valueHelp: '4444',
      )
      ..addFlag(
        'headless',
        defaultsTo: true,
        help: 'Whether the driver browser is going to be launched in headless mode.',
      )
      ..addOption(
        'browser-name',
        defaultsTo: Browser.chrome.cliName,
        help: 'Name of the browser where tests will be executed.',
        allowed: Browser.values.map((Browser e) => e.cliName),
        allowedHelp: CliEnum.allowedHelp(Browser.values),
      )
      ..addOption(
        'browser-dimension',
        defaultsTo: '1600,1024',
        help:
            'The dimension of the browser when running a Flutter Web test. '
            'This will affect screenshot and all offset-related actions.',
        valueHelp: 'width,height',
      )
      ..addFlag(
        'android-emulator',
        defaultsTo: true,
        help:
            'Whether to perform Flutter Driver testing using an Android Emulator. '
            'Works only if "browser-name" is set to "android-chrome".',
      )
      ..addOption(
        'chrome-binary',
        help:
            'Location of the Chrome binary. '
            'Works only if "browser-name" is set to "chrome".',
      )
      ..addMultiOption(
        'test-arguments',
        help:
            'Additional arguments to pass to the Dart VM running The test script.\n\n'
            'This can be used to opt-in to use "dart test" as a runner for the test script, '
            'which allows, among other things, changing the reporter. For example, to opt-in '
            'to the "expanded" reporter, pass both "test" and "--reporter=expanded".\n\n'
            'Please leave feedback at <https://github.com/flutter/flutter/issues/152409>.',
      )
      ..addOption(
        'profile-memory',
        help:
            'Launch devtools and profile application memory, writing '
            'The output data to the file path provided to this argument as JSON.',
        valueHelp: 'profile_memory.json',
      )
      ..addOption(
        'timeout',
        help:
            'Timeout the test after the given number of seconds. If the '
            '"--screenshot" option is provided, a screenshot will be taken '
            'before exiting. Defaults to no timeout.',
        valueHelp: '360',
      );
  }

  final Signals signals;

  /// The [ProcessSignal]s that will lead to a screenshot being taken (if the option is provided).
  final Set<ProcessSignal> signalsToHandle;

  // `pub` must always be run due to the test script running from source,
  // even if an application binary is used. Default to true unless the user explicitly
  // specified not to.
  @override
  bool get shouldRunPub {
    if (argResults!.wasParsed('pub') && !boolArg('pub')) {
      return false;
    }
    return true;
  }

  FlutterDriverFactory? _flutterDriverFactory;
  final FileSystem _fileSystem;
  final Logger _logger;
  final Platform _platform;
  final Terminal _terminal;
  final OutputPreferences _outputPreferences;
  final FileSystemUtils _fsUtils;
  Timer? timeoutTimer;
  Map<ProcessSignal, Object>? screenshotTokens;

  @override
  final String name = 'drive';

  @override
  final String description =
      'Builds and installs the app, and runs a Dart program that connects to '
      'the app, often to run externally facing integration tests, such as with '
      'package:test and package:flutter_driver.\n'
      '\n'
      'Usage: flutter drive --target <lib/main.dart> --driver <test_driver/main_test.dart>.';

  @override
  String get category => FlutterCommandCategory.project;

  @override
  final List<String> aliases = <String>['driver'];

  String? get userIdentifier => stringArg(FlutterOptions.kDeviceUser);

  String? get screenshot => stringArg('screenshot');

  @override
  bool get startPausedDefault => true;

  @override
  bool get cachePubGet => false;

  String? get applicationBinaryPath => stringArg(FlutterOptions.kUseApplicationBinary);

  Future<Device?> get targetedDevice async {
    return findTargetDevice(includeDevicesUnsupportedByProject: applicationBinaryPath == null);
  }

  // Wireless iOS devices need `publish-port` to be enabled because it requires mDNS.
  // If the flag wasn't provided as an actual argument and it's a wireless device,
  // change it to be enabled.
  @override
  Future<bool> get disablePortPublication async {
    final ArgResults? localArgResults = argResults;
    final Device? device = await targetedDevice;
    final bool isWirelessIOSDevice = device is IOSDevice && device.isWirelesslyConnected;
    if (isWirelessIOSDevice &&
        localArgResults != null &&
        !localArgResults.wasParsed('publish-port')) {
      _logger.printTrace(
        'A wireless iOS device is being used. Changing `publish-port` to be enabled.',
      );
      return false;
    }
    return !boolArg('publish-port');
  }

  @override
  Future<void> validateCommand() async {
    if (userIdentifier != null) {
      final Device? device = await findTargetDevice();
      if (device is! AndroidDevice) {
        throwToolExit('--${FlutterOptions.kDeviceUser} is only supported for Android');
      }
    }
    return super.validateCommand();
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final String? testFile = _getTestFile();
    if (testFile == null) {
      throwToolExit(null);
    }
    if (await _fileSystem.type(testFile) != FileSystemEntityType.file) {
      // A very common source of error is holding "flutter drive" wrong,
      // and providing the "test_driver/foo_test.dart" as the target, when
      // the intention was to provide "lib/foo.dart".
      if (_fileSystem.path.isWithin('test_driver', targetFile)) {
        _logger.printError(
          'The file path passed to --target should be an app entrypoint that '
          'contains a "main()". Did you mean "flutter drive --driver $targetFile"?',
        );
      }
      throwToolExit('Test file not found: $testFile');
    }
    final Device? device = await targetedDevice;
    if (device == null) {
      throwToolExit(null);
    }
    if (screenshot != null && !device.supportsScreenshot) {
      _logger.printError('Screenshot not supported for ${device.displayName}.');
    }

    final bool web = device is WebServerDevice || device is ChromiumDevice;
    _flutterDriverFactory ??= FlutterDriverFactory(
      applicationPackageFactory: ApplicationPackageFactory.instance!,
      logger: _logger,
      platform: _platform,
      terminal: _terminal,
      outputPreferences: _outputPreferences,
      processUtils: globals.processUtils,
      dartSdkPath: globals.artifacts!.getArtifactPath(Artifact.engineDartBinary),
      devtoolsLauncher: DevtoolsLauncher.instance!,
    );
    final File packageConfigFile = findPackageConfigFileOrDefault(_fileSystem.currentDirectory);

    final PackageConfig packageConfig = await loadPackageConfigWithLogging(
      packageConfigFile,
      logger: _logger,
      throwOnError: false,
    );
    final DriverService driverService = _flutterDriverFactory!.createDriverService(web);
    final BuildInfo buildInfo = await getBuildInfo();
    final DebuggingOptions debuggingOptions = await createDebuggingOptions(web);
    final File? applicationBinary =
        applicationBinaryPath == null ? null : _fileSystem.file(applicationBinaryPath);

    bool screenshotTaken = false;
    try {
      if (stringArg('use-existing-app') == null) {
        await driverService.start(
          buildInfo,
          device,
          debuggingOptions,
          applicationBinary: applicationBinary,
          route: route,
          userIdentifier: userIdentifier,
          mainPath: targetFile,
          platformArgs: <String, Object>{
            if (traceStartup) 'trace-startup': traceStartup,
            if (web) '--no-launch-chrome': true,
          },
        );
      } else {
        final Uri? uri = Uri.tryParse(stringArg('use-existing-app')!);
        if (uri == null) {
          throwToolExit('Invalid VM Service URI: ${stringArg('use-existing-app')}');
        }
        await driverService.reuseApplication(uri, device, debuggingOptions);
      }

      final Future<int> testResultFuture = driverService.startTest(
        testFile,
        stringsArg('test-arguments'),
        packageConfig,
        chromeBinary: stringArg('chrome-binary'),
        headless: boolArg('headless'),
        webBrowserFlags: stringsArg(FlutterOptions.kWebBrowserFlag),
        browserDimension: stringArg('browser-dimension')!.split(','),
        browserName: stringArg('browser-name'),
        driverPort:
            stringArg('driver-port') != null ? int.tryParse(stringArg('driver-port')!) : null,
        androidEmulator: boolArg('android-emulator'),
        profileMemory: stringArg('profile-memory'),
      );

      if (screenshot != null) {
        // If the test is sent a signal or times out, take a screenshot
        _registerScreenshotCallbacks(device, _fileSystem.directory(screenshot));
      }

      final int testResult = await testResultFuture;

      if (timeoutTimer != null) {
        timeoutTimer!.cancel();
      }
      _unregisterScreenshotCallbacks();

      if (testResult != 0 && screenshot != null) {
        // Take a screenshot while the app is still running.
        await _takeScreenshot(device, _fileSystem.directory(screenshot));
        screenshotTaken = true;
      }

      if (boolArg('keep-app-running')) {
        _logger.printStatus('Leaving the application running.');
      } else {
        await driverService.stop(userIdentifier: userIdentifier);
      }
      if (testResult != 0) {
        throwToolExit(null);
      }
    } on Exception catch (_) {
      // On exceptions, including ToolExit, take a screenshot on the device
      // unless a screenshot was already taken on test failure.
      if (!screenshotTaken && screenshot != null) {
        await _takeScreenshot(device, _fileSystem.directory(screenshot));
      }
      rethrow;
    }

    return FlutterCommandResult.success();
  }

  int? get _timeoutSeconds {
    final String? timeoutString = stringArg('timeout');
    if (timeoutString == null) {
      return null;
    }
    final int? timeoutSeconds = int.tryParse(timeoutString);
    if (timeoutSeconds == null || timeoutSeconds <= 0) {
      throwToolExit(
        'Invalid value "$timeoutString" provided to the option --timeout: '
        'expected a positive integer representing seconds.',
      );
    }
    return timeoutSeconds;
  }

  void _registerScreenshotCallbacks(Device device, Directory screenshotDir) {
    _logger.printTrace('Registering signal handlers...');
    final Map<ProcessSignal, Object> tokens = <ProcessSignal, Object>{};
    for (final ProcessSignal signal in signalsToHandle) {
      tokens[signal] = signals.addHandler(signal, (ProcessSignal signal) {
        _unregisterScreenshotCallbacks();
        _logger.printError('Caught $signal');
        return _takeScreenshot(device, screenshotDir);
      });
    }
    screenshotTokens = tokens;

    final int? timeoutSeconds = _timeoutSeconds;
    if (timeoutSeconds != null) {
      timeoutTimer = Timer(Duration(seconds: timeoutSeconds), () {
        _unregisterScreenshotCallbacks();
        _takeScreenshot(device, screenshotDir);
        throwToolExit('Timed out after $timeoutSeconds seconds');
      });
    }
  }

  void _unregisterScreenshotCallbacks() {
    if (screenshotTokens != null) {
      _logger.printTrace('Unregistering signal handlers...');
      for (final MapEntry<ProcessSignal, Object> entry in screenshotTokens!.entries) {
        signals.removeHandler(entry.key, entry.value);
      }
    }
    timeoutTimer?.cancel();
  }

  String? _getTestFile() {
    if (argResults!['driver'] != null) {
      return stringArg('driver');
    }

    // If the --driver argument wasn't provided, then derive the value from
    // the target file.
    String appFile = _fileSystem.path.normalize(targetFile);

    // This command extends `flutter run` and therefore CWD == package dir
    final String packageDir = _fileSystem.currentDirectory.path;

    // Make appFile path relative to package directory because we are looking
    // for the corresponding test file relative to it.
    if (!_fileSystem.path.isRelative(appFile)) {
      if (!_fileSystem.path.isWithin(packageDir, appFile)) {
        _logger.printError(
          'Application file $appFile is outside the package directory $packageDir',
        );
        return null;
      }

      appFile = _fileSystem.path.relative(appFile, from: packageDir);
    }

    final List<String> parts = _fileSystem.path.split(appFile);

    if (parts.length < 2) {
      _logger.printError(
        'Application file $appFile must reside in one of the sub-directories '
        'of the package structure, not in the root directory.',
      );
      return null;
    }

    // Look for the test file inside `test_driver/` matching the sub-path, e.g.
    // if the application is `lib/foo/bar.dart`, the test file is expected to
    // be `test_driver/foo/bar_test.dart`.
    final String pathWithNoExtension = _fileSystem.path.withoutExtension(
      _fileSystem.path.joinAll(<String>[packageDir, 'test_driver', ...parts.skip(1)]),
    );
    return '${pathWithNoExtension}_test${_fileSystem.path.extension(appFile)}';
  }

  Future<void> _takeScreenshot(Device device, Directory outputDirectory) async {
    if (!device.supportsScreenshot) {
      return;
    }
    try {
      outputDirectory.createSync(recursive: true);
      final File outputFile = _fsUtils.getUniqueFile(outputDirectory, 'drive', 'png');
      await device.takeScreenshot(outputFile);
      _logger.printStatus('Screenshot written to ${outputFile.path}');
    } on Exception catch (error) {
      _logger.printError('Error taking screenshot: $error');
    }
  }
}
