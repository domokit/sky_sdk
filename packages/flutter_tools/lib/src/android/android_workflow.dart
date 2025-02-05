// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:process/process.dart';

import '../base/common.dart';
import '../base/context.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/user_messages.dart';
import '../base/version.dart';
import '../convert.dart';
import '../doctor_validator.dart';
import '../features.dart';
import 'android_sdk.dart';
import 'java.dart';

const int kAndroidSdkMinVersion = 29;
final Version kAndroidJavaMinVersion = Version(1, 8, 0);
final Version kAndroidSdkBuildToolsMinVersion = Version(28, 0, 3);

AndroidWorkflow? get androidWorkflow => context.get<AndroidWorkflow>();
AndroidValidator? get androidValidator => context.get<AndroidValidator>();
AndroidLicenseValidator? get androidLicenseValidator => context.get<AndroidLicenseValidator>();

enum LicensesAccepted { none, some, all, unknown }

final RegExp licenseCounts = RegExp(r'(\d+) of (\d+) SDK package licenses? not accepted.');
final RegExp licenseNotAccepted = RegExp(r'licenses? not accepted', caseSensitive: false);
final RegExp licenseAccepted = RegExp(r'All SDK package licenses accepted.');

class AndroidWorkflow implements Workflow {
  AndroidWorkflow({required AndroidSdk? androidSdk, required FeatureFlags featureFlags})
    : _androidSdk = androidSdk,
      _featureFlags = featureFlags;

  final AndroidSdk? _androidSdk;
  final FeatureFlags _featureFlags;

  @override
  bool get appliesToHostPlatform => _featureFlags.isAndroidEnabled;

  @override
  bool get canListDevices =>
      appliesToHostPlatform && _androidSdk != null && _androidSdk.adbPath != null;

  @override
  bool get canLaunchDevices =>
      appliesToHostPlatform &&
      _androidSdk != null &&
      _androidSdk.adbPath != null &&
      _androidSdk.validateSdkWellFormed().isEmpty;

  @override
  bool get canListEmulators => canListDevices && _androidSdk?.emulatorPath != null;
}

/// A validator that checks if the Android SDK and Java SDK are available and
/// installed correctly.
///
/// Android development requires the Android SDK, and at least one Java SDK. While
/// newer Java compilers can be used to compile the Java application code, the SDK
/// tools themselves required JDK 1.8. This older JDK is normally bundled with
/// Android Studio.
class AndroidValidator extends DoctorValidator {
  AndroidValidator({
    required Java? java,
    required AndroidSdk? androidSdk,
    required Logger logger,
    required Platform platform,
    required UserMessages userMessages,
  }) : _java = java,
       _androidSdk = androidSdk,
       _logger = logger,
       _platform = platform,
       _userMessages = userMessages,
       super('Android toolchain - develop for Android devices');

  final Java? _java;
  final AndroidSdk? _androidSdk;
  final Logger _logger;
  final Platform _platform;
  final UserMessages _userMessages;

  @override
  String get slowWarning => '${_task ?? 'This'} is taking a long time...';
  String? _task;

  /// Returns false if we cannot determine the Java version or if the version
  /// is older that the minimum allowed version of 1.8.
  Future<bool> _checkJavaVersion(List<ValidationMessage> messages) async {
    _task = 'Checking Java status';
    try {
      if (_java?.binaryPath == null) {
        messages.add(ValidationMessage.error(_userMessages.androidMissingJdk));
        return false;
      }
      messages.add(
        ValidationMessage(_androidJdkLocationMessage(_java!.binaryPath, _java.javaSource)),
      );
      if (!_java.canRun()) {
        messages.add(
          ValidationMessage.error(_userMessages.androidCantRunJavaBinary(_java.binaryPath)),
        );
        return false;
      }
      Version? javaVersion;
      try {
        javaVersion = _java.version;
      } on Exception catch (error) {
        _logger.printTrace(error.toString());
      }
      if (javaVersion == null) {
        // Could not determine the java version.
        messages.add(ValidationMessage.error(_userMessages.androidUnknownJavaVersion));
        return false;
      }
      if (javaVersion < kAndroidJavaMinVersion) {
        messages.add(
          ValidationMessage.error(_userMessages.androidJavaMinimumVersion(javaVersion.toString())),
        );
        return false;
      }
      messages.add(ValidationMessage(_userMessages.androidJavaVersion(javaVersion.toString())));
      return true;
    } finally {
      _task = null;
    }
  }

  @override
  Future<ValidationResult> validateImpl() async {
    final List<ValidationMessage> messages = <ValidationMessage>[];
    final AndroidSdk? androidSdk = _androidSdk;
    if (androidSdk == null) {
      // No Android SDK found.
      if (_platform.environment.containsKey(kAndroidHome)) {
        final String androidHomeDir = _platform.environment[kAndroidHome]!;
        messages.add(
          ValidationMessage.error(_userMessages.androidBadSdkDir(kAndroidHome, androidHomeDir)),
        );
      } else {
        // Instruct user to set [kAndroidSdkRoot] and not deprecated [kAndroidHome]
        // See https://github.com/flutter/flutter/issues/39301
        messages.add(
          ValidationMessage.error(_userMessages.androidMissingSdkInstructions(_platform)),
        );
      }
      return ValidationResult(ValidationType.missing, messages);
    }

    messages.add(ValidationMessage(_userMessages.androidSdkLocation(androidSdk.directory.path)));

    _task = 'Validating Android SDK command line tools are available';
    if (!androidSdk.cmdlineToolsAvailable) {
      messages.add(
        const ValidationMessage.error(
          'cmdline-tools component is missing.\n'
          'Try installing or updating Android Studio.\n'
          'Alternatively, download the tools from https://developer.android.com/studio#command-line-tools-only '
          'and make sure to set the ANDROID_HOME environment variable.\n'
          'See https://developer.android.com/studio/command-line for more details.',
        ),
      );
      return ValidationResult(ValidationType.missing, messages);
    }

    _task = 'Validating Android SDK licenses';
    if (androidSdk.licensesAvailable && !androidSdk.platformToolsAvailable) {
      messages.add(ValidationMessage.hint(_userMessages.androidSdkLicenseOnly(kAndroidHome)));
      return ValidationResult(ValidationType.partial, messages);
    }

    String? sdkVersionText;
    final AndroidSdkVersion? androidSdkLatestVersion = androidSdk.latestVersion;
    if (androidSdkLatestVersion != null) {
      if (androidSdkLatestVersion.sdkLevel < kAndroidSdkMinVersion ||
          androidSdkLatestVersion.buildToolsVersion < kAndroidSdkBuildToolsMinVersion) {
        messages.add(
          ValidationMessage.error(
            _userMessages.androidSdkBuildToolsOutdated(
              kAndroidSdkMinVersion,
              kAndroidSdkBuildToolsMinVersion.toString(),
              _platform,
            ),
          ),
        );
        return ValidationResult(ValidationType.missing, messages);
      }
      sdkVersionText = _userMessages.androidStatusInfo(
        androidSdkLatestVersion.buildToolsVersionName,
      );

      messages.add(
        ValidationMessage(
          _userMessages.androidSdkPlatformToolsVersion(
            androidSdkLatestVersion.platformName,
            androidSdkLatestVersion.buildToolsVersionName,
          ),
        ),
      );
    } else {
      messages.add(ValidationMessage.error(_userMessages.androidMissingSdkInstructions(_platform)));
    }

    if (_platform.environment.containsKey(kAndroidHome)) {
      final String androidHomeDir = _platform.environment[kAndroidHome]!;
      messages.add(ValidationMessage('$kAndroidHome = $androidHomeDir'));
    }
    if (_platform.environment.containsKey(kAndroidSdkRoot)) {
      final String androidSdkRoot = _platform.environment[kAndroidSdkRoot]!;
      messages.add(ValidationMessage('$kAndroidSdkRoot = $androidSdkRoot'));
    }

    _task = 'Validating Android SDK';
    final List<String> validationResult = androidSdk.validateSdkWellFormed();

    if (validationResult.isNotEmpty) {
      // Android SDK is not functional.
      messages.addAll(
        validationResult.map<ValidationMessage>((String message) {
          return ValidationMessage.error(message);
        }),
      );
      messages.add(ValidationMessage(_userMessages.androidSdkInstallHelp(_platform)));
      return ValidationResult(ValidationType.partial, messages, statusInfo: sdkVersionText);
    }

    _task = 'Finding Java binary';

    // Check JDK version.
    if (!await _checkJavaVersion(messages)) {
      return ValidationResult(ValidationType.partial, messages, statusInfo: sdkVersionText);
    }

    // Success.
    return ValidationResult(ValidationType.success, messages, statusInfo: sdkVersionText);
  }
}

/// A subvalidator that checks if the licenses within the detected Android
/// SDK have been accepted.
class AndroidLicenseValidator extends DoctorValidator {
  AndroidLicenseValidator({
    required Java? java,
    required AndroidSdk? androidSdk,
    required Platform platform,
    required ProcessManager processManager,
    required Logger logger,
    required Stdio stdio,
    required UserMessages userMessages,
  }) : _java = java,
       _androidSdk = androidSdk,
       _platform = platform,
       _processManager = processManager,
       _logger = logger,
       _stdio = stdio,
       _userMessages = userMessages,
       super('Android license subvalidator');

  final Java? _java;
  final AndroidSdk? _androidSdk;
  final Stdio _stdio;
  final Platform _platform;
  final ProcessManager _processManager;
  final Logger _logger;
  final UserMessages _userMessages;

  @override
  String get slowWarning => 'Checking Android licenses is taking an unexpectedly long time...';

  @override
  Future<ValidationResult> validateImpl() async {
    final List<ValidationMessage> messages = <ValidationMessage>[];

    // Match pre-existing early termination behavior
    if (_androidSdk == null ||
        _androidSdk.latestVersion == null ||
        _androidSdk.validateSdkWellFormed().isNotEmpty ||
        !await _checkJavaVersionNoOutput()) {
      return ValidationResult(ValidationType.missing, messages);
    }

    final String sdkVersionText = _userMessages.androidStatusInfo(
      _androidSdk.latestVersion!.buildToolsVersionName,
    );

    // Check for licenses.
    switch (await licensesAccepted) {
      case LicensesAccepted.all:
        messages.add(ValidationMessage(_userMessages.androidLicensesAll));
      case LicensesAccepted.some:
        messages.add(ValidationMessage.hint(_userMessages.androidLicensesSome));
        return ValidationResult(ValidationType.partial, messages, statusInfo: sdkVersionText);
      case LicensesAccepted.none:
        messages.add(ValidationMessage.error(_userMessages.androidLicensesNone));
        return ValidationResult(ValidationType.partial, messages, statusInfo: sdkVersionText);
      case LicensesAccepted.unknown:
        messages.add(ValidationMessage.error(_userMessages.androidLicensesUnknown(_platform)));
        return ValidationResult(ValidationType.partial, messages, statusInfo: sdkVersionText);
    }
    return ValidationResult(ValidationType.success, messages, statusInfo: sdkVersionText);
  }

  Future<bool> _checkJavaVersionNoOutput() async {
    final String? javaBinary = _java?.binaryPath;

    if (javaBinary == null) {
      return false;
    }
    if (!_processManager.canRun(javaBinary)) {
      return false;
    }
    String? javaVersion;
    try {
      final ProcessResult result = await _processManager.run(<String>[javaBinary, '-version']);
      if (result.exitCode == 0) {
        final List<String> versionLines = (result.stderr as String).split('\n');
        javaVersion = versionLines.length >= 2 ? versionLines[1] : versionLines[0];
      }
    } on Exception catch (error) {
      _logger.printTrace(error.toString());
    }
    if (javaVersion == null) {
      // Could not determine the java version.
      return false;
    }
    return true;
  }

  Future<LicensesAccepted> get licensesAccepted async {
    LicensesAccepted? status;

    void handleLine(String line) {
      if (licenseCounts.hasMatch(line)) {
        final Match? match = licenseCounts.firstMatch(line);
        if (match?.group(1) != match?.group(2)) {
          status = LicensesAccepted.some;
        } else {
          status = LicensesAccepted.none;
        }
      } else if (licenseNotAccepted.hasMatch(line)) {
        // The licenseNotAccepted pattern is trying to match the same line as
        // licenseCounts, but is more general. In case the format changes, a
        // more general match may keep doctor mostly working.
        status = LicensesAccepted.none;
      } else if (licenseAccepted.hasMatch(line)) {
        status ??= LicensesAccepted.all;
      }
    }

    if (!_canRunSdkManager()) {
      return LicensesAccepted.unknown;
    }

    try {
      final Process process = await _processManager.start(<String>[
        _androidSdk!.sdkManagerPath!,
        '--licenses',
      ], environment: _java?.environment);
      await ProcessUtils.writelnToStdinUnsafe(stdin: process.stdin, line: 'n');
      // We expect logcat streams to occasionally contain invalid utf-8,
      // see: https://github.com/flutter/flutter/pull/8864.
      final Future<void> output =
          process.stdout
              .transform<String>(const Utf8Decoder(reportErrors: false))
              .transform<String>(const LineSplitter())
              .listen(handleLine)
              .asFuture<void>();
      final Future<void> errors =
          process.stderr
              .transform<String>(const Utf8Decoder(reportErrors: false))
              .transform<String>(const LineSplitter())
              .listen(handleLine)
              .asFuture<void>();
      await Future.wait<void>(<Future<void>>[output, errors]);
      return status ?? LicensesAccepted.unknown;
    } on IOException catch (e) {
      _logger.printTrace('Failed to run Android sdk manager: $e');
      return LicensesAccepted.unknown;
    }
  }

  /// Run the Android SDK manager tool in order to accept SDK licenses.
  Future<bool> runLicenseManager() async {
    if (_androidSdk == null) {
      _logger.printStatus(_userMessages.androidSdkShort);
      return false;
    }

    if (!_canRunSdkManager()) {
      throwToolExit(
        'Android sdkmanager not found. Update to the latest Android SDK and ensure that '
        'the cmdline-tools are installed to resolve this.',
      );
    }

    try {
      final Process process = await _processManager.start(<String>[
        _androidSdk.sdkManagerPath!,
        '--licenses',
      ], environment: _java?.environment);

      // The real stdin will never finish streaming. Pipe until the child process
      // finishes.
      unawaited(
        process.stdin
            .addStream(_stdio.stdin)
            // If the process exits unexpectedly with an error, that will be
            // handled by the caller.
            .then(
              (Object? socket) => socket,
              onError: (dynamic err, StackTrace stack) {
                _logger.printTrace('Echoing stdin to the licenses subprocess failed:');
                _logger.printTrace('$err\n$stack');
              },
            ),
      );

      final List<String> stderrLines = <String>[];
      // Wait for stdout and stderr to be fully processed, because process.exitCode
      // may complete first.
      try {
        await Future.wait<void>(<Future<void>>[
          _stdio.addStdoutStream(process.stdout),
          process.stderr.forEach((List<int> event) {
            _stdio.stderr.add(event);
            stderrLines.add(utf8.decode(event));
          }),
        ]);
      } on Exception catch (err, stack) {
        _logger.printTrace('Echoing stdout or stderr from the license subprocess failed:');
        _logger.printTrace('$err\n$stack');
      }

      final int exitCode = await process.exitCode;
      if (exitCode != 0) {
        throwToolExit(_messageForSdkManagerError(stderrLines, exitCode));
      }
      return true;
    } on ProcessException catch (e) {
      throwToolExit(
        _userMessages.androidCannotRunSdkManager(
          _androidSdk.sdkManagerPath ?? '',
          e.toString(),
          _platform,
        ),
      );
    }
  }

  bool _canRunSdkManager() {
    final String? sdkManagerPath = _androidSdk?.sdkManagerPath;
    if (sdkManagerPath == null) {
      return false;
    }
    return _processManager.canRun(sdkManagerPath);
  }

  String _messageForSdkManagerError(List<String> androidSdkStderr, int exitCode) {
    final String sdkManagerPath = _androidSdk!.sdkManagerPath!;

    final bool failedDueToJdkIncompatibility = androidSdkStderr.join().contains(
      RegExp(
        r'java\.lang\.UnsupportedClassVersionError.*SdkManagerCli '
        r'has been compiled by a more recent version of the Java Runtime',
      ),
    );

    if (failedDueToJdkIncompatibility) {
      return 'Android sdkmanager tool was found, but failed to run ($sdkManagerPath): "exited code $exitCode".\n'
          'It appears the version of the Java binary used (${_java!.binaryPath}) is '
          'too out-of-date and is incompatible with the Android sdkmanager tool.\n'
          'If the Java binary came bundled with Android Studio, consider updating '
          'your installation of Android studio. Alternatively, you can uninstall '
          'the Android SDK command-line tools and install an earlier version. ';
    }

    return _userMessages.androidCannotRunSdkManager(
      sdkManagerPath,
      'exited code $exitCode',
      _platform,
    );
  }
}

String _androidJdkLocationMessage(String location, JavaSource source) {
  final String setWithConfigBreadcrumb = switch (source) {
    JavaSource.androidStudio || JavaSource.path || JavaSource.javaHome =>
      'To manually set the JDK path, use: `flutter config --jdk-dir="path/to/jdk"`.',
    JavaSource.flutterConfig =>
      'To change the current JDK, run: `flutter config --jdk-dir="path/to/jdk"`.',
  };
  final String sourceMessagePart = switch (source) {
    JavaSource.androidStudio =>
      'This is the JDK bundled with the latest Android Studio installation on this machine.',
    JavaSource.javaHome => 'This JDK is specified by the JAVA_HOME environment variable.',
    JavaSource.path => 'This JDK was found in the system PATH.',
    JavaSource.flutterConfig => 'This JDK is specified in your Flutter configuration.',
  };

  return 'Java binary at: $location\n'
      '$sourceMessagePart\n'
      '$setWithConfigBreadcrumb';
}
