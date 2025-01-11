// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import 'android/android_studio_validator.dart';
import 'android/android_workflow.dart';
import 'artifacts.dart';
import 'base/async_guard.dart';
import 'base/context.dart';
import 'base/file_system.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/net.dart';
import 'base/os.dart';
import 'base/platform.dart';
import 'base/terminal.dart';
import 'base/time.dart';
import 'base/user_messages.dart';
import 'base/utils.dart';
import 'cache.dart';
import 'custom_devices/custom_device_workflow.dart';
import 'device.dart';
import 'doctor_validator.dart';
import 'features.dart';
import 'globals.dart' as globals;
import 'http_host_validator.dart';
import 'intellij/intellij_validator.dart';
import 'linux/linux_doctor.dart';
import 'linux/linux_workflow.dart';
import 'macos/macos_workflow.dart';
import 'macos/xcode_validator.dart';
import 'proxy_validator.dart';
import 'reporting/reporting.dart';
import 'tester/flutter_tester.dart';
import 'version.dart';
import 'vscode/vscode_validator.dart';
import 'web/chrome.dart';
import 'web/web_validator.dart';
import 'web/workflow.dart';
import 'windows/visual_studio_validator.dart';
import 'windows/windows_version_validator.dart';
import 'windows/windows_workflow.dart';

abstract class DoctorValidatorsProvider {
  // Allow tests to construct a [_DefaultDoctorValidatorsProvider] with explicit
  // [FeatureFlags].
  factory DoctorValidatorsProvider.test({
    Platform? platform,
    Logger? logger,
    required FeatureFlags featureFlags,
  }) {
    return _DefaultDoctorValidatorsProvider(
      featureFlags: featureFlags,
      platform: platform ?? FakePlatform(),
      logger: logger ?? BufferLogger.test(),
    );
  }

  /// The singleton instance, pulled from the [AppContext].
  static DoctorValidatorsProvider get _instance =>
      context.get<DoctorValidatorsProvider>()!;

  static final DoctorValidatorsProvider defaultInstance =
      _DefaultDoctorValidatorsProvider(
    logger: globals.logger,
    platform: globals.platform,
    featureFlags: featureFlags,
  );

  List<DoctorValidator> get validators;
  List<Workflow> get workflows;
}

class _DefaultDoctorValidatorsProvider implements DoctorValidatorsProvider {
  _DefaultDoctorValidatorsProvider({
    required this.platform,
    required this.featureFlags,
    required Logger logger,
  }) : _logger = logger;

  List<DoctorValidator>? _validators;
  List<Workflow>? _workflows;
  final Platform platform;
  final FeatureFlags featureFlags;
  final Logger _logger;

  late final LinuxWorkflow linuxWorkflow = LinuxWorkflow(
    platform: platform,
    featureFlags: featureFlags,
  );

  late final WebWorkflow webWorkflow =
      WebWorkflow(platform: platform, featureFlags: featureFlags);

  late final MacOSWorkflow macOSWorkflow = MacOSWorkflow(
    platform: platform,
    featureFlags: featureFlags,
  );

  late final CustomDeviceWorkflow customDeviceWorkflow = CustomDeviceWorkflow(
    featureFlags: featureFlags,
  );

  @override
  List<DoctorValidator> get validators {
    if (_validators != null) {
      return _validators!;
    }

    final List<DoctorValidator> ideValidators = <DoctorValidator>[
      if (androidWorkflow!.appliesToHostPlatform)
        ...AndroidStudioValidator.allValidators(
          globals.config,
          platform,
          globals.fs,
          globals.userMessages,
        ),
      ...IntelliJValidator.installedValidators(
        fileSystem: globals.fs,
        platform: platform,
        userMessages: globals.userMessages,
        plistParser: globals.plistParser,
        processManager: globals.processManager,
        logger: _logger,
      ),
      ...VsCodeValidator.installedValidators(
          globals.fs, platform, globals.processManager),
    ];
    final ProxyValidator proxyValidator = ProxyValidator(platform: platform);
    _validators = <DoctorValidator>[
      FlutterValidator(
        fileSystem: globals.fs,
        platform: globals.platform,
        flutterVersion: () => globals.flutterVersion
            .fetchTagsAndGetVersion(clock: globals.systemClock),
        devToolsVersion: () => globals.cache.devToolsVersion,
        processManager: globals.processManager,
        userMessages: globals.userMessages,
        artifacts: globals.artifacts!,
        flutterRoot: () => Cache.flutterRoot!,
        operatingSystemUtils: globals.os,
      ),
      if (platform.isWindows)
        WindowsVersionValidator(
          operatingSystemUtils: globals.os,
          processLister: ProcessLister(globals.processManager),
          versionExtractor: WindowsVersionExtractor(
            processManager: globals.processManager,
            logger: globals.logger,
          ),
        ),
      if (androidWorkflow!.appliesToHostPlatform)
        GroupedValidator(
            <DoctorValidator>[androidValidator!, androidLicenseValidator!]),
      if (globals.iosWorkflow!.appliesToHostPlatform ||
          macOSWorkflow.appliesToHostPlatform)
        GroupedValidator(<DoctorValidator>[
          XcodeValidator(
            xcode: globals.xcode!,
            userMessages: globals.userMessages,
            iosSimulatorUtils: globals.iosSimulatorUtils!,
          ),
          globals.cocoapodsValidator!,
        ]),
      if (webWorkflow.appliesToHostPlatform)
        ChromeValidator(
          chromiumLauncher: ChromiumLauncher(
            browserFinder: findChromeExecutable,
            fileSystem: globals.fs,
            operatingSystemUtils: globals.os,
            platform: globals.platform,
            processManager: globals.processManager,
            logger: globals.logger,
          ),
          platform: globals.platform,
        ),
      if (linuxWorkflow.appliesToHostPlatform)
        LinuxDoctorValidator(
          processManager: globals.processManager,
          userMessages: globals.userMessages,
        ),
      if (windowsWorkflow!.appliesToHostPlatform) visualStudioValidator!,
      if (ideValidators.isNotEmpty) ...ideValidators else NoIdeValidator(),
      if (proxyValidator.shouldShow) proxyValidator,
      if (globals.deviceManager?.canListAnything ?? false)
        DeviceValidator(
            deviceManager: globals.deviceManager,
            userMessages: globals.userMessages),
      HttpHostValidator(
        platform: globals.platform,
        featureFlags: featureFlags,
        httpClient: globals.httpClientFactory?.call() ?? HttpClient(),
      ),
    ];
    return _validators!;
  }

  @override
  List<Workflow> get workflows {
    return _workflows ??= <Workflow>[
      if (globals.iosWorkflow!.appliesToHostPlatform) globals.iosWorkflow!,
      if (androidWorkflow?.appliesToHostPlatform ?? false) androidWorkflow!,
      if (linuxWorkflow.appliesToHostPlatform) linuxWorkflow,
      if (macOSWorkflow.appliesToHostPlatform) macOSWorkflow,
      if (windowsWorkflow?.appliesToHostPlatform ?? false) windowsWorkflow!,
      if (webWorkflow.appliesToHostPlatform) webWorkflow,
      if (customDeviceWorkflow.appliesToHostPlatform) customDeviceWorkflow,
    ];
  }
}

class Doctor {
  Doctor(
      {required Logger logger,
      required SystemClock clock,
      Analytics? analytics})
      : _logger = logger,
        _clock = clock,
        _analytics = analytics ?? globals.analytics;

  final Logger _logger;
  final SystemClock _clock;
  final Analytics _analytics;

  List<DoctorValidator> get validators {
    return DoctorValidatorsProvider._instance.validators;
  }

  /// Return a list of [ValidatorTask] objects and starts validation on all
  /// objects in [validators].
  List<ValidatorTask> startValidatorTasks() => <ValidatorTask>[
        for (final DoctorValidator validator in validators)
          ValidatorTask(
            validator,
            // We use an asyncGuard() here to be absolutely certain that
            // DoctorValidators do not result in an uncaught exception. Since the
            // Future returned by the asyncGuard() is not awaited, we pass an
            // onError callback to it and translate errors into ValidationResults.
            asyncGuard<ValidationResult>(
              () {
                final Completer<ValidationResult> timeoutCompleter =
                    Completer<ValidationResult>();
                final Timer timer = Timer(doctorDuration, () {
                  timeoutCompleter.completeError(
                    Exception(
                      '${validator.title} exceeded maximum allowed duration of $doctorDuration',
                    ),
                  );
                });
                final Future<ValidationResult> validatorFuture =
                    validator.validate();
                return Future.any<ValidationResult>(<Future<ValidationResult>>[
                  validatorFuture,
                  // This future can only complete with an error
                  timeoutCompleter.future,
                ]).then((ValidationResult result) async {
                  timer.cancel();
                  return result;
                });
              },
              onError: (Object exception, StackTrace stackTrace) {
                return ValidationResult.crash(exception, stackTrace);
              },
            ),
          ),
      ];

  List<Workflow> get workflows {
    return DoctorValidatorsProvider._instance.workflows;
  }

  /// Print a summary of the state of the tooling, as well as how to get more info.
  Future<void> summary() async {
    _logger.printStatus(await _summaryText());
  }

  Future<String> _summaryText() async {
    final StringBuffer buffer = StringBuffer();

    bool missingComponent = false;
    bool sawACrash = false;

    for (final DoctorValidator validator in validators) {
      final StringBuffer lineBuffer = StringBuffer();
      ValidationResult result;
      try {
        result =
            await asyncGuard<ValidationResult>(() => validator.validateImpl());
      } on Exception catch (exception) {
        // We're generating a summary, so drop the stack trace.
        result = ValidationResult.crash(exception);
      }
      lineBuffer.write('${result.coloredLeadingBox} ${validator.title}: ');
      switch (result.type) {
        case ValidationType.crash:
          lineBuffer.write('the doctor check crashed without a result.');
          sawACrash = true;
        case ValidationType.missing:
          lineBuffer.write('is not installed.');
        case ValidationType.partial:
          lineBuffer
              .write('is partially installed; more components are available.');
        case ValidationType.notAvailable:
          lineBuffer.write('is not available.');
        case ValidationType.success:
          lineBuffer.write('is fully installed.');
      }

      if (result.statusInfo != null) {
        lineBuffer.write(' (${result.statusInfo})');
      }

      buffer.write(
        wrapText(
          lineBuffer.toString(),
          hangingIndent: result.leadingBox.length + 1,
          columnWidth: globals.outputPreferences.wrapColumn,
          shouldWrap: globals.outputPreferences.wrapText,
        ),
      );
      buffer.writeln();

      if (result.type != ValidationType.success) {
        missingComponent = true;
      }
    }

    if (sawACrash) {
      buffer.writeln();
      buffer.writeln(
          'Run "flutter doctor" for information about why a doctor check crashed.');
    }

    if (missingComponent) {
      buffer.writeln();
      buffer.writeln(
        'Run "flutter doctor" for information about installing additional components.',
      );
    }

    return buffer.toString();
  }

  Future<bool> checkRemoteArtifacts(String engineRevision) async {
    return globals.cache
        .areRemoteArtifactsAvailable(engineVersion: engineRevision);
  }

  /// Maximum allowed duration for an entire validator to take.
  ///
  /// This should only ever be reached if a process is stuck.
  // Reduce this to under 5 minutes to diagnose:
  // https://github.com/flutter/flutter/issues/111686
  static const Duration doctorDuration = Duration(minutes: 4, seconds: 30);

  /// Print information about the state of installed tooling.
  ///
  /// To exclude personally identifiable information like device names and
  /// paths, set [showPii] to false.
  Future<bool> diagnose({
    bool androidLicenses = false,
    bool verbose = true,
    AndroidLicenseValidator? androidLicenseValidator,
    bool showPii = true,
    List<ValidatorTask>? startedValidatorTasks,
    bool sendEvent = true,
  }) async {
    final bool showColor = globals.terminal.supportsColor;
    if (androidLicenses && androidLicenseValidator != null) {
      return androidLicenseValidator.runLicenseManager();
    }

    if (!verbose) {
      _logger.printStatus(
          'Doctor summary (to see all details, run flutter doctor -v):');
    }
    bool doctorResult = true;
    int issues = 0;

    // This timestamp will be used on the backend of GA4 to group each of the events that
    // were sent for each doctor validator and its result
    final int analyticsTimestamp = _clock.now().millisecondsSinceEpoch;

    for (final ValidatorTask validatorTask
        in startedValidatorTasks ?? startValidatorTasks()) {
      final DoctorValidator validator = validatorTask.validator;
      final Status status = _logger.startSpinner(
        timeout: validator.slowWarningDuration,
        slowWarningCallback: () => validator.slowWarning,
      );
      ValidationResult result;
      try {
        result = await validatorTask.result;
        status.stop();
      } on Exception catch (exception, stackTrace) {
        result = ValidationResult.crash(exception, stackTrace);
        status.cancel();
      }

      switch (result.type) {
        case ValidationType.crash:
          doctorResult = false;
          issues += 1;
        case ValidationType.missing:
          doctorResult = false;
          issues += 1;
        case ValidationType.partial:
        case ValidationType.notAvailable:
          issues += 1;
        case ValidationType.success:
          break;
      }
      if (sendEvent) {
        if (validator is GroupedValidator) {
          for (int i = 0; i < validator.subValidators.length; i++) {
            final DoctorValidator subValidator = validator.subValidators[i];

            // Ensure that all of the subvalidators in the group have
            // a corresponding subresult in case a validator crashed
            final ValidationResult subResult;
            try {
              subResult = validator.subResults[i];
            } on RangeError {
              continue;
            }

            _analytics.send(
              Event.doctorValidatorResult(
                validatorName: subValidator.title,
                result: subResult.typeStr,
                statusInfo: subResult.statusInfo,
                partOfGroupedValidator: true,
                doctorInvocationId: analyticsTimestamp,
              ),
            );
          }
        } else {
          _analytics.send(
            Event.doctorValidatorResult(
              validatorName: validator.title,
              result: result.typeStr,
              statusInfo: result.statusInfo,
              partOfGroupedValidator: false,
              doctorInvocationId: analyticsTimestamp,
            ),
          );
        }
        // TODO(eliasyishak): remove this after migrating from package:usage,
        //  https://github.com/flutter/flutter/issues/128251
        DoctorResultEvent(validator: validator, result: result).send();
      }

      final String executionDuration = () {
        final Duration? executionTime = result.executionTime;
        if (!verbose || executionTime == null) {
          return '';
        }
        final String formatted = executionTime.inSeconds < 2
            ? getElapsedAsMilliseconds(executionTime)
            : getElapsedAsSeconds(executionTime);
        return ' [$formatted]';
      }();

      final String leadingBox =
          showColor ? result.coloredLeadingBox : result.leadingBox;
      if (result.statusInfo != null) {
        _logger.printStatus(
            '$leadingBox ${validator.title} (${result.statusInfo})$executionDuration',
            hangingIndent: result.leadingBox.length + 1);
      } else {
        _logger.printStatus('$leadingBox ${validator.title}$executionDuration',
            hangingIndent: result.leadingBox.length + 1);
      }

      for (final ValidationMessage message in result.messages) {
        if (!message.isInformation || verbose) {
          int hangingIndent = 2;
          int indent = 4;
          final String indicator =
              showColor ? message.coloredIndicator : message.indicator;
          for (final String line
              in '$indicator ${showPii ? message.message : message.piiStrippedMessage}'
                  .split(
            '\n',
          )) {
            _logger.printStatus(line,
                hangingIndent: hangingIndent, indent: indent, emphasis: true);
            // Only do hanging indent for the first line.
            hangingIndent = 0;
            indent = 6;
          }
          if (message.contextUrl != null) {
            _logger.printStatus(
              '🔨 ${message.contextUrl}',
              hangingIndent: hangingIndent,
              indent: indent,
              emphasis: true,
            );
          }
        }
      }
      if (verbose) {
        _logger.printStatus('');
      }
    }

    // Make sure there's always one line before the summary even when not verbose.
    if (!verbose) {
      _logger.printStatus('');
    }

    if (issues > 0) {
      _logger.printStatus(
        '${showColor ? globals.terminal.color('!', TerminalColor.yellow) : '!'}'
        ' Doctor found issues in $issues categor${issues > 1 ? "ies" : "y"}.',
        hangingIndent: 2,
      );
    } else {
      _logger.printStatus(
        '${showColor ? globals.terminal.color('•', TerminalColor.green) : '•'}'
        ' No issues found!',
        hangingIndent: 2,
      );
    }

    return doctorResult;
  }

  bool get canListAnything =>
      workflows.any((Workflow workflow) => workflow.canListDevices);

  bool get canLaunchAnything {
    if (FlutterTesterDevices.showFlutterTesterDevice) {
      return true;
    }
    return workflows.any((Workflow workflow) => workflow.canLaunchDevices);
  }
}

/// A validator that checks the version of Flutter, as well as some auxiliary information
/// such as the pub or Flutter cache overrides.
///
/// This is primarily useful for diagnosing issues on Github bug reports by displaying
/// specific commit information.
class FlutterValidator extends DoctorValidator {
  FlutterValidator({
    required Platform platform,
    required FlutterVersion Function() flutterVersion,
    required String Function() devToolsVersion,
    required UserMessages userMessages,
    required FileSystem fileSystem,
    required Artifacts artifacts,
    required ProcessManager processManager,
    required String Function() flutterRoot,
    required OperatingSystemUtils operatingSystemUtils,
  })  : _flutterVersion = flutterVersion,
        _devToolsVersion = devToolsVersion,
        _platform = platform,
        _userMessages = userMessages,
        _fileSystem = fileSystem,
        _artifacts = artifacts,
        _processManager = processManager,
        _flutterRoot = flutterRoot,
        _operatingSystemUtils = operatingSystemUtils,
        super('Flutter');

  final Platform _platform;
  final FlutterVersion Function() _flutterVersion;
  final String Function() _devToolsVersion;
  final String Function() _flutterRoot;
  final UserMessages _userMessages;
  final FileSystem _fileSystem;
  final Artifacts _artifacts;
  final ProcessManager _processManager;
  final OperatingSystemUtils _operatingSystemUtils;

  @override
  Future<ValidationResult> validateImpl() async {
    final List<ValidationMessage> messages = <ValidationMessage>[];
    String? versionChannel;
    String? frameworkVersion;

    try {
      final FlutterVersion version = _flutterVersion();
      final String? gitUrl = _platform.environment['FLUTTER_GIT_URL'];
      versionChannel = version.channel;
      frameworkVersion = version.frameworkVersion;

      final String flutterRoot = _flutterRoot();
      messages.add(_getFlutterVersionMessage(
          frameworkVersion, versionChannel, flutterRoot));

      _validateRequiredBinaries(flutterRoot).forEach(messages.add);
      messages.add(_getFlutterUpstreamMessage(version));
      if (gitUrl != null) {
        messages.add(ValidationMessage(_userMessages.flutterGitUrl(gitUrl)));
      }
      messages.add(
        ValidationMessage(
          _userMessages.flutterRevision(
            version.frameworkRevisionShort,
            version.frameworkAge,
            version.frameworkCommitDate,
          ),
        ),
      );
      messages.add(ValidationMessage(
          _userMessages.engineRevision(version.engineRevisionShort)));
      messages.add(ValidationMessage(
          _userMessages.dartRevision(version.dartSdkVersion)));
      messages.add(
          ValidationMessage(_userMessages.devToolsVersion(_devToolsVersion())));
      final String? pubUrl = _platform.environment[kPubDevOverride];
      if (pubUrl != null) {
        messages.add(ValidationMessage(_userMessages.pubMirrorURL(pubUrl)));
      }
      final String? storageBaseUrl =
          _platform.environment[kFlutterStorageBaseUrl];
      if (storageBaseUrl != null) {
        messages.add(
            ValidationMessage(_userMessages.flutterMirrorURL(storageBaseUrl)));
      }
    } on VersionCheckError catch (e) {
      messages.add(ValidationMessage.error(e.message));
    }

    // Check that the binaries we downloaded for this platform actually run on it.
    // If the binaries are not downloaded (because android is not enabled), then do
    // not run this check.
    final String genSnapshotPath =
        _artifacts.getArtifactPath(Artifact.genSnapshot);
    if (_fileSystem.file(genSnapshotPath).existsSync() &&
        !_genSnapshotRuns(genSnapshotPath)) {
      final StringBuffer buffer = StringBuffer();
      buffer.writeln(_userMessages.flutterBinariesDoNotRun);
      if (_platform.isLinux) {
        buffer.writeln(_userMessages.flutterBinariesLinuxRepairCommands);
      } else if (_platform.isMacOS &&
          _operatingSystemUtils.hostPlatform == HostPlatform.darwin_arm64) {
        buffer.writeln(
          'Flutter requires the Rosetta translation environment on ARM Macs. Try running:',
        );
        buffer.writeln(
            '  sudo softwareupdate --install-rosetta --agree-to-license');
      }
      messages.add(ValidationMessage.error(buffer.toString()));
    }

    ValidationType valid;
    if (messages.every((ValidationMessage message) => message.isInformation)) {
      valid = ValidationType.success;
    } else {
      // The issues for this validator stem from broken git configuration of the local install;
      // in that case, make it clear that it is fine to continue, but freshness check/upgrades
      // won't be supported.
      valid = ValidationType.partial;
      messages.add(
          ValidationMessage(_userMessages.flutterValidatorErrorIntentional));
    }

    return ValidationResult(
      valid,
      messages,
      statusInfo: _userMessages.flutterStatusInfo(
        versionChannel,
        frameworkVersion,
        _operatingSystemUtils.name,
        _platform.localeName,
      ),
    );
  }

  ValidationMessage _getFlutterVersionMessage(
    String frameworkVersion,
    String versionChannel,
    String flutterRoot,
  ) {
    String flutterVersionMessage = _userMessages.flutterVersion(
      frameworkVersion,
      versionChannel,
      flutterRoot,
    );

    // The tool sets the channel as kUserBranch, if the current branch is on a
    // "detached HEAD" state, doesn't have an upstream, or is on a user branch,
    // and sets the frameworkVersion as "0.0.0-unknown" if "git describe" on
    // HEAD doesn't produce an expected format to be parsed for the frameworkVersion.
    if (versionChannel != kUserBranch && frameworkVersion != '0.0.0-unknown') {
      return ValidationMessage(flutterVersionMessage);
    }
    if (versionChannel == kUserBranch) {
      flutterVersionMessage =
          '$flutterVersionMessage\n${_userMessages.flutterUnknownChannel}';
    }
    if (frameworkVersion == '0.0.0-unknown') {
      flutterVersionMessage =
          '$flutterVersionMessage\n${_userMessages.flutterUnknownVersion}';
    }
    return ValidationMessage.hint(flutterVersionMessage);
  }

  List<ValidationMessage> _validateRequiredBinaries(String flutterRoot) {
    final ValidationMessage? flutterWarning =
        _validateSdkBinary('flutter', flutterRoot);
    final ValidationMessage? dartWarning =
        _validateSdkBinary('dart', flutterRoot);
    return <ValidationMessage>[
      if (flutterWarning != null) flutterWarning,
      if (dartWarning != null) dartWarning,
    ];
  }

  /// Return a warning if the provided [binary] on the user's path does not
  /// resolve within the Flutter SDK.
  ValidationMessage? _validateSdkBinary(String binary, String flutterRoot) {
    final String flutterBinDir = _fileSystem.path.join(flutterRoot, 'bin');

    final File? flutterBin = _operatingSystemUtils.which(binary);
    if (flutterBin == null) {
      return ValidationMessage.hint(
        'The $binary binary is not on your path. Consider adding '
        '$flutterBinDir to your path.',
      );
    }
    final String resolvedFlutterPath = flutterBin.resolveSymbolicLinksSync();
    if (!_filePathContainsDirPath(flutterRoot, resolvedFlutterPath)) {
      final String hint = 'Warning: `$binary` on your path resolves to '
          '$resolvedFlutterPath, which is not inside your current Flutter '
          'SDK checkout at $flutterRoot. Consider adding $flutterBinDir to '
          'the front of your path.';
      return ValidationMessage.hint(hint);
    }
    return null;
  }

  bool _filePathContainsDirPath(String directory, String file) {
    // calling .canonicalize() will normalize for alphabetic case and path
    // separators
    return _fileSystem.path.canonicalize(file).startsWith(
        _fileSystem.path.canonicalize(directory) + _fileSystem.path.separator);
  }

  ValidationMessage _getFlutterUpstreamMessage(FlutterVersion version) {
    final String? repositoryUrl = version.repositoryUrl;
    final VersionCheckError? upstreamValidationError =
        VersionUpstreamValidator(version: version, platform: _platform).run();

    // VersionUpstreamValidator can produce an error if repositoryUrl is null
    if (upstreamValidationError != null) {
      final String errorMessage = upstreamValidationError.message;
      if (errorMessage.contains(
          'could not determine the remote upstream which is being tracked')) {
        return ValidationMessage.hint(
            _userMessages.flutterUpstreamRepositoryUnknown);
      }
      // At this point, repositoryUrl must not be null
      if (errorMessage
          .contains('Flutter SDK is tracking a non-standard remote')) {
        return ValidationMessage.hint(
          _userMessages.flutterUpstreamRepositoryUrlNonStandard(repositoryUrl!),
        );
      }
      if (errorMessage.contains(
        'Either remove "FLUTTER_GIT_URL" from the environment or set it to',
      )) {
        return ValidationMessage.hint(
          _userMessages.flutterUpstreamRepositoryUrlEnvMismatch(repositoryUrl!),
        );
      }
    }
    return ValidationMessage(
        _userMessages.flutterUpstreamRepositoryUrl(repositoryUrl!));
  }

  bool _genSnapshotRuns(String genSnapshotPath) {
    const int kExpectedExitCode = 255;
    try {
      return _processManager.runSync(<String>[genSnapshotPath]).exitCode ==
          kExpectedExitCode;
    } on Exception {
      return false;
    }
  }
}

class DeviceValidator extends DoctorValidator {
  // TODO(jmagman): Make required once g3 rolls and is updated.
  DeviceValidator({DeviceManager? deviceManager, UserMessages? userMessages})
      : _deviceManager = deviceManager ?? globals.deviceManager!,
        _userMessages = userMessages ?? globals.userMessages,
        super('Connected device');

  final DeviceManager _deviceManager;
  final UserMessages _userMessages;

  @override
  String get slowWarning => 'Scanning for devices is taking a long time...';

  @override
  Future<ValidationResult> validateImpl() async {
    final List<Device> devices = await _deviceManager.refreshAllDevices(
      timeout: DeviceManager.minimumWirelessDeviceDiscoveryTimeout,
    );
    List<ValidationMessage> installedMessages = <ValidationMessage>[];
    if (devices.isNotEmpty) {
      installedMessages = (await Device.descriptions(
        devices,
      ))
          .map<ValidationMessage>((String msg) => ValidationMessage(msg))
          .toList();
    }

    List<ValidationMessage> diagnosticMessages = <ValidationMessage>[];
    final List<String> diagnostics =
        await _deviceManager.getDeviceDiagnostics();
    if (diagnostics.isNotEmpty) {
      diagnosticMessages = diagnostics
          .map<ValidationMessage>(
              (String message) => ValidationMessage.hint(message))
          .toList();
    } else if (devices.isEmpty) {
      diagnosticMessages = <ValidationMessage>[
        ValidationMessage.hint(_userMessages.devicesMissing),
      ];
    }

    if (devices.isEmpty) {
      return ValidationResult(ValidationType.notAvailable, diagnosticMessages);
    } else if (diagnostics.isNotEmpty) {
      installedMessages.addAll(diagnosticMessages);
      return ValidationResult(
        ValidationType.success,
        installedMessages,
        statusInfo: _userMessages.devicesAvailable(devices.length),
      );
    } else {
      return ValidationResult(
        ValidationType.success,
        installedMessages,
        statusInfo: _userMessages.devicesAvailable(devices.length),
      );
    }
  }
}

/// Wrapper for doctor to run multiple times with PII and without, running the validators only once.
class DoctorText {
  DoctorText(BufferLogger logger,
      {SystemClock? clock, @visibleForTesting Doctor? doctor})
      : _doctor = doctor ??
            Doctor(logger: logger, clock: clock ?? globals.systemClock),
        _logger = logger;

  final BufferLogger _logger;
  final Doctor _doctor;
  bool _sendDoctorEvent = true;

  late final Future<String> text = _runDiagnosis(true);
  late final Future<String> piiStrippedText = _runDiagnosis(false);

  // Start the validator tasks only once.
  late final List<ValidatorTask> _validatorTasks =
      _doctor.startValidatorTasks();

  Future<String> _runDiagnosis(bool showPii) async {
    try {
      await _doctor.diagnose(
        startedValidatorTasks: _validatorTasks,
        showPii: showPii,
        sendEvent: _sendDoctorEvent,
      );
      // Do not send the doctor event a second time.
      _sendDoctorEvent = false;
      final String text = _logger.statusText;
      _logger.clear();
      return text;
    } on Exception catch (error, trace) {
      return 'encountered exception: $error\n\n${trace.toString().trim()}\n';
    }
  }
}
