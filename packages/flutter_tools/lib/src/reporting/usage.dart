// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of 'reporting.dart';

const String _kFlutterUA = 'UA-67589403-6';

abstract class Usage {
  /// Create a new Usage instance; [versionOverride], [configDirOverride], and
  /// [logFile] are used for testing.
  factory Usage({
    String settingsName = 'flutter',
    String? versionOverride,
    String? configDirOverride,
    String? logFile,
    AnalyticsFactory? analyticsIOFactory,
    FirstRunMessenger? firstRunMessenger,
    required bool runningOnBot,
  }) => _DefaultUsage.initialize(
    settingsName: settingsName,
    versionOverride: versionOverride,
    configDirOverride: configDirOverride,
    logFile: logFile,
    analyticsIOFactory: analyticsIOFactory,
    runningOnBot: runningOnBot,
    firstRunMessenger: firstRunMessenger,
  );

  /// Uses the global [Usage] instance to send a 'command' to analytics.
  static void command(String command, {CustomDimensions? parameters}) =>
      globals.flutterUsage.sendCommand(command, parameters: parameters);

  /// Whether analytics reporting should be suppressed.
  bool get suppressAnalytics;

  /// Suppress analytics for this session.
  set suppressAnalytics(bool value);

  /// Whether analytics reporting is enabled.
  bool get enabled;

  /// Enable or disable reporting analytics.
  set enabled(bool value);

  /// A stable randomly generated UUID used to deduplicate multiple identical
  /// reports coming from the same computer.
  String get clientId;

  /// Sends a 'command' to the underlying analytics implementation.
  ///
  /// Using [command] above is preferred to ensure that the parameter
  /// keys are well-defined in [CustomDimensions] above.
  void sendCommand(String command, {CustomDimensions? parameters});

  /// Sends an 'event' to the underlying analytics implementation.
  ///
  /// This method should not be used directly, instead see the
  /// event types defined in this directory in `events.dart`.
  @visibleForOverriding
  @visibleForTesting
  void sendEvent(
    String category,
    String parameter, {
    String? label,
    int? value,
    CustomDimensions? parameters,
  });

  /// Sends timing information to the underlying analytics implementation.
  void sendTiming(String category, String variableName, Duration duration, {String? label});

  /// Sends an exception to the underlying analytics implementation.
  void sendException(dynamic exception);

  /// Fires whenever analytics data is sent over the network.
  @visibleForTesting
  Stream<Map<String, dynamic>> get onSend;

  /// Returns when the last analytics event has been sent, or after a fixed
  /// (short) delay, whichever is less.
  Future<void> ensureAnalyticsSent();

  /// Prints a welcome message that informs the tool user about the collection
  /// of anonymous usage information.
  void printWelcome();
}

typedef AnalyticsFactory =
    Analytics Function(
      String trackingId,
      String applicationName,
      String applicationVersion, {
      String analyticsUrl,
      Directory? documentDirectory,
    });

Analytics _defaultAnalyticsIOFactory(
  String trackingId,
  String applicationName,
  String applicationVersion, {
  String? analyticsUrl,
  Directory? documentDirectory,
}) {
  return AnalyticsIO(
    trackingId,
    applicationName,
    applicationVersion,
    analyticsUrl: analyticsUrl,
    documentDirectory: documentDirectory,
  );
}

class _DefaultUsage implements Usage {
  _DefaultUsage._({
    required bool suppressAnalytics,
    required Analytics analytics,
    required this.firstRunMessenger,
    required SystemClock clock,
  }) : _suppressAnalytics = suppressAnalytics,
       _analytics = analytics,
       _clock = clock;

  static _DefaultUsage initialize({
    String settingsName = 'flutter',
    String? versionOverride,
    String? configDirOverride,
    String? logFile,
    AnalyticsFactory? analyticsIOFactory,
    required FirstRunMessenger? firstRunMessenger,
    required bool runningOnBot,
  }) {
    final FlutterVersion flutterVersion = globals.flutterVersion;
    final String version =
        versionOverride ?? flutterVersion.getVersionString(redactUnknownBranches: true);
    final bool suppressEnvFlag =
        globals.platform.environment['FLUTTER_SUPPRESS_ANALYTICS'] == 'true';
    final String? logFilePath =
        logFile ?? globals.platform.environment['FLUTTER_ANALYTICS_LOG_FILE'];
    final bool usingLogFile = logFilePath != null && logFilePath.isNotEmpty;

    final AnalyticsFactory analyticsFactory = analyticsIOFactory ?? _defaultAnalyticsIOFactory;
    bool suppressAnalytics = false;
    bool skipAnalyticsSessionSetup = false;
    Analytics? setupAnalytics;
    if ( // To support testing, only allow other signals to suppress analytics
    // when analytics are not being shunted to a file.
    !usingLogFile &&
        ( // Ignore local user branches.
        version.startsWith('[user-branch]') ||
            // Many CI systems don't do a full git checkout.
                version
                .endsWith('/unknown') ||
            // Ignore bots.
            runningOnBot ||
            // Ignore when suppressed by FLUTTER_SUPPRESS_ANALYTICS.
            suppressEnvFlag)) {
      // If we think we're running on a CI system, suppress sending analytics.
      suppressAnalytics = true;
      setupAnalytics = AnalyticsMock();
      skipAnalyticsSessionSetup = true;
    }
    if (usingLogFile) {
      setupAnalytics ??= LogToFileAnalytics(logFilePath);
    } else {
      try {
        ErrorHandlingFileSystem.noExitOnFailure(() {
          setupAnalytics = analyticsFactory(
            _kFlutterUA,
            settingsName,
            version,
            documentDirectory:
                configDirOverride != null ? globals.fs.directory(configDirOverride) : null,
          );
        });
      } on Exception catch (e) {
        globals.printTrace('Failed to initialize analytics reporting: $e');
        suppressAnalytics = true;
        setupAnalytics ??= AnalyticsMock();
        skipAnalyticsSessionSetup = true;
      }
    }

    final Analytics analytics = setupAnalytics!;
    if (!skipAnalyticsSessionSetup) {
      // Report a more detailed OS version string than package:usage does by default.
      analytics.setSessionValue(CustomDimensionsEnum.sessionHostOsDetails.cdKey, globals.os.name);
      // Send the branch name as the "channel".
      analytics.setSessionValue(
        CustomDimensionsEnum.sessionChannelName.cdKey,
        flutterVersion.getBranchName(redactUnknownBranches: true),
      );
      // For each flutter experimental feature, record a session value in a comma
      // separated list.
      final String enabledFeatures = allFeatures
          .where((Feature feature) {
            final String? configSetting = feature.configSetting;
            return configSetting != null && globals.config.getValue(configSetting) == true;
          })
          .map((Feature feature) => feature.configSetting)
          .join(',');
      analytics.setSessionValue(CustomDimensionsEnum.enabledFlutterFeatures.cdKey, enabledFeatures);

      // Record the host as the application installer ID - the context that flutter_tools is running in.
      if (globals.platform.environment.containsKey('FLUTTER_HOST')) {
        analytics.setSessionValue('aiid', globals.platform.environment['FLUTTER_HOST']);
      }
      analytics.analyticsOpt = AnalyticsOpt.optOut;
    }

    return _DefaultUsage._(
      suppressAnalytics: suppressAnalytics,
      analytics: analytics,
      firstRunMessenger: firstRunMessenger,
      clock: globals.systemClock,
    );
  }

  final Analytics _analytics;
  final FirstRunMessenger? firstRunMessenger;

  bool _printedWelcome = false;
  bool _suppressAnalytics = false;
  final SystemClock _clock;

  @override
  bool get suppressAnalytics => _suppressAnalytics || _analytics.firstRun;

  @override
  set suppressAnalytics(bool value) {
    _suppressAnalytics = value;
  }

  @override
  bool get enabled => _analytics.enabled;

  @override
  set enabled(bool value) {
    _analytics.enabled = value;
  }

  @override
  String get clientId => _analytics.clientId;

  @override
  void sendCommand(String command, {CustomDimensions? parameters}) {
    if (suppressAnalytics) {
      return;
    }

    _analytics.sendScreenView(
      command,
      parameters:
          CustomDimensions(localTime: formatDateTime(_clock.now())).merge(parameters).toMap(),
    );
  }

  @override
  void sendEvent(
    String category,
    String parameter, {
    String? label,
    int? value,
    CustomDimensions? parameters,
  }) {
    if (suppressAnalytics) {
      return;
    }

    _analytics.sendEvent(
      category,
      parameter,
      label: label,
      value: value,
      parameters:
          CustomDimensions(localTime: formatDateTime(_clock.now())).merge(parameters).toMap(),
    );
  }

  @override
  void sendTiming(String category, String variableName, Duration duration, {String? label}) {
    if (suppressAnalytics) {
      return;
    }
    _analytics.sendTiming(variableName, duration.inMilliseconds, category: category, label: label);
  }

  @override
  void sendException(dynamic exception) {
    if (suppressAnalytics) {
      return;
    }
    _analytics.sendException(exception.runtimeType.toString());
  }

  @override
  Stream<Map<String, dynamic>> get onSend => _analytics.onSend;

  @override
  Future<void> ensureAnalyticsSent() async {
    // TODO(devoncarew): This may delay tool exit and could cause some analytics
    // events to not be reported. Perhaps we could send the analytics pings
    // out-of-process from flutter_tools?
    await _analytics.waitForLastPing(timeout: const Duration(milliseconds: 250));
  }

  @override
  void printWelcome() {
    // Only print once per run.
    if (_printedWelcome) {
      return;
    }
    // Display the welcome message if this is the first run of the tool or if
    // the license terms have changed since it was last displayed.
    final FirstRunMessenger? messenger = firstRunMessenger;
    if (messenger != null && messenger.shouldDisplayLicenseTerms()) {
      globals.printStatus('');
      globals.printStatus(messenger.licenseTerms, emphasis: true);
      _printedWelcome = true;
      messenger.confirmLicenseTermsDisplayed();
    }
  }
}

// An Analytics mock that logs to file. Unimplemented methods goes to stdout.
// But stdout can't be used for testing since wrapper scripts like
// xcode_backend.sh etc manipulates them.
class LogToFileAnalytics extends AnalyticsMock {
  LogToFileAnalytics(String logFilePath)
    : logFile = globals.fs.file(logFilePath)..createSync(recursive: true),
      super(true);

  final File logFile;
  final Map<String, String> _sessionValues = <String, String>{};

  final StreamController<Map<String, dynamic>> _sendController =
      StreamController<Map<String, dynamic>>.broadcast(sync: true);

  @override
  Stream<Map<String, dynamic>> get onSend => _sendController.stream;

  @override
  Future<void> sendScreenView(String viewName, {Map<String, String>? parameters}) {
    if (!enabled) {
      return Future<void>.value();
    }
    parameters ??= <String, String>{};
    parameters['viewName'] = viewName;
    parameters.addAll(_sessionValues);
    _sendController.add(parameters);
    logFile.writeAsStringSync('screenView $parameters\n', mode: FileMode.append);
    return Future<void>.value();
  }

  @override
  Future<void> sendEvent(
    String category,
    String action, {
    String? label,
    int? value,
    Map<String, String>? parameters,
  }) {
    if (!enabled) {
      return Future<void>.value();
    }
    parameters ??= <String, String>{};
    parameters['category'] = category;
    parameters['action'] = action;
    _sendController.add(parameters);
    logFile.writeAsStringSync('event $parameters\n', mode: FileMode.append);
    return Future<void>.value();
  }

  @override
  Future<void> sendTiming(String variableName, int time, {String? category, String? label}) {
    if (!enabled) {
      return Future<void>.value();
    }
    final Map<String, String> parameters = <String, String>{
      'variableName': variableName,
      'time': '$time',
      if (category != null) 'category': category,
      if (label != null) 'label': label,
    };
    _sendController.add(parameters);
    logFile.writeAsStringSync('timing $parameters\n', mode: FileMode.append);
    return Future<void>.value();
  }

  @override
  void setSessionValue(String param, dynamic value) {
    _sessionValues[param] = value.toString();
  }
}

/// Create a testing Usage instance.
///
/// All sent events, exceptions, timings, and pages are
/// buffered on the object and can be inspected later.
@visibleForTesting
class TestUsage implements Usage {
  final List<TestUsageCommand> commands = <TestUsageCommand>[];
  final List<TestUsageEvent> events = <TestUsageEvent>[];
  final List<dynamic> exceptions = <dynamic>[];
  final List<TestTimingEvent> timings = <TestTimingEvent>[];
  int ensureAnalyticsSentCalls = 0;
  bool _printedWelcome = false;

  @override
  bool enabled = true;

  @override
  bool suppressAnalytics = false;

  @override
  String get clientId => 'test-client';

  /// Confirms if the [printWelcome] method was invoked.
  bool get printedWelcome => _printedWelcome;

  @override
  Future<void> ensureAnalyticsSent() async {
    ensureAnalyticsSentCalls++;
  }

  @override
  Stream<Map<String, dynamic>> get onSend => throw UnimplementedError();

  @override
  void printWelcome() {
    _printedWelcome = true;
  }

  @override
  void sendCommand(String command, {CustomDimensions? parameters}) {
    commands.add(TestUsageCommand(command, parameters: parameters));
  }

  @override
  void sendEvent(
    String category,
    String parameter, {
    String? label,
    int? value,
    CustomDimensions? parameters,
  }) {
    events.add(
      TestUsageEvent(category, parameter, label: label, value: value, parameters: parameters),
    );
  }

  @override
  void sendException(dynamic exception) {
    exceptions.add(exception);
  }

  @override
  void sendTiming(String category, String variableName, Duration duration, {String? label}) {
    timings.add(TestTimingEvent(category, variableName, duration, label: label));
  }
}

@visibleForTesting
@immutable
class TestUsageCommand {
  const TestUsageCommand(this.command, {this.parameters});

  final String command;
  final CustomDimensions? parameters;

  @override
  bool operator ==(Object other) {
    return other is TestUsageCommand && other.command == command && other.parameters == parameters;
  }

  @override
  int get hashCode => Object.hash(command, parameters);

  @override
  String toString() => 'TestUsageCommand($command, parameters:$parameters)';
}

@visibleForTesting
@immutable
class TestUsageEvent {
  const TestUsageEvent(this.category, this.parameter, {this.label, this.value, this.parameters});

  final String category;
  final String parameter;
  final String? label;
  final int? value;
  final CustomDimensions? parameters;

  @override
  bool operator ==(Object other) {
    return other is TestUsageEvent &&
        other.category == category &&
        other.parameter == parameter &&
        other.label == label &&
        other.value == value &&
        other.parameters == parameters;
  }

  @override
  int get hashCode => Object.hash(category, parameter, label, value, parameters);

  @override
  String toString() =>
      'TestUsageEvent($category, $parameter, label:$label, value:$value, parameters:$parameters)';
}

@visibleForTesting
@immutable
class TestTimingEvent {
  const TestTimingEvent(this.category, this.variableName, this.duration, {this.label});

  final String category;
  final String variableName;
  final Duration duration;
  final String? label;

  @override
  bool operator ==(Object other) {
    return other is TestTimingEvent &&
        other.category == category &&
        other.variableName == variableName &&
        other.duration == duration &&
        other.label == label;
  }

  @override
  int get hashCode => Object.hash(category, variableName, duration, label);

  @override
  String toString() => 'TestTimingEvent($category, $variableName, $duration, label:$label)';
}

bool _mapsEqual(Map<dynamic, dynamic>? a, Map<dynamic, dynamic>? b) {
  if (a == b) {
    return true;
  }
  if (a == null || b == null) {
    return false;
  }
  if (a.length != b.length) {
    return false;
  }

  for (final dynamic k in a.keys) {
    final dynamic bValue = b[k];
    if (bValue == null && !b.containsKey(k)) {
      return false;
    }
    if (bValue != a[k]) {
      return false;
    }
  }

  return true;
}
