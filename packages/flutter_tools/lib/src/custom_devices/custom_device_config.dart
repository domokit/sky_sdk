// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import '../base/platform.dart';
import '../build_info.dart';

/// Quiver has this, but unfortunately we can't depend on it bc flutter_tools
/// uses non-nullsafe quiver by default (because of dwds).
bool _listsEqual(List<dynamic>? a, List<dynamic>? b) {
  if (a == b) {
    return true;
  }
  if (a == null || b == null) {
    return false;
  }
  if (a.length != b.length) {
    return false;
  }

  return a.asMap().entries.every((MapEntry<int, dynamic> e) => e.value == b[e.key]);
}

/// The normal [RegExp.==] operator is inherited from [Object], so only
/// returns true when the regexes are the same instance.
///
/// This function instead _should_ return true when the regexes are
/// functionally the same, i.e. when they have the same matches & captures for
/// any given input. At least that's the goal, in reality this has lots of false
/// negatives (for example when the flags differ). Still better than [RegExp.==].
bool _regexesEqual(RegExp? a, RegExp? b) {
  if (a == b) {
    return true;
  }
  if (a == null || b == null) {
    return false;
  }

  return a.pattern == b.pattern &&
      a.isMultiLine == b.isMultiLine &&
      a.isCaseSensitive == b.isCaseSensitive &&
      a.isUnicode == b.isUnicode &&
      a.isDotAll == b.isDotAll;
}

/// Something went wrong while trying to load the custom devices config from the
/// JSON representation. Maybe some value is missing, maybe something has the
/// wrong type, etc.
@immutable
class CustomDeviceRevivalException implements Exception {
  const CustomDeviceRevivalException(this.message);

  const CustomDeviceRevivalException.fromDescriptions(
    String fieldDescription,
    String expectedValueDescription,
  ) : message = 'Expected $fieldDescription to be $expectedValueDescription.';

  final String message;

  @override
  String toString() {
    return message;
  }

  @override
  bool operator ==(Object other) {
    return (other is CustomDeviceRevivalException) && (other.message == message);
  }

  @override
  int get hashCode => message.hashCode;
}

/// A single configured custom device.
///
/// In the custom devices config file on disk, there may be multiple custom
/// devices configured.
@immutable
class CustomDeviceConfig {
  const CustomDeviceConfig({
    required this.id,
    required this.label,
    required this.sdkNameAndVersion,
    this.platform,
    required this.enabled,
    required this.pingCommand,
    this.pingSuccessRegex,
    required this.postBuildCommand,
    required this.installCommand,
    required this.uninstallCommand,
    required this.runDebugCommand,
    this.forwardPortCommand,
    this.forwardPortSuccessRegex,
    this.screenshotCommand,
  }) : assert(forwardPortCommand == null || forwardPortSuccessRegex != null),
       assert(
         platform == null ||
             platform == TargetPlatform.linux_x64 ||
             platform == TargetPlatform.linux_arm64,
       );

  /// Create a CustomDeviceConfig from some JSON value.
  /// If anything fails internally (some value doesn't have the right type,
  /// some value is missing, etc) a [CustomDeviceRevivalException] with the description
  /// of the error is thrown. (No exceptions/errors other than JsonRevivalException
  /// should ever be thrown by this factory.)
  factory CustomDeviceConfig.fromJson(dynamic json) {
    final Map<String, dynamic> typedMap = _castJsonObject(
      json,
      'device configuration',
      'a JSON object',
    );

    final List<String>? forwardPortCommand = _castStringListOrNull(
      typedMap[_kForwardPortCommand],
      _kForwardPortCommand,
      'null or array of strings with at least one element',
      minLength: 1,
    );

    final RegExp? forwardPortSuccessRegex = _convertToRegexOrNull(
      typedMap[_kForwardPortSuccessRegex],
      _kForwardPortSuccessRegex,
      'null or string-ified regex',
    );

    final String? archString = _castStringOrNull(
      typedMap[_kPlatform],
      _kPlatform,
      'null or one of linux-arm64, linux-x64',
    );

    late TargetPlatform? platform;
    try {
      platform = archString == null ? null : getTargetPlatformForName(archString);
    } on UnsupportedError {
      throw const CustomDeviceRevivalException.fromDescriptions(
        _kPlatform,
        'null or one of linux-arm64, linux-x64',
      );
    }

    if (platform != null &&
        platform != TargetPlatform.linux_arm64 &&
        platform != TargetPlatform.linux_x64) {
      throw const CustomDeviceRevivalException.fromDescriptions(
        _kPlatform,
        'null or one of linux-arm64, linux-x64',
      );
    }

    if (forwardPortCommand != null && forwardPortSuccessRegex == null) {
      throw const CustomDeviceRevivalException(
        'When forwardPort is given, forwardPortSuccessRegex must be specified too.',
      );
    }

    return CustomDeviceConfig(
      id: _castString(typedMap[_kId], _kId, 'a string'),
      label: _castString(typedMap[_kLabel], _kLabel, 'a string'),
      sdkNameAndVersion: _castString(
        typedMap[_kSdkNameAndVersion],
        _kSdkNameAndVersion,
        'a string',
      ),
      platform: platform,
      enabled: _castBool(typedMap[_kEnabled], _kEnabled, 'a boolean'),
      pingCommand: _castStringList(
        typedMap[_kPingCommand],
        _kPingCommand,
        'array of strings with at least one element',
        minLength: 1,
      ),
      pingSuccessRegex: _convertToRegexOrNull(
        typedMap[_kPingSuccessRegex],
        _kPingSuccessRegex,
        'null or string-ified regex',
      ),
      postBuildCommand: _castStringListOrNull(
        typedMap[_kPostBuildCommand],
        _kPostBuildCommand,
        'null or array of strings with at least one element',
        minLength: 1,
      ),
      installCommand: _castStringList(
        typedMap[_kInstallCommand],
        _kInstallCommand,
        'array of strings with at least one element',
        minLength: 1,
      ),
      uninstallCommand: _castStringList(
        typedMap[_kUninstallCommand],
        _kUninstallCommand,
        'array of strings with at least one element',
        minLength: 1,
      ),
      runDebugCommand: _castStringList(
        typedMap[_kRunDebugCommand],
        _kRunDebugCommand,
        'array of strings with at least one element',
        minLength: 1,
      ),
      forwardPortCommand: forwardPortCommand,
      forwardPortSuccessRegex: forwardPortSuccessRegex,
      screenshotCommand: _castStringListOrNull(
        typedMap[_kScreenshotCommand],
        _kScreenshotCommand,
        'array of strings with at least one element',
        minLength: 1,
      ),
    );
  }

  static const String _kId = 'id';
  static const String _kLabel = 'label';
  static const String _kSdkNameAndVersion = 'sdkNameAndVersion';
  static const String _kPlatform = 'platform';
  static const String _kEnabled = 'enabled';
  static const String _kPingCommand = 'ping';
  static const String _kPingSuccessRegex = 'pingSuccessRegex';
  static const String _kPostBuildCommand = 'postBuild';
  static const String _kInstallCommand = 'install';
  static const String _kUninstallCommand = 'uninstall';
  static const String _kRunDebugCommand = 'runDebug';
  static const String _kForwardPortCommand = 'forwardPort';
  static const String _kForwardPortSuccessRegex = 'forwardPortSuccessRegex';
  static const String _kScreenshotCommand = 'screenshot';

  /// An example device config used for creating the default config file.
  /// Uses windows-specific ping and pingSuccessRegex. For the linux and macOs
  /// example config, see [exampleUnix].
  static final CustomDeviceConfig exampleWindows = CustomDeviceConfig(
    id: 'pi',
    label: 'Raspberry Pi',
    sdkNameAndVersion: 'Raspberry Pi 4 Model B+',
    platform: TargetPlatform.linux_arm64,
    enabled: false,
    pingCommand: const <String>['ping', '-w', '500', '-n', '1', 'raspberrypi'],
    pingSuccessRegex: RegExp(r'[<=]\d+ms'),
    postBuildCommand: null,
    installCommand: const <String>[
      'scp',
      '-r',
      '-o',
      'BatchMode=yes',
      r'${localPath}',
      r'pi@raspberrypi:/tmp/${appName}',
    ],
    uninstallCommand: const <String>[
      'ssh',
      '-o',
      'BatchMode=yes',
      'pi@raspberrypi',
      r'rm -rf "/tmp/${appName}"',
    ],
    runDebugCommand: const <String>[
      'ssh',
      '-o',
      'BatchMode=yes',
      'pi@raspberrypi',
      r'flutter-pi "/tmp/${appName}"',
    ],
    forwardPortCommand: const <String>[
      'ssh',
      '-o',
      'BatchMode=yes',
      '-o',
      'ExitOnForwardFailure=yes',
      '-L',
      r'127.0.0.1:${hostPort}:127.0.0.1:${devicePort}',
      'pi@raspberrypi',
      "echo 'Port forwarding success'; read",
    ],
    forwardPortSuccessRegex: RegExp('Port forwarding success'),
    screenshotCommand: const <String>[
      'ssh',
      '-o',
      'BatchMode=yes',
      'pi@raspberrypi',
      r"fbgrab /tmp/screenshot.png && cat /tmp/screenshot.png | base64 | tr -d ' \n\t'",
    ],
  );

  /// An example device config used for creating the default config file.
  /// Uses ping and pingSuccessRegex values that only work on linux or macOs.
  /// For the Windows example config, see [exampleWindows].
  static final CustomDeviceConfig exampleUnix = exampleWindows.copyWith(
    pingCommand: const <String>['ping', '-w', '1', '-c', '1', 'raspberrypi'],
    explicitPingSuccessRegex: true,
  );

  /// Returns an example custom device config that works on the given host platform.
  ///
  /// This is not the platform of the target device, it's the platform of the
  /// development machine. Examples for different platforms may be different
  /// because for example the ping command is different on Windows or Linux/macOS.
  static CustomDeviceConfig getExampleForPlatform(Platform platform) {
    if (platform.isWindows) {
      return exampleWindows;
    }
    if (platform.isLinux || platform.isMacOS) {
      return exampleUnix;
    }
    throw UnsupportedError('Unsupported operating system');
  }

  final String id;
  final String label;
  final String sdkNameAndVersion;
  final TargetPlatform? platform;
  final bool enabled;
  final List<String> pingCommand;
  final RegExp? pingSuccessRegex;
  final List<String>? postBuildCommand;
  final List<String> installCommand;
  final List<String> uninstallCommand;
  final List<String> runDebugCommand;
  final List<String>? forwardPortCommand;
  final RegExp? forwardPortSuccessRegex;
  final List<String>? screenshotCommand;

  /// Returns true when this custom device config uses port forwarding,
  /// which is the case when [forwardPortCommand] is not null.
  bool get usesPortForwarding => forwardPortCommand != null;

  /// Returns true when this custom device config supports screenshotting,
  /// which is the case when the [screenshotCommand] is not null.
  bool get supportsScreenshotting => screenshotCommand != null;

  /// Invokes and returns the result of [closure].
  ///
  /// If anything at all is thrown when executing the closure, a
  /// [CustomDeviceRevivalException] is thrown with the given [fieldDescription] and
  /// [expectedValueDescription].
  static T _maybeRethrowAsRevivalException<T>(
    T Function() closure,
    String fieldDescription,
    String expectedValueDescription,
  ) {
    try {
      return closure();
    } on Object {
      throw CustomDeviceRevivalException.fromDescriptions(
        fieldDescription,
        expectedValueDescription,
      );
    }
  }

  /// Tries to make a string-keyed, non-null map from [value].
  ///
  /// If the value is null or not a valid string-keyed map, a [CustomDeviceRevivalException]
  /// with the given [fieldDescription] and [expectedValueDescription] is thrown.
  static Map<String, dynamic> _castJsonObject(
    dynamic value,
    String fieldDescription,
    String expectedValueDescription,
  ) {
    if (value == null) {
      throw CustomDeviceRevivalException.fromDescriptions(
        fieldDescription,
        expectedValueDescription,
      );
    }

    return _maybeRethrowAsRevivalException(
      () => Map<String, dynamic>.from(value as Map<dynamic, dynamic>),
      fieldDescription,
      expectedValueDescription,
    );
  }

  /// Tries to cast [value] to a bool.
  ///
  /// If the value is null or not a bool, a [CustomDeviceRevivalException] with the given
  /// [fieldDescription] and [expectedValueDescription] is thrown.
  static bool _castBool(dynamic value, String fieldDescription, String expectedValueDescription) {
    if (value == null) {
      throw CustomDeviceRevivalException.fromDescriptions(
        fieldDescription,
        expectedValueDescription,
      );
    }

    return _maybeRethrowAsRevivalException(
      () => value as bool,
      fieldDescription,
      expectedValueDescription,
    );
  }

  /// Tries to cast [value] to a String.
  ///
  /// If the value is null or not a String, a [CustomDeviceRevivalException] with the given
  /// [fieldDescription] and [expectedValueDescription] is thrown.
  static String _castString(
    dynamic value,
    String fieldDescription,
    String expectedValueDescription,
  ) {
    if (value == null) {
      throw CustomDeviceRevivalException.fromDescriptions(
        fieldDescription,
        expectedValueDescription,
      );
    }

    return _maybeRethrowAsRevivalException(
      () => value as String,
      fieldDescription,
      expectedValueDescription,
    );
  }

  /// Tries to cast [value] to a nullable String.
  ///
  /// If the value not null and not a String, a [CustomDeviceRevivalException] with the given
  /// [fieldDescription] and [expectedValueDescription] is thrown.
  static String? _castStringOrNull(
    dynamic value,
    String fieldDescription,
    String expectedValueDescription,
  ) {
    if (value == null) {
      return null;
    }

    return _castString(value, fieldDescription, expectedValueDescription);
  }

  /// Tries to make a list of strings from [value].
  ///
  /// If the value is null or not a list containing only string values,
  /// a [CustomDeviceRevivalException] with the given [fieldDescription] and
  /// [expectedValueDescription] is thrown.
  static List<String> _castStringList(
    dynamic value,
    String fieldDescription,
    String expectedValueDescription, {
    int minLength = 0,
  }) {
    if (value == null) {
      throw CustomDeviceRevivalException.fromDescriptions(
        fieldDescription,
        expectedValueDescription,
      );
    }

    final List<String> list = _maybeRethrowAsRevivalException(
      () => List<String>.from(value as Iterable<dynamic>),
      fieldDescription,
      expectedValueDescription,
    );

    if (list.length < minLength) {
      throw CustomDeviceRevivalException.fromDescriptions(
        fieldDescription,
        expectedValueDescription,
      );
    }

    return list;
  }

  /// Tries to make a list of strings from [value], or returns null if [value]
  /// is null.
  ///
  /// If the value is not null and not a list containing only string values,
  /// a [CustomDeviceRevivalException] with the given [fieldDescription] and
  /// [expectedValueDescription] is thrown.
  static List<String>? _castStringListOrNull(
    dynamic value,
    String fieldDescription,
    String expectedValueDescription, {
    int minLength = 0,
  }) {
    if (value == null) {
      return null;
    }

    return _castStringList(value, fieldDescription, expectedValueDescription, minLength: minLength);
  }

  /// Tries to construct a RegExp from [value], or returns null if [value]
  /// is null.
  ///
  /// If the value is not null and not a valid string-ified regex,
  /// a [CustomDeviceRevivalException] with the given [fieldDescription] and
  /// [expectedValueDescription] is thrown.
  static RegExp? _convertToRegexOrNull(
    dynamic value,
    String fieldDescription,
    String expectedValueDescription,
  ) {
    if (value == null) {
      return null;
    }

    return _maybeRethrowAsRevivalException(
      () => RegExp(value as String),
      fieldDescription,
      expectedValueDescription,
    );
  }

  Object toJson() {
    return <String, Object?>{
      _kId: id,
      _kLabel: label,
      _kSdkNameAndVersion: sdkNameAndVersion,
      _kPlatform: platform == null ? null : getNameForTargetPlatform(platform!),
      _kEnabled: enabled,
      _kPingCommand: pingCommand,
      _kPingSuccessRegex: pingSuccessRegex?.pattern,
      _kPostBuildCommand: (postBuildCommand?.length ?? 0) > 0 ? postBuildCommand : null,
      _kInstallCommand: installCommand,
      _kUninstallCommand: uninstallCommand,
      _kRunDebugCommand: runDebugCommand,
      _kForwardPortCommand: forwardPortCommand,
      _kForwardPortSuccessRegex: forwardPortSuccessRegex?.pattern,
      _kScreenshotCommand: screenshotCommand,
    };
  }

  CustomDeviceConfig copyWith({
    String? id,
    String? label,
    String? sdkNameAndVersion,
    bool explicitPlatform = false,
    TargetPlatform? platform,
    bool? enabled,
    List<String>? pingCommand,
    bool explicitPingSuccessRegex = false,
    RegExp? pingSuccessRegex,
    bool explicitPostBuildCommand = false,
    List<String>? postBuildCommand,
    List<String>? installCommand,
    List<String>? uninstallCommand,
    List<String>? runDebugCommand,
    bool explicitForwardPortCommand = false,
    List<String>? forwardPortCommand,
    bool explicitForwardPortSuccessRegex = false,
    RegExp? forwardPortSuccessRegex,
    bool explicitScreenshotCommand = false,
    List<String>? screenshotCommand,
  }) {
    return CustomDeviceConfig(
      id: id ?? this.id,
      label: label ?? this.label,
      sdkNameAndVersion: sdkNameAndVersion ?? this.sdkNameAndVersion,
      platform: explicitPlatform ? platform : (platform ?? this.platform),
      enabled: enabled ?? this.enabled,
      pingCommand: pingCommand ?? this.pingCommand,
      pingSuccessRegex:
          explicitPingSuccessRegex ? pingSuccessRegex : (pingSuccessRegex ?? this.pingSuccessRegex),
      postBuildCommand:
          explicitPostBuildCommand ? postBuildCommand : (postBuildCommand ?? this.postBuildCommand),
      installCommand: installCommand ?? this.installCommand,
      uninstallCommand: uninstallCommand ?? this.uninstallCommand,
      runDebugCommand: runDebugCommand ?? this.runDebugCommand,
      forwardPortCommand:
          explicitForwardPortCommand
              ? forwardPortCommand
              : (forwardPortCommand ?? this.forwardPortCommand),
      forwardPortSuccessRegex:
          explicitForwardPortSuccessRegex
              ? forwardPortSuccessRegex
              : (forwardPortSuccessRegex ?? this.forwardPortSuccessRegex),
      screenshotCommand:
          explicitScreenshotCommand
              ? screenshotCommand
              : (screenshotCommand ?? this.screenshotCommand),
    );
  }

  @override
  bool operator ==(Object other) {
    return other is CustomDeviceConfig &&
        other.id == id &&
        other.label == label &&
        other.sdkNameAndVersion == sdkNameAndVersion &&
        other.platform == platform &&
        other.enabled == enabled &&
        _listsEqual(other.pingCommand, pingCommand) &&
        _regexesEqual(other.pingSuccessRegex, pingSuccessRegex) &&
        _listsEqual(other.postBuildCommand, postBuildCommand) &&
        _listsEqual(other.installCommand, installCommand) &&
        _listsEqual(other.uninstallCommand, uninstallCommand) &&
        _listsEqual(other.runDebugCommand, runDebugCommand) &&
        _listsEqual(other.forwardPortCommand, forwardPortCommand) &&
        _regexesEqual(other.forwardPortSuccessRegex, forwardPortSuccessRegex) &&
        _listsEqual(other.screenshotCommand, screenshotCommand);
  }

  @override
  int get hashCode {
    return id.hashCode ^
        label.hashCode ^
        sdkNameAndVersion.hashCode ^
        platform.hashCode ^
        enabled.hashCode ^
        pingCommand.hashCode ^
        (pingSuccessRegex?.pattern).hashCode ^
        postBuildCommand.hashCode ^
        installCommand.hashCode ^
        uninstallCommand.hashCode ^
        runDebugCommand.hashCode ^
        forwardPortCommand.hashCode ^
        (forwardPortSuccessRegex?.pattern).hashCode ^
        screenshotCommand.hashCode;
  }

  @override
  String toString() {
    return 'CustomDeviceConfig('
        'id: $id, '
        'label: $label, '
        'sdkNameAndVersion: $sdkNameAndVersion, '
        'platform: $platform, '
        'enabled: $enabled, '
        'pingCommand: $pingCommand, '
        'pingSuccessRegex: $pingSuccessRegex, '
        'postBuildCommand: $postBuildCommand, '
        'installCommand: $installCommand, '
        'uninstallCommand: $uninstallCommand, '
        'runDebugCommand: $runDebugCommand, '
        'forwardPortCommand: $forwardPortCommand, '
        'forwardPortSuccessRegex: $forwardPortSuccessRegex, '
        'screenshotCommand: $screenshotCommand)';
  }
}
