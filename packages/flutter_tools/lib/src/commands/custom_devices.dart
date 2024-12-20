// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:async/async.dart';
import 'package:meta/meta.dart';
import 'package:process/process.dart';

import '../base/common.dart';
import '../base/error_handling_io.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../base/platform.dart';
import '../base/terminal.dart';
import '../convert.dart';
import '../custom_devices/custom_device.dart';
import '../custom_devices/custom_device_config.dart';
import '../custom_devices/custom_devices_config.dart';
import '../device_port_forwarder.dart';
import '../features.dart';
import '../runner/flutter_command.dart';
import '../runner/flutter_command_runner.dart';

/// just the function signature of the [print] function.
/// The Object arg may be null.
typedef PrintFn = void Function(Object);

class CustomDevicesCommand extends FlutterCommand {
  factory CustomDevicesCommand({
    required CustomDevicesConfig customDevicesConfig,
    required OperatingSystemUtils operatingSystemUtils,
    required Terminal terminal,
    required Platform platform,
    required ProcessManager processManager,
    required FileSystem fileSystem,
    required Logger logger,
    required FeatureFlags featureFlags,
  }) {
    return CustomDevicesCommand._common(
      customDevicesConfig: customDevicesConfig,
      operatingSystemUtils: operatingSystemUtils,
      terminal: terminal,
      platform: platform,
      processManager: processManager,
      fileSystem: fileSystem,
      logger: logger,
      featureFlags: featureFlags,
    );
  }

  @visibleForTesting
  factory CustomDevicesCommand.test({
    required CustomDevicesConfig customDevicesConfig,
    required OperatingSystemUtils operatingSystemUtils,
    required Terminal terminal,
    required Platform platform,
    required ProcessManager processManager,
    required FileSystem fileSystem,
    required Logger logger,
    required FeatureFlags featureFlags,
    PrintFn usagePrintFn = print,
  }) {
    return CustomDevicesCommand._common(
      customDevicesConfig: customDevicesConfig,
      operatingSystemUtils: operatingSystemUtils,
      terminal: terminal,
      platform: platform,
      processManager: processManager,
      fileSystem: fileSystem,
      logger: logger,
      featureFlags: featureFlags,
      usagePrintFn: usagePrintFn,
    );
  }

  CustomDevicesCommand._common({
    required CustomDevicesConfig customDevicesConfig,
    required OperatingSystemUtils operatingSystemUtils,
    required Terminal terminal,
    required Platform platform,
    required ProcessManager processManager,
    required FileSystem fileSystem,
    required Logger logger,
    required FeatureFlags featureFlags,
    PrintFn usagePrintFn = print,
  }) : _customDevicesConfig = customDevicesConfig,
       _featureFlags = featureFlags,
       _usagePrintFn = usagePrintFn {
    addSubcommand(
      CustomDevicesListCommand(
        customDevicesConfig: customDevicesConfig,
        featureFlags: featureFlags,
        logger: logger,
      ),
    );
    addSubcommand(
      CustomDevicesResetCommand(
        customDevicesConfig: customDevicesConfig,
        featureFlags: featureFlags,
        fileSystem: fileSystem,
        logger: logger,
      ),
    );
    addSubcommand(
      CustomDevicesAddCommand(
        customDevicesConfig: customDevicesConfig,
        operatingSystemUtils: operatingSystemUtils,
        terminal: terminal,
        platform: platform,
        featureFlags: featureFlags,
        processManager: processManager,
        fileSystem: fileSystem,
        logger: logger,
      ),
    );
    addSubcommand(
      CustomDevicesDeleteCommand(
        customDevicesConfig: customDevicesConfig,
        featureFlags: featureFlags,
        fileSystem: fileSystem,
        logger: logger,
      ),
    );
  }

  final CustomDevicesConfig _customDevicesConfig;
  final FeatureFlags _featureFlags;
  final PrintFn _usagePrintFn;

  @override
  String get description {
    String configFileLine;
    if (_featureFlags.areCustomDevicesEnabled) {
      configFileLine =
          '\nMakes changes to the config file at "${_customDevicesConfig.configPath}".\n';
    } else {
      configFileLine = '';
    }

    return '''
List, reset, add and delete custom devices.
$configFileLine
This is just a collection of commonly used shorthands for things like adding
ssh devices, resetting (with backup) and checking the config file. For advanced
configuration or more complete documentation, edit the config file with an
editor that supports JSON schemas like VS Code.

Requires the custom devices feature to be enabled. You can enable it using "flutter config --enable-custom-devices".
''';
  }

  @override
  String get name => 'custom-devices';

  @override
  String get category => FlutterCommandCategory.tools;

  @override
  Future<FlutterCommandResult> runCommand() async {
    return FlutterCommandResult.success();
  }

  @override
  void printUsage() {
    _usagePrintFn(usage);
  }
}

/// This class is meant to provide some commonly used utility functions
/// to the subcommands, like backing up the config file & checking if the
/// feature is enabled.
abstract class CustomDevicesCommandBase extends FlutterCommand {
  CustomDevicesCommandBase({
    required this.customDevicesConfig,
    required this.featureFlags,
    required this.fileSystem,
    required this.logger,
  });

  @protected
  final CustomDevicesConfig customDevicesConfig;
  @protected
  final FeatureFlags featureFlags;
  @protected
  final FileSystem? fileSystem;
  @protected
  final Logger logger;

  /// The path to the (potentially non-existing) backup of the config file.
  @protected
  String get configBackupPath => '${customDevicesConfig.configPath}.bak';

  /// Copies the current config file to [configBackupPath], overwriting it
  /// if necessary. Returns false and does nothing if the current config file
  /// doesn't exist. (True otherwise)
  @protected
  bool backup() {
    final File configFile = fileSystem!.file(customDevicesConfig.configPath);
    if (configFile.existsSync()) {
      configFile.copySync(configBackupPath);
      return true;
    }
    return false;
  }

  /// Checks if the custom devices feature is enabled and returns true/false
  /// accordingly. Additionally, logs an error if it's not enabled with a hint
  /// on how to enable it.
  @protected
  void checkFeatureEnabled() {
    if (!featureFlags.areCustomDevicesEnabled) {
      throwToolExit(
        'Custom devices feature must be enabled. '
        'Enable using `flutter config --enable-custom-devices`.',
      );
    }
  }
}

class CustomDevicesListCommand extends CustomDevicesCommandBase {
  CustomDevicesListCommand({
    required super.customDevicesConfig,
    required super.featureFlags,
    required super.logger,
  }) : super(fileSystem: null);

  @override
  String get description => '''
List the currently configured custom devices, both enabled and disabled, reachable or not.
''';

  @override
  String get name => 'list';

  @override
  Future<FlutterCommandResult> runCommand() async {
    checkFeatureEnabled();

    late List<CustomDeviceConfig> devices;
    try {
      devices = customDevicesConfig.devices;
    } on Exception {
      throwToolExit('Could not list custom devices.');
    }

    if (devices.isEmpty) {
      logger.printStatus('No custom devices found in "${customDevicesConfig.configPath}"');
    } else {
      logger.printStatus('List of custom devices in "${customDevicesConfig.configPath}":');
      for (final CustomDeviceConfig device in devices) {
        logger.printStatus(
          'id: ${device.id}, label: ${device.label}, enabled: ${device.enabled}',
          indent: 2,
          hangingIndent: 2,
        );
      }
    }

    return FlutterCommandResult.success();
  }
}

class CustomDevicesResetCommand extends CustomDevicesCommandBase {
  CustomDevicesResetCommand({
    required super.customDevicesConfig,
    required super.featureFlags,
    required FileSystem super.fileSystem,
    required super.logger,
  });

  @override
  String get description => '''
Reset the config file to the default.

The current config file will be backed up to the same path, but with a `.bak` appended.
If a file already exists at the backup location, it will be overwritten.
''';

  @override
  String get name => 'reset';

  @override
  Future<FlutterCommandResult> runCommand() async {
    checkFeatureEnabled();

    final bool wasBackedUp = backup();

    ErrorHandlingFileSystem.deleteIfExists(fileSystem!.file(customDevicesConfig.configPath));
    customDevicesConfig.ensureFileExists();

    logger.printStatus(
      wasBackedUp
          ? 'Successfully reset the custom devices config file and created a '
              'backup at "$configBackupPath".'
          : 'Successfully reset the custom devices config file.',
    );
    return FlutterCommandResult.success();
  }
}

class CustomDevicesAddCommand extends CustomDevicesCommandBase {
  CustomDevicesAddCommand({
    required super.customDevicesConfig,
    required OperatingSystemUtils operatingSystemUtils,
    required Terminal terminal,
    required Platform platform,
    required super.featureFlags,
    required ProcessManager processManager,
    required FileSystem super.fileSystem,
    required super.logger,
  }) : _operatingSystemUtils = operatingSystemUtils,
       _terminal = terminal,
       _platform = platform,
       _processManager = processManager {
    argParser.addFlag(
      _kCheck,
      help:
          'Make sure the config actually works. This will execute some of the '
          'commands in the config (if necessary with dummy arguments). This '
          'flag is enabled by default when "--json" is not specified. If '
          '"--json" is given, it is disabled by default.\n'
          'For example, a config with "null" as the "runDebug" command is '
          'invalid. If the "runDebug" command is valid (so it is an array of '
          'strings) but the command is not found (because you have a typo, for '
          'example), the config won\'t work and "--check" will spot that.',
    );

    argParser.addOption(
      _kJson,
      help:
          'Add the custom device described by this JSON-encoded string to the '
          'list of custom-devices instead of using the normal, interactive way '
          'of configuring. Useful if you want to use the "flutter custom-devices '
          'add" command from a script, or use it non-interactively for some '
          'other reason.\n'
          "By default, this won't check whether the passed in config actually "
          'works. For more info see the "--check" option.',
      valueHelp: '{"id": "pi", ...}',
      aliases: _kJsonAliases,
    );

    argParser.addFlag(
      _kSsh,
      help:
          'Add a ssh-device. This will automatically fill out some of the config '
          'options for you with good defaults, and in other cases save you some '
          "typing. So you'll only need to enter some things like hostname and "
          'username of the remote device instead of entering each individual '
          'command.',
      defaultsTo: true,
      negatable: false,
    );
  }

  static const String _kJson = 'json';
  static const List<String> _kJsonAliases = <String>['js'];
  static const String _kCheck = 'check';
  static const String _kSsh = 'ssh';

  // A hostname consists of one or more "names", separated by a dot.
  // A name may consist of alpha-numeric characters. Hyphens are also allowed,
  // but not as the first or last character of the name.
  static final RegExp _hostnameRegex = RegExp(
    r'^(([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\-]*[a-zA-Z0-9])\.)*([A-Za-z0-9]|[A-Za-z0-9][A-Za-z0-9\-]*[A-Za-z0-9])$',
  );

  final OperatingSystemUtils _operatingSystemUtils;
  final Terminal _terminal;
  final Platform _platform;
  final ProcessManager _processManager;
  late StreamQueue<String> inputs;

  @override
  String get description => 'Add a new device the custom devices config file.';

  @override
  String get name => 'add';

  void _printConfigCheckingError(String err) {
    logger.printError(err);
  }

  /// Check this config by executing some of the commands, see if they run
  /// fine.
  Future<bool> _checkConfigWithLogging(final CustomDeviceConfig config) async {
    final CustomDevice device = CustomDevice(
      config: config,
      logger: logger,
      processManager: _processManager,
    );

    bool result = true;

    try {
      final bool reachable = await device.tryPing();
      if (!reachable) {
        _printConfigCheckingError("Couldn't ping device.");
        result = false;
      }
    } on Exception catch (e) {
      _printConfigCheckingError('While executing ping command: $e');
      result = false;
    }

    final Directory temp = await fileSystem!.systemTempDirectory.createTemp();

    try {
      final bool ok = await device.tryInstall(localPath: temp.path, appName: temp.basename);
      if (!ok) {
        _printConfigCheckingError("Couldn't install test app on device.");
        result = false;
      }
    } on Exception catch (e) {
      _printConfigCheckingError('While executing install command: $e');
      result = false;
    }

    await temp.delete();

    try {
      final bool ok = await device.tryUninstall(appName: temp.basename);
      if (!ok) {
        _printConfigCheckingError("Couldn't uninstall test app from device.");
        result = false;
      }
    } on Exception catch (e) {
      _printConfigCheckingError('While executing uninstall command: $e');
      result = false;
    }

    if (config.usesPortForwarding) {
      final CustomDevicePortForwarder portForwarder = CustomDevicePortForwarder(
        deviceName: device.name,
        forwardPortCommand: config.forwardPortCommand!,
        forwardPortSuccessRegex: config.forwardPortSuccessRegex!,
        processManager: _processManager,
        logger: logger,
      );

      try {
        // find a random port we can forward
        final int port = await _operatingSystemUtils.findFreePort();

        final ForwardedPort? forwardedPort = await portForwarder.tryForward(port, port);
        if (forwardedPort == null) {
          _printConfigCheckingError("Couldn't forward test port $port from device.");
          result = false;
        } else {
          await portForwarder.unforward(forwardedPort);
        }
      } on Exception catch (e) {
        _printConfigCheckingError('While forwarding/unforwarding device port: $e');
        result = false;
      }
    }

    if (result) {
      logger.printStatus('Passed all checks successfully.');
    }

    return result;
  }

  /// Run non-interactively (useful if running from scripts or bots),
  /// add value of the `--json` arg to the config.
  ///
  /// Only check if `--check` is explicitly specified. (Don't check by default)
  Future<FlutterCommandResult> runNonInteractively() async {
    final String jsonStr = stringArg(_kJson)!;
    final bool shouldCheck = boolArg(_kCheck);

    dynamic json;
    try {
      json = jsonDecode(jsonStr);
    } on FormatException catch (e) {
      throwToolExit('Could not decode json: $e');
    }

    late CustomDeviceConfig config;
    try {
      config = CustomDeviceConfig.fromJson(json);
    } on CustomDeviceRevivalException catch (e) {
      throwToolExit('Invalid custom device config: $e');
    }

    if (shouldCheck && !await _checkConfigWithLogging(config)) {
      throwToolExit("Custom device didn't pass all checks.");
    }

    customDevicesConfig.add(config);
    printSuccessfullyAdded();

    return FlutterCommandResult.success();
  }

  void printSuccessfullyAdded() {
    logger.printStatus(
      'Successfully added custom device to config file at "${customDevicesConfig.configPath}".',
    );
  }

  bool _isValidHostname(String s) => _hostnameRegex.hasMatch(s);

  bool _isValidIpAddr(String s) => InternetAddress.tryParse(s) != null;

  /// Ask the user to input a string.
  Future<String?> askForString(
    String name, {
    String? description,
    String? example,
    String? defaultsTo,
    Future<bool> Function(String)? validator,
  }) async {
    String msg = description ?? name;

    final String exampleOrDefault = <String>[
      if (example != null) 'example: $example',
      if (defaultsTo != null) 'empty for $defaultsTo',
    ].join(', ');

    if (exampleOrDefault.isNotEmpty) {
      msg += ' ($exampleOrDefault)';
    }

    logger.printStatus(msg);
    while (true) {
      if (!await inputs.hasNext) {
        return null;
      }

      final String input = await inputs.next;

      if (validator != null && !await validator(input)) {
        logger.printStatus('Invalid input. Please enter $name:');
      } else {
        return input;
      }
    }
  }

  /// Ask the user for a y(es) / n(o) or empty input.
  Future<bool> askForBool(String name, {String? description, bool defaultsTo = true}) async {
    final String defaultsToStr = defaultsTo ? '[Y/n]' : '[y/N]';
    logger.printStatus('$description $defaultsToStr (empty for default)');
    while (true) {
      final String input = await inputs.next;

      if (input.isEmpty) {
        return defaultsTo;
      } else if (input.toLowerCase() == 'y') {
        return true;
      } else if (input.toLowerCase() == 'n') {
        return false;
      } else {
        logger.printStatus(
          'Invalid input. Expected is either y, n or empty for default. $name? $defaultsToStr',
        );
      }
    }
  }

  /// Ask the user if he wants to apply the config.
  /// Shows a different prompt if errors or warnings exist in the config.
  Future<bool> askApplyConfig({bool hasErrorsOrWarnings = false}) {
    return askForBool(
      'apply',
      description:
          hasErrorsOrWarnings
              ? 'Warnings or errors exist in custom device. '
                  'Would you like to add the custom device to the config anyway?'
              : 'Would you like to add the custom device to the config now?',
      defaultsTo: !hasErrorsOrWarnings,
    );
  }

  /// Run interactively (with user prompts), the target device should be
  /// connected to via ssh.
  Future<FlutterCommandResult> runInteractivelySsh() async {
    final bool shouldCheck = boolArg(_kCheck);

    // Listen to the keystrokes stream as late as possible, since it's a
    // single-subscription stream apparently.
    // Also, _terminal.keystrokes can be closed unexpectedly, which will result
    // in StreamQueue.next throwing a StateError when make the StreamQueue listen
    // to that directly.
    // This caused errors when using Ctrl+C to terminate while the
    // custom-devices add command is waiting for user input.
    // So instead, we add the keystrokes stream events to a new single-subscription
    // stream and listen to that instead.
    final StreamController<String> nonClosingKeystrokes = StreamController<String>();
    final StreamSubscription<String> keystrokesSubscription = _terminal.keystrokes.listen(
      (String s) => nonClosingKeystrokes.add(s.trim()),
      cancelOnError: true,
    );

    inputs = StreamQueue<String>(nonClosingKeystrokes.stream);

    final String id =
        (await askForString(
          'id',
          description:
              'Please enter the id you want to device to have. Must contain only '
              'alphanumeric or underscore characters.',
          example: 'pi',
          validator: (String s) async => RegExp(r'^\w+$').hasMatch(s),
        ))!;

    final String label =
        (await askForString(
          'label',
          description:
              'Please enter the label of the device, which is a slightly more verbose '
              'name for the device.',
          example: 'Raspberry Pi',
        ))!;

    final String sdkNameAndVersion =
        (await askForString('SDK name and version', example: 'Raspberry Pi 4 Model B+'))!;

    final bool enabled = await askForBool('enabled', description: 'Should the device be enabled?');

    final String targetStr =
        (await askForString(
          'target',
          description: 'Please enter the hostname or IPv4/v6 address of the device.',
          example: 'raspberrypi',
          validator: (String s) async => _isValidHostname(s) || _isValidIpAddr(s),
        ))!;

    final InternetAddress? targetIp = InternetAddress.tryParse(targetStr);
    final bool useIp = targetIp != null;
    final bool ipv6 = useIp && targetIp.type == InternetAddressType.IPv6;
    final InternetAddress loopbackIp =
        ipv6 ? InternetAddress.loopbackIPv6 : InternetAddress.loopbackIPv4;

    final String username =
        (await askForString(
          'username',
          description: 'Please enter the username used for ssh-ing into the remote device.',
          example: 'pi',
          defaultsTo: 'no username',
        ))!;

    final String remoteRunDebugCommand =
        (await askForString(
          'run command',
          description:
              'Please enter the command executed on the remote device for starting '
              r'the app. "/tmp/${appName}" is the path to the asset bundle.',
          example: r'flutter-pi /tmp/${appName}',
        ))!;

    final bool usePortForwarding = await askForBool(
      'use port forwarding',
      description:
          'Should the device use port forwarding? '
          'Using port forwarding is the default because it works in all cases, however if your '
          'remote device has a static IP address and you have a way of '
          'specifying the "--vm-service-host=<ip>" engine option, you might prefer '
          'not using port forwarding.',
    );

    final String screenshotCommand =
        (await askForString(
          'screenshot command',
          description: 'Enter the command executed on the remote device for taking a screenshot.',
          example:
              r"fbgrab /tmp/screenshot.png && cat /tmp/screenshot.png | base64 | tr -d ' \n\t'",
          defaultsTo: 'no screenshotting support',
        ))!;

    // SSH expects IPv6 addresses to use the bracket syntax like URIs do too,
    // but the IPv6 the user enters is a raw IPv6 address, so we need to wrap it.
    final String sshTarget =
        (username.isNotEmpty ? '$username@' : '') + (ipv6 ? '[${targetIp.address}]' : targetStr);

    final String formattedLoopbackIp = ipv6 ? '[${loopbackIp.address}]' : loopbackIp.address;

    CustomDeviceConfig config = CustomDeviceConfig(
      id: id,
      label: label,
      sdkNameAndVersion: sdkNameAndVersion,
      enabled: enabled,

      // host-platform specific, filled out later
      pingCommand: const <String>[],

      postBuildCommand: const <String>[],

      // just install to /tmp/${appName} by default
      installCommand: <String>[
        'scp',
        '-r',
        '-o',
        'BatchMode=yes',
        if (ipv6) '-6',
        r'${localPath}',
        '$sshTarget:/tmp/\${appName}',
      ],

      uninstallCommand: <String>[
        'ssh',
        '-o',
        'BatchMode=yes',
        if (ipv6) '-6',
        sshTarget,
        r'rm -rf "/tmp/${appName}"',
      ],

      runDebugCommand: <String>[
        'ssh',
        '-o',
        'BatchMode=yes',
        if (ipv6) '-6',
        sshTarget,
        remoteRunDebugCommand,
      ],

      forwardPortCommand:
          usePortForwarding
              ? <String>[
                'ssh',
                '-o',
                'BatchMode=yes',
                '-o',
                'ExitOnForwardFailure=yes',
                if (ipv6) '-6',
                '-L',
                '$formattedLoopbackIp:\${hostPort}:$formattedLoopbackIp:\${devicePort}',
                sshTarget,
                "echo 'Port forwarding success'; read",
              ]
              : null,
      forwardPortSuccessRegex: usePortForwarding ? RegExp('Port forwarding success') : null,

      screenshotCommand:
          screenshotCommand.isNotEmpty
              ? <String>['ssh', '-o', 'BatchMode=yes', if (ipv6) '-6', sshTarget, screenshotCommand]
              : null,
    );

    if (_platform.isWindows) {
      config = config.copyWith(
        pingCommand: <String>['ping', if (ipv6) '-6', '-n', '1', '-w', '500', targetStr],
        explicitPingSuccessRegex: true,
        pingSuccessRegex: RegExp(r'[<=]\d+ms'),
      );
    } else if (_platform.isLinux || _platform.isMacOS) {
      config = config.copyWith(
        pingCommand: <String>['ping', if (ipv6) '-6', '-c', '1', '-w', '1', targetStr],
        explicitPingSuccessRegex: true,
      );
    } else {
      throw UnsupportedError('Unsupported operating system');
    }

    final bool apply = await askApplyConfig(
      hasErrorsOrWarnings: shouldCheck && !(await _checkConfigWithLogging(config)),
    );

    unawaited(keystrokesSubscription.cancel());
    unawaited(nonClosingKeystrokes.close());

    if (apply) {
      customDevicesConfig.add(config);
      printSuccessfullyAdded();
    }

    return FlutterCommandResult.success();
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    checkFeatureEnabled();

    if (stringArg(_kJson) != null) {
      return runNonInteractively();
    }
    if (boolArg(_kSsh)) {
      return runInteractivelySsh();
    }
    throw UnsupportedError('Unknown run mode');
  }
}

class CustomDevicesDeleteCommand extends CustomDevicesCommandBase {
  CustomDevicesDeleteCommand({
    required super.customDevicesConfig,
    required super.featureFlags,
    required FileSystem super.fileSystem,
    required super.logger,
  });

  @override
  String get description => '''
Delete a device from the config file.
''';

  @override
  String get name => 'delete';

  @override
  Future<FlutterCommandResult> runCommand() async {
    checkFeatureEnabled();

    final String? id = globalResults![FlutterGlobalOptions.kDeviceIdOption] as String?;
    if (id == null || !customDevicesConfig.contains(id)) {
      throwToolExit(
        'Couldn\'t find device with id "$id" in config at "${customDevicesConfig.configPath}"',
      );
    }

    backup();
    customDevicesConfig.remove(id);
    logger.printStatus(
      'Successfully removed device with id "$id" from config at "${customDevicesConfig.configPath}"',
    );
    return FlutterCommandResult.success();
  }
}
