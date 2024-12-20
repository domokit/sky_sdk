// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../base/common.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../convert.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../runner/flutter_command.dart';

class DevicesCommand extends FlutterCommand {
  DevicesCommand({bool verboseHelp = false}) {
    argParser.addFlag(
      'machine',
      negatable: false,
      help: 'Output device information in machine readable structured JSON format.',
    );
    argParser.addOption(
      'timeout',
      abbr: 't',
      help: '(deprecated) This option has been replaced by "--${FlutterOptions.kDeviceTimeout}".',
      hide: !verboseHelp,
    );
    usesDeviceTimeoutOption();
    usesDeviceConnectionOption();
  }

  @override
  final String name = 'devices';

  @override
  final String description = 'List all connected devices.';

  @override
  final String category = FlutterCommandCategory.tools;

  @override
  Duration? get deviceDiscoveryTimeout {
    if (argResults?['timeout'] != null) {
      final int? timeoutSeconds = int.tryParse(stringArg('timeout')!);
      if (timeoutSeconds == null) {
        throwToolExit('Could not parse -t/--timeout argument. It must be an integer.');
      }
      return Duration(seconds: timeoutSeconds);
    }
    return super.deviceDiscoveryTimeout;
  }

  @override
  Future<void> validateCommand() {
    if (argResults?['timeout'] != null) {
      globals.printWarning(
        '${globals.logger.terminal.warningMark} The "--timeout" argument is deprecated; use "--${FlutterOptions.kDeviceTimeout}" instead.',
      );
    }
    return super.validateCommand();
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    if (globals.doctor?.canListAnything != true) {
      throwToolExit(
        "Unable to locate a development device; please run 'flutter doctor' for "
        'information about installing additional components.',
        exitCode: 1,
      );
    }

    final DevicesCommandOutput output = DevicesCommandOutput(
      platform: globals.platform,
      logger: globals.logger,
      deviceManager: globals.deviceManager,
      deviceDiscoveryTimeout: deviceDiscoveryTimeout,
      deviceConnectionInterface: deviceConnectionInterface,
    );

    await output.findAndOutputAllTargetDevices(machine: boolArg('machine'));

    return FlutterCommandResult.success();
  }
}

class DevicesCommandOutput {
  factory DevicesCommandOutput({
    required Platform platform,
    required Logger logger,
    DeviceManager? deviceManager,
    Duration? deviceDiscoveryTimeout,
    DeviceConnectionInterface? deviceConnectionInterface,
  }) {
    if (platform.isMacOS) {
      return DevicesCommandOutputWithExtendedWirelessDeviceDiscovery(
        logger: logger,
        deviceManager: deviceManager,
        deviceDiscoveryTimeout: deviceDiscoveryTimeout,
        deviceConnectionInterface: deviceConnectionInterface,
      );
    }
    return DevicesCommandOutput._private(
      logger: logger,
      deviceManager: deviceManager,
      deviceDiscoveryTimeout: deviceDiscoveryTimeout,
      deviceConnectionInterface: deviceConnectionInterface,
    );
  }

  DevicesCommandOutput._private({
    required Logger logger,
    required DeviceManager? deviceManager,
    required this.deviceDiscoveryTimeout,
    required this.deviceConnectionInterface,
  }) : _deviceManager = deviceManager,
       _logger = logger;

  final DeviceManager? _deviceManager;
  final Logger _logger;
  final Duration? deviceDiscoveryTimeout;
  final DeviceConnectionInterface? deviceConnectionInterface;

  bool get _includeAttachedDevices =>
      deviceConnectionInterface == null ||
      deviceConnectionInterface == DeviceConnectionInterface.attached;

  bool get _includeWirelessDevices =>
      deviceConnectionInterface == null ||
      deviceConnectionInterface == DeviceConnectionInterface.wireless;

  Future<List<Device>> _getAttachedDevices(DeviceManager deviceManager) async {
    if (!_includeAttachedDevices) {
      return <Device>[];
    }
    return deviceManager.getAllDevices(
      filter: DeviceDiscoveryFilter(deviceConnectionInterface: DeviceConnectionInterface.attached),
    );
  }

  Future<List<Device>> _getWirelessDevices(DeviceManager deviceManager) async {
    if (!_includeWirelessDevices) {
      return <Device>[];
    }
    return deviceManager.getAllDevices(
      filter: DeviceDiscoveryFilter(deviceConnectionInterface: DeviceConnectionInterface.wireless),
    );
  }

  Future<void> findAndOutputAllTargetDevices({required bool machine}) async {
    List<Device> attachedDevices = <Device>[];
    List<Device> wirelessDevices = <Device>[];
    final DeviceManager? deviceManager = _deviceManager;
    if (deviceManager != null) {
      // Refresh the cache and then get the attached and wireless devices from
      // the cache.
      await deviceManager.refreshAllDevices(timeout: deviceDiscoveryTimeout);
      attachedDevices = await _getAttachedDevices(deviceManager);
      wirelessDevices = await _getWirelessDevices(deviceManager);
    }
    final List<Device> allDevices = attachedDevices + wirelessDevices;

    if (machine) {
      await printDevicesAsJson(allDevices);
      return;
    }

    if (allDevices.isEmpty) {
      _logger.printStatus('No authorized devices detected.');
    } else {
      if (attachedDevices.isNotEmpty) {
        _logger.printStatus(
          'Found ${attachedDevices.length} connected ${pluralize('device', attachedDevices.length)}:',
        );
        await Device.printDevices(attachedDevices, _logger, prefix: '  ');
      }
      if (wirelessDevices.isNotEmpty) {
        if (attachedDevices.isNotEmpty) {
          _logger.printStatus('');
        }
        _logger.printStatus(
          'Found ${wirelessDevices.length} wirelessly connected ${pluralize('device', wirelessDevices.length)}:',
        );
        await Device.printDevices(wirelessDevices, _logger, prefix: '  ');
      }
    }
    await _printDiagnostics(foundAny: allDevices.isNotEmpty);
  }

  Future<void> _printDiagnostics({required bool foundAny}) async {
    final StringBuffer status = StringBuffer();
    status.writeln();
    final List<String> diagnostics = await _deviceManager?.getDeviceDiagnostics() ?? <String>[];
    if (diagnostics.isNotEmpty) {
      for (final String diagnostic in diagnostics) {
        status.writeln(diagnostic);
        status.writeln();
      }
    }
    status.writeln('Run "flutter emulators" to list and start any available device emulators.');
    status.writeln();
    status.write(
      'If you expected ${foundAny ? 'another' : 'a'} device to be detected, please run "flutter doctor" to diagnose potential issues. ',
    );
    if (deviceDiscoveryTimeout == null) {
      status.write(
        'You may also try increasing the time to wait for connected devices with the "--${FlutterOptions.kDeviceTimeout}" flag. ',
      );
    }
    status.write('Visit https://flutter.dev/setup/ for troubleshooting tips.');
    _logger.printStatus(status.toString());
  }

  Future<void> printDevicesAsJson(List<Device> devices) async {
    _logger.printStatus(
      const JsonEncoder.withIndent(
        '  ',
      ).convert(await Future.wait(devices.map((Device d) => d.toJson()))),
    );
  }
}

const String _checkingForWirelessDevicesMessage = 'Checking for wireless devices...';
const String _noAttachedCheckForWireless = 'No devices found yet. Checking for wireless devices...';
const String _noWirelessDevicesFoundMessage = 'No wireless devices were found.';

class DevicesCommandOutputWithExtendedWirelessDeviceDiscovery extends DevicesCommandOutput {
  DevicesCommandOutputWithExtendedWirelessDeviceDiscovery({
    required super.logger,
    super.deviceManager,
    super.deviceDiscoveryTimeout,
    super.deviceConnectionInterface,
  }) : super._private();

  @override
  Future<void> findAndOutputAllTargetDevices({required bool machine}) async {
    // When a user defines the timeout or filters to only attached devices,
    // use the super function that does not do longer wireless device discovery.
    if (deviceDiscoveryTimeout != null ||
        deviceConnectionInterface == DeviceConnectionInterface.attached) {
      return super.findAndOutputAllTargetDevices(machine: machine);
    }

    if (machine) {
      final List<Device> devices =
          await _deviceManager?.refreshAllDevices(
            filter: DeviceDiscoveryFilter(deviceConnectionInterface: deviceConnectionInterface),
            timeout: DeviceManager.minimumWirelessDeviceDiscoveryTimeout,
          ) ??
          <Device>[];
      await printDevicesAsJson(devices);
      return;
    }

    final Future<void>? extendedWirelessDiscovery = _deviceManager
        ?.refreshExtendedWirelessDeviceDiscoverers(
          timeout: DeviceManager.minimumWirelessDeviceDiscoveryTimeout,
        );

    List<Device> attachedDevices = <Device>[];
    final DeviceManager? deviceManager = _deviceManager;
    if (deviceManager != null) {
      attachedDevices = await _getAttachedDevices(deviceManager);
    }

    // Number of lines to clear starts at 1 because it's inclusive of the line
    // the cursor is on, which will be blank for this use case.
    int numLinesToClear = 1;

    // Display list of attached devices.
    if (attachedDevices.isNotEmpty) {
      _logger.printStatus(
        'Found ${attachedDevices.length} connected ${pluralize('device', attachedDevices.length)}:',
      );
      await Device.printDevices(attachedDevices, _logger, prefix: '  ');
      _logger.printStatus('');
      numLinesToClear += 1;
    }

    // Display waiting message.
    if (attachedDevices.isEmpty && _includeAttachedDevices) {
      _logger.printStatus(_noAttachedCheckForWireless);
    } else {
      _logger.printStatus(_checkingForWirelessDevicesMessage);
    }
    numLinesToClear += 1;

    final Status waitingStatus = _logger.startSpinner();
    await extendedWirelessDiscovery;
    List<Device> wirelessDevices = <Device>[];
    if (deviceManager != null) {
      wirelessDevices = await _getWirelessDevices(deviceManager);
    }
    waitingStatus.stop();

    final Terminal terminal = _logger.terminal;
    if (_logger.isVerbose && _includeAttachedDevices) {
      // Reprint the attach devices.
      if (attachedDevices.isNotEmpty) {
        _logger.printStatus(
          '\nFound ${attachedDevices.length} connected ${pluralize('device', attachedDevices.length)}:',
        );
        await Device.printDevices(attachedDevices, _logger, prefix: '  ');
      }
    } else if (terminal.supportsColor && terminal is AnsiTerminal) {
      _logger.printStatus(terminal.clearLines(numLinesToClear), newline: false);
    }

    if (attachedDevices.isNotEmpty || !_logger.terminal.supportsColor) {
      _logger.printStatus('');
    }

    if (wirelessDevices.isEmpty) {
      if (attachedDevices.isEmpty) {
        // No wireless or attached devices were found.
        _logger.printStatus('No authorized devices detected.');
      } else {
        // Attached devices found, wireless devices not found.
        _logger.printStatus(_noWirelessDevicesFoundMessage);
      }
    } else {
      // Display list of wireless devices.
      _logger.printStatus(
        'Found ${wirelessDevices.length} wirelessly connected ${pluralize('device', wirelessDevices.length)}:',
      );
      await Device.printDevices(wirelessDevices, _logger, prefix: '  ');
    }
    await _printDiagnostics(foundAny: wirelessDevices.isNotEmpty || attachedDevices.isNotEmpty);
  }
}
