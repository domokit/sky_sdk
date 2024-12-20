// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import '../base/common.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/terminal.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../ios/devices.dart';

const String _checkingForWirelessDevicesMessage = 'Checking for wireless devices...';
const String _chooseOneMessage = 'Please choose one (or "q" to quit)';
const String _connectedDevicesMessage = 'Connected devices:';
const String _foundButUnsupportedDevicesMessage =
    'The following devices were found, but are not supported by this project:';
const String _noAttachedCheckForWirelessMessage =
    'No devices found yet. Checking for wireless devices...';
const String _noDevicesFoundMessage = 'No devices found.';
const String _noWirelessDevicesFoundMessage = 'No wireless devices were found.';
const String _wirelesslyConnectedDevicesMessage = 'Wirelessly connected devices:';

String _chooseDeviceOptionMessage(int option, String name, String deviceId) =>
    '[$option]: $name ($deviceId)';
String _foundMultipleSpecifiedDevicesMessage(String deviceId) =>
    'Found multiple devices with name or id matching $deviceId:';
String _foundSpecifiedDevicesMessage(int count, String deviceId) =>
    'Found $count devices with name or id matching $deviceId:';
String _noMatchingDeviceMessage(String deviceId) =>
    'No supported devices found with name or id '
    "matching '$deviceId'.";
String flutterSpecifiedDeviceDevModeDisabled(String deviceName) =>
    'To use '
    "'$deviceName' for development, enable Developer Mode in Settings → Privacy & Security on the device. "
    'If this does not work, open Xcode, reconnect the device, and look for a '
    'popup on the device asking you to trust this computer.';
String flutterSpecifiedDeviceUnpaired(String deviceName) =>
    "'$deviceName' is not paired. "
    'Open Xcode and trust this computer when prompted.';

/// This class handles functionality of finding and selecting target devices.
///
/// Target devices are devices that are supported and selectable to run
/// a flutter application on.
class TargetDevices {
  factory TargetDevices({
    required Platform platform,
    required DeviceManager deviceManager,
    required Logger logger,
    DeviceConnectionInterface? deviceConnectionInterface,
  }) {
    if (platform.isMacOS) {
      return TargetDevicesWithExtendedWirelessDeviceDiscovery(
        deviceManager: deviceManager,
        logger: logger,
        deviceConnectionInterface: deviceConnectionInterface,
      );
    }
    return TargetDevices._private(
      deviceManager: deviceManager,
      logger: logger,
      deviceConnectionInterface: deviceConnectionInterface,
    );
  }

  TargetDevices._private({
    required DeviceManager deviceManager,
    required Logger logger,
    required this.deviceConnectionInterface,
  }) : _deviceManager = deviceManager,
       _logger = logger;

  final DeviceManager _deviceManager;
  final Logger _logger;
  final DeviceConnectionInterface? deviceConnectionInterface;

  bool get _includeAttachedDevices =>
      deviceConnectionInterface == null ||
      deviceConnectionInterface == DeviceConnectionInterface.attached;
  bool get _includeWirelessDevices =>
      deviceConnectionInterface == null ||
      deviceConnectionInterface == DeviceConnectionInterface.wireless;

  Future<List<Device>> _getAttachedDevices({DeviceDiscoverySupportFilter? supportFilter}) async {
    if (!_includeAttachedDevices) {
      return <Device>[];
    }
    return _deviceManager.getDevices(
      filter: DeviceDiscoveryFilter(
        deviceConnectionInterface: DeviceConnectionInterface.attached,
        supportFilter: supportFilter,
      ),
    );
  }

  Future<List<Device>> _getWirelessDevices({DeviceDiscoverySupportFilter? supportFilter}) async {
    if (!_includeWirelessDevices) {
      return <Device>[];
    }
    return _deviceManager.getDevices(
      filter: DeviceDiscoveryFilter(
        deviceConnectionInterface: DeviceConnectionInterface.wireless,
        supportFilter: supportFilter,
      ),
    );
  }

  Future<List<Device>> _getDeviceById({
    bool includeDevicesUnsupportedByProject = false,
    bool includeDisconnected = false,
  }) async {
    return _deviceManager.getDevices(
      filter: DeviceDiscoveryFilter(
        excludeDisconnected: !includeDisconnected,
        supportFilter: _deviceManager.deviceSupportFilter(
          includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
        ),
        deviceConnectionInterface: deviceConnectionInterface,
      ),
    );
  }

  DeviceDiscoverySupportFilter _defaultSupportFilter(bool includeDevicesUnsupportedByProject) {
    return _deviceManager.deviceSupportFilter(
      includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
    );
  }

  void startExtendedWirelessDeviceDiscovery({Duration? deviceDiscoveryTimeout}) {}

  /// Find and return all target [Device]s based upon criteria entered by the
  /// user on the command line.
  ///
  /// When the user has specified `all` devices, return all devices meeting criteria.
  ///
  /// When the user has specified a device id/name, attempt to find an exact or
  /// partial match. If an exact match or a single partial match is found,
  /// return it immediately.
  ///
  /// When multiple devices are found and there is a terminal attached to
  /// stdin, allow the user to select which device to use. When a terminal
  /// with stdin is not available, print a list of available devices and
  /// return null.
  ///
  /// When no devices meet user specifications, print a list of unsupported
  /// devices and return null.
  Future<List<Device>?> findAllTargetDevices({
    Duration? deviceDiscoveryTimeout,
    bool includeDevicesUnsupportedByProject = false,
  }) async {
    if (!globals.doctor!.canLaunchAnything) {
      _logger.printError(globals.userMessages.flutterNoDevelopmentDevice);
      return null;
    }

    if (deviceDiscoveryTimeout != null) {
      // Reset the cache with the specified timeout.
      await _deviceManager.refreshAllDevices(timeout: deviceDiscoveryTimeout);
    }

    if (_deviceManager.hasSpecifiedDeviceId) {
      // Must check for device match separately from `_getAttachedDevices` and
      // `_getWirelessDevices` because if an exact match is found in one
      // and a partial match is found in another, there is no way to distinguish
      // between them.
      final List<Device> devices = await _getDeviceById(
        includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
      );
      if (devices.length == 1) {
        return devices;
      }
    }

    final List<Device> attachedDevices = await _getAttachedDevices(
      supportFilter: _defaultSupportFilter(includeDevicesUnsupportedByProject),
    );
    final List<Device> wirelessDevices = await _getWirelessDevices(
      supportFilter: _defaultSupportFilter(includeDevicesUnsupportedByProject),
    );
    final List<Device> allDevices = attachedDevices + wirelessDevices;

    if (allDevices.isEmpty) {
      return _handleNoDevices();
    } else if (_deviceManager.hasSpecifiedAllDevices) {
      return allDevices;
    } else if (allDevices.length > 1) {
      return _handleMultipleDevices(attachedDevices, wirelessDevices);
    }
    return allDevices;
  }

  /// When no supported devices are found, display a message and list of
  /// unsupported devices found.
  Future<List<Device>?> _handleNoDevices() async {
    // Get connected devices from cache, including unsupported ones.
    final List<Device> unsupportedDevices = await _deviceManager.getAllDevices(
      filter: DeviceDiscoveryFilter(deviceConnectionInterface: deviceConnectionInterface),
    );

    if (_deviceManager.hasSpecifiedDeviceId) {
      _logger.printStatus(_noMatchingDeviceMessage(_deviceManager.specifiedDeviceId!));
      if (unsupportedDevices.isNotEmpty) {
        _logger.printStatus('');
        _logger.printStatus('The following devices were found:');
        await Device.printDevices(unsupportedDevices, _logger);
      }
      return null;
    }

    _logger.printStatus(
      _deviceManager.hasSpecifiedAllDevices
          ? _noDevicesFoundMessage
          : globals.userMessages.flutterNoSupportedDevices,
    );
    await _printUnsupportedDevice(unsupportedDevices);
    return null;
  }

  /// Determine which device to use when multiple found.
  ///
  /// If user has not specified a device id/name, attempt to prioritize
  /// ephemeral devices. If a single ephemeral device is found, return it
  /// immediately.
  ///
  /// Otherwise, prompt the user to select a device if there is a terminal
  /// with stdin. If there is not a terminal, display the list of devices with
  /// instructions to use a device selection flag.
  Future<List<Device>?> _handleMultipleDevices(
    List<Device> attachedDevices,
    List<Device> wirelessDevices,
  ) async {
    final List<Device> allDevices = attachedDevices + wirelessDevices;

    final Device? ephemeralDevice = _deviceManager.getSingleEphemeralDevice(allDevices);
    if (ephemeralDevice != null) {
      return <Device>[ephemeralDevice];
    }

    if (globals.terminal.stdinHasTerminal) {
      return _selectFromMultipleDevices(attachedDevices, wirelessDevices);
    } else {
      return _printMultipleDevices(attachedDevices, wirelessDevices);
    }
  }

  /// Display a list of found devices. When the user has not specified the
  /// device id/name, display devices unsupported by the project as well and
  /// give instructions to use a device selection flag.
  Future<List<Device>?> _printMultipleDevices(
    List<Device> attachedDevices,
    List<Device> wirelessDevices,
  ) async {
    List<Device> supportedAttachedDevices = attachedDevices;
    List<Device> supportedWirelessDevices = wirelessDevices;
    if (_deviceManager.hasSpecifiedDeviceId) {
      final int allDeviceLength = supportedAttachedDevices.length + supportedWirelessDevices.length;
      _logger.printStatus(
        _foundSpecifiedDevicesMessage(allDeviceLength, _deviceManager.specifiedDeviceId!),
      );
    } else {
      // Get connected devices from cache, including ones unsupported for the
      // project but still supported by Flutter.
      supportedAttachedDevices = await _getAttachedDevices(
        supportFilter: DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutter(),
      );
      supportedWirelessDevices = await _getWirelessDevices(
        supportFilter: DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutter(),
      );

      _logger.printStatus(globals.userMessages.flutterSpecifyDeviceWithAllOption);
      _logger.printStatus('');
    }

    await Device.printDevices(supportedAttachedDevices, _logger);

    if (supportedWirelessDevices.isNotEmpty) {
      if (_deviceManager.hasSpecifiedDeviceId || supportedAttachedDevices.isNotEmpty) {
        _logger.printStatus('');
      }
      _logger.printStatus(_wirelesslyConnectedDevicesMessage);
      await Device.printDevices(supportedWirelessDevices, _logger);
    }

    return null;
  }

  /// Display a list of selectable devices, prompt the user to choose one, and
  /// wait for the user to select a valid option.
  Future<List<Device>?> _selectFromMultipleDevices(
    List<Device> attachedDevices,
    List<Device> wirelessDevices,
  ) async {
    final List<Device> allDevices = attachedDevices + wirelessDevices;

    if (_deviceManager.hasSpecifiedDeviceId) {
      _logger.printStatus(
        _foundSpecifiedDevicesMessage(allDevices.length, _deviceManager.specifiedDeviceId!),
      );
    } else {
      _logger.printStatus(_connectedDevicesMessage);
    }

    await Device.printDevices(attachedDevices, _logger);

    if (wirelessDevices.isNotEmpty) {
      _logger.printStatus('');
      _logger.printStatus(_wirelesslyConnectedDevicesMessage);
      await Device.printDevices(wirelessDevices, _logger);
      _logger.printStatus('');
    }

    final Device chosenDevice = await _chooseOneOfAvailableDevices(allDevices);

    // Update the [DeviceManager.specifiedDeviceId] so that the user will not
    // be prompted again.
    _deviceManager.specifiedDeviceId = chosenDevice.id;

    return <Device>[chosenDevice];
  }

  Future<void> _printUnsupportedDevice(List<Device> unsupportedDevices) async {
    if (unsupportedDevices.isNotEmpty) {
      final StringBuffer result = StringBuffer();
      result.writeln();
      result.writeln(_foundButUnsupportedDevicesMessage);
      result.writeAll(
        (await Device.descriptions(unsupportedDevices)).map((String desc) => desc).toList(),
        '\n',
      );
      result.writeln();
      result.writeln(
        globals.userMessages.flutterMissPlatformProjects(
          Device.devicesPlatformTypes(unsupportedDevices),
        ),
      );
      _logger.printStatus(result.toString(), newline: false);
    }
  }

  Future<Device> _chooseOneOfAvailableDevices(List<Device> devices) async {
    _displayDeviceOptions(devices);
    final String userInput = await _readUserInput(devices.length);
    if (userInput.toLowerCase() == 'q') {
      throwToolExit('');
    }
    return devices[int.parse(userInput) - 1];
  }

  void _displayDeviceOptions(List<Device> devices) {
    int count = 1;
    for (final Device device in devices) {
      _logger.printStatus(_chooseDeviceOptionMessage(count, device.name, device.id));
      count++;
    }
  }

  Future<String> _readUserInput(int deviceCount) async {
    globals.terminal.usesTerminalUi = true;
    final String result = await globals.terminal.promptForCharInput(
      <String>[for (int i = 0; i < deviceCount; i++) '${i + 1}', 'q', 'Q'],
      displayAcceptedCharacters: false,
      logger: _logger,
      prompt: _chooseOneMessage,
    );
    return result;
  }
}

@visibleForTesting
class TargetDevicesWithExtendedWirelessDeviceDiscovery extends TargetDevices {
  TargetDevicesWithExtendedWirelessDeviceDiscovery({
    required super.deviceManager,
    required super.logger,
    super.deviceConnectionInterface,
  }) : super._private();

  Future<void>? _wirelessDevicesRefresh;

  @visibleForTesting
  bool waitForWirelessBeforeInput = false;

  @visibleForTesting
  late final TargetDeviceSelection deviceSelection = TargetDeviceSelection(_logger);

  @override
  void startExtendedWirelessDeviceDiscovery({Duration? deviceDiscoveryTimeout}) {
    if (deviceDiscoveryTimeout == null && _includeWirelessDevices) {
      _wirelessDevicesRefresh ??= _deviceManager.refreshExtendedWirelessDeviceDiscoverers(
        timeout: DeviceManager.minimumWirelessDeviceDiscoveryTimeout,
      );
    }
    return;
  }

  Future<List<Device>> _getRefreshedWirelessDevices({
    bool includeDevicesUnsupportedByProject = false,
  }) async {
    if (!_includeWirelessDevices) {
      return <Device>[];
    }
    startExtendedWirelessDeviceDiscovery();
    return () async {
      await _wirelessDevicesRefresh;
      return _deviceManager.getDevices(
        filter: DeviceDiscoveryFilter(
          deviceConnectionInterface: DeviceConnectionInterface.wireless,
          supportFilter: _defaultSupportFilter(includeDevicesUnsupportedByProject),
        ),
      );
    }();
  }

  Future<Device?> _waitForIOSDeviceToConnect(IOSDevice device) async {
    for (final DeviceDiscovery discoverer in _deviceManager.deviceDiscoverers) {
      if (discoverer is IOSDevices) {
        _logger.printStatus('Waiting for ${device.name} to connect...');
        final Status waitingStatus = _logger.startSpinner(
          timeout: const Duration(seconds: 30),
          warningColor: TerminalColor.red,
          slowWarningCallback: () {
            return 'The device was unable to connect after 30 seconds. Ensure the device is paired and unlocked.';
          },
        );
        final Device? connectedDevice = await discoverer.waitForDeviceToConnect(device, _logger);
        waitingStatus.stop();
        return connectedDevice;
      }
    }
    return null;
  }

  /// Find and return all target [Device]s based upon criteria entered by the
  /// user on the command line.
  ///
  /// When the user has specified `all` devices, return all devices meeting criteria.
  ///
  /// When the user has specified a device id/name, attempt to find an exact or
  /// partial match. If an exact match or a single partial match is found and
  /// the device is connected, return it immediately. If an exact match or a
  /// single partial match is found and the device is not connected and it's
  /// an iOS device, wait for it to connect.
  ///
  /// When multiple devices are found and there is a terminal attached to
  /// stdin, allow the user to select which device to use. When a terminal
  /// with stdin is not available, print a list of available devices and
  /// return null.
  ///
  /// When no devices meet user specifications, print a list of unsupported
  /// devices and return null.
  @override
  Future<List<Device>?> findAllTargetDevices({
    Duration? deviceDiscoveryTimeout,
    bool includeDevicesUnsupportedByProject = false,
  }) async {
    if (!globals.doctor!.canLaunchAnything) {
      _logger.printError(globals.userMessages.flutterNoDevelopmentDevice);
      return null;
    }

    // When a user defines the timeout or filters to only attached devices,
    // use the super function that does not do longer wireless device
    // discovery and does not wait for devices to connect.
    if (deviceDiscoveryTimeout != null ||
        deviceConnectionInterface == DeviceConnectionInterface.attached) {
      return super.findAllTargetDevices(
        deviceDiscoveryTimeout: deviceDiscoveryTimeout,
        includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
      );
    }

    // Start polling for wireless devices that need longer to load if it hasn't
    // already been started.
    startExtendedWirelessDeviceDiscovery();

    if (_deviceManager.hasSpecifiedDeviceId) {
      // Get devices matching the specified device regardless of whether they
      // are currently connected or not.
      // If there is a single matching connected device, return it immediately.
      // If the only device found is an iOS device that is not connected yet,
      // wait for it to connect.
      // If there are multiple matches, continue on to wait for all attached
      // and wireless devices to load so the user can select between all
      // connected matches.
      final List<Device> specifiedDevices = await _getDeviceById(
        includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
        includeDisconnected: true,
      );

      if (specifiedDevices.length == 1) {
        Device? matchedDevice = specifiedDevices.first;
        if (matchedDevice is IOSDevice) {
          // If the only matching device is not paired, print a warning
          if (!matchedDevice.isPaired) {
            _logger.printStatus(flutterSpecifiedDeviceUnpaired(matchedDevice.name));
            return null;
          }
          // If the only matching device does not have Developer Mode enabled,
          // print a warning
          if (!matchedDevice.devModeEnabled) {
            _logger.printStatus(flutterSpecifiedDeviceDevModeDisabled(matchedDevice.name));
            return null;
          }

          if (!matchedDevice.isConnected) {
            matchedDevice = await _waitForIOSDeviceToConnect(matchedDevice);
          }
        }

        if (matchedDevice != null && matchedDevice.isConnected) {
          return <Device>[matchedDevice];
        }
      } else {
        for (final IOSDevice device in specifiedDevices.whereType<IOSDevice>()) {
          // Print warning for every matching unpaired device.
          if (!device.isPaired) {
            _logger.printStatus(flutterSpecifiedDeviceUnpaired(device.name));
          }

          // Print warning for every matching device that does not have Developer Mode enabled.
          if (!device.devModeEnabled) {
            _logger.printStatus(flutterSpecifiedDeviceDevModeDisabled(device.name));
          }
        }
      }
    }

    final List<Device> attachedDevices = await _getAttachedDevices(
      supportFilter: _defaultSupportFilter(includeDevicesUnsupportedByProject),
    );

    // _getRefreshedWirelessDevices must be run after _getAttachedDevices is
    // finished to prevent non-iOS discoverers from running simultaneously.
    // `AndroidDevices` may error if run simultaneously.
    final Future<List<Device>> futureWirelessDevices = _getRefreshedWirelessDevices(
      includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
    );

    if (attachedDevices.isEmpty) {
      return _handleNoAttachedDevices(attachedDevices, futureWirelessDevices);
    } else if (_deviceManager.hasSpecifiedAllDevices) {
      return _handleAllDevices(attachedDevices, futureWirelessDevices);
    }
    // Even if there's only a single attached device, continue to
    // `_handleRemainingDevices` since there might be wireless devices
    // that are not loaded yet.
    return _handleRemainingDevices(attachedDevices, futureWirelessDevices);
  }

  /// When no supported attached devices are found, wait for wireless devices
  /// to load.
  ///
  /// If no wireless devices are found, continue to `_handleNoDevices`.
  ///
  /// If wireless devices are found, continue to `_handleMultipleDevices`.
  Future<List<Device>?> _handleNoAttachedDevices(
    List<Device> attachedDevices,
    Future<List<Device>> futureWirelessDevices,
  ) async {
    if (_includeAttachedDevices) {
      _logger.printStatus(_noAttachedCheckForWirelessMessage);
    } else {
      _logger.printStatus(_checkingForWirelessDevicesMessage);
    }

    final List<Device> wirelessDevices = await futureWirelessDevices;
    final List<Device> allDevices = attachedDevices + wirelessDevices;

    if (allDevices.isEmpty) {
      _logger.printStatus('');
      return _handleNoDevices();
    } else if (_deviceManager.hasSpecifiedAllDevices) {
      return allDevices;
    } else if (allDevices.length > 1) {
      _logger.printStatus('');
      return _handleMultipleDevices(attachedDevices, wirelessDevices);
    }
    return allDevices;
  }

  /// Wait for wireless devices to load and then return all attached and
  /// wireless devices.
  Future<List<Device>?> _handleAllDevices(
    List<Device> devices,
    Future<List<Device>> futureWirelessDevices,
  ) async {
    _logger.printStatus(_checkingForWirelessDevicesMessage);
    final List<Device> wirelessDevices = await futureWirelessDevices;
    return devices + wirelessDevices;
  }

  /// Determine which device to use when one or more are found.
  ///
  /// If user has not specified a device id/name, attempt to prioritize
  /// ephemeral devices. If a single ephemeral device is found, return it
  /// immediately.
  ///
  /// Otherwise, prompt the user to select a device if there is a terminal
  /// with stdin. If there is not a terminal, display the list of devices with
  /// instructions to use a device selection flag.
  Future<List<Device>?> _handleRemainingDevices(
    List<Device> attachedDevices,
    Future<List<Device>> futureWirelessDevices,
  ) async {
    final Device? ephemeralDevice = _deviceManager.getSingleEphemeralDevice(attachedDevices);
    if (ephemeralDevice != null) {
      return <Device>[ephemeralDevice];
    }

    if (!globals.terminal.stdinHasTerminal || !_logger.supportsColor) {
      _logger.printStatus(_checkingForWirelessDevicesMessage);
      final List<Device> wirelessDevices = await futureWirelessDevices;
      if (attachedDevices.length + wirelessDevices.length == 1) {
        return attachedDevices + wirelessDevices;
      }
      _logger.printStatus('');
      // If the terminal has stdin but does not support color/ANSI (which is
      // needed to clear lines), fallback to standard selection of device.
      if (globals.terminal.stdinHasTerminal && !_logger.supportsColor) {
        return _handleMultipleDevices(attachedDevices, wirelessDevices);
      }
      // If terminal does not have stdin, print out device list.
      return _printMultipleDevices(attachedDevices, wirelessDevices);
    }

    return _selectFromDevicesAndCheckForWireless(attachedDevices, futureWirelessDevices);
  }

  /// Display a list of selectable attached devices and prompt the user to
  /// choose one.
  ///
  /// Also, display a message about waiting for wireless devices to load. Once
  /// wireless devices have loaded, update waiting message, device list, and
  /// selection options.
  ///
  /// Wait for the user to select a device.
  Future<List<Device>?> _selectFromDevicesAndCheckForWireless(
    List<Device> attachedDevices,
    Future<List<Device>> futureWirelessDevices,
  ) async {
    if (attachedDevices.length == 1 || !_deviceManager.hasSpecifiedDeviceId) {
      _logger.printStatus(_connectedDevicesMessage);
    } else if (_deviceManager.hasSpecifiedDeviceId) {
      // Multiple devices were found with part of the name/id provided.
      _logger.printStatus(_foundMultipleSpecifiedDevicesMessage(_deviceManager.specifiedDeviceId!));
    }

    // Display list of attached devices.
    await Device.printDevices(attachedDevices, _logger);

    // Display waiting message.
    _logger.printStatus('');
    _logger.printStatus(_checkingForWirelessDevicesMessage);
    _logger.printStatus('');

    // Start user device selection so user can select device while waiting
    // for wireless devices to load if they want.
    _displayDeviceOptions(attachedDevices);
    deviceSelection.devices = attachedDevices;
    final Future<Device> futureChosenDevice = deviceSelection.userSelectDevice();
    Device? chosenDevice;

    // Once wireless devices are found, we clear out the waiting message (3),
    // device option list (attachedDevices.length), and device option prompt (1).
    int numLinesToClear = attachedDevices.length + 4;

    futureWirelessDevices = futureWirelessDevices.then((List<Device> wirelessDevices) async {
      // If device is already chosen, don't update terminal with
      // wireless device list.
      if (chosenDevice != null) {
        return wirelessDevices;
      }

      final List<Device> allDevices = attachedDevices + wirelessDevices;

      if (_logger.isVerbose) {
        await _verbosePrintWirelessDevices(attachedDevices, wirelessDevices);
      } else {
        // Also clear any invalid device selections.
        numLinesToClear += deviceSelection.invalidAttempts;
        await _printWirelessDevices(wirelessDevices, numLinesToClear);
      }
      _logger.printStatus('');

      // Reprint device option list.
      _displayDeviceOptions(allDevices);
      deviceSelection.devices = allDevices;
      // Reprint device option prompt.
      _logger.printStatus('$_chooseOneMessage: ', emphasis: true, newline: false);
      return wirelessDevices;
    });

    // Used for testing.
    if (waitForWirelessBeforeInput) {
      await futureWirelessDevices;
    }

    // Wait for user to select a device.
    chosenDevice = await futureChosenDevice;

    // Update the [DeviceManager.specifiedDeviceId] so that the user will not
    // be prompted again.
    _deviceManager.specifiedDeviceId = chosenDevice.id;

    return <Device>[chosenDevice];
  }

  /// Reprint list of attached devices before printing list of wireless devices.
  Future<void> _verbosePrintWirelessDevices(
    List<Device> attachedDevices,
    List<Device> wirelessDevices,
  ) async {
    if (wirelessDevices.isEmpty) {
      _logger.printStatus(_noWirelessDevicesFoundMessage);
    }
    // The iOS xcdevice outputs once wireless devices are done loading, so
    // reprint attached devices so they're grouped with the wireless ones.
    _logger.printStatus(_connectedDevicesMessage);
    await Device.printDevices(attachedDevices, _logger);

    if (wirelessDevices.isNotEmpty) {
      _logger.printStatus('');
      _logger.printStatus(_wirelesslyConnectedDevicesMessage);
      await Device.printDevices(wirelessDevices, _logger);
    }
  }

  /// Clear [numLinesToClear] lines from terminal. Print message and list of
  /// wireless devices.
  Future<void> _printWirelessDevices(List<Device> wirelessDevices, int numLinesToClear) async {
    _logger.printStatus(globals.terminal.clearLines(numLinesToClear), newline: false);
    _logger.printStatus('');
    if (wirelessDevices.isEmpty) {
      _logger.printStatus(_noWirelessDevicesFoundMessage);
    } else {
      _logger.printStatus(_wirelesslyConnectedDevicesMessage);
      await Device.printDevices(wirelessDevices, _logger);
    }
  }
}

@visibleForTesting
class TargetDeviceSelection {
  TargetDeviceSelection(this._logger);

  List<Device> devices = <Device>[];
  final Logger _logger;
  int invalidAttempts = 0;

  /// Prompt user to select a device and wait until they select a valid device.
  ///
  /// If the user selects `q`, exit the tool.
  ///
  /// If the user selects an invalid number, reprompt them and continue waiting.
  Future<Device> userSelectDevice() async {
    Device? chosenDevice;
    while (chosenDevice == null) {
      final String userInputString = await readUserInput();
      if (userInputString.toLowerCase() == 'q') {
        throwToolExit('');
      }
      final int deviceIndex = int.parse(userInputString) - 1;
      if (deviceIndex > -1 && deviceIndex < devices.length) {
        chosenDevice = devices[deviceIndex];
      }
    }

    return chosenDevice;
  }

  /// Prompt user to select a device and wait until they select a valid
  /// character.
  ///
  /// Only allow input of a number or `q`.
  @visibleForTesting
  Future<String> readUserInput() async {
    final RegExp pattern = RegExp(r'\d+$|q', caseSensitive: false);
    String? choice;
    globals.terminal.singleCharMode = true;
    while (choice == null || choice.length > 1 || !pattern.hasMatch(choice)) {
      _logger.printStatus(_chooseOneMessage, emphasis: true, newline: false);
      // prompt ends with ': '
      _logger.printStatus(': ', emphasis: true, newline: false);
      choice = (await globals.terminal.keystrokes.first).trim();
      _logger.printStatus(choice);
      invalidAttempts++;
    }
    globals.terminal.singleCharMode = false;
    return choice;
  }
}
