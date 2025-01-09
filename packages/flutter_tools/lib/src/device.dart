// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math' as math;

import 'package:meta/meta.dart';

import 'application_package.dart';
import 'base/context.dart';
import 'base/dds.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/utils.dart';
import 'build_info.dart';
import 'devfs.dart';
import 'device_port_forwarder.dart';
import 'device_vm_service_discovery_for_attach.dart';
import 'project.dart';
import 'vmservice.dart';
import 'web/compile.dart';

DeviceManager? get deviceManager => context.get<DeviceManager>();

/// A description of the kind of workflow the device supports.
enum Category {
  web._('web'),
  desktop._('desktop'),
  mobile._('mobile');

  const Category._(this.value);

  final String value;

  @override
  String toString() => value;

  static Category? fromString(String category) {
    return const <String, Category>{'web': web, 'desktop': desktop, 'mobile': mobile}[category];
  }
}

/// The platform sub-folder that a device type supports.
enum PlatformType {
  web,
  android,
  ios,
  linux,
  macos,
  windows,
  fuchsia,
  custom,
  windowsPreview;

  @override
  String toString() => name;

  static PlatformType? fromString(String platformType) => values.asNameMap()[platformType];
}

/// A discovery mechanism for flutter-supported development devices.
abstract class DeviceManager {
  DeviceManager({required Logger logger}) : _logger = logger;

  final Logger _logger;

  /// Constructing DeviceManagers is cheap; they only do expensive work if some
  /// of their methods are called.
  List<DeviceDiscovery> get deviceDiscoverers;

  String? _specifiedDeviceId;

  /// A user-specified device ID.
  String? get specifiedDeviceId {
    if (_specifiedDeviceId == null || _specifiedDeviceId == 'all') {
      return null;
    }
    return _specifiedDeviceId;
  }

  set specifiedDeviceId(String? id) {
    _specifiedDeviceId = id;
  }

  /// A minimum duration to use when discovering wireless iOS devices.
  static const Duration minimumWirelessDeviceDiscoveryTimeout = Duration(seconds: 5);

  /// True when the user has specified a single specific device.
  bool get hasSpecifiedDeviceId => specifiedDeviceId != null;

  /// True when the user has specified all devices by setting
  /// specifiedDeviceId = 'all'.
  bool get hasSpecifiedAllDevices => _specifiedDeviceId == 'all';

  /// Get devices filtered by [filter] that match the given device id/name.
  ///
  /// If [filter] is not provided, a default filter that requires devices to be
  /// connected will be used.
  ///
  /// If an exact match is found, return it immediately. Otherwise wait for all
  /// discoverers to complete and return any partial matches.
  Future<List<Device>> getDevicesById(String deviceId, {DeviceDiscoveryFilter? filter}) async {
    filter ??= DeviceDiscoveryFilter();

    final String lowerDeviceId = deviceId.toLowerCase();
    bool exactlyMatchesDeviceId(Device device) =>
        device.id.toLowerCase() == lowerDeviceId || device.name.toLowerCase() == lowerDeviceId;
    bool startsWithDeviceId(Device device) =>
        device.id.toLowerCase().startsWith(lowerDeviceId) ||
        device.name.toLowerCase().startsWith(lowerDeviceId);

    // Some discoverers have hard-coded device IDs and return quickly, and others
    // shell out to other processes and can take longer.
    // If an ID was specified, first check if it was a "well-known" device id.
    final Set<String> wellKnownIds =
        _platformDiscoverers.expand((DeviceDiscovery discovery) => discovery.wellKnownIds).toSet();
    final bool hasWellKnownId = hasSpecifiedDeviceId && wellKnownIds.contains(specifiedDeviceId);

    // Process discoverers as they can return results, so if an exact match is
    // found quickly, we don't wait for all the discoverers to complete.
    final List<Device> prefixMatches = <Device>[];
    final Completer<Device> exactMatchCompleter = Completer<Device>();
    final List<Future<List<Device>?>> futureDevices = <Future<List<Device>?>>[
      for (final DeviceDiscovery discoverer in _platformDiscoverers)
        if (!hasWellKnownId || discoverer.wellKnownIds.contains(specifiedDeviceId))
          discoverer
              .devices(filter: filter)
              .then(
                (List<Device> devices) {
                  for (final Device device in devices) {
                    if (exactlyMatchesDeviceId(device)) {
                      exactMatchCompleter.complete(device);
                      return null;
                    }
                    if (startsWithDeviceId(device)) {
                      prefixMatches.add(device);
                    }
                  }
                  return null;
                },
                onError: (dynamic error, StackTrace stackTrace) {
                  // Return matches from other discoverers even if one fails.
                  _logger.printTrace('Ignored error discovering $deviceId: $error');
                },
              ),
    ];

    // Wait for an exact match, or for all discoverers to return results.
    await Future.any<Object>(<Future<Object>>[
      exactMatchCompleter.future,
      Future.wait<List<Device>?>(futureDevices),
    ]);

    if (exactMatchCompleter.isCompleted) {
      return <Device>[await exactMatchCompleter.future];
    }
    return prefixMatches;
  }

  /// Returns a list of devices filtered by the user-specified device
  /// id/name (if applicable) and [filter].
  ///
  /// If [filter] is not provided, a default filter that requires devices to be
  /// connected will be used.
  Future<List<Device>> getDevices({DeviceDiscoveryFilter? filter}) {
    filter ??= DeviceDiscoveryFilter();
    final String? id = specifiedDeviceId;
    if (id == null) {
      return getAllDevices(filter: filter);
    }
    return getDevicesById(id, filter: filter);
  }

  Iterable<DeviceDiscovery> get _platformDiscoverers {
    return deviceDiscoverers.where((DeviceDiscovery discoverer) => discoverer.supportsPlatform);
  }

  /// Returns a list of devices filtered by [filter].
  ///
  /// If [filter] is not provided, a default filter that requires devices to be
  /// connected will be used.
  Future<List<Device>> getAllDevices({DeviceDiscoveryFilter? filter}) async {
    filter ??= DeviceDiscoveryFilter();
    final List<List<Device>> devices = await Future.wait<List<Device>>(<Future<List<Device>>>[
      for (final DeviceDiscovery discoverer in _platformDiscoverers)
        discoverer.devices(filter: filter),
    ]);

    return devices.expand<Device>((List<Device> deviceList) => deviceList).toList();
  }

  /// Returns a list of devices filtered by [filter]. Discards existing cache of devices.
  ///
  /// If [filter] is not provided, a default filter that requires devices to be
  /// connected will be used.
  ///
  /// Search for devices to populate the cache for no longer than [timeout].
  Future<List<Device>> refreshAllDevices({Duration? timeout, DeviceDiscoveryFilter? filter}) async {
    filter ??= DeviceDiscoveryFilter();
    final List<List<Device>> devices = await Future.wait<List<Device>>(<Future<List<Device>>>[
      for (final DeviceDiscovery discoverer in _platformDiscoverers)
        discoverer.discoverDevices(filter: filter, timeout: timeout),
    ]);

    return devices.expand<Device>((List<Device> deviceList) => deviceList).toList();
  }

  /// Discard existing cache of discoverers that are known to take longer to
  /// discover wireless devices.
  ///
  /// Then, search for devices for those discoverers to populate the cache for
  /// no longer than [timeout].
  Future<void> refreshExtendedWirelessDeviceDiscoverers({
    Duration? timeout,
    DeviceDiscoveryFilter? filter,
  }) async {
    await Future.wait<List<Device>>(<Future<List<Device>>>[
      for (final DeviceDiscovery discoverer in _platformDiscoverers)
        if (discoverer.requiresExtendedWirelessDeviceDiscovery)
          discoverer.discoverDevices(timeout: timeout),
    ]);
  }

  /// Whether we're capable of listing any devices given the current environment configuration.
  bool get canListAnything {
    return _platformDiscoverers.any((DeviceDiscovery discoverer) => discoverer.canListAnything);
  }

  /// Get diagnostics about issues with any connected devices.
  Future<List<String>> getDeviceDiagnostics() async {
    return <String>[
      for (final DeviceDiscovery discoverer in _platformDiscoverers)
        ...await discoverer.getDiagnostics(),
    ];
  }

  /// Determines how to filter devices.
  ///
  /// By default, filters to only include devices that are supported by Flutter.
  ///
  /// If the user has not specified a device, filters to only include devices
  /// that are supported by Flutter and supported by the project.
  ///
  /// If the user has specified `--device all`, filters to only include devices
  /// that are supported by Flutter, supported by the project, and supported for `all`.
  ///
  /// If [includeDevicesUnsupportedByProject] is true, all devices will be
  /// considered supported by the project, regardless of user specifications.
  ///
  /// This also exists to allow the check to be overridden for google3 clients.
  DeviceDiscoverySupportFilter deviceSupportFilter({
    bool includeDevicesUnsupportedByProject = false,
  }) {
    FlutterProject? flutterProject;
    if (!includeDevicesUnsupportedByProject) {
      flutterProject = FlutterProject.current();
    }
    if (hasSpecifiedAllDevices) {
      return DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutterOrProjectOrAll(
        flutterProject: flutterProject,
      );
    } else if (!hasSpecifiedDeviceId) {
      return DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutterOrProject(
        flutterProject: flutterProject,
      );
    } else {
      return DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutter();
    }
  }

  /// If the user did not specify to run all or a specific device, then attempt
  /// to prioritize ephemeral devices.
  ///
  /// If there is not exactly one ephemeral device return null.
  ///
  /// For example, if the user only typed 'flutter run' and both an Android
  /// device and desktop device are available, choose the Android device.
  ///
  /// Note: ephemeral is nullable for device types where this is not well
  /// defined.
  Device? getSingleEphemeralDevice(List<Device> devices) {
    if (!hasSpecifiedDeviceId) {
      try {
        return devices.singleWhere((Device device) => device.ephemeral);
      } on StateError {
        return null;
      }
    }
    return null;
  }
}

/// A class for determining how to filter devices based on if they are supported.
class DeviceDiscoverySupportFilter {
  /// Filter devices to only include those supported by Flutter.
  DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutter()
    : _excludeDevicesNotSupportedByProject = false,
      _excludeDevicesNotSupportedByAll = false,
      _flutterProject = null;

  /// Filter devices to only include those supported by Flutter and the
  /// provided [flutterProject].
  ///
  /// If [flutterProject] is null, all devices will be considered supported by
  /// the project.
  DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutterOrProject({
    required FlutterProject? flutterProject,
  }) : _flutterProject = flutterProject,
       _excludeDevicesNotSupportedByProject = true,
       _excludeDevicesNotSupportedByAll = false;

  /// Filter devices to only include those supported by Flutter, the provided
  /// [flutterProject], and `--device all`.
  ///
  /// If [flutterProject] is null, all devices will be considered supported by
  /// the project.
  DeviceDiscoverySupportFilter.excludeDevicesUnsupportedByFlutterOrProjectOrAll({
    required FlutterProject? flutterProject,
  }) : _flutterProject = flutterProject,
       _excludeDevicesNotSupportedByProject = true,
       _excludeDevicesNotSupportedByAll = true;

  final FlutterProject? _flutterProject;
  final bool _excludeDevicesNotSupportedByProject;
  final bool _excludeDevicesNotSupportedByAll;

  Future<bool> matchesRequirements(Device device) async {
    final bool meetsSupportByFlutterRequirement = device.isSupported();
    final bool meetsSupportForProjectRequirement =
        !_excludeDevicesNotSupportedByProject || isDeviceSupportedForProject(device);
    final bool meetsSupportForAllRequirement =
        !_excludeDevicesNotSupportedByAll || await isDeviceSupportedForAll(device);

    return meetsSupportByFlutterRequirement &&
        meetsSupportForProjectRequirement &&
        meetsSupportForAllRequirement;
  }

  /// User has specified `--device all`.
  ///
  /// Always remove web and fuchsia devices from `all`. This setting
  /// currently requires devices to share a frontend_server and resident
  /// runner instance. Both web and fuchsia require differently configured
  /// compilers, and web requires an entirely different resident runner.
  Future<bool> isDeviceSupportedForAll(Device device) async {
    final TargetPlatform devicePlatform = await device.targetPlatform;
    return device.isSupported() &&
        devicePlatform != TargetPlatform.fuchsia_arm64 &&
        devicePlatform != TargetPlatform.fuchsia_x64 &&
        devicePlatform != TargetPlatform.web_javascript &&
        isDeviceSupportedForProject(device);
  }

  /// Returns whether the device is supported for the project.
  ///
  /// A device can be supported by Flutter but not supported for the project
  /// (e.g. when the user has removed the iOS directory from their project).
  ///
  /// This also exists to allow the check to be overridden for google3 clients. If
  /// [_flutterProject] is null then return true.
  bool isDeviceSupportedForProject(Device device) {
    if (!device.isSupported()) {
      return false;
    }
    if (_flutterProject == null) {
      return true;
    }
    return device.isSupportedForProject(_flutterProject);
  }
}

/// A class for filtering devices.
///
/// If [excludeDisconnected] is true, only devices detected as connected will be included.
///
/// If [supportFilter] is provided, only devices matching the requirements will be included.
///
/// If [deviceConnectionInterface] is provided, only devices matching the DeviceConnectionInterface will be included.
class DeviceDiscoveryFilter {
  DeviceDiscoveryFilter({
    this.excludeDisconnected = true,
    this.supportFilter,
    this.deviceConnectionInterface,
  });

  final bool excludeDisconnected;
  final DeviceDiscoverySupportFilter? supportFilter;
  final DeviceConnectionInterface? deviceConnectionInterface;

  Future<bool> matchesRequirements(Device device) async {
    final DeviceDiscoverySupportFilter? localSupportFilter = supportFilter;

    final bool meetsConnectionRequirement = !excludeDisconnected || device.isConnected;
    final bool meetsSupportRequirements =
        localSupportFilter == null || (await localSupportFilter.matchesRequirements(device));
    final bool meetsConnectionInterfaceRequirement = matchesDeviceConnectionInterface(
      device,
      deviceConnectionInterface,
    );

    return meetsConnectionRequirement &&
        meetsSupportRequirements &&
        meetsConnectionInterfaceRequirement;
  }

  Future<List<Device>> filterDevices(List<Device> devices) async {
    devices = <Device>[
      for (final Device device in devices)
        if (await matchesRequirements(device)) device,
    ];
    return devices;
  }

  bool matchesDeviceConnectionInterface(
    Device device,
    DeviceConnectionInterface? deviceConnectionInterface,
  ) {
    if (deviceConnectionInterface == null) {
      return true;
    }
    return device.connectionInterface == deviceConnectionInterface;
  }
}

/// An abstract class to discover and enumerate a specific type of devices.
abstract class DeviceDiscovery {
  bool get supportsPlatform;

  /// Whether this device discovery is capable of listing any devices given the
  /// current environment configuration.
  bool get canListAnything;

  /// Whether this device discovery is known to take longer to discover
  /// wireless devices.
  bool get requiresExtendedWirelessDeviceDiscovery => false;

  /// Return all connected devices, cached on subsequent calls.
  Future<List<Device>> devices({DeviceDiscoveryFilter? filter});

  /// Return all connected devices. Discards existing cache of devices.
  Future<List<Device>> discoverDevices({Duration? timeout, DeviceDiscoveryFilter? filter});

  /// Gets a list of diagnostic messages pertaining to issues with any connected
  /// devices (will be an empty list if there are no issues).
  Future<List<String>> getDiagnostics() => Future<List<String>>.value(<String>[]);

  /// Hard-coded device IDs that the discoverer can produce.
  ///
  /// These values are used by the device discovery to determine if it can
  /// short-circuit the other detectors if a specific ID is provided. If a
  /// discoverer has no valid fixed IDs, these should be left empty.
  ///
  /// For example, 'windows' or 'linux'.
  List<String> get wellKnownIds;
}

/// A [DeviceDiscovery] implementation that uses polling to discover device adds
/// and removals.
abstract class PollingDeviceDiscovery extends DeviceDiscovery {
  PollingDeviceDiscovery(this.name);

  static const Duration _pollingInterval = Duration(seconds: 4);
  static const Duration _pollingTimeout = Duration(seconds: 30);

  final String name;

  @protected
  @visibleForTesting
  final ItemListNotifier<Device> deviceNotifier = ItemListNotifier<Device>();

  Timer? _timer;

  Future<List<Device>> pollingGetDevices({Duration? timeout});

  void startPolling() {
    // Make initial population the default, fast polling timeout.
    _timer ??= _initTimer(null, initialCall: true);
  }

  Timer _initTimer(Duration? pollingTimeout, {bool initialCall = false}) {
    // Poll for devices immediately on the initial call for faster initial population.
    return Timer(initialCall ? Duration.zero : _pollingInterval, () async {
      try {
        final List<Device> devices = await pollingGetDevices(timeout: pollingTimeout);
        deviceNotifier.updateWithNewList(devices);
      } on TimeoutException {
        // Do nothing on a timeout.
      }
      // Subsequent timeouts after initial population should wait longer.
      _timer = _initTimer(_pollingTimeout);
    });
  }

  void stopPolling() {
    _timer?.cancel();
    _timer = null;
  }

  /// Get devices from cache filtered by [filter].
  ///
  /// If the cache is empty, populate the cache.
  ///
  /// If [filter] is null, it may return devices that are not connected.
  @override
  Future<List<Device>> devices({DeviceDiscoveryFilter? filter}) {
    return _populateDevices(filter: filter);
  }

  /// Empty the cache and repopulate it before getting devices from cache filtered by [filter].
  ///
  /// Search for devices to populate the cache for no longer than [timeout].
  ///
  /// If [filter] is null, it may return devices that are not connected.
  @override
  Future<List<Device>> discoverDevices({Duration? timeout, DeviceDiscoveryFilter? filter}) {
    return _populateDevices(timeout: timeout, filter: filter, resetCache: true);
  }

  /// Get devices from cache filtered by [filter].
  ///
  /// If the cache is empty or [resetCache] is true, populate the cache.
  ///
  /// Search for devices to populate the cache for no longer than [timeout].
  Future<List<Device>> _populateDevices({
    Duration? timeout,
    DeviceDiscoveryFilter? filter,
    bool resetCache = false,
  }) async {
    if (!deviceNotifier.isPopulated || resetCache) {
      final List<Device> devices = await pollingGetDevices(timeout: timeout);
      // If the cache was populated while the polling was ongoing, do not
      // overwrite the cache unless it's explicitly refreshing the cache.
      if (!deviceNotifier.isPopulated || resetCache) {
        deviceNotifier.updateWithNewList(devices);
      }
    }

    // If a filter is provided, filter cache to only return devices matching.
    if (filter != null) {
      return filter.filterDevices(deviceNotifier.items);
    }
    return deviceNotifier.items;
  }

  Stream<Device> get onAdded {
    return deviceNotifier.onAdded;
  }

  Stream<Device> get onRemoved {
    return deviceNotifier.onRemoved;
  }

  void dispose() => stopPolling();

  @override
  String toString() => '$name device discovery';
}

/// How a device is connected.
enum DeviceConnectionInterface { attached, wireless }

/// Returns the `DeviceConnectionInterface` enum based on its string name.
DeviceConnectionInterface getDeviceConnectionInterfaceForName(String name) {
  return switch (name) {
    'attached' => DeviceConnectionInterface.attached,
    'wireless' => DeviceConnectionInterface.wireless,
    _ => throw Exception('Unsupported DeviceConnectionInterface name "$name"'),
  };
}

/// Returns a `DeviceConnectionInterface`'s string name.
String getNameForDeviceConnectionInterface(DeviceConnectionInterface connectionInterface) {
  return switch (connectionInterface) {
    DeviceConnectionInterface.attached => 'attached',
    DeviceConnectionInterface.wireless => 'wireless',
  };
}

/// A device is a physical hardware that can run a Flutter application.
///
/// This may correspond to a connected iOS or Android device, or represent
/// the host operating system in the case of Flutter Desktop.
abstract class Device {
  Device(
    this.id, {
    required Logger logger,
    required this.category,
    required this.platformType,
    required this.ephemeral,
  }) : dds = DartDevelopmentService(logger: logger);

  final String id;

  /// The [Category] for this device type.
  final Category? category;

  /// The [PlatformType] for this device.
  final PlatformType? platformType;

  /// Whether this is an ephemeral device.
  final bool ephemeral;

  bool get isConnected => true;

  DeviceConnectionInterface get connectionInterface => DeviceConnectionInterface.attached;

  bool get isWirelesslyConnected => connectionInterface == DeviceConnectionInterface.wireless;

  String get name;

  bool get supportsStartPaused => true;

  /// Whether it is an emulated device running on localhost.
  ///
  /// This may return `true` for certain physical Android devices, and is
  /// generally only a best effort guess.
  Future<bool> get isLocalEmulator;

  /// The unique identifier for the emulator that corresponds to this device, or
  /// null if it is not an emulator.
  ///
  /// The ID returned matches that in the output of `flutter emulators`. Fetching
  /// this name may require connecting to the device and if an error occurs null
  /// will be returned.
  Future<String?> get emulatorId;

  /// Whether this device can run the provided [buildMode].
  ///
  /// For example, some emulator architectures cannot run profile or
  /// release builds.
  FutureOr<bool> supportsRuntimeMode(BuildMode buildMode) => true;

  /// Whether the device is a simulator on a platform which supports hardware rendering.
  // This is soft-deprecated since the logic is not correct expect for iOS simulators.
  Future<bool> get supportsHardwareRendering async {
    return true;
  }

  /// Whether the device is supported for the current project directory.
  bool isSupportedForProject(FlutterProject flutterProject);

  /// Check if a version of the given app is already installed.
  ///
  /// Specify [userIdentifier] to check if installed for a particular user (Android only).
  Future<bool> isAppInstalled(ApplicationPackage app, {String? userIdentifier});

  /// Check if the latest build of the [app] is already installed.
  Future<bool> isLatestBuildInstalled(ApplicationPackage app);

  /// Install an app package on the current device.
  ///
  /// Specify [userIdentifier] to install for a particular user (Android only).
  Future<bool> installApp(ApplicationPackage app, {String? userIdentifier});

  /// Uninstall an app package from the current device.
  ///
  /// Specify [userIdentifier] to uninstall for a particular user,
  /// defaults to all users (Android only).
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier});

  /// Check if the device is supported by Flutter.
  bool isSupported();

  // String meant to be displayed to the user indicating if the device is
  // supported by Flutter, and, if not, why.
  String supportMessage() => isSupported() ? 'Supported' : 'Unsupported';

  /// The device's platform.
  Future<TargetPlatform> get targetPlatform;

  /// Platform name for display only.
  Future<String> get targetPlatformDisplayName async =>
      getNameForTargetPlatform(await targetPlatform);

  Future<String> get sdkNameAndVersion;

  /// Create a platform-specific [DevFSWriter] for the given [app], or
  /// null if the device does not support them.
  ///
  /// For example, the desktop device classes can use a writer which
  /// copies the files across the local file system.
  DevFSWriter? createDevFSWriter(ApplicationPackage? app, String? userIdentifier) {
    return null;
  }

  /// Get a log reader for this device.
  ///
  /// If `app` is specified, this will return a log reader specific to that
  /// application. Otherwise, a global log reader will be returned.
  ///
  /// If `includePastLogs` is true and the device type supports it, the log
  /// reader will also include log messages from before the invocation time.
  /// Defaults to false.
  FutureOr<DeviceLogReader> getLogReader({ApplicationPackage? app, bool includePastLogs = false});

  /// Get the port forwarder for this device.
  DevicePortForwarder? get portForwarder;

  /// Get the DDS instance for this device.
  final DartDevelopmentService dds;

  /// Clear the device's logs.
  void clearLogs();

  /// Get the [VMServiceDiscoveryForAttach] instance for this device, which
  /// discovers, and forwards any necessary ports to the vm service uri of a
  /// running app on the device.
  ///
  /// If `appId` is specified, on supported platforms, the service discovery
  /// will only return the VM service URI from the given app.
  ///
  /// If `fuchsiaModule` is specified, this will only return the VM service uri
  /// from the specified Fuchsia module.
  ///
  /// If `filterDevicePort` is specified, this will only return the VM service
  /// uri that matches the given port on the device.
  VMServiceDiscoveryForAttach getVMServiceDiscoveryForAttach({
    String? appId,
    String? fuchsiaModule,
    int? filterDevicePort,
    int? expectedHostPort,
    required bool ipv6,
    required Logger logger,
  }) => LogScanningVMServiceDiscoveryForAttach(
    Future<DeviceLogReader>.value(getLogReader()),
    portForwarder: portForwarder,
    devicePort: filterDevicePort,
    hostPort: expectedHostPort,
    ipv6: ipv6,
    logger: logger,
  );

  /// Start an app package on the current device.
  ///
  /// [platformArgs] allows callers to pass platform-specific arguments to the
  /// start call. The build mode is not used by all platforms.
  Future<LaunchResult> startApp(
    covariant ApplicationPackage? package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs,
    bool prebuiltApplication = false,
    String? userIdentifier,
  });

  /// Whether this device implements support for hot reload.
  bool get supportsHotReload => true;

  /// Whether this device implements support for hot restart.
  bool get supportsHotRestart => true;

  /// Whether Flutter applications running on this device can be terminated
  /// from the VM Service.
  bool get supportsFlutterExit => true;

  /// Whether the device supports taking screenshots of a running flutter
  /// application.
  bool get supportsScreenshot => false;

  /// Whether the device supports the '--fast-start' development mode.
  bool get supportsFastStart => false;

  /// Whether the Flavors feature ('--flavor') is supported for this device.
  bool get supportsFlavors => false;

  /// Stop an app package on the current device.
  ///
  /// Specify [userIdentifier] to stop app installed to a profile (Android only).
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier});

  /// Query the current application memory usage..
  ///
  /// If the device does not support this callback, an empty map
  /// is returned.
  Future<MemoryInfo> queryMemoryInfo() {
    return Future<MemoryInfo>.value(const MemoryInfo.empty());
  }

  Future<void> takeScreenshot(File outputFile) => Future<void>.error('unimplemented');

  @nonVirtual
  @override
  // ignore: avoid_equals_and_hash_code_on_mutable_classes
  int get hashCode => id.hashCode;

  @nonVirtual
  @override
  // ignore: avoid_equals_and_hash_code_on_mutable_classes
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    return other is Device && other.id == id;
  }

  @override
  String toString() => name;

  static Future<List<String>> descriptions(List<Device> devices) async {
    if (devices.isEmpty) {
      return const <String>[];
    }

    // Extract device information
    final List<List<String>> table = <List<String>>[];
    for (final Device device in devices) {
      String supportIndicator = device.isSupported() ? '' : ' (unsupported)';
      final TargetPlatform targetPlatform = await device.targetPlatform;
      if (await device.isLocalEmulator) {
        final String type = targetPlatform == TargetPlatform.ios ? 'simulator' : 'emulator';
        supportIndicator += ' ($type)';
      }
      table.add(<String>[
        '${device.name} (${device.category})',
        device.id,
        await device.targetPlatformDisplayName,
        '${await device.sdkNameAndVersion}$supportIndicator',
      ]);
    }

    // Calculate column widths
    final List<int> indices = List<int>.generate(table[0].length - 1, (int i) => i);
    List<int> widths = indices.map<int>((int i) => 0).toList();
    for (final List<String> row in table) {
      widths = indices.map<int>((int i) => math.max(widths[i], row[i].length)).toList();
    }

    // Join columns into lines of text
    return <String>[
      for (final List<String> row in table)
        indices
            .map<String>((int i) => row[i].padRight(widths[i]))
            .followedBy(<String>[row.last])
            .join(' • '),
    ];
  }

  static Future<void> printDevices(
    List<Device> devices,
    Logger logger, {
    String prefix = '',
  }) async {
    for (final String line in await descriptions(devices)) {
      logger.printStatus('$prefix$line');
    }
  }

  static List<String> devicesPlatformTypes(List<Device> devices) {
    return devices.map((Device d) => d.platformType.toString()).toSet().toList()..sort();
  }

  /// Convert the Device object to a JSON representation suitable for serialization.
  Future<Map<String, Object>> toJson() async {
    final bool isLocalEmu = await isLocalEmulator;
    return <String, Object>{
      'name': name,
      'id': id,
      'isSupported': isSupported(),
      'targetPlatform': getNameForTargetPlatform(await targetPlatform),
      'emulator': isLocalEmu,
      'sdk': await sdkNameAndVersion,
      'capabilities': <String, Object>{
        'hotReload': supportsHotReload,
        'hotRestart': supportsHotRestart,
        'screenshot': supportsScreenshot,
        'fastStart': supportsFastStart,
        'flutterExit': supportsFlutterExit,
        'hardwareRendering': isLocalEmu && await supportsHardwareRendering,
        'startPaused': supportsStartPaused,
      },
    };
  }

  /// Clean up resources allocated by device.
  ///
  /// For example log readers or port forwarders.
  Future<void> dispose();
}

/// Information about an application's memory usage.
abstract class MemoryInfo {
  /// Const constructor to allow subclasses to be const.
  const MemoryInfo();

  /// Create a [MemoryInfo] object with no information.
  const factory MemoryInfo.empty() = _NoMemoryInfo;

  /// Convert the object to a JSON representation suitable for serialization.
  Map<String, Object> toJson();
}

class _NoMemoryInfo implements MemoryInfo {
  const _NoMemoryInfo();

  @override
  Map<String, Object> toJson() => <String, Object>{};
}

enum ImpellerStatus {
  platformDefault._(null),
  enabled._(true),
  disabled._(false);

  const ImpellerStatus._(this.asBool);

  factory ImpellerStatus.fromBool(bool? b) => switch (b) {
    true => enabled,
    false => disabled,
    null => platformDefault,
  };

  final bool? asBool;
}

class DebuggingOptions {
  DebuggingOptions.enabled(
    this.buildInfo, {
    this.startPaused = false,
    this.disableServiceAuthCodes = false,
    this.enableDds = true,
    this.cacheStartupProfile = false,
    this.dartEntrypointArgs = const <String>[],
    this.dartFlags = '',
    this.enableSoftwareRendering = false,
    this.skiaDeterministicRendering = false,
    this.traceSkia = false,
    this.traceAllowlist,
    this.traceSkiaAllowlist,
    this.traceSystrace = false,
    this.traceToFile,
    this.endlessTraceBuffer = false,
    this.dumpSkpOnShaderCompilation = false,
    this.cacheSkSL = false,
    this.purgePersistentCache = false,
    this.useTestFonts = false,
    this.verboseSystemLogs = false,
    this.hostVmServicePort,
    this.disablePortPublication = false,
    this.deviceVmServicePort,
    this.ddsPort,
    this.devToolsServerAddress,
    this.hostname,
    this.port,
    this.tlsCertPath,
    this.tlsCertKeyPath,
    this.webEnableExposeUrl,
    this.webUseSseForDebugProxy = true,
    this.webUseSseForDebugBackend = true,
    this.webUseSseForInjectedClient = true,
    this.webRunHeadless = false,
    this.webBrowserDebugPort,
    this.webBrowserFlags = const <String>[],
    this.webEnableExpressionEvaluation = false,
    this.webHeaders = const <String, String>{},
    this.webLaunchUrl,
    WebRendererMode? webRenderer,
    this.webUseWasm = false,
    this.vmserviceOutFile,
    this.fastStart = false,
    this.nullAssertions = false,
    this.nativeNullAssertions = false,
    this.enableImpeller = ImpellerStatus.platformDefault,
    this.enableVulkanValidation = false,
    this.uninstallFirst = false,
    this.serveObservatory = false,
    this.enableDartProfiling = true,
    this.enableEmbedderApi = false,
    this.usingCISystem = false,
    this.debugLogsDirectoryPath,
    this.enableDevTools = true,
    this.ipv6 = false,
    this.google3WorkspaceRoot,
    this.printDtd = false,
  }) : debuggingEnabled = true,
       webRenderer = webRenderer ?? WebRendererMode.getDefault(useWasm: webUseWasm);

  DebuggingOptions.disabled(
    this.buildInfo, {
    this.dartEntrypointArgs = const <String>[],
    this.port,
    this.hostname,
    this.tlsCertPath,
    this.tlsCertKeyPath,
    this.webEnableExposeUrl,
    this.webUseSseForDebugProxy = true,
    this.webUseSseForDebugBackend = true,
    this.webUseSseForInjectedClient = true,
    this.webRunHeadless = false,
    this.webBrowserDebugPort,
    this.webBrowserFlags = const <String>[],
    this.webLaunchUrl,
    this.webHeaders = const <String, String>{},
    WebRendererMode? webRenderer,
    this.webUseWasm = false,
    this.cacheSkSL = false,
    this.traceAllowlist,
    this.enableImpeller = ImpellerStatus.platformDefault,
    this.enableVulkanValidation = false,
    this.uninstallFirst = false,
    this.enableDartProfiling = true,
    this.enableEmbedderApi = false,
    this.usingCISystem = false,
    this.debugLogsDirectoryPath,
  }) : debuggingEnabled = false,
       useTestFonts = false,
       startPaused = false,
       dartFlags = '',
       disableServiceAuthCodes = false,
       enableDds = false,
       cacheStartupProfile = false,
       enableSoftwareRendering = false,
       skiaDeterministicRendering = false,
       traceSkia = false,
       traceSkiaAllowlist = null,
       traceSystrace = false,
       traceToFile = null,
       endlessTraceBuffer = false,
       dumpSkpOnShaderCompilation = false,
       purgePersistentCache = false,
       verboseSystemLogs = false,
       hostVmServicePort = null,
       disablePortPublication = false,
       deviceVmServicePort = null,
       ddsPort = null,
       devToolsServerAddress = null,
       vmserviceOutFile = null,
       fastStart = false,
       webEnableExpressionEvaluation = false,
       nullAssertions = false,
       nativeNullAssertions = false,
       serveObservatory = false,
       enableDevTools = false,
       ipv6 = false,
       google3WorkspaceRoot = null,
       printDtd = false,
       webRenderer = webRenderer ?? WebRendererMode.getDefault(useWasm: webUseWasm);

  DebuggingOptions._({
    required this.buildInfo,
    required this.debuggingEnabled,
    required this.startPaused,
    required this.dartFlags,
    required this.dartEntrypointArgs,
    required this.disableServiceAuthCodes,
    required this.enableDds,
    required this.cacheStartupProfile,
    required this.enableSoftwareRendering,
    required this.skiaDeterministicRendering,
    required this.traceSkia,
    required this.traceAllowlist,
    required this.traceSkiaAllowlist,
    required this.traceSystrace,
    required this.traceToFile,
    required this.endlessTraceBuffer,
    required this.dumpSkpOnShaderCompilation,
    required this.cacheSkSL,
    required this.purgePersistentCache,
    required this.useTestFonts,
    required this.verboseSystemLogs,
    required this.hostVmServicePort,
    required this.deviceVmServicePort,
    required this.disablePortPublication,
    required this.ddsPort,
    required this.devToolsServerAddress,
    required this.port,
    required this.hostname,
    required this.tlsCertPath,
    required this.tlsCertKeyPath,
    required this.webEnableExposeUrl,
    required this.webUseSseForDebugProxy,
    required this.webUseSseForDebugBackend,
    required this.webUseSseForInjectedClient,
    required this.webRunHeadless,
    required this.webBrowserDebugPort,
    required this.webBrowserFlags,
    required this.webEnableExpressionEvaluation,
    required this.webHeaders,
    required this.webLaunchUrl,
    required this.webRenderer,
    required this.webUseWasm,
    required this.vmserviceOutFile,
    required this.fastStart,
    required this.nullAssertions,
    required this.nativeNullAssertions,
    required this.enableImpeller,
    required this.enableVulkanValidation,
    required this.uninstallFirst,
    required this.serveObservatory,
    required this.enableDartProfiling,
    required this.enableEmbedderApi,
    required this.usingCISystem,
    required this.debugLogsDirectoryPath,
    required this.enableDevTools,
    required this.ipv6,
    required this.google3WorkspaceRoot,
    required this.printDtd,
  });

  final bool debuggingEnabled;

  final BuildInfo buildInfo;
  final bool startPaused;
  final String dartFlags;
  final List<String> dartEntrypointArgs;
  final bool disableServiceAuthCodes;
  final bool enableDds;
  final bool cacheStartupProfile;
  final bool enableSoftwareRendering;
  final bool skiaDeterministicRendering;
  final bool traceSkia;
  final String? traceAllowlist;
  final String? traceSkiaAllowlist;
  final bool traceSystrace;
  final String? traceToFile;
  final bool endlessTraceBuffer;
  final bool dumpSkpOnShaderCompilation;
  final bool cacheSkSL;
  final bool purgePersistentCache;
  final bool useTestFonts;
  final bool verboseSystemLogs;
  final int? hostVmServicePort;
  final int? deviceVmServicePort;
  final bool disablePortPublication;
  final int? ddsPort;
  final Uri? devToolsServerAddress;
  final String? port;
  final String? hostname;
  final String? tlsCertPath;
  final String? tlsCertKeyPath;
  final bool? webEnableExposeUrl;
  final bool webUseSseForDebugProxy;
  final bool webUseSseForDebugBackend;
  final bool webUseSseForInjectedClient;
  final ImpellerStatus enableImpeller;
  final bool enableVulkanValidation;
  final bool serveObservatory;
  final bool enableDartProfiling;
  final bool enableEmbedderApi;
  final bool usingCISystem;
  final String? debugLogsDirectoryPath;
  final bool enableDevTools;
  final bool ipv6;
  final String? google3WorkspaceRoot;
  final bool printDtd;

  /// Whether the tool should try to uninstall a previously installed version of the app.
  ///
  /// This is not implemented for every platform.
  final bool uninstallFirst;

  /// Whether to run the browser in headless mode.
  ///
  /// Some CI environments do not provide a display and fail to launch the
  /// browser with full graphics stack. Some browsers provide a special
  /// "headless" mode that runs the browser with no graphics.
  final bool webRunHeadless;

  /// The port the browser should use for its debugging protocol.
  final int? webBrowserDebugPort;

  /// Arbitrary browser flags.
  final List<String> webBrowserFlags;

  /// Enable expression evaluation for web target.
  final bool webEnableExpressionEvaluation;

  /// Allow developers to customize the browser's launch URL
  final String? webLaunchUrl;

  /// Allow developers to add custom headers to web server
  final Map<String, String> webHeaders;

  /// Which web renderer to use for the debugging session
  final WebRendererMode webRenderer;

  /// Whether to compile to webassembly
  final bool webUseWasm;

  /// A file where the VM Service URL should be written after the application is started.
  final String? vmserviceOutFile;
  final bool fastStart;

  final bool nullAssertions;

  /// Additional null runtime checks inserted for web applications.
  ///
  /// See also:
  ///   * https://github.com/dart-lang/sdk/blob/main/sdk/lib/html/doc/NATIVE_NULL_ASSERTIONS.md
  final bool nativeNullAssertions;

  List<String> getIOSLaunchArguments(
    EnvironmentType environmentType,
    String? route,
    Map<String, Object?> platformArgs, {
    DeviceConnectionInterface interfaceType = DeviceConnectionInterface.attached,
    bool isCoreDevice = false,
  }) {
    final String dartVmFlags = computeDartVmFlags(this);
    return <String>[
      if (enableDartProfiling) '--enable-dart-profiling',
      if (disableServiceAuthCodes) '--disable-service-auth-codes',
      if (disablePortPublication) '--disable-vm-service-publication',
      if (startPaused) '--start-paused',
      // Wrap dart flags in quotes for physical devices
      if (environmentType == EnvironmentType.physical && dartVmFlags.isNotEmpty)
        '--dart-flags="$dartVmFlags"',
      if (environmentType == EnvironmentType.simulator && dartVmFlags.isNotEmpty)
        '--dart-flags=$dartVmFlags',
      if (useTestFonts) '--use-test-fonts',
      // Core Devices (iOS 17 devices) are debugged through Xcode so don't
      // include these flags, which are used to check if the app was launched
      // via Flutter CLI and `ios-deploy`.
      if (debuggingEnabled && !isCoreDevice) ...<String>[
        '--enable-checked-mode',
        '--verify-entry-points',
      ],
      if (enableSoftwareRendering) '--enable-software-rendering',
      if (traceSystrace) '--trace-systrace',
      if (traceToFile != null) '--trace-to-file="$traceToFile"',
      if (skiaDeterministicRendering) '--skia-deterministic-rendering',
      if (traceSkia) '--trace-skia',
      if (traceAllowlist != null) '--trace-allowlist="$traceAllowlist"',
      if (traceSkiaAllowlist != null) '--trace-skia-allowlist="$traceSkiaAllowlist"',
      if (endlessTraceBuffer) '--endless-trace-buffer',
      if (dumpSkpOnShaderCompilation) '--dump-skp-on-shader-compilation',
      if (verboseSystemLogs) '--verbose-logging',
      if (cacheSkSL) '--cache-sksl',
      if (purgePersistentCache) '--purge-persistent-cache',
      if (route != null) '--route=$route',
      if (platformArgs['trace-startup'] as bool? ?? false) '--trace-startup',
      if (enableImpeller == ImpellerStatus.enabled) '--enable-impeller=true',
      if (enableImpeller == ImpellerStatus.disabled) '--enable-impeller=false',
      if (environmentType == EnvironmentType.physical && deviceVmServicePort != null)
        '--vm-service-port=$deviceVmServicePort',
      // The simulator "device" is actually on the host machine so no ports will be forwarded.
      // Use the suggested host port.
      if (environmentType == EnvironmentType.simulator && hostVmServicePort != null)
        '--vm-service-port=$hostVmServicePort',
      // Tell the VM service to listen on all interfaces, don't restrict to the loopback.
      if (interfaceType == DeviceConnectionInterface.wireless)
        '--vm-service-host=${ipv6 ? '::0' : '0.0.0.0'}',
      if (enableEmbedderApi) '--enable-embedder-api',
    ];
  }

  Map<String, Object?> toJson() => <String, Object?>{
    'debuggingEnabled': debuggingEnabled,
    'startPaused': startPaused,
    'dartFlags': dartFlags,
    'dartEntrypointArgs': dartEntrypointArgs,
    'disableServiceAuthCodes': disableServiceAuthCodes,
    'enableDds': enableDds,
    'cacheStartupProfile': cacheStartupProfile,
    'enableSoftwareRendering': enableSoftwareRendering,
    'skiaDeterministicRendering': skiaDeterministicRendering,
    'traceSkia': traceSkia,
    'traceAllowlist': traceAllowlist,
    'traceSkiaAllowlist': traceSkiaAllowlist,
    'traceSystrace': traceSystrace,
    'traceToFile': traceToFile,
    'endlessTraceBuffer': endlessTraceBuffer,
    'dumpSkpOnShaderCompilation': dumpSkpOnShaderCompilation,
    'cacheSkSL': cacheSkSL,
    'purgePersistentCache': purgePersistentCache,
    'useTestFonts': useTestFonts,
    'verboseSystemLogs': verboseSystemLogs,
    'hostVmServicePort': hostVmServicePort,
    'deviceVmServicePort': deviceVmServicePort,
    'disablePortPublication': disablePortPublication,
    'ddsPort': ddsPort,
    'devToolsServerAddress': devToolsServerAddress.toString(),
    'port': port,
    'hostname': hostname,
    'tlsCertPath': tlsCertPath,
    'tlsCertKeyPath': tlsCertKeyPath,
    'webEnableExposeUrl': webEnableExposeUrl,
    'webUseSseForDebugProxy': webUseSseForDebugProxy,
    'webUseSseForDebugBackend': webUseSseForDebugBackend,
    'webUseSseForInjectedClient': webUseSseForInjectedClient,
    'webRunHeadless': webRunHeadless,
    'webBrowserDebugPort': webBrowserDebugPort,
    'webBrowserFlags': webBrowserFlags,
    'webEnableExpressionEvaluation': webEnableExpressionEvaluation,
    'webLaunchUrl': webLaunchUrl,
    'webHeaders': webHeaders,
    'webRenderer': webRenderer.name,
    'webUseWasm': webUseWasm,
    'vmserviceOutFile': vmserviceOutFile,
    'fastStart': fastStart,
    'nullAssertions': nullAssertions,
    'nativeNullAssertions': nativeNullAssertions,
    'enableImpeller': enableImpeller.asBool,
    'enableVulkanValidation': enableVulkanValidation,
    'serveObservatory': serveObservatory,
    'enableDartProfiling': enableDartProfiling,
    'enableEmbedderApi': enableEmbedderApi,
    'usingCISystem': usingCISystem,
    'debugLogsDirectoryPath': debugLogsDirectoryPath,
    'enableDevTools': enableDevTools,
    'ipv6': ipv6,
    'google3WorkspaceRoot': google3WorkspaceRoot,
    'printDtd': printDtd,
    // TODO(jsimmons): This field is required for backward compatibility with
    // the flutter_tools binary that is currently checked into Google3.
    // Remove this when that binary has been updated.
    'webUseLocalCanvaskit': false,
  };

  static DebuggingOptions fromJson(Map<String, Object?> json, BuildInfo buildInfo) =>
      DebuggingOptions._(
        buildInfo: buildInfo,
        debuggingEnabled: json['debuggingEnabled']! as bool,
        startPaused: json['startPaused']! as bool,
        dartFlags: json['dartFlags']! as String,
        dartEntrypointArgs: (json['dartEntrypointArgs']! as List<dynamic>).cast<String>(),
        disableServiceAuthCodes: json['disableServiceAuthCodes']! as bool,
        enableDds: json['enableDds']! as bool,
        cacheStartupProfile: json['cacheStartupProfile']! as bool,
        enableSoftwareRendering: json['enableSoftwareRendering']! as bool,
        skiaDeterministicRendering: json['skiaDeterministicRendering']! as bool,
        traceSkia: json['traceSkia']! as bool,
        traceAllowlist: json['traceAllowlist'] as String?,
        traceSkiaAllowlist: json['traceSkiaAllowlist'] as String?,
        traceSystrace: json['traceSystrace']! as bool,
        traceToFile: json['traceToFile'] as String?,
        endlessTraceBuffer: json['endlessTraceBuffer']! as bool,
        dumpSkpOnShaderCompilation: json['dumpSkpOnShaderCompilation']! as bool,
        cacheSkSL: json['cacheSkSL']! as bool,
        purgePersistentCache: json['purgePersistentCache']! as bool,
        useTestFonts: json['useTestFonts']! as bool,
        verboseSystemLogs: json['verboseSystemLogs']! as bool,
        hostVmServicePort: json['hostVmServicePort'] as int?,
        deviceVmServicePort: json['deviceVmServicePort'] as int?,
        disablePortPublication: json['disablePortPublication']! as bool,
        ddsPort: json['ddsPort'] as int?,
        devToolsServerAddress:
            json['devToolsServerAddress'] != null
                ? Uri.parse(json['devToolsServerAddress']! as String)
                : null,
        port: json['port'] as String?,
        hostname: json['hostname'] as String?,
        tlsCertPath: json['tlsCertPath'] as String?,
        tlsCertKeyPath: json['tlsCertKeyPath'] as String?,
        webEnableExposeUrl: json['webEnableExposeUrl'] as bool?,
        webUseSseForDebugProxy: json['webUseSseForDebugProxy']! as bool,
        webUseSseForDebugBackend: json['webUseSseForDebugBackend']! as bool,
        webUseSseForInjectedClient: json['webUseSseForInjectedClient']! as bool,
        webRunHeadless: json['webRunHeadless']! as bool,
        webBrowserDebugPort: json['webBrowserDebugPort'] as int?,
        webBrowserFlags: (json['webBrowserFlags']! as List<dynamic>).cast<String>(),
        webEnableExpressionEvaluation: json['webEnableExpressionEvaluation']! as bool,
        webHeaders: (json['webHeaders']! as Map<dynamic, dynamic>).cast<String, String>(),
        webLaunchUrl: json['webLaunchUrl'] as String?,
        webRenderer: WebRendererMode.values.byName(json['webRenderer']! as String),
        webUseWasm: json['webUseWasm']! as bool,
        vmserviceOutFile: json['vmserviceOutFile'] as String?,
        fastStart: json['fastStart']! as bool,
        nullAssertions: json['nullAssertions']! as bool,
        nativeNullAssertions: json['nativeNullAssertions']! as bool,
        enableImpeller: ImpellerStatus.fromBool(json['enableImpeller'] as bool?),
        enableVulkanValidation: (json['enableVulkanValidation'] as bool?) ?? false,
        uninstallFirst: (json['uninstallFirst'] as bool?) ?? false,
        serveObservatory: (json['serveObservatory'] as bool?) ?? false,
        enableDartProfiling: (json['enableDartProfiling'] as bool?) ?? true,
        enableEmbedderApi: (json['enableEmbedderApi'] as bool?) ?? false,
        usingCISystem: (json['usingCISystem'] as bool?) ?? false,
        debugLogsDirectoryPath: json['debugLogsDirectoryPath'] as String?,
        enableDevTools: (json['enableDevTools'] as bool?) ?? true,
        ipv6: (json['ipv6'] as bool?) ?? false,
        google3WorkspaceRoot: json['google3WorkspaceRoot'] as String?,
        printDtd: (json['printDtd'] as bool?) ?? false,
      );
}

class LaunchResult {
  LaunchResult.succeeded({Uri? vmServiceUri, Uri? observatoryUri})
    : started = true,
      vmServiceUri = vmServiceUri ?? observatoryUri;

  LaunchResult.failed() : started = false, vmServiceUri = null;

  bool get hasVmService => vmServiceUri != null;

  final bool started;
  final Uri? vmServiceUri;

  @override
  String toString() {
    final StringBuffer buf = StringBuffer('started=$started');
    if (vmServiceUri != null) {
      buf.write(', vmService=$vmServiceUri');
    }
    return buf.toString();
  }
}

/// Read the log for a particular device.
abstract class DeviceLogReader {
  String get name;

  /// A broadcast stream where each element in the string is a line of log output.
  Stream<String> get logLines;

  /// Some logs can be obtained from a VM service stream.
  /// Set this after the VM services are connected.
  Future<void> provideVmService(FlutterVmService connectedVmService);

  @override
  String toString() => name;

  // Clean up resources allocated by log reader e.g. subprocesses
  void dispose();
}

/// Describes an app running on the device.
class DiscoveredApp {
  DiscoveredApp(this.id, this.vmServicePort);
  final String id;
  final int vmServicePort;
}

// An empty device log reader
class NoOpDeviceLogReader implements DeviceLogReader {
  NoOpDeviceLogReader(String? nameOrNull) : name = nameOrNull ?? '';

  @override
  final String name;

  @override
  Stream<String> get logLines => const Stream<String>.empty();

  @override
  void dispose() {}

  @override
  Future<void> provideVmService(FlutterVmService connectedVmService) async {}
}

/// Append --null_assertions to any existing Dart VM flags if
/// [debuggingOptions.nullAssertions] is true.
String computeDartVmFlags(DebuggingOptions debuggingOptions) {
  return <String>[
    if (debuggingOptions.dartFlags.isNotEmpty) debuggingOptions.dartFlags,
    if (debuggingOptions.nullAssertions) '--null_assertions',
  ].join(',');
}
