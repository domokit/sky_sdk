// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../artifacts.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/version.dart';
import '../build_info.dart';
import '../cache.dart';
import '../convert.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../ios/core_devices.dart';
import '../ios/devices.dart';
import '../ios/ios_deploy.dart';
import '../ios/iproxy.dart';
import '../ios/mac.dart';
import '../ios/xcode_debug.dart';
import 'xcode.dart';

class XCDeviceEventNotification {
  XCDeviceEventNotification(this.eventType, this.eventInterface, this.deviceIdentifier);

  final XCDeviceEvent eventType;
  final XCDeviceEventInterface eventInterface;
  final String deviceIdentifier;
}

enum XCDeviceEvent { attach, detach }

enum XCDeviceEventInterface {
  usb(name: 'usb', connectionInterface: DeviceConnectionInterface.attached),
  wifi(name: 'wifi', connectionInterface: DeviceConnectionInterface.wireless);

  const XCDeviceEventInterface({required this.name, required this.connectionInterface});

  final String name;
  final DeviceConnectionInterface connectionInterface;
}

/// A utility class for interacting with Xcode xcdevice command line tools.
class XCDevice {
  XCDevice({
    required Artifacts artifacts,
    required Cache cache,
    required ProcessManager processManager,
    required Logger logger,
    required Xcode xcode,
    required Platform platform,
    required IProxy iproxy,
    required FileSystem fileSystem,
    required Analytics analytics,
    required ShutdownHooks shutdownHooks,
    @visibleForTesting IOSCoreDeviceControl? coreDeviceControl,
    XcodeDebug? xcodeDebug,
  }) : _processUtils = ProcessUtils(logger: logger, processManager: processManager),
       _logger = logger,
       _iMobileDevice = IMobileDevice(
         artifacts: artifacts,
         cache: cache,
         logger: logger,
         processManager: processManager,
       ),
       _iosDeploy = IOSDeploy(
         artifacts: artifacts,
         cache: cache,
         logger: logger,
         platform: platform,
         processManager: processManager,
       ),
       _coreDeviceControl =
           coreDeviceControl ??
           IOSCoreDeviceControl(
             logger: logger,
             processManager: processManager,
             xcode: xcode,
             fileSystem: fileSystem,
           ),
       _xcodeDebug =
           xcodeDebug ??
           XcodeDebug(
             logger: logger,
             processManager: processManager,
             xcode: xcode,
             fileSystem: fileSystem,
           ),
       _iProxy = iproxy,
       _xcode = xcode,
       _analytics = analytics {
    shutdownHooks.addShutdownHook(dispose);

    _setupDeviceIdentifierByEventStream();
  }

  void dispose() {
    _stopObservingTetheredIOSDevices();
    _usbDeviceWaitProcess?.kill();
    _wifiDeviceWaitProcess?.kill();
  }

  final ProcessUtils _processUtils;
  final Logger _logger;
  final IMobileDevice _iMobileDevice;
  final IOSDeploy _iosDeploy;
  final Xcode _xcode;
  final IProxy _iProxy;
  final IOSCoreDeviceControl _coreDeviceControl;
  final XcodeDebug _xcodeDebug;
  final Analytics _analytics;

  List<Object>? _cachedListResults;

  Process? _usbDeviceObserveProcess;
  Process? _wifiDeviceObserveProcess;
  StreamController<XCDeviceEventNotification>? _observeStreamController;

  @visibleForTesting
  StreamController<XCDeviceEventNotification>? waitStreamController;

  Process? _usbDeviceWaitProcess;
  Process? _wifiDeviceWaitProcess;

  void _setupDeviceIdentifierByEventStream() {
    // _observeStreamController Should always be available for listeners
    // in case polling needs to be stopped and restarted.
    _observeStreamController = StreamController<XCDeviceEventNotification>.broadcast(
      onListen: _startObservingTetheredIOSDevices,
      onCancel: _stopObservingTetheredIOSDevices,
    );
  }

  bool get isInstalled => _xcode.isInstalledAndMeetsVersionCheck;

  Future<List<Object>?> _getAllDevices({bool useCache = false, required Duration timeout}) async {
    if (!isInstalled) {
      _logger.printTrace("Xcode not found. Run 'flutter doctor' for more information.");
      return null;
    }
    if (useCache && _cachedListResults != null) {
      return _cachedListResults;
    }
    try {
      // USB-tethered devices should be found quickly. 1 second timeout is faster than the default.
      final RunResult result = await _processUtils.run(<String>[
        ..._xcode.xcrunCommand(),
        'xcdevice',
        'list',
        '--timeout',
        timeout.inSeconds.toString(),
      ], throwOnError: true);
      if (result.exitCode == 0) {
        final String listOutput = result.stdout;
        try {
          final List<Object> listResults =
              (json.decode(result.stdout) as List<Object?>).whereType<Object>().toList();
          _cachedListResults = listResults;
          return listResults;
        } on FormatException {
          // xcdevice logs errors and crashes to stdout.
          _logger.printError('xcdevice returned non-JSON response: $listOutput');
          return null;
        }
      }
      _logger.printTrace('xcdevice returned an error:\n${result.stderr}');
    } on ProcessException catch (exception) {
      _logger.printTrace('Process exception running xcdevice list:\n$exception');
    } on ArgumentError catch (exception) {
      _logger.printTrace('Argument exception running xcdevice list:\n$exception');
    }

    return null;
  }

  /// Observe identifiers (UDIDs) of devices as they attach and detach.
  ///
  /// Each attach and detach event contains information on the event type,
  /// the event interface, and the device identifier.
  Stream<XCDeviceEventNotification>? observedDeviceEvents() {
    if (!isInstalled) {
      _logger.printTrace("Xcode not found. Run 'flutter doctor' for more information.");
      return null;
    }
    return _observeStreamController?.stream;
  }

  // Attach: d83d5bc53967baa0ee18626ba87b6254b2ab5418
  // Attach: 00008027-00192736010F802E
  // Detach: d83d5bc53967baa0ee18626ba87b6254b2ab5418
  final RegExp _observationIdentifierPattern = RegExp(r'^(\w*): ([\w-]*)$');

  Future<void> _startObservingTetheredIOSDevices() async {
    try {
      if (_usbDeviceObserveProcess != null || _wifiDeviceObserveProcess != null) {
        throw Exception('xcdevice observe restart failed');
      }

      _usbDeviceObserveProcess = await _startObserveProcess(XCDeviceEventInterface.usb);

      _wifiDeviceObserveProcess = await _startObserveProcess(XCDeviceEventInterface.wifi);

      final Future<void> usbProcessExited = _usbDeviceObserveProcess!.exitCode.then((int status) {
        _logger.printTrace('xcdevice observe --usb exited with code $exitCode');
        // Kill other process in case only one was killed.
        _stopObservingTetheredIOSDevices();
      });

      final Future<void> wifiProcessExited = _wifiDeviceObserveProcess!.exitCode.then((int status) {
        _logger.printTrace('xcdevice observe --wifi exited with code $exitCode');
        // Kill other process in case only one was killed.
        _stopObservingTetheredIOSDevices();
      });

      unawaited(
        Future.wait(<Future<void>>[usbProcessExited, wifiProcessExited]).whenComplete(() async {
          if (_observeStreamController?.hasListener ?? false) {
            // Tell listeners the process died.
            await _observeStreamController?.close();
          }
          _usbDeviceObserveProcess = null;
          _wifiDeviceObserveProcess = null;

          // Reopen it so new listeners can resume polling.
          _setupDeviceIdentifierByEventStream();
        }),
      );
    } on ProcessException catch (exception, stackTrace) {
      _observeStreamController?.addError(exception, stackTrace);
    } on ArgumentError catch (exception, stackTrace) {
      _observeStreamController?.addError(exception, stackTrace);
    }
  }

  Future<Process> _startObserveProcess(XCDeviceEventInterface eventInterface) {
    // Run in interactive mode (via script) to convince
    // xcdevice it has a terminal attached in order to redirect stdout.
    return _streamXCDeviceEventCommand(
      <String>[
        'script',
        '-t',
        '0',
        '/dev/null',
        ..._xcode.xcrunCommand(),
        'xcdevice',
        'observe',
        '--${eventInterface.name}',
      ],
      prefix: 'xcdevice observe --${eventInterface.name}: ',
      mapFunction: (String line) {
        final XCDeviceEventNotification? event = _processXCDeviceStdOut(line, eventInterface);
        if (event != null) {
          _observeStreamController?.add(event);
        }
        return line;
      },
    );
  }

  /// Starts the command and streams stdout/stderr from the child process to
  /// this process' stdout/stderr.
  ///
  /// If [mapFunction] is present, all lines are forwarded to [mapFunction] for
  /// further processing.
  Future<Process> _streamXCDeviceEventCommand(
    List<String> cmd, {
    String prefix = '',
    StringConverter? mapFunction,
  }) async {
    final Process process = await _processUtils.start(cmd);

    final StreamSubscription<String> stdoutSubscription = process.stdout
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter())
        .listen((String line) {
          String? mappedLine = line;
          if (mapFunction != null) {
            mappedLine = mapFunction(line);
          }
          if (mappedLine != null) {
            final String message = '$prefix$mappedLine';
            _logger.printTrace(message);
          }
        });
    final StreamSubscription<String> stderrSubscription = process.stderr
        .transform<String>(utf8.decoder)
        .transform<String>(const LineSplitter())
        .listen((String line) {
          String? mappedLine = line;
          if (mapFunction != null) {
            mappedLine = mapFunction(line);
          }
          if (mappedLine != null) {
            _logger.printError('$prefix$mappedLine', wrap: false);
          }
        });

    unawaited(
      process.exitCode.whenComplete(() {
        stdoutSubscription.cancel();
        stderrSubscription.cancel();
      }),
    );

    return process;
  }

  void _stopObservingTetheredIOSDevices() {
    // xcdevice observe is running in an interactive shell.
    // Signal script child jobs to exit and exit the shell.
    // See https://linux.die.net/Bash-Beginners-Guide/sect_12_01.html#sect_12_01_01_02.
    if (_usbDeviceObserveProcess != null) {
      ProcessSignal.sighup.kill(_usbDeviceObserveProcess!);
    }
    if (_wifiDeviceObserveProcess != null) {
      ProcessSignal.sighup.kill(_wifiDeviceObserveProcess!);
    }
  }

  XCDeviceEventNotification? _processXCDeviceStdOut(
    String line,
    XCDeviceEventInterface eventInterface,
  ) {
    // xcdevice observe example output of UDIDs:
    //
    // Listening for all devices, on both interfaces.
    // Attach: d83d5bc53967baa0ee18626ba87b6254b2ab5418
    // Attach: 00008027-00192736010F802E
    // Detach: d83d5bc53967baa0ee18626ba87b6254b2ab5418
    // Attach: d83d5bc53967baa0ee18626ba87b6254b2ab5418
    final RegExpMatch? match = _observationIdentifierPattern.firstMatch(line);
    if (match != null && match.groupCount == 2) {
      final String verb = match.group(1)!.toLowerCase();
      final String identifier = match.group(2)!;
      if (verb.startsWith('attach')) {
        return XCDeviceEventNotification(XCDeviceEvent.attach, eventInterface, identifier);
      } else if (verb.startsWith('detach')) {
        return XCDeviceEventNotification(XCDeviceEvent.detach, eventInterface, identifier);
      }
    }
    return null;
  }

  /// Wait for a connect event for a specific device. Must use device's exact UDID.
  ///
  /// To cancel this process, call [cancelWaitForDeviceToConnect].
  Future<XCDeviceEventNotification?> waitForDeviceToConnect(String deviceId) async {
    try {
      if (_usbDeviceWaitProcess != null || _wifiDeviceWaitProcess != null) {
        throw Exception('xcdevice wait restart failed');
      }

      waitStreamController = StreamController<XCDeviceEventNotification>();

      _usbDeviceWaitProcess = await _startWaitProcess(deviceId, XCDeviceEventInterface.usb);

      _wifiDeviceWaitProcess = await _startWaitProcess(deviceId, XCDeviceEventInterface.wifi);

      final Future<void> usbProcessExited = _usbDeviceWaitProcess!.exitCode.then((int status) {
        _logger.printTrace('xcdevice wait --usb exited with code $exitCode');
        // Kill other process in case only one was killed.
        _wifiDeviceWaitProcess?.kill();
      });

      final Future<void> wifiProcessExited = _wifiDeviceWaitProcess!.exitCode.then((int status) {
        _logger.printTrace('xcdevice wait --wifi exited with code $exitCode');
        // Kill other process in case only one was killed.
        _usbDeviceWaitProcess?.kill();
      });

      final Future<void> allProcessesExited = Future.wait(<Future<void>>[
        usbProcessExited,
        wifiProcessExited,
      ]).whenComplete(() async {
        _usbDeviceWaitProcess = null;
        _wifiDeviceWaitProcess = null;
        await waitStreamController?.close();
      });

      return await Future.any(<Future<XCDeviceEventNotification?>>[
        allProcessesExited.then((_) => null),
        waitStreamController!.stream.first.whenComplete(() async {
          cancelWaitForDeviceToConnect();
        }),
      ]);
    } on ProcessException catch (exception, stackTrace) {
      _logger.printTrace('Process exception running xcdevice wait:\n$exception\n$stackTrace');
    } on ArgumentError catch (exception, stackTrace) {
      _logger.printTrace('Process exception running xcdevice wait:\n$exception\n$stackTrace');
    } on StateError {
      _logger.printTrace('Stream broke before first was found');
      return null;
    }
    return null;
  }

  Future<Process> _startWaitProcess(String deviceId, XCDeviceEventInterface eventInterface) {
    // Run in interactive mode (via script) to convince
    // xcdevice it has a terminal attached in order to redirect stdout.
    return _streamXCDeviceEventCommand(
      <String>[
        'script',
        '-t',
        '0',
        '/dev/null',
        ..._xcode.xcrunCommand(),
        'xcdevice',
        'wait',
        '--${eventInterface.name}',
        deviceId,
      ],
      prefix: 'xcdevice wait --${eventInterface.name}: ',
      mapFunction: (String line) {
        final XCDeviceEventNotification? event = _processXCDeviceStdOut(line, eventInterface);
        if (event != null && event.eventType == XCDeviceEvent.attach) {
          waitStreamController?.add(event);
        }
        return line;
      },
    );
  }

  void cancelWaitForDeviceToConnect() {
    _usbDeviceWaitProcess?.kill();
    _wifiDeviceWaitProcess?.kill();
  }

  /// A list of [IOSDevice]s. This list includes connected devices and
  /// disconnected wireless devices.
  ///
  /// Sometimes devices may have incorrect connection information
  /// (`isConnected`, `connectionInterface`) if it timed out before it could get the
  /// information. Wireless devices can take longer to get the correct
  /// information.
  ///
  /// [timeout] defaults to 2 seconds.
  Future<List<IOSDevice>> getAvailableIOSDevices({Duration? timeout}) async {
    final List<Object>? allAvailableDevices = await _getAllDevices(
      timeout: timeout ?? const Duration(seconds: 2),
    );

    if (allAvailableDevices == null) {
      return const <IOSDevice>[];
    }

    final Map<String, IOSCoreDevice> coreDeviceMap = <String, IOSCoreDevice>{};
    if (_xcode.isDevicectlInstalled) {
      final List<IOSCoreDevice> coreDevices = await _coreDeviceControl.getCoreDevices();
      for (final IOSCoreDevice device in coreDevices) {
        if (device.udid == null) {
          continue;
        }
        coreDeviceMap[device.udid!] = device;
      }
    }

    // [
    //  {
    //    "simulator" : true,
    //    "operatingSystemVersion" : "13.3 (17K446)",
    //    "available" : true,
    //    "platform" : "com.apple.platform.appletvsimulator",
    //    "modelCode" : "AppleTV5,3",
    //    "identifier" : "CBB5E1ED-2172-446E-B4E7-F2B5823DBBA6",
    //    "architecture" : "x86_64",
    //    "modelName" : "Apple TV",
    //    "name" : "Apple TV"
    //  },
    //  {
    //    "simulator" : false,
    //    "operatingSystemVersion" : "13.3 (17C54)",
    //    "interface" : "usb",
    //    "available" : true,
    //    "platform" : "com.apple.platform.iphoneos",
    //    "modelCode" : "iPhone8,1",
    //    "identifier" : "d83d5bc53967baa0ee18626ba87b6254b2ab5418",
    //    "architecture" : "arm64",
    //    "modelName" : "iPhone 6s",
    //    "name" : "iPhone"
    //  },
    //  {
    //    "simulator" : true,
    //    "operatingSystemVersion" : "6.1.1 (17S445)",
    //    "available" : true,
    //    "platform" : "com.apple.platform.watchsimulator",
    //    "modelCode" : "Watch5,4",
    //    "identifier" : "2D74FB11-88A0-44D0-B81E-C0C142B1C94A",
    //    "architecture" : "i386",
    //    "modelName" : "Apple Watch Series 5 - 44mm",
    //    "name" : "Apple Watch Series 5 - 44mm"
    //  },
    // ...

    final Map<String, IOSDevice> deviceMap = <String, IOSDevice>{};
    for (final Object device in allAvailableDevices) {
      if (device is Map<String, Object?>) {
        // Only include iPhone, iPad, iPod, or other iOS devices.
        if (!_isIPhoneOSDevice(device)) {
          continue;
        }
        final String? identifier = device['identifier'] as String?;
        final String? name = device['name'] as String?;
        if (identifier == null || name == null) {
          continue;
        }
        bool devModeEnabled = true;
        bool isConnected = true;
        bool isPaired = true;
        final Map<String, Object?>? errorProperties = _errorProperties(device);
        if (errorProperties != null) {
          final String? errorMessage = _parseErrorMessage(errorProperties);
          if (errorMessage != null) {
            if (errorMessage.contains('not paired')) {
              _analytics.send(
                Event.appleUsageEvent(workflow: 'device', parameter: 'ios-trust-failure'),
              );
            }
            _logger.printTrace(errorMessage);
          }

          final int? code = _errorCode(errorProperties);

          // Temporary error -10: iPhone is busy: Preparing debugger support for iPhone.
          // Sometimes the app launch will fail on these devices until Xcode is done setting up the device.
          // Other times this is a false positive and the app will successfully launch despite the error.
          if (code != -10) {
            isConnected = false;
          }
          // Error: iPhone is not paired with your computer. To use iPhone with Xcode, unlock it and choose to trust this computer when prompted. (code -9)
          if (code == -9) {
            isPaired = false;
          }

          if (code == 6) {
            devModeEnabled = false;
          }
        }

        String? sdkVersionString = _sdkVersion(device);

        if (sdkVersionString != null) {
          final String? buildVersion = _buildVersion(device);
          if (buildVersion != null) {
            sdkVersionString = '$sdkVersionString $buildVersion';
          }
        }

        // Duplicate entries started appearing in Xcode 15, possibly due to
        // Xcode's new device connectivity stack.
        // If a duplicate entry is found in `xcdevice list`, don't overwrite
        // existing entry when the existing entry indicates the device is
        // connected and the current entry indicates the device is not connected.
        // Don't overwrite if current entry's sdkVersion is null.
        // Don't overwrite if both entries indicate the device is not
        // connected and the existing entry has a higher sdkVersion.
        if (deviceMap.containsKey(identifier)) {
          final IOSDevice deviceInMap = deviceMap[identifier]!;
          if ((deviceInMap.isConnected && !isConnected) || sdkVersionString == null) {
            continue;
          }

          final Version? sdkVersion = Version.parse(sdkVersionString);
          if (!deviceInMap.isConnected &&
              !isConnected &&
              sdkVersion != null &&
              deviceInMap.sdkVersion != null &&
              deviceInMap.sdkVersion!.compareTo(sdkVersion) > 0) {
            continue;
          }
        }

        DeviceConnectionInterface connectionInterface = _interfaceType(device);

        // CoreDevices (devices with iOS 17 and greater) no longer reflect the
        // correct connection interface or developer mode status in `xcdevice`.
        // Use `devicectl` to get that information for CoreDevices.
        final IOSCoreDevice? coreDevice = coreDeviceMap[identifier];
        if (coreDevice != null) {
          if (coreDevice.connectionInterface != null) {
            connectionInterface = coreDevice.connectionInterface!;
          }

          if (coreDevice.deviceProperties?.developerModeStatus != 'enabled') {
            devModeEnabled = false;
          }
        }

        deviceMap[identifier] = IOSDevice(
          identifier,
          name: name,
          cpuArchitecture: _cpuArchitecture(device),
          connectionInterface: connectionInterface,
          isConnected: isConnected,
          sdkVersion: sdkVersionString,
          iProxy: _iProxy,
          fileSystem: globals.fs,
          logger: _logger,
          iosDeploy: _iosDeploy,
          iMobileDevice: _iMobileDevice,
          coreDeviceControl: _coreDeviceControl,
          xcodeDebug: _xcodeDebug,
          platform: globals.platform,
          devModeEnabled: devModeEnabled,
          isPaired: isPaired,
          isCoreDevice: coreDevice != null,
        );
      }
    }
    return deviceMap.values.toList();
  }

  /// Despite the name, com.apple.platform.iphoneos includes iPhone, iPads, and all iOS devices.
  /// Excludes simulators.
  static bool _isIPhoneOSDevice(Map<String, Object?> deviceProperties) {
    final Object? platform = deviceProperties['platform'];
    if (platform is String) {
      return platform == 'com.apple.platform.iphoneos';
    }
    return false;
  }

  static Map<String, Object?>? _errorProperties(Map<String, Object?> deviceProperties) {
    final Object? error = deviceProperties['error'];
    return error is Map<String, Object?> ? error : null;
  }

  static int? _errorCode(Map<String, Object?>? errorProperties) {
    if (errorProperties == null) {
      return null;
    }
    final Object? code = errorProperties['code'];
    return code is int ? code : null;
  }

  static DeviceConnectionInterface _interfaceType(Map<String, Object?> deviceProperties) {
    // Interface can be "usb" or "network". It can also be missing
    // (e.g. simulators do not have an interface property).
    // If the interface is "network", use `DeviceConnectionInterface.wireless`,
    // otherwise use `DeviceConnectionInterface.attached.
    final Object? interface = deviceProperties['interface'];
    if (interface is String && interface.toLowerCase() == 'network') {
      return DeviceConnectionInterface.wireless;
    }
    return DeviceConnectionInterface.attached;
  }

  static String? _sdkVersion(Map<String, Object?> deviceProperties) {
    final Object? operatingSystemVersion = deviceProperties['operatingSystemVersion'];
    if (operatingSystemVersion is String) {
      // Parse out the OS version, ignore the build number in parentheses.
      // "13.3 (17C54)"
      final RegExp operatingSystemRegex = RegExp(r'(.*) \(.*\)$');
      if (operatingSystemRegex.hasMatch(operatingSystemVersion.trim())) {
        return operatingSystemRegex.firstMatch(operatingSystemVersion.trim())?.group(1);
      }
      return operatingSystemVersion;
    }
    return null;
  }

  static String? _buildVersion(Map<String, Object?> deviceProperties) {
    final Object? operatingSystemVersion = deviceProperties['operatingSystemVersion'];
    if (operatingSystemVersion is String) {
      // Parse out the build version, for example 17C54 from "13.3 (17C54)".
      final RegExp buildVersionRegex = RegExp(r'\(.*\)$');
      return buildVersionRegex
          .firstMatch(operatingSystemVersion)
          ?.group(0)
          ?.replaceAll(RegExp('[()]'), '');
    }
    return null;
  }

  DarwinArch _cpuArchitecture(Map<String, Object?> deviceProperties) {
    DarwinArch? cpuArchitecture;
    final Object? architecture = deviceProperties['architecture'];
    if (architecture is String) {
      try {
        cpuArchitecture = getIOSArchForName(architecture);
      } on Exception {
        // Fallback to default iOS architecture. Future-proof against a
        // theoretical version of Xcode that changes this string to something
        // slightly different like "ARM64", or armv7 variations like
        // armv7s and armv7f.
        if (architecture.startsWith('armv7')) {
          cpuArchitecture = DarwinArch.armv7;
        } else {
          cpuArchitecture = DarwinArch.arm64;
        }
        _logger.printWarning(
          'Unknown architecture $architecture, defaulting to '
          '${cpuArchitecture.name}',
        );
      }
    }
    return cpuArchitecture ?? DarwinArch.arm64;
  }

  /// Error message parsed from xcdevice. null if no error.
  static String? _parseErrorMessage(Map<String, Object?>? errorProperties) {
    //  {
    //    "simulator" : false,
    //    "operatingSystemVersion" : "13.3 (17C54)",
    //    "interface" : "usb",
    //    "available" : false,
    //    "platform" : "com.apple.platform.iphoneos",
    //    "modelCode" : "iPhone8,1",
    //    "identifier" : "98206e7a4afd4aedaff06e687594e089dede3c44",
    //    "architecture" : "arm64",
    //    "modelName" : "iPhone 6s",
    //    "name" : "iPhone",
    //    "error" : {
    //      "code" : -9,
    //      "failureReason" : "",
    //      "underlyingErrors" : [
    //        {
    //          "code" : 5,
    //          "failureReason" : "allowsSecureServices: 1. isConnected: 0. Platform: <DVTPlatform:0x7f804ce32880:'com.apple.platform.iphoneos':<DVTFilePath:0x7f804ce32800:'\/Users\/magder\/Applications\/Xcode_11-3-1.app\/Contents\/Developer\/Platforms\/iPhoneOS.platform'>>. DTDKDeviceIdentifierIsIDID: 0",
    //          "description" : "📱<DVTiOSDevice (0x7f801f190450), iPhone, iPhone, 13.3 (17C54), d83d5bc53967baa0ee18626ba87b6254b2ab5418> -- Failed _shouldMakeReadyForDevelopment check even though device is not locked by passcode.",
    //          "recoverySuggestion" : "",
    //          "domain" : "com.apple.platform.iphoneos"
    //        }
    //      ],
    //      "description" : "iPhone is not paired with your computer.",
    //      "recoverySuggestion" : "To use iPhone with Xcode, unlock it and choose to trust this computer when prompted.",
    //      "domain" : "com.apple.platform.iphoneos"
    //    }
    //  },
    //  {
    //    "simulator" : false,
    //    "operatingSystemVersion" : "13.3 (17C54)",
    //    "interface" : "usb",
    //    "available" : false,
    //    "platform" : "com.apple.platform.iphoneos",
    //    "modelCode" : "iPhone8,1",
    //    "identifier" : "d83d5bc53967baa0ee18626ba87b6254b2ab5418",
    //    "architecture" : "arm64",
    //    "modelName" : "iPhone 6s",
    //    "name" : "iPhone",
    //    "error" : {
    //      "code" : -9,
    //      "failureReason" : "",
    //      "description" : "iPhone is not paired with your computer.",
    //      "domain" : "com.apple.platform.iphoneos"
    //    }
    //  }
    // ...

    if (errorProperties == null) {
      return null;
    }

    final StringBuffer errorMessage = StringBuffer('Error: ');

    final Object? description = errorProperties['description'];
    if (description is String) {
      errorMessage.write(description);
      if (!description.endsWith('.')) {
        errorMessage.write('.');
      }
    } else {
      errorMessage.write('Xcode pairing error.');
    }

    final Object? recoverySuggestion = errorProperties['recoverySuggestion'];
    if (recoverySuggestion is String) {
      errorMessage.write(' $recoverySuggestion');
    }

    final int? code = _errorCode(errorProperties);
    if (code != null) {
      errorMessage.write(' (code $code)');
    }

    return errorMessage.toString();
  }

  /// List of all devices reporting errors.
  Future<List<String>> getDiagnostics() async {
    final List<Object>? allAvailableDevices = await _getAllDevices(
      useCache: true,
      timeout: const Duration(seconds: 2),
    );

    if (allAvailableDevices == null) {
      return const <String>[];
    }

    final List<String> diagnostics = <String>[];
    for (final Object deviceProperties in allAvailableDevices) {
      if (deviceProperties is! Map<String, Object?>) {
        continue;
      }
      final Map<String, Object?>? errorProperties = _errorProperties(deviceProperties);
      final String? errorMessage = _parseErrorMessage(errorProperties);
      if (errorMessage != null) {
        final int? code = _errorCode(errorProperties);
        // Error -13: iPhone is not connected. Xcode will continue when iPhone is connected.
        // This error is confusing since the device is not connected and maybe has not been connected
        // for a long time. Avoid showing it.
        if (code == -13 && errorMessage.contains('not connected')) {
          continue;
        }

        diagnostics.add(errorMessage);
      }
    }
    return diagnostics;
  }
}
