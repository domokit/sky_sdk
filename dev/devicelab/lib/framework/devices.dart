// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;

import 'package:path/path.dart' as path;
import 'package:retry/retry.dart';

import 'utils.dart';

const String DeviceIdEnvName = 'FLUTTER_DEVICELAB_DEVICEID';

class DeviceException implements Exception {
  const DeviceException(this.message);

  final String message;

  @override
  String toString() => '$DeviceException: $message';
}

/// Gets the artifact path relative to the current directory.
String getArtifactPath() {
  return path.normalize(path.join(path.current, '../../bin/cache/artifacts'));
}

/// Return the item is in idList if find a match, otherwise return null
String? _findMatchId(List<String> idList, String idPattern) {
  String? candidate;
  idPattern = idPattern.toLowerCase();
  for (final String id in idList) {
    if (id.toLowerCase() == idPattern) {
      return id;
    }
    if (id.toLowerCase().startsWith(idPattern)) {
      candidate ??= id;
    }
  }
  return candidate;
}

/// The root of the API for controlling devices.
DeviceDiscovery get devices => DeviceDiscovery();

/// Device operating system the test is configured to test.
enum DeviceOperatingSystem {
  android,
  androidArm,
  androidArm64,
  fake,
  fuchsia,
  ios,
  linux,
  macos,
  windows,
}

/// Device OS to test on.
DeviceOperatingSystem deviceOperatingSystem = DeviceOperatingSystem.android;

/// Discovers available devices and chooses one to work with.
abstract class DeviceDiscovery {
  factory DeviceDiscovery() {
    switch (deviceOperatingSystem) {
      case DeviceOperatingSystem.android:
        return AndroidDeviceDiscovery();
      case DeviceOperatingSystem.androidArm:
        return AndroidDeviceDiscovery(cpu: AndroidCPU.arm);
      case DeviceOperatingSystem.androidArm64:
        return AndroidDeviceDiscovery(cpu: AndroidCPU.arm64);
      case DeviceOperatingSystem.ios:
        return IosDeviceDiscovery();
      case DeviceOperatingSystem.fuchsia:
        return FuchsiaDeviceDiscovery();
      case DeviceOperatingSystem.linux:
        return LinuxDeviceDiscovery();
      case DeviceOperatingSystem.macos:
        return MacosDeviceDiscovery();
      case DeviceOperatingSystem.windows:
        return WindowsDeviceDiscovery();
      case DeviceOperatingSystem.fake:
        print('Looking for fake devices! You should not see this in release builds.');
        return FakeDeviceDiscovery();
    }
  }

  /// Selects a device to work with, load-balancing between devices if more than
  /// one are available.
  ///
  /// Calling this method does not guarantee that the same device will be
  /// returned. For such behavior see [workingDevice].
  Future<void> chooseWorkingDevice();

  /// Selects a device to work with by device ID.
  Future<void> chooseWorkingDeviceById(String deviceId);

  /// A device to work with.
  ///
  /// Returns the same device when called repeatedly (unlike
  /// [chooseWorkingDevice]). This is useful when you need to perform multiple
  /// operations on one.
  Future<Device> get workingDevice;

  /// Lists all available devices' IDs.
  Future<List<String>> discoverDevices();

  /// Checks the health of the available devices.
  Future<Map<String, HealthCheckResult>> checkDevices();

  /// Prepares the system to run tasks.
  Future<void> performPreflightTasks();
}

/// A proxy for one specific device.
abstract class Device {
  // Const constructor so subclasses may be const.
  const Device();

  /// A unique device identifier.
  String get deviceId;

  /// Switch the device into fixed/regular performance mode.
  Future<void> toggleFixedPerformanceMode(bool enable) async {}

  /// Whether the device is awake.
  Future<bool> isAwake();

  /// Whether the device is asleep.
  Future<bool> isAsleep();

  /// Wake up the device if it is not awake.
  Future<void> wakeUp();

  /// Send the device to sleep mode.
  Future<void> sendToSleep();

  /// Emulates pressing the home button.
  Future<void> home();

  /// Emulates pressing the power button, toggling the device's on/off state.
  Future<void> togglePower();

  /// Unlocks the device.
  ///
  /// Assumes the device doesn't have a secure unlock pattern.
  Future<void> unlock();

  /// Attempt to reboot the phone, if possible.
  Future<void> reboot();

  /// Emulate a tap on the touch screen.
  Future<void> tap(int x, int y);

  /// Read memory statistics for a process.
  Future<Map<String, dynamic>> getMemoryStats(String packageName);

  /// Stream the system log from the device.
  ///
  /// Flutter applications' `print` statements end up in this log
  /// with some prefix.
  Stream<String> get logcat;

  /// Clears the device logs.
  ///
  /// This is important because benchmarks tests rely on the logs produced by
  /// the flutter run command.
  ///
  /// On Android, those logs may contain logs from previous test.
  Future<void> clearLogs();

  /// Whether this device supports calls to [startLoggingToSink]
  /// and [stopLoggingToSink].
  bool get canStreamLogs => false;

  /// Starts logging to an [IOSink].
  ///
  /// If `clear` is set to true, the log will be cleared before starting. This
  /// is not supported on all platforms.
  Future<void> startLoggingToSink(IOSink sink, {bool clear = true}) {
    throw UnimplementedError();
  }

  /// Stops logging that was started by [startLoggingToSink].
  Future<void> stopLoggingToSink() {
    throw UnimplementedError();
  }

  /// Stop a process.
  Future<void> stop(String packageName);

  /// Wait for the device to become ready.
  Future<void> awaitDevice();

  Future<void> uninstallApp() async {
    await flutter('install', options: <String>['--uninstall-only', '-d', deviceId]);

    await Future<void>.delayed(const Duration(seconds: 2));

    await awaitDevice();
  }

  @override
  String toString() {
    return 'device: $deviceId';
  }
}

enum AndroidCPU { arm, arm64 }

class AndroidDeviceDiscovery implements DeviceDiscovery {
  factory AndroidDeviceDiscovery({AndroidCPU? cpu}) {
    return _instance ??= AndroidDeviceDiscovery._(cpu);
  }

  AndroidDeviceDiscovery._(this.cpu);

  final AndroidCPU? cpu;

  // Parses information about a device. Example:
  //
  // 015d172c98400a03       device usb:340787200X product:nakasi model:Nexus_7 device:grouper
  static final RegExp _kDeviceRegex = RegExp(r'^(\S+)\s+(\S+)(.*)');

  static AndroidDeviceDiscovery? _instance;

  AndroidDevice? _workingDevice;

  @override
  Future<AndroidDevice> get workingDevice async {
    if (_workingDevice == null) {
      if (Platform.environment.containsKey(DeviceIdEnvName)) {
        final String deviceId = Platform.environment[DeviceIdEnvName]!;
        await chooseWorkingDeviceById(deviceId);
        return _workingDevice!;
      }
      await chooseWorkingDevice();
    }

    return _workingDevice!;
  }

  Future<bool> _matchesCPURequirement(AndroidDevice device) async {
    return switch (cpu) {
      null => Future<bool>.value(true),
      AndroidCPU.arm64 => device.isArm64(),
      AndroidCPU.arm => device.isArm(),
    };
  }

  /// Picks a random Android device out of connected devices and sets it as
  /// [workingDevice].
  @override
  Future<void> chooseWorkingDevice() async {
    final List<AndroidDevice> allDevices =
        (await discoverDevices())
            .map<AndroidDevice>((String id) => AndroidDevice(deviceId: id))
            .toList();

    if (allDevices.isEmpty) {
      throw const DeviceException('No Android devices detected');
    }

    if (cpu != null) {
      for (final AndroidDevice device in allDevices) {
        if (await _matchesCPURequirement(device)) {
          _workingDevice = device;
          break;
        }
      }
    } else {
      // TODO(yjbanov): filter out and warn about those with low battery level
      _workingDevice = allDevices[math.Random().nextInt(allDevices.length)];
    }

    if (_workingDevice == null) {
      throw const DeviceException('Cannot find a suitable Android device');
    }

    print('Device chosen: $_workingDevice');
  }

  @override
  Future<void> chooseWorkingDeviceById(String deviceId) async {
    final String? matchedId = _findMatchId(await discoverDevices(), deviceId);
    if (matchedId != null) {
      _workingDevice = AndroidDevice(deviceId: matchedId);
      if (cpu != null) {
        if (!await _matchesCPURequirement(_workingDevice!)) {
          throw DeviceException(
            'The selected device $matchedId does not match the cpu requirement',
          );
        }
      }
      print('Choose device by ID: $matchedId');
      return;
    }
    throw DeviceException(
      'Device with ID $deviceId is not found for operating system: '
      '$deviceOperatingSystem',
    );
  }

  @override
  Future<List<String>> discoverDevices() async {
    final List<String> output = (await eval(adbPath, <String>['devices', '-l'])).trim().split('\n');
    final List<String> results = <String>[];
    for (final String line in output) {
      // Skip lines like: * daemon started successfully *
      if (line.startsWith('* daemon ')) {
        continue;
      }

      if (line.startsWith('List of devices')) {
        continue;
      }

      if (_kDeviceRegex.hasMatch(line)) {
        final Match match = _kDeviceRegex.firstMatch(line)!;

        final String deviceID = match[1]!;
        final String deviceState = match[2]!;

        if (!const <String>['unauthorized', 'offline'].contains(deviceState)) {
          results.add(deviceID);
        }
      } else {
        throw FormatException('Failed to parse device from adb output: "$line"');
      }
    }

    return results;
  }

  @override
  Future<Map<String, HealthCheckResult>> checkDevices() async {
    final Map<String, HealthCheckResult> results = <String, HealthCheckResult>{};
    for (final String deviceId in await discoverDevices()) {
      try {
        final AndroidDevice device = AndroidDevice(deviceId: deviceId);
        // Just a smoke test that we can read wakefulness state
        // TODO(yjbanov): check battery level
        await device._getWakefulness();
        results['android-device-$deviceId'] = HealthCheckResult.success();
      } on Exception catch (e, s) {
        results['android-device-$deviceId'] = HealthCheckResult.error(e, s);
      }
    }
    return results;
  }

  @override
  Future<void> performPreflightTasks() async {
    // Kills the `adb` server causing it to start a new instance upon next
    // command.
    //
    // Restarting `adb` helps with keeping device connections alive. When `adb`
    // runs non-stop for too long it loses connections to devices. There may be
    // a better method, but so far that's the best one I've found.
    await exec(adbPath, <String>['kill-server']);
  }
}

class LinuxDeviceDiscovery implements DeviceDiscovery {
  factory LinuxDeviceDiscovery() {
    return _instance ??= LinuxDeviceDiscovery._();
  }

  LinuxDeviceDiscovery._();

  static LinuxDeviceDiscovery? _instance;

  static const LinuxDevice _device = LinuxDevice();

  @override
  Future<Map<String, HealthCheckResult>> checkDevices() async {
    return <String, HealthCheckResult>{};
  }

  @override
  Future<void> chooseWorkingDevice() async {}

  @override
  Future<void> chooseWorkingDeviceById(String deviceId) async {}

  @override
  Future<List<String>> discoverDevices() async {
    return <String>['linux'];
  }

  @override
  Future<void> performPreflightTasks() async {}

  @override
  Future<Device> get workingDevice async => _device;
}

class MacosDeviceDiscovery implements DeviceDiscovery {
  factory MacosDeviceDiscovery() {
    return _instance ??= MacosDeviceDiscovery._();
  }

  MacosDeviceDiscovery._();

  static MacosDeviceDiscovery? _instance;

  static const MacosDevice _device = MacosDevice();

  @override
  Future<Map<String, HealthCheckResult>> checkDevices() async {
    return <String, HealthCheckResult>{};
  }

  @override
  Future<void> chooseWorkingDevice() async {}

  @override
  Future<void> chooseWorkingDeviceById(String deviceId) async {}

  @override
  Future<List<String>> discoverDevices() async {
    return <String>['macos'];
  }

  @override
  Future<void> performPreflightTasks() async {}

  @override
  Future<Device> get workingDevice async => _device;
}

class WindowsDeviceDiscovery implements DeviceDiscovery {
  factory WindowsDeviceDiscovery() {
    return _instance ??= WindowsDeviceDiscovery._();
  }

  WindowsDeviceDiscovery._();

  static WindowsDeviceDiscovery? _instance;

  static const WindowsDevice _device = WindowsDevice();

  @override
  Future<Map<String, HealthCheckResult>> checkDevices() async {
    return <String, HealthCheckResult>{};
  }

  @override
  Future<void> chooseWorkingDevice() async {}

  @override
  Future<void> chooseWorkingDeviceById(String deviceId) async {}

  @override
  Future<List<String>> discoverDevices() async {
    return <String>['windows'];
  }

  @override
  Future<void> performPreflightTasks() async {}

  @override
  Future<Device> get workingDevice async => _device;
}

class FuchsiaDeviceDiscovery implements DeviceDiscovery {
  factory FuchsiaDeviceDiscovery() {
    return _instance ??= FuchsiaDeviceDiscovery._();
  }

  FuchsiaDeviceDiscovery._();

  static FuchsiaDeviceDiscovery? _instance;

  FuchsiaDevice? _workingDevice;

  String get _ffx {
    final String ffx = path.join(getArtifactPath(), 'fuchsia', 'tools', 'x64', 'ffx');
    if (!File(ffx).existsSync()) {
      throw FileSystemException("Couldn't find ffx at location $ffx");
    }
    return ffx;
  }

  @override
  Future<FuchsiaDevice> get workingDevice async {
    if (_workingDevice == null) {
      if (Platform.environment.containsKey(DeviceIdEnvName)) {
        final String deviceId = Platform.environment[DeviceIdEnvName]!;
        await chooseWorkingDeviceById(deviceId);
        return _workingDevice!;
      }
      await chooseWorkingDevice();
    }
    return _workingDevice!;
  }

  /// Picks the first connected Fuchsia device.
  @override
  Future<void> chooseWorkingDevice() async {
    final List<FuchsiaDevice> allDevices =
        (await discoverDevices())
            .map<FuchsiaDevice>((String id) => FuchsiaDevice(deviceId: id))
            .toList();

    if (allDevices.isEmpty) {
      throw const DeviceException('No Fuchsia devices detected');
    }
    _workingDevice = allDevices.first;
    print('Device chosen: $_workingDevice');
  }

  @override
  Future<void> chooseWorkingDeviceById(String deviceId) async {
    final String? matchedId = _findMatchId(await discoverDevices(), deviceId);
    if (matchedId != null) {
      _workingDevice = FuchsiaDevice(deviceId: matchedId);
      print('Choose device by ID: $matchedId');
      return;
    }
    throw DeviceException(
      'Device with ID $deviceId is not found for operating system: '
      '$deviceOperatingSystem',
    );
  }

  @override
  Future<List<String>> discoverDevices() async {
    final List<String> output = (await eval(_ffx, <String>[
      'target',
      'list',
      '-f',
      's',
    ])).trim().split('\n');

    final List<String> devices = <String>[];
    for (final String line in output) {
      final List<String> parts = line.split(' ');
      assert(parts.length == 2);
      devices.add(parts.last); // The device id.
    }
    return devices;
  }

  @override
  Future<Map<String, HealthCheckResult>> checkDevices() async {
    final Map<String, HealthCheckResult> results = <String, HealthCheckResult>{};
    for (final String deviceId in await discoverDevices()) {
      try {
        final int resolveResult = await exec(_ffx, <String>['target', 'list', '-f', 'a', deviceId]);
        if (resolveResult == 0) {
          results['fuchsia-device-$deviceId'] = HealthCheckResult.success();
        } else {
          results['fuchsia-device-$deviceId'] = HealthCheckResult.failure(
            'Cannot resolve device $deviceId',
          );
        }
      } on Exception catch (error, stacktrace) {
        results['fuchsia-device-$deviceId'] = HealthCheckResult.error(error, stacktrace);
      }
    }
    return results;
  }

  @override
  Future<void> performPreflightTasks() async {}
}

class AndroidDevice extends Device {
  AndroidDevice({required this.deviceId}) {
    _updateDeviceInfo();
  }

  @override
  final String deviceId;
  String deviceInfo = '';
  int apiLevel = 0;

  @override
  Future<void> toggleFixedPerformanceMode(bool enable) async {
    await shellExec('cmd', <String>[
      'power',
      'set-fixed-performance-mode-enabled',
      if (enable) 'true' else 'false',
    ]);
  }

  /// Whether the device is awake.
  @override
  Future<bool> isAwake() async {
    return await _getWakefulness() == 'Awake';
  }

  /// Whether the device is asleep.
  @override
  Future<bool> isAsleep() async {
    return await _getWakefulness() == 'Asleep';
  }

  /// Wake up the device if it is not awake using [togglePower].
  @override
  Future<void> wakeUp() async {
    if (!(await isAwake())) {
      await togglePower();
    }
  }

  /// Send the device to sleep mode if it is not asleep using [togglePower].
  @override
  Future<void> sendToSleep() async {
    if (!(await isAsleep())) {
      await togglePower();
    }
  }

  /// Sends `KEYCODE_HOME` (3), which causes the device to go to the home screen.
  @override
  Future<void> home() async {
    await shellExec('input', const <String>['keyevent', '3']);
  }

  /// Sends `KEYCODE_POWER` (26), which causes the device to toggle its mode
  /// between awake and asleep.
  @override
  Future<void> togglePower() async {
    await shellExec('input', const <String>['keyevent', '26']);
  }

  /// Unlocks the device by sending `KEYCODE_MENU` (82).
  ///
  /// This only works when the device doesn't have a secure unlock pattern.
  @override
  Future<void> unlock() async {
    await wakeUp();
    await shellExec('input', const <String>['keyevent', '82']);
  }

  @override
  Future<void> tap(int x, int y) async {
    await shellExec('input', <String>['tap', '$x', '$y']);
  }

  /// Retrieves device's wakefulness state.
  ///
  /// See: https://android.googlesource.com/platform/frameworks/base/+/main/core/java/android/os/PowerManagerInternal.java
  Future<String> _getWakefulness() async {
    final String powerInfo = await shellEval('dumpsys', <String>['power']);
    // A motoG4 phone returns `mWakefulness=Awake`.
    // A Samsung phone returns `getWakefullnessLocked()=Awake`.
    final RegExp wakefulnessRegexp = RegExp(r'.*(mWakefulness=|getWakefulnessLocked\(\)=).*');
    final String wakefulness = grep(wakefulnessRegexp, from: powerInfo).single.split('=')[1].trim();
    return wakefulness;
  }

  Future<bool> isArm64() async {
    final String cpuInfo = await shellEval('getprop', const <String>['ro.product.cpu.abi']);
    return cpuInfo.contains('arm64');
  }

  Future<bool> isArm() async {
    final String cpuInfo = await shellEval('getprop', const <String>['ro.product.cpu.abi']);
    return cpuInfo.contains('armeabi');
  }

  Future<void> _updateDeviceInfo() async {
    String info;
    try {
      info = await shellEval('getprop', <String>[
        'ro.bootimage.build.fingerprint',
        ';',
        'getprop',
        'ro.build.version.release',
        ';',
        'getprop',
        'ro.build.version.sdk',
      ], silent: true);
    } on IOException {
      info = '';
    }
    final List<String> list = info.split('\n');
    if (list.length == 3) {
      apiLevel = int.parse(list[2]);
      deviceInfo = 'fingerprint: ${list[0]} os: ${list[1]}  api-level: $apiLevel';
    } else {
      apiLevel = 0;
      deviceInfo = '';
    }
  }

  /// Executes [command] on `adb shell`.
  Future<void> shellExec(
    String command,
    List<String> arguments, {
    Map<String, String>? environment,
    bool silent = false,
  }) async {
    await adb(<String>['shell', command, ...arguments], environment: environment, silent: silent);
  }

  /// Executes [command] on `adb shell` and returns its standard output as a [String].
  Future<String> shellEval(
    String command,
    List<String> arguments, {
    Map<String, String>? environment,
    bool silent = false,
  }) {
    return adb(<String>['shell', command, ...arguments], environment: environment, silent: silent);
  }

  /// Runs `adb` with the given [arguments], selecting this device.
  Future<String> adb(
    List<String> arguments, {
    Map<String, String>? environment,
    bool silent = false,
    bool canFail = false, // as in, whether failures are ok. False means that they are fatal.
  }) {
    return eval(
      adbPath,
      <String>['-s', deviceId, ...arguments],
      environment: environment,
      printStdout: !silent,
      printStderr: !silent,
      canFail: canFail,
    );
  }

  @override
  Future<Map<String, dynamic>> getMemoryStats(String packageName) async {
    final String meminfo = await shellEval('dumpsys', <String>['meminfo', packageName]);
    final Match? match = RegExp(r'TOTAL\s+(\d+)').firstMatch(meminfo);
    assert(match != null, 'could not parse dumpsys meminfo output');
    return <String, dynamic>{'total_kb': int.parse(match!.group(1)!)};
  }

  @override
  bool get canStreamLogs => true;

  bool _abortedLogging = false;
  Process? _loggingProcess;

  @override
  Future<void> startLoggingToSink(IOSink sink, {bool clear = true}) async {
    if (clear) {
      await adb(<String>['logcat', '--clear'], silent: true, canFail: true);
    }
    _loggingProcess = await startProcess(
      adbPath,
      // Catch the whole log.
      <String>['-s', deviceId, 'logcat'],
    );
    _loggingProcess!.stdout.transform<String>(const Utf8Decoder(allowMalformed: true)).listen((
      String line,
    ) {
      sink.write(line);
    });
    _loggingProcess!.stderr.transform<String>(const Utf8Decoder(allowMalformed: true)).listen((
      String line,
    ) {
      sink.write(line);
    });
    unawaited(
      _loggingProcess!.exitCode.then<void>((int exitCode) {
        if (!_abortedLogging) {
          sink.writeln('adb logcat failed with exit code $exitCode.\n');
        }
      }),
    );
  }

  @override
  Future<void> stopLoggingToSink() async {
    if (_loggingProcess != null) {
      _abortedLogging = true;
      _loggingProcess!.kill();
      await _loggingProcess!.exitCode;
    }
  }

  @override
  Future<void> clearLogs() {
    return adb(<String>['logcat', '-c'], canFail: true);
  }

  @override
  Stream<String> get logcat {
    final Completer<void> stdoutDone = Completer<void>();
    final Completer<void> stderrDone = Completer<void>();
    final Completer<void> processDone = Completer<void>();
    final Completer<void> abort = Completer<void>();
    bool aborted = false;
    late final StreamController<String> stream;
    stream = StreamController<String>(
      onListen: () async {
        await clearLogs();
        final Process process = await startProcess(
          adbPath,
          // Make logcat less chatty by filtering down to just ActivityManager
          // (to let us know when app starts), flutter (needed by tests to see
          // log output), and fatal messages (hopefully catches tombstones).
          // For local testing, this can just be:
          //   <String>['-s', deviceId, 'logcat']
          // to view the whole log, or just run logcat alongside this.
          <String>['-s', deviceId, 'logcat', 'ActivityManager:I', 'flutter:V', '*:F'],
        );
        process.stdout
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen(
              (String line) {
                print('adb logcat: $line');
                if (!stream.isClosed) {
                  stream.sink.add(line);
                }
              },
              onDone: () {
                stdoutDone.complete();
              },
            );
        process.stderr
            .transform<String>(utf8.decoder)
            .transform<String>(const LineSplitter())
            .listen(
              (String line) {
                print('adb logcat stderr: $line');
              },
              onDone: () {
                stderrDone.complete();
              },
            );
        unawaited(
          process.exitCode.then<void>((int exitCode) {
            print('adb logcat process terminated with exit code $exitCode');
            if (!aborted) {
              stream.addError(BuildFailedError('adb logcat failed with exit code $exitCode.\n'));
              processDone.complete();
            }
          }),
        );
        await Future.any<dynamic>(<Future<dynamic>>[
          Future.wait<void>(<Future<void>>[
            stdoutDone.future,
            stderrDone.future,
            processDone.future,
          ]),
          abort.future,
        ]);
        aborted = true;
        print('terminating adb logcat');
        process.kill();
        print('closing logcat stream');
        await stream.close();
      },
      onCancel: () {
        if (!aborted) {
          print('adb logcat aborted');
          aborted = true;
          abort.complete();
        }
      },
    );
    return stream.stream;
  }

  @override
  Future<void> stop(String packageName) async {
    return shellExec('am', <String>['force-stop', packageName]);
  }

  @override
  String toString() {
    return '$deviceId $deviceInfo';
  }

  @override
  Future<void> reboot() {
    return adb(<String>['reboot']);
  }

  @override
  Future<void> awaitDevice() async {
    print('Waiting for device.');
    final String waitOut = await adb(<String>['wait-for-device']);
    print(waitOut);
    const RetryOptions retryOptions = RetryOptions(
      delayFactor: Duration(seconds: 1),
      maxAttempts: 10,
      maxDelay: Duration(minutes: 1),
    );
    await retryOptions.retry(() async {
      final String adbShellOut = await adb(<String>['shell', 'getprop sys.boot_completed']);
      if (adbShellOut != '1') {
        print('Device not ready.');
        print(adbShellOut);
        throw const DeviceException('Phone not ready.');
      }
    }, retryIf: (Exception e) => e is DeviceException);
    print('Done waiting for device.');
  }
}

class IosDeviceDiscovery implements DeviceDiscovery {
  factory IosDeviceDiscovery() {
    return _instance ??= IosDeviceDiscovery._();
  }

  IosDeviceDiscovery._();

  static IosDeviceDiscovery? _instance;

  IosDevice? _workingDevice;

  @override
  Future<IosDevice> get workingDevice async {
    if (_workingDevice == null) {
      if (Platform.environment.containsKey(DeviceIdEnvName)) {
        final String deviceId = Platform.environment[DeviceIdEnvName]!;
        await chooseWorkingDeviceById(deviceId);
        return _workingDevice!;
      }
      await chooseWorkingDevice();
    }

    return _workingDevice!;
  }

  /// Picks a random iOS device out of connected devices and sets it as
  /// [workingDevice].
  @override
  Future<void> chooseWorkingDevice() async {
    final List<IosDevice> allDevices =
        (await discoverDevices()).map<IosDevice>((String id) => IosDevice(deviceId: id)).toList();

    if (allDevices.isEmpty) {
      throw const DeviceException('No iOS devices detected');
    }

    // TODO(yjbanov): filter out and warn about those with low battery level
    _workingDevice = allDevices[math.Random().nextInt(allDevices.length)];
    print('Device chosen: $_workingDevice');
  }

  @override
  Future<void> chooseWorkingDeviceById(String deviceId) async {
    final String? matchedId = _findMatchId(await discoverDevices(), deviceId);
    if (matchedId != null) {
      _workingDevice = IosDevice(deviceId: matchedId);
      print('Choose device by ID: $matchedId');
      return;
    }
    throw DeviceException(
      'Device with ID $deviceId is not found for operating system: '
      '$deviceOperatingSystem',
    );
  }

  @override
  Future<List<String>> discoverDevices() async {
    final List<dynamic> results =
        json.decode(
              await eval(path.join(flutterDirectory.path, 'bin', 'flutter'), <String>[
                'devices',
                '--machine',
                '--suppress-analytics',
                '--device-timeout',
                '5',
              ]),
            )
            as List<dynamic>;

    // [
    //   {
    //     "name": "Flutter's iPhone",
    //     "id": "00008020-00017DA80CC1002E",
    //     "isSupported": true,
    //     "targetPlatform": "ios",
    //     "emulator": false,
    //     "sdk": "iOS 13.2",
    //     "capabilities": {
    //       "hotReload": true,
    //       "hotRestart": true,
    //       "screenshot": true,
    //       "fastStart": false,
    //       "flutterExit": true,
    //       "hardwareRendering": false,
    //       "startPaused": false
    //     }
    //   }
    // ]

    final List<String> deviceIds = <String>[];

    for (final dynamic result in results) {
      final Map<String, dynamic> device = result as Map<String, dynamic>;
      if (device['targetPlatform'] == 'ios' &&
          device['id'] != null &&
          device['emulator'] != true &&
          device['isSupported'] == true) {
        deviceIds.add(device['id'] as String);
      }
    }

    if (deviceIds.isEmpty) {
      throw const DeviceException('No connected physical iOS devices found.');
    }
    return deviceIds;
  }

  @override
  Future<Map<String, HealthCheckResult>> checkDevices() async {
    final Map<String, HealthCheckResult> results = <String, HealthCheckResult>{};
    for (final String deviceId in await discoverDevices()) {
      // TODO(ianh): do a more meaningful connectivity check than just recording the ID
      results['ios-device-$deviceId'] = HealthCheckResult.success();
    }
    return results;
  }

  @override
  Future<void> performPreflightTasks() async {
    // Currently we do not have preflight tasks for iOS.
  }
}

/// iOS device.
class IosDevice extends Device {
  IosDevice({required this.deviceId});

  @override
  final String deviceId;

  String get idevicesyslogPath {
    return path.join(
      flutterDirectory.path,
      'bin',
      'cache',
      'artifacts',
      'libimobiledevice',
      'idevicesyslog',
    );
  }

  String get dyldLibraryPath {
    final List<String> dylibsPaths = <String>[
      path.join(flutterDirectory.path, 'bin', 'cache', 'artifacts', 'libimobiledevice'),
      path.join(flutterDirectory.path, 'bin', 'cache', 'artifacts', 'openssl'),
      path.join(flutterDirectory.path, 'bin', 'cache', 'artifacts', 'usbmuxd'),
      path.join(flutterDirectory.path, 'bin', 'cache', 'artifacts', 'libplist'),
    ];
    return dylibsPaths.join(':');
  }

  @override
  bool get canStreamLogs => true;

  bool _abortedLogging = false;
  Process? _loggingProcess;

  @override
  Future<void> startLoggingToSink(IOSink sink, {bool clear = true}) async {
    // Clear is not supported.
    _loggingProcess = await startProcess(
      idevicesyslogPath,
      <String>['-u', deviceId, '--quiet'],
      environment: <String, String>{'DYLD_LIBRARY_PATH': dyldLibraryPath},
    );
    _loggingProcess!.stdout.transform<String>(const Utf8Decoder(allowMalformed: true)).listen((
      String line,
    ) {
      sink.write(line);
    });
    _loggingProcess!.stderr.transform<String>(const Utf8Decoder(allowMalformed: true)).listen((
      String line,
    ) {
      sink.write(line);
    });
    unawaited(
      _loggingProcess!.exitCode.then<void>((int exitCode) {
        if (!_abortedLogging) {
          sink.writeln('idevicesyslog failed with exit code $exitCode.\n');
        }
      }),
    );
  }

  @override
  Future<void> stopLoggingToSink() async {
    if (_loggingProcess != null) {
      _abortedLogging = true;
      _loggingProcess!.kill();
      await _loggingProcess!.exitCode;
    }
  }

  // The methods below are stubs for now. They will need to be expanded.
  // We currently do not have a way to lock/unlock iOS devices. So we assume the
  // devices are already unlocked. For now we'll just keep them at minimum
  // screen brightness so they don't drain battery too fast.

  @override
  Future<bool> isAwake() async => true;

  @override
  Future<bool> isAsleep() async => false;

  @override
  Future<void> wakeUp() async {}

  @override
  Future<void> sendToSleep() async {}

  @override
  Future<void> home() async {}

  @override
  Future<void> togglePower() async {}

  @override
  Future<void> unlock() async {}

  @override
  Future<void> tap(int x, int y) async {
    throw UnimplementedError();
  }

  @override
  Future<Map<String, dynamic>> getMemoryStats(String packageName) async {
    throw UnimplementedError();
  }

  @override
  Stream<String> get logcat {
    throw UnimplementedError();
  }

  @override
  Future<void> clearLogs() async {}

  @override
  Future<void> stop(String packageName) async {}

  @override
  Future<void> reboot() {
    return Process.run('idevicediagnostics', <String>['restart', '-u', deviceId]);
  }

  @override
  Future<void> awaitDevice() async {}
}

class LinuxDevice extends Device {
  const LinuxDevice();

  @override
  String get deviceId => 'linux';

  @override
  Future<Map<String, dynamic>> getMemoryStats(String packageName) async {
    return <String, dynamic>{};
  }

  @override
  Future<void> home() async {}

  @override
  Future<bool> isAsleep() async {
    return false;
  }

  @override
  Future<bool> isAwake() async {
    return true;
  }

  @override
  Stream<String> get logcat => const Stream<String>.empty();

  @override
  Future<void> clearLogs() async {}

  @override
  Future<void> reboot() async {}

  @override
  Future<void> sendToSleep() async {}

  @override
  Future<void> stop(String packageName) async {}

  @override
  Future<void> tap(int x, int y) async {}

  @override
  Future<void> togglePower() async {}

  @override
  Future<void> unlock() async {}

  @override
  Future<void> wakeUp() async {}

  @override
  Future<void> awaitDevice() async {}
}

class MacosDevice extends Device {
  const MacosDevice();

  @override
  String get deviceId => 'macos';

  @override
  Future<Map<String, dynamic>> getMemoryStats(String packageName) async {
    return <String, dynamic>{};
  }

  @override
  Future<void> home() async {}

  @override
  Future<bool> isAsleep() async {
    return false;
  }

  @override
  Future<bool> isAwake() async {
    return true;
  }

  @override
  Stream<String> get logcat => const Stream<String>.empty();

  @override
  Future<void> clearLogs() async {}

  @override
  Future<void> reboot() async {}

  @override
  Future<void> sendToSleep() async {}

  @override
  Future<void> stop(String packageName) async {}

  @override
  Future<void> tap(int x, int y) async {}

  @override
  Future<void> togglePower() async {}

  @override
  Future<void> unlock() async {}

  @override
  Future<void> wakeUp() async {}

  @override
  Future<void> awaitDevice() async {}
}

class WindowsDevice extends Device {
  const WindowsDevice();

  @override
  String get deviceId => 'windows';

  @override
  Future<Map<String, dynamic>> getMemoryStats(String packageName) async {
    return <String, dynamic>{};
  }

  @override
  Future<void> home() async {}

  @override
  Future<bool> isAsleep() async {
    return false;
  }

  @override
  Future<bool> isAwake() async {
    return true;
  }

  @override
  Stream<String> get logcat => const Stream<String>.empty();

  @override
  Future<void> clearLogs() async {}

  @override
  Future<void> reboot() async {}

  @override
  Future<void> sendToSleep() async {}

  @override
  Future<void> stop(String packageName) async {}

  @override
  Future<void> tap(int x, int y) async {}

  @override
  Future<void> togglePower() async {}

  @override
  Future<void> unlock() async {}

  @override
  Future<void> wakeUp() async {}

  @override
  Future<void> awaitDevice() async {}
}

/// Fuchsia device.
class FuchsiaDevice extends Device {
  const FuchsiaDevice({required this.deviceId});

  @override
  final String deviceId;

  // TODO(egarciad): Implement these for Fuchsia.
  @override
  Future<bool> isAwake() async => true;

  @override
  Future<bool> isAsleep() async => false;

  @override
  Future<void> wakeUp() async {}

  @override
  Future<void> sendToSleep() async {}

  @override
  Future<void> home() async {}

  @override
  Future<void> togglePower() async {}

  @override
  Future<void> unlock() async {}

  @override
  Future<void> tap(int x, int y) async {}

  @override
  Future<void> stop(String packageName) async {}

  @override
  Future<Map<String, dynamic>> getMemoryStats(String packageName) async {
    throw UnimplementedError();
  }

  @override
  Stream<String> get logcat {
    throw UnimplementedError();
  }

  @override
  Future<void> clearLogs() async {}

  @override
  Future<void> reboot() async {
    // Unsupported.
  }

  @override
  Future<void> awaitDevice() async {}
}

/// Path to the `adb` executable.
String get adbPath {
  final String? androidHome =
      Platform.environment['ANDROID_HOME'] ?? Platform.environment['ANDROID_SDK_ROOT'];

  if (androidHome == null) {
    throw const DeviceException(
      'The ANDROID_HOME environment variable is '
      'missing. The variable must point to the Android '
      'SDK directory containing platform-tools.',
    );
  }

  final String adbPath = path.join(androidHome, 'platform-tools/adb');

  if (!canRun(adbPath)) {
    throw DeviceException('adb not found at: $adbPath');
  }

  return path.absolute(adbPath);
}

class FakeDevice extends Device {
  const FakeDevice({required this.deviceId});

  @override
  final String deviceId;

  @override
  Future<bool> isAwake() async => true;

  @override
  Future<bool> isAsleep() async => false;

  @override
  Future<void> wakeUp() async {}

  @override
  Future<void> sendToSleep() async {}

  @override
  Future<void> home() async {}

  @override
  Future<void> togglePower() async {}

  @override
  Future<void> unlock() async {}

  @override
  Future<void> tap(int x, int y) async {
    throw UnimplementedError();
  }

  @override
  Future<Map<String, dynamic>> getMemoryStats(String packageName) async {
    throw UnimplementedError();
  }

  @override
  Stream<String> get logcat {
    throw UnimplementedError();
  }

  @override
  Future<void> clearLogs() async {}

  @override
  Future<void> stop(String packageName) async {}

  @override
  Future<void> reboot() async {
    // Unsupported.
  }

  @override
  Future<void> awaitDevice() async {}
}

class FakeDeviceDiscovery implements DeviceDiscovery {
  factory FakeDeviceDiscovery() {
    return _instance ??= FakeDeviceDiscovery._();
  }

  FakeDeviceDiscovery._();

  static FakeDeviceDiscovery? _instance;

  FakeDevice? _workingDevice;

  @override
  Future<FakeDevice> get workingDevice async {
    if (_workingDevice == null) {
      if (Platform.environment.containsKey(DeviceIdEnvName)) {
        final String deviceId = Platform.environment[DeviceIdEnvName]!;
        await chooseWorkingDeviceById(deviceId);
        return _workingDevice!;
      }
      await chooseWorkingDevice();
    }

    return _workingDevice!;
  }

  /// The Fake is only available for by ID device discovery.
  @override
  Future<void> chooseWorkingDevice() async {
    throw const DeviceException('No fake devices detected');
  }

  @override
  Future<void> chooseWorkingDeviceById(String deviceId) async {
    final String? matchedId = _findMatchId(await discoverDevices(), deviceId);
    if (matchedId != null) {
      _workingDevice = FakeDevice(deviceId: matchedId);
      print('Choose device by ID: $matchedId');
      return;
    }
    throw DeviceException(
      'Device with ID $deviceId is not found for operating system: '
      '$deviceOperatingSystem',
    );
  }

  @override
  Future<List<String>> discoverDevices() async {
    return <String>['FAKE_SUCCESS', 'THIS_IS_A_FAKE'];
  }

  @override
  Future<Map<String, HealthCheckResult>> checkDevices() async {
    final Map<String, HealthCheckResult> results = <String, HealthCheckResult>{};
    for (final String deviceId in await discoverDevices()) {
      results['fake-device-$deviceId'] = HealthCheckResult.success();
    }
    return results;
  }

  @override
  Future<void> performPreflightTasks() async {}
}
