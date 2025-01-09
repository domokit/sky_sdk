// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:collection/collection.dart' show ListEquality, MapEquality;

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:meta/meta.dart';

import 'common.dart';

void main() {
  group('device', () {
    late Device device;

    setUp(() {
      FakeDevice.resetLog();
      device = FakeDevice(deviceId: 'fakeDeviceId');
    });

    tearDown(() {});

    group('cpu check', () {
      test('arm64', () async {
        FakeDevice.pretendArm64();
        final AndroidDevice androidDevice = device as AndroidDevice;
        expect(await androidDevice.isArm64(), isTrue);
        expectLog(<CommandArgs>[
          cmd(
            command: 'getprop',
            arguments: <String>[
              'ro.bootimage.build.fingerprint',
              ';',
              'getprop',
              'ro.build.version.release',
              ';',
              'getprop',
              'ro.build.version.sdk',
            ],
          ),
          cmd(command: 'getprop', arguments: <String>['ro.product.cpu.abi']),
        ]);
      });
    });

    group('isAwake/isAsleep', () {
      test('reads Awake', () async {
        FakeDevice.pretendAwake();
        expect(await device.isAwake(), isTrue);
        expect(await device.isAsleep(), isFalse);
      });

      test('reads Awake - samsung devices', () async {
        FakeDevice.pretendAwakeSamsung();
        expect(await device.isAwake(), isTrue);
        expect(await device.isAsleep(), isFalse);
      });

      test('reads Asleep', () async {
        FakeDevice.pretendAsleep();
        expect(await device.isAwake(), isFalse);
        expect(await device.isAsleep(), isTrue);
      });
    });

    group('togglePower', () {
      test('sends power event', () async {
        await device.togglePower();
        expectLog(<CommandArgs>[
          cmd(
            command: 'getprop',
            arguments: <String>[
              'ro.bootimage.build.fingerprint',
              ';',
              'getprop',
              'ro.build.version.release',
              ';',
              'getprop',
              'ro.build.version.sdk',
            ],
          ),
          cmd(command: 'input', arguments: <String>['keyevent', '26']),
        ]);
      });
    });

    group('wakeUp', () {
      test('when awake', () async {
        FakeDevice.pretendAwake();
        await device.wakeUp();
        expectLog(<CommandArgs>[
          cmd(
            command: 'getprop',
            arguments: <String>[
              'ro.bootimage.build.fingerprint',
              ';',
              'getprop',
              'ro.build.version.release',
              ';',
              'getprop',
              'ro.build.version.sdk',
            ],
          ),
          cmd(command: 'dumpsys', arguments: <String>['power']),
        ]);
      });

      test('when asleep', () async {
        FakeDevice.pretendAsleep();
        await device.wakeUp();
        expectLog(<CommandArgs>[
          cmd(
            command: 'getprop',
            arguments: <String>[
              'ro.bootimage.build.fingerprint',
              ';',
              'getprop',
              'ro.build.version.release',
              ';',
              'getprop',
              'ro.build.version.sdk',
            ],
          ),
          cmd(command: 'dumpsys', arguments: <String>['power']),
          cmd(command: 'input', arguments: <String>['keyevent', '26']),
        ]);
      });
    });

    group('sendToSleep', () {
      test('when asleep', () async {
        FakeDevice.pretendAsleep();
        await device.sendToSleep();
        expectLog(<CommandArgs>[
          cmd(
            command: 'getprop',
            arguments: <String>[
              'ro.bootimage.build.fingerprint',
              ';',
              'getprop',
              'ro.build.version.release',
              ';',
              'getprop',
              'ro.build.version.sdk',
            ],
          ),
          cmd(command: 'dumpsys', arguments: <String>['power']),
        ]);
      });

      test('when awake', () async {
        FakeDevice.pretendAwake();
        await device.sendToSleep();
        expectLog(<CommandArgs>[
          cmd(
            command: 'getprop',
            arguments: <String>[
              'ro.bootimage.build.fingerprint',
              ';',
              'getprop',
              'ro.build.version.release',
              ';',
              'getprop',
              'ro.build.version.sdk',
            ],
          ),
          cmd(command: 'dumpsys', arguments: <String>['power']),
          cmd(command: 'input', arguments: <String>['keyevent', '26']),
        ]);
      });
    });

    group('unlock', () {
      test('sends unlock event', () async {
        FakeDevice.pretendAwake();
        await device.unlock();
        expectLog(<CommandArgs>[
          cmd(
            command: 'getprop',
            arguments: <String>[
              'ro.bootimage.build.fingerprint',
              ';',
              'getprop',
              'ro.build.version.release',
              ';',
              'getprop',
              'ro.build.version.sdk',
            ],
          ),
          cmd(command: 'dumpsys', arguments: <String>['power']),
          cmd(command: 'input', arguments: <String>['keyevent', '82']),
        ]);
      });
    });

    group('adb', () {
      test('tap', () async {
        FakeDevice.resetLog();
        await device.tap(100, 200);
        expectLog(<CommandArgs>[
          cmd(command: 'input', arguments: <String>['tap', '100', '200']),
        ]);
      });

      test('awaitDevice', () async {
        FakeDevice.resetLog();
        // The expected value from `adb shell getprop sys.boot_completed`
        FakeDevice.output = '1';
        await device.awaitDevice();
        expectLog(<CommandArgs>[
          cmd(
            command: 'adb',
            environment: <String, String>{FakeDevice.canFailKey: 'false'},
            arguments: <String>['-s', device.deviceId, 'wait-for-device'],
          ),
          cmd(
            command: 'adb',
            environment: <String, String>{FakeDevice.canFailKey: 'false'},
            arguments: <String>['-s', device.deviceId, 'shell', 'getprop sys.boot_completed'],
          ),
        ]);
      });

      test('reboot', () async {
        FakeDevice.resetLog();
        await device.reboot();
        expectLog(<CommandArgs>[
          cmd(
            command: 'adb',
            environment: <String, String>{FakeDevice.canFailKey: 'false'},
            arguments: <String>['-s', device.deviceId, 'reboot'],
          ),
        ]);
      });

      test('clearLog', () async {
        FakeDevice.resetLog();
        await device.clearLogs();
        expectLog(<CommandArgs>[
          cmd(
            command: 'adb',
            environment: <String, String>{FakeDevice.canFailKey: 'true'},
            arguments: <String>['-s', device.deviceId, 'logcat', '-c'],
          ),
        ]);
      });

      test('startLoggingToSink calls adb', () async {
        FakeDevice.resetLog();
        await device.startLoggingToSink(IOSink(_MemoryIOSink()));
        expectLog(<CommandArgs>[
          cmd(
            command: 'adb',
            environment: <String, String>{FakeDevice.canFailKey: 'true'},
            arguments: <String>['-s', device.deviceId, 'logcat', '--clear'],
          ),
        ]);
      });
    });
  });
}

void expectLog(List<CommandArgs> log) {
  expect(FakeDevice.commandLog, log);
}

CommandArgs cmd({
  required String command,
  List<String>? arguments,
  Map<String, String>? environment,
}) {
  return CommandArgs(command: command, arguments: arguments, environment: environment);
}

@immutable
class CommandArgs {
  const CommandArgs({required this.command, this.arguments, this.environment});

  final String command;
  final List<String>? arguments;
  final Map<String, String>? environment;

  @override
  String toString() =>
      'CommandArgs(command: $command, arguments: $arguments, environment: $environment)';

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != CommandArgs) {
      return false;
    }
    return other is CommandArgs &&
        other.command == command &&
        const ListEquality<String>().equals(other.arguments, arguments) &&
        const MapEquality<String, String>().equals(other.environment, environment);
  }

  @override
  int get hashCode {
    return Object.hash(
      command,
      Object.hashAll(arguments ?? const <String>[]),
      Object.hashAllUnordered(environment?.keys ?? const <String>[]),
      Object.hashAllUnordered(environment?.values ?? const <String>[]),
    );
  }
}

class FakeDevice extends AndroidDevice {
  FakeDevice({required super.deviceId});

  static const String canFailKey = 'canFail';

  static String output = '';

  static List<CommandArgs> commandLog = <CommandArgs>[];

  static void resetLog() {
    commandLog.clear();
  }

  static void pretendAwake() {
    output = '''
      mWakefulness=Awake
    ''';
  }

  static void pretendAwakeSamsung() {
    output = '''
      getWakefulnessLocked()=Awake
    ''';
  }

  static void pretendAsleep() {
    output = '''
      mWakefulness=Asleep
    ''';
  }

  static void pretendArm64() {
    output = '''
      arm64
    ''';
  }

  @override
  Future<String> adb(
    List<String> arguments, {
    Map<String, String>? environment,
    bool silent = false,
    bool canFail = false,
  }) async {
    environment ??= <String, String>{};
    commandLog.add(
      CommandArgs(
        command: 'adb',
        // ignore: prefer_spread_collections
        arguments: <String>['-s', deviceId]..addAll(arguments),
        environment: environment..putIfAbsent('canFail', () => '$canFail'),
      ),
    );
    return output;
  }

  @override
  Future<String> shellEval(
    String command,
    List<String> arguments, {
    Map<String, String>? environment,
    bool silent = false,
  }) async {
    commandLog.add(CommandArgs(command: command, arguments: arguments, environment: environment));
    return output;
  }

  @override
  Future<void> shellExec(
    String command,
    List<String> arguments, {
    Map<String, String>? environment,
    bool silent = false,
  }) async {
    commandLog.add(CommandArgs(command: command, arguments: arguments, environment: environment));
  }
}

/// An IOSink that collects whatever is written to it.
/// Inspired by packages/flutter_tools/lib/src/base/net.dart
class _MemoryIOSink implements IOSink {
  @override
  Encoding encoding = utf8;

  final BytesBuilder writes = BytesBuilder(copy: false);

  @override
  void add(List<int> data) {
    writes.add(data);
  }

  @override
  Future<void> addStream(Stream<List<int>> stream) {
    final Completer<void> completer = Completer<void>();
    stream.listen(add).onDone(completer.complete);
    return completer.future;
  }

  @override
  void writeCharCode(int charCode) {
    add(<int>[charCode]);
  }

  @override
  void write(Object? obj) {
    add(encoding.encode('$obj'));
  }

  @override
  void writeln([Object? obj = '']) {
    add(encoding.encode('$obj\n'));
  }

  @override
  void writeAll(Iterable<dynamic> objects, [String separator = '']) {
    bool addSeparator = false;
    for (final dynamic object in objects) {
      if (addSeparator) {
        write(separator);
      }
      write(object);
      addSeparator = true;
    }
  }

  @override
  void addError(dynamic error, [StackTrace? stackTrace]) {
    throw UnimplementedError();
  }

  @override
  Future<void> get done => close();

  @override
  Future<void> close() async {}

  @override
  Future<void> flush() async {}
}
