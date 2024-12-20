// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/custom_devices/custom_device_config.dart';

void writeCustomDevicesConfigFile(
  Directory dir, {
  List<CustomDeviceConfig>? configs,
  dynamic json,
}) {
  dir.createSync(recursive: true);

  final File file = dir.childFile('.flutter_custom_devices.json');
  file.writeAsStringSync(
    jsonEncode(<String, dynamic>{
      'custom-devices':
          configs != null
              ? configs.map<dynamic>((CustomDeviceConfig c) => c.toJson()).toList()
              : json,
    }),
  );
}

final CustomDeviceConfig testConfig = CustomDeviceConfig(
  id: 'testid',
  label: 'testlabel',
  sdkNameAndVersion: 'testsdknameandversion',
  enabled: true,
  pingCommand: const <String>['testping'],
  pingSuccessRegex: RegExp('testpingsuccess'),
  postBuildCommand: const <String>['testpostbuild'],
  installCommand: const <String>['testinstall'],
  uninstallCommand: const <String>['testuninstall'],
  runDebugCommand: const <String>['testrundebug'],
  forwardPortCommand: const <String>['testforwardport'],
  forwardPortSuccessRegex: RegExp('testforwardportsuccess'),
);

const Map<String, dynamic> testConfigJson = <String, dynamic>{
  'id': 'testid',
  'label': 'testlabel',
  'sdkNameAndVersion': 'testsdknameandversion',
  'enabled': true,
  'ping': <String>['testping'],
  'pingSuccessRegex': 'testpingsuccess',
  'postBuild': <String>['testpostbuild'],
  'install': <String>['testinstall'],
  'uninstall': <String>['testuninstall'],
  'runDebug': <String>['testrundebug'],
  'forwardPort': <String>['testforwardport'],
  'forwardPortSuccessRegex': 'testforwardportsuccess',
};
