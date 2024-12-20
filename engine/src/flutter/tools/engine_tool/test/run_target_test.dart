// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:engine_tool/src/commands/run_command.dart';
import 'package:engine_tool/src/flutter_tool_interop/device.dart';
import 'package:engine_tool/src/flutter_tool_interop/target_platform.dart';
import 'package:engine_tool/src/label.dart';
import 'package:test/test.dart';

import 'src/matchers.dart';

void main() {
  group('detectAndSelect', () {
    test('returns null on an empty list', () {
      final target = RunTarget.detectAndSelect([]);
      expect(target, isNull);
    });

    test('returns the only target', () {
      final device = _device(TargetPlatform.androidArm64);

      final target = RunTarget.detectAndSelect([device]);
      expect(target, isA<RunTarget>().having((t) => t.device, 'device', device));
    });

    test('returns the first target if multiple are available', () {
      final device1 = _device(TargetPlatform.androidArm64);
      final device2 = _device(TargetPlatform.androidX64);

      final target = RunTarget.detectAndSelect([device1, device2]);
      expect(target, isA<RunTarget>().having((t) => t.device, 'device', device1));
    });

    test('returns the android target', () {
      final device1 = _device(TargetPlatform.darwinArm64);
      final device2 = _device(TargetPlatform.androidArm64);

      final target = RunTarget.detectAndSelect([device1, device2], idPrefix: 'android');
      expect(target, isA<RunTarget>().having((t) => t.device, 'device', device2));
    });

    test('returns the first android target', () {
      final device1 = _device(TargetPlatform.androidArm64);
      final device2 = _device(TargetPlatform.androidX64);

      final target = RunTarget.detectAndSelect([device1, device2], idPrefix: 'android');
      expect(target, isA<RunTarget>().having((t) => t.device, 'device', device1));
    });

    test('returns null if no android targets are available', () {
      final device1 = _device(TargetPlatform.darwinArm64);
      final device2 = _device(TargetPlatform.darwinX64);

      final target = RunTarget.detectAndSelect([device1, device2], idPrefix: 'android');
      expect(target, isNull);
    });
  });

  group('buildConfigFor', () {
    final expectedDebugTargets = {
      TargetPlatform.androidUnspecified: 'android_debug',
      TargetPlatform.androidX86: 'android_debug_x86',
      TargetPlatform.androidX64: 'android_debug_x64',
      TargetPlatform.androidArm64: 'android_debug_arm64',
      TargetPlatform.darwinUnspecified: 'host_debug',
      TargetPlatform.darwinX64: 'host_debug',
      TargetPlatform.darwinArm64: 'host_debug_arm64',
      TargetPlatform.linuxX64: 'host_debug',
      TargetPlatform.linuxArm64: 'host_debug_arm64',
      TargetPlatform.windowsX64: 'host_debug',
      TargetPlatform.windowsArm64: 'host_debug_arm64',
      TargetPlatform.webJavascript: 'chrome_debug',
    };

    for (final platform in TargetPlatform.knownPlatforms) {
      if (expectedDebugTargets.containsKey(platform)) {
        test('${platform.identifier} => ${expectedDebugTargets[platform]}', () {
          final device = _device(platform);
          final target = RunTarget.fromDevice(device);

          expect(target.buildConfigFor('debug'), expectedDebugTargets[platform]);
        });
      } else {
        test('${platform.identifier} => FatalError', () {
          final device = _device(platform);
          final target = RunTarget.fromDevice(device);

          expect(() => target.buildConfigFor('debug'), throwsFatalError);
        });
      }
    }
  });

  group('buildTargetsForShell', () {
    final expectedShellTargets = {
      TargetPlatform.androidUnspecified: [
        Label.parseGn('//flutter/shell/platform/android:android_jar'),
      ],
      TargetPlatform.androidX86: [Label.parseGn('//flutter/shell/platform/android:android_jar')],
      TargetPlatform.androidX64: [Label.parseGn('//flutter/shell/platform/android:android_jar')],
      TargetPlatform.androidArm64: [Label.parseGn('//flutter/shell/platform/android:android_jar')],
      TargetPlatform.iOSUnspecified: [
        Label.parseGn('//flutter/shell/platform/darwin/ios:flutter_framework'),
      ],
      TargetPlatform.iOSX64: [
        Label.parseGn('//flutter/shell/platform/darwin/ios:flutter_framework'),
      ],
      TargetPlatform.iOSArm64: [
        Label.parseGn('//flutter/shell/platform/darwin/ios:flutter_framework'),
      ],
      TargetPlatform.darwinUnspecified: [
        Label.parseGn('//flutter/shell/platform/darwin/macos:flutter_framework'),
      ],
      TargetPlatform.darwinX64: [
        Label.parseGn('//flutter/shell/platform/darwin/macos:flutter_framework'),
      ],
      TargetPlatform.darwinArm64: [
        Label.parseGn('//flutter/shell/platform/darwin/macos:flutter_framework'),
      ],
      TargetPlatform.linuxX64: [Label.parseGn('//flutter/shell/platform/linux:flutter_linux_gtk')],
      TargetPlatform.linuxArm64: [
        Label.parseGn('//flutter/shell/platform/linux:flutter_linux_gtk'),
      ],
      TargetPlatform.windowsX64: [Label.parseGn('//flutter/shell/platform/windows')],
      TargetPlatform.windowsArm64: [Label.parseGn('//flutter/shell/platform/windows')],
      TargetPlatform.webJavascript: [Label.parseGn('//flutter/web_sdk:flutter_web_sdk_archive')],
    };

    for (final platform in TargetPlatform.knownPlatforms) {
      if (expectedShellTargets.containsKey(platform)) {
        test('${platform.identifier} => ${expectedShellTargets[platform]}', () {
          final device = _device(platform);
          final target = RunTarget.fromDevice(device);

          expect(target.buildTargetsForShell, expectedShellTargets[platform]);
        });
      } else {
        test('${platform.identifier} => FatalError', () {
          final device = _device(platform);
          final target = RunTarget.fromDevice(device);

          expect(() => target.buildTargetsForShell, throwsFatalError);
        });
      }
    }
  });
}

Device _device(TargetPlatform platform) {
  return Device(
    name: 'Test Device <${platform.identifier}>',
    id: platform.identifier,
    targetPlatform: platform,
  );
}
