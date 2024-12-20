// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter_driver/flutter_driver.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('semantics performance test', () {
    late FlutterDriver driver;

    setUpAll(() async {
      // Turn off any accessibility services that may be running. The purpose of
      // the test is to measure the time it takes to create the initial
      // semantics tree in isolation. If accessibility services are on, the
      // semantics tree gets generated during the first frame and we can't
      // measure it in isolation.
      final Process run = await Process.start(_adbPath(), const <String>[
        'shell',
        'settings',
        'put',
        'secure',
        'enabled_accessibility_services',
        'null',
      ]);
      await run.exitCode;

      driver = await FlutterDriver.connect(printCommunication: true);
    });

    tearDownAll(() async {
      driver.close();
    });

    test('initial tree creation', () async {
      // Let app become fully idle.
      await Future<void>.delayed(const Duration(seconds: 2));

      await driver.forceGC();

      final Timeline timeline = await driver.traceAction(() async {
        expect(
          await driver.setSemantics(true),
          isTrue,
          reason:
              'Could not toggle semantics to on because semantics were already '
              'on, but the test needs to toggle semantics to measure the initial '
              'semantics tree generation in isolation.',
        );
      });

      final Iterable<TimelineEvent>? semanticsEvents = timeline.events?.where(
        (TimelineEvent event) => event.name == 'SEMANTICS',
      );
      if (semanticsEvents?.length != 2) {
        fail(
          'Expected exactly two "SEMANTICS" events, got ${semanticsEvents?.length}:\n$semanticsEvents',
        );
      }
      final Duration semanticsTreeCreation = Duration(
        microseconds:
            semanticsEvents!.last.timestampMicros! - semanticsEvents.first.timestampMicros!,
      );

      final String jsonEncoded = json.encode(<String, dynamic>{
        'initialSemanticsTreeCreation': semanticsTreeCreation.inMilliseconds,
      });
      File(
        p.join(testOutputsDirectory, 'complex_layout_semantics_perf.json'),
      ).writeAsStringSync(jsonEncoded);
    }, timeout: Timeout.none);
  });
}

String _adbPath() {
  final String? androidHome =
      Platform.environment['ANDROID_HOME'] ?? Platform.environment['ANDROID_SDK_ROOT'];
  if (androidHome == null) {
    return 'adb';
  } else {
    return p.join(androidHome, 'platform-tools', 'adb');
  }
}
