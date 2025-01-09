// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/config.dart';
import 'package:flutter_tools/src/reporting/unified_analytics.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../src/common.dart';
import '../src/fakes.dart';

void main() {
  const String userBranch = 'abc123';
  const String clientIde = 'VSCode';

  late MemoryFileSystem fs;
  late Config config;
  late FakeAnalytics analyticsOverride;

  setUp(() {
    fs = MemoryFileSystem.test();
    config = Config.test();

    analyticsOverride = getInitializedFakeAnalyticsInstance(
      fs: fs,
      fakeFlutterVersion: FakeFlutterVersion(branch: userBranch),
      clientIde: clientIde,
    );
  });

  group('Unit testing util:', () {
    test('getEnabledFeatures is null', () {
      final String? enabledFeatures = getEnabledFeatures(config);
      expect(enabledFeatures, isNull);
    });

    testWithoutContext('getEnabledFeatures not null', () {
      config.setValue('cli-animations', true);
      config.setValue('enable-flutter-preview', true);

      final String? enabledFeatures = getEnabledFeatures(config);
      expect(enabledFeatures, isNotNull);
      expect(
        enabledFeatures!.split(','),
        unorderedEquals(<String>['enable-flutter-preview', 'cli-animations']),
      );
    });
  });

  group('Unit testing getAnalytics', () {
    testWithoutContext('Successfully creates the instance for standard branch', () {
      final Analytics analytics = getAnalytics(
        runningOnBot: false,
        flutterVersion: FakeFlutterVersion(),
        environment: const <String, String>{},
        analyticsOverride: analyticsOverride,
        clientIde: clientIde,
        config: config,
      );

      expect(
        analytics.clientId,
        isNot(NoOpAnalytics.staticClientId),
        reason: 'The CLIENT ID should be a randomly generated id',
      );
      expect(analytics, isNot(isA<NoOpAnalytics>()));
    });

    testWithoutContext('NoOp instance for user branch', () {
      final Analytics analytics = getAnalytics(
        runningOnBot: false,
        flutterVersion: FakeFlutterVersion(
          branch: userBranch,
          frameworkRevision: '3.14.0-14.0.pre.370',
        ),
        environment: const <String, String>{},
        analyticsOverride: analyticsOverride,
        clientIde: clientIde,
        config: config,
      );

      expect(
        analytics.clientId,
        NoOpAnalytics.staticClientId,
        reason: 'The client ID should match the NoOp client id',
      );
      expect(analytics, isA<NoOpAnalytics>());
    });

    testWithoutContext('NoOp instance for unknown branch', () {
      final Analytics analytics = getAnalytics(
        runningOnBot: false,
        flutterVersion: FakeFlutterVersion(frameworkRevision: 'unknown'),
        environment: const <String, String>{},
        analyticsOverride: analyticsOverride,
        clientIde: clientIde,
        config: config,
      );

      expect(
        analytics.clientId,
        NoOpAnalytics.staticClientId,
        reason: 'The client ID should match the NoOp client id',
      );
      expect(analytics, isA<NoOpAnalytics>());
    });

    testWithoutContext('NoOp instance when running on bots', () {
      final Analytics analytics = getAnalytics(
        runningOnBot: true,
        flutterVersion: FakeFlutterVersion(),
        environment: const <String, String>{},
        analyticsOverride: analyticsOverride,
        clientIde: clientIde,
        config: config,
      );

      expect(
        analytics.clientId,
        NoOpAnalytics.staticClientId,
        reason: 'The client ID should match the NoOp client id',
      );
      expect(analytics, isA<NoOpAnalytics>());
    });

    testWithoutContext('NoOp instance when suppressing via env variable', () {
      final Analytics analytics = getAnalytics(
        runningOnBot: false,
        flutterVersion: FakeFlutterVersion(),
        environment: const <String, String>{'FLUTTER_SUPPRESS_ANALYTICS': 'true'},
        analyticsOverride: analyticsOverride,
        clientIde: clientIde,
        config: config,
      );

      expect(
        analytics.clientId,
        NoOpAnalytics.staticClientId,
        reason: 'The client ID should match the NoOp client id',
      );
      expect(analytics, isA<NoOpAnalytics>());
    });

    testWithoutContext('Suppression prevents events from being sent', () {
      expect(analyticsOverride.okToSend, true);
      analyticsOverride.send(Event.surveyShown(surveyId: 'surveyId'));
      expect(analyticsOverride.sentEvents, hasLength(1));

      analyticsOverride.suppressTelemetry();
      expect(analyticsOverride.okToSend, false);
      analyticsOverride.send(Event.surveyShown(surveyId: 'surveyId'));

      expect(analyticsOverride.sentEvents, hasLength(1));
    });

    testWithoutContext('Client IDE is passed and found in events', () {
      final Analytics analytics = getAnalytics(
        runningOnBot: false,
        flutterVersion: FakeFlutterVersion(),
        environment: const <String, String>{},
        analyticsOverride: analyticsOverride,
        clientIde: clientIde,
        config: config,
      );
      analytics as FakeAnalytics;

      expect(analytics.userProperty.clientIde, 'VSCode');
    });
  });
}
