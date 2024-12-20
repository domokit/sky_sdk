// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'widget_inspector_test_utils.dart';

void main() {
  StructureErrorTestWidgetInspectorService.runTests();
}

class StructureErrorTestWidgetInspectorService extends TestWidgetInspectorService {
  static void runTests() {
    final StructureErrorTestWidgetInspectorService service =
        StructureErrorTestWidgetInspectorService();
    WidgetInspectorService.instance = service;

    test('ext.flutter.inspector.structuredErrors - custom FlutterError.onError', () async {
      // Regression test for https://github.com/flutter/flutter/issues/41540

      // Ensures that
      //  * structured errors are enabled by default
      //  * FlutterError.onError without FlutterError.presentError doesn't present structured errors
      //  * FlutterError.onError with FlutterError.presentError does present structured errors
      //  * disabling structured errors sets the default FlutterError.presentError

      final FlutterExceptionHandler? oldHandler = FlutterError.onError;

      try {
        expect(service.dispatchedEvents('Flutter.Error'), isEmpty);

        // Set callback that doesn't call presentError.
        bool onErrorCalled = false;
        FlutterError.onError = (FlutterErrorDetails details) {
          onErrorCalled = true;
        };

        // Get the service registered.
        WidgetsFlutterBinding.ensureInitialized();

        final FlutterErrorDetails expectedError = FlutterErrorDetails(
          library: 'rendering library',
          context: ErrorDescription('during layout'),
          exception: StackTrace.current,
        );
        FlutterError.reportError(expectedError);

        // Verify structured errors are not shown.
        expect(onErrorCalled, true);
        expect(service.dispatchedEvents('Flutter.Error'), isEmpty);

        // Set callback that calls presentError.
        onErrorCalled = false;
        FlutterError.onError = (FlutterErrorDetails details) {
          FlutterError.presentError(details);
          onErrorCalled = true;
        };

        FlutterError.reportError(expectedError);

        // Verify structured errors are shown.
        expect(onErrorCalled, true);
        // Structured errors are not supported on web.
        if (!kIsWeb) {
          expect(service.dispatchedEvents('Flutter.Error'), hasLength(1));
        } else {
          expect(service.dispatchedEvents('Flutter.Error'), isEmpty);
        }

        // Verify disabling structured errors sets the default FlutterError.presentError
        expect(
          await service.testBoolExtension('structuredErrors', <String, String>{'enabled': 'true'}),
          equals('true'),
        );
        expect(FlutterError.presentError, isNot(equals(oldHandler)));

        expect(
          await service.testBoolExtension('structuredErrors', <String, String>{'enabled': 'false'}),
          equals('false'),
        );
        expect(FlutterError.presentError, equals(oldHandler));
      } finally {
        FlutterError.onError = oldHandler;
      }
    });
  }
}
