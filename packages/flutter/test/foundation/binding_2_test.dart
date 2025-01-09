// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

class TestBinding extends BindingBase {}

void main() {
  test('BindingBase.debugCheckZone', () async {
    final BindingBase binding = TestBinding();
    binding.debugCheckZone('test1');
    BindingBase.debugZoneErrorsAreFatal = true;
    Zone.current.fork().run(() {
      try {
        binding.debugCheckZone('test2');
        fail('expected an exception');
      } catch (error) {
        expect(error, isA<FlutterError>());
        expect(
          error.toString(),
          'Zone mismatch.\n'
          'The Flutter bindings were initialized in a different zone than is now being used. '
          'This will likely cause confusion and bugs as any zone-specific configuration will '
          'inconsistently use the configuration of the original binding initialization zone '
          'or this zone based on hard-to-predict factors such as which zone was active when '
          'a particular callback was set.\n'
          'It is important to use the same zone when calling `ensureInitialized` on the '
          'binding as when calling `test2` later.\n'
          'To make this error non-fatal, set BindingBase.debugZoneErrorsAreFatal to false '
          'before the bindings are initialized (i.e. as the first statement in `void main() { }`).',
        );
      }
    });
    BindingBase.debugZoneErrorsAreFatal = false;
    Zone.current.fork().run(() {
      bool sawError = false;
      final FlutterExceptionHandler? lastHandler = FlutterError.onError;
      FlutterError.onError = (FlutterErrorDetails details) {
        final Object error = details.exception;
        expect(error, isA<FlutterError>());
        expect(
          error.toString(),
          'Zone mismatch.\n'
          'The Flutter bindings were initialized in a different zone than is now being used. '
          'This will likely cause confusion and bugs as any zone-specific configuration will '
          'inconsistently use the configuration of the original binding initialization zone '
          'or this zone based on hard-to-predict factors such as which zone was active when '
          'a particular callback was set.\n'
          'It is important to use the same zone when calling `ensureInitialized` on the '
          'binding as when calling `test3` later.\n'
          'To make this warning fatal, set BindingBase.debugZoneErrorsAreFatal to true '
          'before the bindings are initialized (i.e. as the first statement in `void main() { }`).',
        );
        sawError = true;
      };
      binding.debugCheckZone('test3');
      expect(sawError, isTrue);
      FlutterError.onError = lastHandler;
    });
  });
}
