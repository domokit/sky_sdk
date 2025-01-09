// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/dds.dart';

import '../../src/common.dart';

void main() {
  group('DartDevelopmentServiceException.fromJSON', () {
    test('parses existing DDS instance error', () {
      final DartDevelopmentServiceException actual =
          DartDevelopmentServiceException.fromJson(<String, Object>{
            'error_code': DartDevelopmentServiceException.existingDdsInstanceError,
            'message': 'Foo',
            'uri': 'http://localhost',
          });
      final DartDevelopmentServiceException expected =
          DartDevelopmentServiceException.existingDdsInstance(
            'Foo',
            ddsUri: Uri.parse('http://localhost'),
          );
      expect(actual.errorCode, expected.errorCode);
      expect(actual.message, expected.message);
      expect(actual, isA<ExistingDartDevelopmentServiceException>());
      expect(
        (actual as ExistingDartDevelopmentServiceException).ddsUri,
        (expected as ExistingDartDevelopmentServiceException).ddsUri,
      );
    });

    test('parses connection issue error', () {
      final DartDevelopmentServiceException actual = DartDevelopmentServiceException.fromJson(
        <String, Object>{
          'error_code': DartDevelopmentServiceException.connectionError,
          'message': 'Foo',
        },
      );
      final DartDevelopmentServiceException expected =
          DartDevelopmentServiceException.connectionIssue('Foo');
      expect(actual.errorCode, expected.errorCode);
      expect(actual.message, expected.message);
    });

    test('parses failed to start error', () {
      final DartDevelopmentServiceException expected =
          DartDevelopmentServiceException.failedToStart();
      final DartDevelopmentServiceException actual = DartDevelopmentServiceException.fromJson(
        <String, Object>{
          'error_code': DartDevelopmentServiceException.failedToStartError,
          'message': expected.message,
        },
      );
      expect(actual.errorCode, expected.errorCode);
      expect(actual.message, expected.message);
    });
  });
}
