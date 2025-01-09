// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/src/extension/_extension_io.dart';

import '../../common.dart';

void main() {
  group('test io_extension', () {
    late Future<Map<String, dynamic>> Function(Map<String, String>) call;

    setUp(() {
      call = (Map<String, String> args) async {
        return Future<Map<String, dynamic>>.value(args);
      };
    });

    test('io_extension should throw exception', () {
      expect(() => registerWebServiceExtension(call), throwsUnsupportedError);
    });
  });
}
