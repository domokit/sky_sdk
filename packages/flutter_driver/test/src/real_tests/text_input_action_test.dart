// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart' as flutter_driver;
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('flutter_driver.TextInputAction should be sync with TextInputAction', () {
    final List<String> actual =
        flutter_driver.TextInputAction.values
            .map((flutter_driver.TextInputAction action) => action.name)
            .toList();
    final List<String> matcher =
        TextInputAction.values.map((TextInputAction action) => action.name).toList();
    expect(actual, matcher);
  });
}
