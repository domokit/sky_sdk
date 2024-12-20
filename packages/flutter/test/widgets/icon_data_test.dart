// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('IconDataDiagnosticsProperty includes valueProperties in JSON', () {
    IconDataProperty property = IconDataProperty('foo', const IconData(101010));
    final Map<String, Object> valueProperties =
        property.toJsonMap(const DiagnosticsSerializationDelegate())['valueProperties']!
            as Map<String, Object>;
    expect(valueProperties['codePoint'], 101010);

    property = IconDataProperty('foo', null);
    final Map<String, Object?> json = property.toJsonMap(const DiagnosticsSerializationDelegate());
    expect(json.containsKey('valueProperties'), isFalse);
  });
}
