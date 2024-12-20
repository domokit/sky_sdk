// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_driver/driver_extension.dart';
import 'package:flutter_driver/src/common/find.dart';
import 'package:flutter_test/flutter_test.dart';

import 'stub_finder.dart';

class StubFinderExtension extends FinderExtension {
  @override
  Finder createFinder(SerializableFinder finder, CreateFinderFactory finderFactory) {
    return find.byWidgetPredicate((Widget widget) {
      final Key? key = widget.key;
      if (key is! ValueKey<String>) {
        return false;
      }
      return key.value == (finder as StubFinder).keyString;
    });
  }

  @override
  SerializableFinder deserialize(
    Map<String, String> params,
    DeserializeFinderFactory finderFactory,
  ) {
    return StubFinder(params['keyString']!);
  }

  @override
  String get finderType => 'Stub';
}
