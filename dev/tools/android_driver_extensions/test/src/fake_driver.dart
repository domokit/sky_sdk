// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';

/// A fake [FlutterDriver] that throws [UnimplementedError] for all methods.
final class NullFlutterDriver implements FlutterDriver {
  const NullFlutterDriver();

  @override
  Never noSuchMethod(Invocation invocation) => throw UnimplementedError();
}
