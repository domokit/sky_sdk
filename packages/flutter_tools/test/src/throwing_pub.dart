// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/dart/pub.dart';

final class ThrowingPub implements Pub {
  const ThrowingPub();

  @override
  Never noSuchMethod(Invocation invocation) {
    throw UnsupportedError(
      'Attempted to invoke pub during test, which otherwise was unexpected. '
      'This error may be caused by either changing the implementation details '
      'of the Flutter CLI in where the "Pub" class is now being used, or '
      'adding a unit test that transitively depends on "Pub".\n'
      '\n'
      'Possible options for resolution:\n'
      ' 1. Refactor the code or test to not rely on "Pub".\n'
      ' 2. Create and use a test-appropriate Fake (grep for "implements Pub") '
      '    for example code across the test/ repo. It is possible that the '
      '    file you are editing already has an appropriate Fake.\n',
    );
  }
}
