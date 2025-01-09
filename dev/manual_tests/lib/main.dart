// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

void main() {
  runApp(
    const Directionality(
      textDirection: TextDirection.ltr,
      child: Center(child: Text('flutter run -t xxx.dart')),
    ),
  );
}
