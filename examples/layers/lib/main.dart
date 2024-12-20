// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

void main() {
  runApp(
    const Center(
      child: Text('Instead run:\nflutter run xxx/yyy.dart', textDirection: TextDirection.ltr),
    ),
  );
}
