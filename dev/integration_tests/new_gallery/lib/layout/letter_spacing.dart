// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

/// Using letter spacing in Flutter for Web can cause a performance drop,
/// see https://github.com/flutter/flutter/issues/51234.
double letterSpacingOrNone(double letterSpacing) => kIsWeb ? 0.0 : letterSpacing;
