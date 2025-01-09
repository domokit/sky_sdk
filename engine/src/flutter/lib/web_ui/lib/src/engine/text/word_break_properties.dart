// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:web_unicode/web_unicode.dart';

import 'unicode_range.dart';

export 'package:web_unicode/web_unicode.dart' show WordCharProperty;

UnicodePropertyLookup<WordCharProperty> wordLookup =
    UnicodePropertyLookup<WordCharProperty>.fromPackedData(
      packedWordBreakProperties,
      singleWordBreakRangesCount,
      WordCharProperty.values,
      defaultWordCharProperty,
    );
