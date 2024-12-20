// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:web_unicode/web_unicode.dart';

import 'unicode_range.dart';

export 'package:web_unicode/web_unicode.dart' show LineCharProperty;

UnicodePropertyLookup<LineCharProperty> get lineLookup => ensureLineLookupInitialized();

/// Initializes [lineLookup], if it's not already initialized.
///
/// Use this function to trigger the initialization before [lineLookup] is
/// actually used. For example, triggering it before the first application
/// frame is rendered will reduce jank by moving the initialization out of
/// the frame.
UnicodePropertyLookup<LineCharProperty> ensureLineLookupInitialized() {
  return _lineLookup ??= UnicodePropertyLookup<LineCharProperty>.fromPackedData(
    packedLineBreakProperties,
    singleLineBreakRangesCount,
    LineCharProperty.values,
    defaultLineCharProperty,
  );
}

UnicodePropertyLookup<LineCharProperty>? _lineLookup;
