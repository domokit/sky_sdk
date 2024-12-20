// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/darwin/string_range_sanitization.h"

namespace fml {

NSRange RangeForCharacterAtIndex(NSString* text, NSUInteger index) {
  if (text == nil || index > text.length) {
    return NSMakeRange(NSNotFound, 0);
  }
  if (index < text.length) {
    return [text rangeOfComposedCharacterSequenceAtIndex:index];
  }
  return NSMakeRange(index, 0);
}

NSRange RangeForCharactersInRange(NSString* text, NSRange range) {
  if (text == nil || range.location + range.length > text.length) {
    return NSMakeRange(NSNotFound, 0);
  }
  NSRange sanitizedRange = [text rangeOfComposedCharacterSequencesForRange:range];
  // We don't want to override the length, we just want to make sure we don't
  // select into the middle of a multi-byte character. Taking the
  // `sanitizedRange`'s length will end up altering the actual selection.
  return NSMakeRange(sanitizedRange.location, range.length);
}

}  // namespace fml
