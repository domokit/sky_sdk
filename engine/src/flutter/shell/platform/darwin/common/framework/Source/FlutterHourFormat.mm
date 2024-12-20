// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterHourFormat.h"

@implementation FlutterHourFormat
+ (BOOL)isAlwaysUse24HourFormat {
  // iOS does not report its "24-Hour Time" user setting in the API. Instead, it applies
  // it automatically to NSDateFormatter when used with [NSLocale currentLocale]. It is
  // essential that [NSLocale currentLocale] is used. Any custom locale, even the one
  // that's the same as [NSLocale currentLocale] will ignore the 24-hour option (there
  // must be some internal field that's not exposed to developers).
  //
  // Therefore this option behaves differently across Android and iOS. On Android this
  // setting is exposed standalone, and can therefore be applied to all locales, whether
  // the "current system locale" or a custom one. On iOS it only applies to the current
  // system locale. Widget implementors must take this into account in order to provide
  // platform-idiomatic behavior in their widgets.
  NSString* dateFormat = [NSDateFormatter dateFormatFromTemplate:@"j"
                                                         options:0
                                                          locale:[NSLocale currentLocale]];
  return [dateFormat rangeOfString:@"a"].location == NSNotFound;
}

@end
