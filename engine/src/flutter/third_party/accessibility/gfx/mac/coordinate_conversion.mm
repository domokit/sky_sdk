// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "coordinate_conversion.h"

#import <Cocoa/Cocoa.h>

#include "gfx/geometry/point.h"
#include "gfx/geometry/rect.h"

namespace gfx {

namespace {

// The height of the primary display, which OSX defines as the monitor with the
// menubar. This is always at index 0.
CGFloat PrimaryDisplayHeight() {
  return NSMaxY([[[NSScreen screens] firstObject] frame]);
}

}  // namespace

NSRect ScreenRectToNSRect(const Rect& rect) {
  return NSMakeRect(rect.x(), PrimaryDisplayHeight() - rect.y() - rect.height(), rect.width(),
                    rect.height());
}

Rect ScreenRectFromNSRect(const NSRect& rect) {
  return Rect(rect.origin.x, PrimaryDisplayHeight() - rect.origin.y - rect.size.height,
              rect.size.width, rect.size.height);
}

NSPoint ScreenPointToNSPoint(const Point& point) {
  return NSMakePoint(point.x(), PrimaryDisplayHeight() - point.y());
}

Point ScreenPointFromNSPoint(const NSPoint& point) {
  return Point(point.x, PrimaryDisplayHeight() - point.y);
}

}  // namespace gfx
