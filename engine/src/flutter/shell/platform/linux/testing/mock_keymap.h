// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_KEYMAP_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_KEYMAP_H_

#include "gmock/gmock.h"

#include <gdk/gdk.h>

namespace flutter {
namespace testing {

class MockKeymap {
 public:
  MockKeymap();

  MOCK_METHOD(GdkKeymap*, gdk_keymap_get_for_display, (GdkDisplay * display));
  MOCK_METHOD(guint,
              gdk_keymap_lookup_key,
              (GdkKeymap * keymap, const GdkKeymapKey* key));
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_MOCK_KEYMAP_H_
