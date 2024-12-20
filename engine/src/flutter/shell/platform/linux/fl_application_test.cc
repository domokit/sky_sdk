// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_application.h"

TEST(FlApplicationTest, ConstructorArgs) {
  g_autoptr(FlApplication) app =
      fl_application_new("com.example.TestApplication",
#ifdef GLIB_VERSION_2_74
                         G_APPLICATION_DEFAULT_FLAGS
#else
                         G_APPLICATION_FLAGS_NONE
#endif
      );

  EXPECT_STREQ(g_application_get_application_id(G_APPLICATION(app)),
               "com.example.TestApplication");

#ifdef GLIB_VERSION_2_74
  EXPECT_EQ(g_application_get_flags(G_APPLICATION(app)),
            G_APPLICATION_DEFAULT_FLAGS);
#else
  EXPECT_EQ(g_application_get_flags(G_APPLICATION(app)),
            G_APPLICATION_FLAGS_NONE);
#endif
}
