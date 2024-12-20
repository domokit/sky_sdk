// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_TEST_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_TEST_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_value.h"

#include <glib.h>
#include <stdint.h>
#include <ostream>

G_BEGIN_DECLS

// Helper functions for the tests. This is not included in the shell library.

// Helper function to convert a hexadecimal string (e.g. "01feab") into GBytes
GBytes* hex_string_to_bytes(const gchar* hex_string);

// Helper function to convert GBytes into a hexadecimal string (e.g. "01feab")
gchar* bytes_to_hex_string(GBytes* bytes);

// GTest printer for FlValue.
void PrintTo(FlValue* v, std::ostream* os);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_TEST_H_
