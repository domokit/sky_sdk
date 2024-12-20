// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fl_test_gtk_logs.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {

bool gtk_initialized = false;
GLogWriterFunc log_writer_cb = nullptr;
GLogLevelFlags fl_received_log_levels = static_cast<GLogLevelFlags>(0x0);

GLogWriterOutput log_writer(GLogLevelFlags log_level,
                            const GLogField* fields,
                            gsize n_fields,
                            gpointer user_data) {
  fl_received_log_levels =
      static_cast<GLogLevelFlags>(log_level | fl_received_log_levels);
  if (log_writer_cb == nullptr) {
    return g_log_writer_default(log_level, fields, n_fields, user_data);
  }
  GLogWriterOutput result =
      log_writer_cb(log_level, fields, n_fields, user_data);
  if (result != G_LOG_WRITER_HANDLED) {
    return g_log_writer_default(log_level, fields, n_fields, user_data);
  }
  return result;
}

}  // namespace

void fl_ensure_gtk_init(GLogWriterFunc writer) {
  if (!gtk_initialized) {
    gtk_init(0, nullptr);
    g_log_set_writer_func(log_writer, nullptr, nullptr);
    gtk_initialized = true;
  }
  fl_reset_received_gtk_log_levels();
  log_writer_cb = writer;
}

void fl_reset_received_gtk_log_levels() {
  fl_received_log_levels = static_cast<GLogLevelFlags>(0x0);
}

GLogLevelFlags fl_get_received_gtk_log_levels() {
  return fl_received_log_levels;
}

}  // namespace testing
}  // namespace flutter