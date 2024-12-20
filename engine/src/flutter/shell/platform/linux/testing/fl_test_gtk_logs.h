// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_TEST_GTK_LOGS_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_TEST_GTK_LOGS_H_

#include <gtk/gtk.h>

namespace flutter {
namespace testing {

/**
 * Ensures that GTK has been initialized. If GTK has not been initialized, it
 * will be initialized using `gtk_init()`. It will also set the GTK log writer
 * function to monitor the log output, recording the log levels that have been
 * received in a bitfield accessible via {@link fl_get_received_gtk_log_levels}
 *
 * To retrieve the bitfield of recorded log levels, use
 * `fl_get_received_gtk_log_levels()`.
 *
 * @param[in] writer The custom log writer function to use. If `nullptr`, or it
 *      returns G_LOG_WRITER_UNHANDLED, the default log writer function will be
 *      called.
 *
 * @brief Ensures that GTK has been initialized and starts monitoring logs.
 */
void fl_ensure_gtk_init(GLogWriterFunc writer = nullptr);

/**
 * Resets the recorded GTK log levels to zero.
 *
 * @brief Resets the recorded log levels.
 */
void fl_reset_received_gtk_log_levels();

/**
 * Returns a bitfield containing the GTK log levels that have been seen since
 * the last time they were reset.
 *
 * @brief Returns the recorded log levels.
 *
 * @return A `GLogLevelFlags` bitfield representing the recorded log levels.
 */
GLogLevelFlags fl_get_received_gtk_log_levels();

}  // namespace testing
}  // namespace flutter
#endif  // FLUTTER_SHELL_PLATFORM_LINUX_TESTING_FL_TEST_GTK_LOGS_H_
