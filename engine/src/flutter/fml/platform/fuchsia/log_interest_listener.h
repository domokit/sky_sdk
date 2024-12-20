// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_FUCHSIA_LOG_INTEREST_LISTENER_H_
#define FLUTTER_FML_PLATFORM_FUCHSIA_LOG_INTEREST_LISTENER_H_

#include <fidl/fuchsia.diagnostics/cpp/fidl.h>
#include <fidl/fuchsia.logger/cpp/fidl.h>
#include <lib/fidl/cpp/client.h>

namespace fml {

// Class to monitor the Fuchsia LogSink service for log interest changes (i.e.
// when the Fuchsia OS requests a change to the minimum log level).
//
// Care should be taken to always use this object on the same thread.
class LogInterestListener {
 public:
  LogInterestListener(fidl::ClientEnd<::fuchsia_logger::LogSink> client_end,
                      async_dispatcher_t* dispatcher)
      : log_sink_(std::move(client_end), dispatcher) {}

  // Schedules async task to monitor the log sink for log interest changes.
  void AsyncWaitForInterestChanged();

  // Updates the global log settings in response to a log interest change.
  static void HandleInterestChange(
      const fuchsia_diagnostics::Interest& interest);

 private:
  fidl::Client<::fuchsia_logger::LogSink> log_sink_;
};

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_FUCHSIA_LOG_INTEREST_LISTENER_H_
