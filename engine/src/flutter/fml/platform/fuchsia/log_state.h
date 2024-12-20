// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_FUCHSIA_LOG_STATE_H_
#define FLUTTER_FML_PLATFORM_FUCHSIA_LOG_STATE_H_

#include <fidl/fuchsia.logger/cpp/fidl.h>
#include <lib/fidl/cpp/wire/internal/transport_channel.h>
#include <lib/zx/socket.h>

#include <atomic>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fml {

// Class for holding the global connection to the Fuchsia LogSink service.
class LogState {
 public:
  // Connects to the Fuchsia LogSink service.
  LogState();

  // Get the socket for sending log messages.
  const zx::socket& socket() const { return socket_; }

  // Get the current list of tags.
  std::shared_ptr<const std::vector<std::string>> tags() const {
    return std::atomic_load(&tags_);
  }

  // Take ownership of the log sink channel (e.g. for LogInterestListener).
  // This is thread-safe.
  fidl::ClientEnd<::fuchsia_logger::LogSink> TakeClientEnd();

  // Updates the default tags.
  // This is thread-safe.
  void SetTags(const std::initializer_list<std::string>& tags);

  // Get the default instance of LogState.
  static LogState& Default();

 private:
  std::mutex mutex_;
  fidl::ClientEnd<::fuchsia_logger::LogSink> client_end_;
  zx::socket socket_;
  std::shared_ptr<const std::vector<std::string>> tags_;
};

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_FUCHSIA_LOG_STATE_H_
