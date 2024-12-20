// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/fuchsia/log_state.h"

#include <fidl/fuchsia.logger/cpp/fidl.h>
#include <lib/component/incoming/cpp/protocol.h>
#include <lib/fidl/cpp/channel.h>
#include <lib/fidl/cpp/wire/internal/transport_channel.h>
#include <lib/zx/socket.h>
#include <zircon/assert.h>
#include <zircon/types.h>

#include <atomic>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "flutter/fml/platform/fuchsia/log_interest_listener.h"

namespace fml {

LogState::LogState() {
  // Get channel to log sink
  auto client_end = component::Connect<fuchsia_logger::LogSink>();
  ZX_ASSERT(client_end.is_ok());
  fidl::SyncClient log_sink(std::move(*client_end));

  // Attempts to create a kernel socket object should never fail.
  zx::socket local, remote;
  zx::socket::create(ZX_SOCKET_DATAGRAM, &local, &remote);
  auto result = log_sink->ConnectStructured({{.socket = std::move(remote)}});
  ZX_ASSERT_MSG(result.is_ok(), "%s",
                result.error_value().FormatDescription().c_str());

  // Wait for the first interest change to set the initial minimum logging
  // level (should return quickly).
  auto interest_result = log_sink->WaitForInterestChange();
  ZX_ASSERT_MSG(interest_result.is_ok(), "%s",
                interest_result.error_value().FormatDescription().c_str());
  LogInterestListener::HandleInterestChange(interest_result->data());

  socket_ = std::move(local);
  client_end_ = log_sink.TakeClientEnd();
}

fidl::ClientEnd<::fuchsia_logger::LogSink> LogState::TakeClientEnd() {
  std::lock_guard lock(mutex_);
  return std::move(client_end_);
}

void LogState::SetTags(const std::initializer_list<std::string>& tags) {
  std::atomic_store(&tags_,
                    std::make_shared<const std::vector<std::string>>(tags));
}

LogState& LogState::Default() {
  static LogState* instance = new LogState();
  return *instance;
}

}  // namespace fml
