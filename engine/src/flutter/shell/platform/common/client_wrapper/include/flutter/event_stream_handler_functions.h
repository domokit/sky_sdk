// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_STREAM_HANDLER_FUNCTIONS_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_STREAM_HANDLER_FUNCTIONS_H_

#include <memory>

#include "event_sink.h"
#include "event_stream_handler.h"

namespace flutter {

class EncodableValue;

// Handler types for each of the StreamHandler setup and teardown
// requests.
template <typename T>
using StreamHandlerListen =
    std::function<std::unique_ptr<StreamHandlerError<T>>(
        const T* arguments,
        std::unique_ptr<EventSink<T>>&& events)>;

template <typename T>
using StreamHandlerCancel =
    std::function<std::unique_ptr<StreamHandlerError<T>>(const T* arguments)>;

// An implementation of StreamHandler that pass calls through to
// provided function objects.
template <typename T = EncodableValue>
class StreamHandlerFunctions : public StreamHandler<T> {
 public:
  // Creates a handler object that calls the provided functions
  // for the corresponding StreamHandler outcomes.
  StreamHandlerFunctions(StreamHandlerListen<T> on_listen,
                         StreamHandlerCancel<T> on_cancel)
      : on_listen_(on_listen), on_cancel_(on_cancel) {}

  virtual ~StreamHandlerFunctions() = default;

  // Prevent copying.
  StreamHandlerFunctions(StreamHandlerFunctions const&) = delete;
  StreamHandlerFunctions& operator=(StreamHandlerFunctions const&) = delete;

 protected:
  // |flutter::StreamHandler|
  std::unique_ptr<StreamHandlerError<T>> OnListenInternal(
      const T* arguments,
      std::unique_ptr<EventSink<T>>&& events) override {
    if (on_listen_) {
      return on_listen_(arguments, std::move(events));
    }

    auto error = std::make_unique<StreamHandlerError<T>>(
        "error", "No OnListen handler set", nullptr);
    return std::move(error);
  }

  // |flutter::StreamHandler|
  std::unique_ptr<StreamHandlerError<T>> OnCancelInternal(
      const T* arguments) override {
    if (on_cancel_) {
      return on_cancel_(arguments);
    }

    auto error = std::make_unique<StreamHandlerError<T>>(
        "error", "No OnCancel handler set", nullptr);
    return std::move(error);
  }

  StreamHandlerListen<T> on_listen_;
  StreamHandlerCancel<T> on_cancel_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_STREAM_HANDLER_FUNCTIONS_H_
