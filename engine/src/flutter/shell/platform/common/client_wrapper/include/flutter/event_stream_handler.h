// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_STREAM_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_STREAM_HANDLER_H_

#include <memory>
#include <string>

#include "event_sink.h"

namespace flutter {

class EncodableValue;

template <typename T = EncodableValue>
struct StreamHandlerError {
  const std::string error_code;
  const std::string error_message;
  const std::unique_ptr<T> error_details;

  StreamHandlerError(const std::string& error_code,
                     const std::string& error_message,
                     std::unique_ptr<T>&& error_details)
      : error_code(error_code),
        error_message(error_message),
        error_details(std::move(error_details)) {}
};

// Handler for stream setup and teardown requests.
// Implementations must be prepared to accept sequences of alternating calls to
// OnListen() and OnCancel(). Implementations should ideally consume no
// resources when the last such call is not OnListen(). In typical situations,
// this means that the implementation should register itself with
// platform-specific event sources OnListen() and deregister again OnCancel().
template <typename T = EncodableValue>
class StreamHandler {
 public:
  StreamHandler() = default;
  virtual ~StreamHandler() = default;

  // Prevent copying.
  StreamHandler(StreamHandler const&) = delete;
  StreamHandler& operator=(StreamHandler const&) = delete;

  // Handles a request to set up an event stream. Returns nullptr on success,
  // or an error on failure.
  // |arguments| is stream configuration arguments and
  // |events| is an EventSink for emitting events to the Flutter receiver.
  std::unique_ptr<StreamHandlerError<T>> OnListen(
      const T* arguments,
      std::unique_ptr<EventSink<T>>&& events) {
    return OnListenInternal(arguments, std::move(events));
  }

  // Handles a request to tear down the most recently created event stream.
  // Returns nullptr on success, or an error on failure.
  // |arguments| is stream configuration arguments.
  std::unique_ptr<StreamHandlerError<T>> OnCancel(const T* arguments) {
    return OnCancelInternal(arguments);
  }

 protected:
  // Implementation of the public interface, to be provided by subclasses.
  virtual std::unique_ptr<StreamHandlerError<T>> OnListenInternal(
      const T* arguments,
      std::unique_ptr<EventSink<T>>&& events) = 0;

  // Implementation of the public interface, to be provided by subclasses.
  virtual std::unique_ptr<StreamHandlerError<T>> OnCancelInternal(
      const T* arguments) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_STREAM_HANDLER_H_
