// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_SINK_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_SINK_H_

namespace flutter {

class EncodableValue;

// Event callback. Events to be sent to Flutter application
// act as clients of this interface for sending events.
template <typename T = EncodableValue>
class EventSink {
 public:
  EventSink() = default;
  virtual ~EventSink() = default;

  // Prevent copying.
  EventSink(EventSink const&) = delete;
  EventSink& operator=(EventSink const&) = delete;

  // Consumes a successful event
  void Success(const T& event) { SuccessInternal(&event); }

  // Consumes a successful event.
  void Success() { SuccessInternal(nullptr); }

  // Consumes an error event.
  void Error(const std::string& error_code,
             const std::string& error_message,
             const T& error_details) {
    ErrorInternal(error_code, error_message, &error_details);
  }

  // Consumes an error event.
  void Error(const std::string& error_code,
             const std::string& error_message = "") {
    ErrorInternal(error_code, error_message, nullptr);
  }

  // Consumes end of stream. Ensuing calls to Success() or
  // Error(), if any, are ignored.
  void EndOfStream() { EndOfStreamInternal(); }

 protected:
  // Implementation of the public interface, to be provided by subclasses.
  virtual void SuccessInternal(const T* event = nullptr) = 0;

  // Implementation of the public interface, to be provided by subclasses.
  virtual void ErrorInternal(const std::string& error_code,
                             const std::string& error_message,
                             const T* error_details) = 0;

  // Implementation of the public interface, to be provided by subclasses.
  virtual void EndOfStreamInternal() = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_EVENT_SINK_H_
