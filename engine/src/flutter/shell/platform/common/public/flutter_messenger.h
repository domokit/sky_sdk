// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_MESSENGER_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_MESSENGER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "flutter_export.h"

#if defined(__cplusplus)
extern "C" {
#endif  // defined(__cplusplus)

// Opaque reference to a Flutter engine messenger.
typedef struct FlutterDesktopMessenger* FlutterDesktopMessengerRef;

// Opaque handle for tracking responses to messages.
typedef struct _FlutterPlatformMessageResponseHandle
    FlutterDesktopMessageResponseHandle;

// The callback expected as a response of a binary message.
typedef void (*FlutterDesktopBinaryReply)(const uint8_t* data,
                                          size_t data_size,
                                          void* user_data);

// A message received from Flutter.
typedef struct {
  // Size of this struct as created by Flutter.
  size_t struct_size;
  // The name of the channel used for this message.
  const char* channel;
  // The raw message data.
  const uint8_t* message;
  // The length of |message|.
  size_t message_size;
  // The response handle. If non-null, the receiver of this message must call
  // FlutterDesktopSendMessageResponse exactly once with this handle.
  const FlutterDesktopMessageResponseHandle* response_handle;
} FlutterDesktopMessage;

// Function pointer type for message handler callback registration.
//
// The user data will be whatever was passed to FlutterDesktopSetMessageHandler
// for the channel the message is received on.
typedef void (*FlutterDesktopMessageCallback)(
    FlutterDesktopMessengerRef /* messenger */,
    const FlutterDesktopMessage* /* message*/,
    void* /* user data */);

// Sends a binary message to the Flutter side on the specified channel.
FLUTTER_EXPORT bool FlutterDesktopMessengerSend(
    FlutterDesktopMessengerRef messenger,
    const char* channel,
    const uint8_t* message,
    const size_t message_size);

// Sends a binary message to the Flutter side on the specified channel.
// The |reply| callback will be executed when a response is received.
FLUTTER_EXPORT bool FlutterDesktopMessengerSendWithReply(
    FlutterDesktopMessengerRef messenger,
    const char* channel,
    const uint8_t* message,
    const size_t message_size,
    const FlutterDesktopBinaryReply reply,
    void* user_data);

// Sends a reply to a FlutterDesktopMessage for the given response handle.
//
// Once this has been called, |handle| is invalid and must not be used again.
FLUTTER_EXPORT void FlutterDesktopMessengerSendResponse(
    FlutterDesktopMessengerRef messenger,
    const FlutterDesktopMessageResponseHandle* handle,
    const uint8_t* data,
    size_t data_length);

// Registers a callback function for incoming binary messages from the Flutter
// side on the specified channel.
//
// Replaces any existing callback. Provide a null handler to unregister the
// existing callback.
//
// If |user_data| is provided, it will be passed in |callback| calls.
FLUTTER_EXPORT void FlutterDesktopMessengerSetCallback(
    FlutterDesktopMessengerRef messenger,
    const char* channel,
    FlutterDesktopMessageCallback callback,
    void* user_data);

// Increments the reference count for the |messenger|.
//
// Operation is thread-safe.
//
// See also: |FlutterDesktopMessengerRelease|
FLUTTER_EXPORT FlutterDesktopMessengerRef FlutterDesktopMessengerAddRef(
    FlutterDesktopMessengerRef messenger);

// Decrements the reference count for the |messenger|.
//
// Operation is thread-safe.
//
// See also: |FlutterDesktopMessengerAddRef|
FLUTTER_EXPORT void FlutterDesktopMessengerRelease(
    FlutterDesktopMessengerRef messenger);

// Returns `true` if the |FlutterDesktopMessengerRef| still references a running
// engine.
//
// This check should be made inside of a |FlutterDesktopMessengerLock| and
// before any other calls are made to the FlutterDesktopMessengerRef when using
// it from a thread other than the platform thread.
FLUTTER_EXPORT bool FlutterDesktopMessengerIsAvailable(
    FlutterDesktopMessengerRef messenger);

// Locks the `FlutterDesktopMessengerRef` ensuring that
// |FlutterDesktopMessengerIsAvailable| does not change while locked.
//
// All calls to the FlutterDesktopMessengerRef from threads other than the
// platform thread should happen inside of a lock.
//
// Operation is thread-safe.
//
// Returns the |messenger| value.
//
// See also: |FlutterDesktopMessengerUnlock|
FLUTTER_EXPORT FlutterDesktopMessengerRef FlutterDesktopMessengerLock(
    FlutterDesktopMessengerRef messenger);

// Unlocks the `FlutterDesktopMessengerRef`.
//
// Operation is thread-safe.
//
// See also: |FlutterDesktopMessengerLock|
FLUTTER_EXPORT void FlutterDesktopMessengerUnlock(
    FlutterDesktopMessengerRef messenger);

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_MESSENGER_H_
