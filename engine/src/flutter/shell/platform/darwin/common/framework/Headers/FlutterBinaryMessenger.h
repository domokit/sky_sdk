// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERBINARYMESSENGER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERBINARYMESSENGER_H_

#import <Foundation/Foundation.h>

#import "FlutterMacros.h"

NS_ASSUME_NONNULL_BEGIN
/**
 * A message reply callback.
 *
 * Used for submitting a binary reply back to a Flutter message sender. Also used
 * in for handling a binary message reply received from Flutter.
 *
 * @param reply The reply.
 */
typedef void (^FlutterBinaryReply)(NSData* _Nullable reply);

/**
 * A strategy for handling incoming binary messages from Flutter and to send
 * asynchronous replies back to Flutter.
 *
 * @param message The message.
 * @param reply A callback for submitting an asynchronous reply to the sender.
 */
typedef void (^FlutterBinaryMessageHandler)(NSData* _Nullable message, FlutterBinaryReply reply);

typedef int64_t FlutterBinaryMessengerConnection;

@protocol FlutterTaskQueue <NSObject>
@end

/**
 * A facility for communicating with the Flutter side using asynchronous message
 * passing with binary messages.
 *
 * Implementated by:
 * - `FlutterBasicMessageChannel`, which supports communication using structured
 * messages.
 * - `FlutterMethodChannel`, which supports communication using asynchronous
 * method calls.
 * - `FlutterEventChannel`, which supports commuication using event streams.
 */
FLUTTER_DARWIN_EXPORT
@protocol FlutterBinaryMessenger <NSObject>
/// TODO(gaaclarke): Remove optional when macos supports Background Platform Channels.
@optional
- (NSObject<FlutterTaskQueue>*)makeBackgroundTaskQueue;

- (FlutterBinaryMessengerConnection)
    setMessageHandlerOnChannel:(NSString*)channel
          binaryMessageHandler:(FlutterBinaryMessageHandler _Nullable)handler
                     taskQueue:(NSObject<FlutterTaskQueue>* _Nullable)taskQueue;

@required
/**
 * Sends a binary message to the Flutter side on the specified channel, expecting
 * no reply.
 *
 * @param channel The channel name.
 * @param message The message.
 */
- (void)sendOnChannel:(NSString*)channel message:(NSData* _Nullable)message;

/**
 * Sends a binary message to the Flutter side on the specified channel, expecting
 * an asynchronous reply.
 *
 * @param channel The channel name.
 * @param message The message.
 * @param callback A callback for receiving a reply.
 */
- (void)sendOnChannel:(NSString*)channel
              message:(NSData* _Nullable)message
          binaryReply:(FlutterBinaryReply _Nullable)callback;

/**
 * Registers a message handler for incoming binary messages from the Flutter side
 * on the specified channel.
 *
 * Replaces any existing handler. Use a `nil` handler for unregistering the
 * existing handler.
 *
 * @param channel The channel name.
 * @param handler The message handler.
 * @return An identifier that represents the connection that was just created to the channel.
 */
- (FlutterBinaryMessengerConnection)setMessageHandlerOnChannel:(NSString*)channel
                                          binaryMessageHandler:
                                              (FlutterBinaryMessageHandler _Nullable)handler;

/**
 * Clears out a channel's message handler if that handler is still the one that
 * was created as a result of
 * `setMessageHandlerOnChannel:binaryMessageHandler:`.
 *
 * @param connection The result from `setMessageHandlerOnChannel:binaryMessageHandler:`.
 */
- (void)cleanUpConnection:(FlutterBinaryMessengerConnection)connection;
@end
NS_ASSUME_NONNULL_END
#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERBINARYMESSENGER_H_
