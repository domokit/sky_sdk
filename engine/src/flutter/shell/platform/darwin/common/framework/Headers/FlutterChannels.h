// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERCHANNELS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERCHANNELS_H_

#import "FlutterBinaryMessenger.h"
#import "FlutterCodecs.h"

NS_ASSUME_NONNULL_BEGIN
/**
 * A message reply callback.
 *
 * Used for submitting a reply back to a Flutter message sender. Also used in
 * the dual capacity for handling a message reply received from Flutter.
 *
 * @param reply The reply.
 */
typedef void (^FlutterReply)(id _Nullable reply);

/**
 * A strategy for handling incoming messages from Flutter and to send
 * asynchronous replies back to Flutter.
 *
 * @param message The message.
 * @param callback A callback for submitting a reply to the sender which can be invoked from any
 * thread.
 */
typedef void (^FlutterMessageHandler)(id _Nullable message, FlutterReply callback);

/**
 * A channel for communicating with the Flutter side using basic, asynchronous
 * message passing.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterBasicMessageChannel : NSObject
/**
 * Creates a `FlutterBasicMessageChannel` with the specified name and binary
 * messenger.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * The channel uses `FlutterStandardMessageCodec` to encode and decode messages.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 */
+ (instancetype)messageChannelWithName:(NSString*)name
                       binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger;

/**
 * Creates a `FlutterBasicMessageChannel` with the specified name, binary
 * messenger, and message codec.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The message codec.
 */
+ (instancetype)messageChannelWithName:(NSString*)name
                       binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                                 codec:(NSObject<FlutterMessageCodec>*)codec;

/**
 * Initializes a `FlutterBasicMessageChannel` with the specified name, binary
 * messenger, and message codec.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The message codec.
 */
- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMessageCodec>*)codec;

/**
 * Initializes a `FlutterBasicMessageChannel` with the specified name, binary
 * messenger, and message codec.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The message codec.
 * @param taskQueue The FlutterTaskQueue that executes the handler (see
                    -[FlutterBinaryMessenger makeBackgroundTaskQueue]).
 */
- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMessageCodec>*)codec
                   taskQueue:(NSObject<FlutterTaskQueue>* _Nullable)taskQueue;

/**
 * Sends the specified message to the Flutter side, ignoring any reply.
 *
 * @param message The message. Must be supported by the codec of this
 * channel.
 */
- (void)sendMessage:(id _Nullable)message;

/**
 * Sends the specified message to the Flutter side, expecting an asynchronous
 * reply.
 *
 * @param message The message. Must be supported by the codec of this channel.
 * @param callback A callback to be invoked with the message reply from Flutter.
 */
- (void)sendMessage:(id _Nullable)message reply:(FlutterReply _Nullable)callback;

/**
 * Registers a message handler with this channel.
 *
 * Replaces any existing handler. Use a `nil` handler for unregistering the
 * existing handler.
 *
 * @param handler The message handler.
 */
- (void)setMessageHandler:(FlutterMessageHandler _Nullable)handler;

/**
 * Adjusts the number of messages that will get buffered when sending messages to
 * channels that aren't fully set up yet.  For example, the engine isn't running
 * yet or the channel's message handler isn't set up on the Dart side yet.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param newSize The number of messages that will get buffered.
 */
+ (void)resizeChannelWithName:(NSString*)name
              binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                         size:(NSInteger)newSize;

/**
 * Adjusts the number of messages that will get buffered when sending messages to
 * channels that aren't fully set up yet.  For example, the engine isn't running
 * yet or the channel's message handler isn't set up on the Dart side yet.
 *
 * @param newSize The number of messages that will get buffered.
 */
- (void)resizeChannelBuffer:(NSInteger)newSize;

/**
 * Defines whether the channel should show warning messages when discarding messages
 * due to overflow.
 *
 * @param warns When false, the channel is expected to overflow and warning messages
 *              will not be shown.
 * @param name The channel name.
 * @param messenger The binary messenger.
 */
+ (void)setWarnsOnOverflow:(BOOL)warns
        forChannelWithName:(NSString*)name
           binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger;

/**
 * Defines whether the channel should show warning messages when discarding messages
 * due to overflow.
 *
 * @param warns When false, the channel is expected to overflow and warning messages
 *              will not be shown.
 */
- (void)setWarnsOnOverflow:(BOOL)warns;

@end

/**
 * A method call result callback.
 *
 * Used for submitting a method call result back to a Flutter caller. Also used in
 * the dual capacity for handling a method call result received from Flutter.
 *
 * @param result The result.
 */
typedef void (^FlutterResult)(id _Nullable result);

/**
 * A strategy for handling method calls.
 *
 * @param call The incoming method call.
 * @param result A callback to asynchronously submit the result of the call.
 *     Invoke the callback with a `FlutterError` to indicate that the call failed.
 *     Invoke the callback with `FlutterMethodNotImplemented` to indicate that the
 *     method was unknown. Any other values, including `nil`, are interpreted as
 *     successful results.  This can be invoked from any thread.
 */
typedef void (^FlutterMethodCallHandler)(FlutterMethodCall* call, FlutterResult result);

/**
 * A constant used with `FlutterMethodCallHandler` to respond to the call of an
 * unknown method.
 */
FLUTTER_DARWIN_EXPORT
extern NSObject const* FlutterMethodNotImplemented;

/**
 * A channel for communicating with the Flutter side using invocation of
 * asynchronous methods.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterMethodChannel : NSObject
/**
 * Creates a `FlutterMethodChannel` with the specified name and binary messenger.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * The channel uses `FlutterStandardMethodCodec` to encode and decode method calls
 * and result envelopes.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 */
+ (instancetype)methodChannelWithName:(NSString*)name
                      binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger;

/**
 * Creates a `FlutterMethodChannel` with the specified name, binary messenger, and
 * method codec.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The method codec.
 */
+ (instancetype)methodChannelWithName:(NSString*)name
                      binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                                codec:(NSObject<FlutterMethodCodec>*)codec;

/**
 * Initializes a `FlutterMethodChannel` with the specified name, binary messenger,
 * and method codec.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The method codec.
 */
- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec;

/**
 * Initializes a `FlutterMethodChannel` with the specified name, binary messenger,
 * method codec, and task queue.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The method codec.
 * @param taskQueue The FlutterTaskQueue that executes the handler (see
                    -[FlutterBinaryMessenger makeBackgroundTaskQueue]).
 */
- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec
                   taskQueue:(NSObject<FlutterTaskQueue>* _Nullable)taskQueue;

// clang-format off
/**
 * Invokes the specified Flutter method with the specified arguments, expecting
 * no results.
 *
 * @see [MethodChannel.setMethodCallHandler](https://api.flutter.dev/flutter/services/MethodChannel/setMethodCallHandler.html)
 *
 * @param method The name of the method to invoke.
 * @param arguments The arguments. Must be a value supported by the codec of this
 *     channel.
 */
// clang-format on
- (void)invokeMethod:(NSString*)method arguments:(id _Nullable)arguments;

/**
 * Invokes the specified Flutter method with the specified arguments, expecting
 * an asynchronous result.
 *
 * @param method The name of the method to invoke.
 * @param arguments The arguments. Must be a value supported by the codec of this
 *     channel.
 * @param callback A callback that will be invoked with the asynchronous result.
 *     The result will be a `FlutterError` instance, if the method call resulted
 *     in an error on the Flutter side. Will be `FlutterMethodNotImplemented`, if
 *     the method called was not implemented on the Flutter side. Any other value,
 *     including `nil`, should be interpreted as successful results.
 */
- (void)invokeMethod:(NSString*)method
           arguments:(id _Nullable)arguments
              result:(FlutterResult _Nullable)callback;
/**
 * Registers a handler for method calls from the Flutter side.
 *
 * Replaces any existing handler. Use a `nil` handler for unregistering the
 * existing handler.
 *
 * @param handler The method call handler.
 */
- (void)setMethodCallHandler:(FlutterMethodCallHandler _Nullable)handler;

/**
 * Adjusts the number of messages that will get buffered when sending messages to
 * channels that aren't fully set up yet.  For example, the engine isn't running
 * yet or the channel's message handler isn't set up on the Dart side yet.
 */
- (void)resizeChannelBuffer:(NSInteger)newSize;

@end

/**
 * An event sink callback.
 *
 * @param event The event.
 */
typedef void (^FlutterEventSink)(id _Nullable event);

/**
 * A strategy for exposing an event stream to the Flutter side.
 */
FLUTTER_DARWIN_EXPORT
@protocol FlutterStreamHandler
/**
 * Sets up an event stream and begin emitting events.
 *
 * Invoked when the first listener is registered with the Stream associated to
 * this channel on the Flutter side.
 *
 * @param arguments Arguments for the stream.
 * @param events A callback to asynchronously emit events. Invoke the
 *     callback with a `FlutterError` to emit an error event. Invoke the
 *     callback with `FlutterEndOfEventStream` to indicate that no more
 *     events will be emitted. Any other value, including `nil` are emitted as
 *     successful events.
 * @return A FlutterError instance, if setup fails.
 */
- (FlutterError* _Nullable)onListenWithArguments:(id _Nullable)arguments
                                       eventSink:(FlutterEventSink)events;

/**
 * Tears down an event stream.
 *
 * Invoked when the last listener is deregistered from the Stream associated to
 * this channel on the Flutter side.
 *
 * The channel implementation may call this method with `nil` arguments
 * to separate a pair of two consecutive set up requests. Such request pairs
 * may occur during Flutter hot restart.
 *
 * @param arguments Arguments for the stream.
 * @return A FlutterError instance, if teardown fails.
 */
- (FlutterError* _Nullable)onCancelWithArguments:(id _Nullable)arguments;
@end

/**
 * A constant used with `FlutterEventChannel` to indicate end of stream.
 */
FLUTTER_DARWIN_EXPORT
extern NSObject const* FlutterEndOfEventStream;

/**
 * A channel for communicating with the Flutter side using event streams.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterEventChannel : NSObject
/**
 * Creates a `FlutterEventChannel` with the specified name and binary messenger.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterViewController`.
 *
 * The channel uses `FlutterStandardMethodCodec` to decode stream setup and
 * teardown requests, and to encode event envelopes.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 */
+ (instancetype)eventChannelWithName:(NSString*)name
                     binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger;

/**
 * Creates a `FlutterEventChannel` with the specified name, binary messenger,
 * and method codec.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The method codec.
 */
+ (instancetype)eventChannelWithName:(NSString*)name
                     binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                               codec:(NSObject<FlutterMethodCodec>*)codec;

/**
 * Initializes a `FlutterEventChannel` with the specified name, binary messenger,
 * and method codec.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The method codec.
 */
- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec;

/**
 * Initializes a `FlutterEventChannel` with the specified name, binary messenger,
 * method codec and task queue.
 *
 * The channel name logically identifies the channel; identically named channels
 * interfere with each other's communication.
 *
 * The binary messenger is a facility for sending raw, binary messages to the
 * Flutter side. This protocol is implemented by `FlutterEngine` and `FlutterViewController`.
 *
 * @param name The channel name.
 * @param messenger The binary messenger.
 * @param codec The method codec.
 * @param taskQueue The FlutterTaskQueue that executes the handler (see
                    -[FlutterBinaryMessenger makeBackgroundTaskQueue]).
 */
- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec
                   taskQueue:(NSObject<FlutterTaskQueue>* _Nullable)taskQueue;
/**
 * Registers a handler for stream setup requests from the Flutter side.
 *
 * Replaces any existing handler. Use a `nil` handler for unregistering the
 * existing handler.
 *
 * @param handler The stream handler.
 */
- (void)setStreamHandler:(NSObject<FlutterStreamHandler>* _Nullable)handler;
@end
NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERCHANNELS_H_
