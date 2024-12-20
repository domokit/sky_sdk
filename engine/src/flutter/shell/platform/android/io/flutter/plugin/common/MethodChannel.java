// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import io.flutter.BuildConfig;
import io.flutter.Log;
import io.flutter.plugin.common.BinaryMessenger.BinaryMessageHandler;
import io.flutter.plugin.common.BinaryMessenger.BinaryReply;
import java.nio.ByteBuffer;

/**
 * A named channel for communicating with the Flutter application using asynchronous method calls.
 *
 * <p>Incoming method calls are decoded from binary on receipt, and Java results are encoded into
 * binary before being transmitted back to Flutter. The {@link MethodCodec} used must be compatible
 * with the one used by the Flutter application. This can be achieved by creating a <a
 * href="https://api.flutter.dev/flutter/services/MethodChannel-class.html">MethodChannel</a>
 * counterpart of this channel on the Dart side. The Java type of method call arguments and results
 * is {@code Object}, but only values supported by the specified {@link MethodCodec} can be used.
 *
 * <p>The logical identity of the channel is given by its name. Identically named channels will
 * interfere with each other's communication.
 */
public class MethodChannel {
  private static final String TAG = "MethodChannel#";

  private final BinaryMessenger messenger;
  private final String name;
  private final MethodCodec codec;
  private final BinaryMessenger.TaskQueue taskQueue;

  /**
   * Creates a new channel associated with the specified {@link BinaryMessenger} and with the
   * specified name and the standard {@link MethodCodec}.
   *
   * @param messenger a {@link BinaryMessenger}.
   * @param name a channel name String.
   */
  public MethodChannel(@NonNull BinaryMessenger messenger, @NonNull String name) {
    this(messenger, name, StandardMethodCodec.INSTANCE);
  }

  /**
   * Creates a new channel associated with the specified {@link BinaryMessenger} and with the
   * specified name and {@link MethodCodec}.
   *
   * @param messenger a {@link BinaryMessenger}.
   * @param name a channel name String.
   * @param codec a {@link MessageCodec}.
   */
  public MethodChannel(
      @NonNull BinaryMessenger messenger, @NonNull String name, @NonNull MethodCodec codec) {
    this(messenger, name, codec, null);
  }

  /**
   * Creates a new channel associated with the specified {@link BinaryMessenger} and with the
   * specified name and {@link MethodCodec}.
   *
   * @param messenger a {@link BinaryMessenger}.
   * @param name a channel name String.
   * @param codec a {@link MessageCodec}.
   * @param taskQueue a {@link BinaryMessenger.TaskQueue} that specifies what thread will execute
   *     the handler. Specifying null means execute on the platform thread. See also {@link
   *     BinaryMessenger#makeBackgroundTaskQueue()}.
   */
  public MethodChannel(
      @NonNull BinaryMessenger messenger,
      @NonNull String name,
      @NonNull MethodCodec codec,
      @Nullable BinaryMessenger.TaskQueue taskQueue) {
    if (BuildConfig.DEBUG) {
      if (messenger == null) {
        Log.e(TAG, "Parameter messenger must not be null.");
      }
      if (name == null) {
        Log.e(TAG, "Parameter name must not be null.");
      }
      if (codec == null) {
        Log.e(TAG, "Parameter codec must not be null.");
      }
    }
    this.messenger = messenger;
    this.name = name;
    this.codec = codec;
    this.taskQueue = taskQueue;
  }

  /**
   * Invokes a method on this channel, expecting no result.
   *
   * @param method the name String of the method.
   * @param arguments the arguments for the invocation, possibly null.
   */
  @UiThread
  public void invokeMethod(@NonNull String method, @Nullable Object arguments) {
    invokeMethod(method, arguments, null);
  }

  /**
   * Invokes a method on this channel, optionally expecting a result.
   *
   * <p>Any uncaught exception thrown by the result callback will be caught and logged.
   *
   * @param method the name String of the method.
   * @param arguments the arguments for the invocation, possibly null.
   * @param callback a {@link Result} callback for the invocation result, or null.
   */
  @UiThread
  public void invokeMethod(
      @NonNull String method, @Nullable Object arguments, @Nullable Result callback) {
    messenger.send(
        name,
        codec.encodeMethodCall(new MethodCall(method, arguments)),
        callback == null ? null : new IncomingResultHandler(callback));
  }

  /**
   * Registers a method call handler on this channel.
   *
   * <p>Overrides any existing handler registration for (the name of) this channel.
   *
   * <p>If no handler has been registered, any incoming method call on this channel will be handled
   * silently by sending a null reply. This results in a <a
   * href="https://api.flutter.dev/flutter/services/MissingPluginException-class.html">MissingPluginException</a>
   * on the Dart side, unless an <a
   * href="https://api.flutter.dev/flutter/services/OptionalMethodChannel-class.html">OptionalMethodChannel</a>
   * is used.
   *
   * @param handler a {@link MethodCallHandler}, or null to deregister.
   */
  @UiThread
  public void setMethodCallHandler(final @Nullable MethodCallHandler handler) {
    // We call the 2 parameter variant specifically to avoid breaking changes in
    // mock verify calls.
    // See https://github.com/flutter/flutter/issues/92582.
    if (taskQueue != null) {
      messenger.setMessageHandler(
          name, handler == null ? null : new IncomingMethodCallHandler(handler), taskQueue);
    } else {
      messenger.setMessageHandler(
          name, handler == null ? null : new IncomingMethodCallHandler(handler));
    }
  }

  /**
   * Adjusts the number of messages that will get buffered when sending messages to channels that
   * aren't fully set up yet. For example, the engine isn't running yet or the channel's message
   * handler isn't set up on the Dart side yet.
   */
  public void resizeChannelBuffer(int newSize) {
    BasicMessageChannel.resizeChannelBuffer(messenger, name, newSize);
  }

  /**
   * Toggles whether the channel should show warning messages when discarding messages due to
   * overflow. When 'warns' is false the channel is expected to overflow and warning messages will
   * not be shown.
   */
  public void setWarnsOnChannelOverflow(boolean warns) {
    BasicMessageChannel.setWarnsOnChannelOverflow(messenger, name, warns);
  }

  /** A handler of incoming method calls. */
  public interface MethodCallHandler {
    /**
     * Handles the specified method call received from Flutter.
     *
     * <p>Handler implementations must submit a result for all incoming calls, by making a single
     * call on the given {@link Result} callback. Failure to do so will result in lingering Flutter
     * result handlers. The result may be submitted asynchronously and on any thread. Calls to
     * unknown or unimplemented methods should be handled using {@link Result#notImplemented()}.
     *
     * <p>Any uncaught exception thrown by this method will be caught by the channel implementation
     * and logged, and an error result will be sent back to Flutter.
     *
     * <p>The handler is called on the platform thread (Android main thread) by default, or
     * otherwise on the thread specified by the {@link BinaryMessenger.TaskQueue} provided to the
     * associated {@link MethodChannel} when it was created. See also <a
     * href="https://github.com/flutter/flutter/wiki/The-Engine-architecture#threading">Threading in
     * the Flutter Engine</a>.
     *
     * @param call A {@link MethodCall}.
     * @param result A {@link Result} used for submitting the result of the call.
     */
    @UiThread
    void onMethodCall(@NonNull MethodCall call, @NonNull Result result);
  }

  /**
   * Method call result callback. Supports dual use: Implementations of methods to be invoked by
   * Flutter act as clients of this interface for sending results back to Flutter. Invokers of
   * Flutter methods provide implementations of this interface for handling results received from
   * Flutter.
   *
   * <p>All methods of this class can be invoked on any thread.
   */
  public interface Result {
    /**
     * Handles a successful result.
     *
     * @param result The result, possibly null. The result must be an Object type supported by the
     *     codec. For instance, if you are using {@link StandardMessageCodec} (default), please see
     *     its documentation on what types are supported.
     */
    void success(@Nullable Object result);

    /**
     * Handles an error result.
     *
     * @param errorCode An error code String.
     * @param errorMessage A human-readable error message String, possibly null.
     * @param errorDetails Error details, possibly null. The details must be an Object type
     *     supported by the codec. For instance, if you are using {@link StandardMessageCodec}
     *     (default), please see its documentation on what types are supported.
     */
    void error(
        @NonNull String errorCode, @Nullable String errorMessage, @Nullable Object errorDetails);

    /** Handles a call to an unimplemented method. */
    void notImplemented();
  }

  private final class IncomingResultHandler implements BinaryReply {
    private final Result callback;

    IncomingResultHandler(Result callback) {
      this.callback = callback;
    }

    @Override
    @UiThread
    public void reply(ByteBuffer reply) {
      try {
        if (reply == null) {
          callback.notImplemented();
        } else {
          try {
            callback.success(codec.decodeEnvelope(reply));
          } catch (FlutterException e) {
            callback.error(e.code, e.getMessage(), e.details);
          }
        }
      } catch (RuntimeException e) {
        Log.e(TAG + name, "Failed to handle method call result", e);
      }
    }
  }

  private final class IncomingMethodCallHandler implements BinaryMessageHandler {
    private final MethodCallHandler handler;

    IncomingMethodCallHandler(MethodCallHandler handler) {
      this.handler = handler;
    }

    @Override
    @UiThread
    public void onMessage(ByteBuffer message, final BinaryReply reply) {
      final MethodCall call = codec.decodeMethodCall(message);
      try {
        handler.onMethodCall(
            call,
            new Result() {
              @Override
              public void success(Object result) {
                reply.reply(codec.encodeSuccessEnvelope(result));
              }

              @Override
              public void error(String errorCode, String errorMessage, Object errorDetails) {
                reply.reply(codec.encodeErrorEnvelope(errorCode, errorMessage, errorDetails));
              }

              @Override
              public void notImplemented() {
                reply.reply(null);
              }
            });
      } catch (RuntimeException e) {
        Log.e(TAG + name, "Failed to handle method call", e);
        reply.reply(
            codec.encodeErrorEnvelopeWithStacktrace(
                "error", e.getMessage(), null, Log.getStackTraceString(e)));
      }
    }
  }
}
