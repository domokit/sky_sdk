// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.common;

import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import io.flutter.BuildConfig;
import io.flutter.Log;
import io.flutter.plugin.common.BinaryMessenger.BinaryMessageHandler;
import io.flutter.plugin.common.BinaryMessenger.BinaryReply;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * A named channel for communicating with the Flutter application using asynchronous event streams.
 *
 * <p>Incoming requests for event stream setup are decoded from binary on receipt, and Java
 * responses and events are encoded into binary before being transmitted back to Flutter. The {@link
 * MethodCodec} used must be compatible with the one used by the Flutter application. This can be
 * achieved by creating an <a
 * href="https://api.flutter.dev/flutter/services/EventChannel-class.html">EventChannel</a>
 * counterpart of this channel on the Dart side. The Java type of stream configuration arguments,
 * events, and error details is {@code Object}, but only values supported by the specified {@link
 * MethodCodec} can be used.
 *
 * <p>The logical identity of the channel is given by its name. Identically named channels will
 * interfere with each other's communication.
 */
public final class EventChannel {
  private static final String TAG = "EventChannel#";

  private final BinaryMessenger messenger;
  private final String name;
  private final MethodCodec codec;
  @Nullable private final BinaryMessenger.TaskQueue taskQueue;

  /**
   * Creates a new channel associated with the specified {@link BinaryMessenger} and with the
   * specified name and the standard {@link MethodCodec}.
   *
   * @param messenger a {@link BinaryMessenger}.
   * @param name a channel name String.
   */
  public EventChannel(BinaryMessenger messenger, String name) {
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
  public EventChannel(BinaryMessenger messenger, String name, MethodCodec codec) {
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
  public EventChannel(
      BinaryMessenger messenger,
      String name,
      MethodCodec codec,
      BinaryMessenger.TaskQueue taskQueue) {
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
   * Registers a stream handler on this channel.
   *
   * <p>Overrides any existing handler registration for (the name of) this channel.
   *
   * <p>If no handler has been registered, any incoming stream setup requests will be handled
   * silently by providing an empty stream.
   *
   * @param handler a {@link StreamHandler}, or null to deregister.
   */
  @UiThread
  public void setStreamHandler(final StreamHandler handler) {
    // We call the 2 parameter variant specifically to avoid breaking changes in
    // mock verify calls.
    // See https://github.com/flutter/flutter/issues/92582.
    if (taskQueue != null) {
      messenger.setMessageHandler(
          name, handler == null ? null : new IncomingStreamRequestHandler(handler), taskQueue);
    } else {
      messenger.setMessageHandler(
          name, handler == null ? null : new IncomingStreamRequestHandler(handler));
    }
  }

  /**
   * Handler of stream setup and teardown requests.
   *
   * <p>Implementations must be prepared to accept sequences of alternating calls to {@link
   * #onListen(Object, EventChannel.EventSink)} and {@link #onCancel(Object)}. Implementations
   * should ideally consume no resources when the last such call is not {@code onListen}. In typical
   * situations, this means that the implementation should register itself with platform-specific
   * event sources {@code onListen} and deregister again {@code onCancel}.
   */
  public interface StreamHandler {
    /**
     * Handles a request to set up an event stream.
     *
     * <p>Any uncaught exception thrown by this method will be caught by the channel implementation
     * and logged. An error result message will be sent back to Flutter.
     *
     * @param arguments stream configuration arguments, possibly null.
     * @param events an {@link EventSink} for emitting events to the Flutter receiver.
     */
    void onListen(Object arguments, EventSink events);

    /**
     * Handles a request to tear down the most recently created event stream.
     *
     * <p>Any uncaught exception thrown by this method will be caught by the channel implementation
     * and logged. An error result message will be sent back to Flutter.
     *
     * <p>The channel implementation may call this method with null arguments to separate a pair of
     * two consecutive set up requests. Such request pairs may occur during Flutter hot restart. Any
     * uncaught exception thrown in this situation will be logged without notifying Flutter.
     *
     * @param arguments stream configuration arguments, possibly null.
     */
    void onCancel(Object arguments);
  }

  /**
   * Event callback. Supports dual use: Producers of events to be sent to Flutter act as clients of
   * this interface for sending events. Consumers of events sent from Flutter implement this
   * interface for handling received events (the latter facility has not been implemented yet).
   */
  public interface EventSink {
    /**
     * Consumes a successful event.
     *
     * @param event the event, possibly null.
     */
    void success(Object event);

    /**
     * Consumes an error event.
     *
     * @param errorCode an error code String.
     * @param errorMessage a human-readable error message String, possibly null.
     * @param errorDetails error details, possibly null
     */
    void error(String errorCode, String errorMessage, Object errorDetails);

    /**
     * Consumes end of stream. Ensuing calls to {@link #success(Object)} or {@link #error(String,
     * String, Object)}, if any, are ignored.
     */
    void endOfStream();
  }

  private final class IncomingStreamRequestHandler implements BinaryMessageHandler {
    private final StreamHandler handler;
    private final AtomicReference<EventSink> activeSink = new AtomicReference<>(null);

    IncomingStreamRequestHandler(StreamHandler handler) {
      this.handler = handler;
    }

    @Override
    public void onMessage(ByteBuffer message, final BinaryReply reply) {
      final MethodCall call = codec.decodeMethodCall(message);
      if (call.method.equals("listen")) {
        onListen(call.arguments, reply);
      } else if (call.method.equals("cancel")) {
        onCancel(call.arguments, reply);
      } else {
        reply.reply(null);
      }
    }

    private void onListen(Object arguments, BinaryReply callback) {
      final EventSink eventSink = new EventSinkImplementation();
      final EventSink oldSink = activeSink.getAndSet(eventSink);
      if (oldSink != null) {
        // Repeated calls to onListen may happen during hot restart.
        // We separate them with a call to onCancel.
        try {
          handler.onCancel(null);
        } catch (RuntimeException e) {
          Log.e(TAG + name, "Failed to close existing event stream", e);
        }
      }
      try {
        handler.onListen(arguments, eventSink);
        callback.reply(codec.encodeSuccessEnvelope(null));
      } catch (RuntimeException e) {
        activeSink.set(null);
        Log.e(TAG + name, "Failed to open event stream", e);
        callback.reply(codec.encodeErrorEnvelope("error", e.getMessage(), null));
      }
    }

    private void onCancel(Object arguments, BinaryReply callback) {
      final EventSink oldSink = activeSink.getAndSet(null);
      if (oldSink != null) {
        try {
          handler.onCancel(arguments);
          callback.reply(codec.encodeSuccessEnvelope(null));
        } catch (RuntimeException e) {
          Log.e(TAG + name, "Failed to close event stream", e);
          callback.reply(codec.encodeErrorEnvelope("error", e.getMessage(), null));
        }
      } else {
        callback.reply(codec.encodeErrorEnvelope("error", "No active stream to cancel", null));
      }
    }

    private final class EventSinkImplementation implements EventSink {
      final AtomicBoolean hasEnded = new AtomicBoolean(false);

      @Override
      @UiThread
      public void success(Object event) {
        if (hasEnded.get() || activeSink.get() != this) {
          return;
        }
        EventChannel.this.messenger.send(name, codec.encodeSuccessEnvelope(event));
      }

      @Override
      @UiThread
      public void error(String errorCode, String errorMessage, Object errorDetails) {
        if (hasEnded.get() || activeSink.get() != this) {
          return;
        }
        EventChannel.this.messenger.send(
            name, codec.encodeErrorEnvelope(errorCode, errorMessage, errorDetails));
      }

      @Override
      @UiThread
      public void endOfStream() {
        if (hasEnded.getAndSet(true) || activeSink.get() != this) {
          return;
        }
        EventChannel.this.messenger.send(name, null);
      }
    }
  }
}
