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
import java.util.Arrays;

/**
 * A named channel for communicating with the Flutter application using basic, asynchronous message
 * passing.
 *
 * <p>Messages are encoded into binary before being sent, and binary messages received are decoded
 * into Java objects. The {@link MessageCodec} used must be compatible with the one used by the
 * Flutter application. This can be achieved by creating a <a
 * href="https://api.flutter.dev/flutter/services/BasicMessageChannel-class.html">BasicMessageChannel</a>
 * counterpart of this channel on the Dart side. The static Java type of messages sent and received
 * is {@code Object}, but only values supported by the specified {@link MessageCodec} can be used.
 *
 * <p>The logical identity of the channel is given by its name. Identically named channels will
 * interfere with each other's communication.
 */
public final class BasicMessageChannel<T> {
  private static final String TAG = "BasicMessageChannel#";
  public static final String CHANNEL_BUFFERS_CHANNEL = "dev.flutter/channel-buffers";

  @NonNull private final BinaryMessenger messenger;
  @NonNull private final String name;
  @NonNull private final MessageCodec<T> codec;
  @Nullable private final BinaryMessenger.TaskQueue taskQueue;

  /**
   * Creates a new channel associated with the specified {@link BinaryMessenger} and with the
   * specified name and {@link MessageCodec}.
   *
   * @param messenger a {@link BinaryMessenger}.
   * @param name a channel name String.
   * @param codec a {@link MessageCodec}.
   */
  public BasicMessageChannel(
      @NonNull BinaryMessenger messenger, @NonNull String name, @NonNull MessageCodec<T> codec) {
    this(messenger, name, codec, null);
  }

  /**
   * Creates a new channel associated with the specified {@link BinaryMessenger} and with the
   * specified name and {@link MessageCodec}.
   *
   * @param messenger a {@link BinaryMessenger}.
   * @param name a channel name String.
   * @param codec a {@link MessageCodec}.
   * @param taskQueue a {@link BinaryMessenger.TaskQueue} that specifies what thread will execute
   *     the handler. Specifying null means execute on the platform thread. See also {@link
   *     BinaryMessenger#makeBackgroundTaskQueue()}.
   */
  public BasicMessageChannel(
      @NonNull BinaryMessenger messenger,
      @NonNull String name,
      @NonNull MessageCodec<T> codec,
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
   * Sends the specified message to the Flutter application on this channel.
   *
   * @param message the message, possibly null.
   */
  public void send(@Nullable T message) {
    send(message, null);
  }

  /**
   * Sends the specified message to the Flutter application, optionally expecting a reply.
   *
   * <p>Any uncaught exception thrown by the reply callback will be caught and logged.
   *
   * @param message the message, possibly null.
   * @param callback a {@link Reply} callback, possibly null.
   */
  @UiThread
  public void send(@Nullable T message, @Nullable final Reply<T> callback) {
    messenger.send(
        name,
        codec.encodeMessage(message),
        callback == null ? null : new IncomingReplyHandler(callback));
  }

  /**
   * Registers a message handler on this channel for receiving messages sent from the Flutter
   * application.
   *
   * <p>Overrides any existing handler registration for (the name of) this channel.
   *
   * <p>If no handler has been registered, any incoming message on this channel will be handled
   * silently by sending a null reply.
   *
   * @param handler a {@link MessageHandler}, or null to deregister.
   */
  @UiThread
  public void setMessageHandler(@Nullable final MessageHandler<T> handler) {
    // We call the 2 parameter variant specifically to avoid breaking changes in
    // mock verify calls.
    // See https://github.com/flutter/flutter/issues/92582.
    if (taskQueue != null) {
      messenger.setMessageHandler(
          name, handler == null ? null : new IncomingMessageHandler(handler), taskQueue);
    } else {
      messenger.setMessageHandler(
          name, handler == null ? null : new IncomingMessageHandler(handler));
    }
  }

  /**
   * Adjusts the number of messages that will get buffered when sending messages to channels that
   * aren't fully set up yet. For example, the engine isn't running yet or the channel's message
   * handler isn't set up on the Dart side yet.
   */
  public void resizeChannelBuffer(int newSize) {
    resizeChannelBuffer(messenger, name, newSize);
  }

  /**
   * Toggles whether the channel should show warning messages when discarding messages due to
   * overflow. When 'warns' is false the channel is expected to overflow and warning messages will
   * not be shown.
   */
  public void setWarnsOnChannelOverflow(boolean warns) {
    setWarnsOnChannelOverflow(messenger, name, warns);
  }

  private static ByteBuffer packetFromEncodedMessage(ByteBuffer message) {
    // Create a bytes array using the buffer content (messages.array() can not be used here).
    message.flip();
    final byte[] bytes = new byte[message.remaining()];
    message.get(bytes);

    // The current Android Java/JNI platform message implementation assumes
    // that all buffers passed to native are direct buffers.
    ByteBuffer packet = ByteBuffer.allocateDirect(bytes.length);
    packet.put(bytes);

    return packet;
  }

  /**
   * Adjusts the number of messages that will get buffered when sending messages to channels that
   * aren't fully set up yet. For example, the engine isn't running yet or the channel's message
   * handler isn't set up on the Dart side yet.
   */
  public static void resizeChannelBuffer(
      @NonNull BinaryMessenger messenger, @NonNull String channel, int newSize) {
    final StandardMethodCodec codec = StandardMethodCodec.INSTANCE;
    Object[] arguments = {channel, newSize};
    MethodCall methodCall = new MethodCall("resize", Arrays.asList(arguments));
    ByteBuffer message = codec.encodeMethodCall(methodCall);
    ByteBuffer packet = packetFromEncodedMessage(message);
    messenger.send(BasicMessageChannel.CHANNEL_BUFFERS_CHANNEL, packet);
  }

  /**
   * Toggles whether the channel should show warning messages when discarding messages due to
   * overflow. When 'warns' is false the channel is expected to overflow and warning messages will
   * not be shown.
   */
  public static void setWarnsOnChannelOverflow(
      @NonNull BinaryMessenger messenger, @NonNull String channel, boolean warns) {
    final StandardMethodCodec codec = StandardMethodCodec.INSTANCE;
    Object[] arguments = {channel, !warns};
    MethodCall methodCall = new MethodCall("overflow", Arrays.asList(arguments));
    ByteBuffer message = codec.encodeMethodCall(methodCall);
    ByteBuffer packet = packetFromEncodedMessage(message);
    messenger.send(BasicMessageChannel.CHANNEL_BUFFERS_CHANNEL, packet);
  }

  /** A handler of incoming messages. */
  public interface MessageHandler<T> {

    /**
     * Handles the specified message received from Flutter.
     *
     * <p>Handler implementations must reply to all incoming messages, by submitting a single reply
     * message to the given {@link Reply}. Failure to do so will result in lingering Flutter reply
     * handlers. The reply may be submitted asynchronously and invoked on any thread.
     *
     * <p>Any uncaught exception thrown by this method, or the preceding message decoding, will be
     * caught by the channel implementation and logged, and a null reply message will be sent back
     * to Flutter.
     *
     * <p>Any uncaught exception thrown during encoding a reply message submitted to the {@link
     * Reply} is treated similarly: the exception is logged, and a null reply is sent to Flutter.
     *
     * @param message the message, possibly null.
     * @param reply a {@link Reply} for sending a single message reply back to Flutter.
     */
    void onMessage(@Nullable T message, @NonNull Reply<T> reply);
  }

  /**
   * Message reply callback. Used to submit a reply to an incoming message from Flutter. Also used
   * in the dual capacity to handle a reply received from Flutter after sending a message.
   */
  public interface Reply<T> {
    /**
     * Handles the specified message reply.
     *
     * @param reply the reply, possibly null.
     */
    void reply(@Nullable T reply);
  }

  private final class IncomingReplyHandler implements BinaryReply {
    private final Reply<T> callback;

    private IncomingReplyHandler(@NonNull Reply<T> callback) {
      this.callback = callback;
    }

    @Override
    public void reply(@Nullable ByteBuffer reply) {
      try {
        callback.reply(codec.decodeMessage(reply));
      } catch (RuntimeException e) {
        Log.e(TAG + name, "Failed to handle message reply", e);
      }
    }
  }

  private final class IncomingMessageHandler implements BinaryMessageHandler {
    private final MessageHandler<T> handler;

    private IncomingMessageHandler(@NonNull MessageHandler<T> handler) {
      this.handler = handler;
    }

    @Override
    public void onMessage(@Nullable ByteBuffer message, @NonNull final BinaryReply callback) {
      try {
        handler.onMessage(
            codec.decodeMessage(message),
            new Reply<T>() {
              @Override
              public void reply(T reply) {
                callback.reply(codec.encodeMessage(reply));
              }
            });
      } catch (RuntimeException e) {
        Log.e(TAG + name, "Failed to handle message", e);
        callback.reply(null);
      }
    }
  }
}
