// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.dart;

import android.content.res.AssetManager;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import io.flutter.FlutterInjector;
import io.flutter.Log;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.StringCodec;
import io.flutter.util.TraceSection;
import io.flutter.view.FlutterCallbackInformation;
import java.nio.ByteBuffer;
import java.util.List;

/**
 * Configures, bootstraps, and starts executing Dart code.
 *
 * <p>To specify a top-level Dart function to execute, use a {@link DartEntrypoint} to tell {@link
 * DartExecutor} where to find the Dart code to execute, and which Dart function to use as the
 * entrypoint. To execute the entrypoint, pass the {@link DartEntrypoint} to {@link
 * #executeDartEntrypoint(DartEntrypoint)}.
 *
 * <p>To specify a Dart callback to execute, use a {@link DartCallback}. A given Dart callback must
 * be registered with the Dart VM to be invoked by a {@link DartExecutor}. To execute the callback,
 * pass the {@link DartCallback} to {@link #executeDartCallback(DartCallback)}.
 *
 * <p>Once started, a {@link DartExecutor} cannot be stopped. The associated Dart code will execute
 * until it completes, or until the {@link io.flutter.embedding.engine.FlutterEngine} that owns this
 * {@link DartExecutor} is destroyed.
 */
public class DartExecutor implements BinaryMessenger {
  private static final String TAG = "DartExecutor";

  @NonNull private final FlutterJNI flutterJNI;
  @NonNull private final AssetManager assetManager;
  @NonNull private final DartMessenger dartMessenger;
  @NonNull private final BinaryMessenger binaryMessenger;
  private boolean isApplicationRunning = false;
  @Nullable private String isolateServiceId;
  @Nullable private IsolateServiceIdListener isolateServiceIdListener;

  private final BinaryMessenger.BinaryMessageHandler isolateChannelMessageHandler =
      new BinaryMessenger.BinaryMessageHandler() {
        @Override
        public void onMessage(ByteBuffer message, final BinaryReply callback) {
          isolateServiceId = StringCodec.INSTANCE.decodeMessage(message);
          if (isolateServiceIdListener != null) {
            isolateServiceIdListener.onIsolateServiceIdAvailable(isolateServiceId);
          }
        }
      };

  public DartExecutor(@NonNull FlutterJNI flutterJNI, @NonNull AssetManager assetManager) {
    this.flutterJNI = flutterJNI;
    this.assetManager = assetManager;
    this.dartMessenger = new DartMessenger(flutterJNI);
    dartMessenger.setMessageHandler("flutter/isolate", isolateChannelMessageHandler);
    this.binaryMessenger = new DefaultBinaryMessenger(dartMessenger);
    // The JNI might already be attached if coming from a spawned engine. If so, correctly report
    // that this DartExecutor is already running.
    if (flutterJNI.isAttached()) {
      isApplicationRunning = true;
    }
  }

  /**
   * Invoked when the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@link
   * DartExecutor} attaches to JNI.
   *
   * <p>When attached to JNI, this {@link DartExecutor} begins handling 2-way communication to/from
   * the Dart execution context. This communication is facilitate via 2 APIs:
   *
   * <ul>
   *   <li>{@link BinaryMessenger}, which sends messages to Dart
   *   <li>{@link PlatformMessageHandler}, which receives messages from Dart
   * </ul>
   */
  public void onAttachedToJNI() {
    Log.v(
        TAG,
        "Attached to JNI. Registering the platform message handler for this Dart execution"
            + " context.");
    flutterJNI.setPlatformMessageHandler(dartMessenger);
  }

  /**
   * Invoked when the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@link
   * DartExecutor} detaches from JNI.
   *
   * <p>When detached from JNI, this {@link DartExecutor} stops handling 2-way communication to/from
   * the Dart execution context.
   */
  public void onDetachedFromJNI() {
    Log.v(
        TAG,
        "Detached from JNI. De-registering the platform message handler for this Dart execution"
            + " context.");
    flutterJNI.setPlatformMessageHandler(null);
  }

  /**
   * Is this {@link DartExecutor} currently executing Dart code?
   *
   * @return true if Dart code is being executed, false otherwise
   */
  public boolean isExecutingDart() {
    return isApplicationRunning;
  }

  /**
   * Starts executing Dart code based on the given {@code dartEntrypoint}.
   *
   * <p>See {@link DartEntrypoint} for configuration options.
   *
   * @param dartEntrypoint specifies which Dart function to run, and where to find it
   */
  public void executeDartEntrypoint(@NonNull DartEntrypoint dartEntrypoint) {
    executeDartEntrypoint(dartEntrypoint, null);
  }

  /**
   * Starts executing Dart code based on the given {@code dartEntrypoint} and the {@code
   * dartEntrypointArgs}.
   *
   * <p>See {@link DartEntrypoint} for configuration options.
   *
   * @param dartEntrypoint specifies which Dart function to run, and where to find it
   * @param dartEntrypointArgs Arguments passed as a list of string to Dart's entrypoint function.
   */
  public void executeDartEntrypoint(
      @NonNull DartEntrypoint dartEntrypoint, @Nullable List<String> dartEntrypointArgs) {
    if (isApplicationRunning) {
      Log.w(TAG, "Attempted to run a DartExecutor that is already running.");
      return;
    }

    try (TraceSection e = TraceSection.scoped("DartExecutor#executeDartEntrypoint")) {
      Log.v(TAG, "Executing Dart entrypoint: " + dartEntrypoint);
      flutterJNI.runBundleAndSnapshotFromLibrary(
          dartEntrypoint.pathToBundle,
          dartEntrypoint.dartEntrypointFunctionName,
          dartEntrypoint.dartEntrypointLibrary,
          assetManager,
          dartEntrypointArgs);

      isApplicationRunning = true;
    }
  }

  /**
   * Starts executing Dart code based on the given {@code dartCallback}.
   *
   * <p>See {@link DartCallback} for configuration options.
   *
   * @param dartCallback specifies which Dart callback to run, and where to find it
   */
  public void executeDartCallback(@NonNull DartCallback dartCallback) {
    if (isApplicationRunning) {
      Log.w(TAG, "Attempted to run a DartExecutor that is already running.");
      return;
    }

    try (TraceSection e = TraceSection.scoped("DartExecutor#executeDartCallback")) {
      Log.v(TAG, "Executing Dart callback: " + dartCallback);
      flutterJNI.runBundleAndSnapshotFromLibrary(
          dartCallback.pathToBundle,
          dartCallback.callbackHandle.callbackName,
          dartCallback.callbackHandle.callbackLibraryPath,
          dartCallback.androidAssetManager,
          null);

      isApplicationRunning = true;
    }
  }

  /**
   * Returns a {@link BinaryMessenger} that can be used to send messages to, and receive messages
   * from, Dart code that this {@code DartExecutor} is executing.
   */
  @NonNull
  public BinaryMessenger getBinaryMessenger() {
    return binaryMessenger;
  }

  // ------ START BinaryMessenger (Deprecated: use getBinaryMessenger() instead) -----
  /** @deprecated Use {@link #getBinaryMessenger()} instead. */
  @Deprecated
  @UiThread
  @Override
  public TaskQueue makeBackgroundTaskQueue(TaskQueueOptions options) {
    return binaryMessenger.makeBackgroundTaskQueue(options);
  }

  /** @deprecated Use {@link #getBinaryMessenger()} instead. */
  @Deprecated
  @Override
  @UiThread
  public void send(@NonNull String channel, @Nullable ByteBuffer message) {
    binaryMessenger.send(channel, message);
  }

  /** @deprecated Use {@link #getBinaryMessenger()} instead. */
  @Deprecated
  @Override
  @UiThread
  public void send(
      @NonNull String channel,
      @Nullable ByteBuffer message,
      @Nullable BinaryMessenger.BinaryReply callback) {
    binaryMessenger.send(channel, message, callback);
  }

  /** @deprecated Use {@link #getBinaryMessenger()} instead. */
  @Deprecated
  @Override
  @UiThread
  public void setMessageHandler(
      @NonNull String channel, @Nullable BinaryMessenger.BinaryMessageHandler handler) {
    binaryMessenger.setMessageHandler(channel, handler);
  }

  /** @deprecated Use {@link #getBinaryMessenger()} instead. */
  @Deprecated
  @Override
  @UiThread
  public void setMessageHandler(
      @NonNull String channel,
      @Nullable BinaryMessenger.BinaryMessageHandler handler,
      @Nullable TaskQueue taskQueue) {
    binaryMessenger.setMessageHandler(channel, handler, taskQueue);
  }

  /** @deprecated Use {@link #getBinaryMessenger()} instead. */
  @Deprecated
  @Override
  public void enableBufferingIncomingMessages() {
    dartMessenger.enableBufferingIncomingMessages();
  }

  /** @deprecated Use {@link #getBinaryMessenger()} instead. */
  @Deprecated
  @Override
  public void disableBufferingIncomingMessages() {
    dartMessenger.disableBufferingIncomingMessages();
  }
  // ------ END BinaryMessenger -----

  /**
   * Returns the number of pending channel callback replies.
   *
   * <p>When sending messages to the Flutter application using {@link BinaryMessenger#send(String,
   * ByteBuffer, io.flutter.plugin.common.BinaryMessenger.BinaryReply)}, developers can optionally
   * specify a reply callback if they expect a reply from the Flutter application.
   *
   * <p>This method tracks all the pending callbacks that are waiting for response, and is supposed
   * to be called from the main thread (as other methods). Calling from a different thread could
   * possibly capture an indeterministic internal state, so don't do it.
   *
   * <p>Currently, it's mainly useful for a testing framework like Espresso to determine whether all
   * the async channel callbacks are handled and the app is idle.
   */
  @UiThread
  public int getPendingChannelResponseCount() {
    return dartMessenger.getPendingChannelResponseCount();
  }

  /**
   * Returns an identifier for this executor's primary isolate. This identifier can be used in
   * queries to the Dart service protocol.
   */
  @Nullable
  public String getIsolateServiceId() {
    return isolateServiceId;
  }

  /** Callback interface invoked when the isolate identifier becomes available. */
  public interface IsolateServiceIdListener {
    void onIsolateServiceIdAvailable(@NonNull String isolateServiceId);
  }

  /**
   * Set a listener that will be notified when an isolate identifier is available for this
   * executor's primary isolate.
   */
  public void setIsolateServiceIdListener(@Nullable IsolateServiceIdListener listener) {
    isolateServiceIdListener = listener;
    if (isolateServiceIdListener != null && isolateServiceId != null) {
      isolateServiceIdListener.onIsolateServiceIdAvailable(isolateServiceId);
    }
  }

  /**
   * Notify the Dart VM of a low memory event, or that the application is in a state such that now
   * is an appropriate time to free resources, such as going to the background.
   *
   * <p>This does not notify a Flutter application about memory pressure. For that, use the {@link
   * io.flutter.embedding.engine.systemchannels.SystemChannel#sendMemoryPressureWarning}.
   *
   * <p>Calling this method may cause jank or latency in the application. Avoid calling it during
   * critical periods like application startup or periods of animation.
   */
  public void notifyLowMemoryWarning() {
    if (flutterJNI.isAttached()) {
      flutterJNI.notifyLowMemoryWarning();
    }
  }

  /**
   * Configuration options that specify which Dart entrypoint function is executed and where to find
   * that entrypoint and other assets required for Dart execution.
   */
  public static class DartEntrypoint {
    /**
     * Create a DartEntrypoint pointing to the default Flutter assets location with a default Dart
     * entrypoint.
     */
    @NonNull
    public static DartEntrypoint createDefault() {
      FlutterLoader flutterLoader = FlutterInjector.instance().flutterLoader();

      if (!flutterLoader.initialized()) {
        throw new AssertionError(
            "DartEntrypoints can only be created once a FlutterEngine is created.");
      }
      return new DartEntrypoint(flutterLoader.findAppBundlePath(), "main");
    }

    /** The path within the AssetManager where the app will look for assets. */
    @NonNull public final String pathToBundle;

    /** The library or file location that contains the Dart entrypoint function. */
    @Nullable public final String dartEntrypointLibrary;

    /** The name of a Dart function to execute. */
    @NonNull public final String dartEntrypointFunctionName;

    public DartEntrypoint(
        @NonNull String pathToBundle, @NonNull String dartEntrypointFunctionName) {
      this.pathToBundle = pathToBundle;
      dartEntrypointLibrary = null;
      this.dartEntrypointFunctionName = dartEntrypointFunctionName;
    }

    public DartEntrypoint(
        @NonNull String pathToBundle,
        @NonNull String dartEntrypointLibrary,
        @NonNull String dartEntrypointFunctionName) {
      this.pathToBundle = pathToBundle;
      this.dartEntrypointLibrary = dartEntrypointLibrary;
      this.dartEntrypointFunctionName = dartEntrypointFunctionName;
    }

    @Override
    @NonNull
    public String toString() {
      return "DartEntrypoint( bundle path: "
          + pathToBundle
          + ", function: "
          + dartEntrypointFunctionName
          + " )";
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (o == null || getClass() != o.getClass()) return false;

      DartEntrypoint that = (DartEntrypoint) o;

      if (!pathToBundle.equals(that.pathToBundle)) return false;
      return dartEntrypointFunctionName.equals(that.dartEntrypointFunctionName);
    }

    @Override
    public int hashCode() {
      int result = pathToBundle.hashCode();
      result = 31 * result + dartEntrypointFunctionName.hashCode();
      return result;
    }
  }

  /**
   * Configuration options that specify which Dart callback function is executed and where to find
   * that callback and other assets required for Dart execution.
   */
  public static class DartCallback {
    /** Standard Android AssetManager, provided from some {@code Context} or {@code Resources}. */
    public final AssetManager androidAssetManager;

    /** The path within the AssetManager where the app will look for assets. */
    public final String pathToBundle;

    /** A Dart callback that was previously registered with the Dart VM. */
    public final FlutterCallbackInformation callbackHandle;

    public DartCallback(
        @NonNull AssetManager androidAssetManager,
        @NonNull String pathToBundle,
        @NonNull FlutterCallbackInformation callbackHandle) {
      this.androidAssetManager = androidAssetManager;
      this.pathToBundle = pathToBundle;
      this.callbackHandle = callbackHandle;
    }

    @Override
    @NonNull
    public String toString() {
      return "DartCallback( bundle path: "
          + pathToBundle
          + ", library path: "
          + callbackHandle.callbackLibraryPath
          + ", function: "
          + callbackHandle.callbackName
          + " )";
    }
  }

  private static class DefaultBinaryMessenger implements BinaryMessenger {
    private final DartMessenger messenger;

    private DefaultBinaryMessenger(@NonNull DartMessenger messenger) {
      this.messenger = messenger;
    }

    public TaskQueue makeBackgroundTaskQueue(TaskQueueOptions options) {
      return messenger.makeBackgroundTaskQueue(options);
    }

    /**
     * Sends the given {@code message} from Android to Dart over the given {@code channel}.
     *
     * @param channel the name of the logical channel used for the message.
     * @param message the message payload, a direct-allocated {@link ByteBuffer} with the message
     *     bytes
     */
    @Override
    @UiThread
    public void send(@NonNull String channel, @Nullable ByteBuffer message) {
      messenger.send(channel, message, null);
    }

    /**
     * Sends the given {@code messages} from Android to Dart over the given {@code channel} and then
     * has the provided {@code callback} invoked when the Dart side responds.
     *
     * @param channel the name of the logical channel used for the message.
     * @param message the message payload, a direct-allocated {@link ByteBuffer} with the message
     *     bytes between position zero and current position, or null.
     * @param callback a callback invoked when the Dart application responds to the message
     */
    @Override
    @UiThread
    public void send(
        @NonNull String channel,
        @Nullable ByteBuffer message,
        @Nullable BinaryMessenger.BinaryReply callback) {
      messenger.send(channel, message, callback);
    }

    /**
     * Sets the given {@link io.flutter.plugin.common.BinaryMessenger.BinaryMessageHandler} as the
     * singular handler for all incoming messages received from the Dart side of this Dart execution
     * context.
     *
     * @param channel the name of the channel.
     * @param handler a {@link BinaryMessageHandler} to be invoked on incoming messages, or null.
     */
    @Override
    @UiThread
    public void setMessageHandler(
        @NonNull String channel, @Nullable BinaryMessenger.BinaryMessageHandler handler) {
      messenger.setMessageHandler(channel, handler);
    }

    @Override
    @UiThread
    public void setMessageHandler(
        @NonNull String channel,
        @Nullable BinaryMessenger.BinaryMessageHandler handler,
        @Nullable TaskQueue taskQueue) {
      messenger.setMessageHandler(channel, handler, taskQueue);
    }

    @Override
    public void enableBufferingIncomingMessages() {
      messenger.enableBufferingIncomingMessages();
    }

    @Override
    public void disableBufferingIncomingMessages() {
      messenger.disableBufferingIncomingMessages();
    }
  }
}
