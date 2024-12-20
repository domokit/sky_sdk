package io.flutter.embedding.engine.systemchannels;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.BasicMessageChannel;
import io.flutter.plugin.common.StandardMessageCodec;
import io.flutter.view.AccessibilityBridge;
import java.util.HashMap;

/**
 * System channel that sends accessibility requests and events from Flutter to Android.
 *
 * <p>See {@link AccessibilityMessageHandler}, which lists all accessibility requests and events
 * that might be sent from Flutter to the Android platform.
 */
public class AccessibilityChannel {
  private static final String TAG = "AccessibilityChannel";

  @NonNull public final BasicMessageChannel<Object> channel;
  @NonNull public final FlutterJNI flutterJNI;
  @Nullable private AccessibilityMessageHandler handler;

  public final BasicMessageChannel.MessageHandler<Object> parsingMessageHandler =
      new BasicMessageChannel.MessageHandler<Object>() {
        @Override
        public void onMessage(
            @Nullable Object message, @NonNull BasicMessageChannel.Reply<Object> reply) {
          // If there is no handler to respond to this message then we don't need to
          // parse it. Return.
          if (handler == null) {
            reply.reply(null);
            return;
          }

          @SuppressWarnings("unchecked")
          final HashMap<String, Object> annotatedEvent = (HashMap<String, Object>) message;
          final String type = (String) annotatedEvent.get("type");
          @SuppressWarnings("unchecked")
          final HashMap<String, Object> data = (HashMap<String, Object>) annotatedEvent.get("data");

          Log.v(TAG, "Received " + type + " message.");
          switch (type) {
            case "announce":
              String announceMessage = (String) data.get("message");
              if (announceMessage != null) {
                handler.announce(announceMessage);
              }
              break;
            case "tap":
              {
                Integer nodeId = (Integer) annotatedEvent.get("nodeId");
                if (nodeId != null) {
                  handler.onTap(nodeId);
                }
                break;
              }
            case "longPress":
              {
                Integer nodeId = (Integer) annotatedEvent.get("nodeId");
                if (nodeId != null) {
                  handler.onLongPress(nodeId);
                }
                break;
              }
            case "focus":
              {
                Integer nodeId = (Integer) annotatedEvent.get("nodeId");
                if (nodeId != null) {
                  handler.onFocus(nodeId);
                }
                break;
              }
            case "tooltip":
              {
                String tooltipMessage = (String) data.get("message");
                if (tooltipMessage != null) {
                  handler.onTooltip(tooltipMessage);
                }
                break;
              }
          }
          reply.reply(null);
        }
      };

  /**
   * Constructs an {@code AccessibilityChannel} that connects Android to the Dart code running in
   * {@code dartExecutor}.
   *
   * <p>The given {@code dartExecutor} is permitted to be idle or executing code.
   *
   * <p>See {@link DartExecutor}.
   */
  public AccessibilityChannel(@NonNull DartExecutor dartExecutor, @NonNull FlutterJNI flutterJNI) {
    channel =
        new BasicMessageChannel<>(
            dartExecutor, "flutter/accessibility", StandardMessageCodec.INSTANCE);
    channel.setMessageHandler(parsingMessageHandler);
    this.flutterJNI = flutterJNI;
  }

  @VisibleForTesting
  public AccessibilityChannel(
      @NonNull BasicMessageChannel<Object> channel, @NonNull FlutterJNI flutterJNI) {
    this.channel = channel;
    this.flutterJNI = flutterJNI;
  }

  /**
   * Informs Flutter that the Android OS currently has accessibility enabled.
   *
   * <p>To accommodate enabled accessibility, this method instructs Flutter to activate its
   * semantics tree, which forms the basis of Flutter's accessibility support.
   */
  public void onAndroidAccessibilityEnabled() {
    flutterJNI.setSemanticsEnabled(true);
  }

  /**
   * Informs Flutter that the Android OS currently has accessibility disabled.
   *
   * <p>Given that accessibility is not required at this time, this method instructs Flutter to
   * deactivate its semantics tree.
   */
  public void onAndroidAccessibilityDisabled() {
    flutterJNI.setSemanticsEnabled(false);
  }

  /**
   * Instructs Flutter to activate/deactivate accessibility features corresponding to the flags
   * provided by {@code accessibilityFeatureFlags}.
   */
  public void setAccessibilityFeatures(int accessibilityFeatureFlags) {
    flutterJNI.setAccessibilityFeatures(accessibilityFeatureFlags);
  }

  /**
   * Instructs Flutter to perform the given {@code action} on the {@code SemanticsNode} referenced
   * by the given {@code virtualViewId}.
   *
   * <p>One might wonder why Flutter would need to be instructed that the user wants to perform an
   * action. When the user is touching the screen in accessibility mode, Android takes over the
   * touch input, categorizing input as one of a many accessibility gestures. Therefore, Flutter
   * does not have an opportunity to react to said touch input. Instead, Flutter must be notified by
   * Android of the desired action. Additionally, some accessibility systems use other input
   * methods, such as speech, to take virtual actions. Android interprets those requests and then
   * instructs the app to take the appropriate action.
   */
  public void dispatchSemanticsAction(
      int virtualViewId, @NonNull AccessibilityBridge.Action action) {
    flutterJNI.dispatchSemanticsAction(virtualViewId, action);
  }

  /**
   * Instructs Flutter to perform the given {@code action} on the {@code SemanticsNode} referenced
   * by the given {@code virtualViewId}, passing the given {@code args}.
   */
  public void dispatchSemanticsAction(
      int virtualViewId, @NonNull AccessibilityBridge.Action action, @Nullable Object args) {
    flutterJNI.dispatchSemanticsAction(virtualViewId, action, args);
  }

  /**
   * Sets the {@link AccessibilityMessageHandler} which receives all events and requests that are
   * parsed from the underlying accessibility channel.
   */
  public void setAccessibilityMessageHandler(@Nullable AccessibilityMessageHandler handler) {
    this.handler = handler;
    flutterJNI.setAccessibilityDelegate(handler);
  }

  /**
   * Handler that receives accessibility messages sent from Flutter to Android through a given
   * {@link AccessibilityChannel}.
   *
   * <p>To register an {@code AccessibilityMessageHandler} with a {@link AccessibilityChannel}, see
   * {@link AccessibilityChannel#setAccessibilityMessageHandler(AccessibilityMessageHandler)}.
   */
  public interface AccessibilityMessageHandler extends FlutterJNI.AccessibilityDelegate {
    /** The Dart application would like the given {@code message} to be announced. */
    void announce(@NonNull String message);

    /** The user has tapped on the semantics node with the given {@code nodeId}. */
    void onTap(int nodeId);

    /** The user has long pressed on the semantics node with the given {@code nodeId}. */
    void onLongPress(int nodeId);

    /** The framework has requested focus on the semantics node with the given {@code nodeId}. */
    void onFocus(int nodeId);

    /** The user has opened a tooltip. */
    void onTooltip(@NonNull String message);
  }
}
