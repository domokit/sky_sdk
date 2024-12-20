// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.view;

import static io.flutter.Build.API_LEVELS;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.app.Activity;
import android.content.ContentResolver;
import android.content.Context;
import android.content.res.Configuration;
import android.database.ContentObserver;
import android.graphics.Rect;
import android.net.Uri;
import android.opengl.Matrix;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.provider.Settings;
import android.text.SpannableString;
import android.text.TextUtils;
import android.text.style.LocaleSpan;
import android.text.style.TtsSpan;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowInsets;
import android.view.WindowManager;
import android.view.accessibility.AccessibilityEvent;
import android.view.accessibility.AccessibilityManager;
import android.view.accessibility.AccessibilityNodeInfo;
import android.view.accessibility.AccessibilityNodeProvider;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.annotation.VisibleForTesting;
import io.flutter.BuildConfig;
import io.flutter.Log;
import io.flutter.embedding.engine.systemchannels.AccessibilityChannel;
import io.flutter.plugin.platform.PlatformViewsAccessibilityDelegate;
import io.flutter.util.Predicate;
import io.flutter.util.ViewUtils;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Bridge between Android's OS accessibility system and Flutter's accessibility system.
 *
 * <p>An {@code AccessibilityBridge} requires:
 *
 * <ul>
 *   <li>A real Android {@link View}, called the {@link #rootAccessibilityView}, which contains a
 *       Flutter UI. The {@link #rootAccessibilityView} is required at the time of {@code
 *       AccessibilityBridge}'s instantiation and is held for the duration of {@code
 *       AccessibilityBridge}'s lifespan. {@code AccessibilityBridge} invokes various accessibility
 *       methods on the {@link #rootAccessibilityView}, e.g., {@link
 *       View#onInitializeAccessibilityNodeInfo(AccessibilityNodeInfo)}. The {@link
 *       #rootAccessibilityView} is expected to notify the {@code AccessibilityBridge} of relevant
 *       interactions: {@link #onAccessibilityHoverEvent(MotionEvent)}, {@link #reset()}, {@link
 *       #updateSemantics(ByteBuffer, String[], ByteBuffer[])}, and {@link
 *       #updateCustomAccessibilityActions(ByteBuffer, String[])}
 *   <li>An {@link AccessibilityChannel} that is connected to the running Flutter app.
 *   <li>Android's {@link AccessibilityManager} to query and listen for accessibility settings.
 *   <li>Android's {@link ContentResolver} to listen for changes to system animation settings.
 * </ul>
 *
 * The {@code AccessibilityBridge} causes Android to treat Flutter {@code SemanticsNode}s as if they
 * were accessible Android {@link View}s. Accessibility requests may be sent from a Flutter widget
 * to the Android OS, as if it were an Android {@link View}, and accessibility events may be
 * consumed by a Flutter widget, as if it were an Android {@link View}. {@code AccessibilityBridge}
 * refers to Flutter's accessible widgets as "virtual views" and identifies them with "virtual view
 * IDs".
 */
public class AccessibilityBridge extends AccessibilityNodeProvider {
  private static final String TAG = "AccessibilityBridge";

  // Constants from higher API levels.
  // TODO(goderbauer): Get these from Android Support Library when
  // https://github.com/flutter/flutter/issues/11099 is resolved.
  private static final int ACTION_SHOW_ON_SCREEN = 16908342; // API level 23

  private static final float SCROLL_EXTENT_FOR_INFINITY = 100000.0f;
  private static final float SCROLL_POSITION_CAP_FOR_INFINITY = 70000.0f;
  private static final int ROOT_NODE_ID = 0;
  private static final int SCROLLABLE_ACTIONS =
      Action.SCROLL_RIGHT.value
          | Action.SCROLL_LEFT.value
          | Action.SCROLL_UP.value
          | Action.SCROLL_DOWN.value;
  // Flags that make a node accessibilty focusable.
  private static final int FOCUSABLE_FLAGS =
      Flag.HAS_CHECKED_STATE.value
          | Flag.IS_CHECKED.value
          | Flag.IS_SELECTED.value
          | Flag.IS_TEXT_FIELD.value
          | Flag.IS_FOCUSED.value
          | Flag.HAS_ENABLED_STATE.value
          | Flag.IS_ENABLED.value
          | Flag.IS_IN_MUTUALLY_EXCLUSIVE_GROUP.value
          | Flag.HAS_TOGGLED_STATE.value
          | Flag.IS_TOGGLED.value
          | Flag.IS_FOCUSABLE.value
          | Flag.IS_SLIDER.value;

  // The minimal ID for an engine generated AccessibilityNodeInfo.
  //
  // The AccessibilityNodeInfo node IDs are generated by the framework for most Flutter semantic
  // nodes.
  // When embedding platform views, the framework does not have the accessibility information for
  // the embedded view;
  // in this case the engine generates AccessibilityNodeInfo that mirrors the a11y information
  // exposed by the platform
  // view. To avoid the need of synchronizing the framework and engine mechanisms for generating the
  // next ID, we split
  // the 32bit range of virtual node IDs into 2. The least significant 16 bits are used for
  // framework generated IDs
  // and the most significant 16 bits are used for engine generated IDs.
  private static final int MIN_ENGINE_GENERATED_NODE_ID = 1 << 16;

  // Font weight adjustment for bold text. FontWeight.Bold - FontWeight.Normal = w700 - w400 = 300.
  private static final int BOLD_TEXT_WEIGHT_ADJUSTMENT = 300;

  // Default transition animation scale (animations enabled)
  private static final float DEFAULT_TRANSITION_ANIMATION_SCALE = 1.0f;

  // Transition animation scale when animations are disabled
  private static final float DISABLED_TRANSITION_ANIMATION_SCALE = 0.0f;

  /// Value is derived from ACTION_TYPE_MASK in AccessibilityNodeInfo.java
  private static int FIRST_RESOURCE_ID = 267386881;

  // Real Android View, which internally holds a Flutter UI.
  @NonNull private final View rootAccessibilityView;

  // The accessibility communication API between Flutter's Android embedding and
  // the Flutter framework.
  @NonNull private final AccessibilityChannel accessibilityChannel;

  // Android's {@link AccessibilityManager}, which we can query to see if accessibility is
  // turned on, as well as listen for changes to accessibility's activation.
  @NonNull private final AccessibilityManager accessibilityManager;

  @NonNull private final AccessibilityViewEmbedder accessibilityViewEmbedder;

  // The delegate for interacting with embedded platform views. Used to embed accessibility data for
  // an embedded view in the accessibility tree.
  @NonNull private final PlatformViewsAccessibilityDelegate platformViewsAccessibilityDelegate;

  // Android's {@link ContentResolver}, which is used to observe the global
  // TRANSITION_ANIMATION_SCALE,
  // which determines whether Flutter's animations should be enabled or disabled for accessibility
  // purposes.
  @NonNull private final ContentResolver contentResolver;

  // The entire Flutter semantics tree of the running Flutter app, stored as a Map
  // from each SemanticsNode's ID to a Java representation of a Flutter SemanticsNode.
  //
  // Flutter's semantics tree is cached here because Android might ask for information about
  // a given SemanticsNode at any moment in time. Caching the tree allows for immediate
  // response to Android's request.
  //
  // The structure of flutterSemanticsTree may be 1 or 2 frames behind the Flutter app
  // due to the time required to communicate tree changes from Flutter to Android.
  //
  // See the Flutter docs on SemanticsNode:
  // https://api.flutter.dev/flutter/semantics/SemanticsNode-class.html
  @NonNull private final Map<Integer, SemanticsNode> flutterSemanticsTree = new HashMap<>();

  // The set of all custom Flutter accessibility actions that are present in the running
  // Flutter app, stored as a Map from each action's ID to the definition of the custom
  // accessibility
  // action.
  //
  // Flutter and Android support a number of built-in accessibility actions. However, these
  // predefined actions are not always sufficient for a desired interaction. Android facilitates
  // custom accessibility actions,
  // https://developer.android.com/reference/android/view/accessibility/AccessibilityNodeInfo.AccessibilityAction.
  // Flutter supports custom accessibility actions via {@code customSemanticsActions} within
  // a {@code Semantics} widget, https://api.flutter.dev/flutter/widgets/Semantics-class.html.
  // {@code customAccessibilityActions} are an Android-side cache of all custom accessibility
  // types declared within the running Flutter app.
  //
  // Custom accessibility actions are comprised of only a few fields, and therefore it is likely
  // that a given app may define the same custom accessibility action many times. Identical
  // custom accessibility actions are de-duped such that {@code customAccessibilityActions} only
  // caches unique custom accessibility actions.
  //
  // See the Android documentation for custom accessibility actions:
  // https://developer.android.com/reference/android/view/accessibility/AccessibilityNodeInfo.AccessibilityAction
  //
  // See the Flutter documentation for the Semantics widget:
  // https://api.flutter.dev/flutter/widgets/Semantics-class.html
  @NonNull
  private final Map<Integer, CustomAccessibilityAction> customAccessibilityActions =
      new HashMap<>();

  // The {@code SemanticsNode} within Flutter that currently has the focus of Android's
  // accessibility system.
  //
  // This is null when a node embedded by the AccessibilityViewEmbedder has the focus.
  @Nullable private SemanticsNode accessibilityFocusedSemanticsNode;

  // The virtual ID of the currently embedded node with accessibility focus.
  //
  // This is the ID of a node generated by the AccessibilityViewEmbedder if an embedded node is
  // focused,
  // null otherwise.
  private Integer embeddedAccessibilityFocusedNodeId;

  // The virtual ID of the currently embedded node with input focus.
  //
  // This is the ID of a node generated by the AccessibilityViewEmbedder if an embedded node is
  // focused,
  // null otherwise.
  private Integer embeddedInputFocusedNodeId;

  // The accessibility features that should currently be active within Flutter, represented as
  // a bitmask whose values comes from {@link AccessibilityFeature}.
  private int accessibilityFeatureFlags = 0;

  // The {@code SemanticsNode} within Flutter that currently has the focus of Android's input
  // system.
  //
  // Input focus is independent of accessibility focus. It is possible that accessibility focus
  // and input focus target the same {@code SemanticsNode}, but it is also possible that one
  // {@code SemanticsNode} has input focus while a different {@code SemanticsNode} has
  // accessibility focus. For example, a user may use a D-Pad to navigate to a text field, giving
  // it accessibility focus, and then enable input on that text field, giving it input focus. Then
  // the user moves the accessibility focus to a nearby label to get info about the label, while
  // maintaining input focus on the original text field.
  @Nullable private SemanticsNode inputFocusedSemanticsNode;

  // Keeps track of the last semantics node that had the input focus.
  //
  // This is used to determine if the input focus has changed since the last time the
  // {@code inputFocusSemanticsNode} has been set, so that we can send a {@code TYPE_VIEW_FOCUSED}
  // event when it changes.
  @Nullable private SemanticsNode lastInputFocusedSemanticsNode;

  // The widget within Flutter that currently sits beneath a cursor, e.g,
  // beneath a stylus or mouse cursor.
  @Nullable private SemanticsNode hoveredObject;

  @VisibleForTesting
  public int getHoveredObjectId() {
    return hoveredObject.id;
  }

  // A Java/Android cached representation of the Flutter app's navigation stack. The Flutter
  // navigation stack is tracked so that accessibility announcements can be made during Flutter's
  // navigation changes.
  // TODO(mattcarroll): take this cache into account for new routing solution so accessibility does
  //                    not get left behind.
  @NonNull private final List<Integer> flutterNavigationStack = new ArrayList<>();

  // TODO(mattcarroll): why do we need previouseRouteId if we have flutterNavigationStack
  private int previousRouteId = ROOT_NODE_ID;

  // Tracks the left system inset of the screen because Flutter needs to manually adjust
  // accessibility positioning when in reverse-landscape. This is an Android bug that Flutter
  // is solving for itself.
  @NonNull private Integer lastLeftFrameInset = 0;

  @Nullable private OnAccessibilityChangeListener onAccessibilityChangeListener;

  // Whether the users are using assistive technologies to interact with the devices.
  //
  // The getter returns true when at least one of the assistive technologies is running:
  // TalkBack, SwitchAccess, or VoiceAccess.
  @VisibleForTesting
  public boolean getAccessibleNavigation() {
    return accessibleNavigation;
  }

  private boolean accessibleNavigation = false;

  private void setAccessibleNavigation(boolean value) {
    if (accessibleNavigation == value) {
      return;
    }
    accessibleNavigation = value;
    if (accessibleNavigation) {
      accessibilityFeatureFlags |= AccessibilityFeature.ACCESSIBLE_NAVIGATION.value;
    } else {
      accessibilityFeatureFlags &= ~AccessibilityFeature.ACCESSIBLE_NAVIGATION.value;
    }
    sendLatestAccessibilityFlagsToFlutter();
  }

  // Set to true after {@code release} has been invoked.
  private boolean isReleased = false;

  // Handler for all messages received from Flutter via the {@code accessibilityChannel}
  private final AccessibilityChannel.AccessibilityMessageHandler accessibilityMessageHandler =
      new AccessibilityChannel.AccessibilityMessageHandler() {
        /** The Dart application would like the given {@code message} to be announced. */
        @Override
        public void announce(@NonNull String message) {
          rootAccessibilityView.announceForAccessibility(message);
        }

        /** The user has tapped on the widget with the given {@code nodeId}. */
        @Override
        public void onTap(int nodeId) {
          sendAccessibilityEvent(nodeId, AccessibilityEvent.TYPE_VIEW_CLICKED);
        }

        /** The user has long pressed on the widget with the given {@code nodeId}. */
        @Override
        public void onLongPress(int nodeId) {
          sendAccessibilityEvent(nodeId, AccessibilityEvent.TYPE_VIEW_LONG_CLICKED);
        }

        /** The framework has requested focus on the given {@code nodeId}. */
        @Override
        public void onFocus(int nodeId) {
          sendAccessibilityEvent(nodeId, AccessibilityEvent.TYPE_VIEW_FOCUSED);
        }

        /** The user has opened a tooltip. */
        @Override
        public void onTooltip(@NonNull String message) {
          // Native Android tooltip is no longer announced when it pops up after API 28 and is
          // handled by
          // AccessibilityNodeInfo.setTooltipText instead.
          //
          // To reproduce native behavior, see
          // https://developer.android.com/guide/topics/ui/tooltips.
          if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
            return;
          }
          AccessibilityEvent e =
              obtainAccessibilityEvent(ROOT_NODE_ID, AccessibilityEvent.TYPE_WINDOW_STATE_CHANGED);
          e.getText().add(message);
          sendAccessibilityEvent(e);
        }

        /** New custom accessibility actions exist in Flutter. Update our Android-side cache. */
        @Override
        public void updateCustomAccessibilityActions(ByteBuffer buffer, String[] strings) {
          buffer.order(ByteOrder.LITTLE_ENDIAN);
          AccessibilityBridge.this.updateCustomAccessibilityActions(buffer, strings);
        }

        /** Flutter's semantics tree has changed. Update our Android-side cache. */
        @Override
        public void updateSemantics(
            ByteBuffer buffer, String[] strings, ByteBuffer[] stringAttributeArgs) {
          buffer.order(ByteOrder.LITTLE_ENDIAN);
          for (ByteBuffer args : stringAttributeArgs) {
            args.order(ByteOrder.LITTLE_ENDIAN);
          }
          AccessibilityBridge.this.updateSemantics(buffer, strings, stringAttributeArgs);
        }
      };

  // Listener that is notified when accessibility is turned on/off.
  private final AccessibilityManager.AccessibilityStateChangeListener
      accessibilityStateChangeListener =
          new AccessibilityManager.AccessibilityStateChangeListener() {
            @Override
            public void onAccessibilityStateChanged(boolean accessibilityEnabled) {
              if (isReleased) {
                return;
              }
              if (accessibilityEnabled) {
                accessibilityChannel.setAccessibilityMessageHandler(accessibilityMessageHandler);
                accessibilityChannel.onAndroidAccessibilityEnabled();
              } else {
                setAccessibleNavigation(false);
                accessibilityChannel.setAccessibilityMessageHandler(null);
                accessibilityChannel.onAndroidAccessibilityDisabled();
              }

              if (onAccessibilityChangeListener != null) {
                onAccessibilityChangeListener.onAccessibilityChanged(
                    accessibilityEnabled, accessibilityManager.isTouchExplorationEnabled());
              }
            }
          };

  // Listener that is notified when accessibility touch exploration is turned on/off.
  // This is guarded at instantiation time.
  private final AccessibilityManager.TouchExplorationStateChangeListener
      touchExplorationStateChangeListener;

  // Listener that is notified when the global TRANSITION_ANIMATION_SCALE. When this scale goes
  // to zero, we instruct Flutter to disable animations.
  private final ContentObserver animationScaleObserver =
      new ContentObserver(new Handler()) {
        @Override
        public void onChange(boolean selfChange) {
          this.onChange(selfChange, null);
        }

        @Override
        public void onChange(boolean selfChange, Uri uri) {
          if (isReleased) {
            return;
          }
          // Retrieve the current value of TRANSITION_ANIMATION_SCALE from the OS.
          float value =
              Settings.Global.getFloat(
                  contentResolver,
                  Settings.Global.TRANSITION_ANIMATION_SCALE,
                  DEFAULT_TRANSITION_ANIMATION_SCALE);

          boolean shouldAnimationsBeDisabled = value == DISABLED_TRANSITION_ANIMATION_SCALE;
          if (shouldAnimationsBeDisabled) {
            accessibilityFeatureFlags |= AccessibilityFeature.DISABLE_ANIMATIONS.value;
          } else {
            accessibilityFeatureFlags &= ~AccessibilityFeature.DISABLE_ANIMATIONS.value;
          }
          sendLatestAccessibilityFlagsToFlutter();
        }
      };

  public AccessibilityBridge(
      @NonNull View rootAccessibilityView,
      @NonNull AccessibilityChannel accessibilityChannel,
      @NonNull AccessibilityManager accessibilityManager,
      @NonNull ContentResolver contentResolver,
      @NonNull PlatformViewsAccessibilityDelegate platformViewsAccessibilityDelegate) {
    this(
        rootAccessibilityView,
        accessibilityChannel,
        accessibilityManager,
        contentResolver,
        new AccessibilityViewEmbedder(rootAccessibilityView, MIN_ENGINE_GENERATED_NODE_ID),
        platformViewsAccessibilityDelegate);
  }

  @VisibleForTesting
  public AccessibilityBridge(
      @NonNull View rootAccessibilityView,
      @NonNull AccessibilityChannel accessibilityChannel,
      @NonNull AccessibilityManager accessibilityManager,
      @NonNull ContentResolver contentResolver,
      @NonNull AccessibilityViewEmbedder accessibilityViewEmbedder,
      @NonNull PlatformViewsAccessibilityDelegate platformViewsAccessibilityDelegate) {
    this.rootAccessibilityView = rootAccessibilityView;
    this.accessibilityChannel = accessibilityChannel;
    this.accessibilityManager = accessibilityManager;
    this.contentResolver = contentResolver;
    this.accessibilityViewEmbedder = accessibilityViewEmbedder;
    this.platformViewsAccessibilityDelegate = platformViewsAccessibilityDelegate;
    // Tell Flutter whether accessibility is initially active or not. Then register a listener
    // to be notified of changes in the future.
    accessibilityStateChangeListener.onAccessibilityStateChanged(accessibilityManager.isEnabled());
    this.accessibilityManager.addAccessibilityStateChangeListener(accessibilityStateChangeListener);

    // Tell Flutter whether touch exploration is initially active or not. Then register a listener
    // to be notified of changes in the future.
    touchExplorationStateChangeListener =
        new AccessibilityManager.TouchExplorationStateChangeListener() {
          @Override
          public void onTouchExplorationStateChanged(boolean isTouchExplorationEnabled) {
            if (isReleased) {
              return;
            }
            if (!isTouchExplorationEnabled) {
              setAccessibleNavigation(false);
              onTouchExplorationExit();
            }

            if (onAccessibilityChangeListener != null) {
              onAccessibilityChangeListener.onAccessibilityChanged(
                  accessibilityManager.isEnabled(), isTouchExplorationEnabled);
            }
          }
        };
    touchExplorationStateChangeListener.onTouchExplorationStateChanged(
        accessibilityManager.isTouchExplorationEnabled());
    this.accessibilityManager.addTouchExplorationStateChangeListener(
        touchExplorationStateChangeListener);

    // Tell Flutter whether animations should initially be enabled or disabled. Then register a
    // listener to be notified of changes in the future.
    animationScaleObserver.onChange(false);
    Uri transitionUri = Settings.Global.getUriFor(Settings.Global.TRANSITION_ANIMATION_SCALE);
    this.contentResolver.registerContentObserver(transitionUri, false, animationScaleObserver);

    // Tells Flutter whether the text should be bolded or not. If the user changes bold text
    // setting, the configuration will change and trigger a re-build of the accesibiltyBridge.
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_31) {
      setBoldTextFlag();
    }

    platformViewsAccessibilityDelegate.attachAccessibilityBridge(this);
  }

  /**
   * Disconnects any listeners and/or delegates that were initialized in {@code
   * AccessibilityBridge}'s constructor, or added after.
   *
   * <p>Do not use this instance after invoking {@code release}. The behavior of any method invoked
   * on this {@code AccessibilityBridge} after invoking {@code release()} is undefined.
   */
  public void release() {
    isReleased = true;
    platformViewsAccessibilityDelegate.detachAccessibilityBridge();
    setOnAccessibilityChangeListener(null);
    accessibilityManager.removeAccessibilityStateChangeListener(accessibilityStateChangeListener);
    accessibilityManager.removeTouchExplorationStateChangeListener(
        touchExplorationStateChangeListener);
    contentResolver.unregisterContentObserver(animationScaleObserver);
    accessibilityChannel.setAccessibilityMessageHandler(null);
  }

  /** Returns true if the Android OS currently has accessibility enabled, false otherwise. */
  public boolean isAccessibilityEnabled() {
    return accessibilityManager.isEnabled();
  }

  /** Returns true if the Android OS currently has touch exploration enabled, false otherwise. */
  public boolean isTouchExplorationEnabled() {
    return accessibilityManager.isTouchExplorationEnabled();
  }

  /**
   * Sets a listener on this {@code AccessibilityBridge}, which is notified whenever accessibility
   * activation, or touch exploration activation changes.
   */
  public void setOnAccessibilityChangeListener(@Nullable OnAccessibilityChangeListener listener) {
    this.onAccessibilityChangeListener = listener;
  }

  /** Sends the current value of {@link #accessibilityFeatureFlags} to Flutter. */
  private void sendLatestAccessibilityFlagsToFlutter() {
    accessibilityChannel.setAccessibilityFeatures(accessibilityFeatureFlags);
  }

  private boolean shouldSetCollectionInfo(final SemanticsNode semanticsNode) {
    // TalkBack expects a number of rows and/or columns greater than 0 to announce
    // in list and out of list.  For an infinite or growing list, you have to
    // specify something > 0 to get "in list" announcements.
    // TalkBack will also only track one list at a time, so we only want to set this
    // for a list that contains the current a11y focused semanticsNode - otherwise, if there
    // are two lists or nested lists, we may end up with announcements for only the last
    // one that is currently available in the semantics tree.  However, we also want
    // to set it if we're exiting a list to a non-list, so that we can get the "out of list"
    // announcement when A11y focus moves out of a list and not into another list.
    return semanticsNode.scrollChildren > 0
        && (SemanticsNode.nullableHasAncestor(
                accessibilityFocusedSemanticsNode, o -> o == semanticsNode)
            || !SemanticsNode.nullableHasAncestor(
                accessibilityFocusedSemanticsNode, o -> o.hasFlag(Flag.HAS_IMPLICIT_SCROLLING)));
  }

  @TargetApi(API_LEVELS.API_31)
  @RequiresApi(API_LEVELS.API_31)
  private void setBoldTextFlag() {
    if (rootAccessibilityView == null || rootAccessibilityView.getResources() == null) {
      return;
    }
    int fontWeightAdjustment =
        rootAccessibilityView.getResources().getConfiguration().fontWeightAdjustment;
    boolean shouldBold =
        fontWeightAdjustment != Configuration.FONT_WEIGHT_ADJUSTMENT_UNDEFINED
            && fontWeightAdjustment >= BOLD_TEXT_WEIGHT_ADJUSTMENT;

    if (shouldBold) {
      accessibilityFeatureFlags |= AccessibilityFeature.BOLD_TEXT.value;
    } else {
      accessibilityFeatureFlags &= ~AccessibilityFeature.BOLD_TEXT.value;
    }
    sendLatestAccessibilityFlagsToFlutter();
  }

  @VisibleForTesting
  public AccessibilityNodeInfo obtainAccessibilityNodeInfo(View rootView) {
    return AccessibilityNodeInfo.obtain(rootView);
  }

  @VisibleForTesting
  public AccessibilityNodeInfo obtainAccessibilityNodeInfo(View rootView, int virtualViewId) {
    return AccessibilityNodeInfo.obtain(rootView, virtualViewId);
  }

  /**
   * Returns {@link AccessibilityNodeInfo} for the view corresponding to the given {@code
   * virtualViewId}.
   *
   * <p>This method is invoked by Android's accessibility system when Android needs accessibility
   * info for a given view.
   *
   * <p>When a {@code virtualViewId} of {@link View#NO_ID} is requested, accessibility node info is
   * returned for our {@link #rootAccessibilityView}. Otherwise, Flutter's semantics tree,
   * represented by {@link #flutterSemanticsTree}, is searched for a {@link SemanticsNode} with the
   * given {@code virtualViewId}. If no such {@link SemanticsNode} is found, then this method
   * returns null. If the desired {@link SemanticsNode} is found, then an {@link
   * AccessibilityNodeInfo} is obtained from the {@link #rootAccessibilityView}, filled with
   * appropriate info, and then returned.
   *
   * <p>Depending on the type of Flutter {@code SemanticsNode} that is requested, the returned
   * {@link AccessibilityNodeInfo} pretends that the {@code SemanticsNode} in question comes from a
   * specialize Android view, e.g., {@link Flag#IS_TEXT_FIELD} maps to {@code
   * android.widget.EditText}, {@link Flag#IS_BUTTON} maps to {@code android.widget.Button}, and
   * {@link Flag#IS_IMAGE} maps to {@code android.widget.ImageView}. In the case that no specialized
   * view applies, the returned {@link AccessibilityNodeInfo} pretends that it represents a {@code
   * android.view.View}.
   */
  @Override
  @SuppressWarnings("deprecation")
  // Suppressing Lint warning for new API, as we are version guarding all calls to newer APIs
  @SuppressLint("NewApi")
  public AccessibilityNodeInfo createAccessibilityNodeInfo(int virtualViewId) {
    setAccessibleNavigation(true);
    if (virtualViewId >= MIN_ENGINE_GENERATED_NODE_ID) {
      // The node is in the engine generated range, and is provided by the accessibility view
      // embedder.
      return accessibilityViewEmbedder.createAccessibilityNodeInfo(virtualViewId);
    }

    if (virtualViewId == View.NO_ID) {
      AccessibilityNodeInfo result = obtainAccessibilityNodeInfo(rootAccessibilityView);
      rootAccessibilityView.onInitializeAccessibilityNodeInfo(result);
      // TODO(mattcarroll): what does it mean for the semantics tree to contain or not contain
      //                    the root node ID?
      if (flutterSemanticsTree.containsKey(ROOT_NODE_ID)) {
        result.addChild(rootAccessibilityView, ROOT_NODE_ID);
      }
      if (Build.VERSION.SDK_INT >= API_LEVELS.API_24) {
        result.setImportantForAccessibility(false);
      }
      return result;
    }

    SemanticsNode semanticsNode = flutterSemanticsTree.get(virtualViewId);
    if (semanticsNode == null) {
      return null;
    }

    // Generate accessibility node for platform views using a virtual display.
    //
    // In this case, register the accessibility node in the view embedder,
    // so the accessibility tree can be mirrored as a subtree of the Flutter accessibility tree.
    // This is in constrast to hybrid composition where the embedded view is in the view hiearchy,
    // so it doesn't need to be mirrored.
    //
    // See the case down below for how hybrid composition is handled.
    if (semanticsNode.platformViewId != -1) {
      if (platformViewsAccessibilityDelegate.usesVirtualDisplay(semanticsNode.platformViewId)) {
        View embeddedView =
            platformViewsAccessibilityDelegate.getPlatformViewById(semanticsNode.platformViewId);
        if (embeddedView == null) {
          return null;
        }
        Rect bounds = semanticsNode.getGlobalRect();
        return accessibilityViewEmbedder.getRootNode(embeddedView, semanticsNode.id, bounds);
      }
    }

    AccessibilityNodeInfo result =
        obtainAccessibilityNodeInfo(rootAccessibilityView, virtualViewId);

    // Accessibility Scanner uses isImportantForAccessibility to decide whether to check
    // or skip this node.
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_24) {
      result.setImportantForAccessibility(isImportant(semanticsNode));
    }

    // Work around for https://github.com/flutter/flutter/issues/21030
    result.setViewIdResourceName("");
    if (semanticsNode.identifier != null) {
      result.setViewIdResourceName(semanticsNode.identifier);
    }
    result.setPackageName(rootAccessibilityView.getContext().getPackageName());
    result.setClassName("android.view.View");
    result.setSource(rootAccessibilityView, virtualViewId);
    result.setFocusable(semanticsNode.isFocusable());
    if (inputFocusedSemanticsNode != null) {
      result.setFocused(inputFocusedSemanticsNode.id == virtualViewId);
    }

    if (accessibilityFocusedSemanticsNode != null) {
      result.setAccessibilityFocused(accessibilityFocusedSemanticsNode.id == virtualViewId);
    }

    if (semanticsNode.hasFlag(Flag.IS_TEXT_FIELD)) {
      result.setPassword(semanticsNode.hasFlag(Flag.IS_OBSCURED));
      if (!semanticsNode.hasFlag(Flag.IS_READ_ONLY)) {
        result.setClassName("android.widget.EditText");
      }
      result.setEditable(!semanticsNode.hasFlag(Flag.IS_READ_ONLY));
      if (semanticsNode.textSelectionBase != -1 && semanticsNode.textSelectionExtent != -1) {
        result.setTextSelection(semanticsNode.textSelectionBase, semanticsNode.textSelectionExtent);
      }
      // Text fields will always be created as a live region when they have input focus,
      // so that updates to the label trigger polite announcements. This makes it easy to
      // follow a11y guidelines for text fields on Android.
      if (accessibilityFocusedSemanticsNode != null
          && accessibilityFocusedSemanticsNode.id == virtualViewId) {
        result.setLiveRegion(View.ACCESSIBILITY_LIVE_REGION_POLITE);
      }

      // Cursor movements
      int granularities = 0;
      if (semanticsNode.hasAction(Action.MOVE_CURSOR_FORWARD_BY_CHARACTER)) {
        result.addAction(AccessibilityNodeInfo.ACTION_NEXT_AT_MOVEMENT_GRANULARITY);
        granularities |= AccessibilityNodeInfo.MOVEMENT_GRANULARITY_CHARACTER;
      }
      if (semanticsNode.hasAction(Action.MOVE_CURSOR_BACKWARD_BY_CHARACTER)) {
        result.addAction(AccessibilityNodeInfo.ACTION_PREVIOUS_AT_MOVEMENT_GRANULARITY);
        granularities |= AccessibilityNodeInfo.MOVEMENT_GRANULARITY_CHARACTER;
      }
      if (semanticsNode.hasAction(Action.MOVE_CURSOR_FORWARD_BY_WORD)) {
        result.addAction(AccessibilityNodeInfo.ACTION_NEXT_AT_MOVEMENT_GRANULARITY);
        granularities |= AccessibilityNodeInfo.MOVEMENT_GRANULARITY_WORD;
      }
      if (semanticsNode.hasAction(Action.MOVE_CURSOR_BACKWARD_BY_WORD)) {
        result.addAction(AccessibilityNodeInfo.ACTION_PREVIOUS_AT_MOVEMENT_GRANULARITY);
        granularities |= AccessibilityNodeInfo.MOVEMENT_GRANULARITY_WORD;
      }
      result.setMovementGranularities(granularities);
      if (semanticsNode.maxValueLength >= 0) {
        // Account for the fact that Flutter is counting Unicode scalar values and Android
        // is counting UTF16 words.
        final int length = semanticsNode.value == null ? 0 : semanticsNode.value.length();
        int a = length - semanticsNode.currentValueLength + semanticsNode.maxValueLength;
        result.setMaxTextLength(
            length - semanticsNode.currentValueLength + semanticsNode.maxValueLength);
      }
    }

    // These are non-ops on older devices. Attempting to interact with the text will cause Talkback
    // to read the contents of the text box instead.
    if (semanticsNode.hasAction(Action.SET_SELECTION)) {
      result.addAction(AccessibilityNodeInfo.ACTION_SET_SELECTION);
    }
    if (semanticsNode.hasAction(Action.COPY)) {
      result.addAction(AccessibilityNodeInfo.ACTION_COPY);
    }
    if (semanticsNode.hasAction(Action.CUT)) {
      result.addAction(AccessibilityNodeInfo.ACTION_CUT);
    }
    if (semanticsNode.hasAction(Action.PASTE)) {
      result.addAction(AccessibilityNodeInfo.ACTION_PASTE);
    }

    if (semanticsNode.hasAction(Action.SET_TEXT)) {
      result.addAction(AccessibilityNodeInfo.ACTION_SET_TEXT);
    }

    if (semanticsNode.hasFlag(Flag.IS_BUTTON) || semanticsNode.hasFlag(Flag.IS_LINK)) {
      result.setClassName("android.widget.Button");
    }
    if (semanticsNode.hasFlag(Flag.IS_IMAGE)) {
      result.setClassName("android.widget.ImageView");
      // TODO(jonahwilliams): Figure out a way conform to the expected id from TalkBack's
      // CustomLabelManager. talkback/src/main/java/labeling/CustomLabelManager.java#L525
    }
    if (semanticsNode.hasAction(Action.DISMISS)) {
      result.setDismissable(true);
      result.addAction(AccessibilityNodeInfo.ACTION_DISMISS);
    }

    if (semanticsNode.parent != null) {
      if (BuildConfig.DEBUG && semanticsNode.id <= ROOT_NODE_ID) {
        Log.e(TAG, "Semantics node id is not > ROOT_NODE_ID.");
      }
      result.setParent(rootAccessibilityView, semanticsNode.parent.id);
    } else {
      if (BuildConfig.DEBUG && semanticsNode.id != ROOT_NODE_ID) {
        Log.e(TAG, "Semantics node id does not equal ROOT_NODE_ID.");
      }
      result.setParent(rootAccessibilityView);
    }

    if (semanticsNode.previousNodeId != -1 && Build.VERSION.SDK_INT >= API_LEVELS.API_22) {
      result.setTraversalAfter(rootAccessibilityView, semanticsNode.previousNodeId);
    }

    Rect bounds = semanticsNode.getGlobalRect();
    if (semanticsNode.parent != null) {
      Rect parentBounds = semanticsNode.parent.getGlobalRect();
      Rect boundsInParent = new Rect(bounds);
      boundsInParent.offset(-parentBounds.left, -parentBounds.top);
      result.setBoundsInParent(boundsInParent);
    } else {
      result.setBoundsInParent(bounds);
    }
    final Rect boundsInScreen = getBoundsInScreen(bounds);
    result.setBoundsInScreen(boundsInScreen);
    result.setVisibleToUser(true);
    result.setEnabled(
        !semanticsNode.hasFlag(Flag.HAS_ENABLED_STATE) || semanticsNode.hasFlag(Flag.IS_ENABLED));

    if (semanticsNode.hasAction(Action.TAP)) {
      if (semanticsNode.onTapOverride != null) {
        result.addAction(
            new AccessibilityNodeInfo.AccessibilityAction(
                AccessibilityNodeInfo.ACTION_CLICK, semanticsNode.onTapOverride.hint));
        result.setClickable(true);
      } else {
        result.addAction(AccessibilityNodeInfo.ACTION_CLICK);
        result.setClickable(true);
      }
    } else {
      // Prevent Slider to receive a regular tap which will change the value.
      //
      // This is needed because it causes slider to select to middle if it
      // doesn't have a semantics tap.
      if (semanticsNode.hasFlag(Flag.IS_SLIDER)) {
        result.addAction(AccessibilityNodeInfo.ACTION_CLICK);
        result.setClickable(true);
      }
    }
    if (semanticsNode.hasAction(Action.LONG_PRESS)) {
      if (semanticsNode.onLongPressOverride != null) {
        result.addAction(
            new AccessibilityNodeInfo.AccessibilityAction(
                AccessibilityNodeInfo.ACTION_LONG_CLICK, semanticsNode.onLongPressOverride.hint));
        result.setLongClickable(true);
      } else {
        result.addAction(AccessibilityNodeInfo.ACTION_LONG_CLICK);
        result.setLongClickable(true);
      }
    }
    if (semanticsNode.hasAction(Action.SCROLL_LEFT)
        || semanticsNode.hasAction(Action.SCROLL_UP)
        || semanticsNode.hasAction(Action.SCROLL_RIGHT)
        || semanticsNode.hasAction(Action.SCROLL_DOWN)) {
      result.setScrollable(true);

      // This tells Android's a11y to send scroll events when reaching the end of
      // the visible viewport of a scrollable, unless the node itself does not
      // allow implicit scrolling - then we leave the className as view.View.
      //
      // We should prefer setCollectionInfo to the class names, as this way we get "In List"
      // and "Out of list" announcements.  But we don't always know the counts, so we
      // can fallback to the generic scroll view class names.
      //
      // On older APIs, we always fall back to the generic scroll view class names here.
      //
      // TODO(dnfield): We should add semantics properties for rows and columns in 2 dimensional
      // lists, e.g.
      // GridView.  Right now, we're only supporting ListViews and only if they have scroll
      // children.
      if (semanticsNode.hasFlag(Flag.HAS_IMPLICIT_SCROLLING)) {
        if (semanticsNode.hasAction(Action.SCROLL_LEFT)
            || semanticsNode.hasAction(Action.SCROLL_RIGHT)) {
          if (shouldSetCollectionInfo(semanticsNode)) {
            result.setCollectionInfo(
                AccessibilityNodeInfo.CollectionInfo.obtain(
                    0, // rows
                    semanticsNode.scrollChildren, // columns
                    false // hierarchical
                    ));
          } else {
            result.setClassName("android.widget.HorizontalScrollView");
          }
        } else {
          if (shouldSetCollectionInfo(semanticsNode)) {
            result.setCollectionInfo(
                AccessibilityNodeInfo.CollectionInfo.obtain(
                    semanticsNode.scrollChildren, // rows
                    0, // columns
                    false // hierarchical
                    ));
          } else {
            result.setClassName("android.widget.ScrollView");
          }
        }
      }
      // TODO(ianh): Once we're on SDK v23+, call addAction to
      // expose AccessibilityAction.ACTION_SCROLL_LEFT, _RIGHT,
      // _UP, and _DOWN when appropriate.
      if (semanticsNode.hasAction(Action.SCROLL_LEFT)
          || semanticsNode.hasAction(Action.SCROLL_UP)) {
        result.addAction(AccessibilityNodeInfo.ACTION_SCROLL_FORWARD);
      }
      if (semanticsNode.hasAction(Action.SCROLL_RIGHT)
          || semanticsNode.hasAction(Action.SCROLL_DOWN)) {
        result.addAction(AccessibilityNodeInfo.ACTION_SCROLL_BACKWARD);
      }
    }
    if (semanticsNode.hasAction(Action.INCREASE) || semanticsNode.hasAction(Action.DECREASE)) {
      // TODO(jonahwilliams): support AccessibilityAction.ACTION_SET_PROGRESS once SDK is
      // updated.
      result.setClassName("android.widget.SeekBar");
      if (semanticsNode.hasAction(Action.INCREASE)) {
        result.addAction(AccessibilityNodeInfo.ACTION_SCROLL_FORWARD);
      }
      if (semanticsNode.hasAction(Action.DECREASE)) {
        result.addAction(AccessibilityNodeInfo.ACTION_SCROLL_BACKWARD);
      }
    }
    if (semanticsNode.hasFlag(Flag.IS_LIVE_REGION)) {
      result.setLiveRegion(View.ACCESSIBILITY_LIVE_REGION_POLITE);
    }

    // Scopes routes are not focusable, only need to set the content
    // for non-scopes-routes semantics nodes.
    if (semanticsNode.hasFlag(Flag.IS_TEXT_FIELD)) {
      result.setText(semanticsNode.getValue());
      if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
        result.setHintText(semanticsNode.getTextFieldHint());
      }
    } else if (!semanticsNode.hasFlag(Flag.SCOPES_ROUTE)) {
      CharSequence content = semanticsNode.getValueLabelHint();
      if (Build.VERSION.SDK_INT < API_LEVELS.API_28) {
        if (semanticsNode.tooltip != null) {
          // For backward compatibility with Flutter SDK before Android API
          // level 28, the tooltip is appended at the end of content description.
          content = content != null ? content : "";
          content = content + "\n" + semanticsNode.tooltip;
        }
      }
      if (content != null) {
        result.setContentDescription(content);
      }
    }

    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      if (semanticsNode.tooltip != null) {
        result.setTooltipText(semanticsNode.tooltip);
      }
    }

    boolean hasCheckedState = semanticsNode.hasFlag(Flag.HAS_CHECKED_STATE);
    boolean hasToggledState = semanticsNode.hasFlag(Flag.HAS_TOGGLED_STATE);
    if (BuildConfig.DEBUG && (hasCheckedState && hasToggledState)) {
      Log.e(TAG, "Expected semanticsNode to have checked state and toggled state.");
    }
    result.setCheckable(hasCheckedState || hasToggledState);
    if (hasCheckedState) {
      result.setChecked(semanticsNode.hasFlag(Flag.IS_CHECKED));
      if (semanticsNode.hasFlag(Flag.IS_IN_MUTUALLY_EXCLUSIVE_GROUP)) {
        result.setClassName("android.widget.RadioButton");
      } else {
        result.setClassName("android.widget.CheckBox");
      }
    } else if (hasToggledState) {
      result.setChecked(semanticsNode.hasFlag(Flag.IS_TOGGLED));
      result.setClassName("android.widget.Switch");
    }
    result.setSelected(semanticsNode.hasFlag(Flag.IS_SELECTED));

    // Heading support
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      result.setHeading(semanticsNode.hasFlag(Flag.IS_HEADER));
    }

    // Accessibility Focus
    if (accessibilityFocusedSemanticsNode != null
        && accessibilityFocusedSemanticsNode.id == virtualViewId) {
      result.addAction(AccessibilityNodeInfo.ACTION_CLEAR_ACCESSIBILITY_FOCUS);
    } else {
      result.addAction(AccessibilityNodeInfo.ACTION_ACCESSIBILITY_FOCUS);
    }

    // Actions on the local context menu
    if (semanticsNode.customAccessibilityActions != null) {
      for (CustomAccessibilityAction action : semanticsNode.customAccessibilityActions) {
        result.addAction(
            new AccessibilityNodeInfo.AccessibilityAction(action.resourceId, action.label));
      }
    }

    for (SemanticsNode child : semanticsNode.childrenInTraversalOrder) {
      if (child.hasFlag(Flag.IS_HIDDEN)) {
        continue;
      }
      if (child.platformViewId != -1) {
        View embeddedView =
            platformViewsAccessibilityDelegate.getPlatformViewById(child.platformViewId);

        // Add the embedded view as a child of the current accessibility node if it's not
        // using a virtual display.
        //
        // In this case, the view is in the Activity's view hierarchy, so it doesn't need to be
        // mirrored.
        //
        // See the case above for how virtual displays are handled.
        if (!platformViewsAccessibilityDelegate.usesVirtualDisplay(child.platformViewId)) {
          result.addChild(embeddedView);
          continue;
        }
      }
      result.addChild(rootAccessibilityView, child.id);
    }
    return result;
  }

  private boolean isImportant(SemanticsNode node) {
    if (node.hasFlag(Flag.SCOPES_ROUTE)) {
      return false;
    }

    if (node.getValueLabelHint() != null) {
      return true;
    }

    // Return true if the node has had any user action (not including system actions)
    return (node.actions & ~systemAction) != 0;
  }

  /**
   * Get the bounds in screen with root FlutterView's offset.
   *
   * @param bounds the bounds in FlutterView
   * @return the bounds with offset
   */
  private Rect getBoundsInScreen(Rect bounds) {
    Rect boundsInScreen = new Rect(bounds);
    int[] locationOnScreen = new int[2];
    rootAccessibilityView.getLocationOnScreen(locationOnScreen);
    boundsInScreen.offset(locationOnScreen[0], locationOnScreen[1]);
    return boundsInScreen;
  }

  /**
   * Instructs the view represented by {@code virtualViewId} to carry out the desired {@code
   * accessibilityAction}, perhaps configured by additional {@code arguments}.
   *
   * <p>This method is invoked by Android's accessibility system. This method returns true if the
   * desired {@code SemanticsNode} was found and was capable of performing the desired action, false
   * otherwise.
   *
   * <p>In a traditional Android app, the given view ID refers to a {@link View} within an Android
   * {@link View} hierarchy. Flutter does not have an Android {@link View} hierarchy, therefore the
   * given view ID is a {@code virtualViewId} that refers to a {@code SemanticsNode} within a
   * Flutter app. The given arguments of this method are forwarded from Android to Flutter.
   */
  @Override
  public boolean performAction(
      int virtualViewId, int accessibilityAction, @Nullable Bundle arguments) {
    if (virtualViewId >= MIN_ENGINE_GENERATED_NODE_ID) {
      // The node is in the engine generated range, and is handled by the accessibility view
      // embedder.
      boolean didPerform =
          accessibilityViewEmbedder.performAction(virtualViewId, accessibilityAction, arguments);
      if (didPerform
          && accessibilityAction == AccessibilityNodeInfo.ACTION_CLEAR_ACCESSIBILITY_FOCUS) {
        embeddedAccessibilityFocusedNodeId = null;
      }
      return didPerform;
    }
    SemanticsNode semanticsNode = flutterSemanticsTree.get(virtualViewId);
    if (semanticsNode == null) {
      return false;
    }
    switch (accessibilityAction) {
      case AccessibilityNodeInfo.ACTION_CLICK:
        {
          // Note: TalkBack prior to Oreo doesn't use this handler and instead simulates a
          //     click event at the center of the SemanticsNode. Other a11y services might go
          //     through this handler though.
          accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.TAP);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_LONG_CLICK:
        {
          // Note: TalkBack doesn't use this handler and instead simulates a long click event
          //     at the center of the SemanticsNode. Other a11y services might go through this
          //     handler though.
          accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.LONG_PRESS);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_SCROLL_FORWARD:
        {
          if (semanticsNode.hasAction(Action.SCROLL_UP)) {
            accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.SCROLL_UP);
          } else if (semanticsNode.hasAction(Action.SCROLL_LEFT)) {
            // TODO(ianh): bidi support using textDirection
            accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.SCROLL_LEFT);
          } else if (semanticsNode.hasAction(Action.INCREASE)) {
            semanticsNode.value = semanticsNode.increasedValue;
            semanticsNode.valueAttributes = semanticsNode.increasedValueAttributes;
            // Event causes Android to read out the updated value.
            sendAccessibilityEvent(virtualViewId, AccessibilityEvent.TYPE_VIEW_SELECTED);
            accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.INCREASE);
          } else {
            return false;
          }
          return true;
        }
      case AccessibilityNodeInfo.ACTION_SCROLL_BACKWARD:
        {
          if (semanticsNode.hasAction(Action.SCROLL_DOWN)) {
            accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.SCROLL_DOWN);
          } else if (semanticsNode.hasAction(Action.SCROLL_RIGHT)) {
            // TODO(ianh): bidi support using textDirection
            accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.SCROLL_RIGHT);
          } else if (semanticsNode.hasAction(Action.DECREASE)) {
            semanticsNode.value = semanticsNode.decreasedValue;
            semanticsNode.valueAttributes = semanticsNode.decreasedValueAttributes;
            // Event causes Android to read out the updated value.
            sendAccessibilityEvent(virtualViewId, AccessibilityEvent.TYPE_VIEW_SELECTED);
            accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.DECREASE);
          } else {
            return false;
          }
          return true;
        }
      case AccessibilityNodeInfo.ACTION_PREVIOUS_AT_MOVEMENT_GRANULARITY:
        {
          return performCursorMoveAction(semanticsNode, virtualViewId, arguments, false);
        }
      case AccessibilityNodeInfo.ACTION_NEXT_AT_MOVEMENT_GRANULARITY:
        {
          return performCursorMoveAction(semanticsNode, virtualViewId, arguments, true);
        }
      case AccessibilityNodeInfo.ACTION_CLEAR_ACCESSIBILITY_FOCUS:
        {
          // Focused semantics node must be reset before sending the
          // TYPE_VIEW_ACCESSIBILITY_FOCUS_CLEARED event. Otherwise,
          // TalkBack may think the node is still focused.
          if (accessibilityFocusedSemanticsNode != null
              && accessibilityFocusedSemanticsNode.id == virtualViewId) {
            accessibilityFocusedSemanticsNode = null;
          }
          if (embeddedAccessibilityFocusedNodeId != null
              && embeddedAccessibilityFocusedNodeId == virtualViewId) {
            embeddedAccessibilityFocusedNodeId = null;
          }
          accessibilityChannel.dispatchSemanticsAction(
              virtualViewId, Action.DID_LOSE_ACCESSIBILITY_FOCUS);
          sendAccessibilityEvent(
              virtualViewId, AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUS_CLEARED);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_ACCESSIBILITY_FOCUS:
        {
          if (accessibilityFocusedSemanticsNode == null) {
            // When Android focuses a node, it doesn't invalidate the view.
            // (It does when it sends ACTION_CLEAR_ACCESSIBILITY_FOCUS, so
            // we only have to worry about this when the focused node is null.)
            rootAccessibilityView.invalidate();
          }
          // Focused semantics node must be set before sending the TYPE_VIEW_ACCESSIBILITY_FOCUSED
          // event. Otherwise, TalkBack may think the node is not focused yet.
          accessibilityFocusedSemanticsNode = semanticsNode;

          accessibilityChannel.dispatchSemanticsAction(
              virtualViewId, Action.DID_GAIN_ACCESSIBILITY_FOCUS);

          HashMap<String, Object> message = new HashMap<>();
          message.put("type", "didGainFocus");
          message.put("nodeId", semanticsNode.id);
          accessibilityChannel.channel.send(message);

          sendAccessibilityEvent(virtualViewId, AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUSED);

          if (semanticsNode.hasAction(Action.INCREASE)
              || semanticsNode.hasAction(Action.DECREASE)) {
            // SeekBars only announce themselves after this event.
            sendAccessibilityEvent(virtualViewId, AccessibilityEvent.TYPE_VIEW_SELECTED);
          }

          return true;
        }
      case ACTION_SHOW_ON_SCREEN:
        {
          accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.SHOW_ON_SCREEN);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_SET_SELECTION:
        {
          final Map<String, Integer> selection = new HashMap<>();
          final boolean hasSelection =
              arguments != null
                  && arguments.containsKey(
                      AccessibilityNodeInfo.ACTION_ARGUMENT_SELECTION_START_INT)
                  && arguments.containsKey(AccessibilityNodeInfo.ACTION_ARGUMENT_SELECTION_END_INT);
          if (hasSelection) {
            selection.put(
                "base",
                arguments.getInt(AccessibilityNodeInfo.ACTION_ARGUMENT_SELECTION_START_INT));
            selection.put(
                "extent",
                arguments.getInt(AccessibilityNodeInfo.ACTION_ARGUMENT_SELECTION_END_INT));
          } else {
            // Clear the selection
            selection.put("base", semanticsNode.textSelectionExtent);
            selection.put("extent", semanticsNode.textSelectionExtent);
          }
          accessibilityChannel.dispatchSemanticsAction(
              virtualViewId, Action.SET_SELECTION, selection);
          // The voice access expects the semantics node to update immediately. We update the
          // semantics node based on prediction. If the result is incorrect, it will be updated in
          // the next frame.
          SemanticsNode node = flutterSemanticsTree.get(virtualViewId);
          node.textSelectionBase = selection.get("base");
          node.textSelectionExtent = selection.get("extent");
          return true;
        }
      case AccessibilityNodeInfo.ACTION_COPY:
        {
          accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.COPY);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_CUT:
        {
          accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.CUT);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_PASTE:
        {
          accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.PASTE);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_DISMISS:
        {
          accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.DISMISS);
          return true;
        }
      case AccessibilityNodeInfo.ACTION_SET_TEXT:
        {
          return performSetText(semanticsNode, virtualViewId, arguments);
        }
      default:
        // might be a custom accessibility accessibilityAction.
        final int flutterId = accessibilityAction - FIRST_RESOURCE_ID;
        CustomAccessibilityAction contextAction = customAccessibilityActions.get(flutterId);
        if (contextAction != null) {
          accessibilityChannel.dispatchSemanticsAction(
              virtualViewId, Action.CUSTOM_ACTION, contextAction.id);
          return true;
        }
    }
    return false;
  }

  /**
   * Handles the responsibilities of {@link #performAction(int, int, Bundle)} for the specific
   * scenario of cursor movement.
   */
  private boolean performCursorMoveAction(
      @NonNull SemanticsNode semanticsNode,
      int virtualViewId,
      @NonNull Bundle arguments,
      boolean forward) {
    final int granularity =
        arguments.getInt(AccessibilityNodeInfo.ACTION_ARGUMENT_MOVEMENT_GRANULARITY_INT);
    final boolean extendSelection =
        arguments.getBoolean(AccessibilityNodeInfo.ACTION_ARGUMENT_EXTEND_SELECTION_BOOLEAN);
    // The voice access expects the semantics node to update immediately. We update the semantics
    // node based on prediction. If the result is incorrect, it will be updated in the next frame.
    final int previousTextSelectionBase = semanticsNode.textSelectionBase;
    final int previousTextSelectionExtent = semanticsNode.textSelectionExtent;
    predictCursorMovement(semanticsNode, granularity, forward, extendSelection);

    if (previousTextSelectionBase != semanticsNode.textSelectionBase
        || previousTextSelectionExtent != semanticsNode.textSelectionExtent) {
      final String value = semanticsNode.value != null ? semanticsNode.value : "";
      final AccessibilityEvent selectionEvent =
          obtainAccessibilityEvent(
              semanticsNode.id, AccessibilityEvent.TYPE_VIEW_TEXT_SELECTION_CHANGED);
      selectionEvent.getText().add(value);
      selectionEvent.setFromIndex(semanticsNode.textSelectionBase);
      selectionEvent.setToIndex(semanticsNode.textSelectionExtent);
      selectionEvent.setItemCount(value.length());
      sendAccessibilityEvent(selectionEvent);
    }

    switch (granularity) {
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_CHARACTER:
        {
          if (forward && semanticsNode.hasAction(Action.MOVE_CURSOR_FORWARD_BY_CHARACTER)) {
            accessibilityChannel.dispatchSemanticsAction(
                virtualViewId, Action.MOVE_CURSOR_FORWARD_BY_CHARACTER, extendSelection);
            return true;
          }
          if (!forward && semanticsNode.hasAction(Action.MOVE_CURSOR_BACKWARD_BY_CHARACTER)) {
            accessibilityChannel.dispatchSemanticsAction(
                virtualViewId, Action.MOVE_CURSOR_BACKWARD_BY_CHARACTER, extendSelection);
            return true;
          }
          break;
        }
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_WORD:
        if (forward && semanticsNode.hasAction(Action.MOVE_CURSOR_FORWARD_BY_WORD)) {
          accessibilityChannel.dispatchSemanticsAction(
              virtualViewId, Action.MOVE_CURSOR_FORWARD_BY_WORD, extendSelection);
          return true;
        }
        if (!forward && semanticsNode.hasAction(Action.MOVE_CURSOR_BACKWARD_BY_WORD)) {
          accessibilityChannel.dispatchSemanticsAction(
              virtualViewId, Action.MOVE_CURSOR_BACKWARD_BY_WORD, extendSelection);
          return true;
        }
        break;
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_LINE:
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_PARAGRAPH:
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_PAGE:
        return true;
    }
    return false;
  }

  private void predictCursorMovement(
      @NonNull SemanticsNode node, int granularity, boolean forward, boolean extendSelection) {
    if (node.textSelectionExtent < 0 || node.textSelectionBase < 0) {
      return;
    }

    switch (granularity) {
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_CHARACTER:
        if (forward && node.textSelectionExtent < node.value.length()) {
          node.textSelectionExtent += 1;
        } else if (!forward && node.textSelectionExtent > 0) {
          node.textSelectionExtent -= 1;
        }
        break;
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_WORD:
        if (forward && node.textSelectionExtent < node.value.length()) {
          Pattern pattern = Pattern.compile("\\p{L}(\\b)");
          Matcher result = pattern.matcher(node.value.substring(node.textSelectionExtent));
          // we discard the first result because we want to find the "next" word
          result.find();
          if (result.find()) {
            node.textSelectionExtent += result.start(1);
          } else {
            node.textSelectionExtent = node.value.length();
          }
        } else if (!forward && node.textSelectionExtent > 0) {
          // Finds last beginning of the word boundary.
          Pattern pattern = Pattern.compile("(?s:.*)(\\b)\\p{L}");
          Matcher result = pattern.matcher(node.value.substring(0, node.textSelectionExtent));
          if (result.find()) {
            node.textSelectionExtent = result.start(1);
          }
        }
        break;
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_LINE:
        if (forward && node.textSelectionExtent < node.value.length()) {
          // Finds the next new line.
          Pattern pattern = Pattern.compile("(?!^)(\\n)");
          Matcher result = pattern.matcher(node.value.substring(node.textSelectionExtent));
          if (result.find()) {
            node.textSelectionExtent += result.start(1);
          } else {
            node.textSelectionExtent = node.value.length();
          }
        } else if (!forward && node.textSelectionExtent > 0) {
          // Finds the last new line.
          Pattern pattern = Pattern.compile("(?s:.*)(\\n)");
          Matcher result = pattern.matcher(node.value.substring(0, node.textSelectionExtent));
          if (result.find()) {
            node.textSelectionExtent = result.start(1);
          } else {
            node.textSelectionExtent = 0;
          }
        }
        break;
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_PARAGRAPH:
      case AccessibilityNodeInfo.MOVEMENT_GRANULARITY_PAGE:
        if (forward) {
          node.textSelectionExtent = node.value.length();
        } else {
          node.textSelectionExtent = 0;
        }
        break;
    }
    if (!extendSelection) {
      node.textSelectionBase = node.textSelectionExtent;
    }
  }

  /**
   * Handles the responsibilities of {@link #performAction(int, int, Bundle)} for the specific
   * scenario of cursor movement.
   */
  private boolean performSetText(SemanticsNode node, int virtualViewId, @NonNull Bundle arguments) {
    String newText = "";
    if (arguments != null
        && arguments.containsKey(AccessibilityNodeInfo.ACTION_ARGUMENT_SET_TEXT_CHARSEQUENCE)) {
      newText = arguments.getString(AccessibilityNodeInfo.ACTION_ARGUMENT_SET_TEXT_CHARSEQUENCE);
    }
    accessibilityChannel.dispatchSemanticsAction(virtualViewId, Action.SET_TEXT, newText);
    // The voice access expects the semantics node to update immediately. Update the semantics
    // node based on prediction. If the result is incorrect, it will be updated in the next frame.
    node.value = newText;
    node.valueAttributes = null;
    return true;
  }

  // TODO(ianh): implement findAccessibilityNodeInfosByText()

  /**
   * Finds the view in a hierarchy that currently has the given type of {@code focus}.
   *
   * <p>This method is invoked by Android's accessibility system.
   *
   * <p>Flutter does not have an Android {@link View} hierarchy. Therefore, Flutter conceptually
   * handles this request by searching its semantics tree for the given {@code focus}, represented
   * by {@link #flutterSemanticsTree}. In practice, this {@code AccessibilityBridge} always caches
   * any active {@link #accessibilityFocusedSemanticsNode} and {@link #inputFocusedSemanticsNode}.
   * Therefore, no searching is necessary. This method directly inspects the given {@code focus}
   * type to return one of the cached nodes, null if the cached node is null, or null if a different
   * {@code focus} type is requested.
   */
  @Override
  public AccessibilityNodeInfo findFocus(int focus) {
    switch (focus) {
      case AccessibilityNodeInfo.FOCUS_INPUT:
        {
          if (inputFocusedSemanticsNode != null) {
            return createAccessibilityNodeInfo(inputFocusedSemanticsNode.id);
          }
          if (embeddedInputFocusedNodeId != null) {
            return createAccessibilityNodeInfo(embeddedInputFocusedNodeId);
          }
        }
        // Fall through to check FOCUS_ACCESSIBILITY
      case AccessibilityNodeInfo.FOCUS_ACCESSIBILITY:
        {
          if (accessibilityFocusedSemanticsNode != null) {
            return createAccessibilityNodeInfo(accessibilityFocusedSemanticsNode.id);
          }
          if (embeddedAccessibilityFocusedNodeId != null) {
            return createAccessibilityNodeInfo(embeddedAccessibilityFocusedNodeId);
          }
        }
    }
    return null;
  }

  /** Returns the {@link SemanticsNode} at the root of Flutter's semantics tree. */
  private SemanticsNode getRootSemanticsNode() {
    if (BuildConfig.DEBUG && !flutterSemanticsTree.containsKey(0)) {
      Log.e(TAG, "Attempted to getRootSemanticsNode without a root semantics node.");
    }
    return flutterSemanticsTree.get(0);
  }

  /**
   * Returns an existing {@link SemanticsNode} with the given {@code id}, if it exists within {@link
   * #flutterSemanticsTree}, or creates and returns a new {@link SemanticsNode} with the given
   * {@code id}, adding the new {@link SemanticsNode} to the {@link #flutterSemanticsTree}.
   *
   * <p>This method should only be invoked as a result of receiving new information from Flutter.
   * The {@link #flutterSemanticsTree} is an Android cache of the last known state of a Flutter
   * app's semantics tree, therefore, invoking this method in any other situation will result in a
   * corrupt cache of Flutter's semantics tree.
   */
  private SemanticsNode getOrCreateSemanticsNode(int id) {
    SemanticsNode semanticsNode = flutterSemanticsTree.get(id);
    if (semanticsNode == null) {
      semanticsNode = new SemanticsNode(this);
      semanticsNode.id = id;
      flutterSemanticsTree.put(id, semanticsNode);
    }
    return semanticsNode;
  }

  /**
   * Returns an existing {@link CustomAccessibilityAction} with the given {@code id}, if it exists
   * within {@link #customAccessibilityActions}, or creates and returns a new {@link
   * CustomAccessibilityAction} with the given {@code id}, adding the new {@link
   * CustomAccessibilityAction} to the {@link #customAccessibilityActions}.
   *
   * <p>This method should only be invoked as a result of receiving new information from Flutter.
   * The {@link #customAccessibilityActions} is an Android cache of the last known state of a
   * Flutter app's registered custom accessibility actions, therefore, invoking this method in any
   * other situation will result in a corrupt cache of Flutter's accessibility actions.
   */
  private CustomAccessibilityAction getOrCreateAccessibilityAction(int id) {
    CustomAccessibilityAction action = customAccessibilityActions.get(id);
    if (action == null) {
      action = new CustomAccessibilityAction();
      action.id = id;
      action.resourceId = id + FIRST_RESOURCE_ID;
      customAccessibilityActions.put(id, action);
    }
    return action;
  }

  /**
   * A hover {@link MotionEvent} has occurred in the {@code View} that corresponds to this {@code
   * AccessibilityBridge}.
   *
   * <p>This method returns true if Flutter's accessibility system handled the hover event, false
   * otherwise.
   *
   * <p>This method should be invoked from the corresponding {@code View}'s {@link
   * View#onHoverEvent(MotionEvent)}.
   */
  public boolean onAccessibilityHoverEvent(MotionEvent event) {
    return onAccessibilityHoverEvent(event, false);
  }

  /**
   * A hover {@link MotionEvent} has occurred in the {@code View} that corresponds to this {@code
   * AccessibilityBridge}.
   *
   * <p>If {@code ignorePlatformViews} is true, if hit testing for the event finds a platform view,
   * the event will not be handled. This is useful when handling accessibility events for views
   * overlaying platform views. See {@code PlatformOverlayView} for details.
   *
   * <p>This method returns true if Flutter's accessibility system handled the hover event, false
   * otherwise.
   *
   * <p>This method should be invoked from the corresponding {@code View}'s {@link
   * View#onHoverEvent(MotionEvent)}.
   */
  public boolean onAccessibilityHoverEvent(MotionEvent event, boolean ignorePlatformViews) {
    if (!accessibilityManager.isTouchExplorationEnabled()) {
      return false;
    }
    if (flutterSemanticsTree.isEmpty()) {
      return false;
    }

    SemanticsNode semanticsNodeUnderCursor =
        getRootSemanticsNode()
            .hitTest(new float[] {event.getX(), event.getY(), 0, 1}, ignorePlatformViews);
    // semanticsNodeUnderCursor can be null when hovering over non-flutter UI such as
    // the Android navigation bar due to hitTest() bounds checking.
    if (semanticsNodeUnderCursor != null && semanticsNodeUnderCursor.platformViewId != -1) {
      if (ignorePlatformViews) {
        return false;
      }
      return accessibilityViewEmbedder.onAccessibilityHoverEvent(
          semanticsNodeUnderCursor.id, event);
    }

    if (event.getAction() == MotionEvent.ACTION_HOVER_ENTER
        || event.getAction() == MotionEvent.ACTION_HOVER_MOVE) {
      handleTouchExploration(event.getX(), event.getY(), ignorePlatformViews);
    } else if (event.getAction() == MotionEvent.ACTION_HOVER_EXIT) {
      onTouchExplorationExit();
    } else {
      Log.d("flutter", "unexpected accessibility hover event: " + event);
      return false;
    }
    return true;
  }

  /**
   * This method should be invoked when a hover interaction has the cursor move off of a {@code
   * SemanticsNode}.
   *
   * <p>This method informs the Android accessibility system that a {@link
   * AccessibilityEvent#TYPE_VIEW_HOVER_EXIT} has occurred.
   */
  private void onTouchExplorationExit() {
    if (hoveredObject != null) {
      sendAccessibilityEvent(hoveredObject.id, AccessibilityEvent.TYPE_VIEW_HOVER_EXIT);
      hoveredObject = null;
    }
  }

  /**
   * This method should be invoked when a new hover interaction begins with a {@code SemanticsNode},
   * or when an existing hover interaction sees a movement of the cursor.
   *
   * <p>This method checks to see if the cursor has moved from one {@code SemanticsNode} to another.
   * If it has, this method informs the Android accessibility system of the change by first sending
   * a {@link AccessibilityEvent#TYPE_VIEW_HOVER_ENTER} event for the new hover node, followed by a
   * {@link AccessibilityEvent#TYPE_VIEW_HOVER_EXIT} event for the old hover node.
   */
  private void handleTouchExploration(float x, float y, boolean ignorePlatformViews) {
    if (flutterSemanticsTree.isEmpty()) {
      return;
    }
    SemanticsNode semanticsNodeUnderCursor =
        getRootSemanticsNode().hitTest(new float[] {x, y, 0, 1}, ignorePlatformViews);
    if (semanticsNodeUnderCursor != hoveredObject) {
      // sending ENTER before EXIT is how Android wants it
      if (semanticsNodeUnderCursor != null) {
        sendAccessibilityEvent(
            semanticsNodeUnderCursor.id, AccessibilityEvent.TYPE_VIEW_HOVER_ENTER);
      }
      if (hoveredObject != null) {
        sendAccessibilityEvent(hoveredObject.id, AccessibilityEvent.TYPE_VIEW_HOVER_EXIT);
      }
      hoveredObject = semanticsNodeUnderCursor;
    }
  }

  /**
   * Updates the Android cache of Flutter's currently registered custom accessibility actions.
   *
   * <p>The buffer received here is encoded by PlatformViewAndroid::UpdateSemantics, and the decode
   * logic here must be kept in sync with that method's encoding logic.
   */
  // TODO(mattcarroll): Consider introducing ability to delete custom actions because they can
  //                    probably come and go in Flutter, so we may want to reflect that here in
  //                    the Android cache as well.
  void updateCustomAccessibilityActions(@NonNull ByteBuffer buffer, @NonNull String[] strings) {
    while (buffer.hasRemaining()) {
      int id = buffer.getInt();
      CustomAccessibilityAction action = getOrCreateAccessibilityAction(id);
      action.overrideId = buffer.getInt();
      int stringIndex = buffer.getInt();
      action.label = stringIndex == -1 ? null : strings[stringIndex];
      stringIndex = buffer.getInt();
      action.hint = stringIndex == -1 ? null : strings[stringIndex];
    }
  }

  /**
   * Updates {@link #flutterSemanticsTree} to reflect the latest state of Flutter's semantics tree.
   *
   * <p>The latest state of Flutter's semantics tree is encoded in the given {@code buffer}. The
   * buffer is encoded by PlatformViewAndroid::UpdateSemantics, and the decode logic must be kept in
   * sync with that method's encoding logic.
   */
  void updateSemantics(
      @NonNull ByteBuffer buffer,
      @NonNull String[] strings,
      @NonNull ByteBuffer[] stringAttributeArgs) {
    ArrayList<SemanticsNode> updated = new ArrayList<>();
    while (buffer.hasRemaining()) {
      int id = buffer.getInt();
      SemanticsNode semanticsNode = getOrCreateSemanticsNode(id);
      semanticsNode.updateWith(buffer, strings, stringAttributeArgs);
      if (semanticsNode.hasFlag(Flag.IS_HIDDEN)) {
        continue;
      }
      if (semanticsNode.hasFlag(Flag.IS_FOCUSED)) {
        inputFocusedSemanticsNode = semanticsNode;
      }
      if (semanticsNode.hadPreviousConfig) {
        updated.add(semanticsNode);
      }
      if (semanticsNode.platformViewId != -1
          && !platformViewsAccessibilityDelegate.usesVirtualDisplay(semanticsNode.platformViewId)) {
        View embeddedView =
            platformViewsAccessibilityDelegate.getPlatformViewById(semanticsNode.platformViewId);
        if (embeddedView != null) {
          embeddedView.setImportantForAccessibility(View.IMPORTANT_FOR_ACCESSIBILITY_AUTO);
        }
      }
    }

    Set<SemanticsNode> visitedObjects = new HashSet<>();
    SemanticsNode rootObject = getRootSemanticsNode();
    List<SemanticsNode> newRoutes = new ArrayList<>();
    if (rootObject != null) {
      final float[] identity = new float[16];
      Matrix.setIdentityM(identity, 0);
      // In Android devices API 23 and above, the system nav bar can be placed on the left side
      // of the screen in landscape mode. We must handle the translation ourselves for the
      // a11y nodes.
      if (Build.VERSION.SDK_INT >= API_LEVELS.API_23) {
        boolean needsToApplyLeftCutoutInset = true;
        // In Android devices API 28 and above, the `layoutInDisplayCutoutMode` window attribute
        // can be set to allow overlapping content within the cutout area. Query the attribute
        // to figure out whether the content overlaps with the cutout and decide whether to
        // apply cutout inset.
        if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
          needsToApplyLeftCutoutInset = doesLayoutInDisplayCutoutModeRequireLeftInset();
        }

        if (needsToApplyLeftCutoutInset) {
          WindowInsets insets = rootAccessibilityView.getRootWindowInsets();
          if (insets != null) {
            if (!lastLeftFrameInset.equals(insets.getSystemWindowInsetLeft())) {
              rootObject.globalGeometryDirty = true;
              rootObject.inverseTransformDirty = true;
            }
            lastLeftFrameInset = insets.getSystemWindowInsetLeft();
            Matrix.translateM(identity, 0, lastLeftFrameInset, 0, 0);
          }
        }
      }
      rootObject.updateRecursively(identity, visitedObjects, false);
      rootObject.collectRoutes(newRoutes);
    }

    // Dispatch a TYPE_WINDOW_STATE_CHANGED event if the most recent route id changed from the
    // previously cached route id.

    // Finds the last route that is not in the previous routes.
    SemanticsNode lastAdded = null;
    for (SemanticsNode semanticsNode : newRoutes) {
      if (!flutterNavigationStack.contains(semanticsNode.id)) {
        lastAdded = semanticsNode;
      }
    }

    // If all the routes are in the previous route, get the last route.
    if (lastAdded == null && newRoutes.size() > 0) {
      lastAdded = newRoutes.get(newRoutes.size() - 1);
    }

    // There are two cases if lastAdded != nil
    // 1. lastAdded is not in previous routes. In this case,
    //    lastAdded.id != previousRouteId
    // 2. All new routes are in previous routes and
    //    lastAdded = newRoutes.last.
    // In the first case, we need to announce new route. In the second case,
    // we need to announce if one list is shorter than the other.
    if (lastAdded != null
        && (lastAdded.id != previousRouteId || newRoutes.size() != flutterNavigationStack.size())) {
      previousRouteId = lastAdded.id;
      onWindowNameChange(lastAdded);
    }
    flutterNavigationStack.clear();
    for (SemanticsNode semanticsNode : newRoutes) {
      flutterNavigationStack.add(semanticsNode.id);
    }

    Iterator<Map.Entry<Integer, SemanticsNode>> it = flutterSemanticsTree.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<Integer, SemanticsNode> entry = it.next();
      SemanticsNode object = entry.getValue();
      if (!visitedObjects.contains(object)) {
        willRemoveSemanticsNode(object);
        it.remove();
      }
    }

    // TODO(goderbauer): Send this event only once (!) for changed subtrees,
    //     see https://github.com/flutter/flutter/issues/14534
    sendWindowContentChangeEvent(0);

    for (SemanticsNode object : updated) {
      if (object.didScroll()) {
        AccessibilityEvent event =
            obtainAccessibilityEvent(object.id, AccessibilityEvent.TYPE_VIEW_SCROLLED);

        // Android doesn't support unbound scrolling. So we pretend there is a large
        // bound (SCROLL_EXTENT_FOR_INFINITY), which you can never reach.
        float position = object.scrollPosition;
        float max = object.scrollExtentMax;
        if (Float.isInfinite(object.scrollExtentMax)) {
          max = SCROLL_EXTENT_FOR_INFINITY;
          if (position > SCROLL_POSITION_CAP_FOR_INFINITY) {
            position = SCROLL_POSITION_CAP_FOR_INFINITY;
          }
        }
        if (Float.isInfinite(object.scrollExtentMin)) {
          max += SCROLL_EXTENT_FOR_INFINITY;
          if (position < -SCROLL_POSITION_CAP_FOR_INFINITY) {
            position = -SCROLL_POSITION_CAP_FOR_INFINITY;
          }
          position += SCROLL_EXTENT_FOR_INFINITY;
        } else {
          max -= object.scrollExtentMin;
          position -= object.scrollExtentMin;
        }

        if (object.hadAction(Action.SCROLL_UP) || object.hadAction(Action.SCROLL_DOWN)) {
          event.setScrollY((int) position);
          event.setMaxScrollY((int) max);
        } else if (object.hadAction(Action.SCROLL_LEFT) || object.hadAction(Action.SCROLL_RIGHT)) {
          event.setScrollX((int) position);
          event.setMaxScrollX((int) max);
        }
        if (object.scrollChildren > 0) {
          // We don't need to add 1 to the scroll index because TalkBack does this automagically.
          event.setItemCount(object.scrollChildren);
          event.setFromIndex(object.scrollIndex);
          int visibleChildren = 0;
          // handle hidden children at the beginning and end of the list.
          for (SemanticsNode child : object.childrenInHitTestOrder) {
            if (!child.hasFlag(Flag.IS_HIDDEN)) {
              visibleChildren += 1;
            }
          }
          if (BuildConfig.DEBUG) {
            if (object.scrollIndex + visibleChildren > object.scrollChildren) {
              Log.e(TAG, "Scroll index is out of bounds.");
            }

            if (object.childrenInHitTestOrder.isEmpty()) {
              Log.e(TAG, "Had scrollChildren but no childrenInHitTestOrder");
            }
          }
          // The setToIndex should be the index of the last visible child. Because we counted all
          // children, including the first index we need to subtract one.
          //
          //   [0, 1, 2, 3, 4, 5]
          //    ^     ^
          // In the example above where 0 is the first visible index and 2 is the last, we will
          // count 3 total visible children. We then subtract one to get the correct last visible
          // index of 2.
          event.setToIndex(object.scrollIndex + visibleChildren - 1);
        }
        sendAccessibilityEvent(event);
      }
      if (object.hasFlag(Flag.IS_LIVE_REGION) && object.didChangeLabel()) {
        sendWindowContentChangeEvent(object.id);
      }
      if (accessibilityFocusedSemanticsNode != null
          && accessibilityFocusedSemanticsNode.id == object.id
          && !object.hadFlag(Flag.IS_SELECTED)
          && object.hasFlag(Flag.IS_SELECTED)) {
        AccessibilityEvent event =
            obtainAccessibilityEvent(object.id, AccessibilityEvent.TYPE_VIEW_SELECTED);
        event.getText().add(object.label);
        sendAccessibilityEvent(event);
      }

      // If the object is the input-focused node, then tell the reader about it, but only if
      // it has changed since the last update.
      if (inputFocusedSemanticsNode != null
          && inputFocusedSemanticsNode.id == object.id
          && (lastInputFocusedSemanticsNode == null
              || lastInputFocusedSemanticsNode.id != inputFocusedSemanticsNode.id)) {
        lastInputFocusedSemanticsNode = inputFocusedSemanticsNode;
        sendAccessibilityEvent(
            obtainAccessibilityEvent(object.id, AccessibilityEvent.TYPE_VIEW_FOCUSED));
      } else if (inputFocusedSemanticsNode == null) {
        // There's no TYPE_VIEW_CLEAR_FOCUSED event, so if the current input focus becomes
        // null, then we just set the last one to null too, so that it sends the event again
        // when something regains focus.
        lastInputFocusedSemanticsNode = null;
      }

      if (inputFocusedSemanticsNode != null
          && inputFocusedSemanticsNode.id == object.id
          && object.hadFlag(Flag.IS_TEXT_FIELD)
          && object.hasFlag(Flag.IS_TEXT_FIELD)
          // If we have a TextField that has InputFocus, we should avoid announcing it if something
          // else we track has a11y focus. This needs to still work when, e.g., IME has a11y focus
          // or the "PASTE" popup is used though.
          // See more discussion at https://github.com/flutter/flutter/issues/23180
          && (accessibilityFocusedSemanticsNode == null
              || (accessibilityFocusedSemanticsNode.id == inputFocusedSemanticsNode.id))) {
        String oldValue = object.previousValue != null ? object.previousValue : "";
        String newValue = object.value != null ? object.value : "";
        AccessibilityEvent event = createTextChangedEvent(object.id, oldValue, newValue);
        if (event != null) {
          sendAccessibilityEvent(event);
        }

        if (object.previousTextSelectionBase != object.textSelectionBase
            || object.previousTextSelectionExtent != object.textSelectionExtent) {
          AccessibilityEvent selectionEvent =
              obtainAccessibilityEvent(
                  object.id, AccessibilityEvent.TYPE_VIEW_TEXT_SELECTION_CHANGED);
          selectionEvent.getText().add(newValue);
          selectionEvent.setFromIndex(object.textSelectionBase);
          selectionEvent.setToIndex(object.textSelectionExtent);
          selectionEvent.setItemCount(newValue.length());
          sendAccessibilityEvent(selectionEvent);
        }
      }
    }
  }

  private AccessibilityEvent createTextChangedEvent(int id, String oldValue, String newValue) {
    AccessibilityEvent e = obtainAccessibilityEvent(id, AccessibilityEvent.TYPE_VIEW_TEXT_CHANGED);
    e.setBeforeText(oldValue);
    e.getText().add(newValue);

    int i;
    for (i = 0; i < oldValue.length() && i < newValue.length(); ++i) {
      if (oldValue.charAt(i) != newValue.charAt(i)) {
        break;
      }
    }
    if (i >= oldValue.length() && i >= newValue.length()) {
      return null; // Text did not change
    }
    int firstDifference = i;
    e.setFromIndex(firstDifference);

    int oldIndex = oldValue.length() - 1;
    int newIndex = newValue.length() - 1;
    while (oldIndex >= firstDifference && newIndex >= firstDifference) {
      if (oldValue.charAt(oldIndex) != newValue.charAt(newIndex)) {
        break;
      }
      --oldIndex;
      --newIndex;
    }
    e.setRemovedCount(oldIndex - firstDifference + 1);
    e.setAddedCount(newIndex - firstDifference + 1);

    return e;
  }

  /**
   * Sends an accessibility event of the given {@code eventType} to Android's accessibility system
   * with the given {@code viewId} represented as the source of the event.
   *
   * <p>The given {@code viewId} may either belong to {@link #rootAccessibilityView}, or any Flutter
   * {@link SemanticsNode}.
   */
  @VisibleForTesting
  public void sendAccessibilityEvent(int viewId, int eventType) {
    if (!accessibilityManager.isEnabled()) {
      return;
    }
    sendAccessibilityEvent(obtainAccessibilityEvent(viewId, eventType));
  }

  /**
   * Sends the given {@link AccessibilityEvent} to Android's accessibility system for a given
   * Flutter {@link SemanticsNode}.
   *
   * <p>This method should only be called for a Flutter {@link SemanticsNode}, not a traditional
   * Android {@code View}, i.e., {@link #rootAccessibilityView}.
   */
  private void sendAccessibilityEvent(@NonNull AccessibilityEvent event) {
    if (!accessibilityManager.isEnabled()) {
      return;
    }
    // See
    // https://developer.android.com/reference/android/view/View.html#sendAccessibilityEvent(int)
    // We just want the final part at this point, since the event parameter
    // has already been correctly populated.
    rootAccessibilityView.getParent().requestSendAccessibilityEvent(rootAccessibilityView, event);
  }

  /**
   * Informs the TalkBack user about window name changes.
   *
   * <p>This method sets accessibility panel title if the API level >= 28, otherwise, it creates a
   * {@link AccessibilityEvent#TYPE_WINDOW_STATE_CHANGED} and sends the event to Android's
   * accessibility system. In both cases, TalkBack announces the label of the route and re-addjusts
   * the accessibility focus.
   *
   * <p>The given {@code route} should be a {@link SemanticsNode} that represents a navigation route
   * in the Flutter app.
   */
  private void onWindowNameChange(@NonNull SemanticsNode route) {
    String routeName = route.getRouteName();
    if (routeName == null) {
      // The routeName will be null when there is no semantics node that represnets namesRoute in
      // the scopeRoute. The TYPE_WINDOW_STATE_CHANGED only works the route name is not null and not
      // empty. Gives it a whitespace will make it focus the first semantics node without
      // pronouncing any word.
      //
      // The other way to trigger a focus change is to send a TYPE_VIEW_FOCUSED to the
      // rootAccessibilityView. However, it is less predictable which semantics node it will focus
      // next.
      routeName = " ";
    }
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      setAccessibilityPaneTitle(routeName);
    } else {
      AccessibilityEvent event =
          obtainAccessibilityEvent(route.id, AccessibilityEvent.TYPE_WINDOW_STATE_CHANGED);
      event.getText().add(routeName);
      sendAccessibilityEvent(event);
    }
  }

  @TargetApi(API_LEVELS.API_28)
  @RequiresApi(API_LEVELS.API_28)
  private void setAccessibilityPaneTitle(String title) {
    rootAccessibilityView.setAccessibilityPaneTitle(title);
  }

  /**
   * Creates a {@link AccessibilityEvent#TYPE_WINDOW_CONTENT_CHANGED} and sends the event to
   * Android's accessibility system.
   *
   * <p>It sets the content change types to {@link AccessibilityEvent#CONTENT_CHANGE_TYPE_SUBTREE}
   * when supported by the API level.
   *
   * <p>The given {@code virtualViewId} should be a {@link SemanticsNode} below which the content
   * has changed.
   */
  private void sendWindowContentChangeEvent(int virtualViewId) {
    AccessibilityEvent event =
        obtainAccessibilityEvent(virtualViewId, AccessibilityEvent.TYPE_WINDOW_CONTENT_CHANGED);
    event.setContentChangeTypes(AccessibilityEvent.CONTENT_CHANGE_TYPE_SUBTREE);
    sendAccessibilityEvent(event);
  }

  /**
   * Factory method that creates a new {@link AccessibilityEvent} that is configured to represent
   * the Flutter {@link SemanticsNode} represented by the given {@code virtualViewId}, categorized
   * as the given {@code eventType}.
   *
   * <p>This method should *only* be called for Flutter {@link SemanticsNode}s. It should *not* be
   * invoked to create an {@link AccessibilityEvent} for the {@link #rootAccessibilityView}.
   */
  private AccessibilityEvent obtainAccessibilityEvent(int virtualViewId, int eventType) {
    AccessibilityEvent event = obtainAccessibilityEvent(eventType);
    event.setPackageName(rootAccessibilityView.getContext().getPackageName());
    event.setSource(rootAccessibilityView, virtualViewId);
    return event;
  }

  @VisibleForTesting
  public AccessibilityEvent obtainAccessibilityEvent(int eventType) {
    return AccessibilityEvent.obtain(eventType);
  }

  /**
   * Reads the {@code layoutInDisplayCutoutMode} value from the window attribute and returns whether
   * a left cutout inset is required.
   *
   * <p>The {@code layoutInDisplayCutoutMode} is added after API level 28.
   */
  @TargetApi(API_LEVELS.API_28)
  @RequiresApi(API_LEVELS.API_28)
  private boolean doesLayoutInDisplayCutoutModeRequireLeftInset() {
    Context context = rootAccessibilityView.getContext();
    Activity activity = ViewUtils.getActivity(context);
    if (activity == null || activity.getWindow() == null) {
      // The activity is not visible, it does not matter whether to apply left inset
      // or not.
      return false;
    }
    int layoutInDisplayCutoutMode = activity.getWindow().getAttributes().layoutInDisplayCutoutMode;
    return layoutInDisplayCutoutMode
            == WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_NEVER
        || layoutInDisplayCutoutMode
            == WindowManager.LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_DEFAULT;
  }

  /**
   * Hook called just before a {@link SemanticsNode} is removed from the Android cache of Flutter's
   * semantics tree.
   */
  private void willRemoveSemanticsNode(SemanticsNode semanticsNodeToBeRemoved) {
    if (BuildConfig.DEBUG) {
      if (!flutterSemanticsTree.containsKey(semanticsNodeToBeRemoved.id)) {
        Log.e(TAG, "Attempted to remove a node that is not in the tree.");
      }
      if (flutterSemanticsTree.get(semanticsNodeToBeRemoved.id) != semanticsNodeToBeRemoved) {
        Log.e(TAG, "Flutter semantics tree failed to get expected node when searching by id.");
      }
    }
    // TODO(mattcarroll): should parent be set to "null" here? Changing the parent seems like the
    //                    behavior of a method called "removeSemanticsNode()". The same is true
    //                    for null'ing accessibilityFocusedSemanticsNode, inputFocusedSemanticsNode,
    //                    and hoveredObject.  Is this a hook method or a command?
    semanticsNodeToBeRemoved.parent = null;

    if (semanticsNodeToBeRemoved.platformViewId != -1
        && embeddedAccessibilityFocusedNodeId != null
        && accessibilityViewEmbedder.platformViewOfNode(embeddedAccessibilityFocusedNodeId)
            == platformViewsAccessibilityDelegate.getPlatformViewById(
                semanticsNodeToBeRemoved.platformViewId)) {
      // If the currently focused a11y node is within a platform view that is
      // getting removed: clear it's a11y focus.
      sendAccessibilityEvent(
          embeddedAccessibilityFocusedNodeId,
          AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUS_CLEARED);
      embeddedAccessibilityFocusedNodeId = null;
    }

    if (semanticsNodeToBeRemoved.platformViewId != -1) {
      View embeddedView =
          platformViewsAccessibilityDelegate.getPlatformViewById(
              semanticsNodeToBeRemoved.platformViewId);
      if (embeddedView != null) {
        embeddedView.setImportantForAccessibility(
            View.IMPORTANT_FOR_ACCESSIBILITY_NO_HIDE_DESCENDANTS);
      }
    }

    if (accessibilityFocusedSemanticsNode == semanticsNodeToBeRemoved) {
      sendAccessibilityEvent(
          accessibilityFocusedSemanticsNode.id,
          AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUS_CLEARED);
      accessibilityFocusedSemanticsNode = null;
    }

    if (inputFocusedSemanticsNode == semanticsNodeToBeRemoved) {
      inputFocusedSemanticsNode = null;
    }

    if (hoveredObject == semanticsNodeToBeRemoved) {
      hoveredObject = null;
    }
  }

  /**
   * Resets the {@code AccessibilityBridge}:
   *
   * <ul>
   *   <li>Clears {@link #flutterSemanticsTree}, the Android cache of Flutter's semantics tree
   *   <li>Releases focus on any active {@link #accessibilityFocusedSemanticsNode}
   *   <li>Clears any hovered {@code SemanticsNode}
   *   <li>Sends a {@link AccessibilityEvent#TYPE_WINDOW_CONTENT_CHANGED} event
   * </ul>
   */
  // TODO(mattcarroll): under what conditions is this method expected to be invoked?
  public void reset() {
    flutterSemanticsTree.clear();
    if (accessibilityFocusedSemanticsNode != null) {
      sendAccessibilityEvent(
          accessibilityFocusedSemanticsNode.id,
          AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUS_CLEARED);
    }
    accessibilityFocusedSemanticsNode = null;
    hoveredObject = null;
    sendWindowContentChangeEvent(0);
  }

  /**
   * Listener that can be set on a {@link AccessibilityBridge}, which is invoked any time
   * accessibility is turned on/off, or touch exploration is turned on/off.
   */
  public interface OnAccessibilityChangeListener {
    void onAccessibilityChanged(boolean isAccessibilityEnabled, boolean isTouchExplorationEnabled);
  }

  // Must match SemanticsActions in semantics.dart
  // https://github.com/flutter/engine/blob/main/lib/ui/semantics.dart
  public enum Action {
    TAP(1 << 0),
    LONG_PRESS(1 << 1),
    SCROLL_LEFT(1 << 2),
    SCROLL_RIGHT(1 << 3),
    SCROLL_UP(1 << 4),
    SCROLL_DOWN(1 << 5),
    INCREASE(1 << 6),
    DECREASE(1 << 7),
    SHOW_ON_SCREEN(1 << 8),
    MOVE_CURSOR_FORWARD_BY_CHARACTER(1 << 9),
    MOVE_CURSOR_BACKWARD_BY_CHARACTER(1 << 10),
    SET_SELECTION(1 << 11),
    COPY(1 << 12),
    CUT(1 << 13),
    PASTE(1 << 14),
    DID_GAIN_ACCESSIBILITY_FOCUS(1 << 15),
    DID_LOSE_ACCESSIBILITY_FOCUS(1 << 16),
    CUSTOM_ACTION(1 << 17),
    DISMISS(1 << 18),
    MOVE_CURSOR_FORWARD_BY_WORD(1 << 19),
    MOVE_CURSOR_BACKWARD_BY_WORD(1 << 20),
    SET_TEXT(1 << 21),
    FOCUS(1 << 22),
    SCROLL_TO_OFFSET(1 << 23);

    public final int value;

    Action(int value) {
      this.value = value;
    }
  }

  // Actions that are triggered by Android OS, as opposed to user-triggered actions.
  //
  // This int is intended to be use in a bitwise comparison.
  static int systemAction =
      Action.DID_GAIN_ACCESSIBILITY_FOCUS.value
          & Action.DID_LOSE_ACCESSIBILITY_FOCUS.value
          & Action.SHOW_ON_SCREEN.value;

  // Must match SemanticsFlag in semantics.dart
  // https://github.com/flutter/engine/blob/main/lib/ui/semantics.dart
  /* Package */ enum Flag {
    HAS_CHECKED_STATE(1 << 0),
    IS_CHECKED(1 << 1),
    IS_SELECTED(1 << 2),
    IS_BUTTON(1 << 3),
    IS_TEXT_FIELD(1 << 4),
    IS_FOCUSED(1 << 5),
    HAS_ENABLED_STATE(1 << 6),
    IS_ENABLED(1 << 7),
    IS_IN_MUTUALLY_EXCLUSIVE_GROUP(1 << 8),
    IS_HEADER(1 << 9),
    IS_OBSCURED(1 << 10),
    SCOPES_ROUTE(1 << 11),
    NAMES_ROUTE(1 << 12),
    IS_HIDDEN(1 << 13),
    IS_IMAGE(1 << 14),
    IS_LIVE_REGION(1 << 15),
    HAS_TOGGLED_STATE(1 << 16),
    IS_TOGGLED(1 << 17),
    HAS_IMPLICIT_SCROLLING(1 << 18),
    IS_MULTILINE(1 << 19),
    IS_READ_ONLY(1 << 20),
    IS_FOCUSABLE(1 << 21),
    IS_LINK(1 << 22),
    IS_SLIDER(1 << 23),
    IS_KEYBOARD_KEY(1 << 24),
    IS_CHECK_STATE_MIXED(1 << 25),
    HAS_EXPANDED_STATE(1 << 26),
    IS_EXPANDED(1 << 27),
    HAS_SELECTED_STATE(1 << 28);

    final int value;

    Flag(int value) {
      this.value = value;
    }
  }

  // Must match the enum defined in window.dart.
  private enum AccessibilityFeature {
    ACCESSIBLE_NAVIGATION(1 << 0),
    INVERT_COLORS(1 << 1), // NOT SUPPORTED
    DISABLE_ANIMATIONS(1 << 2),
    BOLD_TEXT(1 << 3), // NOT SUPPORTED
    REDUCE_MOTION(1 << 4), // NOT SUPPORTED
    HIGH_CONTRAST(1 << 5), // NOT SUPPORTED
    ON_OFF_SWITCH_LABELS(1 << 6); // NOT SUPPORTED

    final int value;

    AccessibilityFeature(int value) {
      this.value = value;
    }
  }

  private enum TextDirection {
    UNKNOWN,
    LTR,
    RTL;

    public static TextDirection fromInt(int value) {
      switch (value) {
        case 1:
          return RTL;
        case 2:
          return LTR;
      }
      return UNKNOWN;
    }
  }

  /**
   * Accessibility action that is defined within a given Flutter application, as opposed to the
   * standard accessibility actions that are available in the Flutter framework.
   *
   * <p>Flutter and Android support a number of built-in accessibility actions. However, these
   * predefined actions are not always sufficient for a desired interaction. Android facilitates
   * custom accessibility actions,
   * https://developer.android.com/reference/android/view/accessibility/AccessibilityNodeInfo.AccessibilityAction.
   * Flutter supports custom accessibility actions via {@code customSemanticsActions} within a
   * {@code Semantics} widget, https://api.flutter.dev/flutter/widgets/Semantics-class.html.
   *
   * <p>See the Android documentation for custom accessibility actions:
   * https://developer.android.com/reference/android/view/accessibility/AccessibilityNodeInfo.AccessibilityAction
   *
   * <p>See the Flutter documentation for the Semantics widget:
   * https://api.flutter.dev/flutter/widgets/Semantics-class.html
   */
  private static class CustomAccessibilityAction {
    CustomAccessibilityAction() {}

    // The ID of the custom action plus a minimum value so that the identifier
    // does not collide with existing Android accessibility actions. This ID
    // represents and Android resource ID, not a Flutter ID.
    private int resourceId = -1;

    // The Flutter ID of this custom accessibility action. See Flutter's Semantics widget for
    // custom accessibility action definitions:
    // https://api.flutter.dev/flutter/widgets/Semantics-class.html
    private int id = -1;

    // The ID of the standard Flutter accessibility action that this {@code
    // CustomAccessibilityAction}
    // overrides with a custom {@code label} and/or {@code hint}.
    private int overrideId = -1;

    // The user presented value which is displayed in the local context menu.
    private String label;

    // The text used in overridden standard actions.
    private String hint;
  }

  // When adding a new StringAttributeType, the classes in these file must be
  // updated as well.
  //  * engine/src/flutter/lib/ui/semantics.dart
  //  * engine/src/flutter/lib/web_ui/lib/semantics.dart
  //  * engine/src/flutter/lib/ui/semantics/string_attribute.h

  private enum StringAttributeType {
    SPELLOUT,
    LOCALE,
  }

  private static class StringAttribute {
    int start;
    int end;
    StringAttributeType type;
  }

  private static class SpellOutStringAttribute extends StringAttribute {}

  private static class LocaleStringAttribute extends StringAttribute {
    String locale;
  }

  /**
   * Flutter {@code SemanticsNode} represented in Java/Android.
   *
   * <p>Flutter maintains a semantics tree that is controlled by, but is independent of Flutter's
   * element tree, i.e., widgets/elements/render objects. Flutter's semantics tree must be cached on
   * the Android side so that Android can query any {@code SemanticsNode} at any time. This class
   * represents a single node in the semantics tree, and it is a Java representation of the
   * analogous concept within Flutter.
   *
   * <p>To see how this {@code SemanticsNode}'s fields correspond to Flutter's semantics system, see
   * semantics.dart: https://github.com/flutter/engine/blob/main/lib/ui/semantics.dart
   */
  private static class SemanticsNode {
    private static boolean nullableHasAncestor(
        SemanticsNode target, Predicate<SemanticsNode> tester) {
      return target != null && target.getAncestor(tester) != null;
    }

    final AccessibilityBridge accessibilityBridge;

    // Flutter ID of this {@code SemanticsNode}.
    private int id = -1;

    private int flags;
    private int actions;
    private int maxValueLength;
    private int currentValueLength;
    private int textSelectionBase;
    private int textSelectionExtent;
    private int platformViewId;
    private int scrollChildren;
    private int scrollIndex;
    private float scrollPosition;
    private float scrollExtentMax;
    private float scrollExtentMin;
    private String identifier;
    private String label;
    private List<StringAttribute> labelAttributes;
    private String value;
    private List<StringAttribute> valueAttributes;
    private String increasedValue;
    private List<StringAttribute> increasedValueAttributes;
    private String decreasedValue;
    private List<StringAttribute> decreasedValueAttributes;
    private String hint;
    private List<StringAttribute> hintAttributes;

    // The textual description of the backing widget's tooltip.
    //
    // The tooltip is attached through AccessibilityNodeInfo.setTooltipText if
    // API level >= 28; otherwise, this is attached to the end of content description.
    @Nullable private String tooltip;

    // The id of the sibling node that is before this node in traversal
    // order.
    //
    // The child order alone does not guarantee the TalkBack focus traversal
    // order. The AccessibilityNodeInfo.setTraversalAfter must be called with
    // its previous sibling to determine the focus traversal order.
    //
    // This property is updated in AccessibilityBridge.updateRecursively,
    // which is called at the end of every semantics update, and it is used in
    // AccessibilityBridge.createAccessibilityNodeInfo to set the "traversal
    // after" of this node.
    private int previousNodeId = -1;

    // See Flutter's {@code SemanticsNode#textDirection}.
    private TextDirection textDirection;

    private boolean hadPreviousConfig = false;
    private int previousFlags;
    private int previousActions;
    private int previousTextSelectionBase;
    private int previousTextSelectionExtent;
    private float previousScrollPosition;
    private float previousScrollExtentMax;
    private float previousScrollExtentMin;
    private String previousValue;
    private String previousLabel;

    private float left;
    private float top;
    private float right;
    private float bottom;
    private float[] transform;

    private SemanticsNode parent;
    private List<SemanticsNode> childrenInTraversalOrder = new ArrayList<>();
    private List<SemanticsNode> childrenInHitTestOrder = new ArrayList<>();
    private List<CustomAccessibilityAction> customAccessibilityActions;
    private CustomAccessibilityAction onTapOverride;
    private CustomAccessibilityAction onLongPressOverride;

    private boolean inverseTransformDirty = true;
    private float[] inverseTransform;

    private boolean globalGeometryDirty = true;
    private float[] globalTransform;
    private Rect globalRect;

    SemanticsNode(@NonNull AccessibilityBridge accessibilityBridge) {
      this.accessibilityBridge = accessibilityBridge;
    }

    /**
     * Returns the ancestor of this {@code SemanticsNode} for which {@link Predicate#test(Object)}
     * returns true, or null if no such ancestor exists.
     */
    private SemanticsNode getAncestor(Predicate<SemanticsNode> tester) {
      SemanticsNode nextAncestor = parent;
      while (nextAncestor != null) {
        if (tester.test(nextAncestor)) {
          return nextAncestor;
        }
        nextAncestor = nextAncestor.parent;
      }
      return null;
    }

    /**
     * Returns true if the given {@code action} is supported by this {@code SemanticsNode}.
     *
     * <p>This method only applies to this {@code SemanticsNode} and does not implicitly search its
     * children.
     */
    private boolean hasAction(@NonNull Action action) {
      return (actions & action.value) != 0;
    }

    /**
     * Returns true if the given {@code action} was supported by the immediately previous version of
     * this {@code SemanticsNode}.
     */
    private boolean hadAction(@NonNull Action action) {
      return (previousActions & action.value) != 0;
    }

    private boolean hasFlag(@NonNull Flag flag) {
      return (flags & flag.value) != 0;
    }

    private boolean hadFlag(@NonNull Flag flag) {
      if (BuildConfig.DEBUG && !hadPreviousConfig) {
        Log.e(TAG, "Attempted to check hadFlag but had no previous config.");
      }
      return (previousFlags & flag.value) != 0;
    }

    private boolean didScroll() {
      return !Float.isNaN(scrollPosition)
          && !Float.isNaN(previousScrollPosition)
          && previousScrollPosition != scrollPosition;
    }

    private boolean didChangeLabel() {
      if (label == null && previousLabel == null) {
        return false;
      }
      return label == null || previousLabel == null || !label.equals(previousLabel);
    }

    private void log(@NonNull String indent, boolean recursive) {
      if (BuildConfig.DEBUG) {
        Log.i(
            TAG,
            indent
                + "SemanticsNode id="
                + id
                + " identifier="
                + identifier
                + " label="
                + label
                + " actions="
                + actions
                + " flags="
                + flags
                + "\n"
                + indent
                + "  +-- textDirection="
                + textDirection
                + "\n"
                + indent
                + "  +-- rect.ltrb=("
                + left
                + ", "
                + top
                + ", "
                + right
                + ", "
                + bottom
                + ")\n"
                + indent
                + "  +-- transform="
                + Arrays.toString(transform)
                + "\n");
        if (recursive) {
          String childIndent = indent + "  ";
          for (SemanticsNode child : childrenInTraversalOrder) {
            child.log(childIndent, recursive);
          }
        }
      }
    }

    private void updateWith(
        @NonNull ByteBuffer buffer,
        @NonNull String[] strings,
        @NonNull ByteBuffer[] stringAttributeArgs) {
      hadPreviousConfig = true;
      previousValue = value;
      previousLabel = label;
      previousFlags = flags;
      previousActions = actions;
      previousTextSelectionBase = textSelectionBase;
      previousTextSelectionExtent = textSelectionExtent;
      previousScrollPosition = scrollPosition;
      previousScrollExtentMax = scrollExtentMax;
      previousScrollExtentMin = scrollExtentMin;

      flags = buffer.getInt();
      actions = buffer.getInt();
      maxValueLength = buffer.getInt();
      currentValueLength = buffer.getInt();
      textSelectionBase = buffer.getInt();
      textSelectionExtent = buffer.getInt();
      platformViewId = buffer.getInt();
      scrollChildren = buffer.getInt();
      scrollIndex = buffer.getInt();
      scrollPosition = buffer.getFloat();
      scrollExtentMax = buffer.getFloat();
      scrollExtentMin = buffer.getFloat();

      int stringIndex = buffer.getInt();

      identifier = stringIndex == -1 ? null : strings[stringIndex];
      stringIndex = buffer.getInt();

      label = stringIndex == -1 ? null : strings[stringIndex];

      labelAttributes = getStringAttributesFromBuffer(buffer, stringAttributeArgs);

      stringIndex = buffer.getInt();
      value = stringIndex == -1 ? null : strings[stringIndex];

      valueAttributes = getStringAttributesFromBuffer(buffer, stringAttributeArgs);

      stringIndex = buffer.getInt();
      increasedValue = stringIndex == -1 ? null : strings[stringIndex];

      increasedValueAttributes = getStringAttributesFromBuffer(buffer, stringAttributeArgs);

      stringIndex = buffer.getInt();
      decreasedValue = stringIndex == -1 ? null : strings[stringIndex];

      decreasedValueAttributes = getStringAttributesFromBuffer(buffer, stringAttributeArgs);

      stringIndex = buffer.getInt();
      hint = stringIndex == -1 ? null : strings[stringIndex];

      hintAttributes = getStringAttributesFromBuffer(buffer, stringAttributeArgs);

      stringIndex = buffer.getInt();
      tooltip = stringIndex == -1 ? null : strings[stringIndex];

      textDirection = TextDirection.fromInt(buffer.getInt());

      left = buffer.getFloat();
      top = buffer.getFloat();
      right = buffer.getFloat();
      bottom = buffer.getFloat();

      if (transform == null) {
        transform = new float[16];
      }
      for (int i = 0; i < 16; ++i) {
        transform[i] = buffer.getFloat();
      }
      inverseTransformDirty = true;
      globalGeometryDirty = true;

      final int childCount = buffer.getInt();
      childrenInTraversalOrder.clear();
      childrenInHitTestOrder.clear();
      for (int i = 0; i < childCount; ++i) {
        SemanticsNode child = accessibilityBridge.getOrCreateSemanticsNode(buffer.getInt());
        child.parent = this;
        childrenInTraversalOrder.add(child);
      }
      for (int i = 0; i < childCount; ++i) {
        SemanticsNode child = accessibilityBridge.getOrCreateSemanticsNode(buffer.getInt());
        child.parent = this;
        childrenInHitTestOrder.add(child);
      }

      final int actionCount = buffer.getInt();
      if (actionCount == 0) {
        customAccessibilityActions = null;
      } else {
        if (customAccessibilityActions == null)
          customAccessibilityActions = new ArrayList<>(actionCount);
        else customAccessibilityActions.clear();

        for (int i = 0; i < actionCount; i++) {
          CustomAccessibilityAction action =
              accessibilityBridge.getOrCreateAccessibilityAction(buffer.getInt());
          if (action.overrideId == Action.TAP.value) {
            onTapOverride = action;
          } else if (action.overrideId == Action.LONG_PRESS.value) {
            onLongPressOverride = action;
          } else {
            // If we receive a different overrideId it means that we were passed
            // a standard action to override that we don't yet support.
            if (BuildConfig.DEBUG && action.overrideId != -1) {
              Log.e(TAG, "Expected action.overrideId to be -1.");
            }
            customAccessibilityActions.add(action);
          }
          customAccessibilityActions.add(action);
        }
      }
    }

    private List<StringAttribute> getStringAttributesFromBuffer(
        @NonNull ByteBuffer buffer, @NonNull ByteBuffer[] stringAttributeArgs) {
      final int attributesCount = buffer.getInt();
      if (attributesCount == -1) {
        return null;
      }
      final List<StringAttribute> result = new ArrayList<>(attributesCount);
      for (int i = 0; i < attributesCount; ++i) {
        final int start = buffer.getInt();
        final int end = buffer.getInt();
        final StringAttributeType type = StringAttributeType.values()[buffer.getInt()];
        switch (type) {
          case SPELLOUT:
            {
              // Pops the -1 size.
              buffer.getInt();
              SpellOutStringAttribute attribute = new SpellOutStringAttribute();
              attribute.start = start;
              attribute.end = end;
              attribute.type = type;
              result.add(attribute);
              break;
            }
          case LOCALE:
            {
              final int argsIndex = buffer.getInt();
              final ByteBuffer args = stringAttributeArgs[argsIndex];
              LocaleStringAttribute attribute = new LocaleStringAttribute();
              attribute.start = start;
              attribute.end = end;
              attribute.type = type;
              attribute.locale = Charset.forName("UTF-8").decode(args).toString();
              result.add(attribute);
              break;
            }
          default:
            break;
        }
      }
      return result;
    }

    private void ensureInverseTransform() {
      if (!inverseTransformDirty) {
        return;
      }
      inverseTransformDirty = false;
      if (inverseTransform == null) {
        inverseTransform = new float[16];
      }
      if (!Matrix.invertM(inverseTransform, 0, transform, 0)) {
        Arrays.fill(inverseTransform, 0);
      }
    }

    private Rect getGlobalRect() {
      if (BuildConfig.DEBUG && globalGeometryDirty) {
        Log.e(TAG, "Attempted to getGlobalRect with a dirty geometry.");
      }
      return globalRect;
    }

    /**
     * Hit tests {@code point} to find the deepest focusable node in the node tree at that point.
     *
     * @param point The point to hit test against this node.
     * @param stopAtPlatformView Whether to return a platform view if found, regardless of whether
     *     or not it is focusable.
     * @return The found node, or null if no relevant node was found at the given point.
     */
    private SemanticsNode hitTest(float[] point, boolean stopAtPlatformView) {
      final float w = point[3];
      final float x = point[0] / w;
      final float y = point[1] / w;
      if (x < left || x >= right || y < top || y >= bottom) return null;
      final float[] transformedPoint = new float[4];
      for (SemanticsNode child : childrenInHitTestOrder) {
        if (child.hasFlag(Flag.IS_HIDDEN)) {
          continue;
        }
        child.ensureInverseTransform();
        Matrix.multiplyMV(transformedPoint, 0, child.inverseTransform, 0, point, 0);
        final SemanticsNode result = child.hitTest(transformedPoint, stopAtPlatformView);
        if (result != null) {
          return result;
        }
      }
      final boolean foundPlatformView = stopAtPlatformView && platformViewId != -1;
      return isFocusable() || foundPlatformView ? this : null;
    }

    // TODO(goderbauer): This should be decided by the framework once we have more information
    //     about focusability there.
    private boolean isFocusable() {
      // We enforce in the framework that no other useful semantics are merged with these
      // nodes.
      if (hasFlag(Flag.SCOPES_ROUTE)) {
        return false;
      }
      if (hasFlag(Flag.IS_FOCUSABLE)) {
        return true;
      }
      // If not explicitly set as focusable, then use our legacy
      // algorithm. Once all focusable widgets have a Focus widget, then
      // this won't be needed.
      return (actions & ~SCROLLABLE_ACTIONS) != 0
          || (flags & FOCUSABLE_FLAGS) != 0
          || (label != null && !label.isEmpty())
          || (value != null && !value.isEmpty())
          || (hint != null && !hint.isEmpty());
    }

    private void collectRoutes(List<SemanticsNode> edges) {
      if (hasFlag(Flag.SCOPES_ROUTE)) {
        edges.add(this);
      }
      for (SemanticsNode child : childrenInTraversalOrder) {
        child.collectRoutes(edges);
      }
    }

    private String getRouteName() {
      // Returns the first non-null and non-empty semantic label of a child
      // with an NamesRoute flag. Otherwise returns null.
      if (hasFlag(Flag.NAMES_ROUTE)) {
        if (label != null && !label.isEmpty()) {
          return label;
        }
      }
      for (SemanticsNode child : childrenInTraversalOrder) {
        String newName = child.getRouteName();
        if (newName != null && !newName.isEmpty()) {
          return newName;
        }
      }
      return null;
    }

    private void updateRecursively(
        float[] ancestorTransform, Set<SemanticsNode> visitedObjects, boolean forceUpdate) {
      visitedObjects.add(this);

      if (globalGeometryDirty) {
        forceUpdate = true;
      }

      if (forceUpdate) {
        if (globalTransform == null) {
          globalTransform = new float[16];
        }
        if (transform == null) {
          if (BuildConfig.DEBUG) {
            Log.e(TAG, "transform has not been initialized for id = " + id);
            accessibilityBridge.getRootSemanticsNode().log("Semantics tree:", true);
          }
          transform = new float[16];
        }
        Matrix.multiplyMM(globalTransform, 0, ancestorTransform, 0, transform, 0);

        final float[] sample = new float[4];
        sample[2] = 0;
        sample[3] = 1;

        final float[] point1 = new float[4];
        final float[] point2 = new float[4];
        final float[] point3 = new float[4];
        final float[] point4 = new float[4];

        sample[0] = left;
        sample[1] = top;
        transformPoint(point1, globalTransform, sample);

        sample[0] = right;
        sample[1] = top;
        transformPoint(point2, globalTransform, sample);

        sample[0] = right;
        sample[1] = bottom;
        transformPoint(point3, globalTransform, sample);

        sample[0] = left;
        sample[1] = bottom;
        transformPoint(point4, globalTransform, sample);

        if (globalRect == null) globalRect = new Rect();

        globalRect.set(
            Math.round(min(point1[0], point2[0], point3[0], point4[0])),
            Math.round(min(point1[1], point2[1], point3[1], point4[1])),
            Math.round(max(point1[0], point2[0], point3[0], point4[0])),
            Math.round(max(point1[1], point2[1], point3[1], point4[1])));

        globalGeometryDirty = false;
      }

      if (BuildConfig.DEBUG) {
        if (globalTransform == null) {
          Log.e(TAG, "Expected globalTransform to not be null.");
        }
        if (globalRect == null) {
          Log.e(TAG, "Expected globalRect to not be null.");
        }
      }

      int previousNodeId = -1;
      for (SemanticsNode child : childrenInTraversalOrder) {
        child.previousNodeId = previousNodeId;
        previousNodeId = child.id;
        child.updateRecursively(globalTransform, visitedObjects, forceUpdate);
      }
    }

    private void transformPoint(float[] result, float[] transform, float[] point) {
      Matrix.multiplyMV(result, 0, transform, 0, point, 0);
      final float w = result[3];
      result[0] /= w;
      result[1] /= w;
      result[2] /= w;
      result[3] = 0;
    }

    private float min(float a, float b, float c, float d) {
      return Math.min(a, Math.min(b, Math.min(c, d)));
    }

    private float max(float a, float b, float c, float d) {
      return Math.max(a, Math.max(b, Math.max(c, d)));
    }

    private CharSequence getValue() {
      return createSpannableString(value, valueAttributes);
    }

    private CharSequence getLabel() {
      return createSpannableString(label, labelAttributes);
    }

    private CharSequence getHint() {
      return createSpannableString(hint, hintAttributes);
    }

    private CharSequence getValueLabelHint() {
      CharSequence[] array = new CharSequence[] {getValue(), getLabel(), getHint()};
      CharSequence result = null;
      for (CharSequence word : array) {
        if (word != null && word.length() > 0) {
          if (result == null || result.length() == 0) {
            result = word;
          } else {
            result = TextUtils.concat(result, ", ", word);
          }
        }
      }
      return result;
    }

    private CharSequence getTextFieldHint() {
      CharSequence[] array = new CharSequence[] {getLabel(), getHint()};
      CharSequence result = null;
      for (CharSequence word : array) {
        if (word != null && word.length() > 0) {
          if (result == null || result.length() == 0) {
            result = word;
          } else {
            result = TextUtils.concat(result, ", ", word);
          }
        }
      }
      return result;
    }

    private SpannableString createSpannableString(String string, List<StringAttribute> attributes) {
      if (string == null) {
        return null;
      }
      final SpannableString spannableString = new SpannableString(string);
      if (attributes != null) {
        for (StringAttribute attribute : attributes) {
          switch (attribute.type) {
            case SPELLOUT:
              {
                final TtsSpan ttsSpan = new TtsSpan.Builder<>(TtsSpan.TYPE_VERBATIM).build();
                spannableString.setSpan(ttsSpan, attribute.start, attribute.end, 0);
                break;
              }
            case LOCALE:
              {
                LocaleStringAttribute localeAttribute = (LocaleStringAttribute) attribute;
                Locale locale = Locale.forLanguageTag(localeAttribute.locale);
                final LocaleSpan localeSpan = new LocaleSpan(locale);
                spannableString.setSpan(localeSpan, attribute.start, attribute.end, 0);
                break;
              }
          }
        }
      }
      return spannableString;
    }
  }

  /**
   * Delegates handling of {@link android.view.ViewParent#requestSendAccessibilityEvent} to the
   * accessibility bridge.
   *
   * <p>This is used by embedded platform views to propagate accessibility events from their view
   * hierarchy to the accessibility bridge.
   *
   * <p>As the embedded view doesn't have to be the only View in the embedded hierarchy (it can have
   * child views) and the event might have been originated from any view in this hierarchy, this
   * method gets both a reference to the embedded platform view, and a reference to the view from
   * its hierarchy that sent the event.
   *
   * @param embeddedView the embedded platform view for which the event is delegated
   * @param eventOrigin the view in the embedded view's hierarchy that sent the event.
   * @return True if the event was sent.
   */
  // AccessibilityEvent has many irrelevant cases that would be confusing to list.
  @SuppressLint("SwitchIntDef")
  public boolean externalViewRequestSendAccessibilityEvent(
      View embeddedView, View eventOrigin, AccessibilityEvent event) {
    if (!accessibilityViewEmbedder.requestSendAccessibilityEvent(
        embeddedView, eventOrigin, event)) {
      return false;
    }
    Integer virtualNodeId = accessibilityViewEmbedder.getRecordFlutterId(embeddedView, event);
    if (virtualNodeId == null) {
      return false;
    }
    switch (event.getEventType()) {
      case AccessibilityEvent.TYPE_VIEW_HOVER_ENTER:
        hoveredObject = null;
        break;
      case AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUSED:
        embeddedAccessibilityFocusedNodeId = virtualNodeId;
        accessibilityFocusedSemanticsNode = null;
        break;
      case AccessibilityEvent.TYPE_VIEW_ACCESSIBILITY_FOCUS_CLEARED:
        embeddedInputFocusedNodeId = null;
        embeddedAccessibilityFocusedNodeId = null;
        break;
      case AccessibilityEvent.TYPE_VIEW_FOCUSED:
        embeddedInputFocusedNodeId = virtualNodeId;
        inputFocusedSemanticsNode = null;
        break;
    }
    return true;
  }
}
