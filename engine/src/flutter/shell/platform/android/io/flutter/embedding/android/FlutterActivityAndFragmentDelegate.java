// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static android.content.ComponentCallbacks2.TRIM_MEMORY_RUNNING_LOW;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.DEFAULT_INITIAL_ROUTE;

import android.annotation.TargetApi;
import android.app.Activity;
import android.content.ComponentCallbacks2;
import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.ViewTreeObserver.OnPreDrawListener;
import android.window.BackEvent;
import android.window.OnBackAnimationCallback;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.annotation.VisibleForTesting;
import androidx.lifecycle.Lifecycle;
import io.flutter.Build.API_LEVELS;
import io.flutter.FlutterInjector;
import io.flutter.Log;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineCache;
import io.flutter.embedding.engine.FlutterEngineGroup;
import io.flutter.embedding.engine.FlutterEngineGroupCache;
import io.flutter.embedding.engine.FlutterShellArgs;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.renderer.FlutterUiDisplayListener;
import io.flutter.plugin.platform.PlatformPlugin;
import io.flutter.plugin.view.SensitiveContentPlugin;
import java.util.Arrays;
import java.util.List;

/**
 * Delegate that implements all Flutter logic that is the same between a {@link FlutterActivity} and
 * a {@link FlutterFragment}.
 *
 * <p><strong>Why does this class exist?</strong>
 *
 * <p>One might ask why an {@code Activity} and {@code Fragment} delegate needs to exist. Given that
 * a {@code Fragment} can be placed within an {@code Activity}, it would make more sense to use a
 * {@link FlutterFragment} within a {@link FlutterActivity}.
 *
 * <p>The {@code Fragment} support library adds 100k of binary size to an app, and full-Flutter apps
 * do not otherwise require that binary hit. Therefore, it was concluded that Flutter must provide a
 * {@link FlutterActivity} based on the AOSP {@code Activity}, and an independent {@link
 * FlutterFragment} for add-to-app developers.
 *
 * <p>If a time ever comes where the inclusion of {@code Fragment}s in a full-Flutter app is no
 * longer deemed an issue, this class should be immediately decomposed between {@link
 * FlutterActivity} and {@link FlutterFragment} and then eliminated.
 *
 * <p><strong>Caution when modifying this class</strong>
 *
 * <p>Any time that a "delegate" is created with the purpose of encapsulating the internal behaviors
 * of another object, that delegate is highly susceptible to degeneration. It is easy to tack new
 * responsibilities on to the delegate which would not otherwise be added to the original object. It
 * is also easy to begin hanging listeners and callbacks on a delegate object that likewise would
 * not be added to the original object. A delegate can quickly become a complex web of dependencies
 * and optional references that are very difficult to track.
 *
 * <p>Maintainers of this class should take care to only place code in this delegate that would
 * otherwise be placed in either {@link FlutterActivity} or {@link FlutterFragment}, and in exactly
 * the same form. <strong>Do not use this class as a convenient shortcut for any other
 * behavior.</strong>
 */
/* package */ class FlutterActivityAndFragmentDelegate implements ExclusiveAppComponent<Activity> {
  private static final String TAG = "FlutterActivityAndFragmentDelegate";
  private static final String FRAMEWORK_RESTORATION_BUNDLE_KEY = "framework";
  private static final String PLUGINS_RESTORATION_BUNDLE_KEY = "plugins";
  static final String ON_BACK_CALLBACK_ENABLED_KEY = "enableOnBackInvokedCallbackState";
  private static final int FLUTTER_SPLASH_VIEW_FALLBACK_ID = 486947586;

  /** Factory to obtain a FlutterActivityAndFragmentDelegate instance. */
  public interface DelegateFactory {
    FlutterActivityAndFragmentDelegate createDelegate(FlutterActivityAndFragmentDelegate.Host host);
  }

  // The FlutterActivity or FlutterFragment that is delegating most of its calls
  // to this FlutterActivityAndFragmentDelegate.
  @NonNull private Host host;
  @Nullable private FlutterEngine flutterEngine;
  @VisibleForTesting @Nullable FlutterView flutterView;
  @Nullable private PlatformPlugin platformPlugin;
  @Nullable private SensitiveContentPlugin sensitiveContentPlugin;
  @VisibleForTesting @Nullable OnPreDrawListener activePreDrawListener;
  private boolean isFlutterEngineFromHost;
  private boolean isFlutterUiDisplayed;
  private boolean isFirstFrameRendered;
  private boolean isAttached;
  private Integer previousVisibility;
  @Nullable private FlutterEngineGroup engineGroup;

  @NonNull
  private final FlutterUiDisplayListener flutterUiDisplayListener =
      new FlutterUiDisplayListener() {
        @Override
        public void onFlutterUiDisplayed() {
          host.onFlutterUiDisplayed();
          isFlutterUiDisplayed = true;
          isFirstFrameRendered = true;
        }

        @Override
        public void onFlutterUiNoLongerDisplayed() {
          host.onFlutterUiNoLongerDisplayed();
          isFlutterUiDisplayed = false;
        }
      };

  FlutterActivityAndFragmentDelegate(@NonNull Host host) {
    this(host, null);
  }

  FlutterActivityAndFragmentDelegate(@NonNull Host host, @Nullable FlutterEngineGroup engineGroup) {
    this.host = host;
    this.isFirstFrameRendered = false;
    this.engineGroup = engineGroup;
  }

  /**
   * Disconnects this {@code FlutterActivityAndFragmentDelegate} from its host {@code Activity} or
   * {@code Fragment}.
   *
   * <p>No further method invocations may occur on this {@code FlutterActivityAndFragmentDelegate}
   * after invoking this method. If a method is invoked, an exception will occur.
   *
   * <p>This method only clears out references. It does not destroy its {@link
   * io.flutter.embedding.engine.FlutterEngine}. The behavior that destroys a {@link
   * io.flutter.embedding.engine.FlutterEngine} can be found in {@link #onDetach()}.
   */
  void release() {
    this.host = null;
    this.flutterEngine = null;
    this.flutterView = null;
    this.platformPlugin = null;
    this.sensitiveContentPlugin = null;
  }

  /**
   * Returns the {@link io.flutter.embedding.engine.FlutterEngine} that is owned by this delegate
   * and its host {@code Activity} or {@code Fragment}.
   */
  @Nullable
  /* package */ FlutterEngine getFlutterEngine() {
    return flutterEngine;
  }

  /**
   * Returns true if the host {@code Activity}/{@code Fragment} provided a {@code FlutterEngine}, as
   * opposed to this delegate creating a new one.
   */
  /* package */ boolean isFlutterEngineFromHost() {
    return isFlutterEngineFromHost;
  }

  /**
   * Whether or not this {@code FlutterActivityAndFragmentDelegate} is attached to a {@code
   * FlutterEngine}.
   */
  /* package */ boolean isAttached() {
    return isAttached;
  }

  /**
   * Invoke this method from {@code Activity#onCreate(Bundle)} or {@code
   * Fragment#onAttach(Context)}.
   *
   * <p>This method does the following:
   *
   * <p>
   *
   * <ol>
   *   <li>Initializes the Flutter system.
   *   <li>Obtains or creates a {@link io.flutter.embedding.engine.FlutterEngine}.
   *   <li>Creates and configures a {@link PlatformPlugin}.
   *   <li>Attaches the {@link io.flutter.embedding.engine.FlutterEngine} to the surrounding {@code
   *       Activity}, if desired.
   *   <li>Configures the {@link io.flutter.embedding.engine.FlutterEngine} via {@link
   *       Host#configureFlutterEngine(FlutterEngine)}.
   * </ol>
   */
  void onAttach(@NonNull Context context) {
    ensureAlive();

    // When "retain instance" is true, the FlutterEngine will survive configuration
    // changes. Therefore, we create a new one only if one does not already exist.
    if (flutterEngine == null) {
      setUpFlutterEngine();
    }

    if (host.shouldAttachEngineToActivity()) {
      // Notify any plugins that are currently attached to our FlutterEngine that they
      // are now attached to an Activity.
      //
      // Passing this Fragment's Lifecycle should be sufficient because as long as this Fragment
      // is attached to its Activity, the lifecycles should be in sync. Once this Fragment is
      // detached from its Activity, that Activity will be detached from the FlutterEngine, too,
      // which means there shouldn't be any possibility for the Fragment Lifecycle to get out of
      // sync with the Activity. We use the Fragment's Lifecycle because it is possible that the
      // attached Activity is not a LifecycleOwner.
      Log.v(TAG, "Attaching FlutterEngine to the Activity that owns this delegate.");
      flutterEngine.getActivityControlSurface().attachToActivity(this, host.getLifecycle());
    }

    // Regardless of whether or not a FlutterEngine already existed, the PlatformPlugin and
    // SensitiveContetPlugin are bound to a specific Activity. Therefore, they need to be
    // created and configured every time this Fragment attaches to a new Activity.
    // TODO(mattcarroll): the PlatformPlugin needs to be reimagined because it implicitly takes
    //                    control of the entire window. This is unacceptable for non-fullscreen
    //                    use-cases.
    platformPlugin = host.providePlatformPlugin(host.getActivity(), flutterEngine);
    sensitiveContentPlugin = host.provideSensitiveContentPlugin(host.getActivity(), flutterEngine);

    host.configureFlutterEngine(flutterEngine);
    isAttached = true;
  }

  @Override
  public @NonNull Activity getAppComponent() {
    final Activity activity = host.getActivity();
    if (activity == null) {
      throw new AssertionError(
          "FlutterActivityAndFragmentDelegate's getAppComponent should only "
              + "be queried after onAttach, when the host's activity should always be non-null");
    }
    return activity;
  }

  private FlutterEngineGroup.Options addEntrypointOptions(FlutterEngineGroup.Options options) {
    String appBundlePathOverride = host.getAppBundlePath();
    if (appBundlePathOverride == null || appBundlePathOverride.isEmpty()) {
      appBundlePathOverride = FlutterInjector.instance().flutterLoader().findAppBundlePath();
    }

    DartExecutor.DartEntrypoint dartEntrypoint =
        new DartExecutor.DartEntrypoint(
            appBundlePathOverride, host.getDartEntrypointFunctionName());
    String initialRoute = host.getInitialRoute();
    if (initialRoute == null) {
      initialRoute = maybeGetInitialRouteFromIntent(host.getActivity().getIntent());
      if (initialRoute == null) {
        initialRoute = DEFAULT_INITIAL_ROUTE;
      }
    }
    return options
        .setDartEntrypoint(dartEntrypoint)
        .setInitialRoute(initialRoute)
        .setDartEntrypointArgs(host.getDartEntrypointArgs());
  }

  /**
   * Obtains a reference to a FlutterEngine to back this delegate and its {@code host}.
   *
   * <p>
   *
   * <p>First, the {@code host} is asked if it would like to use a cached {@link
   * io.flutter.embedding.engine.FlutterEngine}, and if so, the cached {@link
   * io.flutter.embedding.engine.FlutterEngine} is retrieved.
   *
   * <p>Second, the {@code host} is given an opportunity to provide a {@link
   * io.flutter.embedding.engine.FlutterEngine} via {@link Host#provideFlutterEngine(Context)}.
   *
   * <p>Third, the {@code host} is asked if it would like to use a cached {@link
   * io.flutter.embedding.engine.FlutterEngineGroup} to create a new {@link FlutterEngine} by {@link
   * FlutterEngineGroup#createAndRunEngine}
   *
   * <p>If the {@code host} does not provide a {@link io.flutter.embedding.engine.FlutterEngine},
   * then a new {@link FlutterEngine} is instantiated.
   */
  @VisibleForTesting
  /* package */ void setUpFlutterEngine() {
    Log.v(TAG, "Setting up FlutterEngine.");

    // First, check if the host wants to use a cached FlutterEngine.
    String cachedEngineId = host.getCachedEngineId();
    if (cachedEngineId != null) {
      flutterEngine = FlutterEngineCache.getInstance().get(cachedEngineId);
      isFlutterEngineFromHost = true;
      if (flutterEngine == null) {
        throw new IllegalStateException(
            "The requested cached FlutterEngine did not exist in the FlutterEngineCache: '"
                + cachedEngineId
                + "'");
      }
      return;
    }

    // Second, defer to subclasses for a custom FlutterEngine.
    flutterEngine = host.provideFlutterEngine(host.getContext());
    if (flutterEngine != null) {
      isFlutterEngineFromHost = true;
      return;
    }

    // Third, check if the host wants to use a cached FlutterEngineGroup
    // and create new FlutterEngine using FlutterEngineGroup#createAndRunEngine
    String cachedEngineGroupId = host.getCachedEngineGroupId();
    if (cachedEngineGroupId != null) {
      FlutterEngineGroup flutterEngineGroup =
          FlutterEngineGroupCache.getInstance().get(cachedEngineGroupId);
      if (flutterEngineGroup == null) {
        throw new IllegalStateException(
            "The requested cached FlutterEngineGroup did not exist in the FlutterEngineGroupCache: '"
                + cachedEngineGroupId
                + "'");
      }

      flutterEngine =
          flutterEngineGroup.createAndRunEngine(
              addEntrypointOptions(new FlutterEngineGroup.Options(host.getContext())));
      isFlutterEngineFromHost = false;
      return;
    }

    // Our host did not provide a custom FlutterEngine. Create a FlutterEngine to back our
    // FlutterView.
    Log.v(
        TAG,
        "No preferred FlutterEngine was provided. Creating a new FlutterEngine for"
            + " this FlutterFragment.");

    FlutterEngineGroup group =
        engineGroup == null
            ? new FlutterEngineGroup(host.getContext(), host.getFlutterShellArgs().toArray())
            : engineGroup;
    flutterEngine =
        group.createAndRunEngine(
            addEntrypointOptions(
                new FlutterEngineGroup.Options(host.getContext())
                    .setAutomaticallyRegisterPlugins(false)
                    .setWaitForRestorationData(host.shouldRestoreAndSaveState())));
    isFlutterEngineFromHost = false;
  }

  /**
   * Invoke this method from {@code Activity#onCreate(Bundle)} to create the content {@code View},
   * or from {@code Fragment#onCreateView(LayoutInflater, ViewGroup, Bundle)}.
   *
   * <p>{@code inflater} and {@code container} may be null when invoked from an {@code Activity}.
   *
   * <p>{@code shouldDelayFirstAndroidViewDraw} determines whether to set up an {@link
   * android.view.ViewTreeObserver.OnPreDrawListener}, which will defer the current drawing pass
   * till after the Flutter UI has been displayed. This results in more accurate timings reported
   * with Android tools, such as "Displayed" timing printed with `am start`.
   *
   * <p>Note that it should only be set to true when {@code Host#getRenderMode()} is {@code
   * RenderMode.surface}.
   *
   * <p>This method:
   *
   * <ol>
   *   <li>creates a new {@link FlutterView} in a {@code View} hierarchy
   *   <li>adds a {@link FlutterUiDisplayListener} to it
   *   <li>attaches a {@link io.flutter.embedding.engine.FlutterEngine} to the new {@link
   *       FlutterView}
   *   <li>returns the new {@code View} hierarchy
   * </ol>
   */
  @NonNull
  View onCreateView(
      LayoutInflater inflater,
      @Nullable ViewGroup container,
      @Nullable Bundle savedInstanceState,
      int flutterViewId,
      boolean shouldDelayFirstAndroidViewDraw) {
    Log.v(TAG, "Creating FlutterView.");
    ensureAlive();

    if (host.getRenderMode() == RenderMode.surface) {
      FlutterSurfaceView flutterSurfaceView =
          new FlutterSurfaceView(
              host.getContext(), host.getTransparencyMode() == TransparencyMode.transparent);

      // Allow our host to customize FlutterSurfaceView, if desired.
      host.onFlutterSurfaceViewCreated(flutterSurfaceView);

      // Create the FlutterView that owns the FlutterSurfaceView.
      flutterView = new FlutterView(host.getContext(), flutterSurfaceView);
    } else {
      FlutterTextureView flutterTextureView = new FlutterTextureView(host.getContext());

      flutterTextureView.setOpaque(host.getTransparencyMode() == TransparencyMode.opaque);

      // Allow our host to customize FlutterSurfaceView, if desired.
      host.onFlutterTextureViewCreated(flutterTextureView);

      // Create the FlutterView that owns the FlutterTextureView.
      flutterView = new FlutterView(host.getContext(), flutterTextureView);
    }

    // Add listener to be notified when Flutter renders its first frame.
    flutterView.addOnFirstFrameRenderedListener(flutterUiDisplayListener);

    if (host.attachToEngineAutomatically()) {
      Log.v(TAG, "Attaching FlutterEngine to FlutterView.");
      flutterView.attachToFlutterEngine(flutterEngine);
    }
    flutterView.setId(flutterViewId);

    if (shouldDelayFirstAndroidViewDraw) {
      delayFirstAndroidViewDraw(flutterView);
    }
    return flutterView;
  }

  void onRestoreInstanceState(@Nullable Bundle bundle) {
    Log.v(
        TAG,
        "onRestoreInstanceState. Giving framework and plugins an opportunity to restore state.");
    ensureAlive();

    Bundle pluginState = null;
    byte[] frameworkState = null;
    if (bundle != null) {
      pluginState = bundle.getBundle(PLUGINS_RESTORATION_BUNDLE_KEY);
      frameworkState = bundle.getByteArray(FRAMEWORK_RESTORATION_BUNDLE_KEY);
    }

    if (host.shouldRestoreAndSaveState()) {
      flutterEngine.getRestorationChannel().setRestorationData(frameworkState);
    }

    if (host.shouldAttachEngineToActivity()) {
      flutterEngine.getActivityControlSurface().onRestoreInstanceState(pluginState);
    }
  }

  /**
   * Invoke this from {@code Activity#onStart()} or {@code Fragment#onStart()}.
   *
   * <p>This method:
   *
   * <p>
   *
   * <ol>
   *   <li>Begins executing Dart code, if it is not already executing.
   * </ol>
   */
  void onStart() {
    Log.v(TAG, "onStart()");
    ensureAlive();
    doInitialFlutterViewRun();
    // This is a workaround for a bug on some OnePlus phones. The visibility of the application
    // window is still true after locking the screen on some OnePlus phones, and shows a black
    // screen when unlocked. We can work around this by changing the visibility of FlutterView in
    // onStart and onStop.
    // See https://github.com/flutter/flutter/issues/93276
    if (previousVisibility != null) {
      flutterView.setVisibility(previousVisibility);
    }
  }

  /**
   * Starts running Dart within the FlutterView for the first time.
   *
   * <p>Reloading/restarting Dart within a given FlutterView is not supported. If this method is
   * invoked while Dart is already executing then it does nothing.
   *
   * <p>{@code flutterEngine} must be non-null when invoking this method.
   */
  private void doInitialFlutterViewRun() {
    // Don't attempt to start a FlutterEngine if we're using a cached FlutterEngine.
    if (host.getCachedEngineId() != null) {
      return;
    }

    if (flutterEngine.getDartExecutor().isExecutingDart()) {
      // No warning is logged because this situation will happen on every config
      // change if the developer does not choose to retain the Fragment instance.
      // So this is expected behavior in many cases.
      return;
    }
    String initialRoute = host.getInitialRoute();
    if (initialRoute == null) {
      initialRoute = maybeGetInitialRouteFromIntent(host.getActivity().getIntent());
      if (initialRoute == null) {
        initialRoute = DEFAULT_INITIAL_ROUTE;
      }
    }
    @Nullable String libraryUri = host.getDartEntrypointLibraryUri();
    Log.v(
        TAG,
        "Executing Dart entrypoint: "
                    + host.getDartEntrypointFunctionName()
                    + ", library uri: "
                    + libraryUri
                == null
            ? "\"\""
            : libraryUri + ", and sending initial route: " + initialRoute);

    // The engine needs to receive the Flutter app's initial route before executing any
    // Dart code to ensure that the initial route arrives in time to be applied.
    flutterEngine.getNavigationChannel().setInitialRoute(initialRoute);

    String appBundlePathOverride = host.getAppBundlePath();
    if (appBundlePathOverride == null || appBundlePathOverride.isEmpty()) {
      appBundlePathOverride = FlutterInjector.instance().flutterLoader().findAppBundlePath();
    }

    // Configure the Dart entrypoint and execute it.
    DartExecutor.DartEntrypoint entrypoint =
        libraryUri == null
            ? new DartExecutor.DartEntrypoint(
                appBundlePathOverride, host.getDartEntrypointFunctionName())
            : new DartExecutor.DartEntrypoint(
                appBundlePathOverride, libraryUri, host.getDartEntrypointFunctionName());
    flutterEngine.getDartExecutor().executeDartEntrypoint(entrypoint, host.getDartEntrypointArgs());
  }

  private String maybeGetInitialRouteFromIntent(Intent intent) {
    if (host.shouldHandleDeeplinking()) {
      Uri data = intent.getData();
      if (data != null) {
        return data.toString();
      }
    }
    return null;
  }

  /**
   * Delays the first drawing of the {@code flutterView} until the Flutter first has been displayed.
   */
  private void delayFirstAndroidViewDraw(FlutterView flutterView) {
    if (host.getRenderMode() != RenderMode.surface) {
      // Using a TextureView will cause a deadlock, where the underlying SurfaceTexture is never
      // available since it will wait for drawing to be completed first. At the same time, the
      // preDraw listener keeps returning false since the Flutter Engine waits for the
      // SurfaceTexture to be available.
      throw new IllegalArgumentException(
          "Cannot delay the first Android view draw when the render mode is not set to"
              + " `RenderMode.surface`.");
    }

    if (activePreDrawListener != null) {
      flutterView.getViewTreeObserver().removeOnPreDrawListener(activePreDrawListener);
    }

    activePreDrawListener =
        new OnPreDrawListener() {
          @Override
          public boolean onPreDraw() {
            if (isFlutterUiDisplayed && activePreDrawListener != null) {
              flutterView.getViewTreeObserver().removeOnPreDrawListener(this);
              activePreDrawListener = null;
            }
            return isFlutterUiDisplayed;
          }
        };
    flutterView.getViewTreeObserver().addOnPreDrawListener(activePreDrawListener);
  }

  /**
   * Invoke this from {@code Activity#onResume()} or {@code Fragment#onResume()}.
   *
   * <p>This method notifies the running Flutter app that it is "resumed" as per the Flutter app
   * lifecycle.
   */
  void onResume() {
    Log.v(TAG, "onResume()");
    ensureAlive();
    if (host.shouldDispatchAppLifecycleState() && flutterEngine != null) {
      flutterEngine.getLifecycleChannel().appIsResumed();
    }
  }

  /**
   * Invoke this from {@code Activity#onPostResume()}.
   *
   * <p>A {@code Fragment} host must have its containing {@code Activity} forward this call so that
   * the {@code Fragment} can then invoke this method.
   *
   * <p>This method informs the {@link PlatformPlugin} that {@code onPostResume()} has run, which is
   * used to update system UI overlays.
   */
  // TODO(mattcarroll): determine why this can't be in onResume(). Comment reason, or move if
  // possible.
  void onPostResume() {
    Log.v(TAG, "onPostResume()");
    ensureAlive();
    if (flutterEngine != null) {
      updateSystemUiOverlays();
      flutterEngine.getPlatformViewsController().onResume();
    } else {
      Log.w(TAG, "onPostResume() invoked before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Refreshes Android's window system UI (AKA system chrome) to match Flutter's desired system
   * chrome style.
   */
  void updateSystemUiOverlays() {
    if (platformPlugin != null) {
      // TODO(mattcarroll): find a better way to handle the update of UI overlays than calling
      // through to platformPlugin. We're implicitly entangling the Window, Activity,
      // Fragment, and engine all with this one call.
      platformPlugin.updateSystemUiOverlays();
    }
  }

  /**
   * Invoke this from {@code Activity#onPause()} or {@code Fragment#onPause()}.
   *
   * <p>This method notifies the running Flutter app that it is "inactive" as per the Flutter app
   * lifecycle.
   */
  void onPause() {
    Log.v(TAG, "onPause()");
    ensureAlive();
    if (host.shouldDispatchAppLifecycleState() && flutterEngine != null) {
      flutterEngine.getLifecycleChannel().appIsInactive();
    }
  }

  /**
   * Invoke this from {@code Activity#onStop()} or {@code Fragment#onStop()}.
   *
   * <p>This method:
   *
   * <p>
   *
   * <ol>
   *   <li>This method notifies the running Flutter app that it is "paused" as per the Flutter app
   *       lifecycle.
   *   <li>Detaches this delegate's {@link io.flutter.embedding.engine.FlutterEngine} from this
   *       delegate's {@link FlutterView}.
   * </ol>
   */
  void onStop() {
    Log.v(TAG, "onStop()");
    ensureAlive();

    if (host.shouldDispatchAppLifecycleState() && flutterEngine != null) {
      flutterEngine.getLifecycleChannel().appIsPaused();
    }

    // This is a workaround for a bug on some OnePlus phones. The visibility of the application
    // window is still true after locking the screen on some OnePlus phones, and shows a black
    // screen when unlocked. We can work around this by changing the visibility of FlutterView in
    // onStart and onStop.
    // See https://github.com/flutter/flutter/issues/93276
    previousVisibility = flutterView.getVisibility();
    flutterView.setVisibility(View.GONE);
    if (flutterEngine != null) {
      // When an Activity is stopped it won't have its onTrimMemory callback invoked. Normally,
      // this isn't a problem but because of a bug in some builds of Android 14 we must act as
      // if the onTrimMemory callback has been called.
      flutterEngine.getRenderer().onTrimMemory(ComponentCallbacks2.TRIM_MEMORY_BACKGROUND);
    }
  }

  /**
   * Invoke this from {@code Activity#onDestroy()} or {@code Fragment#onDestroyView()}.
   *
   * <p>This method removes this delegate's {@link FlutterView}'s {@link FlutterUiDisplayListener}.
   */
  void onDestroyView() {
    Log.v(TAG, "onDestroyView()");
    ensureAlive();

    if (activePreDrawListener != null) {
      flutterView.getViewTreeObserver().removeOnPreDrawListener(activePreDrawListener);
      activePreDrawListener = null;
    }

    // flutterView can be null in instances where a delegate.onDestroyView is called without
    // onCreateView being called. See https://github.com/flutter/engine/pull/41082 for more detail.
    if (flutterView != null) {
      flutterView.detachFromFlutterEngine();
      flutterView.removeOnFirstFrameRenderedListener(flutterUiDisplayListener);
    }
  }

  void onSaveInstanceState(@Nullable Bundle bundle) {
    Log.v(TAG, "onSaveInstanceState. Giving framework and plugins an opportunity to save state.");
    ensureAlive();

    if (host.shouldRestoreAndSaveState()) {
      bundle.putByteArray(
          FRAMEWORK_RESTORATION_BUNDLE_KEY,
          flutterEngine.getRestorationChannel().getRestorationData());
    }

    if (host.shouldAttachEngineToActivity()) {
      final Bundle plugins = new Bundle();
      flutterEngine.getActivityControlSurface().onSaveInstanceState(plugins);
      bundle.putBundle(PLUGINS_RESTORATION_BUNDLE_KEY, plugins);
    }

    // If using a cached engine, we need to save whether the framework or the system should handle
    // backs.
    if (host.getCachedEngineId() != null && !host.shouldDestroyEngineWithHost()) {
      bundle.putBoolean(ON_BACK_CALLBACK_ENABLED_KEY, host.getBackCallbackState());
    }
  }

  @Override
  public void detachFromFlutterEngine() {
    if (host.shouldDestroyEngineWithHost()) {
      // The host owns the engine and should never have its engine taken by another exclusive
      // activity.
      throw new AssertionError(
          "The internal FlutterEngine created by "
              + host
              + " has been attached to by another activity. To persist a FlutterEngine beyond the "
              + "ownership of this activity, explicitly create a FlutterEngine");
    }

    // Default, but customizable, behavior is for the host to call {@link #onDetach}
    // deterministically as to not mix more events during the lifecycle of the next exclusive
    // activity.
    host.detachFromFlutterEngine();
  }

  /**
   * Invoke this from {@code Activity#onDestroy()} or {@code Fragment#onDetach()}.
   *
   * <p>This method:
   *
   * <p>
   *
   * <ol>
   *   <li>Detaches this delegate's {@link io.flutter.embedding.engine.FlutterEngine} from its
   *       surrounding {@code Activity}, if it was previously attached.
   *   <li>Destroys this delegate's {@link PlatformPlugin}.
   *   <li>Destroys this delegate's {@link io.flutter.embedding.engine.FlutterEngine} if {@link
   *       Host#shouldDestroyEngineWithHost()} ()} returns true.
   * </ol>
   */
  void onDetach() {
    if (!isAttached) {
      // Already detached.
      return;
    }
    Log.v(TAG, "onDetach()");
    ensureAlive();

    // Give the host an opportunity to cleanup any references that were created in
    // configureFlutterEngine().
    host.cleanUpFlutterEngine(flutterEngine);

    if (host.shouldAttachEngineToActivity()) {
      // Notify plugins that they are no longer attached to an Activity.
      Log.v(TAG, "Detaching FlutterEngine from the Activity that owns this Fragment.");
      if (host.getActivity().isChangingConfigurations()) {
        flutterEngine.getActivityControlSurface().detachFromActivityForConfigChanges();
      } else {
        flutterEngine.getActivityControlSurface().detachFromActivity();
      }
    }

    // Null out the platformPlugin and sensitiveContentPlugin to avoid a possible
    // retain cycle between the plugin, this Fragment, and this Fragment's Activity.
    if (platformPlugin != null) {
      platformPlugin.destroy();
      platformPlugin = null;
    }
    if (sensitiveContentPlugin != null) {
      sensitiveContentPlugin.destroy();
      sensitiveContentPlugin = null;
    }

    if (host.shouldDispatchAppLifecycleState() && flutterEngine != null) {
      flutterEngine.getLifecycleChannel().appIsDetached();
    }

    // Destroy our FlutterEngine if we're not set to retain it.
    if (host.shouldDestroyEngineWithHost()) {
      flutterEngine.destroy();

      if (host.getCachedEngineId() != null) {
        FlutterEngineCache.getInstance().remove(host.getCachedEngineId());
      }

      flutterEngine = null;
    }

    isAttached = false;
  }

  /**
   * Invoke this from {@link android.app.Activity#onBackPressed()}.
   *
   * <p>A {@code Fragment} host must have its containing {@code Activity} forward this call so that
   * the {@code Fragment} can then invoke this method.
   *
   * <p>This method instructs Flutter's navigation system to "pop route".
   */
  void onBackPressed() {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(TAG, "Forwarding onBackPressed() to FlutterEngine.");
      flutterEngine.getNavigationChannel().popRoute();
    } else {
      Log.w(TAG, "Invoked onBackPressed() before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@link OnBackAnimationCallback#onBackStarted(BackEvent)}.
   *
   * <p>This method should be called when the back gesture is initiated. It should be invoked as
   * part of the implementation of {@link OnBackAnimationCallback}.
   *
   * <p>This method delegates the handling of the start of a back gesture to the Flutter framework,
   * which is responsible for the appropriate response, such as initiating animations or preparing
   * the UI for the back navigation process.
   *
   * @param backEvent The BackEvent object containing information about the touch.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  void startBackGesture(@NonNull BackEvent backEvent) {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(TAG, "Forwarding startBackGesture() to FlutterEngine.");
      flutterEngine.getBackGestureChannel().startBackGesture(backEvent);
    } else {
      Log.w(TAG, "Invoked startBackGesture() before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@link OnBackAnimationCallback#onBackProgressed(BackEvent)}.
   *
   * <p>This method should be called in response to progress in a back gesture, as part of the
   * implementation of {@link OnBackAnimationCallback}.
   *
   * <p>This method delegates to the Flutter framework to update UI elements or animations based on
   * the progression of the back gesture.
   *
   * @param backEvent An BackEvent object describing the progress event.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  void updateBackGestureProgress(@NonNull BackEvent backEvent) {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(TAG, "Forwarding updateBackGestureProgress() to FlutterEngine.");
      flutterEngine.getBackGestureChannel().updateBackGestureProgress(backEvent);
    } else {
      Log.w(
          TAG,
          "Invoked updateBackGestureProgress() before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@link OnBackAnimationCallback#onBackInvoked()}.
   *
   * <p>This method is called to signify the completion of a back gesture and commits the navigation
   * action initiated by the gesture. It should be invoked as the final step in handling a back
   * gesture.
   *
   * <p>This method indicates to the Flutter framework that it should proceed with the back
   * navigation, including finalizing animations and updating the UI to reflect the navigation
   * outcome.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  void commitBackGesture() {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(TAG, "Forwarding commitBackGesture() to FlutterEngine.");
      flutterEngine.getBackGestureChannel().commitBackGesture();
    } else {
      Log.w(TAG, "Invoked commitBackGesture() before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@link OnBackAnimationCallback#onBackCancelled()}.
   *
   * <p>This method should be called when a back gesture is cancelled or the back button is pressed.
   * It informs the Flutter framework about the cancellation.
   *
   * <p>This method enables the Flutter framework to rollback any UI changes or animations initiated
   * in response to the back gesture. This includes resetting UI elements to their state prior to
   * the gesture's start.
   */
  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  void cancelBackGesture() {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(TAG, "Forwarding cancelBackGesture() to FlutterEngine.");
      flutterEngine.getBackGestureChannel().cancelBackGesture();
    } else {
      Log.w(TAG, "Invoked cancelBackGesture() before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@link android.app.Activity#onRequestPermissionsResult(int, String[], int[])}
   * or {@code Fragment#onRequestPermissionsResult(int, String[], int[])}.
   *
   * <p>This method forwards to interested Flutter plugins.
   */
  void onRequestPermissionsResult(
      int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(
          TAG,
          "Forwarding onRequestPermissionsResult() to FlutterEngine:\n"
              + "requestCode: "
              + requestCode
              + "\n"
              + "permissions: "
              + Arrays.toString(permissions)
              + "\n"
              + "grantResults: "
              + Arrays.toString(grantResults));
      flutterEngine
          .getActivityControlSurface()
          .onRequestPermissionsResult(requestCode, permissions, grantResults);
    } else {
      Log.w(
          TAG,
          "onRequestPermissionResult() invoked before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@code Activity#onNewIntent(Intent)}.
   *
   * <p>A {@code Fragment} host must have its containing {@code Activity} forward this call so that
   * the {@code Fragment} can then invoke this method.
   *
   * <p>This method forwards to interested Flutter plugins.
   */
  void onNewIntent(@NonNull Intent intent) {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(
          TAG,
          "Forwarding onNewIntent() to FlutterEngine and sending pushRouteInformation message.");
      flutterEngine.getActivityControlSurface().onNewIntent(intent);
      String initialRoute = maybeGetInitialRouteFromIntent(intent);
      if (initialRoute != null && !initialRoute.isEmpty()) {
        flutterEngine.getNavigationChannel().pushRouteInformation(initialRoute);
      }
    } else {
      Log.w(TAG, "onNewIntent() invoked before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@code Activity#onActivityResult(int, int, Intent)} or {@code
   * Fragment#onActivityResult(int, int, Intent)}.
   *
   * <p>This method forwards to interested Flutter plugins.
   */
  void onActivityResult(int requestCode, int resultCode, Intent data) {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(
          TAG,
          "Forwarding onActivityResult() to FlutterEngine:\n"
              + "requestCode: "
              + requestCode
              + "\n"
              + "resultCode: "
              + resultCode
              + "\n"
              + "data: "
              + data);
      flutterEngine.getActivityControlSurface().onActivityResult(requestCode, resultCode, data);
    } else {
      Log.w(TAG, "onActivityResult() invoked before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@code Activity#onUserLeaveHint()}.
   *
   * <p>A {@code Fragment} host must have its containing {@code Activity} forward this call so that
   * the {@code Fragment} can then invoke this method.
   *
   * <p>This method forwards to interested Flutter plugins.
   */
  void onUserLeaveHint() {
    ensureAlive();
    if (flutterEngine != null) {
      Log.v(TAG, "Forwarding onUserLeaveHint() to FlutterEngine.");
      flutterEngine.getActivityControlSurface().onUserLeaveHint();
    } else {
      Log.w(TAG, "onUserLeaveHint() invoked before FlutterFragment was attached to an Activity.");
    }
  }

  /**
   * Invoke this from {@code Activity#onWindowFocusChanged()}.
   *
   * <p>A {@code Fragment} host must have its containing {@code Activity} forward this call so that
   * the {@code Fragment} can then invoke this method.
   */
  void onWindowFocusChanged(boolean hasFocus) {
    ensureAlive();
    Log.v(TAG, "Received onWindowFocusChanged: " + (hasFocus ? "true" : "false"));
    if (host.shouldDispatchAppLifecycleState() && flutterEngine != null) {
      // TODO(gspencergoog): Once we have support for multiple windows/views,
      // this code will need to consult the list of windows/views to determine if
      // any windows in the app are focused and call the appropriate function.
      if (hasFocus) {
        flutterEngine.getLifecycleChannel().aWindowIsFocused();
      } else {
        flutterEngine.getLifecycleChannel().noWindowsAreFocused();
      }
    }
  }

  /**
   * Invoke this from {@link android.app.Activity#onTrimMemory(int)}.
   *
   * <p>A {@code Fragment} host must have its containing {@code Activity} forward this call so that
   * the {@code Fragment} can then invoke this method.
   *
   * <p>This method sends a "memory pressure warning" message to Flutter over the "system channel".
   */
  void onTrimMemory(int level) {
    ensureAlive();
    if (flutterEngine != null) {
      // Use a trim level delivered while the application is running so the
      // framework has a chance to react to the notification.
      // Avoid being too aggressive before the first frame is rendered. If it is
      // not at least running critical, we should avoid delaying the frame for
      // an overly aggressive GC.
      boolean trim = isFirstFrameRendered && level >= TRIM_MEMORY_RUNNING_LOW;
      if (trim) {
        flutterEngine.getDartExecutor().notifyLowMemoryWarning();
        flutterEngine.getSystemChannel().sendMemoryPressureWarning();
      }
      flutterEngine.getRenderer().onTrimMemory(level);
      flutterEngine.getPlatformViewsController().onTrimMemory(level);
    }
  }

  /**
   * Ensures that this delegate has not been {@link #release()}'ed.
   *
   * <p>An {@code IllegalStateException} is thrown if this delegate has been {@link #release()}'ed.
   */
  private void ensureAlive() {
    if (host == null) {
      throw new IllegalStateException(
          "Cannot execute method on a destroyed FlutterActivityAndFragmentDelegate.");
    }
  }

  /**
   * The {@link FlutterActivity} or {@link FlutterFragment} that owns this {@code
   * FlutterActivityAndFragmentDelegate}.
   */
  /* package */ interface Host
      extends FlutterEngineProvider,
          FlutterEngineConfigurator,
          PlatformPlugin.PlatformPluginDelegate {
    /**
     * Returns the {@link Context} that backs the host {@link android.app.Activity} or {@code
     * Fragment}.
     */
    @NonNull
    Context getContext();

    /** Returns true if the delegate should retrieve the initial route from the {@link Intent}. */
    @Nullable
    boolean shouldHandleDeeplinking();

    /**
     * Returns the host {@link android.app.Activity} or the {@code Activity} that is currently
     * attached to the host {@code Fragment}.
     */
    @Nullable
    Activity getActivity();

    /**
     * Returns the {@link Lifecycle} that backs the host {@link android.app.Activity} or {@code
     * Fragment}.
     */
    @NonNull
    Lifecycle getLifecycle();

    /** Returns the {@link FlutterShellArgs} that should be used when initializing Flutter. */
    @NonNull
    FlutterShellArgs getFlutterShellArgs();

    /**
     * Returns the ID of a statically cached {@link io.flutter.embedding.engine.FlutterEngine} to
     * use within this delegate's host, or {@code null} if this delegate's host does not want to use
     * a cached {@link FlutterEngine}.
     */
    @Nullable
    String getCachedEngineId();

    @Nullable
    String getCachedEngineGroupId();

    /**
     * Returns true if the {@link io.flutter.embedding.engine.FlutterEngine} used in this delegate
     * should be destroyed when the host/delegate are destroyed.
     *
     * <p>The default value is {@code true} in cases where {@code FlutterFragment} created its own
     * {@link io.flutter.embedding.engine.FlutterEngine}, and {@code false} in cases where a cached
     * {@link io.flutter.embedding.engine.FlutterEngine} was provided.
     */
    boolean shouldDestroyEngineWithHost();

    /**
     * Callback called when the {@link io.flutter.embedding.engine.FlutterEngine} has been attached
     * to by another activity before this activity was destroyed.
     *
     * <p>The expected behavior is for this activity to synchronously stop using the {@link
     * FlutterEngine} to avoid lifecycle crosstalk with the new activity.
     */
    void detachFromFlutterEngine();

    /**
     * Returns the Dart entrypoint that should run when a new {@link
     * io.flutter.embedding.engine.FlutterEngine} is created.
     */
    @NonNull
    String getDartEntrypointFunctionName();

    /**
     * Returns the URI of the Dart library which contains the entrypoint method (example
     * "package:foo_package/main.dart"). If null, this will default to the same library as the
     * `main()` function in the Dart program.
     */
    @Nullable
    String getDartEntrypointLibraryUri();

    /** Returns arguments that passed as a list of string to Dart's entrypoint function. */
    @Nullable
    List<String> getDartEntrypointArgs();

    /** Returns the path to the app bundle where the Dart code exists. */
    @NonNull
    String getAppBundlePath();

    /** Returns the initial route that Flutter renders. */
    @Nullable
    String getInitialRoute();

    /**
     * Returns the {@link RenderMode} used by the {@link FlutterView} that displays the {@link
     * FlutterEngine}'s content.
     */
    @NonNull
    RenderMode getRenderMode();

    /**
     * Returns the {@link TransparencyMode} used by the {@link FlutterView} that displays the {@link
     * FlutterEngine}'s content.
     */
    @NonNull
    TransparencyMode getTransparencyMode();

    /**
     * Returns the {@link ExclusiveAppComponent<Activity>} that is associated with {@link
     * io.flutter.embedding.engine.FlutterEngine}.
     *
     * <p>In the scenario where multiple {@link FlutterActivity} or {@link FlutterFragment} share
     * the same {@link FlutterEngine}, to attach/re-attache a {@link FlutterActivity} or {@link
     * FlutterFragment} to the shared {@link FlutterEngine}, we MUST manually invoke {@link
     * ActivityControlSurface#attachToActivity(ExclusiveAppComponent, Lifecycle)}.
     *
     * <p>The {@link ExclusiveAppComponent} is exposed here so that subclasses of {@link
     * FlutterActivity} or {@link FlutterFragment} can access it.
     */
    ExclusiveAppComponent<Activity> getExclusiveAppComponent();

    /**
     * Returns the {@link io.flutter.embedding.engine.FlutterEngine} that should be rendered to a
     * {@link FlutterView}.
     *
     * <p>If {@code null} is returned, a new {@link io.flutter.embedding.engine.FlutterEngine} will
     * be created automatically.
     */
    @Nullable
    FlutterEngine provideFlutterEngine(@NonNull Context context);

    /**
     * Hook for the host to create/provide a {@link PlatformPlugin} if the associated Flutter
     * experience should control system chrome.
     */
    @Nullable
    PlatformPlugin providePlatformPlugin(
        @Nullable Activity activity, @NonNull FlutterEngine flutterEngine);

    /**
     * Hook for host to create/provide a {@link SensitiveContentPlugin} if the associated Flutter
     * experience should set content sensitivity.
     */
    @Nullable
    SensitiveContentPlugin provideSensitiveContentPlugin(
        @Nullable Activity activity, @NonNull FlutterEngine flutterEngine);

    /**
     * Hook for the host to configure the {@link io.flutter.embedding.engine.FlutterEngine} as
     * desired.
     */
    void configureFlutterEngine(@NonNull FlutterEngine flutterEngine);

    /**
     * Hook for the host to cleanup references that were established in {@link
     * #configureFlutterEngine(FlutterEngine)} before the host is destroyed or detached.
     */
    void cleanUpFlutterEngine(@NonNull FlutterEngine flutterEngine);

    /**
     * Returns true if the {@link io.flutter.embedding.engine.FlutterEngine}'s plugin system should
     * be connected to the host {@link android.app.Activity}, allowing plugins to interact with it.
     */
    boolean shouldAttachEngineToActivity();

    /**
     * Invoked by this delegate when the {@link FlutterSurfaceView} that renders the Flutter UI is
     * initially instantiated.
     *
     * <p>This method is only invoked if the {@link
     * io.flutter.embedding.android.FlutterView.RenderMode} is set to {@link
     * io.flutter.embedding.android.FlutterView.RenderMode#surface}. Otherwise, {@link
     * #onFlutterTextureViewCreated(FlutterTextureView)} is invoked.
     *
     * <p>This method is invoked before the given {@link FlutterSurfaceView} is attached to the
     * {@code View} hierarchy. Implementers should not attempt to climb the {@code View} hierarchy
     * or make assumptions about relationships with other {@code View}s.
     */
    void onFlutterSurfaceViewCreated(@NonNull FlutterSurfaceView flutterSurfaceView);

    /**
     * Invoked by this delegate when the {@link FlutterTextureView} that renders the Flutter UI is
     * initially instantiated.
     *
     * <p>This method is only invoked if the {@link
     * io.flutter.embedding.android.FlutterView.RenderMode} is set to {@link
     * io.flutter.embedding.android.FlutterView.RenderMode#texture}. Otherwise, {@link
     * #onFlutterSurfaceViewCreated(FlutterSurfaceView)} is invoked.
     *
     * <p>This method is invoked before the given {@link FlutterTextureView} is attached to the
     * {@code View} hierarchy. Implementers should not attempt to climb the {@code View} hierarchy
     * or make assumptions about relationships with other {@code View}s.
     */
    void onFlutterTextureViewCreated(@NonNull FlutterTextureView flutterTextureView);

    /** Invoked by this delegate when its {@link FlutterView} starts painting pixels. */
    void onFlutterUiDisplayed();

    /** Invoked by this delegate when its {@link FlutterView} stops painting pixels. */
    void onFlutterUiNoLongerDisplayed();

    /**
     * Whether state restoration is enabled.
     *
     * <p>When this returns true, the instance state provided to {@code
     * onRestoreInstanceState(Bundle)} will be forwarded to the framework via the {@code
     * RestorationChannel} and during {@code onSaveInstanceState(Bundle)} the current framework
     * instance state obtained from {@code RestorationChannel} will be stored in the provided
     * bundle.
     *
     * <p>This defaults to true, unless a cached engine is used.
     */
    boolean shouldRestoreAndSaveState();

    /**
     * Refreshes Android's window system UI (AKA system chrome) to match Flutter's desired system
     * chrome style.
     *
     * <p>This is useful when using the splash screen API available in Android 12. {@code
     * SplashScreenView#remove} resets the system UI colors to the values set prior to the execution
     * of the Dart entrypoint. As a result, the values set from Dart are reverted by this API. To
     * workaround this issue, call this method after removing the splash screen with {@code
     * SplashScreenView#remove}.
     */
    void updateSystemUiOverlays();

    /**
     * Give the host application a chance to take control of the app lifecycle events to avoid
     * lifecycle crosstalk.
     *
     * <p>In the add-to-app scenario where multiple {@link FlutterActivity} shares the same {@link
     * FlutterEngine}, the application lifecycle state will have crosstalk causing the page to
     * freeze. For example, we open a new page called FlutterActivity#2 from the previous page
     * called FlutterActivity#1. The flow of app lifecycle states received by dart is as follows:
     *
     * <p>inactive (from FlutterActivity#1) -> resumed (from FlutterActivity#2) -> paused (from
     * FlutterActivity#1)
     *
     * <p>On the one hand, the {@code paused} state from FlutterActivity#1 will cause the
     * FlutterActivity#2 page to be stuck; On the other hand, these states are not expected from the
     * perspective of the entire application lifecycle. If the host application gets the control of
     * sending {@link AppLifecycleState}, It will be possible to correctly match the {@link
     * AppLifecycleState} with the application-level lifecycle.
     *
     * <p>Return {@code false} means the host application dispatches these app lifecycle events,
     * while return {@code true} means the engine dispatches these events.
     */
    boolean shouldDispatchAppLifecycleState();

    /**
     * Whether to automatically attach the {@link FlutterView} to the engine.
     *
     * <p>In the add-to-app scenario where multiple {@link FlutterView} share the same {@link
     * FlutterEngine}, the host application desires to determine the timing of attaching the {@link
     * FlutterView} to the engine, for example, during the {@code onResume} instead of the {@code
     * onCreateView}.
     *
     * <p>Defaults to {@code true}.
     */
    boolean attachToEngineAutomatically();

    boolean getBackCallbackState();
  }
}
