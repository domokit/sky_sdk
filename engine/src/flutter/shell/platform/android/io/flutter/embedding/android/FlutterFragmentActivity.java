// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.DART_ENTRYPOINT_META_DATA_KEY;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.DART_ENTRYPOINT_URI_META_DATA_KEY;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.DEFAULT_BACKGROUND_MODE;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.DEFAULT_DART_ENTRYPOINT;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.DEFAULT_INITIAL_ROUTE;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_BACKGROUND_MODE;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_CACHED_ENGINE_GROUP_ID;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_CACHED_ENGINE_ID;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_DART_ENTRYPOINT;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_DART_ENTRYPOINT_ARGS;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_DESTROY_ENGINE_WITH_ACTIVITY;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_INITIAL_ROUTE;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.INITIAL_ROUTE_META_DATA_KEY;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.NORMAL_THEME_META_DATA_KEY;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.deepLinkEnabled;

import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.os.Bundle;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.widget.FrameLayout;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import androidx.fragment.app.FragmentActivity;
import androidx.fragment.app.FragmentManager;
import io.flutter.Log;
import io.flutter.embedding.android.FlutterActivityLaunchConfigs.BackgroundMode;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterShellArgs;
import io.flutter.embedding.engine.plugins.util.GeneratedPluginRegister;
import io.flutter.plugin.platform.PlatformPlugin;
import java.util.ArrayList;
import java.util.List;

/**
 * A Flutter {@code Activity} that is based upon {@link FragmentActivity}.
 *
 * <p>{@code FlutterFragmentActivity} exists because there are some Android APIs in the ecosystem
 * that only accept a {@link FragmentActivity}. If a {@link FragmentActivity} is not required, you
 * should consider using a regular {@link FlutterActivity} instead, because {@link FlutterActivity}
 * is considered to be the standard, canonical implementation of a Flutter {@code Activity}.
 */
// A number of methods in this class have the same implementation as FlutterActivity. These methods
// are duplicated for readability purposes. Be sure to replicate any change in this class in
// FlutterActivity, too.
public class FlutterFragmentActivity extends FragmentActivity
    implements FlutterEngineProvider, FlutterEngineConfigurator {
  private static final String TAG = "FlutterFragmentActivity";

  // FlutterFragment management.
  private static final String TAG_FLUTTER_FRAGMENT = "flutter_fragment";
  // TODO(mattcarroll): replace ID with R.id when build system supports R.java
  public static final int FRAGMENT_CONTAINER_ID = View.generateViewId();

  /**
   * Creates an {@link Intent} that launches a {@code FlutterFragmentActivity}, which executes a
   * {@code main()} Dart entrypoint, and displays the "/" route as Flutter's initial route.
   */
  @NonNull
  public static Intent createDefaultIntent(@NonNull Context launchContext) {
    return withNewEngine().build(launchContext);
  }

  /**
   * Creates an {@link FlutterFragmentActivity.NewEngineIntentBuilder}, which can be used to
   * configure an {@link Intent} to launch a {@code FlutterFragmentActivity} that internally creates
   * a new {@link io.flutter.embedding.engine.FlutterEngine} using the desired Dart entrypoint,
   * initial route, etc.
   */
  @NonNull
  public static NewEngineIntentBuilder withNewEngine() {
    return new NewEngineIntentBuilder(FlutterFragmentActivity.class);
  }

  /**
   * Builder to create an {@code Intent} that launches a {@code FlutterFragmentActivity} with a new
   * {@link io.flutter.embedding.engine.FlutterEngine} and the desired configuration.
   */
  public static class NewEngineIntentBuilder {
    private final Class<? extends FlutterFragmentActivity> activityClass;
    private String initialRoute = DEFAULT_INITIAL_ROUTE;
    private String backgroundMode = DEFAULT_BACKGROUND_MODE;
    @Nullable private List<String> dartEntrypointArgs;

    /**
     * Constructor that allows this {@code NewEngineIntentBuilder} to be used by subclasses of
     * {@code FlutterFragmentActivity}.
     *
     * <p>Subclasses of {@code FlutterFragmentActivity} should provide their own static version of
     * {@link #withNewEngine()}, which returns an instance of {@code NewEngineIntentBuilder}
     * constructed with a {@code Class} reference to the {@code FlutterFragmentActivity} subclass,
     * e.g.:
     *
     * <p>{@code return new NewEngineIntentBuilder(MyFlutterActivity.class); }
     */
    public NewEngineIntentBuilder(@NonNull Class<? extends FlutterFragmentActivity> activityClass) {
      this.activityClass = activityClass;
    }

    /**
     * The initial route that a Flutter app will render in this {@code FlutterFragmentActivity},
     * defaults to "/".
     */
    @NonNull
    public NewEngineIntentBuilder initialRoute(@NonNull String initialRoute) {
      this.initialRoute = initialRoute;
      return this;
    }

    /**
     * The mode of {@code FlutterFragmentActivity}'s background, either {@link
     * BackgroundMode#opaque} or {@link BackgroundMode#transparent}.
     *
     * <p>The default background mode is {@link BackgroundMode#opaque}.
     *
     * <p>Choosing a background mode of {@link BackgroundMode#transparent} will configure the inner
     * {@link FlutterView} of this {@code FlutterFragmentActivity} to be configured with a {@link
     * FlutterTextureView} to support transparency. This choice has a non-trivial performance
     * impact. A transparent background should only be used if it is necessary for the app design
     * being implemented.
     *
     * <p>A {@code FlutterFragmentActivity} that is configured with a background mode of {@link
     * BackgroundMode#transparent} must have a theme applied to it that includes the following
     * property: {@code <item name="android:windowIsTranslucent">true</item>}.
     */
    @NonNull
    public NewEngineIntentBuilder backgroundMode(@NonNull BackgroundMode backgroundMode) {
      this.backgroundMode = backgroundMode.name();
      return this;
    }

    /**
     * The Dart entrypoint arguments will be passed as a list of string to Dart's entrypoint
     * function.
     *
     * <p>A value of null means do not pass any arguments to Dart's entrypoint function.
     *
     * @param dartEntrypointArgs The Dart entrypoint arguments.
     * @return The engine intent builder.
     */
    @NonNull
    public NewEngineIntentBuilder dartEntrypointArgs(@Nullable List<String> dartEntrypointArgs) {
      this.dartEntrypointArgs = dartEntrypointArgs;
      return this;
    }

    /**
     * Creates and returns an {@link Intent} that will launch a {@code FlutterFragmentActivity} with
     * the desired configuration.
     */
    @NonNull
    public Intent build(@NonNull Context context) {
      Intent intent =
          new Intent(context, activityClass)
              .putExtra(EXTRA_INITIAL_ROUTE, initialRoute)
              .putExtra(EXTRA_BACKGROUND_MODE, backgroundMode)
              .putExtra(EXTRA_DESTROY_ENGINE_WITH_ACTIVITY, true);
      if (dartEntrypointArgs != null) {
        intent.putExtra(EXTRA_DART_ENTRYPOINT_ARGS, new ArrayList(dartEntrypointArgs));
      }
      return intent;
    }
  }

  /**
   * Creates a {@link CachedEngineIntentBuilder}, which can be used to configure an {@link Intent}
   * to launch a {@code FlutterFragmentActivity} that internally uses an existing {@link
   * FlutterEngine} that is cached in {@link io.flutter.embedding.engine.FlutterEngineCache}.
   */
  @NonNull
  public static CachedEngineIntentBuilder withCachedEngine(@NonNull String cachedEngineId) {
    return new CachedEngineIntentBuilder(FlutterFragmentActivity.class, cachedEngineId);
  }

  /**
   * Builder to create an {@code Intent} that launches a {@code FlutterFragmentActivity} with an
   * existing {@link io.flutter.embedding.engine.FlutterEngine} that is cached in {@link
   * io.flutter.embedding.engine.FlutterEngineCache}.
   */
  public static class CachedEngineIntentBuilder {
    private final Class<? extends FlutterFragmentActivity> activityClass;
    private final String cachedEngineId;
    private boolean destroyEngineWithActivity = false;
    private String backgroundMode = DEFAULT_BACKGROUND_MODE;

    /**
     * Constructor that allows this {@code CachedEngineIntentBuilder} to be used by subclasses of
     * {@code FlutterFragmentActivity}.
     *
     * <p>Subclasses of {@code FlutterFragmentActivity} should provide their own static version of
     * {@link #withCachedEngine(String)}, which returns an instance of {@code
     * CachedEngineIntentBuilder} constructed with a {@code Class} reference to the {@code
     * FlutterFragmentActivity} subclass, e.g.:
     *
     * <p>{@code return new CachedEngineIntentBuilder(MyFlutterActivity.class, engineId); }
     */
    public CachedEngineIntentBuilder(
        @NonNull Class<? extends FlutterFragmentActivity> activityClass, @NonNull String engineId) {
      this.activityClass = activityClass;
      this.cachedEngineId = engineId;
    }

    /**
     * Returns true if the cached {@link io.flutter.embedding.engine.FlutterEngine} should be
     * destroyed and removed from the cache when this {@code FlutterFragmentActivity} is destroyed.
     *
     * <p>The default value is {@code false}.
     */
    public CachedEngineIntentBuilder destroyEngineWithActivity(boolean destroyEngineWithActivity) {
      this.destroyEngineWithActivity = destroyEngineWithActivity;
      return this;
    }

    /**
     * The mode of {@code FlutterFragmentActivity}'s background, either {@link
     * BackgroundMode#opaque} or {@link BackgroundMode#transparent}.
     *
     * <p>The default background mode is {@link BackgroundMode#opaque}.
     *
     * <p>Choosing a background mode of {@link BackgroundMode#transparent} will configure the inner
     * {@link FlutterView} of this {@code FlutterFragmentActivity} to be configured with a {@link
     * FlutterTextureView} to support transparency. This choice has a non-trivial performance
     * impact. A transparent background should only be used if it is necessary for the app design
     * being implemented.
     *
     * <p>A {@code FlutterFragmentActivity} that is configured with a background mode of {@link
     * BackgroundMode#transparent} must have a theme applied to it that includes the following
     * property: {@code <item name="android:windowIsTranslucent">true</item>}.
     */
    @NonNull
    public CachedEngineIntentBuilder backgroundMode(@NonNull BackgroundMode backgroundMode) {
      this.backgroundMode = backgroundMode.name();
      return this;
    }

    /**
     * Creates and returns an {@link Intent} that will launch a {@code FlutterFragmentActivity} with
     * the desired configuration.
     */
    @NonNull
    public Intent build(@NonNull Context context) {
      return new Intent(context, activityClass)
          .putExtra(EXTRA_CACHED_ENGINE_ID, cachedEngineId)
          .putExtra(EXTRA_DESTROY_ENGINE_WITH_ACTIVITY, destroyEngineWithActivity)
          .putExtra(EXTRA_BACKGROUND_MODE, backgroundMode);
    }
  }

  /**
   * Creates a {@link NewEngineInGroupIntentBuilder}, which can be used to configure an {@link
   * Intent} to launch a {@code FlutterFragmentActivity} that internally uses an existing {@link
   * io.flutter.embedding.engine.FlutterEngineGroup} that is cached in {@link
   * io.flutter.embedding.engine.FlutterEngineGroupCache}.
   *
   * @param engineGroupId A cached engine group ID.
   * @return The builder.
   */
  public static NewEngineInGroupIntentBuilder withNewEngineInGroup(@NonNull String engineGroupId) {
    return new NewEngineInGroupIntentBuilder(FlutterFragmentActivity.class, engineGroupId);
  }

  /**
   * Builder to create an {@code Intent} that launches a {@code FlutterFragmentActivity} with a new
   * {@link FlutterEngine} by FlutterEngineGroup#createAndRunEngine.
   */
  public static class NewEngineInGroupIntentBuilder {
    private final Class<? extends FlutterFragmentActivity> activityClass;
    private final String cachedEngineGroupId;
    private String dartEntrypoint = DEFAULT_DART_ENTRYPOINT;
    private String initialRoute = DEFAULT_INITIAL_ROUTE;
    private String backgroundMode = DEFAULT_BACKGROUND_MODE;

    /**
     * Constructor that allows this {@code NewEngineInGroupIntentBuilder} to be used by subclasses
     * of {@code FlutterActivity}.
     *
     * <p>Subclasses of {@code FlutterFragmentActivity} should provide their own static version of
     * {@link #withNewEngineInGroup}, which returns an instance of {@code
     * NewEngineInGroupIntentBuilder} constructed with a {@code Class} reference to the {@code
     * FlutterFragmentActivity} subclass, e.g.:
     *
     * <p>{@code return new NewEngineInGroupIntentBuilder(FlutterFragmentActivity.class,
     * cacheedEngineGroupId); }
     *
     * @param activityClass A subclass of {@code FlutterFragmentActivity}.
     * @param engineGroupId The engine group id.
     */
    public NewEngineInGroupIntentBuilder(
        @NonNull Class<? extends FlutterFragmentActivity> activityClass,
        @NonNull String engineGroupId) {
      this.activityClass = activityClass;
      this.cachedEngineGroupId = engineGroupId;
    }

    /**
     * The Dart entrypoint that will be executed as soon as the Dart snapshot is loaded, default to
     * "main".
     *
     * @param dartEntrypoint The dart entrypoint's name
     * @return The engine group intent builder
     */
    @NonNull
    public NewEngineInGroupIntentBuilder dartEntrypoint(@NonNull String dartEntrypoint) {
      this.dartEntrypoint = dartEntrypoint;
      return this;
    }

    /**
     * The initial route that a Flutter app will render in this {@code FlutterFragmentActivity},
     * defaults to "/".
     */
    @NonNull
    public NewEngineInGroupIntentBuilder initialRoute(@NonNull String initialRoute) {
      this.initialRoute = initialRoute;
      return this;
    }

    /**
     * The mode of {@code FlutterFragmentActivity}'s background, either {@link
     * BackgroundMode#opaque} or {@link BackgroundMode#transparent}.
     *
     * <p>The default background mode is {@link BackgroundMode#opaque}.
     *
     * <p>Choosing a background mode of {@link BackgroundMode#transparent} will configure the inner
     * {@link FlutterView} of this {@code FlutterFragmentActivity} to be configured with a {@link
     * FlutterTextureView} to support transparency. This choice has a non-trivial performance
     * impact. A transparent background should only be used if it is necessary for the app design
     * being implemented.
     *
     * <p>A {@code FlutterFragmentActivity} that is configured with a background mode of {@link
     * BackgroundMode#transparent} must have a theme applied to it that includes the following
     * property: {@code <item name="android:windowIsTranslucent">true</item>}.
     */
    @NonNull
    public NewEngineInGroupIntentBuilder backgroundMode(@NonNull BackgroundMode backgroundMode) {
      this.backgroundMode = backgroundMode.name();
      return this;
    }

    /**
     * Creates and returns an {@link Intent} that will launch a {@code FlutterFragmentActivity} with
     * the desired configuration.
     */
    @NonNull
    public Intent build(@NonNull Context context) {
      return new Intent(context, activityClass)
          .putExtra(EXTRA_DART_ENTRYPOINT, dartEntrypoint)
          .putExtra(EXTRA_INITIAL_ROUTE, initialRoute)
          .putExtra(EXTRA_CACHED_ENGINE_GROUP_ID, cachedEngineGroupId)
          .putExtra(EXTRA_BACKGROUND_MODE, backgroundMode)
          .putExtra(EXTRA_DESTROY_ENGINE_WITH_ACTIVITY, true);
    }
  }

  @Nullable private FlutterFragment flutterFragment;

  @Override
  protected void onCreate(@Nullable Bundle savedInstanceState) {
    switchLaunchThemeForNormalTheme();
    // Get an existing fragment reference first before onCreate since onCreate would re-attach
    // existing fragments. This would cause FlutterFragment to reference the host activity which
    // should be aware of its child fragment.
    flutterFragment = retrieveExistingFlutterFragmentIfPossible();

    super.onCreate(savedInstanceState);

    configureWindowForTransparency();
    setContentView(createFragmentContainer());
    configureStatusBarForFullscreenFlutterExperience();
    ensureFlutterFragmentCreated();
  }

  /**
   * Switches themes for this {@code Activity} from the theme used to launch this {@code Activity}
   * to a "normal theme" that is intended for regular {@code Activity} operation.
   *
   * <p>This behavior is offered so that a "launch screen" can be displayed while the application
   * initially loads. To utilize this behavior in an app, do the following:
   *
   * <ol>
   *   <li>Create 2 different themes in style.xml: one theme for the launch screen and one theme for
   *       normal display.
   *   <li>In the launch screen theme, set the "windowBackground" property to a {@code Drawable} of
   *       your choice.
   *   <li>In the normal theme, customize however you'd like.
   *   <li>In the AndroidManifest.xml, set the theme of your {@code FlutterFragmentActivity} to your
   *       launch theme.
   *   <li>Add a {@code <meta-data>} property to your {@code FlutterFragmentActivity} with a name of
   *       "io.flutter.embedding.android.NormalTheme" and set the resource to your normal theme,
   *       e.g., {@code android:resource="@style/MyNormalTheme}.
   * </ol>
   *
   * With the above settings, your launch theme will be used when loading the app, and then the
   * theme will be switched to your normal theme once the app has initialized.
   *
   * <p>Do not change aspects of system chrome between a launch theme and normal theme. Either
   * define both themes to be fullscreen or not, and define both themes to display the same status
   * bar and navigation bar settings. If you wish to adjust system chrome once your Flutter app
   * renders, use platform channels to instruct Android to do so at the appropriate time. This will
   * avoid any jarring visual changes during app startup.
   */
  private void switchLaunchThemeForNormalTheme() {
    try {
      Bundle metaData = getMetaData();
      if (metaData != null) {
        int normalThemeRID = metaData.getInt(NORMAL_THEME_META_DATA_KEY, -1);
        if (normalThemeRID != -1) {
          setTheme(normalThemeRID);
        }
      } else {
        Log.v(TAG, "Using the launch theme as normal theme.");
      }
    } catch (PackageManager.NameNotFoundException exception) {
      Log.e(
          TAG,
          "Could not read meta-data for FlutterFragmentActivity. Using the launch theme as normal theme.");
    }
  }

  /**
   * Sets this {@code Activity}'s {@code Window} background to be transparent, and hides the status
   * bar, if this {@code Activity}'s desired {@link BackgroundMode} is {@link
   * BackgroundMode#transparent}.
   *
   * <p>For {@code Activity} transparency to work as expected, the theme applied to this {@code
   * Activity} must include {@code <item name="android:windowIsTranslucent">true</item>}.
   */
  private void configureWindowForTransparency() {
    BackgroundMode backgroundMode = getBackgroundMode();
    if (backgroundMode == BackgroundMode.transparent) {
      getWindow().setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));
    }
  }

  /**
   * Creates a {@link FrameLayout} with an ID of {@code #FRAGMENT_CONTAINER_ID} that will contain
   * the {@link FlutterFragment} displayed by this {@code FlutterFragmentActivity}.
   *
   * <p>
   *
   * @return the FrameLayout container
   */
  @NonNull
  private View createFragmentContainer() {
    FrameLayout container = provideRootLayout(this);
    container.setId(FRAGMENT_CONTAINER_ID);
    container.setLayoutParams(
        new ViewGroup.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
    return container;
  }

  /**
   * Retrieves the previously created {@link FlutterFragment} if possible.
   *
   * <p>If the activity is recreated, an existing {@link FlutterFragment} may already exist. Retain
   * a reference to that {@link FlutterFragment} in the {@code #flutterFragment} field and avoid
   * re-creating another {@link FlutterFragment}.
   */
  @VisibleForTesting
  FlutterFragment retrieveExistingFlutterFragmentIfPossible() {
    FragmentManager fragmentManager = getSupportFragmentManager();
    return (FlutterFragment) fragmentManager.findFragmentByTag(TAG_FLUTTER_FRAGMENT);
  }

  /**
   * Ensure that a {@link FlutterFragment} is attached to this {@code FlutterFragmentActivity}.
   *
   * <p>If no {@link FlutterFragment} exists in this {@code FlutterFragmentActivity}, then a {@link
   * FlutterFragment} is created and added.
   */
  private void ensureFlutterFragmentCreated() {
    if (flutterFragment == null) {
      // If both activity and fragment have been destroyed, the activity restore may have
      // already recreated a new instance of the fragment again via the FragmentActivity.onCreate
      // and the FragmentManager.
      flutterFragment = retrieveExistingFlutterFragmentIfPossible();
    }
    if (flutterFragment == null) {
      // No FlutterFragment exists yet. This must be the initial Activity creation. We will create
      // and add a new FlutterFragment to this Activity.
      flutterFragment = createFlutterFragment();
      FragmentManager fragmentManager = getSupportFragmentManager();
      fragmentManager
          .beginTransaction()
          .add(FRAGMENT_CONTAINER_ID, flutterFragment, TAG_FLUTTER_FRAGMENT)
          .commit();
    }
  }

  /**
   * Creates the instance of the {@link FlutterFragment} that this {@code FlutterFragmentActivity}
   * displays.
   *
   * <p>Subclasses may override this method to return a specialization of {@link FlutterFragment}.
   */
  @NonNull
  protected FlutterFragment createFlutterFragment() {
    final BackgroundMode backgroundMode = getBackgroundMode();
    final RenderMode renderMode = getRenderMode();
    final TransparencyMode transparencyMode =
        backgroundMode == BackgroundMode.opaque
            ? TransparencyMode.opaque
            : TransparencyMode.transparent;
    final boolean shouldDelayFirstAndroidViewDraw = renderMode == RenderMode.surface;
    final boolean shouldAutomaticallyHandleOnBackPressed = true;

    if (getCachedEngineId() != null) {
      Log.v(
          TAG,
          "Creating FlutterFragment with cached engine:\n"
              + "Cached engine ID: "
              + getCachedEngineId()
              + "\n"
              + "Will destroy engine when Activity is destroyed: "
              + shouldDestroyEngineWithHost()
              + "\n"
              + "Background transparency mode: "
              + backgroundMode
              + "\n"
              + "Will attach FlutterEngine to Activity: "
              + shouldAttachEngineToActivity());

      return FlutterFragment.withCachedEngine(getCachedEngineId())
          .renderMode(renderMode)
          .transparencyMode(transparencyMode)
          .handleDeeplinking(shouldHandleDeeplinking())
          .shouldAttachEngineToActivity(shouldAttachEngineToActivity())
          .destroyEngineWithFragment(shouldDestroyEngineWithHost())
          .shouldDelayFirstAndroidViewDraw(shouldDelayFirstAndroidViewDraw)
          .shouldAutomaticallyHandleOnBackPressed(shouldAutomaticallyHandleOnBackPressed)
          .build();
    } else {
      Log.v(
          TAG,
          "Creating FlutterFragment with new engine:\n"
              + "Cached engine group ID: "
              + getCachedEngineGroupId()
              + "\n"
              + "Background transparency mode: "
              + backgroundMode
              + "\n"
              + "Dart entrypoint: "
              + getDartEntrypointFunctionName()
              + "\n"
              + "Dart entrypoint library uri: "
              + (getDartEntrypointLibraryUri() != null ? getDartEntrypointLibraryUri() : "\"\"")
              + "\n"
              + "Initial route: "
              + getInitialRoute()
              + "\n"
              + "App bundle path: "
              + getAppBundlePath()
              + "\n"
              + "Will attach FlutterEngine to Activity: "
              + shouldAttachEngineToActivity());

      if (getCachedEngineGroupId() != null) {
        return FlutterFragment.withNewEngineInGroup(getCachedEngineGroupId())
            .dartEntrypoint(getDartEntrypointFunctionName())
            .initialRoute(getInitialRoute())
            .handleDeeplinking(shouldHandleDeeplinking())
            .renderMode(renderMode)
            .transparencyMode(transparencyMode)
            .shouldAttachEngineToActivity(shouldAttachEngineToActivity())
            .shouldDelayFirstAndroidViewDraw(shouldDelayFirstAndroidViewDraw)
            .shouldAutomaticallyHandleOnBackPressed(shouldAutomaticallyHandleOnBackPressed)
            .build();
      }

      return FlutterFragment.withNewEngine()
          .dartEntrypoint(getDartEntrypointFunctionName())
          .dartLibraryUri(getDartEntrypointLibraryUri())
          .dartEntrypointArgs(getDartEntrypointArgs())
          .initialRoute(getInitialRoute())
          .appBundlePath(getAppBundlePath())
          .flutterShellArgs(FlutterShellArgs.fromIntent(getIntent()))
          .handleDeeplinking(shouldHandleDeeplinking())
          .renderMode(renderMode)
          .transparencyMode(transparencyMode)
          .shouldAttachEngineToActivity(shouldAttachEngineToActivity())
          .shouldDelayFirstAndroidViewDraw(shouldDelayFirstAndroidViewDraw)
          .shouldAutomaticallyHandleOnBackPressed(shouldAutomaticallyHandleOnBackPressed)
          .build();
    }
  }

  private void configureStatusBarForFullscreenFlutterExperience() {
    Window window = getWindow();
    window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);
    window.setStatusBarColor(0x40000000);
    window.getDecorView().setSystemUiVisibility(PlatformPlugin.DEFAULT_SYSTEM_UI);
  }

  @Override
  public void onPostResume() {
    super.onPostResume();
    flutterFragment.onPostResume();
  }

  @Override
  protected void onNewIntent(@NonNull Intent intent) {
    // Forward Intents to our FlutterFragment in case it cares.
    flutterFragment.onNewIntent(intent);
    super.onNewIntent(intent);
  }

  @Override
  public void onRequestPermissionsResult(
      int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
    super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    flutterFragment.onRequestPermissionsResult(requestCode, permissions, grantResults);
  }

  @Override
  @SuppressWarnings("MissingSuperCall")
  public void onUserLeaveHint() {
    flutterFragment.onUserLeaveHint();
  }

  @Override
  public void onTrimMemory(int level) {
    super.onTrimMemory(level);
    flutterFragment.onTrimMemory(level);
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    super.onActivityResult(requestCode, resultCode, data);
    flutterFragment.onActivityResult(requestCode, resultCode, data);
  }

  @SuppressWarnings("unused")
  @Nullable
  protected FlutterEngine getFlutterEngine() {
    return flutterFragment.getFlutterEngine();
  }

  /**
   * Returns false if the {@link io.flutter.embedding.engine.FlutterEngine} backing this {@code
   * FlutterFragmentActivity} should outlive this {@code FlutterFragmentActivity}, or true to be
   * destroyed when the {@code FlutterFragmentActivity} is destroyed.
   *
   * <p>The default value is {@code true} in cases where {@code FlutterFragmentActivity} created its
   * own {@link io.flutter.embedding.engine.FlutterEngine}, and {@code false} in cases where a
   * cached {@link io.flutter.embedding.engine.FlutterEngine} was provided.
   */
  public boolean shouldDestroyEngineWithHost() {
    return getIntent().getBooleanExtra(EXTRA_DESTROY_ENGINE_WITH_ACTIVITY, false);
  }

  /**
   * Hook for subclasses to control whether or not the {@link FlutterFragment} within this {@code
   * Activity} automatically attaches its {@link io.flutter.embedding.engine.FlutterEngine} to this
   * {@code Activity}.
   *
   * <p>For an explanation of why this control exists, see {@link
   * FlutterFragment.NewEngineFragmentBuilder#shouldAttachEngineToActivity()}.
   *
   * <p>This property is controlled with a protected method instead of an {@code Intent} argument
   * because the only situation where changing this value would help, is a situation in which {@code
   * FlutterFragmentActivity} is being subclassed to utilize a custom and/or cached {@link
   * FlutterEngine}.
   *
   * <p>Defaults to {@code true}.
   */
  protected boolean shouldAttachEngineToActivity() {
    return true;
  }

  /**
   * Whether to handle the deeplinking from the {@code Intent} automatically if the {@code
   * getInitialRoute} returns null.
   *
   * <p>The default implementation looks {@code <meta-data>} called {@link
   * FlutterActivityLaunchConfigs#HANDLE_DEEPLINKING_META_DATA_KEY} within the Android manifest
   * definition for this {@code FlutterFragmentActivity}.
   */
  @VisibleForTesting
  protected boolean shouldHandleDeeplinking() {
    try {
      Bundle metaData = getMetaData();
      return deepLinkEnabled(metaData);
    } catch (PackageManager.NameNotFoundException e) {
      return false;
    }
  }

  /** Hook for subclasses to easily provide a custom {@code FlutterEngine}. */
  @Nullable
  @Override
  public FlutterEngine provideFlutterEngine(@NonNull Context context) {
    // No-op. Hook for subclasses.
    return null;
  }

  /**
   * Hook for subclasses to easily configure a {@code FlutterEngine}.
   *
   * <p>This method is called after {@link #provideFlutterEngine(Context)}.
   *
   * <p>All plugins listed in the app's pubspec are registered in the base implementation of this
   * method unless the FlutterEngine for this activity was externally created. To avoid the
   * automatic plugin registration for implicitly created FlutterEngines, override this method
   * without invoking super(). To keep automatic plugin registration and further configure the
   * FlutterEngine, override this method, invoke super(), and then configure the FlutterEngine as
   * desired.
   */
  @Override
  public void configureFlutterEngine(@NonNull FlutterEngine flutterEngine) {
    if (flutterFragment != null && flutterFragment.isFlutterEngineInjected()) {
      // If the FlutterEngine was explicitly built and injected into this FlutterActivity, the
      // builder should explicitly decide whether to automatically register plugins via the
      // FlutterEngine's construction parameter or via the AndroidManifest metadata.
      return;
    }

    GeneratedPluginRegister.registerGeneratedPlugins(flutterEngine);
  }

  /**
   * Hook for the host to cleanup references that were established in {@link
   * #configureFlutterEngine(FlutterEngine)} before the host is destroyed or detached.
   *
   * <p>This method is called in {@link #onDestroy()}.
   */
  @Override
  public void cleanUpFlutterEngine(@NonNull FlutterEngine flutterEngine) {
    // No-op. Hook for subclasses.
  }

  /**
   * A custom path to the bundle that contains this Flutter app's resources, e.g., Dart code
   * snapshots.
   *
   * <p>When this {@code FlutterFragmentActivity} is run by Flutter tooling and a data String is
   * included in the launching {@code Intent}, that data String is interpreted as an app bundle
   * path.
   *
   * <p>When otherwise unspecified, the value is null, which defaults to the app bundle path defined
   * in {@link io.flutter.embedding.engine.loader.FlutterLoader#findAppBundlePath()}.
   *
   * <p>Subclasses may override this method to return a custom app bundle path.
   */
  @NonNull
  protected String getAppBundlePath() {
    // If this Activity was launched from tooling, and the incoming Intent contains
    // a custom app bundle path, return that path.
    // TODO(mattcarroll): determine if we should have an explicit FlutterTestActivity instead of
    // conflating.
    if (isDebuggable() && Intent.ACTION_RUN.equals(getIntent().getAction())) {
      String appBundlePath = getIntent().getDataString();
      if (appBundlePath != null) {
        return appBundlePath;
      }
    }

    return null;
  }

  /** Retrieves the meta data specified in the AndroidManifest.xml. */
  @Nullable
  protected Bundle getMetaData() throws PackageManager.NameNotFoundException {
    ActivityInfo activityInfo =
        getPackageManager().getActivityInfo(getComponentName(), PackageManager.GET_META_DATA);
    return activityInfo.metaData;
  }

  /**
   * The Dart entrypoint that will be executed as soon as the Dart snapshot is loaded.
   *
   * <p>This preference can be controlled by setting a {@code <meta-data>} called {@link
   * FlutterActivityLaunchConfigs#DART_ENTRYPOINT_META_DATA_KEY} within the Android manifest
   * definition for this {@code FlutterFragmentActivity}.
   *
   * <p>Subclasses may override this method to directly control the Dart entrypoint.
   */
  @NonNull
  public String getDartEntrypointFunctionName() {
    try {
      Bundle metaData = getMetaData();
      String desiredDartEntrypoint =
          metaData != null ? metaData.getString(DART_ENTRYPOINT_META_DATA_KEY) : null;
      return desiredDartEntrypoint != null ? desiredDartEntrypoint : DEFAULT_DART_ENTRYPOINT;
    } catch (PackageManager.NameNotFoundException e) {
      return DEFAULT_DART_ENTRYPOINT;
    }
  }

  /**
   * The Dart entrypoint arguments will be passed as a list of string to Dart's entrypoint function.
   *
   * <p>A value of null means do not pass any arguments to Dart's entrypoint function.
   *
   * <p>Subclasses may override this method to directly control the Dart entrypoint arguments.
   */
  @Nullable
  public List<String> getDartEntrypointArgs() {
    return (List<String>) getIntent().getSerializableExtra(EXTRA_DART_ENTRYPOINT_ARGS);
  }

  /**
   * The Dart library URI for the entrypoint that will be executed as soon as the Dart snapshot is
   * loaded.
   *
   * <p>Example value: "package:foo/bar.dart"
   *
   * <p>This preference can be controlled by setting a {@code <meta-data>} called {@link
   * FlutterActivityLaunchConfigs#DART_ENTRYPOINT_URI_META_DATA_KEY} within the Android manifest
   * definition for this {@code FlutterFragmentActivity}.
   *
   * <p>A value of null means use the default root library.
   *
   * <p>Subclasses may override this method to directly control the Dart entrypoint uri.
   */
  @Nullable
  public String getDartEntrypointLibraryUri() {
    try {
      Bundle metaData = getMetaData();
      String desiredDartLibraryUri =
          metaData != null ? metaData.getString(DART_ENTRYPOINT_URI_META_DATA_KEY) : null;
      return desiredDartLibraryUri;
    } catch (PackageManager.NameNotFoundException e) {
      return null;
    }
  }

  /**
   * The initial route that a Flutter app will render upon loading and executing its Dart code.
   *
   * <p>This preference can be controlled with 2 methods:
   *
   * <ol>
   *   <li>Pass a boolean as {@link FlutterActivityLaunchConfigs#EXTRA_INITIAL_ROUTE} with the
   *       launching {@code Intent}, or
   *   <li>Set a {@code <meta-data>} called {@link
   *       FlutterActivityLaunchConfigs#INITIAL_ROUTE_META_DATA_KEY} for this {@code Activity} in
   *       the Android manifest.
   * </ol>
   *
   * If both preferences are set, the {@code Intent} preference takes priority.
   *
   * <p>The reason that a {@code <meta-data>} preference is supported is because this {@code
   * Activity} might be the very first {@code Activity} launched, which means the developer won't
   * have control over the incoming {@code Intent}.
   *
   * <p>Subclasses may override this method to directly control the initial route.
   *
   * <p>If this method returns null and the {@code shouldHandleDeeplinking} returns true, the
   * initial route is derived from the {@code Intent} through the Intent.getData() instead.
   */
  protected String getInitialRoute() {
    if (getIntent().hasExtra(EXTRA_INITIAL_ROUTE)) {
      return getIntent().getStringExtra(EXTRA_INITIAL_ROUTE);
    }

    try {
      Bundle metaData = getMetaData();
      String desiredInitialRoute =
          metaData != null ? metaData.getString(INITIAL_ROUTE_META_DATA_KEY) : null;
      return desiredInitialRoute;
    } catch (PackageManager.NameNotFoundException e) {
      return null;
    }
  }

  /**
   * Returns the ID of a statically cached {@link io.flutter.embedding.engine.FlutterEngine} to use
   * within this {@code FlutterFragmentActivity}, or {@code null} if this {@code
   * FlutterFragmentActivity} does not want to use a cached {@link
   * io.flutter.embedding.engine.FlutterEngine}.
   */
  @Nullable
  protected String getCachedEngineId() {
    return getIntent().getStringExtra(EXTRA_CACHED_ENGINE_ID);
  }

  @Nullable
  protected String getCachedEngineGroupId() {
    return getIntent().getStringExtra(EXTRA_CACHED_ENGINE_GROUP_ID);
  }

  /**
   * The desired window background mode of this {@code Activity}, which defaults to {@link
   * BackgroundMode#opaque}.
   */
  @NonNull
  protected BackgroundMode getBackgroundMode() {
    if (getIntent().hasExtra(EXTRA_BACKGROUND_MODE)) {
      return BackgroundMode.valueOf(getIntent().getStringExtra(EXTRA_BACKGROUND_MODE));
    } else {
      return BackgroundMode.opaque;
    }
  }

  /**
   * Returns the desired {@link RenderMode} for the {@link FlutterView} displayed in this {@code
   * FlutterFragmentActivity}.
   *
   * <p>That is, {@link RenderMode#surface} if {@link FlutterFragmentActivity#getBackgroundMode()}
   * is {@link BackgroundMode#opaque} or {@link RenderMode#texture} otherwise.
   */
  @NonNull
  protected RenderMode getRenderMode() {
    final BackgroundMode backgroundMode = getBackgroundMode();
    return backgroundMode == BackgroundMode.opaque ? RenderMode.surface : RenderMode.texture;
  }

  /**
   * Returns true if Flutter is running in "debug mode", and false otherwise.
   *
   * <p>Debug mode allows Flutter to operate with hot reload and hot restart. Release mode does not.
   */
  private boolean isDebuggable() {
    return (getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE) != 0;
  }

  /** Returns a {@link FrameLayout} that is used as the content view of this activity. */
  @NonNull
  protected FrameLayout provideRootLayout(Context context) {
    return new FrameLayout(context);
  }
}
