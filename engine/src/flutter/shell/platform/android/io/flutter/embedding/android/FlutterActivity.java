// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static io.flutter.Build.API_LEVELS;
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
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_ENABLE_STATE_RESTORATION;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.EXTRA_INITIAL_ROUTE;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.INITIAL_ROUTE_META_DATA_KEY;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.NORMAL_THEME_META_DATA_KEY;
import static io.flutter.embedding.android.FlutterActivityLaunchConfigs.deepLinkEnabled;

import android.annotation.TargetApi;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.window.BackEvent;
import android.window.OnBackAnimationCallback;
import android.window.OnBackInvokedCallback;
import android.window.OnBackInvokedDispatcher;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.annotation.VisibleForTesting;
import androidx.lifecycle.Lifecycle;
import androidx.lifecycle.LifecycleOwner;
import androidx.lifecycle.LifecycleRegistry;
import io.flutter.Log;
import io.flutter.embedding.android.FlutterActivityLaunchConfigs.BackgroundMode;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterShellArgs;
import io.flutter.embedding.engine.plugins.activity.ActivityControlSurface;
import io.flutter.embedding.engine.plugins.util.GeneratedPluginRegister;
import io.flutter.plugin.platform.PlatformPlugin;
import io.flutter.plugin.view.SensitiveContentPlugin;
import java.util.ArrayList;
import java.util.List;

/**
 * {@code Activity} which displays a fullscreen Flutter UI.
 *
 * <p>{@code FlutterActivity} is the simplest and most direct way to integrate Flutter within an
 * Android app.
 *
 * <p><strong>FlutterActivity responsibilities</strong>
 *
 * <p>{@code FlutterActivity} maintains the following responsibilities:
 *
 * <ul>
 *   <li>Displays an Android launch screen.
 *   <li>Configures the status bar appearance.
 *   <li>Chooses the Dart execution app bundle path, entrypoint and entrypoint arguments.
 *   <li>Chooses Flutter's initial route.
 *   <li>Renders {@code Activity} transparently, if desired.
 *   <li>Offers hooks for subclasses to provide and configure a {@link
 *       io.flutter.embedding.engine.FlutterEngine}.
 *   <li>Save and restore instance state, see {@code #shouldRestoreAndSaveState()};
 * </ul>
 *
 * <p><strong>Dart entrypoint, entrypoint arguments, initial route, and app bundle path</strong>
 *
 * <p>The Dart entrypoint executed within this {@code Activity} is "main()" by default. To change
 * the entrypoint that a {@code FlutterActivity} executes, subclass {@code FlutterActivity} and
 * override {@link #getDartEntrypointFunctionName()}. For non-main Dart entrypoints to not be
 * tree-shaken away, you need to annotate those functions with {@code @pragma('vm:entry-point')} in
 * Dart.
 *
 * <p>The Dart entrypoint arguments will be passed as a list of string to Dart's entrypoint
 * function. It can be passed using a {@link NewEngineIntentBuilder} via {@link
 * NewEngineIntentBuilder#dartEntrypointArgs}.
 *
 * <p>The Flutter route that is initially loaded within this {@code Activity} is "/". The initial
 * route may be specified explicitly by passing the name of the route as a {@code String} in {@link
 * FlutterActivityLaunchConfigs#EXTRA_INITIAL_ROUTE}, e.g., "my/deep/link".
 *
 * <p>The initial route can each be controlled using a {@link NewEngineIntentBuilder} via {@link
 * NewEngineIntentBuilder#initialRoute}.
 *
 * <p>The app bundle path, Dart entrypoint, Dart entrypoint arguments, and initial route can also be
 * controlled in a subclass of {@code FlutterActivity} by overriding their respective methods:
 *
 * <ul>
 *   <li>{@link #getAppBundlePath()}
 *   <li>{@link #getDartEntrypointFunctionName()}
 *   <li>{@link #getDartEntrypointArgs()}
 *   <li>{@link #getInitialRoute()}
 * </ul>
 *
 * <p>The Dart entrypoint and app bundle path are not supported as {@code Intent} parameters since
 * your Dart library entrypoints are your private APIs and Intents are invocable by other processes.
 *
 * <p><strong>Using a cached FlutterEngine</strong>
 *
 * <p>{@code FlutterActivity} can be used with a cached {@link
 * io.flutter.embedding.engine.FlutterEngine} instead of creating a new one. Use {@link
 * #withCachedEngine(String)} to build a {@code FlutterActivity} {@code Intent} that is configured
 * to use an existing, cached {@link io.flutter.embedding.engine.FlutterEngine}. {@link
 * io.flutter.embedding.engine.FlutterEngineCache} is the cache that is used to obtain a given
 * cached {@link io.flutter.embedding.engine.FlutterEngine}. You must create and put a {@link
 * io.flutter.embedding.engine.FlutterEngine} into the {@link
 * io.flutter.embedding.engine.FlutterEngineCache} yourself before using the {@link
 * #withCachedEngine(String)} builder. An {@code IllegalStateException} will be thrown if a cached
 * engine is requested but does not exist in the cache.
 *
 * <p>When using a cached {@link io.flutter.embedding.engine.FlutterEngine}, that {@link
 * io.flutter.embedding.engine.FlutterEngine} should already be executing Dart code, which means
 * that the Dart entrypoint and initial route have already been defined. Therefore, {@link
 * CachedEngineIntentBuilder} does not offer configuration of these properties.
 *
 * <p>It is generally recommended to use a cached {@link io.flutter.embedding.engine.FlutterEngine}
 * to avoid a momentary delay when initializing a new {@link
 * io.flutter.embedding.engine.FlutterEngine}. The two exceptions to using a cached {@link
 * FlutterEngine} are:
 *
 * <ul>
 *   <li>When {@code FlutterActivity} is the first {@code Activity} displayed by the app, because
 *       pre-warming a {@link io.flutter.embedding.engine.FlutterEngine} would have no impact in
 *       this situation.
 *   <li>When you are unsure when/if you will need to display a Flutter experience.
 * </ul>
 *
 * <p>See https://flutter.dev/docs/development/add-to-app/performance for additional performance
 * explorations on engine loading.
 *
 * <p>The following illustrates how to pre-warm and cache a {@link
 * io.flutter.embedding.engine.FlutterEngine}:
 *
 * <pre>{@code
 * // Create and pre-warm a FlutterEngine.
 * FlutterEngineGroup group = new FlutterEngineGroup(context);
 * FlutterEngine flutterEngine = group.createAndRunDefaultEngine(context);
 * flutterEngine.getDartExecutor().executeDartEntrypoint(DartEntrypoint.createDefault());
 *
 * // Cache the pre-warmed FlutterEngine in the FlutterEngineCache.
 * FlutterEngineCache.getInstance().put("my_engine", flutterEngine);
 * }</pre>
 *
 * <p><strong>Alternatives to FlutterActivity</strong>
 *
 * <p>If Flutter is needed in a location that cannot use an {@code Activity}, consider using a
 * {@link FlutterFragment}. Using a {@link FlutterFragment} requires forwarding some calls from an
 * {@code Activity} to the {@link FlutterFragment}.
 *
 * <p>If Flutter is needed in a location that can only use a {@code View}, consider using a {@link
 * FlutterView}. Using a {@link FlutterView} requires forwarding some calls from an {@code
 * Activity}, as well as forwarding lifecycle calls from an {@code Activity} or a {@code Fragment}.
 *
 * <p><strong>Launch Screen</strong>
 *
 * <p>{@code FlutterActivity} supports the display of an Android "launch screen", which is displayed
 * while the Android application loads. It is only applicable if {@code FlutterActivity} is the
 * first {@code Activity} displayed upon loading the app.
 *
 * <p>Prior to Flutter 2.5, {@code FlutterActivity} supported the display of a Flutter-specific
 * "splash screen" that would be displayed after the launch screen passes. This has since been
 * deprecated. If a launch screen is specified, it will automatically persist for as long as it
 * takes Flutter to initialize and render its first frame.
 *
 * <p>Use Android themes to display a launch screen. Create two themes: a launch theme and a normal
 * theme. In the launch theme, set {@code windowBackground} to the desired {@code Drawable} for the
 * launch screen. In the normal theme, set {@code windowBackground} to any desired background color
 * that should normally appear behind your Flutter content. In most cases this background color will
 * never be seen, but for possible transition edge cases it is a good idea to explicitly replace the
 * launch screen window background with a neutral color.
 *
 * <p>Do not change aspects of system chrome between a launch theme and normal theme. Either define
 * both themes to be fullscreen or not, and define both themes to display the same status bar and
 * navigation bar settings. To adjust system chrome once the Flutter app renders, use platform
 * channels to instruct Android to do so at the appropriate time. This will avoid any jarring visual
 * changes during app startup.
 *
 * <p>In the AndroidManifest.xml, set the theme of {@code FlutterActivity} to the defined launch
 * theme. In the metadata section for {@code FlutterActivity}, defined the following reference to
 * your normal theme:
 *
 * <p>{@code <meta-data android:name="io.flutter.embedding.android.NormalTheme"
 * android:resource="@style/YourNormalTheme" /> }
 *
 * <p>With themes defined, and AndroidManifest.xml updated, Flutter displays the specified launch
 * screen until the Android application is initialized.
 *
 * <p><strong>Alternative Activity</strong> {@link FlutterFragmentActivity} is also available, which
 * is similar to {@code FlutterActivity} but it extends {@code FragmentActivity}. You should use
 * {@code FlutterActivity}, if possible, but if you need a {@code FragmentActivity} then you should
 * use {@link FlutterFragmentActivity}.
 */
// A number of methods in this class have the same implementation as FlutterFragmentActivity. These
// methods are duplicated for readability purposes. Be sure to replicate any change in this class in
// FlutterFragmentActivity, too.
public class FlutterActivity extends Activity
    implements FlutterActivityAndFragmentDelegate.Host, LifecycleOwner {
  private static final String TAG = "FlutterActivity";

  @VisibleForTesting boolean hasRegisteredBackCallback = false;

  /**
   * The ID of the {@code FlutterView} created by this activity.
   *
   * <p>This ID can be used to lookup {@code FlutterView} in the Android view hierarchy. For more,
   * see {@link android.view.View#findViewById}.
   */
  public static final int FLUTTER_VIEW_ID = 0;

  /**
   * Creates an {@link Intent} that launches a {@code FlutterActivity}, which creates a {@link
   * FlutterEngine} that executes a {@code main()} Dart entrypoint, and displays the "/" route as
   * Flutter's initial route.
   *
   * <p>Consider using the {@link #withCachedEngine(String)} {@link Intent} builder to control when
   * the {@link io.flutter.embedding.engine.FlutterEngine} should be created in your application.
   *
   * @param launchContext The launch context. e.g. An Activity.
   * @return The default intent.
   */
  @NonNull
  public static Intent createDefaultIntent(@NonNull Context launchContext) {
    return withNewEngine().build(launchContext);
  }

  /**
   * Creates an {@link NewEngineIntentBuilder}, which can be used to configure an {@link Intent} to
   * launch a {@code FlutterActivity} that internally creates a new {@link
   * io.flutter.embedding.engine.FlutterEngine} using the desired Dart entrypoint, initial route,
   * etc.
   *
   * @return The engine intent builder.
   */
  @NonNull
  public static NewEngineIntentBuilder withNewEngine() {
    return new NewEngineIntentBuilder(FlutterActivity.class);
  }

  /**
   * Builder to create an {@code Intent} that launches a {@code FlutterActivity} with a new {@link
   * FlutterEngine} and the desired configuration.
   */
  public static class NewEngineIntentBuilder {
    private final Class<? extends FlutterActivity> activityClass;
    private String initialRoute = DEFAULT_INITIAL_ROUTE;
    private String backgroundMode = DEFAULT_BACKGROUND_MODE;
    @Nullable private List<String> dartEntrypointArgs;

    /**
     * Constructor that allows this {@code NewEngineIntentBuilder} to be used by subclasses of
     * {@code FlutterActivity}.
     *
     * <p>Subclasses of {@code FlutterActivity} should provide their own static version of {@link
     * #withNewEngine()}, which returns an instance of {@code NewEngineIntentBuilder} constructed
     * with a {@code Class} reference to the {@code FlutterActivity} subclass, e.g.:
     *
     * <p>{@code return new NewEngineIntentBuilder(MyFlutterActivity.class); }
     */
    public NewEngineIntentBuilder(@NonNull Class<? extends FlutterActivity> activityClass) {
      this.activityClass = activityClass;
    }

    /**
     * The initial route that a Flutter app will render in this {@link FlutterActivity}, defaults to
     * "/".
     *
     * @param initialRoute The route.
     * @return The engine intent builder.
     */
    @NonNull
    public NewEngineIntentBuilder initialRoute(@NonNull String initialRoute) {
      this.initialRoute = initialRoute;
      return this;
    }

    /**
     * The mode of {@code FlutterActivity}'s background, either {@link BackgroundMode#opaque} or
     * {@link BackgroundMode#transparent}.
     *
     * <p>The default background mode is {@link BackgroundMode#opaque}.
     *
     * <p>Choosing a background mode of {@link BackgroundMode#transparent} will configure the inner
     * {@link FlutterView} of this {@code FlutterActivity} to be configured with a {@link
     * FlutterTextureView} to support transparency. This choice has a non-trivial performance
     * impact. A transparent background should only be used if it is necessary for the app design
     * being implemented.
     *
     * <p>A {@code FlutterActivity} that is configured with a background mode of {@link
     * BackgroundMode#transparent} must have a theme applied to it that includes the following
     * property: {@code <item name="android:windowIsTranslucent">true</item>}.
     *
     * @param backgroundMode The background mode.
     * @return The engine intent builder.
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
     * Creates and returns an {@link Intent} that will launch a {@code FlutterActivity} with the
     * desired configuration.
     *
     * @param context The context. e.g. An Activity.
     * @return The intent.
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
   * to launch a {@code FlutterActivity} that internally uses an existing {@link
   * io.flutter.embedding.engine.FlutterEngine} that is cached in {@link
   * io.flutter.embedding.engine.FlutterEngineCache}.
   *
   * @param cachedEngineId A cached engine ID.
   * @return The builder.
   */
  public static CachedEngineIntentBuilder withCachedEngine(@NonNull String cachedEngineId) {
    return new CachedEngineIntentBuilder(FlutterActivity.class, cachedEngineId);
  }

  /**
   * Builder to create an {@code Intent} that launches a {@code FlutterActivity} with an existing
   * {@link io.flutter.embedding.engine.FlutterEngine} that is cached in {@link
   * io.flutter.embedding.engine.FlutterEngineCache}.
   */
  public static class CachedEngineIntentBuilder {
    private final Class<? extends FlutterActivity> activityClass;
    private final String cachedEngineId;
    private boolean destroyEngineWithActivity = false;
    private String backgroundMode = DEFAULT_BACKGROUND_MODE;

    /**
     * Constructor that allows this {@code CachedEngineIntentBuilder} to be used by subclasses of
     * {@code FlutterActivity}.
     *
     * <p>Subclasses of {@code FlutterActivity} should provide their own static version of {@link
     * FlutterActivity#withCachedEngine(String)}, which returns an instance of {@code
     * CachedEngineIntentBuilder} constructed with a {@code Class} reference to the {@code
     * FlutterActivity} subclass, e.g.:
     *
     * <p>{@code return new CachedEngineIntentBuilder(MyFlutterActivity.class, engineId); }
     *
     * @param activityClass A subclass of {@code FlutterActivity}.
     * @param engineId The engine id.
     */
    public CachedEngineIntentBuilder(
        @NonNull Class<? extends FlutterActivity> activityClass, @NonNull String engineId) {
      this.activityClass = activityClass;
      this.cachedEngineId = engineId;
    }

    /**
     * Whether the cached {@link io.flutter.embedding.engine.FlutterEngine} should be destroyed and
     * removed from the cache when this {@code FlutterActivity} is destroyed.
     *
     * <p>The default value is {@code false}.
     *
     * @param destroyEngineWithActivity Whether to destroy the engine.
     * @return The builder.
     */
    public CachedEngineIntentBuilder destroyEngineWithActivity(boolean destroyEngineWithActivity) {
      this.destroyEngineWithActivity = destroyEngineWithActivity;
      return this;
    }

    /**
     * The mode of {@code FlutterActivity}'s background, either {@link BackgroundMode#opaque} or
     * {@link BackgroundMode#transparent}.
     *
     * <p>The default background mode is {@link BackgroundMode#opaque}.
     *
     * <p>Choosing a background mode of {@link BackgroundMode#transparent} will configure the inner
     * {@link FlutterView} of this {@code FlutterActivity} to be configured with a {@link
     * FlutterTextureView} to support transparency. This choice has a non-trivial performance
     * impact. A transparent background should only be used if it is necessary for the app design
     * being implemented.
     *
     * <p>A {@code FlutterActivity} that is configured with a background mode of {@link
     * BackgroundMode#transparent} must have a theme applied to it that includes the following
     * property: {@code <item name="android:windowIsTranslucent">true</item>}.
     *
     * @param backgroundMode The background mode
     * @return The builder.
     */
    @NonNull
    public CachedEngineIntentBuilder backgroundMode(@NonNull BackgroundMode backgroundMode) {
      this.backgroundMode = backgroundMode.name();
      return this;
    }

    /**
     * Creates and returns an {@link Intent} that will launch a {@code FlutterActivity} with the
     * desired configuration.
     *
     * @param context The context. e.g. An Activity.
     * @return The intent.
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
   * Intent} to launch a {@code FlutterActivity} by internally creating a FlutterEngine from an
   * existing {@link io.flutter.embedding.engine.FlutterEngineGroup} cached in a specified {@link
   * io.flutter.embedding.engine.FlutterEngineGroupCache}.
   *
   * <pre>{@code
   * // Create a FlutterEngineGroup, such as in the onCreate method of the Application.
   * FlutterEngineGroup engineGroup = new FlutterEngineGroup(this);
   * FlutterEngineGroupCache.getInstance().put("my_cached_engine_group_id", engineGroup);
   *
   * // Start a FlutterActivity with the FlutterEngineGroup by creating an intent with withNewEngineInGroup
   * Intent intent = FlutterActivity.withNewEngineInGroup("my_cached_engine_group_id")
   *     .dartEntrypoint("custom_entrypoint")
   *     .initialRoute("/custom/route")
   *     .backgroundMode(BackgroundMode.transparent)
   *     .build(context);
   * startActivity(intent);
   * }</pre>
   *
   * @param engineGroupId A cached engine group ID.
   * @return The builder.
   */
  public static NewEngineInGroupIntentBuilder withNewEngineInGroup(@NonNull String engineGroupId) {
    return new NewEngineInGroupIntentBuilder(FlutterActivity.class, engineGroupId);
  }

  /**
   * Builder to create an {@code Intent} that launches a {@code FlutterActivity} with a new {@link
   * FlutterEngine} created by FlutterEngineGroup#createAndRunEngine.
   */
  public static class NewEngineInGroupIntentBuilder {
    private final Class<? extends FlutterActivity> activityClass;
    private final String cachedEngineGroupId;
    private String dartEntrypoint = DEFAULT_DART_ENTRYPOINT;
    private String initialRoute = DEFAULT_INITIAL_ROUTE;
    private String backgroundMode = DEFAULT_BACKGROUND_MODE;

    /**
     * Constructor that allows this {@code NewEngineInGroupIntentBuilder} to be used by subclasses
     * of {@code FlutterActivity}.
     *
     * <p>Subclasses of {@code FlutterActivity} should provide their own static version of {@link
     * #withNewEngineInGroup}, which returns an instance of {@code NewEngineInGroupIntentBuilder}
     * constructed with a {@code Class} reference to the {@code FlutterActivity} subclass, e.g.:
     *
     * <p>{@code return new NewEngineInGroupIntentBuilder(MyFlutterActivity.class,
     * cacheedEngineGroupId); }
     *
     * <pre>{@code
     * // Create a FlutterEngineGroup, such as in the onCreate method of the Application.
     * FlutterEngineGroup engineGroup = new FlutterEngineGroup(this);
     * FlutterEngineGroupCache.getInstance().put("my_cached_engine_group_id", engineGroup);
     *
     * // Create a NewEngineInGroupIntentBuilder that would build an intent to start my custom FlutterActivity subclass.
     * FlutterActivity.NewEngineInGroupIntentBuilder intentBuilder =
     *     new FlutterActivity.NewEngineInGroupIntentBuilder(
     *           MyFlutterActivity.class,
     *           app.engineGroupId);
     * intentBuilder.dartEntrypoint("main")
     *     .initialRoute("/custom/route")
     *     .backgroundMode(BackgroundMode.transparent);
     * startActivity(intentBuilder.build(context));
     * }</pre>
     *
     * @param activityClass A subclass of {@code FlutterActivity}.
     * @param engineGroupId The engine group id.
     */
    public NewEngineInGroupIntentBuilder(
        @NonNull Class<? extends FlutterActivity> activityClass, @NonNull String engineGroupId) {
      this.activityClass = activityClass;
      this.cachedEngineGroupId = engineGroupId;
    }

    /**
     * The Dart entrypoint that will be executed in the newly created FlutterEngine as soon as the
     * Dart snapshot is loaded. Default to "main".
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
     * The initial route that a Flutter app will render in this {@link FlutterActivity}, defaults to
     * "/".
     *
     * @param initialRoute The route.
     * @return The engine group intent builder.
     */
    @NonNull
    public NewEngineInGroupIntentBuilder initialRoute(@NonNull String initialRoute) {
      this.initialRoute = initialRoute;
      return this;
    }

    /**
     * The mode of {@code FlutterActivity}'s background, either {@link BackgroundMode#opaque} or
     * {@link BackgroundMode#transparent}.
     *
     * <p>The default background mode is {@link BackgroundMode#opaque}.
     *
     * <p>Choosing a background mode of {@link BackgroundMode#transparent} will configure the inner
     * {@link FlutterView} of this {@code FlutterActivity} to be configured with a {@link
     * FlutterTextureView} to support transparency. This choice has a non-trivial performance
     * impact. A transparent background should only be used if it is necessary for the app design
     * being implemented.
     *
     * <p>A {@code FlutterActivity} that is configured with a background mode of {@link
     * BackgroundMode#transparent} must have a theme applied to it that includes the following
     * property: {@code <item name="android:windowIsTranslucent">true</item>}.
     *
     * @param backgroundMode The background mode.
     * @return The engine group intent builder.
     */
    @NonNull
    public NewEngineInGroupIntentBuilder backgroundMode(@NonNull BackgroundMode backgroundMode) {
      this.backgroundMode = backgroundMode.name();
      return this;
    }

    /**
     * Creates and returns an {@link Intent} that will launch a {@code FlutterActivity} with the
     * desired configuration.
     *
     * @param context The context. e.g. An Activity.
     * @return The intent.
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

  // Delegate that runs all lifecycle and OS hook logic that is common between
  // FlutterActivity and FlutterFragment. See the FlutterActivityAndFragmentDelegate
  // implementation for details about why it exists.
  @VisibleForTesting protected FlutterActivityAndFragmentDelegate delegate;

  @NonNull private LifecycleRegistry lifecycle;

  public FlutterActivity() {
    lifecycle = new LifecycleRegistry(this);
  }

  /**
   * This method exists so that JVM tests can ensure that a delegate exists without putting this
   * Activity through any lifecycle events, because JVM tests cannot handle executing any lifecycle
   * methods, at the time of writing this.
   *
   * <p>The testing infrastructure should be upgraded to make FlutterActivity tests easy to write
   * while exercising real lifecycle methods. At such a time, this method should be removed.
   *
   * @param delegate The delegate.
   */
  // TODO(mattcarroll): remove this when tests allow for it
  // (https://github.com/flutter/flutter/issues/43798)
  @VisibleForTesting
  /* package */ void setDelegate(@NonNull FlutterActivityAndFragmentDelegate delegate) {
    this.delegate = delegate;
  }

  /**
   * Returns the Android App Component exclusively attached to {@link
   * io.flutter.embedding.engine.FlutterEngine}.
   */
  @Override
  public ExclusiveAppComponent<Activity> getExclusiveAppComponent() {
    return delegate;
  }

  @Override
  protected void onCreate(@Nullable Bundle savedInstanceState) {
    switchLaunchThemeForNormalTheme();

    super.onCreate(savedInstanceState);

    if (savedInstanceState != null) {
      boolean frameworkHandlesBack =
          savedInstanceState.getBoolean(
              FlutterActivityAndFragmentDelegate.ON_BACK_CALLBACK_ENABLED_KEY);
      setFrameworkHandlesBack(frameworkHandlesBack);
    }

    delegate = new FlutterActivityAndFragmentDelegate(this);
    delegate.onAttach(this);
    delegate.onRestoreInstanceState(savedInstanceState);

    lifecycle.handleLifecycleEvent(Lifecycle.Event.ON_CREATE);

    configureWindowForTransparency();

    setContentView(createFlutterView());

    configureStatusBarForFullscreenFlutterExperience();
  }

  /**
   * Registers the callback with OnBackInvokedDispatcher to capture back navigation gestures and
   * pass them to the framework.
   *
   * <p>This replaces the deprecated onBackPressed method override in order to support API 33's
   * predictive back navigation feature.
   *
   * <p>The callback must be unregistered in order to prevent unpredictable behavior once outside
   * the Flutter app.
   */
  @VisibleForTesting
  public void registerOnBackInvokedCallback() {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
      getOnBackInvokedDispatcher()
          .registerOnBackInvokedCallback(
              OnBackInvokedDispatcher.PRIORITY_DEFAULT, onBackInvokedCallback);
      hasRegisteredBackCallback = true;
    }
  }

  /**
   * Unregisters the callback from OnBackInvokedDispatcher.
   *
   * <p>This should be called when the activity is no longer in use to prevent unpredictable
   * behavior such as being stuck and unable to press back.
   */
  @VisibleForTesting
  public void unregisterOnBackInvokedCallback() {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_33) {
      getOnBackInvokedDispatcher().unregisterOnBackInvokedCallback(onBackInvokedCallback);
      hasRegisteredBackCallback = false;
    }
  }

  private final OnBackInvokedCallback onBackInvokedCallback =
      Build.VERSION.SDK_INT < API_LEVELS.API_33 ? null : createOnBackInvokedCallback();

  @VisibleForTesting
  protected OnBackInvokedCallback getOnBackInvokedCallback() {
    return onBackInvokedCallback;
  }

  @NonNull
  @TargetApi(API_LEVELS.API_33)
  @RequiresApi(API_LEVELS.API_33)
  private OnBackInvokedCallback createOnBackInvokedCallback() {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_34) {
      return new OnBackAnimationCallback() {
        @Override
        public void onBackInvoked() {
          commitBackGesture();
        }

        @Override
        public void onBackCancelled() {
          cancelBackGesture();
        }

        @Override
        public void onBackProgressed(@NonNull BackEvent backEvent) {
          updateBackGestureProgress(backEvent);
        }

        @Override
        public void onBackStarted(@NonNull BackEvent backEvent) {
          startBackGesture(backEvent);
        }
      };
    }

    return this::onBackPressed;
  }

  @Override
  public void setFrameworkHandlesBack(boolean frameworkHandlesBack) {
    if (frameworkHandlesBack && !hasRegisteredBackCallback) {
      registerOnBackInvokedCallback();
    } else if (!frameworkHandlesBack && hasRegisteredBackCallback) {
      unregisterOnBackInvokedCallback();
    }
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
   *   <li>In the AndroidManifest.xml, set the theme of your {@code FlutterActivity} to your launch
   *       theme.
   *   <li>Add a {@code <meta-data>} property to your {@code FlutterActivity} with a name of
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
          "Could not read meta-data for FlutterActivity. Using the launch theme as normal theme.");
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

  @NonNull
  private View createFlutterView() {
    return delegate.onCreateView(
        /* inflater=*/ null,
        /* container=*/ null,
        /* savedInstanceState=*/ null,
        /*flutterViewId=*/ FLUTTER_VIEW_ID,
        /*shouldDelayFirstAndroidViewDraw=*/ getRenderMode() == RenderMode.surface);
  }

  private void configureStatusBarForFullscreenFlutterExperience() {
    Window window = getWindow();
    window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);
    window.setStatusBarColor(0x40000000);
    window.getDecorView().setSystemUiVisibility(PlatformPlugin.DEFAULT_SYSTEM_UI);
  }

  @Override
  protected void onStart() {
    super.onStart();
    lifecycle.handleLifecycleEvent(Lifecycle.Event.ON_START);
    if (stillAttachedForEvent("onStart")) {
      delegate.onStart();
    }
  }

  @Override
  protected void onResume() {
    super.onResume();
    lifecycle.handleLifecycleEvent(Lifecycle.Event.ON_RESUME);
    if (stillAttachedForEvent("onResume")) {
      delegate.onResume();
    }
  }

  @Override
  public void onPostResume() {
    super.onPostResume();
    if (stillAttachedForEvent("onPostResume")) {
      delegate.onPostResume();
    }
  }

  @Override
  protected void onPause() {
    super.onPause();
    if (stillAttachedForEvent("onPause")) {
      delegate.onPause();
    }
    lifecycle.handleLifecycleEvent(Lifecycle.Event.ON_PAUSE);
  }

  @Override
  protected void onStop() {
    super.onStop();
    if (stillAttachedForEvent("onStop")) {
      delegate.onStop();
    }
    lifecycle.handleLifecycleEvent(Lifecycle.Event.ON_STOP);
  }

  @Override
  protected void onSaveInstanceState(Bundle outState) {
    super.onSaveInstanceState(outState);
    if (stillAttachedForEvent("onSaveInstanceState")) {
      delegate.onSaveInstanceState(outState);
    }
  }

  /**
   * Irreversibly release this activity's control of the {@link
   * io.flutter.embedding.engine.FlutterEngine} and its subcomponents.
   *
   * <p>Calling will disconnect this activity's view from the Flutter renderer, disconnect this
   * activity from plugins' {@link ActivityControlSurface}, and stop system channel messages from
   * this activity.
   *
   * <p>After calling, this activity should be disposed immediately and not be re-used.
   */
  @VisibleForTesting
  public void release() {
    unregisterOnBackInvokedCallback();
    if (delegate != null) {
      delegate.release();
      delegate = null;
    }
  }

  @Override
  public void detachFromFlutterEngine() {
    Log.w(
        TAG,
        "FlutterActivity "
            + this
            + " connection to the engine "
            + getFlutterEngine()
            + " evicted by another attaching activity");
    if (delegate != null) {
      delegate.onDestroyView();
      delegate.onDetach();
    }
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    if (stillAttachedForEvent("onDestroy")) {
      delegate.onDestroyView();
      delegate.onDetach();
    }
    release();
    lifecycle.handleLifecycleEvent(Lifecycle.Event.ON_DESTROY);
  }

  @Override
  protected void onActivityResult(int requestCode, int resultCode, Intent data) {
    if (stillAttachedForEvent("onActivityResult")) {
      delegate.onActivityResult(requestCode, resultCode, data);
    }
  }

  @Override
  protected void onNewIntent(@NonNull Intent intent) {
    // TODO(mattcarroll): change G3 lint rule that forces us to call super
    super.onNewIntent(intent);
    if (stillAttachedForEvent("onNewIntent")) {
      delegate.onNewIntent(intent);
    }
  }

  @Override
  public void onBackPressed() {
    if (stillAttachedForEvent("onBackPressed")) {
      delegate.onBackPressed();
    }
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void startBackGesture(@NonNull BackEvent backEvent) {
    if (stillAttachedForEvent("startBackGesture")) {
      delegate.startBackGesture(backEvent);
    }
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void updateBackGestureProgress(@NonNull BackEvent backEvent) {
    if (stillAttachedForEvent("updateBackGestureProgress")) {
      delegate.updateBackGestureProgress(backEvent);
    }
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void commitBackGesture() {
    if (stillAttachedForEvent("commitBackGesture")) {
      delegate.commitBackGesture();
    }
  }

  @TargetApi(API_LEVELS.API_34)
  @RequiresApi(API_LEVELS.API_34)
  public void cancelBackGesture() {
    if (stillAttachedForEvent("cancelBackGesture")) {
      delegate.cancelBackGesture();
    }
  }

  @Override
  public void onRequestPermissionsResult(
      int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
    if (stillAttachedForEvent("onRequestPermissionsResult")) {
      delegate.onRequestPermissionsResult(requestCode, permissions, grantResults);
    }
  }

  @Override
  public void onUserLeaveHint() {
    if (stillAttachedForEvent("onUserLeaveHint")) {
      delegate.onUserLeaveHint();
    }
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);
    if (stillAttachedForEvent("onWindowFocusChanged")) {
      delegate.onWindowFocusChanged(hasFocus);
    }
  }

  @Override
  public void onTrimMemory(int level) {
    super.onTrimMemory(level);
    if (stillAttachedForEvent("onTrimMemory")) {
      delegate.onTrimMemory(level);
    }
  }

  /**
   * {@link FlutterActivityAndFragmentDelegate.Host} method that is used by {@link
   * FlutterActivityAndFragmentDelegate} to obtain a {@code Context} reference as needed.
   */
  @Override
  @NonNull
  public Context getContext() {
    return this;
  }

  /**
   * {@link FlutterActivityAndFragmentDelegate.Host} method that is used by {@link
   * FlutterActivityAndFragmentDelegate} to obtain an {@code Activity} reference as needed. This
   * reference is used by the delegate to instantiate a {@link FlutterView}, a {@link
   * PlatformPlugin}, and to determine if the {@code Activity} is changing configurations.
   */
  @Override
  @NonNull
  public Activity getActivity() {
    return this;
  }

  /**
   * {@link FlutterActivityAndFragmentDelegate.Host} method that is used by {@link
   * FlutterActivityAndFragmentDelegate} to obtain a {@code Lifecycle} reference as needed. This
   * reference is used by the delegate to provide Flutter plugins with access to lifecycle events.
   */
  @Override
  @NonNull
  public Lifecycle getLifecycle() {
    return lifecycle;
  }

  /**
   * {@link FlutterActivityAndFragmentDelegate.Host} method that is used by {@link
   * FlutterActivityAndFragmentDelegate} to obtain Flutter shell arguments when initializing
   * Flutter.
   */
  @NonNull
  @Override
  public FlutterShellArgs getFlutterShellArgs() {
    return FlutterShellArgs.fromIntent(getIntent());
  }

  /**
   * Returns the ID of a statically cached {@link io.flutter.embedding.engine.FlutterEngine} to use
   * within this {@code FlutterActivity}, or {@code null} if this {@code FlutterActivity} does not
   * want to use a cached {@link io.flutter.embedding.engine.FlutterEngine}.
   */
  @Override
  @Nullable
  public String getCachedEngineId() {
    return getIntent().getStringExtra(EXTRA_CACHED_ENGINE_ID);
  }

  /**
   * Returns the ID of a statically cached {@link io.flutter.embedding.engine.FlutterEngineGroup} to
   * use within this {@code FlutterActivity}, or {@code null} if this {@code FlutterActivity} does
   * not want to use a cached {@link io.flutter.embedding.engine.FlutterEngineGroup}.
   */
  @Override
  @Nullable
  public String getCachedEngineGroupId() {
    return getIntent().getStringExtra(EXTRA_CACHED_ENGINE_GROUP_ID);
  }

  /**
   * Returns false if the {@link io.flutter.embedding.engine.FlutterEngine} backing this {@code
   * FlutterActivity} should outlive this {@code FlutterActivity}, or true to be destroyed when the
   * {@code FlutterActivity} is destroyed.
   *
   * <p>The default value is {@code true} in cases where {@code FlutterActivity} created its own
   * {@link io.flutter.embedding.engine.FlutterEngine}, and {@code false} in cases where a cached
   * {@link io.flutter.embedding.engine.FlutterEngine} was provided.
   */
  @Override
  public boolean shouldDestroyEngineWithHost() {
    boolean explicitDestructionRequested =
        getIntent().getBooleanExtra(EXTRA_DESTROY_ENGINE_WITH_ACTIVITY, false);
    if (getCachedEngineId() != null || delegate.isFlutterEngineFromHost()) {
      // Only destroy a cached engine if explicitly requested by app developer.
      return explicitDestructionRequested;
    } else {
      // If this Activity created the FlutterEngine, destroy it by default unless
      // explicitly requested not to.
      return getIntent().getBooleanExtra(EXTRA_DESTROY_ENGINE_WITH_ACTIVITY, true);
    }
  }

  /**
   * The Dart entrypoint that will be executed as soon as the Dart snapshot is loaded.
   *
   * <p>This preference can be controlled with 2 methods:
   *
   * <ol>
   *   <li>Pass a boolean as {@link FlutterActivityLaunchConfigs#EXTRA_DART_ENTRYPOINT} with the
   *       launching {@code Intent}, or
   *   <li>Set a {@code <meta-data>} called {@link
   *       FlutterActivityLaunchConfigs#DART_ENTRYPOINT_META_DATA_KEY} within the Android manifest
   *       definition for this {@code FlutterActivity}
   * </ol>
   *
   * If both preferences are set, the {@code Intent} preference takes priority.
   *
   * <p>Subclasses may override this method to directly control the Dart entrypoint.
   */
  @NonNull
  public String getDartEntrypointFunctionName() {
    if (getIntent().hasExtra(EXTRA_DART_ENTRYPOINT)) {
      return getIntent().getStringExtra(EXTRA_DART_ENTRYPOINT);
    }

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
   * definition for this {@code FlutterActivity}.
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
  public String getInitialRoute() {
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
   * A custom path to the bundle that contains this Flutter app's resources, e.g., Dart code
   * snapshots.
   *
   * <p>When this {@code FlutterActivity} is run by Flutter tooling and a data String is included in
   * the launching {@code Intent}, that data String is interpreted as an app bundle path.
   *
   * <p>When otherwise unspecified, the value is null, which defaults to the app bundle path defined
   * in {@link io.flutter.embedding.engine.loader.FlutterLoader#findAppBundlePath()}.
   *
   * <p>Subclasses may override this method to return a custom app bundle path.
   */
  @NonNull
  public String getAppBundlePath() {
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

  /**
   * Returns true if Flutter is running in "debug mode", and false otherwise.
   *
   * <p>Debug mode allows Flutter to operate with hot reload and hot restart. Release mode does not.
   */
  private boolean isDebuggable() {
    return (getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE) != 0;
  }

  /**
   * {@link FlutterActivityAndFragmentDelegate.Host} method that is used by {@link
   * FlutterActivityAndFragmentDelegate} to obtain the desired {@link RenderMode} that should be
   * used when instantiating a {@link FlutterView}.
   */
  @NonNull
  @Override
  public RenderMode getRenderMode() {
    return getBackgroundMode() == BackgroundMode.opaque ? RenderMode.surface : RenderMode.texture;
  }

  /**
   * {@link FlutterActivityAndFragmentDelegate.Host} method that is used by {@link
   * FlutterActivityAndFragmentDelegate} to obtain the desired {@link TransparencyMode} that should
   * be used when instantiating a {@link FlutterView}.
   */
  @NonNull
  @Override
  public TransparencyMode getTransparencyMode() {
    return getBackgroundMode() == BackgroundMode.opaque
        ? TransparencyMode.opaque
        : TransparencyMode.transparent;
  }

  /**
   * The desired window background mode of this {@code Activity}, which defaults to {@link
   * BackgroundMode#opaque}.
   *
   * @return The background mode.
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
   * Hook for subclasses to easily provide a custom {@link
   * io.flutter.embedding.engine.FlutterEngine}.
   *
   * <p>This hook is where a cached {@link io.flutter.embedding.engine.FlutterEngine} should be
   * provided, if a cached {@link FlutterEngine} is desired.
   */
  @Nullable
  @Override
  public FlutterEngine provideFlutterEngine(@NonNull Context context) {
    // No-op. Hook for subclasses.
    return null;
  }

  /**
   * Hook for subclasses to obtain a reference to the {@link
   * io.flutter.embedding.engine.FlutterEngine} that is owned by this {@code FlutterActivity}.
   *
   * @return The Flutter engine.
   */
  @Nullable
  protected FlutterEngine getFlutterEngine() {
    return delegate.getFlutterEngine();
  }

  /**
   * Retrieves the meta data specified in the AndroidManifest.xml.
   *
   * @return The meta data.
   * @throws PackageManager.NameNotFoundException if a package with the given name cannot be found
   *     on the system.
   */
  @Nullable
  protected Bundle getMetaData() throws PackageManager.NameNotFoundException {
    ActivityInfo activityInfo =
        getPackageManager().getActivityInfo(getComponentName(), PackageManager.GET_META_DATA);
    return activityInfo.metaData;
  }

  @Nullable
  @Override
  public PlatformPlugin providePlatformPlugin(
      @Nullable Activity activity, @NonNull FlutterEngine flutterEngine) {
    return new PlatformPlugin(getActivity(), flutterEngine.getPlatformChannel(), this);
  }

  @Nullable
  @Override
  public SensitiveContentPlugin provideSensitiveContentPlugin(
      @Nullable Activity activity, @NonNull FlutterEngine flutterEngine) {
    return new SensitiveContentPlugin(getActivity(), flutterEngine.getSensitiveContentChannel());
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
    if (delegate.isFlutterEngineFromHost()) {
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
   * Hook for subclasses to control whether or not the {@link FlutterFragment} within this {@code
   * Activity} automatically attaches its {@link io.flutter.embedding.engine.FlutterEngine} to this
   * {@code Activity}.
   *
   * <p>This property is controlled with a protected method instead of an {@code Intent} argument
   * because the only situation where changing this value would help, is a situation in which {@code
   * FlutterActivity} is being subclassed to utilize a custom and/or cached {@link
   * io.flutter.embedding.engine.FlutterEngine}.
   *
   * <p>Defaults to {@code true}.
   *
   * <p>Control surfaces are used to provide Android resources and lifecycle events to plugins that
   * are attached to the {@link io.flutter.embedding.engine.FlutterEngine}. If {@code
   * shouldAttachEngineToActivity} is true, then this {@code FlutterActivity} will connect its
   * {@link io.flutter.embedding.engine.FlutterEngine} to itself, along with any plugins that are
   * registered with that {@link io.flutter.embedding.engine.FlutterEngine}. This allows plugins to
   * access the {@code Activity}, as well as receive {@code Activity}-specific calls, e.g. {@link
   * Activity#onNewIntent(Intent)}. If {@code shouldAttachEngineToActivity} is false, then this
   * {@code FlutterActivity} will not automatically manage the connection between its {@link
   * FlutterEngine} and itself. In this case, plugins will not be offered a reference to an {@code
   * Activity} or its OS hooks.
   *
   * <p>Returning false from this method does not preclude a {@link
   * io.flutter.embedding.engine.FlutterEngine} from being attaching to a {@code FlutterActivity} -
   * it just prevents the attachment from happening automatically. A developer can choose to
   * subclass {@code FlutterActivity} and then invoke {@link
   * ActivityControlSurface#attachToActivity(ExclusiveAppComponent, Lifecycle)} and {@link
   * ActivityControlSurface#detachFromActivity()} at the desired times.
   *
   * <p>One reason that a developer might choose to manually manage the relationship between the
   * {@code Activity} and {@link io.flutter.embedding.engine.FlutterEngine} is if the developer
   * wants to move the {@link FlutterEngine} somewhere else. For example, a developer might want the
   * {@link io.flutter.embedding.engine.FlutterEngine} to outlive this {@code FlutterActivity} so
   * that it can be used later in a different {@code Activity}. To accomplish this, the {@link
   * io.flutter.embedding.engine.FlutterEngine} may need to be disconnected from this {@code
   * FlutterActivity} at an unusual time, preventing this {@code FlutterActivity} from correctly
   * managing the relationship between the {@link io.flutter.embedding.engine.FlutterEngine} and
   * itself.
   */
  @Override
  public boolean shouldAttachEngineToActivity() {
    return true;
  }

  /**
   * Whether to handle the deeplinking from the {@code Intent} automatically if the {@code
   * getInitialRoute} returns null.
   *
   * <p>The default implementation looks {@code <meta-data>} called {@link
   * FlutterActivityLaunchConfigs#HANDLE_DEEPLINKING_META_DATA_KEY} within the Android manifest
   * definition for this {@code FlutterActivity}.
   */
  @Override
  public boolean shouldHandleDeeplinking() {
    try {
      Bundle metaData = getMetaData();
      return deepLinkEnabled(metaData);
    } catch (PackageManager.NameNotFoundException e) {
      return false;
    }
  }

  @Override
  public void onFlutterSurfaceViewCreated(@NonNull FlutterSurfaceView flutterSurfaceView) {
    // Hook for subclasses.
  }

  @Override
  public void onFlutterTextureViewCreated(@NonNull FlutterTextureView flutterTextureView) {
    // Hook for subclasses.
  }

  @Override
  public void onFlutterUiDisplayed() {
    // Notifies Android that we're fully drawn so that performance metrics can be collected by
    // Flutter performance tests. A few considerations:
    // * reportFullyDrawn was supported in KitKat (API 19), but has a bug around requiring
    // permissions in some Android versions.
    // * reportFullyDrawn behavior isn't tested on pre-Q versions.
    // See https://github.com/flutter/flutter/issues/46172, and
    // https://github.com/flutter/flutter/issues/88767.
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      reportFullyDrawn();
    }
  }

  @Override
  public void onFlutterUiNoLongerDisplayed() {
    // no-op
  }

  @Override
  public boolean shouldRestoreAndSaveState() {
    if (getIntent().hasExtra(EXTRA_ENABLE_STATE_RESTORATION)) {
      return getIntent().getBooleanExtra(EXTRA_ENABLE_STATE_RESTORATION, false);
    }
    if (getCachedEngineId() != null) {
      // Prevent overwriting the existing state in a cached engine with restoration state.
      return false;
    }
    return true;
  }

  /**
   * Give the host application a chance to take control of the app lifecycle events.
   *
   * <p>Return {@code false} means the host application dispatches these app lifecycle events, while
   * return {@code true} means the engine dispatches these events.
   *
   * <p>Defaults to {@code true}.
   */
  @Override
  public boolean shouldDispatchAppLifecycleState() {
    return true;
  }

  /**
   * Whether to automatically attach the {@link FlutterView} to the engine.
   *
   * <p>Returning {@code false} means that the task of attaching the {@link FlutterView} to the
   * engine will be taken over by the host application.
   *
   * <p>Defaults to {@code true}.
   */
  @Override
  public boolean attachToEngineAutomatically() {
    return true;
  }

  @Override
  public boolean getBackCallbackState() {
    return hasRegisteredBackCallback;
  }

  @Override
  public boolean popSystemNavigator() {
    // Hook for subclass. No-op if returns false.
    return false;
  }

  @Override
  public void updateSystemUiOverlays() {
    if (delegate != null) {
      delegate.updateSystemUiOverlays();
    }
  }

  private boolean stillAttachedForEvent(String event) {
    if (delegate == null) {
      Log.w(TAG, "FlutterActivity " + hashCode() + " " + event + " called after release.");
      return false;
    }
    if (!delegate.isAttached()) {
      Log.w(TAG, "FlutterActivity " + hashCode() + " " + event + " called after detach.");
      return false;
    }
    return true;
  }
}
