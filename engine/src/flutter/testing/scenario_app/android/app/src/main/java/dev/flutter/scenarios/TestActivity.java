// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.Window;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.view.WindowCompat;
import androidx.core.view.WindowInsetsCompat;
import androidx.core.view.WindowInsetsControllerCompat;
import io.flutter.FlutterInjector;
import io.flutter.embedding.engine.FlutterShellArgs;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.plugin.common.JSONMethodCodec;
import io.flutter.plugin.common.MethodChannel;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;

public abstract class TestActivity extends TestableFlutterActivity {
  @Override
  protected void onCreate(@Nullable Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    hideSystemBars(getWindow());
    testFlutterLoaderCallbackWhenInitializedTwice();
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
  }

  @Override
  @NonNull
  public FlutterShellArgs getFlutterShellArgs() {
    FlutterShellArgs args = FlutterShellArgs.fromIntent(getIntent());
    args.add(FlutterShellArgs.ARG_TRACE_STARTUP);
    args.add(FlutterShellArgs.ARG_ENABLE_DART_PROFILING);
    args.add(FlutterShellArgs.ARG_VERBOSE_LOGGING);
    return args;
  }

  @Override
  public void onFlutterUiDisplayed() {
    final Intent launchIntent = getIntent();
    MethodChannel channel =
        new MethodChannel(
            Objects.requireNonNull(getFlutterEngine()).getDartExecutor(),
            "driver",
            JSONMethodCodec.INSTANCE);
    Map<String, Object> test = new HashMap<>(2);
    if (launchIntent.hasExtra("scenario_name")) {
      test.put("name", launchIntent.getStringExtra("scenario_name"));
    } else {
      test.put("name", "animated_color_square");
    }
    test.put("use_android_view", launchIntent.getBooleanExtra("use_android_view", false));
    test.put(
        "expect_android_view_fallback",
        launchIntent.getBooleanExtra("expect_android_view_fallback", false));
    test.put("view_type", launchIntent.getStringExtra("view_type"));
    getScenarioParams(test);
    channel.invokeMethod("set_scenario", test);
  }

  /**
   * Populates test-specific parameters that are sent to the Dart test scenario.
   *
   * @param args The map of test arguments
   */
  protected void getScenarioParams(@NonNull Map<String, Object> args) {}

  /**
   * This method verifies that {@link
   * io.flutter.embedding.engine.loader.FlutterLoader#ensureInitializationCompleteAsync(Context,
   * String[], Handler, Runnable)} invokes its callback when called after initialization.
   */
  protected void testFlutterLoaderCallbackWhenInitializedTwice() {
    FlutterLoader flutterLoader = FlutterInjector.instance().flutterLoader();

    // Flutter is probably already loaded in this app based on
    // code that ran before this method. Nonetheless, invoke the
    // blocking initialization here to ensure it's initialized.
    flutterLoader.startInitialization(getApplicationContext());
    flutterLoader.ensureInitializationComplete(getApplication(), new String[] {});

    // Now that Flutter is loaded, invoke ensureInitializationCompleteAsync with
    // a callback and verify that the callback is invoked.
    Handler mainHandler = new Handler(Looper.getMainLooper());

    final AtomicBoolean didInvokeCallback = new AtomicBoolean(false);

    flutterLoader.ensureInitializationCompleteAsync(
        getApplication(), new String[] {}, mainHandler, () -> didInvokeCallback.set(true));

    mainHandler.post(
        () -> {
          if (!didInvokeCallback.get()) {
            throw new RuntimeException(
                "Failed test: FlutterLoader#ensureInitializationCompleteAsync() did not invoke its callback.");
          }
        });
  }

  private static void hideSystemBars(Window window) {
    final WindowInsetsControllerCompat insetController =
        WindowCompat.getInsetsController(window, window.getDecorView());
    assert insetController != null;
    insetController.setSystemBarsBehavior(
        WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
    insetController.hide(WindowInsetsCompat.Type.systemBars());
  }
}
