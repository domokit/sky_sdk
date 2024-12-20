// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.renderer;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import android.content.Context;
import android.content.Intent;
import androidx.test.core.app.ApplicationProvider;
import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineCache;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.loader.FlutterLoader;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

/**
 * Prepares and returns a {@link FlutterEngine} and {@link Intent} primed with an engine for tests.
 */
public final class FlutterEngineRule extends TestWatcher {
  private static final String cachedEngineId = "flutter_engine_rule_cached_engine";
  private final Context ctx = ApplicationProvider.getApplicationContext();
  private FlutterJNI flutterJNI;
  private FlutterEngine flutterEngine;
  private boolean jniIsAttached = true;

  @Override
  protected void starting(Description description) {
    // Setup mock JNI.
    flutterJNI = mock(FlutterJNI.class);
    when(flutterJNI.isAttached()).thenAnswer(i -> jniIsAttached);

    // We will not try to load plugins in these tests.
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    when(mockFlutterLoader.automaticallyRegisterPlugins()).thenReturn(false);

    // Create an engine.
    flutterEngine = new FlutterEngine(ctx, mockFlutterLoader, flutterJNI);

    // Place it in the engine cache.
    FlutterEngineCache.getInstance().put(cachedEngineId, flutterEngine);
  }

  @Override
  protected void finished(Description description) {
    FlutterEngineCache.getInstance().clear();
  }

  /**
   * Returns a Mockito-mocked version of {@link FlutterJNI}.
   *
   * @return an instance that is already considered attached.
   */
  FlutterJNI getFlutterJNI() {
    return this.flutterJNI;
  }

  /**
   * Returns a pre-configured engine.
   *
   * @return flutter engine using the mock provided by {{@link #getFlutterJNI()}}.
   */
  FlutterEngine getFlutterEngine() {
    return this.flutterEngine;
  }

  /**
   * Sets what {@link FlutterJNI#isAttached()} returns. If not invoked, defaults to true.
   *
   * @param isAttached whether to consider JNI attached.
   */
  void setJniIsAttached(boolean isAttached) {
    this.jniIsAttached = isAttached;
  }

  /**
   * Creates an intent with {@link FlutterEngine} instance already provided.
   *
   * @return intent, i.e. to use with {@link androidx.test.ext.junit.rules.ActivityScenarioRule}.
   */
  Intent makeIntent() {
    return FlutterActivity.withCachedEngine(cachedEngineId).build(ctx);
  }
}
