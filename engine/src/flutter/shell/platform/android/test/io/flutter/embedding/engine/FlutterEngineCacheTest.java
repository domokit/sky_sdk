// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterEngineCacheTest {
  @Test
  public void itHoldsFlutterEngines() {
    // --- Test Setup ---
    FlutterEngine flutterEngine = mock(FlutterEngine.class);
    FlutterEngineCache cache = new FlutterEngineCache();

    // --- Execute Test ---
    cache.put("my_flutter_engine", flutterEngine);

    // --- Verify Results ---
    assertEquals(flutterEngine, cache.get("my_flutter_engine"));
  }

  @Test
  public void itQueriesFlutterEngineExistence() {
    // --- Test Setup ---
    FlutterEngine flutterEngine = mock(FlutterEngine.class);
    FlutterEngineCache cache = new FlutterEngineCache();

    // --- Execute Test ---
    assertFalse(cache.contains("my_flutter_engine"));

    cache.put("my_flutter_engine", flutterEngine);

    // --- Verify Results ---
    assertTrue(cache.contains("my_flutter_engine"));
  }

  @Test
  public void itRemovesFlutterEngines() {
    // --- Test Setup ---
    FlutterEngine flutterEngine = mock(FlutterEngine.class);
    FlutterEngineCache cache = new FlutterEngineCache();

    // --- Execute Test ---
    cache.put("my_flutter_engine", flutterEngine);
    cache.remove("my_flutter_engine");

    // --- Verify Results ---
    assertNull(cache.get("my_flutter_engine"));
  }

  @Test
  public void itRemovesAllFlutterEngines() {
    // --- Test Setup ---
    FlutterEngine flutterEngine = mock(FlutterEngine.class);
    FlutterEngine flutterEngine2 = mock(FlutterEngine.class);
    FlutterEngineCache cache = new FlutterEngineCache();

    // --- Execute Test ---
    cache.put("my_flutter_engine", flutterEngine);
    cache.put("my_flutter_engine_2", flutterEngine2);

    // --- Verify Results ---
    assertEquals(flutterEngine, cache.get("my_flutter_engine"));
    assertEquals(flutterEngine2, cache.get("my_flutter_engine_2"));

    cache.clear();

    // --- Verify Results ---
    assertNull(cache.get("my_flutter_engine"));
    assertNull(cache.get("my_flutter_engine_2"));
  }
}
