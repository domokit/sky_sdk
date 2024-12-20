// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import java.util.HashMap;
import java.util.Map;

/**
 * Static singleton cache that holds {@link io.flutter.embedding.engine.FlutterEngine} instances
 * identified by {@code String}s.
 *
 * <p>The ID of a given {@link io.flutter.embedding.engine.FlutterEngine} can be whatever {@code
 * String} is desired.
 *
 * <p>{@code FlutterEngineCache} is useful for storing pre-warmed {@link
 * io.flutter.embedding.engine.FlutterEngine} instances. {@link
 * io.flutter.embedding.android.FlutterActivity} and {@link
 * io.flutter.embedding.android.FlutterFragment} use the {@code FlutterEngineCache} singleton
 * internally when instructed to use a cached {@link io.flutter.embedding.engine.FlutterEngine}
 * based on a given ID. See {@link
 * io.flutter.embedding.android.FlutterActivity.CachedEngineIntentBuilder} and {@link
 * io.flutter.embedding.android.FlutterFragment#withCachedEngine(String)} for related APIs.
 */
public class FlutterEngineCache {
  private static FlutterEngineCache instance;

  /**
   * Returns the static singleton instance of {@code FlutterEngineCache}.
   *
   * <p>Creates a new instance if one does not yet exist.
   */
  @NonNull
  public static FlutterEngineCache getInstance() {
    if (instance == null) {
      instance = new FlutterEngineCache();
    }
    return instance;
  }

  private final Map<String, FlutterEngine> cachedEngines = new HashMap<>();

  @VisibleForTesting
  /* package */ FlutterEngineCache() {}

  /**
   * Returns {@code true} if a {@link io.flutter.embedding.engine.FlutterEngine} in this cache is
   * associated with the given {@code engineId}.
   */
  public boolean contains(@NonNull String engineId) {
    return cachedEngines.containsKey(engineId);
  }

  /**
   * Returns the {@link io.flutter.embedding.engine.FlutterEngine} in this cache that is associated
   * with the given {@code engineId}, or {@code null} is no such {@link
   * io.flutter.embedding.engine.FlutterEngine} exists.
   */
  @Nullable
  public FlutterEngine get(@NonNull String engineId) {
    return cachedEngines.get(engineId);
  }

  /**
   * Places the given {@link io.flutter.embedding.engine.FlutterEngine} in this cache and associates
   * it with the given {@code engineId}.
   *
   * <p>If a {@link io.flutter.embedding.engine.FlutterEngine} already exists in this cache for the
   * given {@code engineId}, that {@link io.flutter.embedding.engine.FlutterEngine} is removed from
   * this cache.
   */
  public void put(@NonNull String engineId, @Nullable FlutterEngine engine) {
    if (engine != null) {
      cachedEngines.put(engineId, engine);
    } else {
      cachedEngines.remove(engineId);
    }
  }

  /**
   * Removes any {@link io.flutter.embedding.engine.FlutterEngine} that is currently in the cache
   * that is identified by the given {@code engineId}.
   */
  public void remove(@NonNull String engineId) {
    put(engineId, null);
  }

  /**
   * Removes all {@link io.flutter.embedding.engine.FlutterEngine}'s that are currently in the
   * cache.
   */
  public void clear() {
    cachedEngines.clear();
  }
}
