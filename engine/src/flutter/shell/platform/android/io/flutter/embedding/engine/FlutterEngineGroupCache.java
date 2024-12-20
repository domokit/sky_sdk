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
 * Static singleton cache that holds {@link io.flutter.embedding.engine.FlutterEngineGroup}
 * instances identified by {@code String}s.
 *
 * <p>The ID of a given {@link io.flutter.embedding.engine.FlutterEngineGroup} can be whatever
 * {@code String} is desired.
 *
 * <p>{@link io.flutter.embedding.android.FlutterActivity} and {@link
 * io.flutter.embedding.android.FlutterFragment} use the {@code FlutterEngineGroupCache} singleton
 * internally when instructed to use a cached {@link io.flutter.embedding.engine.FlutterEngineGroup}
 * based on a given ID. See {@link
 * io.flutter.embedding.android.FlutterActivity.NewEngineInGroupIntentBuilder} and {@link
 * io.flutter.embedding.android.FlutterFragment#withNewEngineInGroup(String)} for related APIs.
 */
public class FlutterEngineGroupCache {
  private static volatile FlutterEngineGroupCache instance;

  /**
   * Returns the static singleton instance of {@code FlutterEngineGroupCache}.
   *
   * <p>Creates a new instance if one does not yet exist.
   */
  @NonNull
  public static FlutterEngineGroupCache getInstance() {
    if (instance == null) {
      synchronized (FlutterEngineGroupCache.class) {
        if (instance == null) {
          instance = new FlutterEngineGroupCache();
        }
      }
    }
    return instance;
  }

  private final Map<String, FlutterEngineGroup> cachedEngineGroups = new HashMap<>();

  @VisibleForTesting
  /* package */ FlutterEngineGroupCache() {}

  /**
   * Returns {@code true} if a {@link io.flutter.embedding.engine.FlutterEngineGroup} in this cache
   * is associated with the given {@code engineGroupId}.
   */
  public boolean contains(@NonNull String engineGroupId) {
    return cachedEngineGroups.containsKey(engineGroupId);
  }

  /**
   * Returns the {@link io.flutter.embedding.engine.FlutterEngineGroup} in this cache that is
   * associated with the given {@code engineGroupId}, or {@code null} is no such {@link
   * io.flutter.embedding.engine.FlutterEngineGroup} exists.
   */
  @Nullable
  public FlutterEngineGroup get(@NonNull String engineGroupId) {
    return cachedEngineGroups.get(engineGroupId);
  }

  /**
   * Places the given {@link io.flutter.embedding.engine.FlutterEngineGroup} in this cache and
   * associates it with the given {@code engineGroupId}.
   *
   * <p>If a {@link io.flutter.embedding.engine.FlutterEngineGroup} is null, that {@link
   * io.flutter.embedding.engine.FlutterEngineGroup} is removed from this cache.
   */
  public void put(@NonNull String engineGroupId, @Nullable FlutterEngineGroup engineGroup) {
    if (engineGroup != null) {
      cachedEngineGroups.put(engineGroupId, engineGroup);
    } else {
      cachedEngineGroups.remove(engineGroupId);
    }
  }

  /**
   * Removes any {@link io.flutter.embedding.engine.FlutterEngineGroup} that is currently in the
   * cache that is identified by the given {@code engineGroupId}.
   */
  public void remove(@NonNull String engineGroupId) {
    put(engineGroupId, null);
  }

  /**
   * Removes all {@link io.flutter.embedding.engine.FlutterEngineGroup}'s that are currently in the
   * cache.
   */
  public void clear() {
    cachedEngineGroups.clear();
  }
}
