// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import android.content.Context;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.embedding.engine.FlutterEngine;

/**
 * Provides a {@link io.flutter.embedding.engine.FlutterEngine} instance to be used by a {@code
 * FlutterActivity} or {@code FlutterFragment}.
 *
 * <p>{@link io.flutter.embedding.engine.FlutterEngine} instances require significant time to warm
 * up. Therefore, a developer might choose to hold onto an existing {@link
 * io.flutter.embedding.engine.FlutterEngine} and connect it to various {@link FlutterActivity}s
 * and/or {@code FlutterFragment}s. This interface facilitates providing a cached, pre-warmed {@link
 * io.flutter.embedding.engine.FlutterEngine}.
 */
public interface FlutterEngineProvider {
  /**
   * Returns the {@link io.flutter.embedding.engine.FlutterEngine} that should be used by a child
   * {@code FlutterFragment}.
   *
   * <p>This method may return a new {@link io.flutter.embedding.engine.FlutterEngine}, an existing,
   * cached {@link FlutterEngine}, or null to express that the {@code FlutterEngineProvider} would
   * like the {@code FlutterFragment} to provide its own {@code FlutterEngine} instance.
   *
   * @param context The current context. e.g. An activity.
   * @return The Flutter engine.
   */
  @Nullable
  FlutterEngine provideFlutterEngine(@NonNull Context context);
}
