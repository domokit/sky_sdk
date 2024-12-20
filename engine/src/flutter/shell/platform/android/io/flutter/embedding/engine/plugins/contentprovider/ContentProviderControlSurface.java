// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.contentprovider;

import android.content.ContentProvider;
import androidx.annotation.NonNull;
import androidx.lifecycle.Lifecycle;

/**
 * Control surface through which a {@link ContentProvider} attaches to a {@link
 * io.flutter.embedding.engine.FlutterEngine}.
 *
 * <p>A {@link ContentProvider} that contains a {@link io.flutter.embedding.engine.FlutterEngine}
 * should coordinate itself with the {@link io.flutter.embedding.engine.FlutterEngine}'s {@code
 * ContentProviderControlSurface}.
 */
public interface ContentProviderControlSurface {
  /**
   * Call this method from the {@link ContentProvider} that is running the {@link
   * io.flutter.embedding.engine.FlutterEngine} that is associated with this {@code
   * ContentProviderControlSurface}.
   *
   * <p>Once a {@link ContentProvider} is created, and its associated {@link
   * io.flutter.embedding.engine.FlutterEngine} is executing Dart code, the {@link ContentProvider}
   * should invoke this method. At that point the {@link io.flutter.embedding.engine.FlutterEngine}
   * is considered "attached" to the {@link ContentProvider} and all {@link ContentProviderAware}
   * plugins are given access to the {@link ContentProvider}.
   */
  void attachToContentProvider(
      @NonNull ContentProvider contentProvider, @NonNull Lifecycle lifecycle);

  /**
   * Call this method from the {@link ContentProvider} that is attached to this {@code
   * ContentProviderControlSurfaces}'s {@link io.flutter.embedding.engine.FlutterEngine} when the
   * {@link ContentProvider} is about to be destroyed.
   *
   * <p>This method gives each {@link ContentProviderAware} plugin an opportunity to clean up its
   * references before the {@link ContentProvider is destroyed}.
   */
  void detachFromContentProvider();
}
