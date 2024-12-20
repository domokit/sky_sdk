// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.contentprovider;

import androidx.annotation.NonNull;

/**
 * A {@link io.flutter.embedding.engine.plugins.FlutterPlugin} that wants to know when it is running
 * within a {@link android.content.ContentProvider}.
 */
public interface ContentProviderAware {
  /**
   * Callback triggered when a {@code ContentProviderAware} {@link
   * io.flutter.embedding.engine.plugins.FlutterPlugin} is associated with a {@link
   * android.content.ContentProvider}.
   */
  void onAttachedToContentProvider(@NonNull ContentProviderPluginBinding binding);

  /**
   * Callback triggered when a {@code ContentProviderAware} {@link
   * io.flutter.embedding.engine.plugins.FlutterPlugin} is detached from a {@link
   * android.content.ContentProvider}.
   */
  void onDetachedFromContentProvider();
}
