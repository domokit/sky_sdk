// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.contentprovider;

import android.content.ContentProvider;
import androidx.annotation.NonNull;

/**
 * Binding that gives {@link ContentProviderAware} plugins access to an associated {@link
 * ContentProvider}.
 */
public interface ContentProviderPluginBinding {

  /**
   * Returns the {@link ContentProvider} that is currently attached to the {@link
   * io.flutter.embedding.engine.FlutterEngine} that owns this {@code
   * ContentProviderAwarePluginBinding}.
   */
  @NonNull
  ContentProvider getContentProvider();
}
