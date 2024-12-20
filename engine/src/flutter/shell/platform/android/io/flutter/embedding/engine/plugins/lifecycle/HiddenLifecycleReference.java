// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.plugins.lifecycle;

import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.lifecycle.Lifecycle;

/**
 * An {@code Object} that can be used to obtain a {@link Lifecycle} reference.
 *
 * <p><strong>DO NOT USE THIS CLASS IN AN APP OR A PLUGIN.</strong>
 *
 * <p>This class is used by the flutter_android_lifecycle package to provide access to a {@link
 * Lifecycle} in a way that makes it easier for Flutter and the Flutter plugin ecosystem to handle
 * breaking changes in Lifecycle libraries.
 */
@Keep
public class HiddenLifecycleReference {
  @NonNull private final Lifecycle lifecycle;

  public HiddenLifecycleReference(@NonNull Lifecycle lifecycle) {
    this.lifecycle = lifecycle;
  }

  @NonNull
  public Lifecycle getLifecycle() {
    return lifecycle;
  }
}
