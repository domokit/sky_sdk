// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import android.view.Surface;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;

@Keep
public class FlutterOverlaySurface {
  @NonNull private final Surface surface;

  private final int id;

  public FlutterOverlaySurface(int id, @NonNull Surface surface) {
    this.id = id;
    this.surface = surface;
  }

  public int getId() {
    return id;
  }

  public Surface getSurface() {
    return surface;
  }
}
