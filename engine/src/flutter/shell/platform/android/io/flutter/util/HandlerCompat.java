// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.util;

import static io.flutter.Build.API_LEVELS;

import android.os.Build;
import android.os.Handler;
import android.os.Looper;

/** Compatability wrapper over {@link Handler}. */
public final class HandlerCompat {
  /**
   * Create a new Handler whose posted messages and runnables are not subject to synchronization
   * barriers such as display vsync.
   *
   * <p>Messages sent to an async handler are guaranteed to be ordered with respect to one another,
   * but not necessarily with respect to messages from other Handlers. Compatibility behavior:
   *
   * <ul>
   *   <li>SDK 28 and above, this method matches platform behavior.
   *   <li>Otherwise, returns a synchronous handler instance.
   * </ul>
   *
   * @param looper the Looper that the new Handler should be bound to
   * @return a new async Handler instance
   * @see Handler#createAsync(Looper)
   */
  public static Handler createAsyncHandler(Looper looper) {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      return Handler.createAsync(looper);
    }
    return new Handler(looper);
  }
}
