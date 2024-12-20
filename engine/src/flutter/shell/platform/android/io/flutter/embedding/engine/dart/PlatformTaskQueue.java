// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.dart;

import android.os.Handler;
import android.os.Looper;
import androidx.annotation.NonNull;
import io.flutter.util.HandlerCompat;

/** A BinaryMessenger.TaskQueue that posts to the platform thread (aka main thread). */
public class PlatformTaskQueue implements DartMessenger.DartMessengerTaskQueue {
  // Use an async handler because the default is subject to vsync synchronization and can result
  // in delays when dispatching tasks.
  @NonNull private final Handler handler = HandlerCompat.createAsyncHandler(Looper.getMainLooper());

  @Override
  public void dispatch(@NonNull Runnable runnable) {
    handler.post(runnable);
  }
}
