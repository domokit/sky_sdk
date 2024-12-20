// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import android.app.Activity;
import androidx.annotation.NonNull;
import androidx.core.util.Consumer;
import androidx.window.java.layout.WindowInfoTrackerCallbackAdapter;
import androidx.window.layout.WindowLayoutInfo;
import java.util.concurrent.Executor;

/** Wraps {@link WindowInfoTrackerCallbackAdapter} in order to be able to mock it during testing. */
public class WindowInfoRepositoryCallbackAdapterWrapper {

  @NonNull final WindowInfoTrackerCallbackAdapter adapter;

  public WindowInfoRepositoryCallbackAdapterWrapper(
      @NonNull WindowInfoTrackerCallbackAdapter adapter) {
    this.adapter = adapter;
  }

  public void addWindowLayoutInfoListener(
      @NonNull Activity activity,
      @NonNull Executor executor,
      @NonNull Consumer<WindowLayoutInfo> consumer) {
    adapter.addWindowLayoutInfoListener(activity, executor, consumer);
  }

  public void removeWindowLayoutInfoListener(@NonNull Consumer<WindowLayoutInfo> consumer) {
    adapter.removeWindowLayoutInfoListener(consumer);
  }
}
