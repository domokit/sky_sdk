// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import android.annotation.SuppressLint;
import android.view.View;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/** A handle to an Android view to be embedded in the Flutter hierarchy. */
public interface PlatformView {
  /** Returns the Android view to be embedded in the Flutter hierarchy. */
  @Nullable
  View getView();

  /**
   * Called by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
   * PlatformView} when the Android {@link View} responsible for rendering a Flutter UI is
   * associated with the {@link io.flutter.embedding.engine.FlutterEngine}.
   *
   * <p>This means that our associated {@link io.flutter.embedding.engine.FlutterEngine} can now
   * render a UI and interact with the user.
   *
   * <p>Some platform views may have unusual dependencies on the {@link View} that renders Flutter
   * UIs, such as unique keyboard interactions. That {@link View} is provided here for those
   * purposes. Use of this {@link View} should be avoided if it is not absolutely necessary, because
   * depending on this {@link View} will tend to make platform view code more brittle to future
   * changes.
   */
  // Default interface methods are supported on all min SDK versions of Android.
  @SuppressLint("NewApi")
  default void onFlutterViewAttached(@NonNull View flutterView) {}

  /**
   * Called by the {@link io.flutter.embedding.engine.FlutterEngine} that owns this {@code
   * PlatformView} when the Android {@link View} responsible for rendering a Flutter UI is detached
   * and disassociated from the {@link io.flutter.embedding.engine.FlutterEngine}.
   *
   * <p>This means that our associated {@link io.flutter.embedding.engine.FlutterEngine} no longer
   * has a rendering surface, or a user interaction surface of any kind.
   *
   * <p>This platform view must release any references related to the Android {@link View} that was
   * provided in {@link #onFlutterViewAttached(View)}.
   */
  // Default interface methods are supported on all min SDK versions of Android.
  @SuppressLint("NewApi")
  default void onFlutterViewDetached() {}

  /**
   * Dispose this platform view.
   *
   * <p>The {@link PlatformView} object is unusable after this method is called.
   *
   * <p>Plugins implementing {@link PlatformView} must clear all references to the View object and
   * the PlatformView after this method is called. Failing to do so will result in a memory leak.
   *
   * <p>References related to the Android {@link View} attached in {@link
   * #onFlutterViewAttached(View)} must be released in {@code dispose()} to avoid memory leaks.
   */
  void dispose();

  /**
   * Callback fired when the platform's input connection is locked, or should be used.
   *
   * <p>This hook only exists for rare cases where the plugin relies on the state of the input
   * connection. This probably doesn't need to be implemented.
   */
  @SuppressLint("NewApi")
  default void onInputConnectionLocked() {}

  /**
   * Callback fired when the platform input connection has been unlocked.
   *
   * <p>This hook only exists for rare cases where the plugin relies on the state of the input
   * connection. This probably doesn't need to be implemented.
   */
  @SuppressLint("NewApi")
  default void onInputConnectionUnlocked() {}
}
