// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import android.content.Intent;
import androidx.annotation.NonNull;
import org.robolectric.Robolectric;
import org.robolectric.android.controller.ActivityController;

/**
 * Creates a {@code FlutterActivity} for use by test code that do not sit within the {@code
 * io.flutter.embedding.android} package, and offers public access to some package private
 * properties of {@code FlutterActivity} for testing purposes.
 */
public class RobolectricFlutterActivity {
  /**
   * Creates a {@code FlutterActivity} that is controlled by Robolectric, which otherwise can not be
   * done in a test outside of the io.flutter.embedding.android package.
   */
  @NonNull
  public static FlutterActivity createFlutterActivity(@NonNull Intent intent) {
    ActivityController<FlutterActivity> activityController =
        Robolectric.buildActivity(FlutterActivity.class, intent);
    FlutterActivity flutterActivity = activityController.get();
    flutterActivity.setDelegate(new FlutterActivityAndFragmentDelegate(flutterActivity));
    return flutterActivity;
  }

  /**
   * Returns a given {@code FlutterActivity}'s {@code BackgroundMode} for use by tests that do not
   * sit in the {@code io.flutter.embedding.android} package.
   */
  @NonNull
  public static FlutterActivityLaunchConfigs.BackgroundMode getBackgroundMode(
      @NonNull FlutterActivity activity) {
    return activity.getBackgroundMode();
  }
}
