// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenariosui;

import android.content.Intent;
import androidx.annotation.NonNull;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.filters.LargeTest;
import androidx.test.rule.ActivityTestRule;
import dev.flutter.scenarios.PlatformViewsActivity;
import leakcanary.FailTestOnLeak;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
@LargeTest
public class MemoryLeakTests {
  @Rule @NonNull
  public ActivityTestRule<PlatformViewsActivity> activityRule =
      new ActivityTestRule<>(
          PlatformViewsActivity.class, /*initialTouchMode=*/ false, /*launchActivity=*/ false);

  @Test
  @FailTestOnLeak
  public void platformViewHybridComposition_launchActivityFinishAndLaunchAgain() throws Exception {
    Intent intent = new Intent(Intent.ACTION_MAIN);
    intent.putExtra("scenario_name", "platform_view");
    intent.putExtra("use_android_view", true);
    intent.putExtra("view_type", PlatformViewsActivity.TEXT_VIEW_PV);

    activityRule.launchActivity(intent);
  }
}
