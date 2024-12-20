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
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
@LargeTest
public class PlatformViewWithSurfaceViewBadContextUiTest {
  Intent intent;

  @Rule @NonNull
  public ActivityTestRule<PlatformViewsActivity> activityRule =
      new ActivityTestRule<>(
          PlatformViewsActivity.class, /*initialTouchMode=*/ false, /*launchActivity=*/ false);

  private static String goldName(String suffix) {
    return "PlatformViewWithSurfaceViewBadContextUiTest_" + suffix;
  }

  @Before
  public void setUp() {
    intent = new Intent(Intent.ACTION_MAIN);
    // Render a texture.
    intent.putExtra("use_android_view", false);
    intent.putExtra("view_type", PlatformViewsActivity.SURFACE_VIEW_BAD_CONTEXT_PV);
  }

  @Test
  public void testPlatformView() throws Exception {
    intent.putExtra("scenario_name", "platform_view");
    ScreenshotUtil.capture(activityRule.launchActivity(intent), goldName("testPlatformView"));
  }
}
