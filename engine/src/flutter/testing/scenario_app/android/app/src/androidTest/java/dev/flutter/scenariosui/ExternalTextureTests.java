// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenariosui;

import android.content.Intent;
import androidx.annotation.NonNull;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.filters.LargeTest;
import androidx.test.rule.ActivityTestRule;
import dev.flutter.scenarios.ExternalTextureFlutterActivity;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
@LargeTest
public class ExternalTextureTests {
  private Intent intent;

  @Rule @NonNull
  public ActivityTestRule<ExternalTextureFlutterActivity> activityRule =
      new ActivityTestRule<>(
          ExternalTextureFlutterActivity.class,
          /*initialTouchMode=*/ false,
          /*launchActivity=*/ false);

  @Before
  public void setUp() {
    intent = new Intent(Intent.ACTION_MAIN);
  }

  @Test
  public void testCanvasSurface() throws Exception {
    intent.putExtra("scenario_name", "display_texture");
    intent.putExtra("surface_renderer", "canvas");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), "ExternalTextureTests_testCanvasSurface");
  }

  @Test
  public void testMediaSurface() throws Exception {
    intent.putExtra("scenario_name", "display_texture");
    intent.putExtra("surface_renderer", "media");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), "ExternalTextureTests_testMediaSurface");
  }
}
