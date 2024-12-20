// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenariosui;

import android.app.Instrumentation;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import androidx.annotation.NonNull;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.filters.LargeTest;
import androidx.test.platform.app.InstrumentationRegistry;
import androidx.test.rule.ActivityTestRule;
import dev.flutter.scenarios.PlatformViewsActivity;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
@LargeTest
public class PlatformViewWithSurfaceViewUiTest {
  private Instrumentation instrumentation;
  private Intent intent;

  @Rule @NonNull
  public ActivityTestRule<PlatformViewsActivity> activityRule =
      new ActivityTestRule<>(
          PlatformViewsActivity.class, /*initialTouchMode=*/ false, /*launchActivity=*/ false);

  private static String goldName(String suffix) {
    return "PlatformViewWithSurfaceViewUiTest_" + suffix;
  }

  @Before
  public void setUp() {
    instrumentation = InstrumentationRegistry.getInstrumentation();
    intent = new Intent(Intent.ACTION_MAIN);
    // Render a texture.
    intent.putExtra("use_android_view", false);
    intent.putExtra("view_type", PlatformViewsActivity.SURFACE_VIEW_PV);
  }

  @Test
  public void testPlatformView() throws Exception {
    intent.putExtra("scenario_name", "platform_view");
    ScreenshotUtil.capture(activityRule.launchActivity(intent), goldName("testPlatformView"));
  }

  @Test
  public void testPlatformViewMultiple() throws Exception {
    intent.putExtra("scenario_name", "platform_view_multiple");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewMultiple"));
  }

  @Test
  public void testPlatformViewMultipleBackgroundForeground() throws Exception {
    intent.putExtra("scenario_name", "platform_view_multiple_background_foreground");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent),
        goldName("testPlatformViewMultipleBackgroundForeground"));
  }

  @Test
  public void testPlatformViewCliprect() throws Exception {
    intent.putExtra("scenario_name", "platform_view_cliprect");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewCliprect"));
  }

  @Test
  public void testPlatformViewCliprrect() throws Exception {
    intent.putExtra("scenario_name", "platform_view_cliprrect");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewCliprrect"));
  }

  @Test
  public void testPlatformViewClippath() throws Exception {
    intent.putExtra("scenario_name", "platform_view_clippath");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewClippath"));
  }

  @Test
  public void testPlatformViewTransform() throws Exception {
    intent.putExtra("scenario_name", "platform_view_transform");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewTransform"));
  }

  @Test
  public void testPlatformViewOpacity() throws Exception {
    intent.putExtra("scenario_name", "platform_view_opacity");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewOpacity"));
  }

  @Test
  public void testPlatformViewRotate() throws Exception {
    intent.putExtra("scenario_name", "platform_view_rotate");
    PlatformViewsActivity activity = activityRule.launchActivity(intent);
    activity.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
    instrumentation.waitForIdleSync();
    ScreenshotUtil.capture(activity, goldName("testPlatformViewRotate"));
  }

  @Test
  public void testPlatformViewMultipleWithoutOverlays() throws Exception {
    intent.putExtra("scenario_name", "platform_view_multiple_without_overlays");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewMultipleWithoutOverlays"));
  }

  @Test
  public void testPlatformViewTwoIntersectingOverlays() throws Exception {
    intent.putExtra("scenario_name", "platform_view_two_intersecting_overlays");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewTwoIntersectingOverlays"));
  }

  @Test
  public void testPlatformViewWithoutOverlayIntersection() throws Exception {
    intent.putExtra("scenario_name", "platform_view_no_overlay_intersection");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent),
        goldName("testPlatformViewWithoutOverlayIntersection"));
  }

  // TODO(dnfield): This is not safe until https://github.com/flutter/flutter/issues/31990
  // is resolved.
  @Ignore("not safe until https://github.com/flutter/flutter/issues/31990 is resolved")
  @Test
  public void testPlatformViewLargerThanDisplaySize() throws Exception {
    // Regression test for https://github.com/flutter/flutter/issues/28978.
    intent.putExtra("scenario_name", "platform_view_larger_than_display_size");
    ScreenshotUtil.capture(
        activityRule.launchActivity(intent), goldName("testPlatformViewLargerThanDisplaySize"));
  }
}
