// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenariosui;

import android.content.Intent;
import androidx.annotation.NonNull;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.filters.LargeTest;
import androidx.test.rule.ActivityTestRule;
import dev.flutter.scenarios.SpawnMultiEngineActivity;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
@LargeTest
public class SpawnMultiEngineTest {
  Intent intent;

  @Rule @NonNull
  public ActivityTestRule<SpawnMultiEngineActivity> activityRule =
      new ActivityTestRule<>(
          SpawnMultiEngineActivity.class, /*initialTouchMode=*/ false, /*launchActivity=*/ false);

  @Before
  public void setUp() {
    intent = new Intent(Intent.ACTION_MAIN);
  }

  @Test
  public void testSpawnedEngine() throws Exception {
    activityRule.launchActivity(intent);
  }
}
