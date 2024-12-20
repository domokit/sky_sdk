package dev.flutter.scenariosui;

import android.content.Intent;
import androidx.annotation.NonNull;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.filters.LargeTest;
import androidx.test.rule.ActivityTestRule;
import dev.flutter.scenarios.PlatformViewsActivity;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
@LargeTest
public final class DrawSolidBlueScreenTest {
  @Rule @NonNull
  public ActivityTestRule<PlatformViewsActivity> activityRule =
      new ActivityTestRule<>(
          PlatformViewsActivity.class, /*initialTouchMode=*/ false, /*launchActivity=*/ false);

  @Test
  public void test() throws Exception {
    Intent intent = new Intent(Intent.ACTION_MAIN);
    intent.putExtra("scenario_name", "solid_blue");
    ScreenshotUtil.capture(activityRule.launchActivity(intent), "DrawSolidBlueScreenTest");
  }
}
