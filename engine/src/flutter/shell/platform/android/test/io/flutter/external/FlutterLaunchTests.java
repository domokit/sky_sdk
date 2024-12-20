package io.flutter.external;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import android.content.Intent;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.android.FlutterActivityLaunchConfigs.BackgroundMode;
import io.flutter.embedding.android.RenderMode;
import io.flutter.embedding.android.RobolectricFlutterActivity;
import io.flutter.embedding.android.TransparencyMode;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterLaunchTests {
  @Test
  public void launchFlutterActivity_with_defaultIntent_expect_defaultConfiguration() {
    Intent intent =
        FlutterActivity.createDefaultIntent(ApplicationProvider.getApplicationContext());
    FlutterActivity flutterActivity = RobolectricFlutterActivity.createFlutterActivity(intent);

    assertEquals("main", flutterActivity.getDartEntrypointFunctionName());
    assertEquals("/", flutterActivity.getInitialRoute());
    assertArrayEquals(new String[] {}, flutterActivity.getFlutterShellArgs().toArray());
    assertTrue(flutterActivity.shouldAttachEngineToActivity());
    assertNull(flutterActivity.getCachedEngineId());
    assertTrue(flutterActivity.shouldDestroyEngineWithHost());
    assertEquals(
        BackgroundMode.opaque, RobolectricFlutterActivity.getBackgroundMode(flutterActivity));
    assertEquals(RenderMode.surface, flutterActivity.getRenderMode());
    assertEquals(TransparencyMode.opaque, flutterActivity.getTransparencyMode());
  }
}
