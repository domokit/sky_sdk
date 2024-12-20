package io.flutter.plugin.common;

import static org.junit.Assert.assertNull;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class BinaryCodecTest {
  @Test
  public void decodeNull() {
    assertNull(BinaryCodec.INSTANCE.decodeMessage(null));
  }
}
