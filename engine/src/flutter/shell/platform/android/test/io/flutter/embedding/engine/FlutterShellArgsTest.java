// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package test.io.flutter.embedding.engine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import android.content.Intent;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterShellArgs;
import java.util.Arrays;
import java.util.HashSet;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterShellArgsTest {
  @Test
  public void itProcessesShellFlags() {
    // Setup the test.
    Intent intent = new Intent();
    intent.putExtra("dart-flags", "--observe --no-hot --no-pub");
    intent.putExtra("trace-skia-allowlist", "skia.a,skia.b");

    // Execute the behavior under test.
    FlutterShellArgs args = FlutterShellArgs.fromIntent(intent);
    HashSet<String> argValues = new HashSet<String>(Arrays.asList(args.toArray()));

    // Verify results.
    assertEquals(2, argValues.size());
    assertTrue(argValues.contains("--dart-flags=--observe --no-hot --no-pub"));
    assertTrue(argValues.contains("--trace-skia-allowlist=skia.a,skia.b"));
  }
}
