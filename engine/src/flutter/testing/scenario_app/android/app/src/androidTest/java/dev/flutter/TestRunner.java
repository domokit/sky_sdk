// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter;

import android.os.Bundle;
import androidx.annotation.Nullable;
import androidx.test.runner.AndroidJUnitRunner;
import dev.flutter.scenariosui.ScreenshotUtil;
import io.flutter.FlutterInjector;
import io.flutter.embedding.engine.renderer.FlutterRenderer;

public class TestRunner extends AndroidJUnitRunner {
  @Override
  public void onCreate(@Nullable Bundle arguments) {
    String[] engineArguments = null;
    assert arguments != null;
    if ("true".equals(arguments.getString("enable-impeller"))) {
      // Set up the global settings object so that Impeller is enabled for all tests.
      engineArguments =
          new String[] {
            "--enable-impeller=true",
            "--impeller-backend=" + arguments.getString("impeller-backend", "vulkan")
          };
    } else {
      engineArguments = new String[] {"--enable-impeller=false"};
    }
    FlutterRenderer.debugDisableSurfaceClear = true;
    if ("true".equals(arguments.getString("force-surface-producer-surface-texture"))) {
      // Set a test flag to force the SurfaceProducer to use SurfaceTexture.
      FlutterRenderer.debugForceSurfaceProducerGlTextures = true;
    }
    // For consistency, just always initilaize FlutterJNI etc.
    FlutterInjector.instance().flutterLoader().startInitialization(getTargetContext());
    FlutterInjector.instance()
        .flutterLoader()
        .ensureInitializationComplete(getTargetContext(), engineArguments);
    ScreenshotUtil.onCreate();
    super.onCreate(arguments);
  }

  @Override
  public void finish(int resultCode, @Nullable Bundle results) {
    ScreenshotUtil.finish();
    super.finish(resultCode, results);
  }
}
