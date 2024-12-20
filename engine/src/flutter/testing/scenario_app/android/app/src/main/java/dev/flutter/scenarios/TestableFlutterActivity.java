// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import android.os.Bundle;
import android.view.WindowManager;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.engine.FlutterEngine;
import java.util.concurrent.CountDownLatch;

public abstract class TestableFlutterActivity extends FlutterActivity {
  private final CountDownLatch flutterUiRenderedLatch = new CountDownLatch(1);

  @Override
  public void configureFlutterEngine(@NonNull FlutterEngine flutterEngine) {
    // Do not call super. We have no plugins to register, and the automatic
    // registration will fail and print a scary exception in the logs.
    flutterEngine
        .getDartExecutor()
        .setMessageHandler("take_screenshot", (byteBuffer, binaryReply) -> notifyFlutterRendered());
  }

  @Override
  protected void onCreate(@Nullable Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    // On newer versions of Android, this is the default. Because these tests are being used to take
    // screenshots on Skia Gold, we don't want any of the System UI to show up, even for older API
    // versions (i.e. 28).
    //
    // See also:
    // https://github.com/flutter/engine/blob/a9081cce1f0dd730577a36ee1ca6d7af5cdc5a9b/shell/platform/android/io/flutter/embedding/android/FlutterView.java#L696
    // https://github.com/flutter/flutter/issues/143471
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
  }

  protected void notifyFlutterRendered() {
    flutterUiRenderedLatch.countDown();
  }

  public void waitUntilFlutterRendered() {
    try {
      flutterUiRenderedLatch.await();
    } catch (InterruptedException e) {
      throw new RuntimeException(e);
    }
  }
}
