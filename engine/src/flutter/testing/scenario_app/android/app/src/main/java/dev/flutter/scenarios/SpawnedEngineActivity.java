// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import android.content.Context;
import androidx.annotation.NonNull;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineGroup;

public class SpawnedEngineActivity extends TestActivity {
  static final String TAG = "Scenarios";

  @Override
  @NonNull
  public FlutterEngine provideFlutterEngine(@NonNull Context context) {
    FlutterEngineGroup engineGroup = new FlutterEngineGroup(context);
    FlutterEngineGroup.Options options =
        new FlutterEngineGroup.Options(context).setAutomaticallyRegisterPlugins(false);
    engineGroup.createAndRunEngine(options);

    FlutterEngine secondEngine = engineGroup.createAndRunEngine(options);

    secondEngine
        .getDartExecutor()
        .setMessageHandler("take_screenshot", (byteBuffer, binaryReply) -> notifyFlutterRendered());

    return secondEngine;
  }
}
