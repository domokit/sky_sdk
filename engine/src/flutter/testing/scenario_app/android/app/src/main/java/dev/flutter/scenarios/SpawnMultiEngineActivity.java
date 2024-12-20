// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.scenarios;

import android.content.Context;
import androidx.annotation.NonNull;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngineGroup;

public class SpawnMultiEngineActivity extends TestActivity {
  static final String TAG = "Scenarios";

  @Override
  @NonNull
  public FlutterEngine provideFlutterEngine(@NonNull Context context) {
    FlutterEngineGroup engineGroup = new FlutterEngineGroup(context);
    FlutterEngineGroup.Options options =
        new FlutterEngineGroup.Options(context).setAutomaticallyRegisterPlugins(false);
    FlutterEngine firstEngine = engineGroup.createAndRunEngine(options);

    FlutterEngine secondEngine = engineGroup.createAndRunEngine(options);

    // Check that a new engine can be spawned from the group even if the group's
    // original engine has been destroyed.
    firstEngine.destroy();
    FlutterEngine thirdEngine = engineGroup.createAndRunEngine(options);

    return thirdEngine;
  }
}
