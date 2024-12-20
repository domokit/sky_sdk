// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import androidx.annotation.CallSuper;
import com.google.android.play.core.splitcompat.SplitCompatApplication;
import io.flutter.FlutterInjector;
import io.flutter.embedding.engine.deferredcomponents.PlayStoreDeferredComponentManager;

// TODO(garyq): Add a note about deferred components automatically adding this to manifest via
// manifest variable injection once it is implemented.
/**
 * Flutter's extension of {@link SplitCompatApplication} that injects a {@link
 * PlayStoreDeferredComponentManager} with {@link FlutterInjector} to enable Split AOT Flutter apps.
 *
 * <p>To use this class, either have your custom application class extend
 * FlutterPlayStoreSplitApplication or use it directly in the app's AndroidManifest.xml by adding
 * the following line:
 *
 * <pre>{@code
 * <manifest
 *    ...
 *    <application
 *       android:name="io.flutter.embedding.android.FlutterPlayStoreSplitApplication"
 *       ...>
 *    </application>
 *  </manifest>
 * }</pre>
 *
 * This class is meant to be used with the Google Play store. Custom non-play store applications do
 * not need to extend {@link com.google.android.play.core.splitcompat.SplitCompatApplication} and
 * should inject a custom {@link
 * io.flutter.embedding.engine.deferredcomponents.DeferredComponentManager} implementation like so:
 *
 * <pre>{@code
 * FlutterInjector.setInstance(
 *      new FlutterInjector.Builder().setDeferredComponentManager(yourCustomManager).build());
 * }</pre>
 */
public class FlutterPlayStoreSplitApplication extends SplitCompatApplication {
  @Override
  @CallSuper
  public void onCreate() {
    super.onCreate();
    // Create and inject a PlayStoreDeferredComponentManager, which is the default manager for
    // interacting with the Google Play Store.
    PlayStoreDeferredComponentManager deferredComponentManager =
        new PlayStoreDeferredComponentManager(this, null);
    FlutterInjector.setInstance(
        new FlutterInjector.Builder()
            .setDeferredComponentManager(deferredComponentManager)
            .build());
  }
}
