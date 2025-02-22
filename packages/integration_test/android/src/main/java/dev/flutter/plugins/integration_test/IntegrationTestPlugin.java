// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.plugins.integration_test;

import android.app.Activity;
import android.content.Context;
import com.google.common.util.concurrent.SettableFuture;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.activity.ActivityAware;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.MethodChannel.MethodCallHandler;
import io.flutter.plugin.common.MethodChannel.Result;
import java.io.IOException;
import java.util.Map;
import java.util.concurrent.Future;

/** IntegrationTestPlugin */
public class IntegrationTestPlugin implements MethodCallHandler, FlutterPlugin, ActivityAware {
  // CAMILLE: this is the method channel used to communicate with Java screenshot code
  private static final String CHANNEL = "plugins.flutter.io/integration_test";
  private static final SettableFuture<Map<String, String>> testResultsSettable =
      SettableFuture.create();

  private MethodChannel methodChannel;
  private Activity flutterActivity;
  public static final Future<Map<String, String>> testResults = testResultsSettable;

  /** Plugin registration. */

  @Override
  public void onAttachedToEngine(FlutterPluginBinding binding) {
    onAttachedToEngine(binding.getApplicationContext(), binding.getBinaryMessenger());
  }

  private void onAttachedToEngine(Context unusedApplicationContext, BinaryMessenger messenger) {
    methodChannel = new MethodChannel(messenger, CHANNEL);
    methodChannel.setMethodCallHandler(this);
  }

  @Override
  public void onDetachedFromEngine(FlutterPluginBinding binding) {
    methodChannel.setMethodCallHandler(null);
    methodChannel = null;
  }

  @Override
  public void onAttachedToActivity(ActivityPluginBinding binding) {
    flutterActivity = binding.getActivity();
  }

  @Override
  public void onReattachedToActivityForConfigChanges(ActivityPluginBinding binding) {
    flutterActivity = binding.getActivity();
  }

  @Override
  public void onDetachedFromActivity() {
    flutterActivity = null;
  }

  @Override
  public void onDetachedFromActivityForConfigChanges() {
    flutterActivity = null;
  }

  @Override
  public void onMethodCall(MethodCall call, Result result) {
    switch (call.method) {
      case "allTestsFinished":
        final Map<String, String> results = call.argument("results");
        testResultsSettable.set(results);
        result.success(null);
        return;
      case "convertFlutterSurfaceToImage":
        if (flutterActivity == null) {
          result.error("Could not convert to image", "Activity not initialized", null);
          return;
        }
        FlutterDeviceScreenshot.convertFlutterSurfaceToImage(flutterActivity);
        result.success(null);
        return;
      case "revertFlutterImage":
        if (flutterActivity == null) {
          result.error("Could not revert Flutter image", "Activity not initialized", null);
          return;
        }
        FlutterDeviceScreenshot.revertFlutterImage(flutterActivity);
        result.success(null);
        return;
      case "captureScreenshot":
        if (FlutterDeviceScreenshot.hasInstrumentation()) {
          byte[] image;
          try {
            image = FlutterDeviceScreenshot.captureWithUiAutomation();
          } catch (IOException exception) {
            result.error("Could not capture screenshot", "UiAutomation failed", exception);
            return;
          }
          result.success(image);
          return;
        }
        if (flutterActivity == null) {
          result.error("Could not capture screenshot", "Activity not initialized", null);
          return;
        }
        // CAMILLE: the current path because hasInstrumentation == false
        FlutterDeviceScreenshot.captureView(flutterActivity, methodChannel, result);
        return;
      default:
        result.notImplemented();
    }
  }
}
