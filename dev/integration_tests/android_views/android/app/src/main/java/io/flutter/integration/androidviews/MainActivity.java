// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.integration.platformviews;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;

import java.util.HashMap;

import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugins.GeneratedPluginRegistrant;

public class MainActivity extends FlutterActivity implements MethodChannel.MethodCallHandler {
    final static int STORAGE_PERMISSION_CODE = 1;

    MethodChannel mMethodChannel;

    // The method result to complete with the Android permission request result.
    // This is null when not waiting for the Android permission request;
    private MethodChannel.Result permissionResult;

    private View getFlutterView() {
      return findViewById(((FlutterActivity) this).flutterViewId);
    }

    @Override
    public void configureFlutterEngine(FlutterEngine flutterEngine) {
        DartExecutor executor = flutterEngine.getDartExecutor();
        flutterEngine
            .getPlatformViewsController()
            .getRegistry()
            .registerViewFactory("simple_view", new SimpleViewFactory(executor));
        mMethodChannel = new MethodChannel(executor, "android_views_integration");
        mMethodChannel.setMethodCallHandler(this);
        GeneratedPluginRegistrant.registerWith(flutterEngine);
    }

    @Override
    public void onMethodCall(MethodCall methodCall, MethodChannel.Result result) {
        switch (methodCall.method) {
            case "pipeFlutterViewEvents":
                result.success(null);
                return;
            case "stopFlutterViewEvents":
                result.success(null);
                return;
            case "getStoragePermission":
                if (permissionResult != null) {
                    result.error("error", "already waiting for permissions", null);
                    return;
                }
                permissionResult = result;
                getExternalStoragePermissions();
                return;
            case "synthesizeEvent":
                synthesizeEvent(methodCall, result);
                return;
        }
        result.notImplemented();
    }

    @SuppressWarnings("unchecked")
    public void synthesizeEvent(MethodCall methodCall, MethodChannel.Result result) {
        MotionEvent event = MotionEventCodec.decode((HashMap<String, Object>) methodCall.arguments());
        getFlutterView().dispatchTouchEvent(event);
        // TODO(egarciad): Remove invokeMethod since it is not necessary.
        mMethodChannel.invokeMethod("onTouch", MotionEventCodec.encode(event));
        result.success(null);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (requestCode != STORAGE_PERMISSION_CODE || permissionResult == null)
            return;
        boolean permissionGranted = grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED;
        sendPermissionResult(permissionGranted);
    }


    private void getExternalStoragePermissions() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.M)
            return;

        if (checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE)
                == PackageManager.PERMISSION_GRANTED) {
            sendPermissionResult(true);
            return;
        }

        requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, STORAGE_PERMISSION_CODE);
    }

    private void sendPermissionResult(boolean result) {
        if (permissionResult == null)
            return;
        permissionResult.success(result);
        permissionResult = null;
    }
}
