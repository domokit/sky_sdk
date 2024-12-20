// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.systemchannels;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import android.content.res.AssetManager;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.deferredcomponents.DeferredComponentManager;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import java.util.HashMap;
import java.util.Map;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

class TestDeferredComponentManager implements DeferredComponentManager {
  DeferredComponentChannel channel;
  String componentName;

  public void setJNI(FlutterJNI flutterJNI) {}

  public void setDeferredComponentChannel(DeferredComponentChannel channel) {
    this.channel = channel;
  }

  public void installDeferredComponent(int loadingUnitId, String componentName) {
    this.componentName = componentName;
  }

  public void completeInstall() {
    channel.completeInstallSuccess(componentName);
  }

  public String getDeferredComponentInstallState(int loadingUnitId, String componentName) {
    return "installed";
  }

  public void loadAssets(int loadingUnitId, String componentName) {}

  public void loadDartLibrary(int loadingUnitId, String componentName) {}

  public boolean uninstallDeferredComponent(int loadingUnitId, String componentName) {
    return true;
  }

  public void destroy() {}
}

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class DeferredComponentChannelTest {
  @Test
  public void deferredComponentChannel_installCompletesResults() {
    MethodChannel rawChannel = mock(MethodChannel.class);
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = new DartExecutor(mockFlutterJNI, mock(AssetManager.class));
    TestDeferredComponentManager testDeferredComponentManager = new TestDeferredComponentManager();
    DeferredComponentChannel fakeDeferredComponentChannel =
        new DeferredComponentChannel(dartExecutor);
    fakeDeferredComponentChannel.setDeferredComponentManager(testDeferredComponentManager);
    testDeferredComponentManager.setDeferredComponentChannel(fakeDeferredComponentChannel);

    Map<String, Object> args = new HashMap<>();
    args.put("loadingUnitId", -1);
    args.put("componentName", "hello");
    MethodCall methodCall = new MethodCall("installDeferredComponent", args);
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    fakeDeferredComponentChannel.parsingMethodHandler.onMethodCall(methodCall, mockResult);

    testDeferredComponentManager.completeInstall();
    verify(mockResult).success(null);
  }

  @Test
  public void deferredComponentChannel_installCompletesMultipleResults() {
    MethodChannel rawChannel = mock(MethodChannel.class);
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = new DartExecutor(mockFlutterJNI, mock(AssetManager.class));
    TestDeferredComponentManager testDeferredComponentManager = new TestDeferredComponentManager();
    DeferredComponentChannel fakeDeferredComponentChannel =
        new DeferredComponentChannel(dartExecutor);
    fakeDeferredComponentChannel.setDeferredComponentManager(testDeferredComponentManager);
    testDeferredComponentManager.setDeferredComponentChannel(fakeDeferredComponentChannel);

    Map<String, Object> args = new HashMap<>();
    args.put("loadingUnitId", -1);
    args.put("componentName", "hello");
    MethodCall methodCall = new MethodCall("installDeferredComponent", args);
    MethodChannel.Result mockResult1 = mock(MethodChannel.Result.class);
    MethodChannel.Result mockResult2 = mock(MethodChannel.Result.class);
    fakeDeferredComponentChannel.parsingMethodHandler.onMethodCall(methodCall, mockResult1);
    fakeDeferredComponentChannel.parsingMethodHandler.onMethodCall(methodCall, mockResult2);

    testDeferredComponentManager.completeInstall();
    verify(mockResult1).success(null);
    verify(mockResult2).success(null);
  }

  @Test
  public void deferredComponentChannel_getInstallState() {
    MethodChannel rawChannel = mock(MethodChannel.class);
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DartExecutor dartExecutor = new DartExecutor(mockFlutterJNI, mock(AssetManager.class));
    TestDeferredComponentManager testDeferredComponentManager = new TestDeferredComponentManager();
    DeferredComponentChannel fakeDeferredComponentChannel =
        new DeferredComponentChannel(dartExecutor);
    fakeDeferredComponentChannel.setDeferredComponentManager(testDeferredComponentManager);
    testDeferredComponentManager.setDeferredComponentChannel(fakeDeferredComponentChannel);

    Map<String, Object> args = new HashMap<>();
    args.put("loadingUnitId", -1);
    args.put("componentName", "hello");
    MethodCall methodCall = new MethodCall("getDeferredComponentInstallState", args);
    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);
    fakeDeferredComponentChannel.parsingMethodHandler.onMethodCall(methodCall, mockResult);

    testDeferredComponentManager.completeInstall();
    verify(mockResult).success("installed");
  }
}
