// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.loader;

import static android.os.Looper.getMainLooper;
import static io.flutter.Build.API_LEVELS;
import static junit.framework.TestCase.assertFalse;
import static junit.framework.TestCase.assertTrue;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.anyLong;
import static org.mockito.Mockito.anyString;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.robolectric.Shadows.shadowOf;

import android.annotation.TargetApi;
import android.app.ActivityManager;
import android.content.Context;
import android.os.Bundle;
import android.util.DisplayMetrics;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterJNI;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterLoaderTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  @Test
  public void itReportsUninitializedAfterCreating() {
    FlutterLoader flutterLoader = new FlutterLoader();
    assertFalse(flutterLoader.initialized());
  }

  @Test
  public void itReportsInitializedAfterInitializing() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();
    assertTrue(flutterLoader.initialized());
    verify(mockFlutterJNI, times(1)).loadLibrary(ctx);
    verify(mockFlutterJNI, times(1)).updateRefreshRate();
  }

  @Test
  public void unsatisfiedLinkErrorPathDoesNotExist() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    ctx.getApplicationInfo().nativeLibraryDir = "/path/that/doesnt/exist";
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    Mockito.doThrow(new UnsatisfiedLinkError("couldn't find \"libflutter.so\""))
        .when(mockFlutterJNI)
        .loadLibrary(ctx);
    try {
      flutterLoader.startInitialization(ctx);
    } catch (UnsupportedOperationException e) {
      assertTrue(
          e.getMessage()
              .contains(
                  "and the native libraries directory (with path /path/that/doesnt/exist) does not exist."));
    }
  }

  @Test
  public void itDefaultsTheOldGenHeapSizeAppropriately() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    ActivityManager activityManager =
        (ActivityManager) ctx.getSystemService(Context.ACTIVITY_SERVICE);
    ActivityManager.MemoryInfo memInfo = new ActivityManager.MemoryInfo();
    activityManager.getMemoryInfo(memInfo);
    int oldGenHeapSizeMegaBytes = (int) (memInfo.totalMem / 1e6 / 2);
    final String oldGenHeapArg = "--old-gen-heap-size=" + oldGenHeapSizeMegaBytes;
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertTrue(arguments.contains(oldGenHeapArg));
  }

  @Test
  public void itDefaultsTheResourceCacheMaxBytesThresholdAppropriately() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    DisplayMetrics displayMetrics = ctx.getResources().getDisplayMetrics();
    int screenWidth = displayMetrics.widthPixels;
    int screenHeight = displayMetrics.heightPixels;
    int resourceCacheMaxBytesThreshold = screenWidth * screenHeight * 12 * 4;
    final String resourceCacheMaxBytesThresholdArg =
        "--resource-cache-max-bytes-threshold=" + resourceCacheMaxBytesThreshold;
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertTrue(arguments.contains(resourceCacheMaxBytesThresholdArg));
  }

  @Test
  public void itSetsLeakVMToTrueByDefault() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    final String leakVMArg = "--leak-vm=true";
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertTrue(arguments.contains(leakVMArg));
  }

  @Test
  public void itSetsTheLeakVMFromMetaData() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);
    Bundle metaData = new Bundle();
    metaData.putBoolean("io.flutter.embedding.android.LeakVM", false);
    ctx.getApplicationInfo().metaData = metaData;

    FlutterLoader.Settings settings = new FlutterLoader.Settings();
    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx, settings);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    final String leakVMArg = "--leak-vm=false";
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertTrue(arguments.contains(leakVMArg));
  }

  @Test
  public void itUsesCorrectExecutorService() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    ExecutorService mockExecutorService = mock(ExecutorService.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI, mockExecutorService);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx);
    verify(mockExecutorService, times(1)).submit(any(Callable.class));
  }

  @Test
  public void itDoesNotSetEnableImpellerByDefault() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    final String enableImpellerArg = "--enable-impeller";
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertFalse(arguments.contains(enableImpellerArg));
  }

  @Test
  public void itDoesNotSetEnableVulkanValidationByDefault() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    final String enableVulkanValidationArg = "--enable-vulkan-validation";
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertFalse(arguments.contains(enableVulkanValidationArg));
  }

  @Test
  public void itSetsEnableImpellerFromMetaData() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);
    Bundle metaData = new Bundle();
    metaData.putBoolean("io.flutter.embedding.android.EnableImpeller", true);
    ctx.getApplicationInfo().metaData = metaData;

    FlutterLoader.Settings settings = new FlutterLoader.Settings();
    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx, settings);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    final String enableImpellerArg = "--enable-impeller=true";
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertTrue(arguments.contains(enableImpellerArg));
  }

  @Test
  public void itSetsDisableSurfaceControlFromMetaData() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);
    Bundle metaData = new Bundle();
    metaData.putBoolean("io.flutter.embedding.android.DisableSurfaceControl", true);
    ctx.getApplicationInfo().metaData = metaData;

    FlutterLoader.Settings settings = new FlutterLoader.Settings();
    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(ctx, settings);
    flutterLoader.ensureInitializationComplete(ctx, null);
    shadowOf(getMainLooper()).idle();

    final String disabledControlArg = "--disable-surface-control";
    ArgumentCaptor<String[]> shellArgsCaptor = ArgumentCaptor.forClass(String[].class);
    verify(mockFlutterJNI, times(1))
        .init(eq(ctx), shellArgsCaptor.capture(), anyString(), anyString(), anyString(), anyLong());
    List<String> arguments = Arrays.asList(shellArgsCaptor.getValue());
    assertTrue(arguments.contains(disabledControlArg));
  }

  @Test
  @TargetApi(API_LEVELS.API_23)
  @Config(sdk = API_LEVELS.API_23)
  public void itReportsFpsToVsyncWaiterAndroidM() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    FlutterLoader flutterLoader = new FlutterLoader(mockFlutterJNI);

    Context appContextSpy = spy(ctx);

    assertFalse(flutterLoader.initialized());
    flutterLoader.startInitialization(appContextSpy);
    verify(appContextSpy, never()).getSystemService(anyString());
  }
}
