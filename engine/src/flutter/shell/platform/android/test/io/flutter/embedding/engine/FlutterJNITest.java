// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.TargetApi;
import android.content.Context;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.os.LocaleList;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.mutatorsstack.FlutterMutatorsStack;
import io.flutter.embedding.engine.renderer.FlutterUiDisplayListener;
import io.flutter.embedding.engine.systemchannels.LocalizationChannel;
import io.flutter.plugin.localization.LocalizationPlugin;
import io.flutter.plugin.platform.PlatformViewsController;
import java.nio.ByteBuffer;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_24) // LocaleList and scriptCode are API 24+.
public class FlutterJNITest {
  @Test
  public void itAllowsFirstFrameListenersToRemoveThemselvesInline() {
    // --- Test Setup ---
    FlutterJNI flutterJNI = new FlutterJNI();

    AtomicInteger callbackInvocationCount = new AtomicInteger(0);
    FlutterUiDisplayListener callback =
        new FlutterUiDisplayListener() {
          @Override
          public void onFlutterUiDisplayed() {
            callbackInvocationCount.incrementAndGet();
            flutterJNI.removeIsDisplayingFlutterUiListener(this);
          }

          @Override
          public void onFlutterUiNoLongerDisplayed() {}
        };
    flutterJNI.addIsDisplayingFlutterUiListener(callback);

    // --- Execute Test ---
    flutterJNI.onFirstFrame();

    // --- Verify Results ---
    assertEquals(1, callbackInvocationCount.get());

    // --- Execute Test ---
    // The callback removed itself from the listener list. A second call doesn't call the callback.
    flutterJNI.onFirstFrame();

    // --- Verify Results ---
    assertEquals(1, callbackInvocationCount.get());
  }

  @Test
  public void computePlatformResolvedLocaleCallsLocalizationPluginProperly() {
    // --- Test Setup ---
    FlutterJNI flutterJNI = new FlutterJNI();

    Context context = mock(Context.class);
    Resources resources = mock(Resources.class);
    Configuration config = mock(Configuration.class);
    DartExecutor dartExecutor = mock(DartExecutor.class);
    LocaleList localeList =
        new LocaleList(new Locale("es", "MX"), new Locale("zh", "CN"), new Locale("en", "US"));
    when(context.getResources()).thenReturn(resources);
    when(resources.getConfiguration()).thenReturn(config);
    when(config.getLocales()).thenReturn(localeList);

    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(context, new LocalizationChannel(dartExecutor)));
    String[] supportedLocales =
        new String[] {
          "fr", "FR", "",
          "zh", "", "",
          "en", "CA", ""
        };
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "zh");
    assertEquals(result[1], "");
    assertEquals(result[2], "");

    supportedLocales =
        new String[] {
          "fr", "FR", "",
          "ar", "", "",
          "en", "CA", ""
        };
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "en");
    assertEquals(result[1], "CA");
    assertEquals(result[2], "");

    supportedLocales =
        new String[] {
          "fr", "FR", "",
          "ar", "", "",
          "en", "US", ""
        };
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "en");
    assertEquals(result[1], "US");
    assertEquals(result[2], "");

    supportedLocales =
        new String[] {
          "ar", "", "",
          "es", "MX", "",
          "en", "US", ""
        };
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "es");
    assertEquals(result[1], "MX");
    assertEquals(result[2], "");

    // Empty supportedLocales.
    supportedLocales = new String[] {};
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 0);

    // Empty preferredLocales.
    supportedLocales =
        new String[] {
          "fr", "FR", "",
          "zh", "", "",
          "en", "CA", ""
        };
    localeList = new LocaleList();
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The first locale is default.
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "FR");
    assertEquals(result[2], "");
  }

  @Test
  public void setAccessibilityIfAttached() {
    // --- Test Setup ---
    FlutterJNITester flutterJNI = new FlutterJNITester(true);
    int expectedFlag = 100;

    flutterJNI.setAccessibilityFeatures(expectedFlag);
    assertEquals(flutterJNI.flags, expectedFlag);

    flutterJNI.setSemanticsEnabled(true);
    assertTrue(flutterJNI.semanticsEnabled);
  }

  @Test
  public void doesNotSetAccessibilityIfNotAttached() {
    // --- Test Setup ---
    FlutterJNITester flutterJNI = new FlutterJNITester(false);
    int flags = 100;

    flutterJNI.setAccessibilityFeatures(flags);
    assertEquals(flutterJNI.flags, 0);

    flutterJNI.setSemanticsEnabled(true);
    assertFalse(flutterJNI.semanticsEnabled);
  }

  public void onDisplayPlatformView_callsPlatformViewsController() {
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);

    FlutterJNI flutterJNI = new FlutterJNI();
    flutterJNI.setPlatformViewsController(platformViewsController);
    FlutterMutatorsStack stack = new FlutterMutatorsStack();
    // --- Execute Test ---
    flutterJNI.onDisplayPlatformView(
        /*viewId=*/ 1,
        /*x=*/ 10,
        /*y=*/ 20,
        /*width=*/ 100,
        /*height=*/ 200,
        /*viewWidth=*/ 100,
        /*viewHeight=*/ 200,
        /*mutatorsStack=*/ stack);

    // --- Verify Results ---
    verify(platformViewsController, times(1))
        .onDisplayPlatformView(
            /*viewId=*/ 1,
            /*x=*/ 10,
            /*y=*/ 20,
            /*width=*/ 100,
            /*height=*/ 200,
            /*viewWidth=*/ 100,
            /*viewHeight=*/ 200,
            /*mutatorsStack=*/ stack);
  }

  @Test
  public void onDisplayOverlaySurface_callsPlatformViewsController() {
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);

    FlutterJNI flutterJNI = new FlutterJNI();
    flutterJNI.setPlatformViewsController(platformViewsController);

    // --- Execute Test ---
    flutterJNI.onDisplayOverlaySurface(
        /*id=*/ 1, /*x=*/ 10, /*y=*/ 20, /*width=*/ 100, /*height=*/ 200);

    // --- Verify Results ---
    verify(platformViewsController, times(1))
        .onDisplayOverlaySurface(/*id=*/ 1, /*x=*/ 10, /*y=*/ 20, /*width=*/ 100, /*height=*/ 200);
  }

  @Test
  public void onBeginFrame_callsPlatformViewsController() {
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);

    // --- Test Setup ---
    FlutterJNI flutterJNI = new FlutterJNI();
    flutterJNI.setPlatformViewsController(platformViewsController);

    // --- Execute Test ---
    flutterJNI.onBeginFrame();

    // --- Verify Results ---
    verify(platformViewsController, times(1)).onBeginFrame();
  }

  @Test
  public void onEndFrame_callsPlatformViewsController() {
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);

    // --- Test Setup ---
    FlutterJNI flutterJNI = new FlutterJNI();
    flutterJNI.setPlatformViewsController(platformViewsController);

    // --- Execute Test ---
    flutterJNI.onEndFrame();

    // --- Verify Results ---
    verify(platformViewsController, times(1)).onEndFrame();
  }

  @Test
  public void createOverlaySurface_callsPlatformViewsController() {
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);

    FlutterJNI flutterJNI = new FlutterJNI();
    flutterJNI.setPlatformViewsController(platformViewsController);

    // --- Execute Test ---
    flutterJNI.createOverlaySurface();

    // --- Verify Results ---
    verify(platformViewsController, times(1)).createOverlaySurface();
  }

  @Test(expected = IllegalArgumentException.class)
  public void invokePlatformMessageResponseCallback_wantsDirectBuffer() {
    FlutterJNI flutterJNI = new FlutterJNI();
    ByteBuffer buffer = ByteBuffer.allocate(4);
    flutterJNI.invokePlatformMessageResponseCallback(0, buffer, buffer.position());
  }

  @Test
  public void setRefreshRateFPS_callsUpdateRefreshRate() {
    FlutterJNI flutterJNI = spy(new FlutterJNI());
    // --- Execute Test ---
    flutterJNI.setRefreshRateFPS(120.0f);
    // --- Verify Results ---
    verify(flutterJNI, times(1)).updateRefreshRate();
  }

  static class FlutterJNITester extends FlutterJNI {
    FlutterJNITester(boolean attached) {
      this.isAttached = attached;
    }

    final boolean isAttached;
    boolean semanticsEnabled = false;
    int flags = 0;

    @Override
    public boolean isAttached() {
      return isAttached;
    }

    @Override
    public void setSemanticsEnabledInNative(boolean enabled) {
      semanticsEnabled = enabled;
    }

    @Override
    public void setAccessibilityFeaturesInNative(int flags) {
      this.flags = flags;
    }
  }
}
