// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static junit.framework.TestCase.assertEquals;
import static junit.framework.TestCase.assertFalse;
import static junit.framework.TestCase.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.clearInvocations;
import static org.mockito.Mockito.doNothing;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.app.Activity;
import android.content.Context;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.graphics.Canvas;
import android.graphics.Insets;
import android.graphics.Rect;
import android.graphics.Region;
import android.hardware.HardwareBuffer;
import android.hardware.display.DisplayManager;
import android.media.Image;
import android.media.Image.Plane;
import android.media.ImageReader;
import android.os.Build;
import android.provider.Settings;
import android.util.DisplayMetrics;
import android.view.DisplayCutout;
import android.view.Surface;
import android.view.View;
import android.view.WindowInsets;
import android.widget.FrameLayout;
import androidx.core.util.Consumer;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.window.layout.FoldingFeature;
import androidx.window.layout.WindowLayoutInfo;
import io.flutter.Build.API_LEVELS;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import io.flutter.embedding.engine.systemchannels.SettingsChannel;
import io.flutter.plugin.platform.PlatformViewsController;
import io.flutter.plugin.platform.PlatformViewsController2;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.mockito.Spy;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;
import org.robolectric.Robolectric;
import org.robolectric.Shadows;
import org.robolectric.annotation.Config;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;
import org.robolectric.shadows.ShadowDisplay;
import org.robolectric.shadows.ShadowViewGroup;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(30)
public class FlutterViewTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  @Mock FlutterJNI mockFlutterJni;
  @Mock FlutterLoader mockFlutterLoader;
  @Spy PlatformViewsController platformViewsController;
  @Spy PlatformViewsController2 platformViewsController2;

  @Before
  public void setUp() {
    MockitoAnnotations.openMocks(this);
    when(mockFlutterJni.isAttached()).thenReturn(true);
    // Uncomment the following line to enable logging output in test.
    // ShadowLog.stream = System.out;
  }

  @Test
  public void attachToFlutterEngine_alertsPlatformViews() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    when(flutterEngine.getPlatformViewsController()).thenReturn(platformViewsController);
    when(flutterEngine.getPlatformViewsController2()).thenReturn(platformViewsController2);

    flutterView.attachToFlutterEngine(flutterEngine);

    verify(platformViewsController, times(1)).attachToView(flutterView);
    verify(platformViewsController2, times(1)).attachToView(flutterView);
  }

  @Test
  public void flutterView_importantForAutofillDoesNotExcludeDescendants() {
    FlutterView flutterView = new FlutterView(ctx);

    // Value should not exclude descendants because platform views are added as child views and
    // can be eligible for autofill (e.g. a WebView).
    assertEquals(View.IMPORTANT_FOR_AUTOFILL_YES, flutterView.getImportantForAutofill());
  }

  @Test
  public void detachFromFlutterEngine_alertsPlatformViews() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    when(flutterEngine.getPlatformViewsController()).thenReturn(platformViewsController);
    when(flutterEngine.getPlatformViewsController2()).thenReturn(platformViewsController2);

    flutterView.attachToFlutterEngine(flutterEngine);
    flutterView.detachFromFlutterEngine();

    verify(platformViewsController, times(1)).detachFromView();
    verify(platformViewsController2, times(1)).detachFromView();
  }

  @Test
  public void detachFromFlutterEngine_turnsOffA11y() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    flutterView.attachToFlutterEngine(flutterEngine);
    flutterView.detachFromFlutterEngine();

    verify(flutterRenderer, times(1)).setSemanticsEnabled(false);
  }

  @Test
  public void detachFromFlutterEngine_revertImageView() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));

    flutterView.attachToFlutterEngine(flutterEngine);
    assertFalse(flutterView.renderSurface instanceof FlutterImageView);

    flutterView.convertToImageView();
    assertTrue(flutterView.renderSurface instanceof FlutterImageView);

    flutterView.detachFromFlutterEngine();
    assertFalse(flutterView.renderSurface instanceof FlutterImageView);
  }

  @Test
  public void detachFromFlutterEngine_removeImageView() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));

    flutterView.attachToFlutterEngine(flutterEngine);
    flutterView.convertToImageView();
    assertEquals(flutterView.getChildCount(), 2);
    View view = flutterView.getChildAt(1);
    assertTrue(view instanceof FlutterImageView);

    flutterView.detachFromFlutterEngine();
    assertEquals(flutterView.getChildCount(), 1);
    view = flutterView.getChildAt(0);
    assertFalse(view instanceof FlutterImageView);
  }

  @Test
  public void detachFromFlutterEngine_closesImageView() {
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));

    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    FlutterImageView imageViewMock = mock(FlutterImageView.class);
    when(imageViewMock.getAttachedRenderer()).thenReturn(flutterRenderer);

    FlutterView flutterView = spy(new FlutterView(ctx));
    when(flutterView.createImageView()).thenReturn(imageViewMock);

    flutterView.attachToFlutterEngine(flutterEngine);

    assertFalse(flutterView.renderSurface == imageViewMock);

    flutterView.convertToImageView();
    assertTrue(flutterView.renderSurface == imageViewMock);

    flutterView.detachFromFlutterEngine();
    assertFalse(flutterView.renderSurface == imageViewMock);
    verify(imageViewMock, times(1)).closeImageReader();
  }

  @Test
  public void flutterImageView_revertImageViewAndAvoidNPE() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    flutterView.attachToFlutterEngine(flutterEngine);
    flutterView.convertToImageView();
    assertTrue(flutterView.renderSurface instanceof FlutterImageView);

    // Register a `FlutterUiDisplayListener` callback.
    // During callback execution it will invoke `flutterImageView.detachFromRenderer()`.
    flutterView.revertImageView(
        () -> {
          // No-op
        });
    assertFalse(flutterView.renderSurface instanceof FlutterImageView);

    flutterView.detachFromFlutterEngine();
    assertEquals(null, flutterView.getCurrentImageSurface());

    // Invoke all registered `FlutterUiDisplayListener` callback
    mockFlutterJni.onFirstFrame();
  }

  @Test
  public void onConfigurationChanged_fizzlesWhenNullEngine() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));

    Configuration configuration = ctx.getResources().getConfiguration();
    // 1 invocation of channels.
    flutterView.attachToFlutterEngine(flutterEngine);
    flutterView.onConfigurationChanged(configuration);
    flutterView.detachFromFlutterEngine();

    // Should fizzle.
    flutterView.onConfigurationChanged(configuration);

    verify(flutterEngine, times(1)).getLocalizationPlugin();
    verify(flutterEngine, times(2)).getSettingsChannel();
  }

  @SuppressWarnings("deprecation")
  // Robolectric.setupActivity
  // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
  @Test
  public void onConfigurationChanged_notifiesEngineOfDisplaySize() {
    FlutterView flutterView = new FlutterView(Robolectric.setupActivity(Activity.class));
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));

    Configuration configuration = ctx.getResources().getConfiguration();

    flutterView.attachToFlutterEngine(flutterEngine);
    flutterView.onConfigurationChanged(configuration);

    verify(flutterEngine, times(1))
        .updateDisplayMetrics(any(Float.class), any(Float.class), any(Float.class));
  }

  public void itSendsTextHidePasswordToFrameworkOnAttach() {
    // Setup test.
    AtomicReference<Boolean> reportedShowPassword = new AtomicReference<>();

    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    Settings.System.putInt(
        flutterView.getContext().getContentResolver(), Settings.System.TEXT_SHOW_PASSWORD, 0);

    SettingsChannel fakeSettingsChannel = mock(SettingsChannel.class);
    SettingsChannel.MessageBuilder fakeMessageBuilder = mock(SettingsChannel.MessageBuilder.class);
    when(fakeMessageBuilder.setTextScaleFactor(any(Float.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setPlatformBrightness(any(SettingsChannel.PlatformBrightness.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setUse24HourFormat(any(Boolean.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setBrieflyShowPassword(any(Boolean.class)))
        .thenAnswer(
            new Answer<SettingsChannel.MessageBuilder>() {
              @Override
              public SettingsChannel.MessageBuilder answer(InvocationOnMock invocation)
                  throws Throwable {
                reportedShowPassword.set((Boolean) invocation.getArguments()[0]);
                return fakeMessageBuilder;
              }
            });
    when(fakeSettingsChannel.startMessage()).thenReturn(fakeMessageBuilder);
    when(flutterEngine.getSettingsChannel()).thenReturn(fakeSettingsChannel);

    flutterView.attachToFlutterEngine(flutterEngine);

    // Verify results.
    assertFalse(reportedShowPassword.get());
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility
  // This test uses the API 30+ Algorithm for window insets. The legacy algorithm is
  // set to -1 values, so it is clear if the wrong algorithm is used.
  @Test
  @TargetApi(30)
  @Config(sdk = 30)
  public void reportSystemInsetWhenNotFullscreen() {
    // Without custom shadows, the default system ui visibility flags is 0.
    FlutterView flutterView = new FlutterView(ctx);
    assertEquals(0, flutterView.getSystemUiVisibility());

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets, the viewport
    // metrics
    // default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets =
        new WindowInsets.Builder()
            .setInsets(
                android.view.WindowInsets.Type.navigationBars()
                    | android.view.WindowInsets.Type.statusBars(),
                Insets.of(100, 100, 100, 100))
            .build();
    flutterView.onApplyWindowInsets(windowInsets);

    // Verify.
    verify(flutterRenderer, times(3)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Top padding is reported as-is.
    validateViewportMetricPadding(viewportMetricsCaptor, 100, 100, 100, 100);
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility
  // This test uses the API 30+ Algorithm for window insets. This test requires API 34 or
  // higher to use the systemOverlays inset. The legacy algorithm is
  // set to -1 values, so it is clear if the wrong algorithm is used.
  @Test
  @TargetApi(34)
  @Config(minSdk = 34)
  public void reportSystemInsetWhenNotFullscreenForSystemBar() {
    // Without custom shadows, the default system ui visibility flags is 0.
    FlutterView flutterView = new FlutterView(ctx);
    assertEquals(0, flutterView.getSystemUiVisibility());

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());

    // When we attach a new FlutterView to the engine without any system insets, the viewport
    // metrics default to 0.
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a statusBar inset.
    WindowInsets statusBarwindowInsets =
        new WindowInsets.Builder()
            .setInsets(android.view.WindowInsets.Type.captionBar(), Insets.of(0, 50, 0, 0))
            .setInsets(android.view.WindowInsets.Type.statusBars(), Insets.of(0, 100, 0, 0))
            .build();
    flutterView.onApplyWindowInsets(statusBarwindowInsets);

    // Verify.
    verify(flutterRenderer, times(3)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Confirm that the statusBar inset is used because it is the largest of the insets faked.
    validateViewportMetricPadding(viewportMetricsCaptor, 0, 100, 0, 0);
    clearInvocations(flutterRenderer);

    // Then we simulate the system applying a navigationBar window inset.
    WindowInsets navigationBarwindowInsets =
        new WindowInsets.Builder()
            .setInsets(android.view.WindowInsets.Type.systemOverlays(), Insets.of(0, 0, 0, 10))
            .setInsets(android.view.WindowInsets.Type.navigationBars(), Insets.of(0, 0, 0, 50))
            .build();
    flutterView.onApplyWindowInsets(navigationBarwindowInsets);

    // Verify.
    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Confirm that the navigationBar inset is used because it is the largest of the insets faked.
    validateViewportMetricPadding(viewportMetricsCaptor, 0, 0, 0, 50);
    clearInvocations(flutterRenderer);

    // Then we simulate the system applying a captionBar window inset.
    WindowInsets captionBarwindowInsets =
        new WindowInsets.Builder()
            .setInsets(android.view.WindowInsets.Type.statusBars(), Insets.of(0, 20, 0, 0))
            .setInsets(android.view.WindowInsets.Type.captionBar(), Insets.of(0, 60, 0, 0))
            .build();
    flutterView.onApplyWindowInsets(captionBarwindowInsets);
    // Verify.
    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Confirm that the captionBar inset is used because it is the largest of the insets faked.
    validateViewportMetricPadding(viewportMetricsCaptor, 0, 60, 0, 0);
    clearInvocations(flutterRenderer);

    // Then we simulate the system applying a systemOverlay window inset.
    WindowInsets systemOverlayWindowInsets =
        new WindowInsets.Builder()
            .setInsets(android.view.WindowInsets.Type.statusBars(), Insets.of(0, 100, 0, 0))
            .setInsets(android.view.WindowInsets.Type.systemOverlays(), Insets.of(0, 200, 0, 0))
            .build();
    flutterView.onApplyWindowInsets(systemOverlayWindowInsets);
    // Verify.
    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Confirm that the systemOverlay inset is used because it is the largest of the insets faked.
    validateViewportMetricPadding(viewportMetricsCaptor, 0, 200, 0, 0);
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility
  // This test uses the pre-API 30 Algorithm for window insets.
  @Test
  @TargetApi(28)
  @Config(sdk = 28)
  public void reportSystemInsetWhenNotFullscreenLegacy() {
    // Without custom shadows, the default system ui visibility flags is 0.
    FlutterView flutterView = new FlutterView(ctx);
    assertEquals(0, flutterView.getSystemUiVisibility());

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets, the viewport
    // metrics
    // default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    mockSystemWindowInsets(windowInsets, 100, 100, 100, 100);
    flutterView.onApplyWindowInsets(windowInsets);

    // Verify.
    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Top padding is reported as-is.
    validateViewportMetricPadding(viewportMetricsCaptor, 100, 100, 100, 0);
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility, getWindowSystemUiVisibility, required to test pre api 30 behavior.
  @Test
  @Config(minSdk = 23, maxSdk = 29, qualifiers = "land")
  public void systemInsetHandlesFullscreenNavbarRight() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_90);
    assertEquals(0, flutterView.getSystemUiVisibility());
    when(flutterView.getWindowSystemUiVisibility())
        .thenReturn(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION);
    when(flutterView.getContext()).thenReturn(ctx);

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets,
    // the viewport metrics default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    mockSystemWindowInsets(windowInsets, 100, 100, 100, 100);
    mockSystemGestureInsetsIfNeed(windowInsets);

    flutterView.onApplyWindowInsets(windowInsets);

    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Top padding is removed due to full screen.
    // Right padding is zero because the rotation is 90deg
    // Bottom padding is removed due to hide navigation.
    validateViewportMetricPadding(viewportMetricsCaptor, 100, 0, 0, 0);
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility, getWindowSystemUiVisibility required to test pre api 30 behavior.
  @Test
  @Config(minSdk = 20, maxSdk = 22, qualifiers = "land")
  public void systemInsetHandlesFullscreenNavbarRightBelowSDK23() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_270);
    assertEquals(0, flutterView.getSystemUiVisibility());
    when(flutterView.getWindowSystemUiVisibility())
        .thenReturn(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION);
    when(flutterView.getContext()).thenReturn(ctx);

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets,
    // the viewport metrics default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    mockSystemWindowInsets(windowInsets, 100, 100, 100, 100);
    mockSystemGestureInsetsIfNeed(windowInsets);

    flutterView.onApplyWindowInsets(windowInsets);

    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Top padding is removed due to full screen.
    // Right padding is zero because the rotation is 270deg under SDK 23
    // Bottom padding is removed due to hide navigation.
    validateViewportMetricPadding(viewportMetricsCaptor, 100, 0, 0, 0);
  }

  @Test
  @Config(minSdk = API_LEVELS.FLUTTER_MIN, maxSdk = API_LEVELS.API_29, qualifiers = "port")
  public void calculateShouldZeroSidesInPortrait() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    assertEquals(FlutterView.ZeroSides.NONE, flutterView.calculateShouldZeroSides());
  }

  @Test
  @Config(minSdk = API_LEVELS.FLUTTER_MIN, maxSdk = API_LEVELS.API_29, qualifiers = "land")
  public void calculateShouldZeroSidesInLandscapeNeutralRotation() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_0);
    assertEquals(FlutterView.ZeroSides.BOTH, flutterView.calculateShouldZeroSides());

    setExpectedDisplayRotation(Surface.ROTATION_180);
    assertEquals(FlutterView.ZeroSides.BOTH, flutterView.calculateShouldZeroSides());
  }

  @Test
  @Config(minSdk = API_LEVELS.FLUTTER_MIN, maxSdk = API_LEVELS.API_29, qualifiers = "land")
  public void calculateShouldZeroSidesInLandscapeRotation90() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_90);
    assertEquals(FlutterView.ZeroSides.RIGHT, flutterView.calculateShouldZeroSides());
  }

  @Test
  @Config(minSdk = API_LEVELS.API_21, maxSdk = API_LEVELS.API_22, qualifiers = "land")
  public void calculateShouldZeroSidesInLandscapeRotation270API22() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_270);
    assertEquals(FlutterView.ZeroSides.RIGHT, flutterView.calculateShouldZeroSides());
  }

  @Test
  @Config(minSdk = API_LEVELS.API_23, maxSdk = API_LEVELS.API_29, qualifiers = "land")
  public void calculateShouldZeroSidesInLandscapeRotation270API23Plus() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_270);
    assertEquals(FlutterView.ZeroSides.LEFT, flutterView.calculateShouldZeroSides());
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility, getWindowSystemUiVisibility required to test pre api 30 behavior.
  @Test
  @Config(minSdk = 23, maxSdk = 29, qualifiers = "land")
  public void systemInsetHandlesFullscreenNavbarLeft() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_270);
    assertEquals(0, flutterView.getSystemUiVisibility());
    when(flutterView.getWindowSystemUiVisibility())
        .thenReturn(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION);
    when(flutterView.getContext()).thenReturn(ctx);

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets,
    // the viewport metrics default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    mockSystemWindowInsets(windowInsets, 100, 100, 100, 100);
    mockSystemGestureInsetsIfNeed(windowInsets);

    flutterView.onApplyWindowInsets(windowInsets);

    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Left padding is zero because the rotation is 270deg
    // Top padding is removed due to full screen.
    // Bottom padding is removed due to hide navigation.
    validateViewportMetricPadding(viewportMetricsCaptor, 0, 0, 100, 0);
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility, getWindowSystemUiVisibility required to test interop with api 30
  // behavior.
  // This test uses the API 30+ Algorithm for window insets. The legacy algorithm is
  // set to -1 values, so it is clear if the wrong algorithm is used.
  @Test
  @TargetApi(30)
  @Config(sdk = 30, qualifiers = "land")
  public void systemInsetGetInsetsFullscreen() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_270);
    assertEquals(0, flutterView.getSystemUiVisibility());
    when(flutterView.getWindowSystemUiVisibility())
        .thenReturn(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION);
    when(flutterView.getContext()).thenReturn(ctx);

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets,
    // the viewport metrics default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    Insets insets = Insets.of(10, 20, 30, 40);
    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    mockSystemWindowInsets(windowInsets, -1, -1, -1, -1);
    when(windowInsets.getInsets(anyInt())).thenReturn(insets);

    flutterView.onApplyWindowInsets(windowInsets);

    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    validateViewportMetricPadding(viewportMetricsCaptor, 10, 20, 30, 40);
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility, getWindowSystemUiVisibility required to test pre api 30 behavior.
  // This test uses the pre-API 30 Algorithm for window insets.
  @Test
  @TargetApi(28)
  @Config(sdk = 28, qualifiers = "land")
  public void systemInsetGetInsetsFullscreenLegacy() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    setExpectedDisplayRotation(Surface.ROTATION_270);
    assertEquals(0, flutterView.getSystemUiVisibility());
    when(flutterView.getWindowSystemUiVisibility())
        .thenReturn(View.SYSTEM_UI_FLAG_FULLSCREEN | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION);
    when(flutterView.getContext()).thenReturn(ctx);

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets,
    // the viewport metrics default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    mockSystemWindowInsets(windowInsets, 102, 100, 103, 101);

    flutterView.onApplyWindowInsets(windowInsets);

    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Left padding is zero because the rotation is 270deg
    // Top padding is removed due to full screen.
    // Bottom padding is removed due to hide navigation.
    validateViewportMetricPadding(viewportMetricsCaptor, 0, 0, 103, 0);
  }

  @SuppressWarnings("deprecation")
  // getSystemUiVisibility, getWindowSystemUiVisibility, getSystemGestureInsets required
  // to test pre api 30 interop behavior.
  // This test uses the API 30+ Algorithm for window insets. The legacy algorithm is
  // set to -1 values, so it is clear if the wrong algorithm is used.
  @Test
  @TargetApi(30)
  @Config(sdk = 30, qualifiers = "land")
  public void systemInsetDisplayCutoutSimple() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    assertEquals(0, flutterView.getSystemUiVisibility());
    when(flutterView.getWindowSystemUiVisibility()).thenReturn(0);
    when(flutterView.getContext()).thenReturn(ctx);

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets,
    // the viewport metrics default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    Insets insets = Insets.of(100, 100, 100, 100);
    Insets systemGestureInsets = Insets.of(110, 110, 110, 110);
    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    DisplayCutout displayCutout = mock(DisplayCutout.class);
    mockSystemWindowInsets(windowInsets, -1, -1, -1, -1);
    when(windowInsets.getInsets(anyInt())).thenReturn(insets);
    when(windowInsets.getSystemGestureInsets()).thenReturn(systemGestureInsets);
    when(windowInsets.getDisplayCutout()).thenReturn(displayCutout);

    Insets waterfallInsets = Insets.of(200, 0, 250, 0);
    when(displayCutout.getWaterfallInsets()).thenReturn(waterfallInsets);
    when(displayCutout.getSafeInsetLeft()).thenReturn(110);
    when(displayCutout.getSafeInsetTop()).thenReturn(120);
    when(displayCutout.getSafeInsetRight()).thenReturn(130);
    when(displayCutout.getSafeInsetBottom()).thenReturn(140);

    flutterView.onApplyWindowInsets(windowInsets);

    verify(flutterRenderer, times(2)).setViewportMetrics(viewportMetricsCaptor.capture());
    // Each dimension of the viewport metric paddings should be the maximum of the corresponding
    // dimension from the display cutout's safe insets and waterfall insets.
    validateViewportMetricPadding(viewportMetricsCaptor, 200, 120, 250, 140);

    assertEquals(100, viewportMetricsCaptor.getValue().viewInsetTop);
  }

  @SuppressWarnings("deprecation")
  @Test
  @Config(minSdk = 28)
  public void onApplyWindowInsetsSetsDisplayCutouts() {
    // Use an Activity context so that FlutterView.onAttachedToWindow completes.
    Context context = Robolectric.setupActivity(Activity.class);
    FlutterView flutterView = spy(new FlutterView(context));
    assertEquals(0, flutterView.getSystemUiVisibility());
    when(flutterView.getWindowSystemUiVisibility()).thenReturn(0);
    when(flutterView.getContext()).thenReturn(context);

    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets,
    // the viewport metrics default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Capture flutterView.setWindowInfoListenerDisplayFeatures.
    WindowInfoRepositoryCallbackAdapterWrapper windowInfoRepo =
        mock(WindowInfoRepositoryCallbackAdapterWrapper.class);
    doReturn(windowInfoRepo).when(flutterView).createWindowInfoRepo();
    ArgumentCaptor<Consumer<WindowLayoutInfo>> consumerCaptor =
        ArgumentCaptor.forClass(Consumer.class);
    flutterView.onAttachedToWindow();
    verify(windowInfoRepo).addWindowLayoutInfoListener(any(), any(), consumerCaptor.capture());
    Consumer<WindowLayoutInfo> consumer = consumerCaptor.getValue();

    // Set display features in flutterView to ensure they are not overridden by display cutouts.
    FoldingFeature displayFeature = mock(FoldingFeature.class);
    Rect featureBounds = new Rect(10, 20, 30, 40);
    when(displayFeature.getBounds()).thenReturn(featureBounds);
    when(displayFeature.getOcclusionType()).thenReturn(FoldingFeature.OcclusionType.FULL);
    when(displayFeature.getState()).thenReturn(FoldingFeature.State.FLAT);
    WindowLayoutInfo windowLayout = new WindowLayoutInfo(Collections.singletonList(displayFeature));
    clearInvocations(flutterRenderer);
    consumer.accept(windowLayout);

    // Assert the display feature is set.
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    List<FlutterRenderer.DisplayFeature> features =
        viewportMetricsCaptor.getValue().getDisplayFeatures();
    assertEquals(1, features.size());
    assertEquals(FlutterRenderer.DisplayFeatureType.HINGE, features.get(0).type);
    assertEquals(FlutterRenderer.DisplayFeatureState.POSTURE_FLAT, features.get(0).state);
    assertEquals(featureBounds, features.get(0).bounds);

    // Then we simulate the system applying a window inset.
    List<Rect> cutoutBoundingRects =
        Arrays.asList(new Rect(0, 200, 300, 400), new Rect(150, 0, 300, 150));
    WindowInsets windowInsets = setupMockDisplayCutout(cutoutBoundingRects);

    clearInvocations(flutterRenderer);
    flutterView.onApplyWindowInsets(windowInsets);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());

    features = viewportMetricsCaptor.getValue().getDisplayFeatures();

    // Assert the old display feature is still present.
    assertEquals(1, features.size());
    assertEquals(FlutterRenderer.DisplayFeatureType.HINGE, features.get(0).type);
    assertEquals(FlutterRenderer.DisplayFeatureState.POSTURE_FLAT, features.get(0).state);
    assertEquals(featureBounds, features.get(0).bounds);

    List<FlutterRenderer.DisplayFeature> cutouts =
        viewportMetricsCaptor.getValue().getDisplayCutouts();
    // Asserts for display cutouts.
    assertEquals(2, cutouts.size());
    for (int i = 0; i < 2; i++) {
      assertEquals(cutoutBoundingRects.get(i), cutouts.get(i).bounds);
      assertEquals(FlutterRenderer.DisplayFeatureType.CUTOUT, cutouts.get(i).type);
      assertEquals(FlutterRenderer.DisplayFeatureState.UNKNOWN, cutouts.get(i).state);
    }
  }

  @SuppressWarnings("deprecation")
  // Robolectric.setupActivity
  // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
  // getDefaultDisplay
  // TODO(jesswrd): https://github.com/flutter/flutter/issues/99421
  @Test
  public void itRegistersAndUnregistersToWindowManager() {
    Context context = Robolectric.setupActivity(Activity.class);
    FlutterView flutterView = spy(new FlutterView(context));
    WindowInfoRepositoryCallbackAdapterWrapper windowInfoRepo =
        mock(WindowInfoRepositoryCallbackAdapterWrapper.class);
    // For reasoning behing using doReturn instead of when, read "Important gotcha" at
    // https://www.javadoc.io/doc/org.mockito/mockito-core/1.10.19/org/mockito/Mockito.html#13
    doReturn(windowInfoRepo).when(flutterView).createWindowInfoRepo();

    // When a new FlutterView is attached to the window
    flutterView.onAttachedToWindow();

    // Then the WindowManager callback is registered
    verify(windowInfoRepo, times(1)).addWindowLayoutInfoListener(any(), any(), any());

    // When the FlutterView is detached from the window
    flutterView.onDetachedFromWindow();

    // Then the WindowManager callback is unregistered
    verify(windowInfoRepo, times(1)).removeWindowLayoutInfoListener(any());
  }

  @SuppressWarnings("deprecation")
  // Robolectric.setupActivity
  // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
  // getDefaultDisplay
  // TODO(jesswrd): https://github.com/flutter/flutter/issues/99421
  @Test
  public void itSendsHingeDisplayFeatureToFlutter() {
    Context context = Robolectric.setupActivity(Activity.class);
    FlutterView flutterView = spy(new FlutterView(context));
    when(flutterView.getContext()).thenReturn(context);
    WindowInfoRepositoryCallbackAdapterWrapper windowInfoRepo =
        mock(WindowInfoRepositoryCallbackAdapterWrapper.class);
    doReturn(windowInfoRepo).when(flutterView).createWindowInfoRepo();
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // Display features should be empty on attaching to engine.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(Collections.emptyList(), viewportMetricsCaptor.getValue().getDisplayFeatures());
    clearInvocations(flutterRenderer);

    // Test that display features do not override cutouts.
    List<Rect> cutoutBoundingRects = Collections.singletonList(new Rect(0, 200, 300, 400));
    WindowInsets windowInsets = setupMockDisplayCutout(cutoutBoundingRects);
    flutterView.onApplyWindowInsets(windowInsets);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(1, viewportMetricsCaptor.getValue().getDisplayCutouts().size());
    assertEquals(
        cutoutBoundingRects.get(0),
        viewportMetricsCaptor.getValue().getDisplayCutouts().get(0).bounds);
    clearInvocations(flutterRenderer);

    FoldingFeature displayFeature = mock(FoldingFeature.class);
    Rect featureRect = new Rect(0, 0, 100, 100);
    when(displayFeature.getBounds()).thenReturn(featureRect);
    when(displayFeature.getOcclusionType()).thenReturn(FoldingFeature.OcclusionType.FULL);
    when(displayFeature.getState()).thenReturn(FoldingFeature.State.FLAT);

    WindowLayoutInfo testWindowLayout =
        new WindowLayoutInfo(Collections.singletonList(displayFeature));

    // When FlutterView is attached to the engine and window, and a hinge display feature exists
    flutterView.onAttachedToWindow();
    ArgumentCaptor<Consumer<WindowLayoutInfo>> wmConsumerCaptor =
        ArgumentCaptor.forClass(Consumer.class);
    verify(windowInfoRepo).addWindowLayoutInfoListener(any(), any(), wmConsumerCaptor.capture());
    Consumer<WindowLayoutInfo> wmConsumer = wmConsumerCaptor.getValue();
    clearInvocations(flutterRenderer);
    wmConsumer.accept(testWindowLayout);

    // Then the Renderer receives the display feature
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(1, viewportMetricsCaptor.getValue().getDisplayFeatures().size());
    FlutterRenderer.DisplayFeature feature =
        viewportMetricsCaptor.getValue().getDisplayFeatures().get(0);
    assertEquals(FlutterRenderer.DisplayFeatureType.HINGE, feature.type);
    assertEquals(FlutterRenderer.DisplayFeatureState.POSTURE_FLAT, feature.state);
    assertEquals(featureRect, feature.bounds);

    // Assert the display cutout is unaffected.
    assertEquals(1, viewportMetricsCaptor.getValue().getDisplayCutouts().size());
    FlutterRenderer.DisplayFeature cutout =
        viewportMetricsCaptor.getValue().getDisplayCutouts().get(0);
    assertEquals(cutoutBoundingRects.get(0), cutout.bounds);
    assertEquals(FlutterRenderer.DisplayFeatureType.CUTOUT, cutout.type);
    assertEquals(FlutterRenderer.DisplayFeatureState.UNKNOWN, cutout.state);
  }

  @Test
  public void flutterImageView_acquiresImageAndInvalidates() {
    final ImageReader mockReader = mock(ImageReader.class);
    when(mockReader.getMaxImages()).thenReturn(2);

    final FlutterImageView imageView =
        spy(new FlutterImageView(ctx, mockReader, FlutterImageView.SurfaceKind.background));

    final FlutterJNI jni = mock(FlutterJNI.class);
    imageView.attachToRenderer(new FlutterRenderer(jni));

    final Image mockImage = mock(Image.class);
    when(mockReader.acquireLatestImage()).thenReturn(mockImage);

    assertTrue(imageView.acquireLatestImage());
    verify(mockReader, times(1)).acquireLatestImage();
    verify(imageView, times(1)).invalidate();
  }

  @Test
  @SuppressLint("WrongCall") /*View#onDraw*/
  public void flutterImageView_acquiresImageClosesPreviousImageUnlessNoNewImage() {
    final ImageReader mockReader = mock(ImageReader.class);
    when(mockReader.getMaxImages()).thenReturn(3);

    final Image mockImage = mock(Image.class);
    when(mockImage.getPlanes()).thenReturn(new Plane[0]);
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
      final HardwareBuffer mockHardwareBuffer = mock(HardwareBuffer.class);
      when(mockHardwareBuffer.getUsage()).thenReturn(HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE);
      when(mockImage.getHardwareBuffer()).thenReturn(mockHardwareBuffer);
    }
    // Mock no latest image on the second time
    when(mockReader.acquireLatestImage())
        .thenReturn(mockImage)
        .thenReturn(null)
        .thenReturn(mockImage);

    final FlutterImageView imageView =
        spy(new FlutterImageView(ctx, mockReader, FlutterImageView.SurfaceKind.background));

    final FlutterJNI jni = mock(FlutterJNI.class);
    imageView.attachToRenderer(new FlutterRenderer(jni));
    doNothing().when(imageView).invalidate();

    assertTrue(imageView.acquireLatestImage()); // No previous, acquire latest image
    assertFalse(
        imageView.acquireLatestImage()); // Mock no image when acquire, don't close, and assertFalse
    assertTrue(imageView.acquireLatestImage()); // Acquire latest image and close previous
    assertTrue(imageView.acquireLatestImage()); // Acquire latest image and close previous
    assertTrue(imageView.acquireLatestImage()); // Acquire latest image and close previous
    verify(mockImage, times(3)).close(); // Close 3 times

    imageView.onDraw(mock(Canvas.class)); // Draw latest image

    assertTrue(imageView.acquireLatestImage()); // acquire latest image and close previous

    imageView.onDraw(mock(Canvas.class)); // Draw latest image
    imageView.onDraw(mock(Canvas.class)); // Draw latest image
    imageView.onDraw(mock(Canvas.class)); // Draw latest image

    verify(mockReader, times(6)).acquireLatestImage();
  }

  @Test
  public void flutterImageView_detachFromRendererClosesPreviousImage() {
    final ImageReader mockReader = mock(ImageReader.class);
    when(mockReader.getMaxImages()).thenReturn(2);

    final Image mockImage = mock(Image.class);
    when(mockReader.acquireLatestImage()).thenReturn(mockImage);

    final FlutterImageView imageView =
        spy(new FlutterImageView(ctx, mockReader, FlutterImageView.SurfaceKind.background));

    final FlutterJNI jni = mock(FlutterJNI.class);
    imageView.attachToRenderer(new FlutterRenderer(jni));

    doNothing().when(imageView).invalidate();
    imageView.acquireLatestImage();
    imageView.acquireLatestImage();
    verify(mockImage, times(1)).close();

    imageView.detachFromRenderer();
    // There's an acquireLatestImage() in detachFromRenderer(),
    // so it will be 2 times called close() inside detachFromRenderer()
    verify(mockImage, times(3)).close();
  }

  @Test
  public void flutterImageView_workaroundWithOnePixelWhenResizeWithZero() {
    final ImageReader mockReader = mock(ImageReader.class);
    when(mockReader.getMaxImages()).thenReturn(2);

    final FlutterImageView imageView =
        spy(new FlutterImageView(ctx, mockReader, FlutterImageView.SurfaceKind.background));

    final FlutterJNI jni = mock(FlutterJNI.class);
    imageView.attachToRenderer(new FlutterRenderer(jni));

    final Image mockImage = mock(Image.class);
    when(mockReader.acquireLatestImage()).thenReturn(mockImage);

    final int incorrectWidth = 0;
    final int incorrectHeight = -100;
    imageView.resizeIfNeeded(incorrectWidth, incorrectHeight);
    assertEquals(1, imageView.getImageReader().getWidth());
    assertEquals(1, imageView.getImageReader().getHeight());
  }

  @Test
  public void flutterImageView_closesReader() {
    final ImageReader mockReader = mock(ImageReader.class);
    when(mockReader.getMaxImages()).thenReturn(1);

    final FlutterImageView imageView =
        spy(new FlutterImageView(ctx, mockReader, FlutterImageView.SurfaceKind.background));

    imageView.closeImageReader();
    verify(mockReader, times(1)).close();
  }

  @Test
  public void flutterSurfaceView_GathersTransparentRegion() {
    final Region mockRegion = mock(Region.class);
    final FlutterSurfaceView surfaceView = new FlutterSurfaceView(ctx);

    surfaceView.setAlpha(0.0f);
    assertFalse(surfaceView.gatherTransparentRegion(mockRegion));
    verify(mockRegion, times(0)).op(anyInt(), anyInt(), anyInt(), anyInt(), any());

    surfaceView.setAlpha(1.0f);
    assertTrue(surfaceView.gatherTransparentRegion(mockRegion));
    verify(mockRegion, times(1)).op(0, 0, 0, 0, Region.Op.DIFFERENCE);
  }

  @Test
  @SuppressLint("PrivateApi")
  @Config(sdk = Build.VERSION_CODES.P)
  public void findViewByAccessibilityIdTraversal_returnsRootViewOnAndroid28() throws Exception {
    FlutterView flutterView = new FlutterView(ctx);

    Method getAccessibilityViewIdMethod = View.class.getDeclaredMethod("getAccessibilityViewId");
    Integer accessibilityViewId = (Integer) getAccessibilityViewIdMethod.invoke(flutterView);

    assertEquals(flutterView, flutterView.findViewByAccessibilityIdTraversal(accessibilityViewId));
  }

  @Test
  @Config(sdk = Build.VERSION_CODES.P)
  @SuppressLint("PrivateApi")
  public void findViewByAccessibilityIdTraversal_returnsChildViewOnAndroid28() throws Exception {
    FlutterView flutterView = new FlutterView(ctx);
    FrameLayout childView1 = new FrameLayout(ctx);
    flutterView.addView(childView1);

    FrameLayout childView2 = new FrameLayout(ctx);
    childView1.addView(childView2);

    Method getAccessibilityViewIdMethod = View.class.getDeclaredMethod("getAccessibilityViewId");
    Integer accessibilityViewId = (Integer) getAccessibilityViewIdMethod.invoke(childView2);

    assertEquals(childView2, flutterView.findViewByAccessibilityIdTraversal(accessibilityViewId));
  }

  @Test
  @Config(sdk = Build.VERSION_CODES.Q)
  @SuppressLint("PrivateApi")
  public void findViewByAccessibilityIdTraversal_returnsRootViewOnAndroid29() throws Exception {
    FlutterView flutterView = new FlutterView(ctx);

    Method getAccessibilityViewIdMethod = View.class.getDeclaredMethod("getAccessibilityViewId");
    Integer accessibilityViewId = (Integer) getAccessibilityViewIdMethod.invoke(flutterView);

    assertEquals(null, flutterView.findViewByAccessibilityIdTraversal(accessibilityViewId));
  }

  @Test
  public void itSendsTextShowPasswordToFrameworkOnAttach() {
    // Setup test.
    AtomicReference<Boolean> reportedShowPassword = new AtomicReference<>();

    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    Settings.System.putInt(
        flutterView.getContext().getContentResolver(), Settings.System.TEXT_SHOW_PASSWORD, 1);

    SettingsChannel fakeSettingsChannel = mock(SettingsChannel.class);
    SettingsChannel.MessageBuilder fakeMessageBuilder = mock(SettingsChannel.MessageBuilder.class);
    when(fakeMessageBuilder.setTextScaleFactor(any(Float.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setDisplayMetrics(any(DisplayMetrics.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setNativeSpellCheckServiceDefined(any(Boolean.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setPlatformBrightness(any(SettingsChannel.PlatformBrightness.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setUse24HourFormat(any(Boolean.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setBrieflyShowPassword(any(Boolean.class)))
        .thenAnswer(
            new Answer<SettingsChannel.MessageBuilder>() {
              @Override
              public SettingsChannel.MessageBuilder answer(InvocationOnMock invocation)
                  throws Throwable {
                reportedShowPassword.set((Boolean) invocation.getArguments()[0]);
                return fakeMessageBuilder;
              }
            });
    when(fakeSettingsChannel.startMessage()).thenReturn(fakeMessageBuilder);
    when(flutterEngine.getSettingsChannel()).thenReturn(fakeSettingsChannel);

    flutterView.attachToFlutterEngine(flutterEngine);

    // Verify results.
    assertTrue(reportedShowPassword.get());
  }

  public void ViewportMetrics_initializedPhysicalTouchSlop() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());

    assertFalse(-1 == viewportMetricsCaptor.getValue().physicalTouchSlop);
  }

  // This test uses the API 30+ Algorithm for window insets. The legacy algorithm is
  // set to -1 values, so it is clear if the wrong algorithm is used.
  @Test
  @TargetApi(30)
  @Config(sdk = 30)
  public void setPaddingTopToZeroForFullscreenMode() {
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets, the viewport
    // metrics
    // default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);

    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets =
        new WindowInsets.Builder()
            .setInsets(
                android.view.WindowInsets.Type.navigationBars()
                    | android.view.WindowInsets.Type.systemBars(),
                Insets.of(100, 100, 100, 100))
            .build();
    flutterView.onApplyWindowInsets(windowInsets);

    // Verify.
    verify(flutterRenderer, times(3)).setViewportMetrics(viewportMetricsCaptor.capture());
    validateViewportMetricPadding(viewportMetricsCaptor, 100, 100, 100, 100);
  }

  @SuppressWarnings("deprecation")
  // SYSTEM_UI_FLAG_FULLSCREEN, getWindowSystemUiVisibility
  // This test uses the pre-API 30 Algorithm for window insets.
  @Test
  @TargetApi(28)
  @Config(
      sdk = 28,
      shadows = {
        FlutterViewTest.ShadowFullscreenView.class,
      })
  public void setPaddingTopToZeroForFullscreenModeLegacy() {
    FlutterView flutterView = spy(new FlutterView(ctx));
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));
    FlutterRenderer flutterRenderer = spy(new FlutterRenderer(mockFlutterJni));
    when(flutterEngine.getRenderer()).thenReturn(flutterRenderer);

    // When we attach a new FlutterView to the engine without any system insets, the viewport
    // metrics
    // default to 0.
    flutterView.attachToFlutterEngine(flutterEngine);
    ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor =
        ArgumentCaptor.forClass(FlutterRenderer.ViewportMetrics.class);
    verify(flutterRenderer).setViewportMetrics(viewportMetricsCaptor.capture());
    assertEquals(0, viewportMetricsCaptor.getValue().viewPaddingTop);
    clearInvocations(flutterRenderer);
    // Then we simulate the system applying a window inset.
    WindowInsets windowInsets = mock(WindowInsets.class);
    mockSystemWindowInsets(windowInsets, 100, 100, 100, 100);
    flutterView.onApplyWindowInsets(windowInsets);

    // Verify.
    verify(flutterRenderer, times(1)).setViewportMetrics(viewportMetricsCaptor.capture());
    validateViewportMetricPadding(viewportMetricsCaptor, 100, 100, 100, 0);
    clearInvocations(flutterRenderer);

    // Validation when fullscreen
    when(flutterView.getWindowSystemUiVisibility()).thenReturn(View.SYSTEM_UI_FLAG_FULLSCREEN);

    // Then we simulate the system applying a window inset.
    mockSystemWindowInsets(windowInsets, 100, 100, 100, 100);
    flutterView.onApplyWindowInsets(windowInsets);

    // Verify.
    verify(flutterRenderer, times(1)).setViewportMetrics(viewportMetricsCaptor.capture());
    validateViewportMetricPadding(viewportMetricsCaptor, 100, 0, 100, 0);
  }

  // TODO(mattcarroll): turn this into an e2e test. GitHub #42990
  @Test
  public void itSendsDarkPlatformBrightnessToFlutter() {
    // Setup test.
    AtomicReference<SettingsChannel.PlatformBrightness> reportedBrightness =
        new AtomicReference<>();

    Context spiedContext = spy(ctx);

    Resources spiedResources = spy(spiedContext.getResources());
    when(spiedContext.getResources()).thenReturn(spiedResources);

    Configuration spiedConfiguration = spy(spiedResources.getConfiguration());
    spiedConfiguration.uiMode =
        (spiedResources.getConfiguration().uiMode | Configuration.UI_MODE_NIGHT_YES)
            & ~Configuration.UI_MODE_NIGHT_NO;
    when(spiedResources.getConfiguration()).thenReturn(spiedConfiguration);

    FlutterView flutterView = new FlutterView(spiedContext);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));

    SettingsChannel fakeSettingsChannel = mock(SettingsChannel.class);
    SettingsChannel.MessageBuilder fakeMessageBuilder = mock(SettingsChannel.MessageBuilder.class);
    when(fakeMessageBuilder.setTextScaleFactor(any(Float.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setDisplayMetrics(any(DisplayMetrics.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setNativeSpellCheckServiceDefined(any(Boolean.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setBrieflyShowPassword(any(Boolean.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setUse24HourFormat(any(Boolean.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setPlatformBrightness(any(SettingsChannel.PlatformBrightness.class)))
        .thenAnswer(
            new Answer<SettingsChannel.MessageBuilder>() {
              @Override
              public SettingsChannel.MessageBuilder answer(InvocationOnMock invocation)
                  throws Throwable {
                reportedBrightness.set(
                    (SettingsChannel.PlatformBrightness) invocation.getArguments()[0]);
                return fakeMessageBuilder;
              }
            });
    when(fakeSettingsChannel.startMessage()).thenReturn(fakeMessageBuilder);
    when(flutterEngine.getSettingsChannel()).thenReturn(fakeSettingsChannel);

    // Execute behavior under test.
    flutterView.attachToFlutterEngine(flutterEngine);
    flutterView.sendUserSettingsToFlutter();

    // Verify results.
    assertEquals(SettingsChannel.PlatformBrightness.dark, reportedBrightness.get());
  }

  @SuppressWarnings("deprecation")
  // SYSTEM_UI_FLAG_FULLSCREEN, getWindowSystemUiVisibility
  // TODO(mattcarroll): turn this into an e2e test. GitHub #42990
  @Test
  public void itSendsLightPlatformBrightnessToFlutter() {
    // Setup test.
    AtomicReference<SettingsChannel.PlatformBrightness> reportedBrightness =
        new AtomicReference<>();

    // FYI - The default brightness is LIGHT, which is why we don't need to configure it.
    FlutterView flutterView = new FlutterView(ctx);
    FlutterEngine flutterEngine = spy(new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJni));

    SettingsChannel fakeSettingsChannel = mock(SettingsChannel.class);
    SettingsChannel.MessageBuilder fakeMessageBuilder = mock(SettingsChannel.MessageBuilder.class);
    when(fakeMessageBuilder.setTextScaleFactor(any(Float.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setDisplayMetrics(any(DisplayMetrics.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setNativeSpellCheckServiceDefined(any(Boolean.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setBrieflyShowPassword(any(Boolean.class)))
        .thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setUse24HourFormat(any(Boolean.class))).thenReturn(fakeMessageBuilder);
    when(fakeMessageBuilder.setPlatformBrightness(any(SettingsChannel.PlatformBrightness.class)))
        .thenAnswer(
            new Answer<SettingsChannel.MessageBuilder>() {
              @Override
              public SettingsChannel.MessageBuilder answer(InvocationOnMock invocation)
                  throws Throwable {
                reportedBrightness.set(
                    (SettingsChannel.PlatformBrightness) invocation.getArguments()[0]);
                return fakeMessageBuilder;
              }
            });
    when(fakeSettingsChannel.startMessage()).thenReturn(fakeMessageBuilder);
    when(flutterEngine.getSettingsChannel()).thenReturn(fakeSettingsChannel);

    flutterView.attachToFlutterEngine(flutterEngine);

    // Execute behavior under test.
    flutterView.sendUserSettingsToFlutter();

    // Verify results.
    assertEquals(SettingsChannel.PlatformBrightness.light, reportedBrightness.get());
  }

  @SuppressWarnings("deprecation")
  private void setExpectedDisplayRotation(int rotation) {
    ShadowDisplay myDisplay =
        Shadows.shadowOf(
            ((DisplayManager) ctx.getSystemService(Context.DISPLAY_SERVICE)).getDisplay(0));
    myDisplay.setRotation(rotation);
  }

  private void validateViewportMetricPadding(
      ArgumentCaptor<FlutterRenderer.ViewportMetrics> viewportMetricsCaptor,
      int left,
      int top,
      int right,
      int bottom) {
    assertEquals(left, viewportMetricsCaptor.getValue().viewPaddingLeft);
    assertEquals(top, viewportMetricsCaptor.getValue().viewPaddingTop);
    assertEquals(right, viewportMetricsCaptor.getValue().viewPaddingRight);
    assertEquals(bottom, viewportMetricsCaptor.getValue().viewPaddingBottom);
  }

  @SuppressWarnings("deprecation")
  private void mockSystemWindowInsets(
      WindowInsets windowInsets, int left, int top, int right, int bottom) {
    when(windowInsets.getSystemWindowInsetLeft()).thenReturn(left);
    when(windowInsets.getSystemWindowInsetTop()).thenReturn(top);
    when(windowInsets.getSystemWindowInsetRight()).thenReturn(right);
    when(windowInsets.getSystemWindowInsetBottom()).thenReturn(bottom);
  }

  @SuppressWarnings("deprecation")
  private void mockSystemGestureInsetsIfNeed(WindowInsets windowInsets) {
    if (Build.VERSION.SDK_INT == Build.VERSION_CODES.Q) {
      when(windowInsets.getSystemGestureInsets()).thenReturn(Insets.NONE);
    }
  }

  @SuppressWarnings("deprecation")
  private WindowInsets setupMockDisplayCutout(List<Rect> boundingRects) {
    WindowInsets windowInsets = mock(WindowInsets.class);
    DisplayCutout displayCutout = mock(DisplayCutout.class);
    when(windowInsets.getDisplayCutout()).thenReturn(displayCutout);
    when(displayCutout.getBoundingRects()).thenReturn(boundingRects);
    // The following mocked methods are necessary to avoid a NullPointerException when calling
    // onApplyWindowInsets, but are irrelevant to the behavior this test concerns.
    Insets unusedInsets = Insets.of(100, 100, 100, 100);
    // WindowInsets::getSystemGestureInsets was added in API 29, deprecated in API 30.
    if (Build.VERSION.SDK_INT == 29) {
      when(windowInsets.getSystemGestureInsets()).thenReturn(unusedInsets);
    }
    // WindowInsets::getInsets was added in API 30.
    if (Build.VERSION.SDK_INT >= 30) {
      when(windowInsets.getInsets(anyInt())).thenReturn(unusedInsets);
    }
    // DisplayCutout::getWaterfallInsets was added in API 30.
    if (Build.VERSION.SDK_INT >= 30) {
      when(displayCutout.getWaterfallInsets()).thenReturn(unusedInsets);
    }
    when(displayCutout.getSafeInsetTop()).thenReturn(100);
    when(displayCutout.getSafeInsetLeft()).thenReturn(100);
    when(displayCutout.getSafeInsetBottom()).thenReturn(100);
    when(displayCutout.getSafeInsetRight()).thenReturn(100);
    return windowInsets;
  }

  /*
   * A custom shadow that reports fullscreen flag for system UI visibility
   */
  @Implements(FrameLayout.class)
  @SuppressWarnings("deprecation")
  public static class ShadowFullscreenView extends ShadowViewGroup {
    @Implementation
    public int getWindowSystemUiVisibility() {
      return View.SYSTEM_UI_FLAG_FULLSCREEN;
    }
  }
}
