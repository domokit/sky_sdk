// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static android.view.View.OnFocusChangeListener;
import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.mockito.Mockito.spy;

import android.annotation.TargetApi;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.view.Surface;
import android.view.View;
import android.view.View.OnFocusChangeListener;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.view.accessibility.AccessibilityEvent;
import android.widget.FrameLayout;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.robolectric.annotation.Config;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;

@TargetApi(API_LEVELS.API_31)
@RunWith(AndroidJUnit4.class)
public class PlatformViewWrapperTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  @Test
  public void invalidateChildInParent_callsInvalidate() {
    final PlatformViewWrapper wrapper = spy(new PlatformViewWrapper(ctx));

    // Mock Android framework calls.
    wrapper.invalidateChildInParent(null, null);

    // Verify.
    verify(wrapper, times(1)).invalidate();
  }

  @Test
  public void draw_withoutSurface() {
    final PlatformViewWrapper wrapper =
        new PlatformViewWrapper(ctx) {
          @Override
          public void onDraw(Canvas canvas) {
            canvas.drawColor(Color.RED);
          }
        };
    // Test.
    final Canvas canvas = mock(Canvas.class);
    wrapper.draw(canvas);

    // Verify.
    verify(canvas, times(1)).drawColor(Color.RED);
  }

  @Test
  public void draw_withoutValidSurface() {
    FlutterRenderer.debugDisableSurfaceClear = true;
    final Surface surface = mock(Surface.class);
    when(surface.isValid()).thenReturn(false);
    final PlatformViewRenderTarget renderTarget = mock(PlatformViewRenderTarget.class);
    when(renderTarget.getSurface()).thenReturn(surface);

    final PlatformViewWrapper wrapper = new PlatformViewWrapper(ctx, renderTarget);
    final Canvas canvas = mock(Canvas.class);
    wrapper.draw(canvas);

    verify(canvas, times(0)).drawColor(Color.TRANSPARENT, android.graphics.PorterDuff.Mode.CLEAR);
  }

  @Test
  public void draw_withValidSurface() {
    FlutterRenderer.debugDisableSurfaceClear = true;
    final Canvas canvas = mock(Canvas.class);
    final Surface surface = mock(Surface.class);
    when(surface.isValid()).thenReturn(true);
    final PlatformViewRenderTarget renderTarget = mock(PlatformViewRenderTarget.class);
    when(renderTarget.getSurface()).thenReturn(surface);
    when(surface.lockHardwareCanvas()).thenReturn(canvas);
    final PlatformViewWrapper wrapper = new PlatformViewWrapper(ctx, renderTarget);

    wrapper.draw(canvas);

    verify(canvas, times(1)).drawColor(Color.TRANSPARENT, android.graphics.PorterDuff.Mode.CLEAR);
  }

  @Test
  public void focusChangeListener_hasFocus() {
    final ViewTreeObserver viewTreeObserver = mock(ViewTreeObserver.class);
    when(viewTreeObserver.isAlive()).thenReturn(true);

    final PlatformViewWrapper view =
        new PlatformViewWrapper(ctx) {
          @Override
          public ViewTreeObserver getViewTreeObserver() {
            return viewTreeObserver;
          }

          @Override
          public boolean hasFocus() {
            return true;
          }
        };

    final OnFocusChangeListener focusListener = mock(OnFocusChangeListener.class);
    view.setOnDescendantFocusChangeListener(focusListener);

    final ArgumentCaptor<ViewTreeObserver.OnGlobalFocusChangeListener> focusListenerCaptor =
        ArgumentCaptor.forClass(ViewTreeObserver.OnGlobalFocusChangeListener.class);
    verify(viewTreeObserver).addOnGlobalFocusChangeListener(focusListenerCaptor.capture());

    focusListenerCaptor.getValue().onGlobalFocusChanged(null, null);
    verify(focusListener).onFocusChange(view, true);
  }

  @Test
  public void focusChangeListener_doesNotHaveFocus() {
    final ViewTreeObserver viewTreeObserver = mock(ViewTreeObserver.class);
    when(viewTreeObserver.isAlive()).thenReturn(true);

    final PlatformViewWrapper view =
        new PlatformViewWrapper(ctx) {
          @Override
          public ViewTreeObserver getViewTreeObserver() {
            return viewTreeObserver;
          }

          @Override
          public boolean hasFocus() {
            return false;
          }
        };

    final OnFocusChangeListener focusListener = mock(OnFocusChangeListener.class);
    view.setOnDescendantFocusChangeListener(focusListener);

    final ArgumentCaptor<ViewTreeObserver.OnGlobalFocusChangeListener> focusListenerCaptor =
        ArgumentCaptor.forClass(ViewTreeObserver.OnGlobalFocusChangeListener.class);
    verify(viewTreeObserver).addOnGlobalFocusChangeListener(focusListenerCaptor.capture());

    focusListenerCaptor.getValue().onGlobalFocusChanged(null, null);
    verify(focusListener).onFocusChange(view, false);
  }

  @Test
  public void focusChangeListener_viewTreeObserverIsAliveFalseDoesNotThrow() {
    final PlatformViewWrapper view =
        new PlatformViewWrapper(ctx) {
          @Override
          public ViewTreeObserver getViewTreeObserver() {
            final ViewTreeObserver viewTreeObserver = mock(ViewTreeObserver.class);
            when(viewTreeObserver.isAlive()).thenReturn(false);
            return viewTreeObserver;
          }
        };
    view.setOnDescendantFocusChangeListener(mock(OnFocusChangeListener.class));
  }

  @Test
  public void setOnDescendantFocusChangeListener_keepsSingleListener() {
    final ViewTreeObserver viewTreeObserver = mock(ViewTreeObserver.class);
    when(viewTreeObserver.isAlive()).thenReturn(true);

    final PlatformViewWrapper view =
        new PlatformViewWrapper(ctx) {
          @Override
          public ViewTreeObserver getViewTreeObserver() {
            return viewTreeObserver;
          }
        };

    assertNull(view.getActiveFocusListener());

    view.setOnDescendantFocusChangeListener(mock(OnFocusChangeListener.class));
    assertNotNull(view.getActiveFocusListener());

    final ViewTreeObserver.OnGlobalFocusChangeListener activeFocusListener =
        view.getActiveFocusListener();

    view.setOnDescendantFocusChangeListener(mock(OnFocusChangeListener.class));
    assertNotNull(view.getActiveFocusListener());

    verify(viewTreeObserver, times(1)).removeOnGlobalFocusChangeListener(activeFocusListener);
  }

  @Test
  public void unsetOnDescendantFocusChangeListener_removesActiveListener() {
    final ViewTreeObserver viewTreeObserver = mock(ViewTreeObserver.class);
    when(viewTreeObserver.isAlive()).thenReturn(true);

    final PlatformViewWrapper view =
        new PlatformViewWrapper(ctx) {
          @Override
          public ViewTreeObserver getViewTreeObserver() {
            return viewTreeObserver;
          }
        };

    assertNull(view.getActiveFocusListener());

    view.setOnDescendantFocusChangeListener(mock(OnFocusChangeListener.class));
    assertNotNull(view.getActiveFocusListener());

    final ViewTreeObserver.OnGlobalFocusChangeListener activeFocusListener =
        view.getActiveFocusListener();

    view.unsetOnDescendantFocusChangeListener();
    assertNull(view.getActiveFocusListener());

    view.unsetOnDescendantFocusChangeListener();
    verify(viewTreeObserver, times(1)).removeOnGlobalFocusChangeListener(activeFocusListener);
  }

  @Test
  @Config(
      shadows = {
        ShadowFrameLayout.class,
        ShadowViewGroup.class,
      })
  public void ignoreAccessibilityEvents() {
    final PlatformViewWrapper wrapperView = new PlatformViewWrapper(ctx);

    final View embeddedView = mock(View.class);
    wrapperView.addView(embeddedView);

    when(embeddedView.getImportantForAccessibility())
        .thenReturn(View.IMPORTANT_FOR_ACCESSIBILITY_NO_HIDE_DESCENDANTS);
    final boolean eventSent =
        wrapperView.requestSendAccessibilityEvent(embeddedView, mock(AccessibilityEvent.class));
    assertFalse(eventSent);
  }

  @Test
  @Config(
      shadows = {
        ShadowFrameLayout.class,
        ShadowViewGroup.class,
      })
  public void sendAccessibilityEvents() {
    final PlatformViewWrapper wrapperView = new PlatformViewWrapper(ctx);

    final View embeddedView = mock(View.class);
    wrapperView.addView(embeddedView);

    when(embeddedView.getImportantForAccessibility())
        .thenReturn(View.IMPORTANT_FOR_ACCESSIBILITY_YES);
    boolean eventSent =
        wrapperView.requestSendAccessibilityEvent(embeddedView, mock(AccessibilityEvent.class));
    assertTrue(eventSent);

    when(embeddedView.getImportantForAccessibility())
        .thenReturn(View.IMPORTANT_FOR_ACCESSIBILITY_AUTO);
    eventSent =
        wrapperView.requestSendAccessibilityEvent(embeddedView, mock(AccessibilityEvent.class));
    assertTrue(eventSent);
  }

  @Implements(ViewGroup.class)
  public static class ShadowViewGroup extends org.robolectric.shadows.ShadowViewGroup {
    @Implementation
    protected boolean requestSendAccessibilityEvent(View child, AccessibilityEvent event) {
      return true;
    }
  }

  @Implements(FrameLayout.class)
  public static class ShadowFrameLayout
      extends io.flutter.plugin.platform.PlatformViewWrapperTest.ShadowViewGroup {}
}
