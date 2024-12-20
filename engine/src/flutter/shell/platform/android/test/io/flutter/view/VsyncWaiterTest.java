// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.view;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.ArgumentMatchers.isNull;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;
import static org.robolectric.Shadows.shadowOf;

import android.hardware.display.DisplayManager;
import android.os.Looper;
import android.view.Display;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterJNI;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class VsyncWaiterTest {
  @Before
  public void setUp() {
    VsyncWaiter.reset();
  }

  @Test
  public void itSetsFpsBelowApi17() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    VsyncWaiter waiter = VsyncWaiter.getInstance(10.0f, mockFlutterJNI);
    verify(mockFlutterJNI, times(1)).setRefreshRateFPS(10.0f);

    waiter.init();

    ArgumentCaptor<FlutterJNI.AsyncWaitForVsyncDelegate> delegateCaptor =
        ArgumentCaptor.forClass(FlutterJNI.AsyncWaitForVsyncDelegate.class);
    verify(mockFlutterJNI, times(1)).setAsyncWaitForVsyncDelegate(delegateCaptor.capture());
    delegateCaptor.getValue().asyncWaitForVsync(1);
    shadowOf(Looper.getMainLooper()).idle();
    verify(mockFlutterJNI, times(1)).onVsync(anyLong(), eq(1000000000l / 10l), eq(1l));
  }

  @Test
  public void itSetsFpsWhenDisplayManagerUpdates() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DisplayManager mockDisplayManager = mock(DisplayManager.class);
    Display mockDisplay = mock(Display.class);
    ArgumentCaptor<VsyncWaiter.DisplayListener> displayListenerCaptor =
        ArgumentCaptor.forClass(VsyncWaiter.DisplayListener.class);
    when(mockDisplayManager.getDisplay(Display.DEFAULT_DISPLAY)).thenReturn(mockDisplay);

    VsyncWaiter waiter = VsyncWaiter.getInstance(mockDisplayManager, mockFlutterJNI);
    verify(mockDisplayManager, times(1))
        .registerDisplayListener(displayListenerCaptor.capture(), isNull());

    when(mockDisplay.getRefreshRate()).thenReturn(90.0f);
    displayListenerCaptor.getValue().onDisplayChanged(Display.DEFAULT_DISPLAY);
    verify(mockFlutterJNI, times(1)).setRefreshRateFPS(90.0f);

    waiter.init();

    ArgumentCaptor<FlutterJNI.AsyncWaitForVsyncDelegate> delegateCaptor =
        ArgumentCaptor.forClass(FlutterJNI.AsyncWaitForVsyncDelegate.class);
    verify(mockFlutterJNI, times(1)).setAsyncWaitForVsyncDelegate(delegateCaptor.capture());
    delegateCaptor.getValue().asyncWaitForVsync(1);
    shadowOf(Looper.getMainLooper()).idle();
    verify(mockFlutterJNI, times(1)).onVsync(anyLong(), eq(1000000000l / 90l), eq(1l));

    when(mockDisplay.getRefreshRate()).thenReturn(60.0f);
    displayListenerCaptor.getValue().onDisplayChanged(Display.DEFAULT_DISPLAY);
    verify(mockFlutterJNI, times(1)).setRefreshRateFPS(60.0f);

    delegateCaptor.getValue().asyncWaitForVsync(1);
    shadowOf(Looper.getMainLooper()).idle();
    verify(mockFlutterJNI, times(1)).onVsync(anyLong(), eq(1000000000l / 60l), eq(1l));
  }

  @Test
  public void itSetsFpsWhenDisplayManagerDoesNotUpdate() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    DisplayManager mockDisplayManager = mock(DisplayManager.class);
    Display mockDisplay = mock(Display.class);
    when(mockDisplayManager.getDisplay(Display.DEFAULT_DISPLAY)).thenReturn(mockDisplay);
    when(mockDisplay.getRefreshRate()).thenReturn(90.0f);

    VsyncWaiter waiter = VsyncWaiter.getInstance(mockDisplayManager, mockFlutterJNI);
    verify(mockDisplayManager, times(1)).registerDisplayListener(any(), isNull());

    verify(mockFlutterJNI, times(1)).setRefreshRateFPS(90.0f);
  }
}
