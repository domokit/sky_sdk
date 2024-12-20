// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.content.Context;
import android.os.SystemClock;
import android.view.MotionEvent;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
public class PlatformOverlayViewTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  @Test
  public void platformOverlayView_forwardsHover() {
    final AccessibilityEventsDelegate mockAccessibilityDelegate =
        mock(AccessibilityEventsDelegate.class);
    when(mockAccessibilityDelegate.onAccessibilityHoverEvent(any(), eq(true))).thenReturn(true);

    final int size = 10;
    final PlatformOverlayView imageView =
        new PlatformOverlayView(ctx, size, size, mockAccessibilityDelegate);
    MotionEvent event =
        MotionEvent.obtain(
            SystemClock.uptimeMillis(),
            SystemClock.uptimeMillis(),
            MotionEvent.ACTION_HOVER_MOVE,
            size / 2,
            size / 2,
            0);
    imageView.onHoverEvent(event);

    verify(mockAccessibilityDelegate, times(1)).onAccessibilityHoverEvent(event, true);
  }
}
