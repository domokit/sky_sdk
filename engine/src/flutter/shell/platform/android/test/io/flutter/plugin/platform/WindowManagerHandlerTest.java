// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoInteractions;

import android.annotation.TargetApi;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import java.util.concurrent.Executor;
import java.util.function.Consumer;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_28)
public class WindowManagerHandlerTest {
  @Test
  @Config(minSdk = API_LEVELS.API_30)
  public void windowManagerHandler_passesCorrectlyToFakeWindowViewGroup() {
    // Mock the WindowManager and FakeWindowViewGroup that get used by the WindowManagerHandler.
    WindowManager mockWindowManager = mock(WindowManager.class);
    SingleViewFakeWindowViewGroup mockSingleViewFakeWindowViewGroup =
        mock(SingleViewFakeWindowViewGroup.class);

    View mockView = mock(View.class);
    ViewGroup.LayoutParams mockLayoutParams = mock(ViewGroup.LayoutParams.class);

    WindowManagerHandler windowManagerHandler =
        new WindowManagerHandler(mockWindowManager, mockSingleViewFakeWindowViewGroup);

    // removeViewImmediate
    windowManagerHandler.removeViewImmediate(mockView);
    verify(mockView).clearAnimation();
    verify(mockSingleViewFakeWindowViewGroup).removeView(mockView);
    verifyNoInteractions(mockWindowManager);

    // addView
    windowManagerHandler.addView(mockView, mockLayoutParams);
    verify(mockSingleViewFakeWindowViewGroup).addView(mockView, mockLayoutParams);
    verifyNoInteractions(mockWindowManager);

    // updateViewLayout
    windowManagerHandler.updateViewLayout(mockView, mockLayoutParams);
    verify(mockSingleViewFakeWindowViewGroup).updateViewLayout(mockView, mockLayoutParams);
    verifyNoInteractions(mockWindowManager);

    // removeView
    windowManagerHandler.updateViewLayout(mockView, mockLayoutParams);
    verify(mockSingleViewFakeWindowViewGroup).removeView(mockView);
    verifyNoInteractions(mockWindowManager);
  }

  @Test
  @Config(minSdk = API_LEVELS.API_30)
  public void windowManagerHandler_logAndReturnEarly_whenFakeWindowViewGroupIsNull() {
    // Mock the WindowManager and FakeWindowViewGroup that get used by the WindowManagerHandler.
    WindowManager mockWindowManager = mock(WindowManager.class);

    View mockView = mock(View.class);
    ViewGroup.LayoutParams mockLayoutParams = mock(ViewGroup.LayoutParams.class);

    WindowManagerHandler windowManagerHandler = new WindowManagerHandler(mockWindowManager, null);

    // removeViewImmediate
    windowManagerHandler.removeViewImmediate(mockView);
    verifyNoInteractions(mockView);
    verifyNoInteractions(mockWindowManager);

    // addView
    windowManagerHandler.addView(mockView, mockLayoutParams);
    verifyNoInteractions(mockWindowManager);

    // updateViewLayout
    windowManagerHandler.updateViewLayout(mockView, mockLayoutParams);
    verifyNoInteractions(mockWindowManager);

    // removeView
    windowManagerHandler.updateViewLayout(mockView, mockLayoutParams);
    verifyNoInteractions(mockWindowManager);
  }

  // This section tests that WindowManagerHandler forwards all of the non-special case calls to the
  // delegate WindowManager. Because this must include some deprecated WindowManager method calls
  // (because the proxy overrides every method), we suppress deprecation warnings here.
  @Test
  @Config(minSdk = API_LEVELS.API_31)
  @SuppressWarnings("deprecation")
  public void windowManagerHandler_forwardsAllOtherCallsToDelegate() {
    // Mock the WindowManager and FakeWindowViewGroup that get used by the WindowManagerHandler.
    WindowManager mockWindowManager = mock(WindowManager.class);
    SingleViewFakeWindowViewGroup mockSingleViewFakeWindowViewGroup =
        mock(SingleViewFakeWindowViewGroup.class);

    WindowManagerHandler windowManagerHandler =
        new WindowManagerHandler(mockWindowManager, mockSingleViewFakeWindowViewGroup);

    // Verify that all other calls get forwarded to the delegate.
    Executor mockExecutor = mock(Executor.class);
    @SuppressWarnings("Unchecked cast")
    Consumer<Boolean> mockListener = (Consumer<Boolean>) mock(Consumer.class);

    // Windowmanager's getDefaultDisplay() function is deprecated in API 30 level 30.
    // See Android docs here:
    // https://developer.android.com/reference/android/view/WindowManager#getDefaultDisplay()
    // We expect this behavior because this unit test expects a blind forward that includes
    // deprecated function calls. See comment above for more details.
    windowManagerHandler.getDefaultDisplay();
    verify(mockWindowManager).getDefaultDisplay();

    windowManagerHandler.getCurrentWindowMetrics();
    verify(mockWindowManager).getCurrentWindowMetrics();

    windowManagerHandler.getMaximumWindowMetrics();
    verify(mockWindowManager).getMaximumWindowMetrics();

    windowManagerHandler.isCrossWindowBlurEnabled();
    verify(mockWindowManager).isCrossWindowBlurEnabled();

    windowManagerHandler.addCrossWindowBlurEnabledListener(mockListener);
    verify(mockWindowManager).addCrossWindowBlurEnabledListener(mockListener);

    windowManagerHandler.addCrossWindowBlurEnabledListener(mockExecutor, mockListener);
    verify(mockWindowManager).addCrossWindowBlurEnabledListener(mockExecutor, mockListener);

    windowManagerHandler.removeCrossWindowBlurEnabledListener(mockListener);
    verify(mockWindowManager).removeCrossWindowBlurEnabledListener(mockListener);
  }
}
