// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static android.view.WindowInsetsController.APPEARANCE_LIGHT_NAVIGATION_BARS;
import static android.view.WindowInsetsController.APPEARANCE_LIGHT_STATUS_BARS;
import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.anyBoolean;
import static org.mockito.Mockito.doThrow;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.mockStatic;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.app.Activity;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.ContentResolver;
import android.content.Context;
import android.content.Intent;
import android.content.res.AssetFileDescriptor;
import android.net.Uri;
import android.os.Build;
import android.view.View;
import android.view.Window;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import androidx.activity.OnBackPressedCallback;
import androidx.fragment.app.FragmentActivity;
import androidx.test.core.app.ApplicationProvider;
import io.flutter.embedding.engine.systemchannels.PlatformChannel;
import io.flutter.embedding.engine.systemchannels.PlatformChannel.Brightness;
import io.flutter.embedding.engine.systemchannels.PlatformChannel.ClipboardContentFormat;
import io.flutter.embedding.engine.systemchannels.PlatformChannel.SystemChromeStyle;
import io.flutter.plugin.platform.PlatformPlugin.PlatformPluginDelegate;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.robolectric.Robolectric;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.android.controller.ActivityController;
import org.robolectric.annotation.Config;
import org.robolectric.shadows.ShadowLooper;

@Config(manifest = Config.NONE)
@RunWith(RobolectricTestRunner.class)
public class PlatformPluginTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  private final PlatformChannel mockPlatformChannel = mock(PlatformChannel.class);

  private ClipboardManager clipboardManager;
  private ClipboardContentFormat clipboardFormat;

  public void setUpForTextClipboardTests(Activity mockActivity) {
    clipboardManager = spy(ctx.getSystemService(ClipboardManager.class));
    when(mockActivity.getSystemService(Context.CLIPBOARD_SERVICE)).thenReturn(clipboardManager);
    clipboardFormat = ClipboardContentFormat.PLAIN_TEXT;
  }

  @Test
  public void platformPlugin_getClipboardDataIsNonNullWhenPlainTextCopied() throws IOException {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);
    setUpForTextClipboardTests(mockActivity);

    // Primary clip contains non-text media.
    ClipData clip = ClipData.newPlainText("label", "Text");
    assertNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
    clipboardManager.setPrimaryClip(clip);
    assertNotNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
  }

  @Test
  public void platformPlugin_getClipboardDataIsNonNullWhenIOExceptionThrown() throws IOException {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);
    setUpForTextClipboardTests(mockActivity);
    ContentResolver contentResolver = mock(ContentResolver.class);
    ClipData.Item mockItem = mock(ClipData.Item.class);
    Uri mockUri = mock(Uri.class);
    AssetFileDescriptor mockAssetFileDescriptor = mock(AssetFileDescriptor.class);

    when(mockActivity.getContentResolver()).thenReturn(contentResolver);
    when(mockUri.getScheme()).thenReturn("content");
    when(mockItem.getText()).thenReturn(null);
    when(mockItem.getUri()).thenReturn(mockUri);
    when(contentResolver.openTypedAssetFileDescriptor(any(Uri.class), any(), any()))
        .thenReturn(mockAssetFileDescriptor);
    when(mockItem.coerceToText(mockActivity)).thenReturn("something non-null");
    doThrow(new IOException()).when(mockAssetFileDescriptor).close();

    ClipData clip = new ClipData("label", new String[0], mockItem);

    assertNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
    clipboardManager.setPrimaryClip(clip);
    assertNotNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
  }

  @Test
  public void platformPlugin_getClipboardDataIsNonNullWhenContentUriWithTextProvided()
      throws IOException {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);
    setUpForTextClipboardTests(mockActivity);
    ContentResolver contentResolver = mock(ContentResolver.class);
    ClipData.Item mockItem = mock(ClipData.Item.class);
    Uri mockUri = mock(Uri.class);

    when(mockActivity.getContentResolver()).thenReturn(contentResolver);
    when(mockUri.getScheme()).thenReturn("content");
    when(mockItem.getText()).thenReturn(null);
    when(mockItem.getUri()).thenReturn(mockUri);
    when(contentResolver.openTypedAssetFileDescriptor(any(Uri.class), any(), any()))
        .thenReturn(mock(AssetFileDescriptor.class));
    when(mockItem.coerceToText(mockActivity)).thenReturn("something non-null");

    ClipData clip = new ClipData("label", new String[0], mockItem);

    assertNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
    clipboardManager.setPrimaryClip(clip);
    assertNotNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
  }

  @Test
  public void platformPlugin_getClipboardDataIsNullWhenContentUriProvidedContainsNoText()
      throws IOException {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);
    setUpForTextClipboardTests(mockActivity);
    ContentResolver contentResolver = ctx.getContentResolver();

    when(mockActivity.getContentResolver()).thenReturn(contentResolver);

    Uri uri = Uri.parse("content://media/external_primary/images/media/");
    ClipData clip = ClipData.newUri(contentResolver, "URI", uri);

    clipboardManager.setPrimaryClip(clip);
    assertNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
  }

  @Test
  public void platformPlugin_getClipboardDataIsNullWhenNonContentUriProvided() throws IOException {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);
    setUpForTextClipboardTests(mockActivity);
    ContentResolver contentResolver = ctx.getContentResolver();

    when(mockActivity.getContentResolver()).thenReturn(contentResolver);

    Uri uri = Uri.parse("file:///");
    ClipData clip = ClipData.newUri(contentResolver, "URI", uri);

    clipboardManager.setPrimaryClip(clip);
    assertNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
  }

  @Test
  public void platformPlugin_getClipboardDataIsNullWhenItemHasNoTextNorUri() throws IOException {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);
    setUpForTextClipboardTests(mockActivity);
    ClipData.Item mockItem = mock(ClipData.Item.class);

    when(mockItem.getText()).thenReturn(null);
    when(mockItem.getUri()).thenReturn(null);

    ClipData clip = new ClipData("label", new String[0], mockItem);

    clipboardManager.setPrimaryClip(clip);
    assertNull(platformPlugin.mPlatformMessageHandler.getClipboardData(clipboardFormat));
  }

  @SuppressWarnings("deprecation")
  // ClipboardManager.getText
  @Config(sdk = API_LEVELS.API_28)
  @Test
  public void platformPlugin_hasStrings() {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);
    setUpForTextClipboardTests(mockActivity);

    // Plain text
    ClipData clip = ClipData.newPlainText("label", "Text");
    clipboardManager.setPrimaryClip(clip);
    assertTrue(platformPlugin.mPlatformMessageHandler.clipboardHasStrings());

    // Empty plain text
    clip = ClipData.newPlainText("", "");
    clipboardManager.setPrimaryClip(clip);
    // Without actually accessing clipboard data (preferred behavior), it is not possible to
    // distinguish between empty and non-empty string contents.
    assertTrue(platformPlugin.mPlatformMessageHandler.clipboardHasStrings());

    // HTML text
    clip = ClipData.newHtmlText("motto", "Don't be evil", "<b>Don't</b> be evil");
    clipboardManager.setPrimaryClip(clip);
    assertTrue(platformPlugin.mPlatformMessageHandler.clipboardHasStrings());

    // Text MIME type
    clip = new ClipData("label", new String[] {"text/something"}, new ClipData.Item("content"));
    clipboardManager.setPrimaryClip(clip);
    assertTrue(platformPlugin.mPlatformMessageHandler.clipboardHasStrings());

    // Other MIME type
    clip =
        new ClipData(
            "label", new String[] {"application/octet-stream"}, new ClipData.Item("content"));
    clipboardManager.setPrimaryClip(clip);
    assertFalse(platformPlugin.mPlatformMessageHandler.clipboardHasStrings());

    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      // Empty clipboard
      clipboardManager.clearPrimaryClip();
      assertFalse(platformPlugin.mPlatformMessageHandler.clipboardHasStrings());
    }

    // Verify that the clipboard contents are never accessed.
    verify(clipboardManager, never()).getPrimaryClip();
    verify(clipboardManager, never()).getText();
  }

  @Config(sdk = API_LEVELS.API_29)
  @SuppressWarnings("deprecation")
  // setStatusBarColor, setNavigationBarColor, setNavigationBarDividerColor,
  // setStatusBarContrastEnforced, setNavigationBarContrastEnforced
  @Test
  public void setNavigationBarDividerColor() {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);

    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      // Default style test
      SystemChromeStyle style =
          new SystemChromeStyle(
              0XFF000000, // statusBarColor
              Brightness.LIGHT, // statusBarIconBrightness
              true, // systemStatusBarContrastEnforced
              0XFFC70039, // systemNavigationBarColor
              Brightness.LIGHT, // systemNavigationBarIconBrightness
              0XFF006DB3, // systemNavigationBarDividerColor
              true); // systemNavigationBarContrastEnforced

      platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);

      verify(fakeWindow).setStatusBarColor(0xFF000000);
      verify(fakeWindow).setNavigationBarColor(0XFFC70039);
      verify(fakeWindow).setNavigationBarDividerColor(0XFF006DB3);
      verify(fakeWindow).setStatusBarContrastEnforced(true);
      verify(fakeWindow).setNavigationBarContrastEnforced(true);

      // Regression test for https://github.com/flutter/flutter/issues/88431
      // A null brightness should not affect changing color settings.
      style =
          new SystemChromeStyle(
              0XFF006DB3, // statusBarColor
              null, // statusBarIconBrightness
              false, // systemStatusBarContrastEnforced
              0XFF000000, // systemNavigationBarColor
              null, // systemNavigationBarIconBrightness
              0XFF006DB3, // systemNavigationBarDividerColor
              false); // systemNavigationBarContrastEnforced

      platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);

      verify(fakeWindow).setStatusBarColor(0XFF006DB3);
      verify(fakeWindow).setNavigationBarColor(0XFF000000);
      verify(fakeWindow, times(2)).setNavigationBarDividerColor(0XFF006DB3);
      verify(fakeWindow).setStatusBarContrastEnforced(false);
      verify(fakeWindow).setNavigationBarContrastEnforced(false);

      // Null contrasts values should be allowed.
      style =
          new SystemChromeStyle(
              0XFF006DB3, // statusBarColor
              null, // statusBarIconBrightness
              null, // systemStatusBarContrastEnforced
              0XFF000000, // systemNavigationBarColor
              null, // systemNavigationBarIconBrightness
              0XFF006DB3, // systemNavigationBarDividerColor
              null); // systemNavigationBarContrastEnforced

      platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);

      verify(fakeWindow, times(2)).setStatusBarColor(0XFF006DB3);
      verify(fakeWindow, times(2)).setNavigationBarColor(0XFF000000);
      verify(fakeWindow, times(3)).setNavigationBarDividerColor(0XFF006DB3);
      // Count is 1 each from earlier calls
      verify(fakeWindow, times(1)).setStatusBarContrastEnforced(true);
      verify(fakeWindow, times(1)).setNavigationBarContrastEnforced(true);
      verify(fakeWindow, times(1)).setStatusBarContrastEnforced(false);
      verify(fakeWindow, times(1)).setNavigationBarContrastEnforced(false);
    }
  }

  @Config(sdk = API_LEVELS.API_30)
  @Test
  public void setNavigationBarIconBrightness() {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);

    if (Build.VERSION.SDK_INT >= API_LEVELS.API_30) {
      WindowInsetsController fakeWindowInsetsController = mock(WindowInsetsController.class);
      when(fakeWindow.getInsetsController()).thenReturn(fakeWindowInsetsController);

      SystemChromeStyle style =
          new SystemChromeStyle(
              null, // statusBarColor
              null, // statusBarIconBrightness
              null, // systemStatusBarContrastEnforced
              null, // systemNavigationBarColor
              Brightness.LIGHT, // systemNavigationBarIconBrightness
              null, // systemNavigationBarDividerColor
              null); // systemNavigationBarContrastEnforced

      platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);

      verify(fakeWindowInsetsController)
          .setSystemBarsAppearance(0, APPEARANCE_LIGHT_NAVIGATION_BARS);

      style =
          new SystemChromeStyle(
              null, // statusBarColor
              null, // statusBarIconBrightness
              null, // systemStatusBarContrastEnforced
              null, // systemNavigationBarColor
              Brightness.DARK, // systemNavigationBarIconBrightness
              null, // systemNavigationBarDividerColor
              null); // systemNavigationBarContrastEnforced

      platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);

      verify(fakeWindowInsetsController)
          .setSystemBarsAppearance(
              APPEARANCE_LIGHT_NAVIGATION_BARS, APPEARANCE_LIGHT_NAVIGATION_BARS);
    }
  }

  @Config(sdk = API_LEVELS.API_30)
  @Test
  public void setStatusBarIconBrightness() {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);

    if (Build.VERSION.SDK_INT >= API_LEVELS.API_30) {
      WindowInsetsController fakeWindowInsetsController = mock(WindowInsetsController.class);
      when(fakeWindow.getInsetsController()).thenReturn(fakeWindowInsetsController);

      SystemChromeStyle style =
          new SystemChromeStyle(
              null, // statusBarColor
              Brightness.LIGHT, // statusBarIconBrightness
              null, // systemStatusBarContrastEnforced
              null, // systemNavigationBarColor
              null, // systemNavigationBarIconBrightness
              null, // systemNavigationBarDividerColor
              null); // systemNavigationBarContrastEnforced

      platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);

      verify(fakeWindowInsetsController).setSystemBarsAppearance(0, APPEARANCE_LIGHT_STATUS_BARS);

      style =
          new SystemChromeStyle(
              null, // statusBarColor
              Brightness.DARK, // statusBarIconBrightness
              null, // systemStatusBarContrastEnforced
              null, // systemNavigationBarColor
              null, // systemNavigationBarIconBrightness
              null, // systemNavigationBarDividerColor
              null); // systemNavigationBarContrastEnforced

      platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);

      verify(fakeWindowInsetsController)
          .setSystemBarsAppearance(APPEARANCE_LIGHT_STATUS_BARS, APPEARANCE_LIGHT_STATUS_BARS);
    }
  }

  @SuppressWarnings("deprecation")
  // SYSTEM_UI_FLAG_*, setSystemUiVisibility
  @Config(sdk = API_LEVELS.API_29)
  @Test
  public void setSystemUiMode() {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);

    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      platformPlugin.mPlatformMessageHandler.showSystemUiMode(
          PlatformChannel.SystemUiMode.LEAN_BACK);
      verify(fakeDecorView)
          .setSystemUiVisibility(
              View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                  | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                  | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                  | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                  | View.SYSTEM_UI_FLAG_FULLSCREEN);

      platformPlugin.mPlatformMessageHandler.showSystemUiMode(
          PlatformChannel.SystemUiMode.IMMERSIVE);
      verify(fakeDecorView)
          .setSystemUiVisibility(
              View.SYSTEM_UI_FLAG_IMMERSIVE
                  | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                  | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                  | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                  | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                  | View.SYSTEM_UI_FLAG_FULLSCREEN);

      platformPlugin.mPlatformMessageHandler.showSystemUiMode(
          PlatformChannel.SystemUiMode.IMMERSIVE_STICKY);
      verify(fakeDecorView)
          .setSystemUiVisibility(
              View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                  | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                  | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                  | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                  | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                  | View.SYSTEM_UI_FLAG_FULLSCREEN);
    }

    if (Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      platformPlugin.mPlatformMessageHandler.showSystemUiMode(
          PlatformChannel.SystemUiMode.EDGE_TO_EDGE);
      verify(fakeDecorView)
          .setSystemUiVisibility(
              View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                  | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                  | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN);
    }
  }

  @SuppressWarnings("deprecation")
  // SYSTEM_UI_FLAG_FULLSCREEN
  @Test
  public void setSystemUiModeListener_overlaysAreHidden() {
    ActivityController<Activity> controller = Robolectric.buildActivity(Activity.class);
    controller.setup();
    Activity fakeActivity = controller.get();

    PlatformPlugin platformPlugin = new PlatformPlugin(fakeActivity, mockPlatformChannel);

    // Subscribe to system UI visibility events.
    platformPlugin.mPlatformMessageHandler.setSystemUiChangeListener();

    // Simulate system UI changed to full screen.
    fakeActivity
        .getWindow()
        .getDecorView()
        .dispatchSystemUiVisibilityChanged(View.SYSTEM_UI_FLAG_FULLSCREEN);

    // No events should have been sent to the platform channel yet. They are scheduled for
    // the next frame.
    verify(mockPlatformChannel, never()).systemChromeChanged(anyBoolean());

    // Simulate the next frame.
    ShadowLooper.runUiThreadTasksIncludingDelayedTasks();

    // Now the platform channel should receive the event.
    verify(mockPlatformChannel).systemChromeChanged(false);
  }

  @SuppressWarnings("deprecation")
  // dispatchSystemUiVisibilityChanged
  @Test
  public void setSystemUiModeListener_overlaysAreVisible() {
    ActivityController<Activity> controller = Robolectric.buildActivity(Activity.class);
    controller.setup();
    Activity fakeActivity = controller.get();
    PlatformPlugin platformPlugin = new PlatformPlugin(fakeActivity, mockPlatformChannel);

    // Subscribe to system Ui visibility events.
    platformPlugin.mPlatformMessageHandler.setSystemUiChangeListener();

    // Simulate system UI changed to *not* full screen.
    fakeActivity.getWindow().getDecorView().dispatchSystemUiVisibilityChanged(0);

    // No events should have been sent to the platform channel yet. They are scheduled for
    // the next frame.
    verify(mockPlatformChannel, never()).systemChromeChanged(anyBoolean());

    // Simulate the next frame.
    ShadowLooper.runUiThreadTasksIncludingDelayedTasks();

    // Now the platform channel should receive the event.
    verify(mockPlatformChannel).systemChromeChanged(true);
  }

  @SuppressWarnings("deprecation")
  // SYSTEM_UI_FLAG_*, setSystemUiVisibility
  @Config(sdk = API_LEVELS.API_28)
  @Test
  public void doNotEnableEdgeToEdgeOnOlderSdk() {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);

    platformPlugin.mPlatformMessageHandler.showSystemUiMode(
        PlatformChannel.SystemUiMode.EDGE_TO_EDGE);
    verify(fakeDecorView, never())
        .setSystemUiVisibility(
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN);
  }

  @SuppressWarnings("deprecation")
  // FLAG_TRANSLUCENT_STATUS, FLAG_TRANSLUCENT_NAVIGATION
  @Config(sdk = API_LEVELS.API_29)
  @Test
  public void verifyWindowFlagsSetToStyleOverlays() {
    View fakeDecorView = mock(View.class);
    Window fakeWindow = mock(Window.class);
    Activity mockActivity = mock(Activity.class);
    when(fakeWindow.getDecorView()).thenReturn(fakeDecorView);
    when(mockActivity.getWindow()).thenReturn(fakeWindow);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);

    SystemChromeStyle style =
        new SystemChromeStyle(
            0XFF000000, Brightness.LIGHT, true, 0XFFC70039, Brightness.LIGHT, 0XFF006DB3, true);

    platformPlugin.mPlatformMessageHandler.setSystemUiOverlayStyle(style);
    verify(fakeWindow).addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);
    verify(fakeWindow)
        .clearFlags(
            WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS
                | WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION);
  }

  @Test
  public void setFrameworkHandlesBackFlutterActivity() {
    Activity mockActivity = mock(Activity.class);
    PlatformPluginDelegate mockPlatformPluginDelegate = mock(PlatformPluginDelegate.class);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(mockActivity, mockPlatformChannel, mockPlatformPluginDelegate);

    platformPlugin.mPlatformMessageHandler.setFrameworkHandlesBack(true);

    verify(mockPlatformPluginDelegate, times(1)).setFrameworkHandlesBack(true);
  }

  @Test
  public void testPlatformPluginDelegateNull() throws Exception {
    Activity mockActivity = mock(Activity.class);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(mockActivity, mockPlatformChannel, null /*platformPluginDelegate*/);

    try {
      platformPlugin.mPlatformMessageHandler.setFrameworkHandlesBack(true);
    } catch (NullPointerException e) {
      // Not expected
      fail("NullPointerException was thrown");
    }
  }

  @Test
  public void popSystemNavigatorFlutterActivity() {
    Activity mockActivity = mock(Activity.class);
    PlatformPluginDelegate mockPlatformPluginDelegate = mock(PlatformPluginDelegate.class);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(mockActivity, mockPlatformChannel, mockPlatformPluginDelegate);

    when(mockPlatformPluginDelegate.popSystemNavigator()).thenReturn(false);

    platformPlugin.mPlatformMessageHandler.popSystemNavigator();

    verify(mockPlatformPluginDelegate, times(1)).popSystemNavigator();
    verify(mockActivity, times(1)).finish();
  }

  @Test
  public void doesNotDoAnythingByDefaultIfPopSystemNavigatorOverridden() {
    Activity mockActivity = mock(Activity.class);
    PlatformPluginDelegate mockPlatformPluginDelegate = mock(PlatformPluginDelegate.class);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(mockActivity, mockPlatformChannel, mockPlatformPluginDelegate);

    when(mockPlatformPluginDelegate.popSystemNavigator()).thenReturn(true);

    platformPlugin.mPlatformMessageHandler.popSystemNavigator();

    verify(mockPlatformPluginDelegate, times(1)).popSystemNavigator();
    // No longer perform the default action when overridden.
    verify(mockActivity, never()).finish();
  }

  @SuppressWarnings("deprecation")
  // Robolectric.setupActivity.
  // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
  @Test
  public void popSystemNavigatorFlutterFragment() {
    // Migrate to ActivityScenario by following https://github.com/robolectric/robolectric/pull/4736
    FragmentActivity activity = spy(Robolectric.setupActivity(FragmentActivity.class));
    final AtomicBoolean onBackPressedCalled = new AtomicBoolean(false);
    OnBackPressedCallback backCallback =
        new OnBackPressedCallback(true) {
          @Override
          public void handleOnBackPressed() {
            onBackPressedCalled.set(true);
          }
        };
    activity.getOnBackPressedDispatcher().addCallback(backCallback);

    PlatformPluginDelegate mockPlatformPluginDelegate = mock(PlatformPluginDelegate.class);
    when(mockPlatformPluginDelegate.popSystemNavigator()).thenReturn(false);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(activity, mockPlatformChannel, mockPlatformPluginDelegate);

    platformPlugin.mPlatformMessageHandler.popSystemNavigator();

    verify(activity, never()).finish();
    verify(mockPlatformPluginDelegate, times(1)).popSystemNavigator();
    assertTrue(onBackPressedCalled.get());
  }

  @SuppressWarnings("deprecation")
  // Robolectric.setupActivity.
  // TODO(reidbaker): https://github.com/flutter/flutter/issues/133151
  @Test
  public void doesNotDoAnythingByDefaultIfFragmentPopSystemNavigatorOverridden() {
    FragmentActivity activity = spy(Robolectric.setupActivity(FragmentActivity.class));
    PlatformPluginDelegate mockPlatformPluginDelegate = mock(PlatformPluginDelegate.class);
    when(mockPlatformPluginDelegate.popSystemNavigator()).thenReturn(true);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(activity, mockPlatformChannel, mockPlatformPluginDelegate);

    platformPlugin.mPlatformMessageHandler.popSystemNavigator();

    verify(mockPlatformPluginDelegate, times(1)).popSystemNavigator();
    // No longer perform the default action when overridden.
    verify(activity, never()).finish();
  }

  @Test
  public void setRequestedOrientationFlutterFragment() {
    FragmentActivity mockFragmentActivity = mock(FragmentActivity.class);
    PlatformPluginDelegate mockPlatformPluginDelegate = mock(PlatformPluginDelegate.class);
    when(mockPlatformPluginDelegate.popSystemNavigator()).thenReturn(false);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(mockFragmentActivity, mockPlatformChannel, mockPlatformPluginDelegate);

    platformPlugin.mPlatformMessageHandler.setPreferredOrientations(0);

    verify(mockFragmentActivity, times(1)).setRequestedOrientation(0);
  }

  @Test
  public void performsDefaultBehaviorWhenNoDelegateProvided() {
    Activity mockActivity = mock(Activity.class);
    PlatformChannel mockPlatformChannel = mock(PlatformChannel.class);
    PlatformPlugin platformPlugin = new PlatformPlugin(mockActivity, mockPlatformChannel);

    platformPlugin.mPlatformMessageHandler.popSystemNavigator();

    verify(mockActivity, times(1)).finish();
  }

  @Test
  public void startChoosenActivityWhenSharingText() {
    Activity mockActivity = mock(Activity.class);
    PlatformChannel mockPlatformChannel = mock(PlatformChannel.class);
    PlatformPluginDelegate mockPlatformPluginDelegate = mock(PlatformPluginDelegate.class);
    PlatformPlugin platformPlugin =
        new PlatformPlugin(mockActivity, mockPlatformChannel, mockPlatformPluginDelegate);

    // Mock Intent.createChooser (in real application it opens a chooser where the user can
    // select which application will be used to share the selected text).
    Intent choosenIntent = new Intent();
    MockedStatic<Intent> intentClass = mockStatic(Intent.class);
    ArgumentCaptor<Intent> intentCaptor = ArgumentCaptor.forClass(Intent.class);
    intentClass
        .when(() -> Intent.createChooser(intentCaptor.capture(), any()))
        .thenReturn(choosenIntent);

    final String expectedContent = "Flutter";
    platformPlugin.mPlatformMessageHandler.share(expectedContent);

    // Activity.startActivity should have been called.
    verify(mockActivity, times(1)).startActivity(choosenIntent);

    // The intent action created by the plugin and passed to Intent.createChooser should be
    // 'Intent.ACTION_SEND'.
    Intent sendToIntent = intentCaptor.getValue();
    assertEquals(sendToIntent.getAction(), Intent.ACTION_SEND);
    assertEquals(sendToIntent.getType(), "text/plain");
    assertEquals(sendToIntent.getStringExtra(Intent.EXTRA_TEXT), expectedContent);
  }
}
