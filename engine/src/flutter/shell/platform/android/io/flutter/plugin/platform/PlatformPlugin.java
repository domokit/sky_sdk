// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.plugin.platform;

import static io.flutter.Build.API_LEVELS;

import android.app.Activity;
import android.app.ActivityManager.TaskDescription;
import android.content.ClipData;
import android.content.ClipDescription;
import android.content.ClipboardManager;
import android.content.Context;
import android.content.Intent;
import android.content.res.AssetFileDescriptor;
import android.net.Uri;
import android.os.Build;
import android.view.HapticFeedbackConstants;
import android.view.SoundEffectConstants;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import androidx.activity.OnBackPressedDispatcherOwner;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;
import androidx.core.view.WindowInsetsControllerCompat;
import io.flutter.Log;
import io.flutter.embedding.engine.systemchannels.PlatformChannel;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.List;

/** Android implementation of the platform plugin. */
public class PlatformPlugin {
  public static final int DEFAULT_SYSTEM_UI =
      View.SYSTEM_UI_FLAG_LAYOUT_STABLE | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN;

  private final Activity activity;
  private final PlatformChannel platformChannel;
  @Nullable private final PlatformPluginDelegate platformPluginDelegate;
  private PlatformChannel.SystemChromeStyle currentTheme;
  private int mEnabledOverlays;
  private static final String TAG = "PlatformPlugin";

  /**
   * The {@link PlatformPlugin} generally has default behaviors implemented for platform
   * functionalities requested by the Flutter framework. However, functionalities exposed through
   * this interface could be customized by the more public-facing APIs that implement this interface
   * such as the {@link io.flutter.embedding.android.FlutterActivity} or the {@link
   * io.flutter.embedding.android.FlutterFragment}.
   */
  public interface PlatformPluginDelegate {
    /**
     * Allow implementer to customize the behavior needed when the Flutter framework calls to pop
     * the Android-side navigation stack.
     *
     * @return true if the implementation consumed the pop signal. If false, a default behavior of
     *     finishing the activity or sending the signal to {@link
     *     androidx.activity.OnBackPressedDispatcher} will be executed.
     */
    boolean popSystemNavigator();

    /**
     * The Flutter application would or would not like to handle navigation pop events itself.
     *
     * <p>Relevant for registering and unregistering the app's OnBackInvokedCallback for the
     * Predictive Back feature, for example as in {@link
     * io.flutter.embedding.android.FlutterActivity}.
     */
    default void setFrameworkHandlesBack(boolean frameworkHandlesBack) {}
  }

  @VisibleForTesting
  final PlatformChannel.PlatformMessageHandler mPlatformMessageHandler =
      new PlatformChannel.PlatformMessageHandler() {
        @Override
        public void playSystemSound(@NonNull PlatformChannel.SoundType soundType) {
          PlatformPlugin.this.playSystemSound(soundType);
        }

        @Override
        public void vibrateHapticFeedback(
            @NonNull PlatformChannel.HapticFeedbackType feedbackType) {
          PlatformPlugin.this.vibrateHapticFeedback(feedbackType);
        }

        @Override
        public void setPreferredOrientations(int androidOrientation) {
          setSystemChromePreferredOrientations(androidOrientation);
        }

        @Override
        public void setApplicationSwitcherDescription(
            @NonNull PlatformChannel.AppSwitcherDescription description) {
          setSystemChromeApplicationSwitcherDescription(description);
        }

        @Override
        public void showSystemOverlays(@NonNull List<PlatformChannel.SystemUiOverlay> overlays) {
          setSystemChromeEnabledSystemUIOverlays(overlays);
        }

        @Override
        public void showSystemUiMode(@NonNull PlatformChannel.SystemUiMode mode) {
          setSystemChromeEnabledSystemUIMode(mode);
        }

        @Override
        public void setSystemUiChangeListener() {
          setSystemChromeChangeListener();
        }

        @Override
        public void restoreSystemUiOverlays() {
          restoreSystemChromeSystemUIOverlays();
        }

        @Override
        public void setSystemUiOverlayStyle(
            @NonNull PlatformChannel.SystemChromeStyle systemUiOverlayStyle) {
          setSystemChromeSystemUIOverlayStyle(systemUiOverlayStyle);
        }

        @Override
        public void setFrameworkHandlesBack(boolean frameworkHandlesBack) {
          PlatformPlugin.this.setFrameworkHandlesBack(frameworkHandlesBack);
        }

        @Override
        public void popSystemNavigator() {
          PlatformPlugin.this.popSystemNavigator();
        }

        @Override
        public CharSequence getClipboardData(
            @Nullable PlatformChannel.ClipboardContentFormat format) {
          return PlatformPlugin.this.getClipboardData(format);
        }

        @Override
        public void setClipboardData(@NonNull String text) {
          PlatformPlugin.this.setClipboardData(text);
        }

        @Override
        public boolean clipboardHasStrings() {
          return PlatformPlugin.this.clipboardHasStrings();
        }

        @Override
        public void share(@NonNull String text) {
          PlatformPlugin.this.share(text);
        }
      };

  public PlatformPlugin(@NonNull Activity activity, @NonNull PlatformChannel platformChannel) {
    this(activity, platformChannel, null);
  }

  public PlatformPlugin(
      @NonNull Activity activity,
      @NonNull PlatformChannel platformChannel,
      @Nullable PlatformPluginDelegate delegate) {
    this.activity = activity;
    this.platformChannel = platformChannel;
    this.platformChannel.setPlatformMessageHandler(mPlatformMessageHandler);
    this.platformPluginDelegate = delegate;

    mEnabledOverlays = DEFAULT_SYSTEM_UI;
  }

  /**
   * Releases all resources held by this {@code PlatformPlugin}.
   *
   * <p>Do not invoke any methods on a {@code PlatformPlugin} after invoking this method.
   */
  public void destroy() {
    this.platformChannel.setPlatformMessageHandler(null);
  }

  private void playSystemSound(@NonNull PlatformChannel.SoundType soundType) {
    if (soundType == PlatformChannel.SoundType.CLICK) {
      View view = activity.getWindow().getDecorView();
      view.playSoundEffect(SoundEffectConstants.CLICK);
    }
  }

  @VisibleForTesting
  /* package */ void vibrateHapticFeedback(
      @NonNull PlatformChannel.HapticFeedbackType feedbackType) {
    View view = activity.getWindow().getDecorView();
    switch (feedbackType) {
      case STANDARD:
        view.performHapticFeedback(HapticFeedbackConstants.LONG_PRESS);
        break;
      case LIGHT_IMPACT:
        view.performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY);
        break;
      case MEDIUM_IMPACT:
        view.performHapticFeedback(HapticFeedbackConstants.KEYBOARD_TAP);
        break;
      case HEAVY_IMPACT:
        if (Build.VERSION.SDK_INT >= API_LEVELS.API_23) {
          view.performHapticFeedback(HapticFeedbackConstants.CONTEXT_CLICK);
        }
        break;
      case SELECTION_CLICK:
        view.performHapticFeedback(HapticFeedbackConstants.CLOCK_TICK);
        break;
    }
  }

  private void setSystemChromePreferredOrientations(int androidOrientation) {
    activity.setRequestedOrientation(androidOrientation);
  }

  @SuppressWarnings("deprecation")
  private void setSystemChromeApplicationSwitcherDescription(
      PlatformChannel.AppSwitcherDescription description) {
    if (Build.VERSION.SDK_INT < API_LEVELS.API_28) {
      activity.setTaskDescription(
          new TaskDescription(description.label, /* icon= */ null, description.color));
    } else {
      TaskDescription taskDescription =
          new TaskDescription(description.label, 0, description.color);
      activity.setTaskDescription(taskDescription);
    }
  }

  private void setSystemChromeChangeListener() {
    // Set up a listener to notify the framework when the system ui has changed.
    View decorView = activity.getWindow().getDecorView();
    decorView.setOnSystemUiVisibilityChangeListener(
        new View.OnSystemUiVisibilityChangeListener() {
          @Override
          public void onSystemUiVisibilityChange(int visibility) {
            // `platformChannel.systemChromeChanged` may trigger a callback that eventually results
            // in a call to `setSystemUiVisibility`.
            // `setSystemUiVisibility` must not be called in the same frame as when
            // `onSystemUiVisibilityChange` is received though.
            //
            // As such, post `platformChannel.systemChromeChanged` to the view handler to ensure
            // that downstream callbacks are trigged on the next frame.
            decorView.post(
                () -> {
                  if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
                    // The system bars are visible. Make any desired adjustments to
                    // your UI, such as showing the action bar or other navigational
                    // controls. Another common action is to set a timer to dismiss
                    // the system bars and restore the fullscreen mode that was
                    // previously enabled.
                    platformChannel.systemChromeChanged(true);
                  } else {
                    // The system bars are NOT visible. Make any desired adjustments
                    // to your UI, such as hiding the action bar or other
                    // navigational controls.
                    platformChannel.systemChromeChanged(false);
                  }
                });
          }
        });
  }

  private void setSystemChromeEnabledSystemUIMode(PlatformChannel.SystemUiMode systemUiMode) {
    int enabledOverlays;

    if (systemUiMode == PlatformChannel.SystemUiMode.LEAN_BACK) {
      // LEAN BACK
      // Available starting at Android SDK 4.1 (API 16).
      //
      // If the Flutter Android app targets Android SDK 15 (API 35) or later then the Android
      // system will ignore this value unless the app also follows the opt out
      // instructions found in
      // https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge.
      //
      // Should not show overlays, tap to reveal overlays, needs onChange callback
      // When the overlays come in on tap, the app does not receive the gesture and does not know
      // the system overlay has changed. The overlays cannot be dismissed, so adding the callback
      // support will allow users to restore the system ui and dismiss the overlays.
      // Not compatible with top/bottom overlays enabled.
      enabledOverlays =
          View.SYSTEM_UI_FLAG_LAYOUT_STABLE
              | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
              | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
              | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
              | View.SYSTEM_UI_FLAG_FULLSCREEN;
    } else if (systemUiMode == PlatformChannel.SystemUiMode.IMMERSIVE) {
      // IMMERSIVE
      // Available starting at Android SDK 4.4 (API 19).
      //
      // If the Flutter Android app targets Android SDK 15 (API 35) or later then the Android
      // system will ignore this value unless the app also follows the opt out
      // instructions found in
      // https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge.
      //
      // Should not show overlays, swipe from edges to reveal overlays, needs onChange callback
      // When the overlays come in on swipe, the app does not receive the gesture and does not know
      // the system overlay has changed. The overlays cannot be dismissed, so adding callback
      // support will allow users to restore the system ui and dismiss the overlays.
      // Not compatible with top/bottom overlays enabled.
      enabledOverlays =
          View.SYSTEM_UI_FLAG_IMMERSIVE
              | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
              | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
              | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
              | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
              | View.SYSTEM_UI_FLAG_FULLSCREEN;
    } else if (systemUiMode == PlatformChannel.SystemUiMode.IMMERSIVE_STICKY) {
      // STICKY IMMERSIVE
      // Available starting at Android SDK 4.4 (API 19).
      //
      // If the Flutter Android app targets Android SDK 15 (API 35) or later then the Android
      // system will ignore this value unless the app also follows the opt out
      // instructions found in
      // https://docs.flutter.dev/release/breaking-changes/default-systemuimode-edge-to-edge.
      //
      // Should not show overlays, swipe from edges to reveal overlays. The app will also receive
      // the swipe gesture. The overlays cannot be dismissed, so adding callback support will
      // allow users to restore the system ui and dismiss the overlays.
      // Not compatible with top/bottom overlays enabled.
      enabledOverlays =
          View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
              | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
              | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
              | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
              | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
              | View.SYSTEM_UI_FLAG_FULLSCREEN;
    } else if (systemUiMode == PlatformChannel.SystemUiMode.EDGE_TO_EDGE
        && Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      // EDGE TO EDGE
      //
      // Available starting at Android SDK 10 (API 29).
      //
      // If the Flutter app targets Android SDK 15 (API 35) or later (Flutter does this by default),
      // then this mode is used by default.
      //
      // SDK 29 and up will apply a translucent body scrim behind 2/3 button navigation bars
      // to ensure contrast with buttons on the nav and status bars, unless the contrast is not
      // enforced in the overlay styling.
      enabledOverlays =
          View.SYSTEM_UI_FLAG_LAYOUT_STABLE
              | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
              | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN;
    } else {
      // When none of the conditions are matched, return without updating the system UI overlays.
      return;
    }

    mEnabledOverlays = enabledOverlays;
    updateSystemUiOverlays();
  }

  private void setSystemChromeEnabledSystemUIOverlays(
      List<PlatformChannel.SystemUiOverlay> overlaysToShow) {
    // Start by assuming we want to hide all system overlays (like an immersive
    // game).
    int enabledOverlays =
        DEFAULT_SYSTEM_UI
            | View.SYSTEM_UI_FLAG_FULLSCREEN
            | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
            | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION;

    if (overlaysToShow.size() == 0) {
      enabledOverlays |= View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY;
    }

    // Re-add any desired system overlays.
    for (int i = 0; i < overlaysToShow.size(); ++i) {
      PlatformChannel.SystemUiOverlay overlayToShow = overlaysToShow.get(i);
      switch (overlayToShow) {
        case TOP_OVERLAYS:
          enabledOverlays &= ~View.SYSTEM_UI_FLAG_FULLSCREEN;
          break;
        case BOTTOM_OVERLAYS:
          enabledOverlays &= ~View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION;
          enabledOverlays &= ~View.SYSTEM_UI_FLAG_HIDE_NAVIGATION;
          break;
      }
    }

    mEnabledOverlays = enabledOverlays;
    updateSystemUiOverlays();
  }

  /**
   * Refreshes Android's window system UI (AKA system chrome) to match Flutter's desired {@link
   * PlatformChannel.SystemChromeStyle}.
   *
   * <p>Updating the system UI Overlays is accomplished by altering the decor view of the {@link
   * Window} associated with the {@link android.app.Activity} that was provided to this {@code
   * PlatformPlugin}.
   */
  public void updateSystemUiOverlays() {
    activity.getWindow().getDecorView().setSystemUiVisibility(mEnabledOverlays);
    if (currentTheme != null) {
      setSystemChromeSystemUIOverlayStyle(currentTheme);
    }
  }

  private void restoreSystemChromeSystemUIOverlays() {
    updateSystemUiOverlays();
  }

  @SuppressWarnings("deprecation")
  private void setSystemChromeSystemUIOverlayStyle(
      PlatformChannel.SystemChromeStyle systemChromeStyle) {
    Window window = activity.getWindow();
    View view = window.getDecorView();
    WindowInsetsControllerCompat windowInsetsControllerCompat =
        new WindowInsetsControllerCompat(window, view);

    if (Build.VERSION.SDK_INT < API_LEVELS.API_30) {
      // Flag set to specify that this window is responsible for drawing the background for the
      // system bars. Must be set for all operations on API < 30 (Android SDK < 11) excluding
      // enforcing
      // system bar contrasts. Deprecated in API 30.
      window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);

      // Flag set to dismiss any requests for translucent system bars to be provided in lieu of what
      // is specified by systemChromeStyle. Must be set for all operations on API < 30 operations
      // excluding enforcing system bar contrasts. Deprecated in API 30.
      window.clearFlags(
          WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS
              | WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION);
    }

    // SYSTEM STATUS BAR -------------------------------------------------------------------
    // You can't change the color of the system status bar until SDK 21, and you can't change the
    // color of the status icons until SDK 23. We only allow both starting at 23 to ensure buttons
    // and icons can be visible when changing the background color.
    // If transparent, SDK 29 and higher may apply a translucent scrim behind the bar to ensure
    // proper contrast. This can be overridden with
    // SystemChromeStyle.systemStatusBarContrastEnforced.
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_23) {
      if (systemChromeStyle.statusBarIconBrightness != null) {
        switch (systemChromeStyle.statusBarIconBrightness) {
          case DARK:
            // Dark status bar icon brightness.
            // Light status bar appearance.
            windowInsetsControllerCompat.setAppearanceLightStatusBars(true);
            break;
          case LIGHT:
            // Light status bar icon brightness.
            // Dark status bar appearance.
            windowInsetsControllerCompat.setAppearanceLightStatusBars(false);
            break;
        }
      }

      if (systemChromeStyle.statusBarColor != null) {
        window.setStatusBarColor(systemChromeStyle.statusBarColor);
      }
    }
    // You can't override the enforced contrast for a transparent status bar until SDK 29.
    // This overrides the translucent scrim that may be placed behind the bar on SDK 29+ to ensure
    // contrast is appropriate when using full screen layout modes like Edge to Edge.
    if (systemChromeStyle.systemStatusBarContrastEnforced != null
        && Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      window.setStatusBarContrastEnforced(systemChromeStyle.systemStatusBarContrastEnforced);
    }

    // SYSTEM NAVIGATION BAR --------------------------------------------------------------
    // You can't change the color of the system navigation bar until SDK 21, and you can't change
    // the color of the navigation buttons until SDK 26. We only allow both starting at 26 to
    // ensure buttons can be visible when changing the background color.
    // If transparent, SDK 29 and higher may apply a translucent scrim behind 2/3 button navigation
    // bars to ensure proper contrast. This can be overridden with
    // SystemChromeStyle.systemNavigationBarContrastEnforced.
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_26) {
      if (systemChromeStyle.systemNavigationBarIconBrightness != null) {
        switch (systemChromeStyle.systemNavigationBarIconBrightness) {
          case DARK:
            // Dark navigation bar icon brightness.
            // Light navigation bar appearance.
            windowInsetsControllerCompat.setAppearanceLightNavigationBars(true);
            break;
          case LIGHT:
            // Light navigation bar icon brightness.
            // Dark navigation bar appearance.
            windowInsetsControllerCompat.setAppearanceLightNavigationBars(false);
            break;
        }
      }

      if (systemChromeStyle.systemNavigationBarColor != null) {
        window.setNavigationBarColor(systemChromeStyle.systemNavigationBarColor);
      }
    }
    // You can't change the color of the navigation bar divider color until SDK 28.
    if (systemChromeStyle.systemNavigationBarDividerColor != null
        && Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      window.setNavigationBarDividerColor(systemChromeStyle.systemNavigationBarDividerColor);
    }

    // You can't override the enforced contrast for a transparent navigation bar until SDK 29.
    // This overrides the translucent scrim that may be placed behind 2/3 button navigation bars on
    // SDK 29+ to ensure contrast is appropriate when using full screen layout modes like
    // Edge to Edge.
    if (systemChromeStyle.systemNavigationBarContrastEnforced != null
        && Build.VERSION.SDK_INT >= API_LEVELS.API_29) {
      window.setNavigationBarContrastEnforced(
          systemChromeStyle.systemNavigationBarContrastEnforced);
    }

    currentTheme = systemChromeStyle;
  }

  private void setFrameworkHandlesBack(boolean frameworkHandlesBack) {
    if (platformPluginDelegate != null) {
      platformPluginDelegate.setFrameworkHandlesBack(frameworkHandlesBack);
    }
  }

  private void popSystemNavigator() {
    if (platformPluginDelegate != null && platformPluginDelegate.popSystemNavigator()) {
      // A custom behavior was executed by the delegate. Don't execute default behavior.
      return;
    }

    if (activity instanceof OnBackPressedDispatcherOwner) {
      ((OnBackPressedDispatcherOwner) activity).getOnBackPressedDispatcher().onBackPressed();
    } else {
      activity.finish();
    }
  }

  private CharSequence getClipboardData(PlatformChannel.ClipboardContentFormat format) {
    ClipboardManager clipboard =
        (ClipboardManager) activity.getSystemService(Context.CLIPBOARD_SERVICE);

    if (!clipboard.hasPrimaryClip()) return null;

    CharSequence itemText = null;
    try {
      ClipData clip = clipboard.getPrimaryClip();
      if (clip == null) return null;
      if (format == null || format == PlatformChannel.ClipboardContentFormat.PLAIN_TEXT) {
        ClipData.Item item = clip.getItemAt(0);
        // First, try getting clipboard data as text; no further processing
        // required if so.
        itemText = item.getText();
        if (itemText == null) {
          // Clipboard data does not contain text, so check whether or not it
          // contains a URI to extract text from.
          Uri itemUri = item.getUri();

          if (itemUri == null) {
            Log.w(
                TAG, "Clipboard item contained no textual content nor a URI to retrieve it from.");
            return null;
          }

          // Will only try to extract text from URI if it has the content scheme.
          String uriScheme = itemUri.getScheme();

          if (!uriScheme.equals("content")) {
            Log.w(
                TAG,
                "Clipboard item contains a Uri with scheme '" + uriScheme + "'that is unhandled.");
            return null;
          }

          AssetFileDescriptor assetFileDescriptor =
              activity.getContentResolver().openTypedAssetFileDescriptor(itemUri, "text/*", null);

          // Safely return clipboard data coerced into text; will return either
          // itemText or text retrieved from its URI.
          itemText = item.coerceToText(activity);
          if (assetFileDescriptor != null) assetFileDescriptor.close();
        }

        return itemText;
      }
    } catch (SecurityException e) {
      Log.w(
          TAG,
          "Attempted to get clipboard data that requires additional permission(s).\n"
              + "See the exception details for which permission(s) are required, and consider adding them to your Android Manifest as described in:\n"
              + "https://developer.android.com/guide/topics/permissions/overview",
          e);
      return null;
    } catch (FileNotFoundException e) {
      Log.w(TAG, "Clipboard text was unable to be received from content URI.");
      return null;
    } catch (IOException e) {
      Log.w(TAG, "Failed to close AssetFileDescriptor while trying to read text from URI.", e);
      return itemText;
    }

    return null;
  }

  private void setClipboardData(String text) {
    ClipboardManager clipboard =
        (ClipboardManager) activity.getSystemService(Context.CLIPBOARD_SERVICE);
    ClipData clip = ClipData.newPlainText("text label?", text);
    clipboard.setPrimaryClip(clip);
  }

  private boolean clipboardHasStrings() {
    ClipboardManager clipboard =
        (ClipboardManager) activity.getSystemService(Context.CLIPBOARD_SERVICE);
    // Android 12 introduces a toast message that appears when an app reads the clipboard. To avoid
    // unintended access, call the appropriate APIs to receive information about the current content
    // that's on the clipboard (rather than the actual content itself).
    if (!clipboard.hasPrimaryClip()) {
      return false;
    }
    ClipDescription description = clipboard.getPrimaryClipDescription();
    if (description == null) {
      return false;
    }
    return description.hasMimeType("text/*");
  }

  private void share(@NonNull String text) {
    Intent intent = new Intent();
    intent.setAction(Intent.ACTION_SEND);
    intent.setType("text/plain");
    intent.putExtra(Intent.EXTRA_TEXT, text);

    activity.startActivity(Intent.createChooser(intent, null));
  }
}
