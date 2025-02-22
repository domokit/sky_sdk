// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package dev.flutter.plugins.integration_test;

import android.annotation.TargetApi;
import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.view.Choreographer;
import android.view.PixelCopy;
import android.view.View;
import android.view.ViewGroup;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.VisibleForTesting;

import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.android.FlutterFragment;
import io.flutter.embedding.android.FlutterFragmentActivity;
import io.flutter.embedding.android.FlutterSurfaceView;
import io.flutter.embedding.android.FlutterView;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.MethodChannel.Result;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.lang.StringBuilder;

import android.content.ContextWrapper;
import java.io.FileOutputStream;
import 	java.io.File;
import android.content.Context;
import io.flutter.Log;
import android.os.Environment;

/**
 * FlutterDeviceScreenshot is a utility class that allows to capture a screenshot
 * that includes both Android views and the Flutter UI.
 *
 * To take screenshots, the rendering surface must be changed to {@code FlutterImageView},
 * since surfaces like {@code FlutterSurfaceView} and {@code FlutterTextureView} are opaque
 * when the view hierarchy is rendered to a bitmap.
 *
 * It's also necessary to ask the framework to schedule a frame, and then add a listener
 * that waits for that frame to be presented by the Android framework.
 */
@TargetApi(19)
class FlutterDeviceScreenshot {
  /**
   * Finds the {@code FlutterView} added to the {@code activity} view hierarchy.
   *
   * <p> This assumes that there's only one {@code FlutterView} per activity, which
   * is always the case.
   *
   * @param activity typically, {code FlutterActivity}.
   * @return the Flutter view.
   */
  @Nullable
  @VisibleForTesting
  public static FlutterView getFlutterView(@NonNull Activity activity) {
    if (activity instanceof FlutterActivity) {
      return (FlutterView)activity.findViewById(FlutterActivity.FLUTTER_VIEW_ID);
    } else if (activity instanceof FlutterFragmentActivity) {
      return (FlutterView)activity.findViewById(FlutterFragment.FLUTTER_VIEW_ID);
    } else {
      return null;
    }
  }

  /**
   * Whether the app is run with instrumentation.
   *
   * @return true if the app is running with instrumentation.
   */
  static boolean hasInstrumentation() {
    // TODO(egarciad): InstrumentationRegistry requires the uiautomator dependency.
    // However, Flutter adds test dependencies to release builds.
    // As a result, disable screenshots with instrumentation until the issue is fixed.
    // https://github.com/flutter/flutter/issues/56591
    // CAMILLE: this is what John was trying to enable in https://github.com/flutter/flutter/pull/159910?
    return false;
  }

  /**
   * Captures a screenshot using ui automation.
   *
   * @return byte array containing the screenshot.
   */
  static byte[] captureWithUiAutomation() throws IOException {
    // CAMILLE: this is where we could implement screenshots with what John was implementing?
    return new byte[0];
  }

  // Whether the flutter surface is already converted to an image.
  private static boolean flutterSurfaceConvertedToImage = false;

  /**
   * Converts the Flutter surface to an image view.
   * This allows to render the view hierarchy to a bitmap since
   * {@code FlutterSurfaceView} and {@code FlutterTextureView} cannot be rendered to a bitmap.
   *
   * @param activity typically {@code FlutterActivity}.
   */
  static void convertFlutterSurfaceToImage(@NonNull Activity activity) {
    final FlutterView flutterView = getFlutterView(activity);
    if (flutterView != null && !flutterSurfaceConvertedToImage) {
      // CAMILLE: this call ensures an FlutterImageView is ready to capture screenshot.
      // See FlutterImageView
      flutterView.convertToImageView();
      flutterSurfaceConvertedToImage = true;
    }
  }

  /**
   * Restores the original Flutter surface.
   * The new surface will either be {@code FlutterSurfaceView} or {@code FlutterTextureView}.
   *
   * @param activity typically {@code FlutterActivity}.
   */
  static void revertFlutterImage(@NonNull Activity activity) {
    final FlutterView flutterView = getFlutterView(activity);
    if (flutterView != null && flutterSurfaceConvertedToImage) {
      flutterView.revertImageView(() -> {
        flutterSurfaceConvertedToImage = false;
      });
    }
  }

  // Handlers used to capture a view.
  private static Handler backgroundHandler;
  private static Handler mainHandler;

  /**
   * Captures a screenshot by drawing the view to a Canvas.
   *
   * <p> {@code convertFlutterSurfaceToImage} must be called prior to capturing the view,
   * otherwise the result is an error.
   *
   * @param activity this is {@link FlutterActivity}.
   * @param methodChannel the method channel to call into Dart.
   * @param result the result for the method channel that will contain the byte array.
   */
  // CAMILLE: how capturing screenshots currently works
  static void captureView(
      @NonNull Activity activity, @NonNull MethodChannel methodChannel, @NonNull Result result) {
    final FlutterView flutterView = getFlutterView(activity);
    if (flutterView == null) {
      result.error("Could not copy the pixels", "FlutterView is null", null);
      return;
    }
    if (!flutterSurfaceConvertedToImage) {
      result.error("Could not copy the pixels", "Flutter surface must be converted to image first", null);
      return;
    }

    // Ask the framework to schedule a new frame.
    methodChannel.invokeMethod("scheduleFrame", null);

    if (backgroundHandler == null) {
      final HandlerThread screenshotBackgroundThread = new HandlerThread("screenshot");
      screenshotBackgroundThread.start();
      backgroundHandler = new Handler(screenshotBackgroundThread.getLooper());
    }
    if (mainHandler == null) {
      mainHandler = new Handler(Looper.getMainLooper());
    }
    takeScreenshot(backgroundHandler, mainHandler, flutterView, result);
  }

  /**
   * Waits for the next Android frame.
   *
   * @param r a callback.
   */
  private static void waitForAndroidFrame(Runnable r) {
    Choreographer.getInstance()
        .postFrameCallback(
            new Choreographer.FrameCallback() {
              @Override
              public void doFrame(long frameTimeNanos) {
                r.run();
              }
            });
  }

  /**
   * Waits until a Flutter frame is rendered by the Android OS.
   *
   * @param backgroundHandler the handler associated to a background thread.
   * @param mainHandler the handler associated to the platform thread.
   * @param view the flutter view.
   * @param result the result that contains the byte array.
   */
  private static void takeScreenshot(
      @NonNull Handler backgroundHandler,
      @NonNull Handler mainHandler,
      @NonNull FlutterView view,
      @NonNull Result result) {
    // CAMILLE: how a "screenshot" is "taken". Implemented in FlutterView
    final boolean acquired = view.acquireLatestImageViewFrame();
    // The next frame may already have already been committed.
    // The next frame is guaranteed to have the Flutter image.
    waitForAndroidFrame(
        () -> {
          waitForAndroidFrame(
              () -> {
                if (acquired) {
                  FlutterDeviceScreenshot.convertViewToBitmap(view, result, backgroundHandler);
                } else {
                  takeScreenshot(backgroundHandler, mainHandler, view, result);
                }
              });
        });
  }

  /**
   * Renders {@code FlutterView} to a Bitmap.
   *
   * If successful, The byte array is provided in the result.
   *
   * @param flutterView the Flutter view.
   * @param result the result that contains the byte array.
   * @param backgroundHandler a background handler to avoid blocking the platform thread.
   */
  private static void convertViewToBitmap(
    @NonNull FlutterView flutterView, @NonNull Result result, @NonNull Handler backgroundHandler) {
    if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) {
      final Bitmap bitmap =
          Bitmap.createBitmap(
              flutterView.getWidth(), flutterView.getHeight(), Bitmap.Config.RGB_565);
      final Canvas canvas = new Canvas(bitmap);
      flutterView.draw(canvas);
      final Activity flutterActivity = (Activity) flutterView.getContext();

      // TODO(camsim99): try saving image to check out what is going on.
      ContextWrapper cw = new ContextWrapper(flutterActivity.getApplicationContext());
      // path to /data/data/yourapp/app_data/imageDir
      File directory = cw.getDir("imageDir", Context.MODE_PRIVATE);
      // Create imageDir
      File mypath=new File(directory,"camille_image_1.jpg");

      FileOutputStream fos = null;
      try {
          fos = new FileOutputStream(mypath);
      // Use the compress method on the BitMap object to write image to the OutputStream
          bitmap.compress(Bitmap.CompressFormat.PNG, 100, fos);
      } catch (Exception e) {
            e.printStackTrace();
      } finally {
          try {
            fos.close();
          } catch (IOException e) {
            e.printStackTrace();
          }
      }
      Log.e("CAMILLE: image directory:", directory.getAbsolutePath());

      final ByteArrayOutputStream output = new ByteArrayOutputStream();
      bitmap.compress(Bitmap.CompressFormat.PNG, /*quality=*/ 100, output);
      result.success(output.toByteArray());
      return;
    }

    final Bitmap bitmap =
        Bitmap.createBitmap(
            flutterView.getWidth(), flutterView.getHeight(), Bitmap.Config.ARGB_8888);
    final Activity flutterActivity = (Activity) flutterView.getContext();

      // TODO(camsim99): try saving image to check out what is going on.
      ContextWrapper cw = new ContextWrapper(flutterActivity.getApplicationContext());
      // path to /data/data/yourapp/app_data/imageDir
      // File directory = cw.getDir("imageDir", Context.MODE_PRIVATE);
      // Create imageDir
      File directory = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
      File mypath=new File(directory,"camille_image_2.png");

      FileOutputStream fos = null;
      try {
          fos = new FileOutputStream(mypath);
      // Use the compress method on the BitMap object to write image to the OutputStream
          bitmap.compress(Bitmap.CompressFormat.PNG, 100, fos);
      } catch (Exception e) {
            e.printStackTrace();
      } finally {
          try {
            fos.close();
          } catch (IOException e) {
            e.printStackTrace();
          }
      }
      Log.e("CAMILLE: image directory:", directory.getAbsolutePath());

    final int[] flutterViewLocation = new int[2];
    flutterView.getLocationInWindow(flutterViewLocation);
    final int flutterViewLeft = flutterViewLocation[0];
    final int flutterViewTop = flutterViewLocation[1];

    final Rect flutterViewRect =
        new Rect(
            flutterViewLeft,
            flutterViewTop,
            flutterViewLeft + flutterView.getWidth(),
            flutterViewTop + flutterView.getHeight());

    PixelCopy.request(
        flutterActivity.getWindow(),
        flutterViewRect,
        bitmap,
        (int copyResult) -> {
          final Handler mainHandler = new Handler(Looper.getMainLooper());
          if (copyResult == PixelCopy.SUCCESS) {
            final ByteArrayOutputStream output = new ByteArrayOutputStream();
            bitmap.compress(Bitmap.CompressFormat.PNG, /*quality=*/ 100, output);
            mainHandler.post(
                () -> {
                  result.success(output.toByteArray());
                });
          } else {
            mainHandler.post(
                () -> {
                  result.error("Could not copy the pixels", "result was " + copyResult, null);
                });
          }
        },
        backgroundHandler);
  }
}
