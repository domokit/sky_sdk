// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import static io.flutter.Build.API_LEVELS;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.ColorSpace;
import android.graphics.ImageDecoder;
import android.graphics.SurfaceTexture;
import android.os.Build;
import android.os.Looper;
import android.util.DisplayMetrics;
import android.util.Size;
import android.util.TypedValue;
import android.view.Surface;
import android.view.SurfaceHolder;
import androidx.annotation.Keep;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import androidx.annotation.VisibleForTesting;
import com.getkeepsafe.relinker.ReLinker;
import io.flutter.Log;
import io.flutter.embedding.engine.FlutterEngine.EngineLifecycleListener;
import io.flutter.embedding.engine.dart.PlatformMessageHandler;
import io.flutter.embedding.engine.deferredcomponents.DeferredComponentManager;
import io.flutter.embedding.engine.mutatorsstack.FlutterMutatorsStack;
import io.flutter.embedding.engine.renderer.FlutterUiDisplayListener;
import io.flutter.embedding.engine.renderer.SurfaceTextureWrapper;
import io.flutter.embedding.engine.systemchannels.SettingsChannel;
import io.flutter.plugin.common.StandardMessageCodec;
import io.flutter.plugin.localization.LocalizationPlugin;
import io.flutter.plugin.platform.PlatformViewsController;
import io.flutter.util.Preconditions;
import io.flutter.view.AccessibilityBridge;
import io.flutter.view.FlutterCallbackInformation;
import io.flutter.view.TextureRegistry;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Interface between Flutter embedding's Java code and Flutter engine's C/C++ code.
 *
 * <p>Flutter's engine is built with C/C++. The Android Flutter embedding is responsible for
 * coordinating Android OS events and app user interactions with the C/C++ engine. Such coordination
 * requires messaging from an Android app in Java code to the C/C++ engine code. This communication
 * requires a JNI (Java Native Interface) API to cross the Java/native boundary.
 *
 * <p>The entirety of Flutter's JNI API is codified in {@code FlutterJNI}. There are multiple
 * reasons that all such calls are centralized in one class. First, JNI calls are inherently static
 * and contain no Java implementation, therefore there is little reason to associate calls with
 * different classes. Second, every JNI call must be registered in C/C++ code and this registration
 * becomes more complicated with every additional Java class that contains JNI calls. Third, most
 * Android developers are not familiar with native development or JNI intricacies, therefore it is
 * in the interest of future maintenance to reduce the API surface that includes JNI declarations.
 * Thus, all Flutter JNI calls are centralized in {@code FlutterJNI}.
 *
 * <p>Despite the fact that individual JNI calls are inherently static, there is state that exists
 * within {@code FlutterJNI}. Most calls within {@code FlutterJNI} correspond to a specific
 * "platform view", of which there may be many. Therefore, each {@code FlutterJNI} instance holds
 * onto a "native platform view ID" after {@link #attachToNative()}, which is shared with the native
 * C/C++ engine code. That ID is passed to every platform-view-specific native method. ID management
 * is handled within {@code FlutterJNI} so that developers don't have to hold onto that ID.
 *
 * <p>To connect part of an Android app to Flutter's C/C++ engine, instantiate a {@code FlutterJNI}
 * and then attach it to the native side:
 *
 * <pre>{@code
 * // Instantiate FlutterJNI and attach to the native side.
 * FlutterJNI flutterJNI = new FlutterJNI();
 * flutterJNI.attachToNative();
 *
 * // Use FlutterJNI as desired. flutterJNI.dispatchPointerDataPacket(...);
 *
 * // Destroy the connection to the native side and cleanup.
 * flutterJNI.detachFromNativeAndReleaseResources();
 * }</pre>
 *
 * <p>To receive callbacks for certain events that occur on the native side, register listeners:
 *
 * <ol>
 *   <li>{@link #addEngineLifecycleListener(FlutterEngine.EngineLifecycleListener)}
 *   <li>{@link #addIsDisplayingFlutterUiListener(FlutterUiDisplayListener)}
 * </ol>
 *
 * To facilitate platform messages between Java and Dart running in Flutter, register a handler:
 *
 * <p>{@link #setPlatformMessageHandler(PlatformMessageHandler)}
 *
 * <p>To invoke a native method that is not associated with a platform view, invoke it statically:
 *
 * <p>{@code bool enabled = FlutterJNI.getIsSoftwareRenderingEnabled(); }
 */
@Keep
public class FlutterJNI {
  private static final String TAG = "FlutterJNI";
  // This serializes the invocation of platform message responses and the
  // attachment and detachment of the shell holder.  This ensures that we don't
  // detach FlutterJNI on the platform thread while a background thread invokes
  // a message response.  Typically accessing the shell holder happens on the
  // platform thread and doesn't require locking.
  private ReentrantReadWriteLock shellHolderLock = new ReentrantReadWriteLock();

  // Prefer using the FlutterJNI.Factory so it's easier to test.
  public FlutterJNI() {
    // We cache the main looper so that we can ensure calls are made on the main thread
    // without consistently paying the synchronization cost of getMainLooper().
    mainLooper = Looper.getMainLooper();
  }

  /**
   * A factory for creating {@code FlutterJNI} instances. Useful for FlutterJNI injections during
   * tests.
   */
  public static class Factory {
    /** @return a {@link FlutterJNI} instance. */
    public FlutterJNI provideFlutterJNI() {
      return new FlutterJNI();
    }
  }

  // BEGIN Methods related to loading for FlutterLoader.
  /**
   * Loads the libflutter.so C++ library.
   *
   * <p>This must be called before any other native methods, and can be overridden by tests to avoid
   * loading native libraries.
   *
   * <p>This method should only be called once across all FlutterJNI instances.
   */
  public void loadLibrary(Context context) {
    if (FlutterJNI.loadLibraryCalled) {
      Log.w(TAG, "FlutterJNI.loadLibrary called more than once");
    }
    ReLinker.loadLibrary(context, "flutter");
    FlutterJNI.loadLibraryCalled = true;
  }

  private static boolean loadLibraryCalled = false;

  private static native void nativePrefetchDefaultFontManager();

  /**
   * Prefetch the default font manager provided by txt::GetDefaultFontManager() which is a
   * process-wide singleton owned by Skia. Note that, the first call to txt::GetDefaultFontManager()
   * will take noticeable time, but later calls will return a reference to the preexisting font
   * manager.
   *
   * <p>This method should only be called once across all FlutterJNI instances.
   */
  public void prefetchDefaultFontManager() {
    if (FlutterJNI.prefetchDefaultFontManagerCalled) {
      Log.w(TAG, "FlutterJNI.prefetchDefaultFontManager called more than once");
    }

    FlutterJNI.nativePrefetchDefaultFontManager();
    FlutterJNI.prefetchDefaultFontManagerCalled = true;
  }

  private static boolean prefetchDefaultFontManagerCalled = false;

  private static native void nativeInit(
      @NonNull Context context,
      @NonNull String[] args,
      @Nullable String bundlePath,
      @NonNull String appStoragePath,
      @NonNull String engineCachesPath,
      long initTimeMillis);

  /**
   * Perform one time initialization of the Dart VM and Flutter engine.
   *
   * <p>This method must be called only once. Calling more than once will cause an exception.
   *
   * @param context The application context.
   * @param args Arguments to the Dart VM/Flutter engine.
   * @param bundlePath For JIT runtimes, the path to the Dart kernel file for the application.
   * @param appStoragePath The path to the application data directory.
   * @param engineCachesPath The path to the application cache directory.
   * @param initTimeMillis The time, in milliseconds, taken for initialization.
   */
  public void init(
      @NonNull Context context,
      @NonNull String[] args,
      @Nullable String bundlePath,
      @NonNull String appStoragePath,
      @NonNull String engineCachesPath,
      long initTimeMillis) {
    if (FlutterJNI.initCalled) {
      Log.w(TAG, "FlutterJNI.init called more than once");
    }

    FlutterJNI.nativeInit(
        context, args, bundlePath, appStoragePath, engineCachesPath, initTimeMillis);
    FlutterJNI.initCalled = true;
  }

  private static boolean initCalled = false;
  // END methods related to FlutterLoader

  @Nullable private static AsyncWaitForVsyncDelegate asyncWaitForVsyncDelegate;

  /**
   * This value is updated by the VsyncWaiter when it is initialized.
   *
   * <p>On API 17+, it is updated whenever the default display refresh rate changes.
   *
   * <p>It is defaulted to 60.
   */
  private static float refreshRateFPS = 60.0f;

  private static float displayWidth = -1.0f;
  private static float displayHeight = -1.0f;
  private static float displayDensity = -1.0f;

  // This is set from native code via JNI.
  @Nullable private static String vmServiceUri;

  private native boolean nativeGetIsSoftwareRenderingEnabled();

  /**
   * Checks launch settings for whether software rendering is requested.
   *
   * <p>The value is the same per program.
   */
  @UiThread
  public boolean getIsSoftwareRenderingEnabled() {
    return nativeGetIsSoftwareRenderingEnabled();
  }

  /**
   * VM Service URI for the VM instance.
   *
   * <p>Its value is set by the native engine once {@link #init(Context, String[], String, String,
   * String, long)} is run.
   */
  @Nullable
  public static String getVMServiceUri() {
    return vmServiceUri;
  }

  /**
   * VM Service URI for the VM instance.
   *
   * <p>Its value is set by the native engine once {@link #init(Context, String[], String, String,
   * String, long)} is run.
   *
   * @deprecated replaced by {@link #getVMServiceUri()}.
   */
  @Deprecated
  @Nullable
  public static String getObservatoryUri() {
    return vmServiceUri;
  }

  /**
   * Notifies the engine about the refresh rate of the display when the API level is below 30.
   *
   * <p>For API 30 and above, this value is ignored.
   *
   * <p>Calling this method multiple times will update the refresh rate for the next vsync period.
   * However, callers should avoid calling {@link android.view.Display#getRefreshRate} frequently,
   * since it is expensive on some vendor implementations.
   *
   * @param refreshRateFPS The refresh rate in nanoseconds.
   */
  public void setRefreshRateFPS(float refreshRateFPS) {
    // This is ok because it only ever tracks the refresh rate of the main
    // display. If we ever need to support the refresh rate of other displays
    // on Android we will need to refactor this. Static lookup makes things a
    // bit easier on the C++ side.
    FlutterJNI.refreshRateFPS = refreshRateFPS;
    updateRefreshRate();
  }

  public void updateDisplayMetrics(int displayId, float width, float height, float density) {
    FlutterJNI.displayWidth = width;
    FlutterJNI.displayHeight = height;
    FlutterJNI.displayDensity = density;
    if (!FlutterJNI.loadLibraryCalled) {
      return;
    }
    nativeUpdateDisplayMetrics(nativeShellHolderId);
  }

  private native void nativeUpdateDisplayMetrics(long nativeShellHolderId);

  public void updateRefreshRate() {
    if (!FlutterJNI.loadLibraryCalled) {
      return;
    }
    nativeUpdateRefreshRate(refreshRateFPS);
  }

  private native void nativeUpdateRefreshRate(float refreshRateFPS);

  /**
   * The Android vsync waiter implementation in C++ needs to know when a vsync signal arrives, which
   * is obtained via Java API. The delegate set here is called on the C++ side when the engine is
   * ready to wait for the next vsync signal. The delegate is expected to add a postFrameCallback to
   * the {@link android.view.Choreographer}, and call {@link onVsync} to notify the engine.
   *
   * @param delegate The delegate that will call the engine back on the next vsync signal.
   */
  public void setAsyncWaitForVsyncDelegate(@Nullable AsyncWaitForVsyncDelegate delegate) {
    asyncWaitForVsyncDelegate = delegate;
  }

  // TODO(mattcarroll): add javadocs
  // Called by native.
  private static void asyncWaitForVsync(final long cookie) {
    if (asyncWaitForVsyncDelegate != null) {
      asyncWaitForVsyncDelegate.asyncWaitForVsync(cookie);
    } else {
      throw new IllegalStateException(
          "An AsyncWaitForVsyncDelegate must be registered with FlutterJNI before asyncWaitForVsync() is invoked.");
    }
  }

  private native void nativeOnVsync(long frameDelayNanos, long refreshPeriodNanos, long cookie);

  /**
   * Notifies the engine that the Choreographer has signaled a vsync.
   *
   * @param frameDelayNanos The time in nanoseconds when the frame started being rendered,
   *     subtracted from the {@link System#nanoTime} timebase.
   * @param refreshPeriodNanos The display refresh period in nanoseconds.
   * @param cookie An opaque handle to the C++ VSyncWaiter object.
   */
  public void onVsync(long frameDelayNanos, long refreshPeriodNanos, long cookie) {
    nativeOnVsync(frameDelayNanos, refreshPeriodNanos, cookie);
  }

  @NonNull
  @Deprecated
  public static native FlutterCallbackInformation nativeLookupCallbackInformation(long handle);

  // ----- Start FlutterTextUtils Methods ----
  private native boolean nativeFlutterTextUtilsIsEmoji(int codePoint);

  public boolean isCodePointEmoji(int codePoint) {
    return nativeFlutterTextUtilsIsEmoji(codePoint);
  }

  private native boolean nativeFlutterTextUtilsIsEmojiModifier(int codePoint);

  public boolean isCodePointEmojiModifier(int codePoint) {
    return nativeFlutterTextUtilsIsEmojiModifier(codePoint);
  }

  private native boolean nativeFlutterTextUtilsIsEmojiModifierBase(int codePoint);

  public boolean isCodePointEmojiModifierBase(int codePoint) {
    return nativeFlutterTextUtilsIsEmojiModifierBase(codePoint);
  }

  private native boolean nativeFlutterTextUtilsIsVariationSelector(int codePoint);

  public boolean isCodePointVariantSelector(int codePoint) {
    return nativeFlutterTextUtilsIsVariationSelector(codePoint);
  }

  private native boolean nativeFlutterTextUtilsIsRegionalIndicator(int codePoint);

  public boolean isCodePointRegionalIndicator(int codePoint) {
    return nativeFlutterTextUtilsIsRegionalIndicator(codePoint);
  }

  // ----- End Engine FlutterTextUtils Methods ----

  // Below represents the stateful part of the FlutterJNI instances that aren't static per program.
  // Conceptually, it represents a native shell instance.

  @Nullable private Long nativeShellHolderId;
  @Nullable private AccessibilityDelegate accessibilityDelegate;
  @Nullable private PlatformMessageHandler platformMessageHandler;
  @Nullable private LocalizationPlugin localizationPlugin;
  @Nullable private PlatformViewsController platformViewsController;

  @Nullable private DeferredComponentManager deferredComponentManager;

  @NonNull
  private final Set<EngineLifecycleListener> engineLifecycleListeners = new CopyOnWriteArraySet<>();

  @NonNull
  private final Set<FlutterUiDisplayListener> flutterUiDisplayListeners =
      new CopyOnWriteArraySet<>();

  @NonNull private final Looper mainLooper; // cached to avoid synchronization on repeat access.

  // ------ Start Native Attach/Detach Support ----
  /**
   * Returns true if this instance of {@code FlutterJNI} is connected to Flutter's native engine via
   * a Java Native Interface (JNI).
   */
  public boolean isAttached() {
    return nativeShellHolderId != null;
  }

  /**
   * Attaches this {@code FlutterJNI} instance to Flutter's native engine, which allows for
   * communication between Android code and Flutter's platform agnostic engine.
   *
   * <p>This method must not be invoked if {@code FlutterJNI} is already attached to native.
   */
  @UiThread
  public void attachToNative() {
    ensureRunningOnMainThread();
    ensureNotAttachedToNative();
    shellHolderLock.writeLock().lock();
    try {
      nativeShellHolderId = performNativeAttach(this);
    } finally {
      shellHolderLock.writeLock().unlock();
    }
  }

  @VisibleForTesting
  public long performNativeAttach(@NonNull FlutterJNI flutterJNI) {
    return nativeAttach(flutterJNI);
  }

  private native long nativeAttach(@NonNull FlutterJNI flutterJNI);

  /**
   * Spawns a new FlutterJNI instance from the current instance.
   *
   * <p>This creates another native shell from the current shell. This causes the 2 shells to re-use
   * some of the shared resources, reducing the total memory consumption versus creating a new
   * FlutterJNI by calling its standard constructor.
   *
   * <p>This can only be called once the current FlutterJNI instance is attached by calling {@link
   * #attachToNative()}.
   *
   * <p>Static methods that should be only called once such as {@link #init(Context, String[],
   * String, String, String, long)} shouldn't be called again on the spawned FlutterJNI instance.
   */
  @UiThread
  @NonNull
  public FlutterJNI spawn(
      @Nullable String entrypointFunctionName,
      @Nullable String pathToEntrypointFunction,
      @Nullable String initialRoute,
      @Nullable List<String> entrypointArgs) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    FlutterJNI spawnedJNI =
        nativeSpawn(
            nativeShellHolderId,
            entrypointFunctionName,
            pathToEntrypointFunction,
            initialRoute,
            entrypointArgs);
    Preconditions.checkState(
        spawnedJNI.nativeShellHolderId != null && spawnedJNI.nativeShellHolderId != 0,
        "Failed to spawn new JNI connected shell from existing shell.");

    return spawnedJNI;
  }

  private native FlutterJNI nativeSpawn(
      long nativeSpawningShellId,
      @Nullable String entrypointFunctionName,
      @Nullable String pathToEntrypointFunction,
      @Nullable String initialRoute,
      @Nullable List<String> entrypointArgs);

  /**
   * Detaches this {@code FlutterJNI} instance from Flutter's native engine, which precludes any
   * further communication between Android code and Flutter's platform agnostic engine.
   *
   * <p>This method must not be invoked if {@code FlutterJNI} is not already attached to native.
   *
   * <p>Invoking this method will result in the release of all native-side resources that were set
   * up during {@link #attachToNative()} or {@link #spawn(String, String, String, List)}, or
   * accumulated thereafter.
   *
   * <p>It is permissible to re-attach this instance to native after detaching it from native.
   */
  @UiThread
  public void detachFromNativeAndReleaseResources() {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    shellHolderLock.writeLock().lock();
    try {
      nativeDestroy(nativeShellHolderId);
      nativeShellHolderId = null;
    } finally {
      shellHolderLock.writeLock().unlock();
    }
  }

  private native void nativeDestroy(long nativeShellHolderId);

  private void ensureNotAttachedToNative() {
    if (nativeShellHolderId != null) {
      throw new RuntimeException(
          "Cannot execute operation because FlutterJNI is attached to native.");
    }
  }

  private void ensureAttachedToNative() {
    if (nativeShellHolderId == null) {
      throw new RuntimeException(
          "Cannot execute operation because FlutterJNI is not attached to native.");
    }
  }
  // ------ End Native Attach/Detach Support ----

  // ----- Start Render Surface Support -----
  /**
   * Adds a {@link FlutterUiDisplayListener}, which receives a callback when Flutter's engine
   * notifies {@code FlutterJNI} that Flutter is painting pixels to the {@link Surface} that was
   * provided to Flutter.
   */
  @UiThread
  public void addIsDisplayingFlutterUiListener(@NonNull FlutterUiDisplayListener listener) {
    ensureRunningOnMainThread();
    flutterUiDisplayListeners.add(listener);
  }

  /**
   * Removes a {@link FlutterUiDisplayListener} that was added with {@link
   * #addIsDisplayingFlutterUiListener(FlutterUiDisplayListener)}.
   */
  @UiThread
  public void removeIsDisplayingFlutterUiListener(@NonNull FlutterUiDisplayListener listener) {
    ensureRunningOnMainThread();
    flutterUiDisplayListeners.remove(listener);
  }

  public static native void nativeImageHeaderCallback(
      long imageGeneratorPointer, int width, int height);

  /**
   * Called by native as a fallback method of image decoding. There are other ways to decode images
   * on lower API levels, they involve copying the native data _and_ do not support any additional
   * formats, whereas ImageDecoder supports HEIF images. Unlike most other methods called from
   * native, this method is expected to be called on a worker thread, since it only uses thread safe
   * methods and may take multiple frames to complete.
   */
  @SuppressWarnings("unused")
  @VisibleForTesting
  @Nullable
  public static Bitmap decodeImage(@NonNull ByteBuffer buffer, long imageGeneratorAddress) {
    if (Build.VERSION.SDK_INT >= API_LEVELS.API_28) {
      ImageDecoder.Source source = ImageDecoder.createSource(buffer);
      try {
        return ImageDecoder.decodeBitmap(
            source,
            (decoder, info, src) -> {
              // i.e. ARGB_8888
              decoder.setTargetColorSpace(ColorSpace.get(ColorSpace.Named.SRGB));
              // TODO(bdero): Switch to ALLOCATOR_HARDWARE for devices that have
              // `AndroidBitmap_getHardwareBuffer` (API 30+) available once Skia supports
              // `SkImage::MakeFromAHardwareBuffer` via dynamic lookups:
              // https://skia-review.googlesource.com/c/skia/+/428960
              decoder.setAllocator(ImageDecoder.ALLOCATOR_SOFTWARE);

              Size size = info.getSize();
              nativeImageHeaderCallback(imageGeneratorAddress, size.getWidth(), size.getHeight());
            });
      } catch (IOException e) {
        Log.e(TAG, "Failed to decode image", e);
        return null;
      }
    }
    return null;
  }

  // Called by native to notify first Flutter frame rendered.
  @SuppressWarnings("unused")
  @VisibleForTesting
  @UiThread
  public void onFirstFrame() {
    ensureRunningOnMainThread();

    for (FlutterUiDisplayListener listener : flutterUiDisplayListeners) {
      listener.onFlutterUiDisplayed();
    }
  }

  // TODO(mattcarroll): get native to call this when rendering stops.
  @VisibleForTesting
  @UiThread
  void onRenderingStopped() {
    ensureRunningOnMainThread();

    for (FlutterUiDisplayListener listener : flutterUiDisplayListeners) {
      listener.onFlutterUiNoLongerDisplayed();
    }
  }

  /**
   * Call this method when a {@link Surface} has been created onto which you would like Flutter to
   * paint.
   *
   * <p>See {@link android.view.SurfaceHolder.Callback#surfaceCreated(SurfaceHolder)} for an example
   * of where this call might originate.
   */
  @UiThread
  public void onSurfaceCreated(@NonNull Surface surface) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeSurfaceCreated(nativeShellHolderId, surface);
  }

  private native void nativeSurfaceCreated(long nativeShellHolderId, @NonNull Surface surface);

  /**
   * In hybrid composition, call this method when the {@link Surface} has changed.
   *
   * <p>In hybrid composition, the root surfaces changes from {@link
   * android.view.SurfaceHolder#getSurface()} to {@link android.media.ImageReader#getSurface()} when
   * a platform view is in the current frame.
   */
  @UiThread
  public void onSurfaceWindowChanged(@NonNull Surface surface) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeSurfaceWindowChanged(nativeShellHolderId, surface);
  }

  private native void nativeSurfaceWindowChanged(
      long nativeShellHolderId, @NonNull Surface surface);

  /**
   * Call this method when the {@link Surface} changes that was previously registered with {@link
   * #onSurfaceCreated(Surface)}.
   *
   * <p>See {@link android.view.SurfaceHolder.Callback#surfaceChanged(SurfaceHolder, int, int, int)}
   * for an example of where this call might originate.
   */
  @UiThread
  public void onSurfaceChanged(int width, int height) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeSurfaceChanged(nativeShellHolderId, width, height);
  }

  private native void nativeSurfaceChanged(long nativeShellHolderId, int width, int height);

  /**
   * Call this method when the {@link Surface} is destroyed that was previously registered with
   * {@link #onSurfaceCreated(Surface)}.
   *
   * <p>See {@link android.view.SurfaceHolder.Callback#surfaceDestroyed(SurfaceHolder)} for an
   * example of where this call might originate.
   */
  @UiThread
  public void onSurfaceDestroyed() {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    onRenderingStopped();
    nativeSurfaceDestroyed(nativeShellHolderId);
  }

  private native void nativeSurfaceDestroyed(long nativeShellHolderId);

  /**
   * Call this method to notify Flutter of the current device viewport metrics that are applies to
   * the Flutter UI that is being rendered.
   *
   * <p>This method should be invoked with initial values upon attaching to native. Then, it should
   * be invoked any time those metrics change while {@code FlutterJNI} is attached to native.
   */
  @UiThread
  public void setViewportMetrics(
      float devicePixelRatio,
      int physicalWidth,
      int physicalHeight,
      int physicalPaddingTop,
      int physicalPaddingRight,
      int physicalPaddingBottom,
      int physicalPaddingLeft,
      int physicalViewInsetTop,
      int physicalViewInsetRight,
      int physicalViewInsetBottom,
      int physicalViewInsetLeft,
      int systemGestureInsetTop,
      int systemGestureInsetRight,
      int systemGestureInsetBottom,
      int systemGestureInsetLeft,
      int physicalTouchSlop,
      int[] displayFeaturesBounds,
      int[] displayFeaturesType,
      int[] displayFeaturesState) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeSetViewportMetrics(
        nativeShellHolderId,
        devicePixelRatio,
        physicalWidth,
        physicalHeight,
        physicalPaddingTop,
        physicalPaddingRight,
        physicalPaddingBottom,
        physicalPaddingLeft,
        physicalViewInsetTop,
        physicalViewInsetRight,
        physicalViewInsetBottom,
        physicalViewInsetLeft,
        systemGestureInsetTop,
        systemGestureInsetRight,
        systemGestureInsetBottom,
        systemGestureInsetLeft,
        physicalTouchSlop,
        displayFeaturesBounds,
        displayFeaturesType,
        displayFeaturesState);
  }

  private native void nativeSetViewportMetrics(
      long nativeShellHolderId,
      float devicePixelRatio,
      int physicalWidth,
      int physicalHeight,
      int physicalPaddingTop,
      int physicalPaddingRight,
      int physicalPaddingBottom,
      int physicalPaddingLeft,
      int physicalViewInsetTop,
      int physicalViewInsetRight,
      int physicalViewInsetBottom,
      int physicalViewInsetLeft,
      int systemGestureInsetTop,
      int systemGestureInsetRight,
      int systemGestureInsetBottom,
      int systemGestureInsetLeft,
      int physicalTouchSlop,
      int[] displayFeaturesBounds,
      int[] displayFeaturesType,
      int[] displayFeaturesState);

  // ----- End Render Surface Support -----

  // ------ Start Touch Interaction Support ---
  /** Sends a packet of pointer data to Flutter's engine. */
  @UiThread
  public void dispatchPointerDataPacket(@NonNull ByteBuffer buffer, int position) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeDispatchPointerDataPacket(nativeShellHolderId, buffer, position);
  }

  private native void nativeDispatchPointerDataPacket(
      long nativeShellHolderId, @NonNull ByteBuffer buffer, int position);
  // ------ End Touch Interaction Support ---

  @UiThread
  public void setPlatformViewsController(@NonNull PlatformViewsController platformViewsController) {
    ensureRunningOnMainThread();
    this.platformViewsController = platformViewsController;
  }

  // ------ Start Accessibility Support -----
  /**
   * Sets the {@link AccessibilityDelegate} for the attached Flutter context.
   *
   * <p>The {@link AccessibilityDelegate} is responsible for maintaining an Android-side cache of
   * Flutter's semantics tree and custom accessibility actions. This cache should be hooked up to
   * Android's accessibility system.
   *
   * <p>See {@link AccessibilityBridge} for an example of an {@link AccessibilityDelegate} and the
   * surrounding responsibilities.
   */
  @UiThread
  public void setAccessibilityDelegate(@Nullable AccessibilityDelegate accessibilityDelegate) {
    ensureRunningOnMainThread();
    this.accessibilityDelegate = accessibilityDelegate;
  }

  /**
   * Invoked by native to send semantics tree updates from Flutter to Android.
   *
   * <p>The {@code buffer} and {@code strings} form a communication protocol that is implemented
   * here:
   * https://github.com/flutter/engine/blob/main/shell/platform/android/platform_view_android.cc#L207
   */
  @SuppressWarnings("unused")
  @UiThread
  private void updateSemantics(
      @NonNull ByteBuffer buffer,
      @NonNull String[] strings,
      @NonNull ByteBuffer[] stringAttributeArgs) {
    ensureRunningOnMainThread();
    if (accessibilityDelegate != null) {
      accessibilityDelegate.updateSemantics(buffer, strings, stringAttributeArgs);
    }
    // TODO(mattcarroll): log dropped messages when in debug mode
    // (https://github.com/flutter/flutter/issues/25391)
  }

  /**
   * Invoked by native to send new custom accessibility events from Flutter to Android.
   *
   * <p>The {@code buffer} and {@code strings} form a communication protocol that is implemented
   * here:
   * https://github.com/flutter/engine/blob/main/shell/platform/android/platform_view_android.cc#L207
   *
   * <p>// TODO(cbracken): expand these docs to include more actionable information.
   */
  @SuppressWarnings("unused")
  @UiThread
  private void updateCustomAccessibilityActions(
      @NonNull ByteBuffer buffer, @NonNull String[] strings) {
    ensureRunningOnMainThread();
    if (accessibilityDelegate != null) {
      accessibilityDelegate.updateCustomAccessibilityActions(buffer, strings);
    }
    // TODO(mattcarroll): log dropped messages when in debug mode
    // (https://github.com/flutter/flutter/issues/25391)
  }

  /** Sends a semantics action to Flutter's engine, without any additional arguments. */
  public void dispatchSemanticsAction(int nodeId, @NonNull AccessibilityBridge.Action action) {
    dispatchSemanticsAction(nodeId, action, null);
  }

  /** Sends a semantics action to Flutter's engine, with additional arguments. */
  public void dispatchSemanticsAction(
      int nodeId, @NonNull AccessibilityBridge.Action action, @Nullable Object args) {
    ensureAttachedToNative();

    ByteBuffer encodedArgs = null;
    int position = 0;
    if (args != null) {
      encodedArgs = StandardMessageCodec.INSTANCE.encodeMessage(args);
      position = encodedArgs.position();
    }
    dispatchSemanticsAction(nodeId, action.value, encodedArgs, position);
  }

  /**
   * Sends a semantics action to Flutter's engine, given arguments that are already encoded for the
   * engine.
   *
   * <p>To send a semantics action that has not already been encoded, see {@link
   * #dispatchSemanticsAction(int, AccessibilityBridge.Action)} and {@link
   * #dispatchSemanticsAction(int, AccessibilityBridge.Action, Object)}.
   */
  @UiThread
  public void dispatchSemanticsAction(
      int nodeId, int action, @Nullable ByteBuffer args, int argsPosition) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeDispatchSemanticsAction(nativeShellHolderId, nodeId, action, args, argsPosition);
  }

  private native void nativeDispatchSemanticsAction(
      long nativeShellHolderId,
      int nodeId,
      int action,
      @Nullable ByteBuffer args,
      int argsPosition);

  /**
   * Instructs Flutter to enable/disable its semantics tree, which is used by Flutter to support
   * accessibility and related behaviors.
   */
  @UiThread
  public void setSemanticsEnabled(boolean enabled) {
    ensureRunningOnMainThread();
    if (isAttached()) {
      setSemanticsEnabledInNative(enabled);
    }
  }

  @VisibleForTesting
  public void setSemanticsEnabledInNative(boolean enabled) {
    nativeSetSemanticsEnabled(nativeShellHolderId, enabled);
  }

  private native void nativeSetSemanticsEnabled(long nativeShellHolderId, boolean enabled);

  // TODO(mattcarroll): figure out what flags are supported and add javadoc about when/why/where to
  // use this.
  @UiThread
  public void setAccessibilityFeatures(int flags) {
    ensureRunningOnMainThread();
    if (isAttached()) {
      setAccessibilityFeaturesInNative(flags);
    }
  }

  @VisibleForTesting
  public void setAccessibilityFeaturesInNative(int flags) {
    nativeSetAccessibilityFeatures(nativeShellHolderId, flags);
  }

  private native void nativeSetAccessibilityFeatures(long nativeShellHolderId, int flags);
  // ------ End Accessibility Support ----

  // ------ Start Texture Registration Support -----
  /**
   * Gives control of a {@link SurfaceTexture} to Flutter so that Flutter can display that texture
   * within Flutter's UI.
   */
  @UiThread
  public void registerTexture(long textureId, @NonNull SurfaceTextureWrapper textureWrapper) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeRegisterTexture(
        nativeShellHolderId, textureId, new WeakReference<SurfaceTextureWrapper>(textureWrapper));
  }

  private native void nativeRegisterTexture(
      long nativeShellHolderId,
      long textureId,
      @NonNull WeakReference<SurfaceTextureWrapper> textureWrapper);

  /**
   * Registers a ImageTexture with the given id.
   *
   * <p>REQUIRED: Callers should eventually unregisterTexture with the same id.
   */
  @UiThread
  public void registerImageTexture(
      long textureId, @NonNull TextureRegistry.ImageConsumer imageTexture) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeRegisterImageTexture(
        nativeShellHolderId,
        textureId,
        new WeakReference<TextureRegistry.ImageConsumer>(imageTexture));
  }

  private native void nativeRegisterImageTexture(
      long nativeShellHolderId,
      long textureId,
      @NonNull WeakReference<TextureRegistry.ImageConsumer> imageTexture);

  /**
   * Call this method to inform Flutter that a texture previously registered with {@link
   * #registerTexture(long, SurfaceTextureWrapper)} has a new frame available.
   *
   * <p>Invoking this method instructs Flutter to update its presentation of the given texture so
   * that the new frame is displayed.
   */
  @UiThread
  public void markTextureFrameAvailable(long textureId) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeMarkTextureFrameAvailable(nativeShellHolderId, textureId);
  }

  private native void nativeMarkTextureFrameAvailable(long nativeShellHolderId, long textureId);

  /** Schedule the engine to draw a frame but does not invalidate the layout tree. */
  @UiThread
  public void scheduleFrame() {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeScheduleFrame(nativeShellHolderId);
  }

  private native void nativeScheduleFrame(long nativeShellHolderId);

  /**
   * Unregisters a texture that was registered with {@link #registerTexture(long,
   * SurfaceTextureWrapper)}.
   */
  @UiThread
  public void unregisterTexture(long textureId) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeUnregisterTexture(nativeShellHolderId, textureId);
  }

  private native void nativeUnregisterTexture(long nativeShellHolderId, long textureId);
  // ------ Start Texture Registration Support -----

  // ------ Start Dart Execution Support -------
  /**
   * Executes a Dart entrypoint.
   *
   * <p>This can only be done once per JNI attachment because a Dart isolate can only be entered
   * once.
   */
  @UiThread
  public void runBundleAndSnapshotFromLibrary(
      @NonNull String bundlePath,
      @Nullable String entrypointFunctionName,
      @Nullable String pathToEntrypointFunction,
      @NonNull AssetManager assetManager,
      @Nullable List<String> entrypointArgs) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeRunBundleAndSnapshotFromLibrary(
        nativeShellHolderId,
        bundlePath,
        entrypointFunctionName,
        pathToEntrypointFunction,
        assetManager,
        entrypointArgs);
  }

  private native void nativeRunBundleAndSnapshotFromLibrary(
      long nativeShellHolderId,
      @NonNull String bundlePath,
      @Nullable String entrypointFunctionName,
      @Nullable String pathToEntrypointFunction,
      @NonNull AssetManager manager,
      @Nullable List<String> entrypointArgs);
  // ------ End Dart Execution Support -------

  // --------- Start Platform Message Support ------
  /**
   * Sets the handler for all platform messages that come from the attached platform view to Java.
   *
   * <p>Communication between a specific Flutter context (Dart) and the host platform (Java) is
   * accomplished by passing messages. Messages can be sent from Java to Dart with the corresponding
   * {@code FlutterJNI} methods:
   *
   * <ul>
   *   <li>{@link #dispatchPlatformMessage(String, ByteBuffer, int, int)}
   *   <li>{@link #dispatchEmptyPlatformMessage(String, int)}
   * </ul>
   *
   * <p>{@code FlutterJNI} is also the recipient of all platform messages sent from its attached
   * Flutter context. {@code FlutterJNI} does not know what to do with these messages, so a handler
   * is exposed to allow these messages to be processed in whatever manner is desired:
   *
   * <p>{@code setPlatformMessageHandler(PlatformMessageHandler)}
   *
   * <p>If a message is received but no {@link PlatformMessageHandler} is registered, that message
   * will be dropped (ignored). Therefore, when using {@code FlutterJNI} to integrate a Flutter
   * context in an app, a {@link PlatformMessageHandler} must be registered for 2-way Java/Dart
   * communication to operate correctly. Moreover, the handler must be implemented such that
   * fundamental platform messages are handled as expected.
   */
  @UiThread
  public void setPlatformMessageHandler(@Nullable PlatformMessageHandler platformMessageHandler) {
    ensureRunningOnMainThread();
    this.platformMessageHandler = platformMessageHandler;
  }

  private native void nativeCleanupMessageData(long messageData);

  /**
   * Destroys the resources provided sent to `handlePlatformMessage`.
   *
   * <p>This can be called on any thread.
   *
   * @param messageData the argument sent to handlePlatformMessage.
   */
  public void cleanupMessageData(long messageData) {
    // This doesn't rely on being attached like other methods.
    nativeCleanupMessageData(messageData);
  }

  // Called by native on any thread.
  // TODO(mattcarroll): determine if message is nonull or nullable
  @SuppressWarnings("unused")
  @VisibleForTesting
  public void handlePlatformMessage(
      @NonNull final String channel,
      ByteBuffer message,
      final int replyId,
      final long messageData) {
    if (platformMessageHandler != null) {
      platformMessageHandler.handleMessageFromDart(channel, message, replyId, messageData);
    } else {
      nativeCleanupMessageData(messageData);
    }
    // TODO(mattcarroll): log dropped messages when in debug mode
    // (https://github.com/flutter/flutter/issues/25391)
  }

  // Called by native to respond to a platform message that we sent.
  // TODO(mattcarroll): determine if reply is nonull or nullable
  @SuppressWarnings("unused")
  private void handlePlatformMessageResponse(int replyId, ByteBuffer reply) {
    if (platformMessageHandler != null) {
      platformMessageHandler.handlePlatformMessageResponse(replyId, reply);
    }
    // TODO(mattcarroll): log dropped messages when in debug mode
    // (https://github.com/flutter/flutter/issues/25391)
  }

  /**
   * Sends an empty reply (identified by {@code responseId}) from Android to Flutter over the given
   * {@code channel}.
   */
  @UiThread
  public void dispatchEmptyPlatformMessage(@NonNull String channel, int responseId) {
    ensureRunningOnMainThread();
    if (isAttached()) {
      nativeDispatchEmptyPlatformMessage(nativeShellHolderId, channel, responseId);
    } else {
      Log.w(
          TAG,
          "Tried to send a platform message to Flutter, but FlutterJNI was detached from native C++. Could not send. Channel: "
              + channel
              + ". Response ID: "
              + responseId);
    }
  }

  // Send an empty platform message to Dart.
  private native void nativeDispatchEmptyPlatformMessage(
      long nativeShellHolderId, @NonNull String channel, int responseId);

  /** Sends a reply {@code message} from Android to Flutter over the given {@code channel}. */
  @UiThread
  public void dispatchPlatformMessage(
      @NonNull String channel, @Nullable ByteBuffer message, int position, int responseId) {
    ensureRunningOnMainThread();
    if (isAttached()) {
      nativeDispatchPlatformMessage(nativeShellHolderId, channel, message, position, responseId);
    } else {
      Log.w(
          TAG,
          "Tried to send a platform message to Flutter, but FlutterJNI was detached from native C++. Could not send. Channel: "
              + channel
              + ". Response ID: "
              + responseId);
    }
  }

  // Send a data-carrying platform message to Dart.
  private native void nativeDispatchPlatformMessage(
      long nativeShellHolderId,
      @NonNull String channel,
      @Nullable ByteBuffer message,
      int position,
      int responseId);

  // TODO(mattcarroll): differentiate between channel responses and platform responses.
  public void invokePlatformMessageEmptyResponseCallback(int responseId) {
    // Called on any thread.
    shellHolderLock.readLock().lock();
    try {
      if (isAttached()) {
        nativeInvokePlatformMessageEmptyResponseCallback(nativeShellHolderId, responseId);
      } else {
        Log.w(
            TAG,
            "Tried to send a platform message response, but FlutterJNI was detached from native C++. Could not send. Response ID: "
                + responseId);
      }
    } finally {
      shellHolderLock.readLock().unlock();
    }
  }

  // Send an empty response to a platform message received from Dart.
  private native void nativeInvokePlatformMessageEmptyResponseCallback(
      long nativeShellHolderId, int responseId);

  // TODO(mattcarroll): differentiate between channel responses and platform responses.
  public void invokePlatformMessageResponseCallback(
      int responseId, @NonNull ByteBuffer message, int position) {
    // Called on any thread.
    if (!message.isDirect()) {
      throw new IllegalArgumentException("Expected a direct ByteBuffer.");
    }
    shellHolderLock.readLock().lock();
    try {
      if (isAttached()) {
        nativeInvokePlatformMessageResponseCallback(
            nativeShellHolderId, responseId, message, position);
      } else {
        Log.w(
            TAG,
            "Tried to send a platform message response, but FlutterJNI was detached from native C++. Could not send. Response ID: "
                + responseId);
      }
    } finally {
      shellHolderLock.readLock().unlock();
    }
  }

  // Send a data-carrying response to a platform message received from Dart.
  private native void nativeInvokePlatformMessageResponseCallback(
      long nativeShellHolderId, int responseId, @Nullable ByteBuffer message, int position);
  // ------- End Platform Message Support ----

  // ----- Start Engine Lifecycle Support ----
  /**
   * Adds the given {@code engineLifecycleListener} to be notified of Flutter engine lifecycle
   * events, e.g., {@link EngineLifecycleListener#onPreEngineRestart()}.
   */
  @UiThread
  public void addEngineLifecycleListener(@NonNull EngineLifecycleListener engineLifecycleListener) {
    ensureRunningOnMainThread();
    engineLifecycleListeners.add(engineLifecycleListener);
  }

  /**
   * Removes the given {@code engineLifecycleListener}, which was previously added using {@link
   * #addIsDisplayingFlutterUiListener(FlutterUiDisplayListener)}.
   */
  @UiThread
  public void removeEngineLifecycleListener(
      @NonNull EngineLifecycleListener engineLifecycleListener) {
    ensureRunningOnMainThread();
    engineLifecycleListeners.remove(engineLifecycleListener);
  }

  // Called by native.
  @SuppressWarnings("unused")
  private void onPreEngineRestart() {
    for (EngineLifecycleListener listener : engineLifecycleListeners) {
      listener.onPreEngineRestart();
    }
  }

  @SuppressWarnings("unused")
  @UiThread
  public void onDisplayOverlaySurface(int id, int x, int y, int width, int height) {
    ensureRunningOnMainThread();
    if (platformViewsController == null) {
      throw new RuntimeException(
          "platformViewsController must be set before attempting to position an overlay surface");
    }
    platformViewsController.onDisplayOverlaySurface(id, x, y, width, height);
  }

  @SuppressWarnings("unused")
  @UiThread
  public void onBeginFrame() {
    ensureRunningOnMainThread();
    if (platformViewsController == null) {
      throw new RuntimeException(
          "platformViewsController must be set before attempting to begin the frame");
    }
    platformViewsController.onBeginFrame();
  }

  @SuppressWarnings("unused")
  @UiThread
  public void onEndFrame() {
    ensureRunningOnMainThread();
    if (platformViewsController == null) {
      throw new RuntimeException(
          "platformViewsController must be set before attempting to end the frame");
    }
    platformViewsController.onEndFrame();
  }

  @SuppressWarnings("unused")
  @UiThread
  public FlutterOverlaySurface createOverlaySurface() {
    ensureRunningOnMainThread();
    if (platformViewsController == null) {
      throw new RuntimeException(
          "platformViewsController must be set before attempting to position an overlay surface");
    }
    return platformViewsController.createOverlaySurface();
  }

  @SuppressWarnings("unused")
  @UiThread
  public void destroyOverlaySurfaces() {
    ensureRunningOnMainThread();
    if (platformViewsController == null) {
      throw new RuntimeException(
          "platformViewsController must be set before attempting to destroy an overlay surface");
    }
    platformViewsController.destroyOverlaySurfaces();
  }
  // ----- End Engine Lifecycle Support ----

  // ----- Start Localization Support ----

  /** Sets the localization plugin that is used in various localization methods. */
  @UiThread
  public void setLocalizationPlugin(@Nullable LocalizationPlugin localizationPlugin) {
    ensureRunningOnMainThread();
    this.localizationPlugin = localizationPlugin;
  }

  /** Invoked by native to obtain the results of Android's locale resolution algorithm. */
  @SuppressWarnings("unused")
  @VisibleForTesting
  public String[] computePlatformResolvedLocale(@NonNull String[] strings) {
    if (localizationPlugin == null) {
      return new String[0];
    }
    List<Locale> supportedLocales = new ArrayList<Locale>();
    final int localeDataLength = 3;
    for (int i = 0; i < strings.length; i += localeDataLength) {
      String languageCode = strings[i + 0];
      String countryCode = strings[i + 1];
      String scriptCode = strings[i + 2];
      // Convert to Locales via LocaleBuilder if available (API 21+) to include scriptCode.
      Locale.Builder localeBuilder = new Locale.Builder();
      if (!languageCode.isEmpty()) {
        localeBuilder.setLanguage(languageCode);
      }
      if (!countryCode.isEmpty()) {
        localeBuilder.setRegion(countryCode);
      }
      if (!scriptCode.isEmpty()) {
        localeBuilder.setScript(scriptCode);
      }
      supportedLocales.add(localeBuilder.build());
    }

    Locale result = localizationPlugin.resolveNativeLocale(supportedLocales);

    if (result == null) {
      return new String[0];
    }
    String[] output = new String[localeDataLength];
    output[0] = result.getLanguage();
    output[1] = result.getCountry();
    output[2] = result.getScript();
    return output;
  }

  // ----- End Localization Support ----
  @Nullable
  public float getScaledFontSize(float fontSize, int configurationId) {
    final DisplayMetrics metrics = SettingsChannel.getPastDisplayMetrics(configurationId);
    if (metrics == null) {
      Log.e(
          TAG,
          "getScaledFontSize called with configurationId "
              + String.valueOf(configurationId)
              + ", which can't be found.");
      return -1f;
    }
    return TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_SP, fontSize, metrics)
        / metrics.density;
  }

  // ----- Start Deferred Components Support ----

  /** Sets the deferred component manager that is used to download and install split features. */
  @UiThread
  public void setDeferredComponentManager(
      @Nullable DeferredComponentManager deferredComponentManager) {
    ensureRunningOnMainThread();
    this.deferredComponentManager = deferredComponentManager;
    if (deferredComponentManager != null) {
      deferredComponentManager.setJNI(this);
    }
  }

  /**
   * Called by dart to request that a Dart deferred library corresponding to loadingUnitId be
   * downloaded (if necessary) and loaded into the dart vm.
   *
   * <p>This method delegates the task to DeferredComponentManager, which handles the download and
   * loading of the dart library and any assets.
   *
   * @param loadingUnitId The loadingUnitId is assigned during compile time by gen_snapshot and is
   *     automatically retrieved when loadLibrary() is called on a dart deferred library.
   */
  @SuppressWarnings("unused")
  @UiThread
  public void requestDartDeferredLibrary(int loadingUnitId) {
    if (deferredComponentManager != null) {
      deferredComponentManager.installDeferredComponent(loadingUnitId, null);
    } else {
      // TODO(garyq): Add link to setup/instructions guide wiki.
      Log.e(
          TAG,
          "No DeferredComponentManager found. Android setup must be completed before using split AOT deferred components.");
    }
  }

  /**
   * Searches each of the provided paths for a valid Dart shared library .so file and resolves
   * symbols to load into the dart VM.
   *
   * <p>Successful loading of the dart library completes the future returned by loadLibrary() that
   * triggered the install/load process.
   *
   * @param loadingUnitId The loadingUnitId is assigned during compile time by gen_snapshot and is
   *     automatically retrieved when loadLibrary() is called on a dart deferred library. This is
   *     used to identify which Dart deferred library the resolved correspond to.
   * @param searchPaths An array of paths in which to look for valid dart shared libraries. This
   *     supports paths within zipped apks as long as the apks are not compressed using the
   *     `path/to/apk.apk!path/inside/apk/lib.so` format. Paths will be tried first to last and ends
   *     when a library is successfully found. When the found library is invalid, no additional
   *     paths will be attempted.
   */
  @UiThread
  public void loadDartDeferredLibrary(int loadingUnitId, @NonNull String[] searchPaths) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeLoadDartDeferredLibrary(nativeShellHolderId, loadingUnitId, searchPaths);
  }

  private native void nativeLoadDartDeferredLibrary(
      long nativeShellHolderId, int loadingUnitId, @NonNull String[] searchPaths);

  /**
   * Adds the specified AssetManager as an APKAssetResolver in the Flutter Engine's AssetManager.
   *
   * <p>This may be used to update the engine AssetManager when a new deferred component is
   * installed and a new Android AssetManager is created with access to new assets.
   *
   * @param assetManager An android AssetManager that is able to access the newly downloaded assets.
   * @param assetBundlePath The subdirectory that the flutter assets are stored in. The typical
   *     value is `flutter_assets`.
   */
  @UiThread
  public void updateJavaAssetManager(
      @NonNull AssetManager assetManager, @NonNull String assetBundlePath) {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeUpdateJavaAssetManager(nativeShellHolderId, assetManager, assetBundlePath);
  }

  private native void nativeUpdateJavaAssetManager(
      long nativeShellHolderId,
      @NonNull AssetManager assetManager,
      @NonNull String assetBundlePath);

  /**
   * Indicates that a failure was encountered during the Android portion of downloading a dynamic
   * feature module and loading a dart deferred library, which is typically done by
   * DeferredComponentManager.
   *
   * <p>This will inform dart that the future returned by loadLibrary() should complete with an
   * error.
   *
   * @param loadingUnitId The loadingUnitId that corresponds to the dart deferred library that
   *     failed to install.
   * @param error The error message to display.
   * @param isTransient When isTransient is false, new attempts to install will automatically result
   *     in same error in Dart before the request is passed to Android.
   */
  @SuppressWarnings("unused")
  @UiThread
  public void deferredComponentInstallFailure(
      int loadingUnitId, @NonNull String error, boolean isTransient) {
    ensureRunningOnMainThread();
    nativeDeferredComponentInstallFailure(loadingUnitId, error, isTransient);
  }

  private native void nativeDeferredComponentInstallFailure(
      int loadingUnitId, @NonNull String error, boolean isTransient);

  // ----- End Deferred Components Support ----

  // @SuppressWarnings("unused")
  @UiThread
  public void onDisplayPlatformView(
      int viewId,
      int x,
      int y,
      int width,
      int height,
      int viewWidth,
      int viewHeight,
      FlutterMutatorsStack mutatorsStack) {
    ensureRunningOnMainThread();
    if (platformViewsController == null) {
      throw new RuntimeException(
          "platformViewsController must be set before attempting to position a platform view");
    }
    platformViewsController.onDisplayPlatformView(
        viewId, x, y, width, height, viewWidth, viewHeight, mutatorsStack);
  }

  // TODO(mattcarroll): determine if this is nonull or nullable
  @UiThread
  public Bitmap getBitmap() {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    return nativeGetBitmap(nativeShellHolderId);
  }

  // TODO(mattcarroll): determine if this is nonull or nullable
  private native Bitmap nativeGetBitmap(long nativeShellHolderId);

  /**
   * Notifies the Dart VM of a low memory event, or that the application is in a state such that now
   * is an appropriate time to free resources, such as going to the background.
   *
   * <p>This is distinct from sending a SystemChannel message about low memory, which only notifies
   * the running Flutter application.
   */
  @UiThread
  public void notifyLowMemoryWarning() {
    ensureRunningOnMainThread();
    ensureAttachedToNative();
    nativeNotifyLowMemoryWarning(nativeShellHolderId);
  }

  private native void nativeNotifyLowMemoryWarning(long nativeShellHolderId);

  private void ensureRunningOnMainThread() {
    if (Looper.myLooper() != mainLooper) {
      throw new RuntimeException(
          "Methods marked with @UiThread must be executed on the main thread. Current thread: "
              + Thread.currentThread().getName());
    }
  }

  /**
   * Delegate responsible for creating and updating Android-side caches of Flutter's semantics tree
   * and custom accessibility actions.
   *
   * <p>{@link AccessibilityBridge} is an example of an {@code AccessibilityDelegate}.
   */
  public interface AccessibilityDelegate {
    /**
     * Sends new custom accessibility actions from Flutter to Android.
     *
     * <p>Implementers are expected to maintain an Android-side cache of custom accessibility
     * actions. This method provides new actions to add to that cache.
     */
    void updateCustomAccessibilityActions(@NonNull ByteBuffer buffer, @NonNull String[] strings);

    /**
     * Sends new {@code SemanticsNode} information from Flutter to Android.
     *
     * <p>Implementers are expected to maintain an Android-side cache of Flutter's semantics tree.
     * This method provides updates from Flutter for the Android-side semantics tree cache.
     */
    void updateSemantics(
        @NonNull ByteBuffer buffer,
        @NonNull String[] strings,
        @NonNull ByteBuffer[] stringAttributeArgs);
  }

  public interface AsyncWaitForVsyncDelegate {
    void asyncWaitForVsync(final long cookie);
  }

  /**
   * Whether Android Hardware Buffer import is known to not work on this particular vendor + API
   * level and should be disabled.
   */
  public boolean ShouldDisableAHB() {
    return nativeShouldDisableAHB();
  }

  private native boolean nativeShouldDisableAHB();
}
