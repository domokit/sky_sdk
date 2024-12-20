// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine.renderer;

import static android.content.ComponentCallbacks2.TRIM_MEMORY_BACKGROUND;
import static android.content.ComponentCallbacks2.TRIM_MEMORY_COMPLETE;
import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyFloat;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.robolectric.Shadows.shadowOf;

import android.graphics.Canvas;
import android.graphics.Rect;
import android.graphics.SurfaceTexture;
import android.media.Image;
import android.os.Looper;
import android.view.Surface;
import androidx.lifecycle.Lifecycle;
import androidx.lifecycle.LifecycleRegistry;
import androidx.lifecycle.ProcessLifecycleOwner;
import androidx.test.ext.junit.rules.ActivityScenarioRule;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.android.FlutterActivity;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.view.TextureRegistry;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.Before;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterRendererTest {
  @Rule(order = 1)
  public final FlutterEngineRule engineRule = new FlutterEngineRule();

  @Rule(order = 2)
  public final ActivityScenarioRule<FlutterActivity> scenarioRule =
      new ActivityScenarioRule<>(engineRule.makeIntent());

  private FlutterJNI fakeFlutterJNI;

  @Before
  public void init() {
    // Uncomment the following line to enable logging output in test.
    // ShadowLog.stream = System.out;
  }

  @Before
  public void setup() {
    fakeFlutterJNI = engineRule.getFlutterJNI();
  }

  @Test
  public void itForwardsSurfaceCreationNotificationToFlutterJNI() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    // Execute the behavior under test.
    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Verify the behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceCreated(eq(fakeSurface));
  }

  @Test
  public void itForwardsSurfaceChangeNotificationToFlutterJNI() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Execute the behavior under test.
    flutterRenderer.surfaceChanged(100, 50);

    // Verify the behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceChanged(eq(100), eq(50));
  }

  @Test
  public void itForwardsSurfaceDestructionNotificationToFlutterJNI() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Execute the behavior under test.
    flutterRenderer.stopRenderingToSurface();

    // Verify the behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceDestroyed();
  }

  @Test
  public void itStopsRenderingToOneSurfaceBeforeRenderingToANewSurface() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    Surface fakeSurface2 = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Execute behavior under test.
    flutterRenderer.startRenderingToSurface(fakeSurface2, false);

    // Verify behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceDestroyed(); // notification of 1st surface's removal.
  }

  @Test
  public void itStopsRenderingToSurfaceWhenRequested() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Execute the behavior under test.
    flutterRenderer.stopRenderingToSurface();

    // Verify behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceDestroyed();
  }

  @Test
  public void iStopsRenderingToSurfaceWhenSurfaceAlreadySet() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);
    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Verify behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceDestroyed();
  }

  @Test
  public void itNeverStopsRenderingToSurfaceWhenRequested() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);
    flutterRenderer.startRenderingToSurface(fakeSurface, true);

    // Verify behavior under test.
    verify(fakeFlutterJNI, never()).onSurfaceDestroyed();
  }

  @Test
  public void itStopsSurfaceTextureCallbackWhenDetached() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry) flutterRenderer.createSurfaceTexture();
    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Execute the behavior under test.
    flutterRenderer.stopRenderingToSurface();

    // Verify behavior under test.
    verify(fakeFlutterJNI, times(0)).markTextureFrameAvailable(eq(entry.id()));
  }

  @Test
  public void itRegistersExistingSurfaceTexture() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    SurfaceTexture surfaceTexture = new SurfaceTexture(0);

    // Execute the behavior under test.
    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry)
            flutterRenderer.registerSurfaceTexture(surfaceTexture);

    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Verify behavior under test.
    assertEquals(surfaceTexture, entry.surfaceTexture());

    verify(fakeFlutterJNI, times(1)).registerTexture(eq(entry.id()), eq(entry.textureWrapper()));
  }

  @Test
  public void itUnregistersTextureWhenSurfaceTextureFinalized() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry) flutterRenderer.createSurfaceTexture();
    long id = entry.id();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Execute the behavior under test.
    runFinalization(entry);

    shadowOf(Looper.getMainLooper()).idle();

    flutterRenderer.stopRenderingToSurface();

    // Verify behavior under test.
    verify(fakeFlutterJNI, times(1)).unregisterTexture(eq(id));
  }

  @Test
  public void itStopsUnregisteringTextureWhenDetached() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    engineRule.setJniIsAttached(false);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry) flutterRenderer.createSurfaceTexture();
    long id = entry.id();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);
    flutterRenderer.stopRenderingToSurface();

    // Execute the behavior under test.
    runFinalization(entry);

    shadowOf(Looper.getMainLooper()).idle();

    // Verify behavior under test.
    verify(fakeFlutterJNI, times(0)).unregisterTexture(eq(id));
  }

  /** @noinspection FinalizeCalledExplicitly */
  void runFinalization(FlutterRenderer.SurfaceTextureRegistryEntry entry) {
    CountDownLatch latch = new CountDownLatch(1);
    Thread fakeFinalizer =
        new Thread(
            () -> {
              try {
                entry.finalize();
                latch.countDown();
              } catch (Throwable e) {
                // do nothing
              }
            });
    fakeFinalizer.start();
    try {
      latch.await();
    } catch (InterruptedException e) {
      // do nothing
    }
  }

  @Test
  public void itConvertsDisplayFeatureArrayToPrimitiveArrays() {
    // Intentionally do not use 'engineRule' in this test, because we are testing a very narrow
    // API (the side-effects of 'setViewportMetrics'). Under normal construction, the engine will
    // invoke 'setViewportMetrics' a number of times automatically, making testing the side-effects
    // of the method call more difficult than needed.
    FlutterJNI fakeFlutterJNI = mock(FlutterJNI.class);
    FlutterRenderer flutterRenderer = new FlutterRenderer(fakeFlutterJNI);

    // Setup the test.
    FlutterRenderer.ViewportMetrics metrics = new FlutterRenderer.ViewportMetrics();
    metrics.width = 1000;
    metrics.height = 1000;
    metrics.devicePixelRatio = 2;
    metrics
        .getDisplayFeatures()
        .add(
            new FlutterRenderer.DisplayFeature(
                new Rect(10, 20, 30, 40),
                FlutterRenderer.DisplayFeatureType.FOLD,
                FlutterRenderer.DisplayFeatureState.POSTURE_HALF_OPENED));
    metrics
        .getDisplayCutouts()
        .add(
            new FlutterRenderer.DisplayFeature(
                new Rect(50, 60, 70, 80),
                FlutterRenderer.DisplayFeatureType.CUTOUT,
                FlutterRenderer.DisplayFeatureState.UNKNOWN));

    // Execute the behavior under test.
    flutterRenderer.setViewportMetrics(metrics);

    // Verify behavior under test.
    ArgumentCaptor<int[]> boundsCaptor = ArgumentCaptor.forClass(int[].class);
    ArgumentCaptor<int[]> typeCaptor = ArgumentCaptor.forClass(int[].class);
    ArgumentCaptor<int[]> stateCaptor = ArgumentCaptor.forClass(int[].class);
    verify(fakeFlutterJNI)
        .setViewportMetrics(
            anyFloat(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            anyInt(),
            boundsCaptor.capture(),
            typeCaptor.capture(),
            stateCaptor.capture());

    assertArrayEquals(new int[] {10, 20, 30, 40, 50, 60, 70, 80}, boundsCaptor.getValue());
    assertArrayEquals(
        new int[] {
          FlutterRenderer.DisplayFeatureType.FOLD.encodedValue,
          FlutterRenderer.DisplayFeatureType.CUTOUT.encodedValue
        },
        typeCaptor.getValue());
    assertArrayEquals(
        new int[] {
          FlutterRenderer.DisplayFeatureState.POSTURE_HALF_OPENED.encodedValue,
          FlutterRenderer.DisplayFeatureState.UNKNOWN.encodedValue
        },
        stateCaptor.getValue());
  }

  @Test
  public void itNotifyImageFrameListener() {
    // Setup the test.
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    AtomicInteger invocationCount = new AtomicInteger(0);
    final TextureRegistry.OnFrameConsumedListener listener = invocationCount::incrementAndGet;

    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry) flutterRenderer.createSurfaceTexture();
    entry.setOnFrameConsumedListener(listener);

    // Execute the behavior under test.
    entry.textureWrapper().updateTexImage();

    // Verify behavior under test.
    assertEquals(1, invocationCount.get());
  }

  @Test
  public void itAddsListenerWhenSurfaceTextureEntryCreated() {
    // Setup the test.
    FlutterRenderer flutterRenderer = spy(engineRule.getFlutterEngine().getRenderer());

    // Execute the behavior under test.
    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry) flutterRenderer.createSurfaceTexture();

    // Verify behavior under test.
    verify(flutterRenderer, times(1)).addOnTrimMemoryListener(entry);
  }

  @Test
  public void itRemovesListenerWhenSurfaceTextureEntryReleased() {
    // Setup the test.
    FlutterRenderer flutterRenderer = spy(engineRule.getFlutterEngine().getRenderer());
    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry) flutterRenderer.createSurfaceTexture();

    // Execute the behavior under test.
    entry.release();

    // Verify behavior under test.
    verify(flutterRenderer, times(1)).removeOnTrimMemoryListener(entry);
  }

  @Test
  @SuppressWarnings("deprecation")
  // TRIM_MEMORY_COMPLETE
  public void itNotifySurfaceTextureEntryWhenMemoryPressureWarning() {
    // Setup the test.
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    AtomicInteger invocationCount = new AtomicInteger(0);
    final TextureRegistry.OnTrimMemoryListener listener =
        level -> invocationCount.incrementAndGet();

    FlutterRenderer.SurfaceTextureRegistryEntry entry =
        (FlutterRenderer.SurfaceTextureRegistryEntry) flutterRenderer.createSurfaceTexture();
    entry.setOnTrimMemoryListener(listener);

    // Execute the behavior under test.
    flutterRenderer.onTrimMemory(TRIM_MEMORY_COMPLETE);

    // Verify behavior under test.
    assertEquals(1, invocationCount.get());
  }

  @Test
  public void itDoesDispatchSurfaceDestructionNotificationOnlyOnce() {
    // Setup the test.
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);

    // Execute the behavior under test.
    // Simulate calling |FlutterRenderer#stopRenderingToSurface| twice with different code paths.
    flutterRenderer.stopRenderingToSurface();
    flutterRenderer.stopRenderingToSurface();

    // Verify behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceDestroyed();
  }

  @Test
  public void itInvokesCreatesSurfaceWhenStartingRendering() {
    Surface fakeSurface = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    flutterRenderer.startRenderingToSurface(fakeSurface, false);
    verify(fakeFlutterJNI, times(1)).onSurfaceCreated(eq(fakeSurface));
  }

  @Test
  public void itDoesNotInvokeCreatesSurfaceWhenResumingRendering() {
    Surface fakeSurface = mock(Surface.class);
    Surface fakeSurface2 = mock(Surface.class);
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();

    // The following call sequence mimics the behaviour of FlutterView when it exits from hybrid
    // composition mode.

    // Install initial rendering surface.
    flutterRenderer.startRenderingToSurface(fakeSurface, false);
    verify(fakeFlutterJNI, times(1)).onSurfaceCreated(eq(fakeSurface));
    verify(fakeFlutterJNI, times(0)).onSurfaceWindowChanged(eq(fakeSurface));

    // Install the image view.
    flutterRenderer.startRenderingToSurface(fakeSurface2, true);
    verify(fakeFlutterJNI, times(1)).onSurfaceCreated(eq(fakeSurface));
    verify(fakeFlutterJNI, times(0)).onSurfaceWindowChanged(eq(fakeSurface));
    verify(fakeFlutterJNI, times(0)).onSurfaceCreated(eq(fakeSurface2));
    verify(fakeFlutterJNI, times(1)).onSurfaceWindowChanged(eq(fakeSurface2));

    flutterRenderer.startRenderingToSurface(fakeSurface, true);
    verify(fakeFlutterJNI, times(1)).onSurfaceCreated(eq(fakeSurface));
    verify(fakeFlutterJNI, times(1)).onSurfaceWindowChanged(eq(fakeSurface));
    verify(fakeFlutterJNI, times(0)).onSurfaceCreated(eq(fakeSurface2));
    verify(fakeFlutterJNI, times(1)).onSurfaceWindowChanged(eq(fakeSurface2));
  }

  @Test
  public void ImageReaderSurfaceProducerProducesImageOfCorrectSize() {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();
    FlutterRenderer.ImageReaderSurfaceProducer texture =
        (FlutterRenderer.ImageReaderSurfaceProducer) producer;
    texture.disableFenceForTest();

    // Returns a null image when one hasn't been produced.
    assertNull(texture.acquireLatestImage());

    // Give the texture an initial size.
    texture.setSize(1, 1);

    // Render a frame.
    Surface surface = texture.getSurface();
    assertNotNull(surface);
    Canvas canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Let callbacks run.
    shadowOf(Looper.getMainLooper()).idle();

    // Extract the image and check its size.
    Image image = texture.acquireLatestImage();
    assert image != null;
    assertEquals(1, image.getWidth());
    assertEquals(1, image.getHeight());
    image.close();

    // Resize the texture.
    texture.setSize(5, 5);

    // Render a frame.
    surface = texture.getSurface();
    assertNotNull(surface);
    canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Let callbacks run.
    shadowOf(Looper.getMainLooper()).idle();

    // Extract the image and check its size.
    image = texture.acquireLatestImage();
    assert image != null;
    assertEquals(5, image.getWidth());
    assertEquals(5, image.getHeight());
    image.close();

    assertNull(texture.acquireLatestImage());

    texture.release();
  }

  @Test
  public void ImageReaderSurfaceProducerDoesNotDropFramesWhenResizeInFlight() {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();
    FlutterRenderer.ImageReaderSurfaceProducer texture =
        (FlutterRenderer.ImageReaderSurfaceProducer) producer;
    texture.disableFenceForTest();

    // Returns a null image when one hasn't been produced.
    assertNull(texture.acquireLatestImage());

    // Give the texture an initial size.
    texture.setSize(1, 1);

    // Render a frame.
    Surface surface = texture.getSurface();
    assertNotNull(surface);
    Canvas canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Resize.
    texture.setSize(4, 4);

    // Let callbacks run. The rendered frame will manifest here.
    shadowOf(Looper.getMainLooper()).idle();

    // We acquired the frame produced above.
    assertNotNull(texture.acquireLatestImage());
  }

  @Test
  public void ImageReaderSurfaceProducerImageReadersAndImagesCount() {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();
    FlutterRenderer.ImageReaderSurfaceProducer texture =
        (FlutterRenderer.ImageReaderSurfaceProducer) producer;
    texture.disableFenceForTest();

    // Returns a null image when one hasn't been produced.
    assertNull(texture.acquireLatestImage());

    // Give the texture an initial size.
    texture.setSize(1, 1);

    // Grab the surface so we can render a frame at 1x1 after resizing.
    Surface surface = texture.getSurface();
    assertNotNull(surface);
    Canvas canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Let callbacks run, this will produce a single frame.
    shadowOf(Looper.getMainLooper()).idle();

    assertEquals(1, texture.numImageReaders());
    assertEquals(1, texture.numImages());

    // Resize.
    texture.setSize(4, 4);

    // Render a frame at the old size (by using the pre-resized Surface)
    canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Let callbacks run.
    shadowOf(Looper.getMainLooper()).idle();

    assertEquals(1, texture.numImageReaders());
    assertEquals(2, texture.numImages());

    // Render a new frame with the current size.
    surface = texture.getSurface();
    assertNotNull(surface);
    canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Let callbacks run.
    shadowOf(Looper.getMainLooper()).idle();

    assertEquals(2, texture.numImageReaders());
    assertEquals(3, texture.numImages());

    // Acquire first frame.
    Image produced = texture.acquireLatestImage();
    assertNotNull(produced);
    assertEquals(1, produced.getWidth());
    assertEquals(1, produced.getHeight());
    assertEquals(2, texture.numImageReaders());
    assertEquals(2, texture.numImages());
    // Acquire second frame. This won't result in the first reader being closed because it has
    // an active image from it.
    produced = texture.acquireLatestImage();
    assertNotNull(produced);
    assertEquals(1, produced.getWidth());
    assertEquals(1, produced.getHeight());
    assertEquals(2, texture.numImageReaders());
    assertEquals(1, texture.numImages());
    // Acquire third frame. We will now close the first reader.
    produced = texture.acquireLatestImage();
    assertNotNull(produced);
    assertEquals(4, produced.getWidth());
    assertEquals(4, produced.getHeight());
    assertEquals(1, texture.numImageReaders());
    assertEquals(0, texture.numImages());

    // Returns null image when no more images are queued.
    assertNull(texture.acquireLatestImage());
    assertEquals(1, texture.numImageReaders());
    assertEquals(0, texture.numImages());
  }

  @Test
  public void ImageReaderSurfaceProducerTrimMemoryCallback() {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();
    FlutterRenderer.ImageReaderSurfaceProducer texture =
        (FlutterRenderer.ImageReaderSurfaceProducer) producer;

    texture.disableFenceForTest();

    // Returns a null image when one hasn't been produced.
    assertNull(texture.acquireLatestImage());

    // Give the texture an initial size.
    texture.setSize(1, 1);

    // Grab the surface so we can render a frame at 1x1 after resizing.
    Surface surface = texture.getSurface();
    assertNotNull(surface);
    Canvas canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Let callbacks run, this will produce a single frame.
    shadowOf(Looper.getMainLooper()).idle();

    assertEquals(1, texture.numImageReaders());
    assertEquals(1, texture.numImages());

    // Invoke the onTrimMemory callback with level 0.
    // This should do nothing.
    texture.onTrimMemory(0);
    shadowOf(Looper.getMainLooper()).idle();

    assertEquals(1, texture.numImageReaders());
    assertEquals(1, texture.numImages());
    assertEquals(0, texture.numTrims());

    // Invoke the onTrimMemory callback with level 40.
    // This should result in a trim.
    texture.onTrimMemory(TRIM_MEMORY_BACKGROUND);
    shadowOf(Looper.getMainLooper()).idle();

    assertEquals(0, texture.numImageReaders());
    assertEquals(0, texture.numImages());
    assertEquals(1, texture.numTrims());

    // Request the surface, this should result in a new image reader.
    surface = texture.getSurface();
    assertEquals(1, texture.numImageReaders());
    assertEquals(0, texture.numImages());
    assertEquals(1, texture.numTrims());

    // Render an image.
    canvas = surface.lockHardwareCanvas();
    canvas.drawARGB(255, 255, 0, 0);
    surface.unlockCanvasAndPost(canvas);

    // Let callbacks run, this will produce a single frame.
    shadowOf(Looper.getMainLooper()).idle();

    assertEquals(1, texture.numImageReaders());
    assertEquals(1, texture.numImages());
    assertEquals(1, texture.numTrims());
  }

  // A 0x0 ImageReader is a runtime error.
  @Test
  public void ImageReaderSurfaceProducerClampsWidthAndHeightTo1() {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();

    // Default values.
    assertEquals(producer.getWidth(), 1);
    assertEquals(producer.getHeight(), 1);

    // Try setting width and height to 0.
    producer.setSize(0, 0);

    // Ensure we can still create/get a surface without an exception being raised.
    assertNotNull(producer.getSurface());

    // Expect clamp to 1.
    assertEquals(producer.getWidth(), 1);
    assertEquals(producer.getHeight(), 1);
  }

  @Test
  public void SurfaceTextureSurfaceProducerCreatesAConnectedTexture() {
    // Force creating a SurfaceTextureSurfaceProducer regardless of Android API version.
    Surface fakeSurface = mock(Surface.class);
    try {
      FlutterRenderer.debugForceSurfaceProducerGlTextures = true;
      FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
      TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();

      flutterRenderer.startRenderingToSurface(fakeSurface, false);

      // Verify behavior under test.
      assertEquals(producer.id(), 0);
      verify(fakeFlutterJNI, times(1)).registerTexture(eq(producer.id()), any());
    } finally {
      FlutterRenderer.debugForceSurfaceProducerGlTextures = false;
    }
  }

  @Test
  public void SurfaceTextureSurfaceProducerDoesNotCropOrRotate() {
    try {
      FlutterRenderer.debugForceSurfaceProducerGlTextures = true;
      FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
      TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();

      assertTrue(producer.handlesCropAndRotation());
    } finally {
      FlutterRenderer.debugForceSurfaceProducerGlTextures = false;
    }
  }

  @Test
  public void ImageReaderSurfaceProducerDoesNotCropOrRotate() {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();

    assertFalse(producer.handlesCropAndRotation());
  }

  @Test
  public void ImageReaderSurfaceProducerIsDestroyedOnTrimMemory() {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();

    // Create and set a mock callback.
    TextureRegistry.SurfaceProducer.Callback callback =
        mock(TextureRegistry.SurfaceProducer.Callback.class);
    producer.setCallback(callback);

    // Trim memory.
    flutterRenderer.onTrimMemory(TRIM_MEMORY_BACKGROUND);

    // Verify.
    verify(callback).onSurfaceDestroyed();
  }

  @Test
  public void ImageReaderSurfaceProducerUnsubscribesWhenReleased() {
    // Regression test for https://github.com/flutter/flutter/issues/156434.
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();

    // Create and set a mock callback.
    TextureRegistry.SurfaceProducer.Callback callback =
        mock(TextureRegistry.SurfaceProducer.Callback.class);
    producer.setCallback(callback);

    // Release the surface.
    producer.release();

    // Call trim memory.
    flutterRenderer.onTrimMemory(TRIM_MEMORY_BACKGROUND);

    // Verify was not called.
    verify(callback, never()).onSurfaceDestroyed();
  }

  @Test
  public void ImageReaderSurfaceProducerIsCreatedOnLifecycleResume() throws Exception {
    FlutterRenderer flutterRenderer = engineRule.getFlutterEngine().getRenderer();
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();

    // Create a callback.
    CountDownLatch latch = new CountDownLatch(1);
    TextureRegistry.SurfaceProducer.Callback callback =
        new TextureRegistry.SurfaceProducer.Callback() {
          @Override
          public void onSurfaceAvailable() {
            latch.countDown();
          }

          @Override
          public void onSurfaceDestroyed() {}
        };
    producer.setCallback(callback);

    // Trim memory.
    flutterRenderer.onTrimMemory(TRIM_MEMORY_BACKGROUND);

    // Trigger a resume.
    ((LifecycleRegistry) ProcessLifecycleOwner.get().getLifecycle())
        .setCurrentState(Lifecycle.State.RESUMED);

    // Verify.
    latch.await();
  }

  @Test
  public void ImageReaderSurfaceProducerSchedulesFrameIfQueueNotEmpty() throws Exception {
    FlutterRenderer flutterRenderer = spy(engineRule.getFlutterEngine().getRenderer());
    TextureRegistry.SurfaceProducer producer = flutterRenderer.createSurfaceProducer();
    FlutterRenderer.ImageReaderSurfaceProducer texture =
        (FlutterRenderer.ImageReaderSurfaceProducer) producer;
    texture.disableFenceForTest();
    texture.setSize(1, 1);

    // Render two frames.
    for (int i = 0; i < 2; i++) {
      Surface surface = texture.getSurface();
      assertNotNull(surface);
      Canvas canvas = surface.lockHardwareCanvas();
      canvas.drawARGB(255, 255, 0, 0);
      surface.unlockCanvasAndPost(canvas);
      shadowOf(Looper.getMainLooper()).idle();
    }

    // Each enqueue of an image should result in a call to scheduleEngineFrame.
    verify(flutterRenderer, times(2)).scheduleEngineFrame();

    // Consume the first image.
    Image image = texture.acquireLatestImage();
    shadowOf(Looper.getMainLooper()).idle();

    // The dequeue should call scheduleEngineFrame because another image
    // remains in the queue.
    verify(flutterRenderer, times(3)).scheduleEngineFrame();

    // Consume the second image.
    image = texture.acquireLatestImage();
    shadowOf(Looper.getMainLooper()).idle();

    // The dequeue should not call scheduleEngineFrame because the queue
    // is now empty.
    verify(flutterRenderer, times(3)).scheduleEngineFrame();
  }
}
