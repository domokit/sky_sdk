// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.android;

import static io.flutter.Build.API_LEVELS;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.TargetApi;
import android.view.Surface;
import android.view.SurfaceHolder;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.renderer.FlutterRenderer;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_30)
public class FlutterSurfaceViewTest {
  @Test
  public void itShouldCreateANewSurfaceWhenReattachedAfterDetachingFromRenderer() {
    // Consider this scenario: In an add-to-app context, where multiple Flutter activities share the
    // same engine, a situation occurs. When navigating from FlutterActivity1 to FlutterActivity2,
    // the Flutter view associated with FlutterActivity1 is detached from the engine. Then, the
    // Flutter view of FlutterActivity2 is attached. Upon navigating back to FlutterActivity1, its
    // Flutter view is re-attached to the shared engine.
    //
    // The expected behavior is: When a Flutter view detaches from the shared engine, the associated
    // surface should be released. When the Flutter view re-attaches, a new surface should be
    // created.

    // Setup the test.
    final FlutterSurfaceView surfaceView =
        spy(new FlutterSurfaceView(ApplicationProvider.getApplicationContext()));

    FlutterJNI fakeFlutterJNI = mock(FlutterJNI.class);
    FlutterRenderer flutterRenderer = new FlutterRenderer(fakeFlutterJNI);

    SurfaceHolder fakeSurfaceHolder = mock(SurfaceHolder.class);
    Surface fakeSurface = mock(Surface.class);
    when(surfaceView.getHolder()).thenReturn(fakeSurfaceHolder);
    when(fakeSurfaceHolder.getSurface()).thenReturn(fakeSurface);
    when(surfaceView.isSurfaceAvailableForRendering()).thenReturn(true);
    when(surfaceView.getWindowToken()).thenReturn(mock(android.os.IBinder.class));

    // Execute the behavior under test.
    surfaceView.attachToRenderer(flutterRenderer);

    // Verify the behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceCreated(any(Surface.class));

    // Execute the behavior under test.
    surfaceView.detachFromRenderer();

    // Verify the behavior under test.
    verify(fakeFlutterJNI, times(1)).onSurfaceDestroyed();

    // Execute the behavior under test.
    surfaceView.attachToRenderer(flutterRenderer);

    // Verify the behavior under test.
    verify(fakeFlutterJNI, never()).onSurfaceWindowChanged(any(Surface.class));
    verify(fakeFlutterJNI, times(2)).onSurfaceCreated(any(Surface.class));
  }
}
