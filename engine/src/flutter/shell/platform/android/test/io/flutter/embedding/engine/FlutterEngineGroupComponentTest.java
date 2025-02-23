// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package io.flutter.embedding.engine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.isNull;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.nullable;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.content.Context;
import android.content.res.AssetManager;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.FlutterInjector;
import io.flutter.embedding.engine.dart.DartExecutor.DartEntrypoint;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.systemchannels.NavigationChannel;
import io.flutter.plugin.platform.PlatformViewsController;
import io.flutter.plugins.GeneratedPluginRegistrant;
import java.util.ArrayList;
import java.util.List;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.robolectric.annotation.Config;

// It's a component test because it tests the FlutterEngineGroup its components such as the
// FlutterEngine and the DartExecutor.
@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterEngineGroupComponentTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  @Mock FlutterJNI mockFlutterJNI;
  @Mock FlutterLoader mockFlutterLoader;
  FlutterEngineGroup engineGroupUnderTest;
  FlutterEngine firstEngineUnderTest;
  boolean jniAttached;

  @Before
  public void setUp() {
    FlutterInjector.reset();

    MockitoAnnotations.openMocks(this);
    jniAttached = false;
    when(mockFlutterJNI.isAttached()).thenAnswer(invocation -> jniAttached);
    doAnswer(invocation -> jniAttached = true).when(mockFlutterJNI).attachToNative();
    GeneratedPluginRegistrant.clearRegisteredEngines();
    FlutterEngine.resetNextEngineId();

    when(mockFlutterLoader.findAppBundlePath()).thenReturn("some/path/to/flutter_assets");

    FlutterJNI.Factory jniFactory =
        new FlutterJNI.Factory() {
          @Override
          public FlutterJNI provideFlutterJNI() {
            // The default implementation is that `new FlutterJNI()` will report errors when
            // creating the engine later,
            // Change mockFlutterJNI to the default so that we can create the engine directly in
            // later tests
            return mockFlutterJNI;
          }
        };

    FlutterInjector.setInstance(
        new FlutterInjector.Builder()
            .setFlutterLoader(mockFlutterLoader)
            .setFlutterJNIFactory(jniFactory)
            .build());

    firstEngineUnderTest =
        spy(
            new FlutterEngine(
                ctx,
                mock(FlutterLoader.class),
                mockFlutterJNI,
                /*dartVmArgs=*/ new String[] {},
                /*automaticallyRegisterPlugins=*/ false));
    engineGroupUnderTest =
        new FlutterEngineGroup(ctx) {
          @Override
          FlutterEngine createEngine(
              Context context,
              PlatformViewsController platformViewsController,
              boolean automaticallyRegisterPlugins,
              boolean waitForRestorationData) {
            return firstEngineUnderTest;
          }
        };
  }

  @After
  public void tearDown() {
    GeneratedPluginRegistrant.clearRegisteredEngines();
    engineGroupUnderTest = null;
    firstEngineUnderTest = null;
  }

  @Test
  public void listensToEngineDestruction() {
    FlutterEngine firstEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class));
    assertEquals(1, engineGroupUnderTest.activeEngines.size());

    firstEngine.destroy();
    assertEquals(0, engineGroupUnderTest.activeEngines.size());
  }

  @Test
  public void canRecreateEngines() {
    FlutterEngine firstEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class));
    assertEquals(1, engineGroupUnderTest.activeEngines.size());

    firstEngine.destroy();
    assertEquals(0, engineGroupUnderTest.activeEngines.size());

    FlutterEngine secondEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class));
    assertEquals(1, engineGroupUnderTest.activeEngines.size());
    // They happen to be equal in our test since we mocked it to be so.
    assertEquals(firstEngine, secondEngine);
  }

  @Test
  public void canSpawnMoreEngines() {
    FlutterEngine firstEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class));
    assertEquals(1, engineGroupUnderTest.activeEngines.size());

    doReturn(mock(FlutterEngine.class))
        .when(firstEngine)
        .spawn(
            any(Context.class),
            any(DartEntrypoint.class),
            nullable(String.class),
            nullable(List.class),
            any(PlatformViewsController.class),
            any(Boolean.class),
            any(Boolean.class));

    FlutterEngine secondEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class));
    assertEquals(2, engineGroupUnderTest.activeEngines.size());

    firstEngine.destroy();
    assertEquals(1, engineGroupUnderTest.activeEngines.size());

    // Now the second spawned engine is the only one left and it will be called to spawn the next
    // engine in the chain.
    when(secondEngine.spawn(
            any(Context.class),
            any(DartEntrypoint.class),
            nullable(String.class),
            nullable(List.class),
            any(PlatformViewsController.class),
            any(Boolean.class),
            any(Boolean.class)))
        .thenReturn(mock(FlutterEngine.class));

    FlutterEngine thirdEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class));
    assertEquals(2, engineGroupUnderTest.activeEngines.size());
  }

  @Test
  public void canCreateAndRunCustomEntrypoints() {
    FlutterEngine firstEngine =
        engineGroupUnderTest.createAndRunEngine(
            ctx,
            new DartEntrypoint(
                FlutterInjector.instance().flutterLoader().findAppBundlePath(),
                "other entrypoint"));
    assertEquals(1, engineGroupUnderTest.activeEngines.size());
    verify(mockFlutterJNI, times(1))
        .runBundleAndSnapshotFromLibrary(
            eq("some/path/to/flutter_assets"),
            eq("other entrypoint"),
            isNull(),
            any(AssetManager.class),
            nullable(List.class),
            eq(1l));
  }

  @Test
  public void canCreateAndRunWithCustomInitialRoute() {
    when(firstEngineUnderTest.getNavigationChannel()).thenReturn(mock(NavigationChannel.class));

    FlutterEngine firstEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class), "/foo");
    assertEquals(1, engineGroupUnderTest.activeEngines.size());
    verify(firstEngine.getNavigationChannel(), times(1)).setInitialRoute("/foo");

    when(mockFlutterJNI.isAttached()).thenReturn(true);
    jniAttached = false;
    FlutterJNI secondMockFlutterJNI = mock(FlutterJNI.class);
    when(secondMockFlutterJNI.isAttached()).thenAnswer(invocation -> jniAttached);
    doAnswer(invocation -> jniAttached = true).when(secondMockFlutterJNI).attachToNative();
    doReturn(secondMockFlutterJNI)
        .when(mockFlutterJNI)
        .spawn(
            nullable(String.class),
            nullable(String.class),
            nullable(String.class),
            nullable(List.class),
            eq(2l));

    FlutterEngine secondEngine =
        engineGroupUnderTest.createAndRunEngine(ctx, mock(DartEntrypoint.class), "/bar");

    assertEquals(2, engineGroupUnderTest.activeEngines.size());
    verify(mockFlutterJNI, times(1))
        .spawn(
            nullable(String.class),
            nullable(String.class),
            eq("/bar"),
            nullable(List.class),
            eq(2l));
  }

  @Test
  public void canCreateAndRunWithCustomEntrypointArgs() {
    List<String> firstDartEntrypointArgs = new ArrayList<String>();
    FlutterEngine firstEngine =
        engineGroupUnderTest.createAndRunEngine(
            new FlutterEngineGroup.Options(ctx)
                .setDartEntrypoint(mock(DartEntrypoint.class))
                .setDartEntrypointArgs(firstDartEntrypointArgs));
    assertEquals(1, engineGroupUnderTest.activeEngines.size());
    verify(mockFlutterJNI, times(1))
        .runBundleAndSnapshotFromLibrary(
            nullable(String.class),
            nullable(String.class),
            isNull(),
            any(AssetManager.class),
            eq(firstDartEntrypointArgs),
            eq(1l));

    when(mockFlutterJNI.isAttached()).thenReturn(true);
    jniAttached = false;
    FlutterJNI secondMockFlutterJNI = mock(FlutterJNI.class);
    when(secondMockFlutterJNI.isAttached()).thenAnswer(invocation -> jniAttached);
    doAnswer(invocation -> jniAttached = true).when(secondMockFlutterJNI).attachToNative();
    doReturn(secondMockFlutterJNI)
        .when(mockFlutterJNI)
        .spawn(
            nullable(String.class),
            nullable(String.class),
            nullable(String.class),
            nullable(List.class),
            eq(2l));
    List<String> secondDartEntrypointArgs = new ArrayList<String>();
    FlutterEngine secondEngine =
        engineGroupUnderTest.createAndRunEngine(
            new FlutterEngineGroup.Options(ctx)
                .setDartEntrypoint(mock(DartEntrypoint.class))
                .setDartEntrypointArgs(secondDartEntrypointArgs));

    assertEquals(2, engineGroupUnderTest.activeEngines.size());
    verify(mockFlutterJNI, times(1))
        .spawn(
            nullable(String.class),
            nullable(String.class),
            nullable(String.class),
            eq(secondDartEntrypointArgs),
            eq(2l));
  }

  @Test
  public void createEngineSupportMoreParams() {
    // Create a new FlutterEngineGroup because the first engine created in engineGroupUnderTest was
    // changed to firstEngineUnderTest in `setUp()`, so can't use it to validate params.
    FlutterEngineGroup engineGroup = new FlutterEngineGroup(ctx);

    PlatformViewsController controller = new PlatformViewsController();
    boolean waitForRestorationData = true;
    boolean automaticallyRegisterPlugins = true;

    when(FlutterInjector.instance().flutterLoader().automaticallyRegisterPlugins())
        .thenReturn(true);
    assertTrue(FlutterInjector.instance().flutterLoader().automaticallyRegisterPlugins());
    assertEquals(0, GeneratedPluginRegistrant.getRegisteredEngines().size());

    FlutterEngine firstEngine =
        engineGroup.createAndRunEngine(
            new FlutterEngineGroup.Options(ctx)
                .setDartEntrypoint(mock(DartEntrypoint.class))
                .setPlatformViewsController(controller)
                .setWaitForRestorationData(waitForRestorationData)
                .setAutomaticallyRegisterPlugins(automaticallyRegisterPlugins));

    assertEquals(1, GeneratedPluginRegistrant.getRegisteredEngines().size());
    assertEquals(controller, firstEngine.getPlatformViewsController());
    assertEquals(
        waitForRestorationData, firstEngine.getRestorationChannel().waitForRestorationData);
  }

  @Test
  public void spawnEngineSupportMoreParams() {
    FlutterEngine firstEngine =
        engineGroupUnderTest.createAndRunEngine(
            new FlutterEngineGroup.Options(ctx).setDartEntrypoint(mock(DartEntrypoint.class)));
    assertEquals(1, engineGroupUnderTest.activeEngines.size());
    verify(mockFlutterJNI, times(1))
        .runBundleAndSnapshotFromLibrary(
            nullable(String.class),
            nullable(String.class),
            isNull(),
            any(AssetManager.class),
            nullable(List.class),
            eq(1l));

    when(mockFlutterJNI.isAttached()).thenReturn(true);
    jniAttached = false;
    FlutterJNI secondMockFlutterJNI = mock(FlutterJNI.class);
    when(secondMockFlutterJNI.isAttached()).thenAnswer(invocation -> jniAttached);
    doAnswer(invocation -> jniAttached = true).when(secondMockFlutterJNI).attachToNative();
    doReturn(secondMockFlutterJNI)
        .when(mockFlutterJNI)
        .spawn(
            nullable(String.class),
            nullable(String.class),
            nullable(String.class),
            nullable(List.class),
            eq(2l));

    PlatformViewsController controller = new PlatformViewsController();
    boolean waitForRestorationData = false;
    boolean automaticallyRegisterPlugins = false;

    when(FlutterInjector.instance().flutterLoader().automaticallyRegisterPlugins())
        .thenReturn(true);
    assertTrue(FlutterInjector.instance().flutterLoader().automaticallyRegisterPlugins());
    assertEquals(0, GeneratedPluginRegistrant.getRegisteredEngines().size());

    FlutterEngine secondEngine =
        engineGroupUnderTest.createAndRunEngine(
            new FlutterEngineGroup.Options(ctx)
                .setDartEntrypoint(mock(DartEntrypoint.class))
                .setWaitForRestorationData(waitForRestorationData)
                .setPlatformViewsController(controller)
                .setAutomaticallyRegisterPlugins(automaticallyRegisterPlugins));

    assertEquals(
        waitForRestorationData, secondEngine.getRestorationChannel().waitForRestorationData);
    assertEquals(controller, secondEngine.getPlatformViewsController());
    assertEquals(0, GeneratedPluginRegistrant.getRegisteredEngines().size());
  }
}
