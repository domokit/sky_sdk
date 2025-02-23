package test.io.flutter.embedding.engine;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.anyInt;
import static org.mockito.Mockito.atLeast;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager.NameNotFoundException;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.os.LocaleList;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.FlutterInjector;
import io.flutter.embedding.engine.FlutterEngine;
import io.flutter.embedding.engine.FlutterEngine.EngineLifecycleListener;
import io.flutter.embedding.engine.FlutterEngineGroup;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.loader.FlutterLoader;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.PluginRegistry;
import io.flutter.plugin.platform.PlatformViewsController;
import io.flutter.plugins.GeneratedPluginRegistrant;
import java.util.List;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;
import org.robolectric.Robolectric;
import org.robolectric.android.controller.ActivityController;
import org.robolectric.annotation.Config;
import org.robolectric.shadows.ShadowLog;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
public class FlutterEngineTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();
  private final Context mockContext = mock(Context.class);
  @Mock FlutterJNI flutterJNI;
  boolean jniAttached;

  @Before
  public void setUp() {
    MockitoAnnotations.openMocks(this);

    Resources mockResources = mock(Resources.class);
    Configuration mockConfiguration = mock(Configuration.class);
    doReturn(mockResources).when(mockContext).getResources();
    doReturn(mockConfiguration).when(mockResources).getConfiguration();
    doReturn(LocaleList.getEmptyLocaleList()).when(mockConfiguration).getLocales();

    jniAttached = false;
    when(flutterJNI.isAttached()).thenAnswer(invocation -> jniAttached);
    doAnswer(
            new Answer() {
              @Override
              public Object answer(InvocationOnMock invocation) throws Throwable {
                jniAttached = true;
                return null;
              }
            })
        .when(flutterJNI)
        .attachToNative();
    GeneratedPluginRegistrant.clearRegisteredEngines();
    FlutterEngine.resetNextEngineId();
  }

  @After
  public void tearDown() {
    GeneratedPluginRegistrant.clearRegisteredEngines();
    // Make sure to not forget to remove the mock exception in the generated plugin registration
    // mock, or everything subsequent will break.
    GeneratedPluginRegistrant.pluginRegistrationException = null;
  }

  @Test
  public void itAutomaticallyRegistersPluginsByDefault() {
    assertTrue(GeneratedPluginRegistrant.getRegisteredEngines().isEmpty());
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    when(mockFlutterLoader.automaticallyRegisterPlugins()).thenReturn(true);
    FlutterEngine flutterEngine = new FlutterEngine(ctx, mockFlutterLoader, flutterJNI);

    List<FlutterEngine> registeredEngines = GeneratedPluginRegistrant.getRegisteredEngines();
    assertEquals(1, registeredEngines.size());
    assertEquals(flutterEngine, registeredEngines.get(0));
  }

  @Test
  public void itUpdatesDisplayMetricsOnConstructionWithActivityContext() {
    // Needs an activity. ApplicationContext won't work for this.
    ActivityController<Activity> activityController = Robolectric.buildActivity(Activity.class);
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    when(mockFlutterJNI.isAttached()).thenReturn(true);

    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterEngine flutterEngine =
        new FlutterEngine(activityController.get(), mockFlutterLoader, mockFlutterJNI);

    verify(mockFlutterJNI, times(1))
        .updateDisplayMetrics(eq(0), any(Float.class), any(Float.class), any(Float.class));
  }

  @Test
  public void itSendLocalesOnEngineInit() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    when(mockFlutterJNI.isAttached()).thenReturn(true);

    assertTrue(GeneratedPluginRegistrant.getRegisteredEngines().isEmpty());
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    when(mockFlutterLoader.automaticallyRegisterPlugins()).thenReturn(true);
    FlutterEngine flutterEngine = new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJNI);

    verify(mockFlutterJNI, times(1))
        .dispatchPlatformMessage(eq("flutter/localization"), any(), anyInt(), anyInt());
  }

  @Test
  public void itCanBeRetrievedByHandle() {
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    when(mockFlutterJNI.isAttached()).thenReturn(true);
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    when(mockFlutterLoader.automaticallyRegisterPlugins()).thenReturn(true);
    FlutterEngine flutterEngine1 = new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJNI);
    FlutterEngine flutterEngine2 = new FlutterEngine(ctx, mockFlutterLoader, mockFlutterJNI);
    assertEquals(flutterEngine1, FlutterEngine.engineForId(1));
    assertEquals(flutterEngine2, FlutterEngine.engineForId(2));
    flutterEngine1.destroy();
    assertEquals(null, FlutterEngine.engineForId(1));
    assertEquals(flutterEngine2, FlutterEngine.engineForId(2));
    flutterEngine2.destroy();
    assertEquals(null, FlutterEngine.engineForId(2));
  }

  // Helps show the root cause of MissingPluginException type errors like
  // https://github.com/flutter/flutter/issues/78625.
  @Test
  public void itCatchesAndDisplaysRegistrationExceptions() {
    assertTrue(GeneratedPluginRegistrant.getRegisteredEngines().isEmpty());
    GeneratedPluginRegistrant.pluginRegistrationException =
        new RuntimeException("I'm a bug in the plugin");
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    when(mockFlutterLoader.automaticallyRegisterPlugins()).thenReturn(true);
    FlutterEngine flutterEngine = new FlutterEngine(ctx, mockFlutterLoader, flutterJNI);

    List<FlutterEngine> registeredEngines = GeneratedPluginRegistrant.getRegisteredEngines();
    // When it crashes, it doesn't end up registering anything.
    assertEquals(0, registeredEngines.size());

    // Check the logs actually says registration failed, so a subsequent MissingPluginException
    // isn't mysterious.
    assertTrue(
        ShadowLog.getLogsForTag("GeneratedPluginsRegister")
            .get(0)
            .msg
            .contains("Tried to automatically register plugins"));
    assertEquals(
        GeneratedPluginRegistrant.pluginRegistrationException,
        ShadowLog.getLogsForTag("GeneratedPluginsRegister").get(1).throwable.getCause());

    GeneratedPluginRegistrant.pluginRegistrationException = null;
  }

  @Test
  public void itDoesNotAutomaticallyRegistersPluginsWhenFlutterLoaderDisablesIt() {
    assertTrue(GeneratedPluginRegistrant.getRegisteredEngines().isEmpty());
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    when(mockFlutterLoader.automaticallyRegisterPlugins()).thenReturn(false);
    new FlutterEngine(ctx, mockFlutterLoader, flutterJNI);

    List<FlutterEngine> registeredEngines = GeneratedPluginRegistrant.getRegisteredEngines();
    assertTrue(registeredEngines.isEmpty());
  }

  @Test
  public void itDoesNotAutomaticallyRegistersPluginsWhenFlutterEngineDisablesIt() {
    assertTrue(GeneratedPluginRegistrant.getRegisteredEngines().isEmpty());
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    when(mockFlutterLoader.automaticallyRegisterPlugins()).thenReturn(true);
    new FlutterEngine(
        ctx,
        mockFlutterLoader,
        flutterJNI,
        /*dartVmArgs=*/ new String[] {},
        /*automaticallyRegisterPlugins=*/ false);

    List<FlutterEngine> registeredEngines = GeneratedPluginRegistrant.getRegisteredEngines();
    assertTrue(registeredEngines.isEmpty());
  }

  @Test
  public void itNotifiesPlatformViewsControllerWhenDevHotRestart() {
    // Setup test.
    FlutterJNI mockFlutterJNI = mock(FlutterJNI.class);
    when(mockFlutterJNI.isAttached()).thenReturn(true);

    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);

    ArgumentCaptor<FlutterEngine.EngineLifecycleListener> engineLifecycleListenerArgumentCaptor =
        ArgumentCaptor.forClass(FlutterEngine.EngineLifecycleListener.class);

    // Execute behavior under test.
    new FlutterEngine(
        ctx,
        mock(FlutterLoader.class),
        mockFlutterJNI,
        platformViewsController,
        /*dartVmArgs=*/ new String[] {},
        /*automaticallyRegisterPlugins=*/ false);

    // Obtain the EngineLifecycleListener within FlutterEngine that was given to FlutterJNI.
    verify(mockFlutterJNI)
        .addEngineLifecycleListener(engineLifecycleListenerArgumentCaptor.capture());
    FlutterEngine.EngineLifecycleListener engineLifecycleListener =
        engineLifecycleListenerArgumentCaptor.getValue();
    assertNotNull(engineLifecycleListener);

    // Simulate a pre-engine restart, AKA hot restart.
    engineLifecycleListener.onPreEngineRestart();

    // Verify that FlutterEngine notified PlatformViewsController of the pre-engine restart,
    // AKA hot restart.
    verify(platformViewsController, times(1)).onPreEngineRestart();
  }

  @Test
  public void itNotifiesPlatformViewsControllerAboutJNILifecycle() {
    PlatformViewsController platformViewsController = mock(PlatformViewsController.class);

    // Execute behavior under test.
    FlutterEngine engine =
        new FlutterEngine(
            ctx,
            mock(FlutterLoader.class),
            flutterJNI,
            platformViewsController,
            /*dartVmArgs=*/ new String[] {},
            /*automaticallyRegisterPlugins=*/ false);

    engine.destroy();
    verify(platformViewsController, times(1)).onDetachedFromJNI();
  }

  @Test
  public void itUsesApplicationContext() throws NameNotFoundException {
    Context packageContext = mock(Context.class);

    when(mockContext.createPackageContext(any(), anyInt())).thenReturn(packageContext);

    new FlutterEngine(
        mockContext,
        mock(FlutterLoader.class),
        flutterJNI,
        /*dartVmArgs=*/ new String[] {},
        /*automaticallyRegisterPlugins=*/ false);

    verify(mockContext, atLeast(1)).getApplicationContext();
  }

  @Test
  public void itUsesPackageContextForAssetManager() throws NameNotFoundException {
    Context packageContext = mock(Context.class);
    when(mockContext.createPackageContext(any(), anyInt())).thenReturn(packageContext);

    new FlutterEngine(
        mockContext,
        mock(FlutterLoader.class),
        flutterJNI,
        /*dartVmArgs=*/ new String[] {},
        /*automaticallyRegisterPlugins=*/ false);

    verify(packageContext, atLeast(1)).getAssets();
    verify(mockContext, times(0)).getAssets();
  }

  @Test
  public void itCanUseFlutterLoaderInjectionViaFlutterInjector() throws NameNotFoundException {
    FlutterInjector.reset();
    FlutterLoader mockFlutterLoader = mock(FlutterLoader.class);
    FlutterInjector.setInstance(
        new FlutterInjector.Builder().setFlutterLoader(mockFlutterLoader).build());
    Context packageContext = mock(Context.class);

    when(mockContext.createPackageContext(any(), anyInt())).thenReturn(packageContext);

    new FlutterEngine(mockContext, null, flutterJNI);

    verify(mockFlutterLoader, times(1)).startInitialization(any());
    verify(mockFlutterLoader, times(1)).ensureInitializationComplete(any(), any());
    FlutterInjector.reset();
  }

  @Test
  public void itNotifiesListenersForDestruction() throws NameNotFoundException {
    Context packageContext = mock(Context.class);

    when(mockContext.createPackageContext(any(), anyInt())).thenReturn(packageContext);

    FlutterEngine engineUnderTest =
        new FlutterEngine(
            mockContext,
            mock(FlutterLoader.class),
            flutterJNI,
            /*dartVmArgs=*/ new String[] {},
            /*automaticallyRegisterPlugins=*/ false);

    EngineLifecycleListener listener = mock(EngineLifecycleListener.class);
    engineUnderTest.addEngineLifecycleListener(listener);
    engineUnderTest.destroy();
    verify(listener, times(1)).onEngineWillDestroy();
  }

  @Test
  public void itDoesNotAttachAgainWhenBuiltWithAnAttachedJNI() throws NameNotFoundException {
    Context packageContext = mock(Context.class);

    when(mockContext.createPackageContext(any(), anyInt())).thenReturn(packageContext);
    when(flutterJNI.isAttached()).thenReturn(true);

    FlutterEngine engineUnderTest =
        new FlutterEngine(
            mockContext,
            mock(FlutterLoader.class),
            flutterJNI,
            /*dartVmArgs=*/ new String[] {},
            /*automaticallyRegisterPlugins=*/ false);

    verify(flutterJNI, never()).attachToNative();
  }

  @Test
  public void itComesWithARunningDartExecutorIfJNIIsAlreadyAttached() throws NameNotFoundException {
    Context packageContext = mock(Context.class);

    when(mockContext.createPackageContext(any(), anyInt())).thenReturn(packageContext);
    when(flutterJNI.isAttached()).thenReturn(true);

    FlutterEngine engineUnderTest =
        new FlutterEngine(
            mockContext,
            mock(FlutterLoader.class),
            flutterJNI,
            /*dartVmArgs=*/ new String[] {},
            /*automaticallyRegisterPlugins=*/ false);

    assertTrue(engineUnderTest.getDartExecutor().isExecutingDart());
  }

  @Test
  public void passesEngineGroupToPlugins() throws NameNotFoundException {
    Context packageContext = mock(Context.class);

    when(mockContext.createPackageContext(any(), anyInt())).thenReturn(packageContext);
    when(flutterJNI.isAttached()).thenReturn(true);

    FlutterEngineGroup mockGroup = mock(FlutterEngineGroup.class);

    FlutterEngine engineUnderTest =
        new FlutterEngine(
            mockContext,
            mock(FlutterLoader.class),
            flutterJNI,
            new PlatformViewsController(),
            /*dartVmArgs=*/ new String[] {},
            /*automaticallyRegisterPlugins=*/ false,
            /*waitForRestorationData=*/ false,
            mockGroup);

    PluginRegistry registry = engineUnderTest.getPlugins();
    FlutterPlugin mockPlugin = mock(FlutterPlugin.class);
    ArgumentCaptor<FlutterPlugin.FlutterPluginBinding> pluginBindingCaptor =
        ArgumentCaptor.forClass(FlutterPlugin.FlutterPluginBinding.class);
    registry.add(mockPlugin);
    verify(mockPlugin).onAttachedToEngine(pluginBindingCaptor.capture());
    assertNotNull(pluginBindingCaptor.getValue());
    assertEquals(mockGroup, pluginBindingCaptor.getValue().getEngineGroup());
  }
}
