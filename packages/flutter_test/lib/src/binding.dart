// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:io';
///
/// @docImport 'controller.dart';
/// @docImport 'test_pointer.dart';
/// @docImport 'widget_tester.dart';
library;

import 'dart:async';
import 'dart:ui' as ui;

import 'package:clock/clock.dart';
import 'package:fake_async/fake_async.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:matcher/expect.dart' show fail;
import 'package:stack_trace/stack_trace.dart' as stack_trace;
import 'package:test_api/scaffolding.dart' as test_package show Timeout;
import 'package:vector_math/vector_math_64.dart';

import '_binding_io.dart' if (dart.library.js_interop) '_binding_web.dart' as binding;
import 'goldens.dart';
import 'platform.dart';
import 'restoration.dart';
import 'stack_manipulation.dart';
import 'test_async_utils.dart';
import 'test_default_binary_messenger.dart';
import 'test_exception_reporter.dart';
import 'test_text_input.dart';
import 'window.dart';

/// Phases that can be reached by [WidgetTester.pumpWidget] and
/// [TestWidgetsFlutterBinding.pump].
///
/// See [WidgetsBinding.drawFrame] for a more detailed description of some of
/// these phases.
enum EnginePhase {
  /// The build phase in the widgets library. See [BuildOwner.buildScope].
  build,

  /// The layout phase in the rendering library. See [PipelineOwner.flushLayout].
  layout,

  /// The compositing bits update phase in the rendering library. See
  /// [PipelineOwner.flushCompositingBits].
  compositingBits,

  /// The paint phase in the rendering library. See [PipelineOwner.flushPaint].
  paint,

  /// The compositing phase in the rendering library. See
  /// [RenderView.compositeFrame]. This is the phase in which data is sent to
  /// the GPU. If semantics are not enabled, then this is the last phase.
  composite,

  /// The semantics building phase in the rendering library. See
  /// [PipelineOwner.flushSemantics].
  flushSemantics,

  /// The final phase in the rendering library, wherein semantics information is
  /// sent to the embedder. See [SemanticsOwner.sendSemanticsUpdate].
  sendSemanticsUpdate,
}

/// Signature of callbacks used to intercept messages on a given channel.
///
/// See [TestDefaultBinaryMessenger.setMockDecodedMessageHandler] for more details.
typedef _MockMessageHandler = Future<void> Function(Object?);

/// Parts of the system that can generate pointer events that reach the test
/// binding.
///
/// This is used to identify how to handle events in the
/// [LiveTestWidgetsFlutterBinding]. See
/// [TestWidgetsFlutterBinding.dispatchEvent].
enum TestBindingEventSource {
  /// The pointer event came from the test framework itself, e.g. from a
  /// [TestGesture] created by [WidgetTester.startGesture].
  test,

  /// The pointer event came from the system, presumably as a result of the user
  /// interactive directly with the device while the test was running.
  device,
}

const Size _kDefaultTestViewportSize = Size(800.0, 600.0);

/// Overrides the [ServicesBinding]'s binary messenger logic to use
/// [TestDefaultBinaryMessenger].
///
/// Test bindings that are used by tests that mock message handlers for plugins
/// should mix in this binding to enable the use of the
/// [TestDefaultBinaryMessenger] APIs.
mixin TestDefaultBinaryMessengerBinding on BindingBase, ServicesBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
  }

  /// The current [TestDefaultBinaryMessengerBinding], if one has been created.
  static TestDefaultBinaryMessengerBinding get instance => BindingBase.checkInstance(_instance);
  static TestDefaultBinaryMessengerBinding? _instance;

  @override
  TestDefaultBinaryMessenger get defaultBinaryMessenger =>
      super.defaultBinaryMessenger as TestDefaultBinaryMessenger;

  @override
  TestDefaultBinaryMessenger createBinaryMessenger() {
    Future<ByteData?> keyboardHandler(ByteData? message) async {
      return const StandardMethodCodec().encodeSuccessEnvelope(<int, int>{});
    }

    return TestDefaultBinaryMessenger(
      super.createBinaryMessenger(),
      outboundHandlers: <String, MessageHandler>{'flutter/keyboard': keyboardHandler},
    );
  }
}

/// Accessibility announcement data passed to [SemanticsService.announce] captured in a test.
///
/// This class is intended to be used by the testing API to store the announcements
/// in a structured form so that tests can verify announcement details. The fields
/// of this class correspond to parameters of the [SemanticsService.announce] method.
///
/// See also:
///
///  * [WidgetTester.takeAnnouncements], which is the test API that uses this class.
class CapturedAccessibilityAnnouncement {
  const CapturedAccessibilityAnnouncement._(this.message, this.textDirection, this.assertiveness);

  /// The accessibility message announced by the framework.
  final String message;

  /// The direction in which the text of the [message] flows.
  final TextDirection textDirection;

  /// Determines the assertiveness level of the accessibility announcement.
  final Assertiveness assertiveness;
}

// Examples can assume:
// late TestWidgetsFlutterBinding binding;
// late Size someSize;

/// Base class for bindings used by widgets library tests.
///
/// The [ensureInitialized] method creates (if necessary) and returns an
/// instance of the appropriate subclass. (If one is already created, it returns
/// that one, even if it's not the one that it would normally create. This
/// allows tests to force the use of [LiveTestWidgetsFlutterBinding] even in a
/// normal unit test environment, e.g. to test that binding.)
///
/// When using these bindings, certain features are disabled. For
/// example, [timeDilation] is reset to 1.0 on initialization.
///
/// In non-browser tests, the binding overrides `HttpClient` creation with a
/// fake client that always returns a status code of 400. This is to prevent
/// tests from making network calls, which could introduce flakiness. A test
/// that actually needs to make a network call should provide its own
/// `HttpClient` to the code making the call, so that it can appropriately mock
/// or fake responses.
///
/// ### Coordinate spaces
///
/// [TestWidgetsFlutterBinding] might be run on devices of different screen
/// sizes, while the testing widget is still told the same size to ensure
/// consistent results. Consequently, code that deals with positions (such as
/// pointer events or painting) must distinguish between two coordinate spaces:
///
///  * The _local coordinate space_ is the one used by the testing widget
///    (typically an 800 by 600 window, but can be altered by [setSurfaceSize]).
///  * The _global coordinate space_ is the one used by the device.
///
/// Positions can be transformed between coordinate spaces with [localToGlobal]
/// and [globalToLocal].
abstract class TestWidgetsFlutterBinding extends BindingBase
    with
        SchedulerBinding,
        ServicesBinding,
        GestureBinding,
        SemanticsBinding,
        RendererBinding,
        PaintingBinding,
        WidgetsBinding,
        TestDefaultBinaryMessengerBinding {
  /// Constructor for [TestWidgetsFlutterBinding].
  ///
  /// This constructor overrides the [debugPrint] global hook to point to
  /// [debugPrintOverride], which can be overridden by subclasses.
  TestWidgetsFlutterBinding()
    : platformDispatcher = TestPlatformDispatcher(platformDispatcher: PlatformDispatcher.instance) {
    platformDispatcher.defaultRouteNameTestValue = '/';
    debugPrint = debugPrintOverride;
    debugDisableShadows = disableShadows;
  }

  /// Deprecated. Will be removed in a future version of Flutter.
  ///
  /// This property has been deprecated to prepare for Flutter's upcoming
  /// support for multiple views and multiple windows.
  ///
  /// This represents a combination of a [TestPlatformDispatcher] and a singular
  /// [TestFlutterView]. Platform-specific test values can be set through
  /// [WidgetTester.platformDispatcher] instead. When testing individual widgets
  /// or applications using [WidgetTester.pumpWidget], view-specific test values
  /// can be set through [WidgetTester.view]. If multiple views are defined, the
  /// appropriate view can be found using [WidgetTester.viewOf] if a sub-view
  /// is needed.
  ///
  /// See also:
  ///
  /// * [WidgetTester.platformDispatcher] for changing platform-specific values
  ///   for testing.
  /// * [WidgetTester.view] and [WidgetTester.viewOf] for changing view-specific
  ///   values for testing.
  /// * [BindingBase.window] for guidance dealing with this property outside of
  ///   a testing context.
  @Deprecated(
    'Use WidgetTester.platformDispatcher or WidgetTester.view instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.9.0-0.1.pre.',
  )
  @override
  late final TestWindow window;

  @override
  final TestPlatformDispatcher platformDispatcher;

  @override
  TestRestorationManager get restorationManager {
    _restorationManager ??= createRestorationManager();
    return _restorationManager!;
  }

  TestRestorationManager? _restorationManager;

  /// Called by the test framework at the beginning of a widget test to
  /// prepare the binding for the next test.
  ///
  /// If [registerTestTextInput] returns true when this method is called,
  /// the [testTextInput] is configured to simulate the keyboard.
  void reset() {
    _restorationManager?.dispose();
    _restorationManager = null;
    platformDispatcher.defaultRouteNameTestValue = '/';
    resetGestureBinding();
    testTextInput.reset();
    if (registerTestTextInput) {
      _testTextInput.register();
    }
    CustomSemanticsAction.resetForTests(); // ignore: invalid_use_of_visible_for_testing_member
    _enableFocusManagerLifecycleAwarenessIfSupported();
  }

  void _enableFocusManagerLifecycleAwarenessIfSupported() {
    if (buildOwner == null) {
      return;
    }
    buildOwner!.focusManager
        .listenToApplicationLifecycleChangesIfSupported(); // ignore: invalid_use_of_visible_for_testing_member
  }

  @override
  TestRestorationManager createRestorationManager() {
    return TestRestorationManager();
  }

  /// The value to set [debugPrint] to while tests are running.
  ///
  /// This can be used to redirect console output from the framework, or to
  /// change the behavior of [debugPrint]. For example,
  /// [AutomatedTestWidgetsFlutterBinding] uses it to make [debugPrint]
  /// synchronous, disabling its normal throttling behavior.
  ///
  /// It is also used by some other parts of the test framework (e.g.
  /// [WidgetTester.printToConsole]) to ensure that messages from the
  /// test framework are displayed to the developer rather than logged
  /// by whatever code is overriding [debugPrint].
  DebugPrintCallback get debugPrintOverride => debugPrint;

  /// The value to set [debugDisableShadows] to while tests are running.
  ///
  /// This can be used to reduce the likelihood of golden file tests being
  /// flaky, because shadow rendering is not always deterministic. The
  /// [AutomatedTestWidgetsFlutterBinding] sets this to true, so that all tests
  /// always run with shadows disabled.
  @protected
  bool get disableShadows => false;

  /// Determines whether the Dart [HttpClient] class should be overridden to
  /// always return a failure response.
  ///
  /// By default, this value is true, so that unit tests will not become flaky
  /// due to intermittent network errors. The value may be overridden by a
  /// binding intended for use in integration tests that do end to end
  /// application testing, including working with real network responses.
  @protected
  bool get overrideHttpClient => true;

  /// Determines whether the binding automatically registers [testTextInput] as
  /// a fake keyboard implementation.
  ///
  /// Unit tests make use of this to mock out text input communication for
  /// widgets. An integration test would set this to false, to test real IME
  /// or keyboard input.
  ///
  /// [TestTextInput.isRegistered] reports whether the text input mock is
  /// registered or not.
  ///
  /// Some of the properties and methods on [testTextInput] are only valid if
  /// [registerTestTextInput] returns true when a test starts. If those
  /// members are accessed when using a binding that sets this flag to false,
  /// they will throw.
  ///
  /// If this property returns true when a test ends, the [testTextInput] is
  /// unregistered.
  ///
  /// This property should not change the value it returns during the lifetime
  /// of the binding. Changing the value of this property risks very confusing
  /// behavior as the [TestTextInput] may be inconsistently registered or
  /// unregistered.
  @protected
  bool get registerTestTextInput => true;

  /// Delay for `duration` of time.
  ///
  /// In the automated test environment ([AutomatedTestWidgetsFlutterBinding],
  /// typically used in `flutter test`), this advances the fake [clock] for the
  /// period.
  ///
  /// In the live test environment ([LiveTestWidgetsFlutterBinding], typically
  /// used for `flutter run` and for [e2e](https://pub.dev/packages/e2e)), it is
  /// equivalent to [Future.delayed].
  Future<void> delayed(Duration duration);

  /// The current [TestWidgetsFlutterBinding], if one has been created.
  ///
  /// Provides access to the features exposed by this binding. The binding must
  /// be initialized before using this getter; this is typically done by calling
  /// [testWidgets] or [TestWidgetsFlutterBinding.ensureInitialized].
  static TestWidgetsFlutterBinding get instance => BindingBase.checkInstance(_instance);
  static TestWidgetsFlutterBinding? _instance;

  /// Creates and initializes the binding. This function is
  /// idempotent; calling it a second time will just return the
  /// previously-created instance.
  ///
  /// This function will use [AutomatedTestWidgetsFlutterBinding] if
  /// the test was run using `flutter test`, and
  /// [LiveTestWidgetsFlutterBinding] otherwise (e.g. if it was run
  /// using `flutter run`). This is determined by looking at the
  /// environment variables for a variable called `FLUTTER_TEST`.
  ///
  /// If `FLUTTER_TEST` is set with a value of 'true', then this test was
  /// invoked by `flutter test`. If `FLUTTER_TEST` is not set, or if it is set
  /// to 'false', then this test was invoked by `flutter run`.
  ///
  /// Browser environments do not currently support the
  /// [LiveTestWidgetsFlutterBinding], so this function will always set up an
  /// [AutomatedTestWidgetsFlutterBinding] when run in a web browser.
  ///
  /// The parameter `environment` is used to test the test framework
  /// itself by checking how it reacts to different environment
  /// variable values, and should not be used outside of this context.
  ///
  /// If a [TestWidgetsFlutterBinding] subclass was explicitly initialized
  /// before calling [ensureInitialized], then that version of the binding is
  /// returned regardless of the logic described above. This allows tests to
  /// force a specific test binding to be used.
  ///
  /// This is called automatically by [testWidgets].
  static TestWidgetsFlutterBinding ensureInitialized([
    @visibleForTesting Map<String, String>? environment,
  ]) {
    return _instance ?? binding.ensureInitialized(environment);
  }

  @override
  void initInstances() {
    // This is initialized here because it's needed for the `super.initInstances`
    // call. It can't be handled as a ctor initializer because it's dependent
    // on `platformDispatcher`. It can't be handled in the ctor itself because
    // the base class ctor is called first and calls `initInstances`.
    window = TestWindow.fromPlatformDispatcher(platformDispatcher: platformDispatcher);

    super.initInstances();
    _instance = this;
    timeDilation = 1.0; // just in case the developer has artificially changed it for development
    if (overrideHttpClient) {
      binding.setupHttpOverrides();
    }
    _testTextInput = TestTextInput(onCleared: _resetFocusedEditable);
  }

  @override
  // ignore: must_call_super
  void initLicenses() {
    // Do not include any licenses, because we're a test, and the LICENSE file
    // doesn't get generated for tests.
  }

  @override
  bool debugCheckZone(String entryPoint) {
    // We skip all the zone checks in tests because the test framework makes heavy use
    // of zones and so the zones never quite match the way the framework expects.
    return true;
  }

  /// Whether there is currently a test executing.
  bool get inTest;

  /// The number of outstanding microtasks in the queue.
  int get microtaskCount;

  /// The default test timeout for tests when using this binding.
  ///
  /// This controls the default for the `timeout` argument on [testWidgets]. It
  /// is 10 minutes for [AutomatedTestWidgetsFlutterBinding] (tests running
  /// using `flutter test`), and unlimited for tests using
  /// [LiveTestWidgetsFlutterBinding] (tests running using `flutter run`).
  test_package.Timeout get defaultTestTimeout;

  /// The current time.
  ///
  /// In the automated test environment (`flutter test`), this is a fake clock
  /// that begins in January 2015 at the start of the test and advances each
  /// time [pump] is called with a non-zero duration.
  ///
  /// In the live testing environment (`flutter run`), this object shows the
  /// actual current wall-clock time.
  Clock get clock;

  @override
  SamplingClock? get debugSamplingClock => _TestSamplingClock(clock);

  /// Triggers a frame sequence (build/layout/paint/etc),
  /// then flushes microtasks.
  ///
  /// If duration is set, then advances the clock by that much first.
  /// Doing this flushes microtasks.
  ///
  /// The supplied EnginePhase is the final phase reached during the pump pass;
  /// if not supplied, the whole pass is executed.
  ///
  /// See also [LiveTestWidgetsFlutterBindingFramePolicy], which affects how
  /// this method works when the test is run with `flutter run`.
  Future<void> pump([Duration? duration, EnginePhase newPhase = EnginePhase.sendSemanticsUpdate]);

  /// Runs a `callback` that performs real asynchronous work.
  ///
  /// This is intended for callers who need to call asynchronous methods where
  /// the methods spawn isolates or OS threads and thus cannot be executed
  /// synchronously by calling [pump].
  ///
  /// The `callback` must return a [Future] that completes to a value of type
  /// `T`.
  ///
  /// If `callback` completes successfully, this will return the future
  /// returned by `callback`.
  ///
  /// If `callback` completes with an error, the error will be caught by the
  /// Flutter framework and made available via [takeException], and this method
  /// will return a future that completes with `null`.
  ///
  /// Re-entrant calls to this method are not allowed; callers of this method
  /// are required to wait for the returned future to complete before calling
  /// this method again. Attempts to do otherwise will result in a
  /// [TestFailure] error being thrown.
  Future<T?> runAsync<T>(Future<T> Function() callback);

  /// Artificially calls dispatchLocalesChanged on the Widget binding,
  /// then flushes microtasks.
  ///
  /// Passes only one single Locale. Use [setLocales] to pass a full preferred
  /// locales list.
  Future<void> setLocale(String languageCode, String countryCode) {
    return TestAsyncUtils.guard<void>(() async {
      assert(inTest);
      final Locale locale = Locale(languageCode, countryCode == '' ? null : countryCode);
      dispatchLocalesChanged(<Locale>[locale]);
    });
  }

  /// Artificially calls dispatchLocalesChanged on the Widget binding,
  /// then flushes microtasks.
  Future<void> setLocales(List<Locale> locales) {
    return TestAsyncUtils.guard<void>(() async {
      assert(inTest);
      dispatchLocalesChanged(locales);
    });
  }

  @override
  Future<ui.AppExitResponse> exitApplication(ui.AppExitType exitType, [int exitCode = 0]) async {
    switch (exitType) {
      case ui.AppExitType.cancelable:
        // The test framework shouldn't actually exit when requested.
        return ui.AppExitResponse.cancel;
      case ui.AppExitType.required:
        throw FlutterError('Unexpected application exit request while running test');
    }
  }

  /// Re-attempts the initialization of the lifecycle state after providing
  /// test values in [TestPlatformDispatcher.initialLifecycleStateTestValue].
  void readTestInitialLifecycleStateFromNativeWindow() {
    readInitialLifecycleStateFromNativeWindow();
  }

  Size? _surfaceSize;

  /// Artificially changes the logical size of [WidgetTester.view] to the
  /// specified size, then flushes microtasks.
  ///
  /// Set to null to use the default surface size.
  ///
  /// To avoid affecting other tests by leaking state, a test that
  /// uses this method should always reset the surface size to the default.
  /// For example, using `addTearDown`:
  /// ```dart
  ///   await binding.setSurfaceSize(someSize);
  ///   addTearDown(() => binding.setSurfaceSize(null));
  /// ```
  ///
  /// This method only affects the size of the [WidgetTester.view]. It does not
  /// affect the size of any other views. Instead of this method, consider
  /// setting [TestFlutterView.physicalSize], which works for any view,
  /// including [WidgetTester.view].
  // TODO(pdblasi-google): Deprecate this. https://github.com/flutter/flutter/issues/123881
  Future<void> setSurfaceSize(Size? size) {
    return TestAsyncUtils.guard<void>(() async {
      assert(inTest);
      if (_surfaceSize == size) {
        return;
      }
      _surfaceSize = size;
      handleMetricsChanged();
    });
  }

  @override
  void addRenderView(RenderView view) {
    _insideAddRenderView = true;
    try {
      super.addRenderView(view);
    } finally {
      _insideAddRenderView = false;
    }
  }

  bool _insideAddRenderView = false;

  @override
  ViewConfiguration createViewConfigurationFor(RenderView renderView) {
    if (_insideAddRenderView &&
        renderView.hasConfiguration &&
        renderView.configuration is TestViewConfiguration &&
        renderView == this.renderView) {
      // If a test has reached out to the now deprecated renderView property to set a custom TestViewConfiguration
      // we are not replacing it. This is to maintain backwards compatibility with how things worked prior to the
      // deprecation of that property.
      // TODO(goderbauer): Remove this "if" when the deprecated renderView property is removed.
      return renderView.configuration;
    }
    final FlutterView view = renderView.flutterView;
    if (_surfaceSize != null && view == platformDispatcher.implicitView) {
      final BoxConstraints constraints = BoxConstraints.tight(_surfaceSize!);
      return ViewConfiguration(
        logicalConstraints: constraints,
        physicalConstraints: constraints * view.devicePixelRatio,
        devicePixelRatio: view.devicePixelRatio,
      );
    }
    return super.createViewConfigurationFor(renderView);
  }

  /// Acts as if the application went idle.
  ///
  /// Runs all remaining microtasks, including those scheduled as a result of
  /// running them, until there are no more microtasks scheduled. Then, runs any
  /// previously scheduled timers with zero time, and completes the returned future.
  ///
  /// May result in an infinite loop or run out of memory if microtasks continue
  /// to recursively schedule new microtasks. Will not run any timers scheduled
  /// after this method was invoked, even if they are zero-time timers.
  Future<void> idle() {
    return TestAsyncUtils.guard<void>(() {
      final Completer<void> completer = Completer<void>();
      Timer.run(() {
        completer.complete();
      });
      return completer.future;
    });
  }

  /// Convert the given point from the global coordinate space of the provided
  /// [RenderView] to its local one.
  ///
  /// This method operates in logical pixels for both coordinate spaces. It does
  /// not apply the device pixel ratio (used to translate to/from physical
  /// pixels).
  ///
  /// For definitions for coordinate spaces, see [TestWidgetsFlutterBinding].
  Offset globalToLocal(Offset point, RenderView view) => point;

  /// Convert the given point from the local coordinate space to the global
  /// coordinate space of the [RenderView].
  ///
  /// This method operates in logical pixels for both coordinate spaces. It does
  /// not apply the device pixel ratio to translate to physical pixels.
  ///
  /// For definitions for coordinate spaces, see [TestWidgetsFlutterBinding].
  Offset localToGlobal(Offset point, RenderView view) => point;

  /// The source of the current pointer event.
  ///
  /// The [pointerEventSource] is set as the `source` parameter of
  /// [handlePointerEventForSource] and can be used in the immediate enclosing
  /// [dispatchEvent].
  ///
  /// When [handlePointerEvent] is called directly, [pointerEventSource]
  /// is [TestBindingEventSource.device].
  ///
  /// This means that pointer events triggered by the [WidgetController] (e.g.
  /// via [WidgetController.tap]) will result in actual interactions with the
  /// UI, but other pointer events such as those from physical taps will be
  /// dropped. See also [shouldPropagateDevicePointerEvents] if this is
  /// undesired.
  TestBindingEventSource get pointerEventSource => _pointerEventSource;
  TestBindingEventSource _pointerEventSource = TestBindingEventSource.device;

  /// Whether pointer events from [TestBindingEventSource.device] will be
  /// propagated to the framework, or dropped.
  ///
  /// Setting this can be useful to interact with the app in some other way
  /// besides through the [WidgetController], such as with `adb shell input tap`
  /// on Android.
  ///
  /// See also [pointerEventSource].
  bool shouldPropagateDevicePointerEvents = false;

  /// Dispatch an event to the targets found by a hit test on its position,
  /// and remember its source as [pointerEventSource].
  ///
  /// This method sets [pointerEventSource] to `source`, forwards the call to
  /// [handlePointerEvent], then resets [pointerEventSource] to the previous
  /// value.
  ///
  /// If `source` is [TestBindingEventSource.device], then the `event` is based
  /// in the global coordinate space (for definitions for coordinate spaces,
  /// see [TestWidgetsFlutterBinding]) and the event is likely triggered by the
  /// user physically interacting with the screen during a live test on a real
  /// device (see [LiveTestWidgetsFlutterBinding]).
  ///
  /// If `source` is [TestBindingEventSource.test], then the `event` is based
  /// in the local coordinate space and the event is likely triggered by
  /// programmatically simulated pointer events, such as:
  ///
  ///  * [WidgetController.tap] and alike methods, as well as directly using
  ///    [TestGesture]. They are usually used in
  ///    [AutomatedTestWidgetsFlutterBinding] but sometimes in live tests too.
  ///  * [WidgetController.timedDrag] and alike methods. They are usually used
  ///    in macrobenchmarks.
  void handlePointerEventForSource(
    PointerEvent event, {
    TestBindingEventSource source = TestBindingEventSource.device,
  }) {
    withPointerEventSource(source, () => handlePointerEvent(event));
  }

  /// Sets [pointerEventSource] to `source`, runs `task`, then resets `source`
  /// to the previous value.
  @protected
  void withPointerEventSource(TestBindingEventSource source, VoidCallback task) {
    final TestBindingEventSource previousSource = _pointerEventSource;
    _pointerEventSource = source;
    try {
      task();
    } finally {
      _pointerEventSource = previousSource;
    }
  }

  /// A stub for the system's onscreen keyboard. Callers must set the
  /// [focusedEditable] before using this value.
  TestTextInput get testTextInput => _testTextInput;
  late TestTextInput _testTextInput;

  /// The [State] of the current [EditableText] client of the onscreen keyboard.
  ///
  /// Setting this property to a new value causes the given [EditableTextState]
  /// to focus itself and request the keyboard to establish a
  /// [TextInputConnection].
  ///
  /// Callers must pump an additional frame after setting this property to
  /// complete the focus change.
  ///
  /// Instead of setting this directly, consider using
  /// [WidgetTester.showKeyboard].
  //
  // TODO(ianh): We should just remove this property and move the call to
  // requestKeyboard to the WidgetTester.showKeyboard method.
  EditableTextState? get focusedEditable => _focusedEditable;
  EditableTextState? _focusedEditable;
  set focusedEditable(EditableTextState? value) {
    if (_focusedEditable != value) {
      _focusedEditable = value;
      value?.requestKeyboard();
    }
  }

  void _resetFocusedEditable() {
    _focusedEditable = null;
  }

  /// Returns the exception most recently caught by the Flutter framework.
  ///
  /// Call this if you expect an exception during a test. If an exception is
  /// thrown and this is not called, then the exception is rethrown when
  /// the [testWidgets] call completes.
  ///
  /// If two exceptions are thrown in a row without the first one being
  /// acknowledged with a call to this method, then when the second exception is
  /// thrown, they are both dumped to the console and then the second is
  /// rethrown from the exception handler. This will likely result in the
  /// framework entering a highly unstable state and everything collapsing.
  ///
  /// It's safe to call this when there's no pending exception; it will return
  /// null in that case.
  dynamic takeException() {
    assert(inTest);
    final dynamic result = _pendingExceptionDetails?.exception;
    _pendingExceptionDetails = null;
    return result;
  }

  FlutterExceptionHandler? _oldExceptionHandler;
  late StackTraceDemangler _oldStackTraceDemangler;
  FlutterErrorDetails? _pendingExceptionDetails;

  _MockMessageHandler? _announcementHandler;
  List<CapturedAccessibilityAnnouncement> _announcements = <CapturedAccessibilityAnnouncement>[];

  /// {@template flutter.flutter_test.TakeAccessibilityAnnouncements}
  /// Returns a list of all the accessibility announcements made by the Flutter
  /// framework since the last time this function was called.
  ///
  /// It's safe to call this when there hasn't been any announcements; it will return
  /// an empty list in that case.
  /// {@endtemplate}
  List<CapturedAccessibilityAnnouncement> takeAnnouncements() {
    assert(inTest);
    final List<CapturedAccessibilityAnnouncement> announcements = _announcements;
    _announcements = <CapturedAccessibilityAnnouncement>[];
    return announcements;
  }

  static const TextStyle _messageStyle = TextStyle(color: Color(0xFF917FFF), fontSize: 40.0);

  static const Widget _preTestMessage = Center(
    child: Text('Test starting...', style: _messageStyle, textDirection: TextDirection.ltr),
  );

  static const Widget _postTestMessage = Center(
    child: Text('Test finished.', style: _messageStyle, textDirection: TextDirection.ltr),
  );

  /// Whether to include the output of debugDumpApp() when reporting
  /// test failures.
  bool showAppDumpInErrors = false;

  /// Call the testBody inside a [FakeAsync] scope on which [pump] can
  /// advance time.
  ///
  /// Returns a future which completes when the test has run.
  ///
  /// Called by the [testWidgets] and [benchmarkWidgets] functions to
  /// run a test.
  ///
  /// The `invariantTester` argument is called after the `testBody`'s [Future]
  /// completes. If it throws, then the test is marked as failed.
  ///
  /// The `description` is used by the [LiveTestWidgetsFlutterBinding] to
  /// show a label on the screen during the test. The description comes from
  /// the value passed to [testWidgets].
  Future<void> runTest(
    Future<void> Function() testBody,
    VoidCallback invariantTester, {
    String description = '',
  });

  /// This is called during test execution before and after the body has been
  /// executed.
  ///
  /// It's used by [AutomatedTestWidgetsFlutterBinding] to drain the microtasks
  /// before the final [pump] that happens during test cleanup.
  void asyncBarrier() {
    TestAsyncUtils.verifyAllScopesClosed();
  }

  Zone? _parentZone;

  VoidCallback _createTestCompletionHandler(String testDescription, Completer<void> completer) {
    return () {
      // This can get called twice, in the case of a Future without listeners failing, and then
      // our main future completing.
      assert(Zone.current == _parentZone);
      if (_pendingExceptionDetails != null) {
        debugPrint =
            debugPrintOverride; // just in case the test overrides it -- otherwise we won't see the error!
        reportTestException(_pendingExceptionDetails!, testDescription);
        _pendingExceptionDetails = null;
      }
      if (!completer.isCompleted) {
        completer.complete();
      }
    };
  }

  /// Called when the framework catches an exception, even if that exception is
  /// being handled by [takeException].
  ///
  /// This is called when there is no pending exception; if multiple exceptions
  /// are thrown and [takeException] isn't used, then subsequent exceptions are
  /// logged to the console regardless (and the test will fail).
  @protected
  void reportExceptionNoticed(FlutterErrorDetails exception) {
    // By default we do nothing.
    // The LiveTestWidgetsFlutterBinding overrides this to report the exception to the console.
  }

  Future<void> _handleAnnouncementMessage(Object? mockMessage) async {
    if (mockMessage! case {
      'type': 'announce',
      'data': final Map<Object?, Object?> data as Map<Object?, Object?>,
    }) {
      _announcements.add(
        CapturedAccessibilityAnnouncement._(
          data['message'].toString(),
          TextDirection.values[data['textDirection']! as int],
          Assertiveness.values[(data['assertiveness'] ?? 0) as int],
        ),
      );
    }
  }

  Future<void> _runTest(
    Future<void> Function() testBody,
    VoidCallback invariantTester,
    String description,
  ) {
    assert(inTest);

    // Set the handler only if there is currently none.
    if (TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
      SystemChannels.accessibility.name,
      null,
    )) {
      _announcementHandler = _handleAnnouncementMessage;
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
          .setMockDecodedMessageHandler<dynamic>(
            SystemChannels.accessibility,
            _announcementHandler,
          );
    }

    _oldExceptionHandler = FlutterError.onError;
    _oldStackTraceDemangler = FlutterError.demangleStackTrace;
    int exceptionCount = 0; // number of un-taken exceptions
    FlutterError.onError = (FlutterErrorDetails details) {
      if (_pendingExceptionDetails != null) {
        debugPrint =
            debugPrintOverride; // just in case the test overrides it -- otherwise we won't see the errors!
        if (exceptionCount == 0) {
          exceptionCount = 2;
          FlutterError.dumpErrorToConsole(_pendingExceptionDetails!, forceReport: true);
        } else {
          exceptionCount += 1;
        }
        FlutterError.dumpErrorToConsole(details, forceReport: true);
        _pendingExceptionDetails = FlutterErrorDetails(
          exception:
              'Multiple exceptions ($exceptionCount) were detected during the running of the current test, and at least one was unexpected.',
          library: 'Flutter test framework',
        );
      } else {
        reportExceptionNoticed(
          details,
        ); // mostly this is just a hook for the LiveTestWidgetsFlutterBinding
        _pendingExceptionDetails = details;
      }
    };
    FlutterError.demangleStackTrace = (StackTrace stack) {
      // package:stack_trace uses ZoneSpecification.errorCallback to add useful
      // information to stack traces, meaning Trace and Chain classes can be
      // present. Because these StackTrace implementations do not follow the
      // format the framework expects, we convert them to a vm trace here.
      if (stack is stack_trace.Trace) {
        return stack.vmTrace;
      }
      if (stack is stack_trace.Chain) {
        return stack.toTrace().vmTrace;
      }
      return stack;
    };
    final Completer<void> testCompleter = Completer<void>();
    final VoidCallback testCompletionHandler = _createTestCompletionHandler(
      description,
      testCompleter,
    );
    void handleUncaughtError(Object exception, StackTrace stack) {
      if (testCompleter.isCompleted) {
        // Well this is not a good sign.
        // Ideally, once the test has failed we would stop getting errors from the test.
        // However, if someone tries hard enough they could get in a state where this happens.
        // If we silently dropped these errors on the ground, nobody would ever know. So instead
        // we raise them and fail the test after it has already completed.
        debugPrint =
            debugPrintOverride; // just in case the test overrides it -- otherwise we won't see the error!
        reportTestException(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            context: ErrorDescription('running a test (but after the test had completed)'),
            library: 'Flutter test framework',
          ),
          description,
        );
        return;
      }
      // This is where test failures, e.g. those in expect(), will end up.
      // Specifically, runUnaryGuarded() will call this synchronously and
      // return our return value if _runTestBody fails synchronously (which it
      // won't, so this never happens), and Future will call this when the
      // Future completes with an error and it would otherwise call listeners
      // if the listener is in a different zone (which it would be for the
      // `whenComplete` handler below), or if the Future completes with an
      // error and the future has no listeners at all.
      //
      // This handler further calls the onError handler above, which sets
      // _pendingExceptionDetails. Nothing gets printed as a result of that
      // call unless we already had an exception pending, because in general
      // we want people to be able to cause the framework to report exceptions
      // and then use takeException to verify that they were really caught.
      // Now, if we actually get here, this isn't going to be one of those
      // cases. We only get here if the test has actually failed. So, once
      // we've carefully reported it, we then immediately end the test by
      // calling the testCompletionHandler in the _parentZone.
      //
      // We have to manually call testCompletionHandler because if the Future
      // library calls us, it is maybe _instead_ of calling a registered
      // listener from a different zone. In our case, that would be instead of
      // calling the whenComplete() listener below.
      //
      // We have to call it in the parent zone because if we called it in
      // _this_ zone, the test framework would find this zone was the current
      // zone and helpfully throw the error in this zone, causing us to be
      // directly called again.
      DiagnosticsNode treeDump;
      try {
        treeDump = rootElement?.toDiagnosticsNode() ?? DiagnosticsNode.message('<no tree>');
        // We try to stringify the tree dump here (though we immediately discard the result) because
        // we want to make sure that if it can't be serialized, we replace it with a message that
        // says the tree could not be serialized. Otherwise, the real exception might get obscured
        // by side-effects of the underlying issues causing the tree dumping code to flail.
        treeDump.toStringDeep();
      } catch (exception) {
        treeDump = DiagnosticsNode.message(
          '<additional error caught while dumping tree: $exception>',
          level: DiagnosticLevel.error,
        );
      }
      final List<DiagnosticsNode> omittedFrames = <DiagnosticsNode>[];
      final int stackLinesToOmit = reportExpectCall(stack, omittedFrames);
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          context: ErrorDescription('running a test'),
          library: 'Flutter test framework',
          stackFilter: (Iterable<String> frames) {
            return FlutterError.defaultStackFilter(frames.skip(stackLinesToOmit));
          },
          informationCollector: () sync* {
            if (stackLinesToOmit > 0) {
              yield* omittedFrames;
            }
            if (showAppDumpInErrors) {
              yield DiagnosticsProperty<DiagnosticsNode>(
                'At the time of the failure, the widget tree looked as follows',
                treeDump,
                linePrefix: '# ',
                style: DiagnosticsTreeStyle.flat,
              );
            }
            if (description.isNotEmpty) {
              yield DiagnosticsProperty<String>(
                'The test description was',
                description,
                style: DiagnosticsTreeStyle.errorProperty,
              );
            }
          },
        ),
      );
      assert(_parentZone != null);
      assert(
        _pendingExceptionDetails != null,
        'A test overrode FlutterError.onError but either failed to return it to its original state, or had unexpected additional errors that it could not handle. Typically, this is caused by using expect() before restoring FlutterError.onError.',
      );
      _parentZone!.run<void>(testCompletionHandler);
    }

    final ZoneSpecification errorHandlingZoneSpecification = ZoneSpecification(
      handleUncaughtError: (
        Zone self,
        ZoneDelegate parent,
        Zone zone,
        Object exception,
        StackTrace stack,
      ) {
        handleUncaughtError(exception, stack);
      },
    );
    _parentZone = Zone.current;
    final Zone testZone = _parentZone!.fork(specification: errorHandlingZoneSpecification);
    testZone
        .runBinary<Future<void>, Future<void> Function(), VoidCallback>(
          _runTestBody,
          testBody,
          invariantTester,
        )
        .whenComplete(testCompletionHandler);
    return testCompleter.future;
  }

  Future<void> _runTestBody(Future<void> Function() testBody, VoidCallback invariantTester) async {
    assert(inTest);
    // So that we can assert that it remains the same after the test finishes.
    _beforeTestCheckIntrinsicSizes = debugCheckIntrinsicSizes;

    runApp(Container(key: UniqueKey(), child: _preTestMessage)); // Reset the tree to a known state.
    await pump();
    // Pretend that the first frame produced in the test body is the first frame
    // sent to the engine.
    resetFirstFrameSent();

    final bool autoUpdateGoldensBeforeTest = autoUpdateGoldenFiles && !isBrowser;
    final TestExceptionReporter reportTestExceptionBeforeTest = reportTestException;
    final ErrorWidgetBuilder errorWidgetBuilderBeforeTest = ErrorWidget.builder;
    final bool shouldPropagateDevicePointerEventsBeforeTest = shouldPropagateDevicePointerEvents;

    // run the test
    await testBody();
    asyncBarrier(); // drains the microtasks in `flutter test` mode (when using AutomatedTestWidgetsFlutterBinding)

    if (_pendingExceptionDetails == null) {
      // We only try to clean up and verify invariants if we didn't already
      // fail. If we got an exception already, then we instead leave everything
      // alone so that we don't cause more spurious errors.
      runApp(
        Container(key: UniqueKey(), child: _postTestMessage),
      ); // Unmount any remaining widgets.
      await pump();
      if (registerTestTextInput) {
        _testTextInput.unregister();
      }
      invariantTester();
      _verifyAutoUpdateGoldensUnset(autoUpdateGoldensBeforeTest && !isBrowser);
      _verifyReportTestExceptionUnset(reportTestExceptionBeforeTest);
      _verifyErrorWidgetBuilderUnset(errorWidgetBuilderBeforeTest);
      _verifyShouldPropagateDevicePointerEventsUnset(shouldPropagateDevicePointerEventsBeforeTest);
      _verifyInvariants();
    }

    assert(inTest);
    asyncBarrier(); // When using AutomatedTestWidgetsFlutterBinding, this flushes the microtasks.
  }

  late bool _beforeTestCheckIntrinsicSizes;

  void _verifyInvariants() {
    assert(
      debugAssertNoTransientCallbacks(
        'An animation is still running even after the widget tree was disposed.',
      ),
    );
    assert(
      debugAssertNoPendingPerformanceModeRequests(
        'A performance mode was requested and not disposed by a test.',
      ),
    );
    assert(debugAssertNoTimeDilation('The timeDilation was changed and not reset by the test.'));
    assert(
      debugAssertAllFoundationVarsUnset(
        'The value of a foundation debug variable was changed by the test.',
        debugPrintOverride: debugPrintOverride,
      ),
    );
    assert(
      debugAssertAllGesturesVarsUnset(
        'The value of a gestures debug variable was changed by the test.',
      ),
    );
    assert(
      debugAssertAllPaintingVarsUnset(
        'The value of a painting debug variable was changed by the test.',
        debugDisableShadowsOverride: disableShadows,
      ),
    );
    assert(
      debugAssertAllRenderVarsUnset(
        'The value of a rendering debug variable was changed by the test.',
        debugCheckIntrinsicSizesOverride: _beforeTestCheckIntrinsicSizes,
      ),
    );
    assert(
      debugAssertAllWidgetVarsUnset(
        'The value of a widget debug variable was changed by the test.',
      ),
    );
    assert(
      debugAssertAllSchedulerVarsUnset(
        'The value of a scheduler debug variable was changed by the test.',
      ),
    );
    assert(
      debugAssertAllServicesVarsUnset(
        'The value of a services debug variable was changed by the test.',
      ),
    );
  }

  void _verifyAutoUpdateGoldensUnset(bool valueBeforeTest) {
    assert(() {
      if (autoUpdateGoldenFiles != valueBeforeTest) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: FlutterError('The value of autoUpdateGoldenFiles was changed by the test.'),
            stack: StackTrace.current,
            library: 'Flutter test framework',
          ),
        );
      }
      return true;
    }());
  }

  void _verifyReportTestExceptionUnset(TestExceptionReporter valueBeforeTest) {
    assert(() {
      if (reportTestException != valueBeforeTest) {
        // We can't report this error to their modified reporter because we
        // can't be guaranteed that their reporter will cause the test to fail.
        // So we reset the error reporter to its initial value and then report
        // this error.
        reportTestException = valueBeforeTest;
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: FlutterError('The value of reportTestException was changed by the test.'),
            stack: StackTrace.current,
            library: 'Flutter test framework',
          ),
        );
      }
      return true;
    }());
  }

  void _verifyErrorWidgetBuilderUnset(ErrorWidgetBuilder valueBeforeTest) {
    assert(() {
      if (ErrorWidget.builder != valueBeforeTest) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: FlutterError('The value of ErrorWidget.builder was changed by the test.'),
            stack: StackTrace.current,
            library: 'Flutter test framework',
          ),
        );
      }
      return true;
    }());
  }

  void _verifyShouldPropagateDevicePointerEventsUnset(bool valueBeforeTest) {
    assert(() {
      if (shouldPropagateDevicePointerEvents != valueBeforeTest) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: FlutterError(
              'The value of shouldPropagateDevicePointerEvents was changed by the test.',
            ),
            stack: StackTrace.current,
            library: 'Flutter test framework',
          ),
        );
      }
      return true;
    }());
  }

  /// Called by the [testWidgets] function after a test is executed.
  void postTest() {
    assert(inTest);
    FlutterError.onError = _oldExceptionHandler;
    FlutterError.demangleStackTrace = _oldStackTraceDemangler;
    _pendingExceptionDetails = null;
    _parentZone = null;
    buildOwner!.focusManager.dispose();

    if (TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
      SystemChannels.accessibility.name,
      _announcementHandler,
    )) {
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
          .setMockDecodedMessageHandler(SystemChannels.accessibility, null);
      _announcementHandler = null;
    }
    _announcements = <CapturedAccessibilityAnnouncement>[];

    ServicesBinding.instance.keyEventManager.keyMessageHandler = null;
    buildOwner!.focusManager = FocusManager()..registerGlobalHandlers();

    // Disabling the warning because @visibleForTesting doesn't take the testing
    // framework itself into account, but we don't want it visible outside of
    // tests.
    // ignore: invalid_use_of_visible_for_testing_member
    RawKeyboard.instance.clearKeysPressed();
    // ignore: invalid_use_of_visible_for_testing_member
    HardwareKeyboard.instance.clearState();
    // ignore: invalid_use_of_visible_for_testing_member
    keyEventManager.clearState();
    // ignore: invalid_use_of_visible_for_testing_member
    RendererBinding.instance.initMouseTracker();

    assert(ServicesBinding.instance == WidgetsBinding.instance);
    // ignore: invalid_use_of_visible_for_testing_member
    ServicesBinding.instance.resetInternalState();
  }
}

/// A variant of [TestWidgetsFlutterBinding] for executing tests typically
/// the `flutter test` environment, unless it is an integration test.
///
/// When doing integration test, [LiveTestWidgetsFlutterBinding] is utilized
/// instead.
///
/// This binding controls time, allowing tests to verify long
/// animation sequences without having to execute them in real time.
///
/// This class assumes it is always run in debug mode (since tests are always
/// run in debug mode).
///
/// See [TestWidgetsFlutterBinding] for a list of mixins that must be
/// provided by the binding active while the test framework is
/// running.
class AutomatedTestWidgetsFlutterBinding extends TestWidgetsFlutterBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
    binding.mockFlutterAssets();
  }

  /// The current [AutomatedTestWidgetsFlutterBinding], if one has been created.
  ///
  /// The binding must be initialized before using this getter. If you
  /// need the binding to be constructed before calling [testWidgets],
  /// you can ensure a binding has been constructed by calling the
  /// [TestWidgetsFlutterBinding.ensureInitialized] function.
  static AutomatedTestWidgetsFlutterBinding get instance => BindingBase.checkInstance(_instance);
  static AutomatedTestWidgetsFlutterBinding? _instance;

  /// Returns an instance of the binding that implements
  /// [AutomatedTestWidgetsFlutterBinding]. If no binding has yet been
  /// initialized, the a new instance is created.
  ///
  /// Generally, there is no need to call this method. Use
  /// [TestWidgetsFlutterBinding.ensureInitialized] instead, as it
  /// will select the correct test binding implementation
  /// automatically.
  static AutomatedTestWidgetsFlutterBinding ensureInitialized() {
    if (AutomatedTestWidgetsFlutterBinding._instance == null) {
      AutomatedTestWidgetsFlutterBinding();
    }
    return AutomatedTestWidgetsFlutterBinding.instance;
  }

  FakeAsync? _currentFakeAsync; // set in runTest; cleared in postTest
  Completer<void>? _pendingAsyncTasks;

  @override
  Clock get clock {
    assert(inTest);
    return _clock!;
  }

  Clock? _clock;

  @override
  DebugPrintCallback get debugPrintOverride => debugPrintSynchronously;

  @override
  bool get disableShadows => true;

  /// The value of [defaultTestTimeout] can be set to `None` to enable debugging
  /// flutter tests where we would not want to timeout the test. This is
  /// expected to be used by test tooling which can detect debug mode.
  @override
  test_package.Timeout defaultTestTimeout = const test_package.Timeout(Duration(minutes: 10));

  @override
  bool get inTest => _currentFakeAsync != null;

  @override
  int get microtaskCount => _currentFakeAsync!.microtaskCount;

  @override
  Future<void> pump([Duration? duration, EnginePhase newPhase = EnginePhase.sendSemanticsUpdate]) {
    return TestAsyncUtils.guard<void>(() {
      assert(inTest);
      assert(_clock != null);
      if (duration != null) {
        _currentFakeAsync!.elapse(duration);
      }
      _phase = newPhase;
      if (hasScheduledFrame) {
        _currentFakeAsync!.flushMicrotasks();
        handleBeginFrame(Duration(microseconds: _clock!.now().microsecondsSinceEpoch));
        _currentFakeAsync!.flushMicrotasks();
        handleDrawFrame();
      }
      _currentFakeAsync!.flushMicrotasks();
      return Future<void>.value();
    });
  }

  @override
  Future<T?> runAsync<T>(Future<T> Function() callback) {
    assert(() {
      if (_pendingAsyncTasks == null) {
        return true;
      }
      fail(
        'Reentrant call to runAsync() denied.\n'
        'runAsync() was called, then before its future completed, it '
        'was called again. You must wait for the first returned future '
        'to complete before calling runAsync() again.',
      );
    }());

    final Zone realAsyncZone = Zone.current.fork(
      specification: ZoneSpecification(
        scheduleMicrotask: (Zone self, ZoneDelegate parent, Zone zone, void Function() f) {
          _rootDelegate.scheduleMicrotask(zone, f);
        },
        createTimer: (
          Zone self,
          ZoneDelegate parent,
          Zone zone,
          Duration duration,
          void Function() f,
        ) {
          return _rootDelegate.createTimer(zone, duration, f);
        },
        createPeriodicTimer: (
          Zone self,
          ZoneDelegate parent,
          Zone zone,
          Duration period,
          void Function(Timer timer) f,
        ) {
          return _rootDelegate.createPeriodicTimer(zone, period, f);
        },
      ),
    );

    return realAsyncZone.run<Future<T?>>(() {
      final Completer<T?> result = Completer<T?>();
      _pendingAsyncTasks = Completer<void>();
      try {
        callback().then(result.complete).catchError((Object exception, StackTrace stack) {
          FlutterError.reportError(
            FlutterErrorDetails(
              exception: exception,
              stack: stack,
              library: 'Flutter test framework',
              context: ErrorDescription('while running async test code'),
              informationCollector: () {
                return <DiagnosticsNode>[ErrorHint('The exception was caught asynchronously.')];
              },
            ),
          );
          result.complete(null);
        });
      } catch (exception, stack) {
        FlutterError.reportError(
          FlutterErrorDetails(
            exception: exception,
            stack: stack,
            library: 'Flutter test framework',
            context: ErrorDescription('while running async test code'),
            informationCollector: () {
              return <DiagnosticsNode>[ErrorHint('The exception was caught synchronously.')];
            },
          ),
        );
        result.complete(null);
      }
      result.future.whenComplete(() {
        _pendingAsyncTasks!.complete();
        _pendingAsyncTasks = null;
      });
      return result.future;
    });
  }

  @override
  void ensureFrameCallbacksRegistered() {
    // Leave PlatformDispatcher alone, do nothing.
    assert(platformDispatcher.onDrawFrame == null);
    assert(platformDispatcher.onBeginFrame == null);
  }

  @override
  void scheduleWarmUpFrame() {
    // We override the default version of this so that the application-startup warm-up frame
    // does not schedule timers which we might never get around to running.
    assert(inTest);
    handleBeginFrame(null);
    _currentFakeAsync!.flushMicrotasks();
    handleDrawFrame();
    _currentFakeAsync!.flushMicrotasks();
  }

  /// The [ZoneDelegate] for [Zone.root].
  ///
  /// Used to schedule (real) microtasks and timers in the root zone,
  /// to be run in the correct zone.
  static final ZoneDelegate _rootDelegate = _captureRootZoneDelegate();

  /// Hack to extract the [ZoneDelegate] for [Zone.root].
  static ZoneDelegate _captureRootZoneDelegate() {
    final Zone captureZone = Zone.root.fork(
      specification: ZoneSpecification(
        run: <R>(Zone self, ZoneDelegate parent, Zone zone, R Function() f) {
          return parent as R;
        },
      ),
    );
    // The `_captureRootZoneDelegate` argument just happens to be a constant
    // function with the necessary type. It's not called recursively.
    return captureZone.run<ZoneDelegate>(_captureRootZoneDelegate);
  }

  @override
  void scheduleAttachRootWidget(Widget rootWidget) {
    // We override the default version of this so that the application-startup widget tree
    // build does not schedule timers which we might never get around to running.
    assert(inTest);
    attachRootWidget(rootWidget);
    _currentFakeAsync!.flushMicrotasks();
  }

  @override
  Future<void> idle() {
    assert(inTest);
    final Future<void> result = super.idle();
    _currentFakeAsync!.elapse(Duration.zero);
    return result;
  }

  int _firstFrameDeferredCount = 0;
  bool _firstFrameSent = false;

  @override
  bool get sendFramesToEngine => _firstFrameSent || _firstFrameDeferredCount == 0;

  @override
  void deferFirstFrame() {
    assert(_firstFrameDeferredCount >= 0);
    _firstFrameDeferredCount += 1;
  }

  @override
  void allowFirstFrame() {
    assert(_firstFrameDeferredCount > 0);
    _firstFrameDeferredCount -= 1;
    // Unlike in RendererBinding.allowFirstFrame we do not force a frame here
    // to give the test full control over frame scheduling.
  }

  @override
  void resetFirstFrameSent() {
    _firstFrameSent = false;
  }

  EnginePhase _phase = EnginePhase.sendSemanticsUpdate;

  // Cloned from RendererBinding.drawFrame() but with early-exit semantics.
  @override
  void drawFrame() {
    assert(inTest);
    try {
      debugBuildingDirtyElements = true;
      buildOwner!.buildScope(rootElement!);
      if (_phase != EnginePhase.build) {
        rootPipelineOwner.flushLayout();
        if (_phase != EnginePhase.layout) {
          rootPipelineOwner.flushCompositingBits();
          if (_phase != EnginePhase.compositingBits) {
            rootPipelineOwner.flushPaint();
            if (_phase != EnginePhase.paint && sendFramesToEngine) {
              _firstFrameSent = true;
              for (final RenderView renderView in renderViews) {
                renderView.compositeFrame(); // this sends the bits to the GPU
              }
              if (_phase != EnginePhase.composite) {
                rootPipelineOwner.flushSemantics(); // this sends the semantics to the OS.
                assert(
                  _phase == EnginePhase.flushSemantics || _phase == EnginePhase.sendSemanticsUpdate,
                );
              }
            }
          }
        }
      }
      buildOwner!.finalizeTree();
    } finally {
      debugBuildingDirtyElements = false;
    }
  }

  @override
  Future<void> delayed(Duration duration) {
    assert(_currentFakeAsync != null);
    _currentFakeAsync!.elapse(duration);
    return Future<void>.value();
  }

  /// Simulates the synchronous passage of time, resulting from blocking or
  /// expensive calls.
  void elapseBlocking(Duration duration) {
    _currentFakeAsync!.elapseBlocking(duration);
  }

  @override
  Future<void> runTest(
    Future<void> Function() testBody,
    VoidCallback invariantTester, {
    String description = '',
  }) {
    assert(!inTest);
    assert(_currentFakeAsync == null);
    assert(_clock == null);

    final FakeAsync fakeAsync = FakeAsync();
    _currentFakeAsync = fakeAsync; // reset in postTest
    _clock = fakeAsync.getClock(DateTime.utc(2015));
    late Future<void> testBodyResult;
    fakeAsync.run((FakeAsync localFakeAsync) {
      assert(fakeAsync == _currentFakeAsync);
      assert(fakeAsync == localFakeAsync);
      testBodyResult = _runTest(testBody, invariantTester, description);
      assert(inTest);
    });

    return Future<void>.microtask(() async {
      // testBodyResult is a Future that was created in the Zone of the
      // fakeAsync. This means that if we await it here, it will register a
      // microtask to handle the future _in the fake async zone_. We avoid this
      // by calling '.then' in the current zone. While flushing the microtasks
      // of the fake-zone below, the new future will be completed and can then
      // be used without fakeAsync.

      final Future<void> resultFuture = testBodyResult.then<void>((_) {
        // Do nothing.
      });

      // Resolve interplay between fake async and real async calls.
      fakeAsync.flushMicrotasks();
      while (_pendingAsyncTasks != null) {
        await _pendingAsyncTasks!.future;
        fakeAsync.flushMicrotasks();
      }
      return resultFuture;
    });
  }

  @override
  void asyncBarrier() {
    assert(_currentFakeAsync != null);
    _currentFakeAsync!.flushMicrotasks();
    super.asyncBarrier();
  }

  @override
  void _verifyInvariants() {
    super._verifyInvariants();

    assert(inTest);

    bool timersPending = false;
    if (_currentFakeAsync!.periodicTimerCount != 0 ||
        _currentFakeAsync!.nonPeriodicTimerCount != 0) {
      debugPrint('Pending timers:');
      for (final FakeTimer timer in _currentFakeAsync!.pendingTimers) {
        debugPrint(
          'Timer (duration: ${timer.duration}, '
          'periodic: ${timer.isPeriodic}), created:',
        );
        debugPrintStack(stackTrace: timer.creationStackTrace);
        debugPrint('');
      }
      timersPending = true;
    }
    assert(!timersPending, 'A Timer is still pending even after the widget tree was disposed.');
    assert(_currentFakeAsync!.microtaskCount == 0); // Shouldn't be possible.
  }

  @override
  void postTest() {
    super.postTest();
    assert(_currentFakeAsync != null);
    assert(_clock != null);
    _clock = null;
    _currentFakeAsync = null;
  }
}

/// Available policies for how a [LiveTestWidgetsFlutterBinding] should paint
/// frames.
///
/// These values are set on the binding's
/// [LiveTestWidgetsFlutterBinding.framePolicy] property.
///
/// {@template flutter.flutter_test.LiveTestWidgetsFlutterBindingFramePolicy}
/// The default is [LiveTestWidgetsFlutterBindingFramePolicy.fadePointers].
/// Setting this to anything other than
/// [LiveTestWidgetsFlutterBindingFramePolicy.onlyPumps] results in pumping
/// extra frames, which might involve calling builders more, or calling paint
/// callbacks more, etc, and might interfere with the test. If you know that
/// your test won't be affected by this, you can set the policy to
/// [LiveTestWidgetsFlutterBindingFramePolicy.fullyLive] or
/// [LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive] in that particular
/// file.
///
/// To set a value while still allowing the test file to work as a normal test,
/// add the following code to your test file at the top of your
/// `void main() { }` function, before calls to [testWidgets]:
///
/// ```dart
/// TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();
/// if (binding is LiveTestWidgetsFlutterBinding) {
///   binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.onlyPumps;
/// }
/// ```
/// {@endtemplate}
enum LiveTestWidgetsFlutterBindingFramePolicy {
  /// Strictly show only frames that are explicitly pumped.
  ///
  /// This most closely matches the [AutomatedTestWidgetsFlutterBinding]
  /// (the default binding for `flutter test`) behavior.
  onlyPumps,

  /// Show pumped frames, and additionally schedule and run frames to fade
  /// out the pointer crosshairs and other debugging information shown by
  /// the binding.
  ///
  /// This will schedule frames when pumped or when there has been some
  /// activity with [TestPointer]s.
  ///
  /// This can result in additional frames being pumped beyond those that
  /// the test itself requests, which can cause differences in behavior.
  fadePointers,

  /// Show every frame that the framework requests, even if the frames are not
  /// explicitly pumped.
  ///
  /// The major difference between [fullyLive] and [benchmarkLive] is the latter
  /// ignores frame requests by [WidgetTester.pump].
  ///
  /// This can help with orienting the developer when looking at
  /// heavily-animated situations, and will almost certainly result in
  /// additional frames being pumped beyond those that the test itself requests,
  /// which can cause differences in behavior.
  fullyLive,

  /// Ignore any request to schedule a frame.
  ///
  /// This is intended to be used by benchmarks (hence the name) that drive the
  /// pipeline directly. It tells the binding to entirely ignore requests for a
  /// frame to be scheduled, while still allowing frames that are pumped
  /// directly to run (either by using [WidgetTester.pumpBenchmark] or invoking
  /// [PlatformDispatcher.onBeginFrame] and [PlatformDispatcher.onDrawFrame]).
  ///
  /// This allows all frame requests from the engine to be serviced, and allows
  /// all frame requests that are artificially triggered to be serviced, but
  /// ignores [SchedulerBinding.scheduleFrame] requests from the framework.
  /// Therefore animation won't run for this mode because the framework
  /// generates an animation by requesting new frames.
  ///
  /// The [SchedulerBinding.hasScheduledFrame] property will never be true in
  /// this mode. This can cause unexpected effects. For instance,
  /// [WidgetTester.pumpAndSettle] does not function in this mode, as it relies
  /// on the [SchedulerBinding.hasScheduledFrame] property to determine when the
  /// application has "settled".
  benchmark,

  /// Ignore any request from pump but respect other requests to schedule a
  /// frame.
  ///
  /// This is used for running the test on a device, where scheduling of new
  /// frames respects what the engine and the device needed.
  ///
  /// Compared to [fullyLive] this policy ignores the frame requests from
  /// [WidgetTester.pump] so that frame scheduling mimics that of the real
  /// environment, and avoids waiting for an artificially pumped frame. (For
  /// example, when driving the test in methods like
  /// [WidgetTester.handlePointerEventRecord] or [WidgetTester.fling].)
  ///
  /// This policy differs from [benchmark] in that it can be used for capturing
  /// animation frames requested by the framework.
  benchmarkLive,
}

enum _HandleDrawFrame { reset, drawFrame, skipFrame }

/// A variant of [TestWidgetsFlutterBinding] for executing tests
/// on a device, typically via `flutter run`, or via integration tests.
/// This is intended to allow interactive test development.
///
/// This is not the way to run a remote-control test. To run a test on
/// a device from a development computer, see the [flutter_driver]
/// package and the `flutter drive` command.
///
/// When running tests using `flutter run`, consider adding the
/// `--use-test-fonts` argument so that the fonts used match those used under
/// `flutter test`. (This forces all text to use the "Ahem" font, which is a
/// font that covers ASCII characters and gives them all the appearance of a
/// square whose size equals the font size.)
///
/// This binding overrides the default [SchedulerBinding] behavior to ensure
/// that tests work in the same way in this environment as they would under the
/// [AutomatedTestWidgetsFlutterBinding]. To override this (and see intermediate
/// frames that the test does not explicitly trigger), set [framePolicy] to
/// [LiveTestWidgetsFlutterBindingFramePolicy.fullyLive]. (This is likely to
/// make tests fail, though, especially if e.g. they test how many times a
/// particular widget was built.) The default behavior is to show pumped frames
/// and a few additional frames when pointers are triggered (to animate the
/// pointer crosshairs).
///
/// This binding does not support the [EnginePhase] argument to
/// [pump]. (There would be no point setting it to a value that
/// doesn't trigger a paint, since then you could not see anything
/// anyway.)
///
/// See [TestWidgetsFlutterBinding] for a list of mixins that must be
/// provided by the binding active while the test framework is
/// running.
class LiveTestWidgetsFlutterBinding extends TestWidgetsFlutterBinding {
  @override
  void initInstances() {
    super.initInstances();
    _instance = this;

    RenderView.debugAddPaintCallback(_handleRenderViewPaint);
  }

  /// The current [LiveTestWidgetsFlutterBinding], if one has been created.
  ///
  /// The binding must be initialized before using this getter. If you
  /// need the binding to be constructed before calling [testWidgets],
  /// you can ensure a binding has been constructed by calling the
  /// [TestWidgetsFlutterBinding.ensureInitialized] function.
  static LiveTestWidgetsFlutterBinding get instance => BindingBase.checkInstance(_instance);
  static LiveTestWidgetsFlutterBinding? _instance;

  /// Returns an instance of the binding that implements
  /// [LiveTestWidgetsFlutterBinding]. If no binding has yet been
  /// initialized, the a new instance is created.
  ///
  /// Generally, there is no need to call this method. Use
  /// [TestWidgetsFlutterBinding.ensureInitialized] instead, as it
  /// will select the correct test binding implementation
  /// automatically.
  static LiveTestWidgetsFlutterBinding ensureInitialized() {
    if (LiveTestWidgetsFlutterBinding._instance == null) {
      LiveTestWidgetsFlutterBinding();
    }
    return LiveTestWidgetsFlutterBinding.instance;
  }

  @override
  bool get inTest => _inTest;
  bool _inTest = false;

  @override
  Clock get clock => const Clock();

  @override
  int get microtaskCount {
    // The Dart SDK doesn't report this number.
    assert(false, 'microtaskCount cannot be reported when running in real time');
    return -1;
  }

  @override
  test_package.Timeout get defaultTestTimeout => test_package.Timeout.none;

  Completer<void>? _pendingFrame;
  bool _expectingFrame = false;
  bool _expectingFrameToReassemble = false;
  bool _viewNeedsPaint = false;
  bool _runningAsyncTasks = false;

  /// The strategy for [pump]ing and requesting new frames.
  ///
  /// The policy decides whether [pump] (with a duration) pumps a single frame
  /// (as would happen in a normal test environment using
  /// [AutomatedTestWidgetsFlutterBinding]), or pumps every frame that the
  /// system requests during an asynchronous pause (as would normally happen
  /// when running an application with [WidgetsFlutterBinding]).
  ///
  /// {@macro flutter.flutter_test.LiveTestWidgetsFlutterBindingFramePolicy}
  ///
  /// See [LiveTestWidgetsFlutterBindingFramePolicy].
  LiveTestWidgetsFlutterBindingFramePolicy framePolicy =
      LiveTestWidgetsFlutterBindingFramePolicy.fadePointers;

  @override
  Future<void> delayed(Duration duration) {
    return Future<void>.delayed(duration);
  }

  @override
  void scheduleFrame() {
    if (framePolicy == LiveTestWidgetsFlutterBindingFramePolicy.benchmark) {
      // In benchmark mode, don't actually schedule any engine frames.
      return;
    }
    super.scheduleFrame();
  }

  @override
  void scheduleForcedFrame() {
    if (framePolicy == LiveTestWidgetsFlutterBindingFramePolicy.benchmark) {
      // In benchmark mode, don't actually schedule any engine frames.
      return;
    }
    super.scheduleForcedFrame();
  }

  @override
  Future<void> reassembleApplication() {
    _expectingFrameToReassemble = true;
    return super.reassembleApplication();
  }

  _HandleDrawFrame _drawFrame = _HandleDrawFrame.reset;

  @override
  void handleBeginFrame(Duration? rawTimeStamp) {
    if (_drawFrame != _HandleDrawFrame.reset) {
      throw StateError('handleBeginFrame() called before previous handleDrawFrame()');
    }
    if (_expectingFrame ||
        _expectingFrameToReassemble ||
        (framePolicy == LiveTestWidgetsFlutterBindingFramePolicy.fullyLive) ||
        (framePolicy == LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive) ||
        (framePolicy == LiveTestWidgetsFlutterBindingFramePolicy.benchmark) ||
        (framePolicy == LiveTestWidgetsFlutterBindingFramePolicy.fadePointers && _viewNeedsPaint)) {
      _drawFrame = _HandleDrawFrame.drawFrame;
      super.handleBeginFrame(rawTimeStamp);
    } else {
      _drawFrame = _HandleDrawFrame.skipFrame;
    }
  }

  @override
  void handleDrawFrame() {
    if (_drawFrame == _HandleDrawFrame.reset) {
      throw StateError('handleDrawFrame() called without paired handleBeginFrame()');
    }
    if (_drawFrame == _HandleDrawFrame.drawFrame) {
      super.handleDrawFrame();
    }
    _drawFrame = _HandleDrawFrame.reset;
    _viewNeedsPaint = false;
    _expectingFrameToReassemble = false;
    if (_expectingFrame) {
      // set during pump
      assert(_pendingFrame != null);
      _pendingFrame!.complete(); // unlocks the test API
      _pendingFrame = null;
      _expectingFrame = false;
    } else if (framePolicy != LiveTestWidgetsFlutterBindingFramePolicy.benchmark) {
      platformDispatcher.scheduleFrame();
    }
  }

  void _markViewsNeedPaint([int? viewId]) {
    _viewNeedsPaint = true;
    final Iterable<RenderView> toMark =
        viewId == null
            ? renderViews
            : renderViews.where((RenderView renderView) => renderView.flutterView.viewId == viewId);
    for (final RenderView renderView in toMark) {
      renderView.markNeedsPaint();
    }
  }

  TextPainter? _label;
  static const TextStyle _labelStyle = TextStyle(fontFamily: 'sans-serif', fontSize: 10.0);

  /// Label describing the test.
  @visibleForTesting
  TextPainter? get label => _label;

  /// Set a description label that is drawn into the test output.
  @protected
  void setLabel(String value) {
    if (value.isEmpty) {
      _label = null;
      return;
    }
    // TODO(ianh): Figure out if the test name is actually RTL.
    _label ??= TextPainter(textAlign: TextAlign.left, textDirection: TextDirection.ltr);
    _label!.text = TextSpan(text: value, style: _labelStyle);
    _label!.layout();
    _markViewsNeedPaint();
  }

  final Expando<Map<int, _LiveTestPointerRecord>> _renderViewToPointerIdToPointerRecord =
      Expando<Map<int, _LiveTestPointerRecord>>();

  void _handleRenderViewPaint(PaintingContext context, Offset offset, RenderView renderView) {
    assert(offset == Offset.zero);

    final Map<int, _LiveTestPointerRecord>? pointerIdToRecord =
        _renderViewToPointerIdToPointerRecord[renderView];
    if (pointerIdToRecord != null && pointerIdToRecord.isNotEmpty) {
      final double radius = renderView.size.shortestSide * 0.05;
      final Path path =
          Path()
            ..addOval(Rect.fromCircle(center: Offset.zero, radius: radius))
            ..moveTo(0.0, -radius * 2.0)
            ..lineTo(0.0, radius * 2.0)
            ..moveTo(-radius * 2.0, 0.0)
            ..lineTo(radius * 2.0, 0.0);
      final Canvas canvas = context.canvas;
      final Paint paint =
          Paint()
            ..strokeWidth = radius / 10.0
            ..style = PaintingStyle.stroke;
      bool dirty = false;
      for (final _LiveTestPointerRecord record in pointerIdToRecord.values) {
        paint.color = record.color.withOpacity(
          record.decay < 0 ? (record.decay / (_kPointerDecay - 1)) : 1.0,
        );
        canvas.drawPath(path.shift(record.position), paint);
        if (record.decay < 0) {
          dirty = true;
        }
        record.decay += 1;
      }
      pointerIdToRecord.keys
          .where((int pointer) => pointerIdToRecord[pointer]!.decay == 0)
          .toList()
          .forEach(pointerIdToRecord.remove);
      if (dirty) {
        scheduleMicrotask(() {
          _markViewsNeedPaint(renderView.flutterView.viewId);
        });
      }
    }

    _label?.paint(context.canvas, offset - const Offset(0.0, 10.0));
  }

  /// An object to which real device events should be routed.
  ///
  /// Normally, device events are silently dropped. However, if this property is
  /// set to a non-null value, then the events will be routed to its
  /// [HitTestDispatcher.dispatchEvent] method instead, unless
  /// [shouldPropagateDevicePointerEvents] is true.
  ///
  /// Events dispatched by [TestGesture] are not affected by this.
  HitTestDispatcher? deviceEventDispatcher;

  /// Dispatch an event to the targets found by a hit test on its position.
  ///
  /// If the [pointerEventSource] is [TestBindingEventSource.test], then
  /// the event is forwarded to [GestureBinding.dispatchEvent] as usual;
  /// additionally, down pointers are painted on the screen.
  ///
  /// If the [pointerEventSource] is [TestBindingEventSource.device], then
  /// the event, after being transformed to the local coordinate system, is
  /// forwarded to [deviceEventDispatcher].
  @override
  void handlePointerEvent(PointerEvent event) {
    switch (pointerEventSource) {
      case TestBindingEventSource.test:
        RenderView? target;
        for (final RenderView renderView in renderViews) {
          if (renderView.flutterView.viewId == event.viewId) {
            target = renderView;
            break;
          }
        }
        if (target != null) {
          final _LiveTestPointerRecord? record =
              _renderViewToPointerIdToPointerRecord[target]?[event.pointer];
          if (record != null) {
            record.position = event.position;
            if (!event.down) {
              record.decay = _kPointerDecay;
            }
            _markViewsNeedPaint(event.viewId);
          } else if (event.down) {
            _renderViewToPointerIdToPointerRecord[target] ??= <int, _LiveTestPointerRecord>{};
            _renderViewToPointerIdToPointerRecord[target]![event.pointer] = _LiveTestPointerRecord(
              event.pointer,
              event.position,
            );
            _markViewsNeedPaint(event.viewId);
          }
        }
        super.handlePointerEvent(event);
      case TestBindingEventSource.device:
        if (shouldPropagateDevicePointerEvents) {
          super.handlePointerEvent(event);
          break;
        }
        if (deviceEventDispatcher != null) {
          // The pointer events received with this source has a global position
          // (see [handlePointerEventForSource]). Transform it to the local
          // coordinate space used by the testing widgets.
          final RenderView renderView = renderViews.firstWhere(
            (RenderView r) => r.flutterView.viewId == event.viewId,
          );
          final PointerEvent localEvent = event.copyWith(
            position: globalToLocal(event.position, renderView),
          );
          withPointerEventSource(
            TestBindingEventSource.device,
            () => super.handlePointerEvent(localEvent),
          );
        }
    }
  }

  @override
  void dispatchEvent(PointerEvent event, HitTestResult? hitTestResult) {
    switch (pointerEventSource) {
      case TestBindingEventSource.test:
        super.dispatchEvent(event, hitTestResult);
      case TestBindingEventSource.device:
        assert(hitTestResult != null || event is PointerAddedEvent || event is PointerRemovedEvent);
        if (shouldPropagateDevicePointerEvents) {
          super.dispatchEvent(event, hitTestResult);
          break;
        }
        assert(deviceEventDispatcher != null);
        if (hitTestResult != null) {
          deviceEventDispatcher!.dispatchEvent(event, hitTestResult);
        }
    }
  }

  @override
  Future<void> pump([Duration? duration, EnginePhase newPhase = EnginePhase.sendSemanticsUpdate]) {
    assert(newPhase == EnginePhase.sendSemanticsUpdate);
    assert(inTest);
    assert(!_expectingFrame);
    assert(_pendingFrame == null);
    if (framePolicy == LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive) {
      // Ignore all pumps and just wait.
      return delayed(duration ?? Duration.zero);
    }
    return TestAsyncUtils.guard<void>(() {
      if (duration != null) {
        Timer(duration, () {
          _expectingFrame = true;
          scheduleFrame();
        });
      } else {
        _expectingFrame = true;
        scheduleFrame();
      }
      _pendingFrame = Completer<void>();
      return _pendingFrame!.future;
    });
  }

  @override
  Future<T?> runAsync<T>(Future<T> Function() callback) async {
    assert(() {
      if (!_runningAsyncTasks) {
        return true;
      }
      fail(
        'Reentrant call to runAsync() denied.\n'
        'runAsync() was called, then before its future completed, it '
        'was called again. You must wait for the first returned future '
        'to complete before calling runAsync() again.',
      );
    }());

    _runningAsyncTasks = true;
    try {
      return await callback();
    } catch (error, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: error,
          stack: stack,
          library: 'Flutter test framework',
          context: ErrorSummary('while running async test code'),
        ),
      );
      return null;
    } finally {
      _runningAsyncTasks = false;
    }
  }

  @override
  Future<void> runTest(
    Future<void> Function() testBody,
    VoidCallback invariantTester, {
    String description = '',
  }) {
    assert(!inTest);
    _inTest = true;
    setLabel(description);
    return _runTest(testBody, invariantTester, description);
  }

  @override
  void reportExceptionNoticed(FlutterErrorDetails exception) {
    final DebugPrintCallback testPrint = debugPrint;
    debugPrint = debugPrintOverride;
    debugPrint('(The following exception is now available via WidgetTester.takeException:)');
    FlutterError.dumpErrorToConsole(exception, forceReport: true);
    debugPrint(
      '(If WidgetTester.takeException is called, the above exception will be ignored. '
      'If it is not, then the above exception will be dumped when another exception is '
      'caught by the framework or when the test ends, whichever happens first, and then '
      'the test will fail due to having not caught or expected the exception.)',
    );
    debugPrint = testPrint;
  }

  @override
  void postTest() {
    super.postTest();
    assert(!_expectingFrame);
    assert(_pendingFrame == null);
    _inTest = false;
  }

  @override
  ViewConfiguration createViewConfigurationFor(RenderView renderView) {
    final FlutterView view = renderView.flutterView;
    if (view == platformDispatcher.implicitView) {
      return TestViewConfiguration.fromView(
        size: _surfaceSize ?? _kDefaultTestViewportSize,
        view: view,
      );
    }
    final double devicePixelRatio = view.devicePixelRatio;
    return TestViewConfiguration.fromView(size: view.physicalSize / devicePixelRatio, view: view);
  }

  @override
  Offset globalToLocal(Offset point, RenderView view) {
    // The method is expected to translate the given point expressed in logical
    // pixels in the global coordinate space to the local coordinate space (also
    // expressed in logical pixels).
    // The inverted transform translates from the global coordinate space in
    // physical pixels to the local coordinate space in logical pixels.
    final Matrix4 transform = view.configuration.toMatrix();
    final double det = transform.invert();
    assert(det != 0.0);
    // In order to use the transform, we need to translate the point first into
    // the physical coordinate space by applying the device pixel ratio.
    return MatrixUtils.transformPoint(transform, point * view.configuration.devicePixelRatio);
  }

  @override
  Offset localToGlobal(Offset point, RenderView view) {
    // The method is expected to translate the given point expressed in logical
    // pixels in the local coordinate space to the global coordinate space (also
    // expressed in logical pixels).
    // The transform translates from the local coordinate space in logical
    // pixels to the global coordinate space in physical pixels.
    final Matrix4 transform = view.configuration.toMatrix();
    final Offset pointInPhysicalPixels = MatrixUtils.transformPoint(transform, point);
    // We need to apply the device pixel ratio to get back to logical pixels.
    return pointInPhysicalPixels / view.configuration.devicePixelRatio;
  }
}

/// A [ViewConfiguration] that pretends the display is of a particular size (in
/// logical pixels).
///
/// The resulting ViewConfiguration maps the given size onto the actual display
/// using the [BoxFit.contain] algorithm.
///
/// If the underlying [FlutterView] changes, a new [TestViewConfiguration] should
/// be created. See [RendererBinding.handleMetricsChanged] and
/// [RendererBinding.createViewConfigurationFor].
class TestViewConfiguration implements ViewConfiguration {
  /// Deprecated. Will be removed in a future version of Flutter.
  ///
  /// This property has been deprecated to prepare for Flutter's upcoming
  /// support for multiple views and multiple windows.
  ///
  /// Use [TestViewConfiguration.fromView] instead.
  @Deprecated(
    'Use TestViewConfiguration.fromView instead. '
    'Deprecated to prepare for the upcoming multi-window support. '
    'This feature was deprecated after v3.7.0-32.0.pre.',
  )
  factory TestViewConfiguration({Size size = _kDefaultTestViewportSize, ui.FlutterView? window}) {
    return TestViewConfiguration.fromView(size: size, view: window ?? ui.window);
  }

  /// Creates a [TestViewConfiguration] with the given size and view.
  ///
  /// The [size] defaults to 800x600.
  ///
  /// The settings of the given [FlutterView] are captured when the constructor
  /// is called, and subsequent changes are ignored. A new
  /// [TestViewConfiguration] should be created if the underlying [FlutterView]
  /// changes. See [RendererBinding.handleMetricsChanged] and
  /// [RendererBinding.createViewConfigurationFor].
  TestViewConfiguration.fromView({
    required ui.FlutterView view,
    Size size = _kDefaultTestViewportSize,
  }) : devicePixelRatio = view.devicePixelRatio,
       logicalConstraints = BoxConstraints.tight(size),
       physicalConstraints = BoxConstraints.tight(size) * view.devicePixelRatio,
       _paintMatrix = _getMatrix(size, view.devicePixelRatio, view),
       _physicalSize = view.physicalSize;

  @override
  final double devicePixelRatio;

  @override
  final BoxConstraints logicalConstraints;

  @override
  final BoxConstraints physicalConstraints;

  static Matrix4 _getMatrix(Size size, double devicePixelRatio, ui.FlutterView window) {
    final double inverseRatio = devicePixelRatio / window.devicePixelRatio;
    final double actualWidth = window.physicalSize.width * inverseRatio;
    final double actualHeight = window.physicalSize.height * inverseRatio;
    final double desiredWidth = size.width;
    final double desiredHeight = size.height;
    double scale, shiftX, shiftY;
    if (desiredWidth == 0.0 || desiredHeight == 0.0) {
      scale = 1.0;
      shiftX = 0.0;
      shiftY = 0.0;
    } else if ((actualWidth / actualHeight) > (desiredWidth / desiredHeight)) {
      scale = actualHeight / desiredHeight;
      shiftX = (actualWidth - desiredWidth * scale) / 2.0;
      shiftY = 0.0;
    } else {
      scale = actualWidth / desiredWidth;
      shiftX = 0.0;
      shiftY = (actualHeight - desiredHeight * scale) / 2.0;
    }
    final Matrix4 matrix = Matrix4.compose(
      Vector3(shiftX, shiftY, 0.0), // translation
      Quaternion.identity(), // rotation
      Vector3(scale, scale, 1.0), // scale
    );
    return matrix;
  }

  final Matrix4 _paintMatrix;

  @override
  Matrix4 toMatrix() => _paintMatrix.clone();

  @override
  bool shouldUpdateMatrix(ViewConfiguration oldConfiguration) {
    if (oldConfiguration.runtimeType != runtimeType) {
      // New configuration could have different logic, so we don't know
      // whether it will need a new transform. Return a conservative result.
      return true;
    }
    oldConfiguration as TestViewConfiguration;
    // Compare the matrices directly since they are cached.
    return oldConfiguration._paintMatrix != _paintMatrix;
  }

  final Size _physicalSize;

  @override
  Size toPhysicalSize(Size logicalSize) => _physicalSize;

  @override
  String toString() => 'TestViewConfiguration';
}

class _TestSamplingClock implements SamplingClock {
  _TestSamplingClock(this._clock);

  @override
  DateTime now() => _clock.now();

  @override
  Stopwatch stopwatch() => _clock.stopwatch();

  final Clock _clock;
}

const int _kPointerDecay = -2;

class _LiveTestPointerRecord {
  _LiveTestPointerRecord(this.pointer, this.position)
    : color = HSVColor.fromAHSV(0.8, (35.0 * pointer) % 360.0, 1.0, 1.0).toColor(),
      decay = 1;
  final int pointer;
  final Color color;
  Offset position;
  int decay; // >0 means down, <0 means up, increases by one each time, removed at 0
}
