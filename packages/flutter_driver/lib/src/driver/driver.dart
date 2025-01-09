// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/services.dart';
/// @docImport 'package:flutter/widgets.dart';
/// @docImport 'package:flutter_driver/driver_extension.dart';

/// @docImport 'package:flutter_test/flutter_test.dart';
library;

import 'dart:convert';
import 'dart:io';

import 'package:meta/meta.dart';
import 'package:vm_service/vm_service.dart' as vms;
import 'package:webdriver/async_io.dart' as async_io;

import '../common/diagnostics_tree.dart';
import '../common/error.dart';
import '../common/find.dart';
import '../common/frame_sync.dart';
import '../common/geometry.dart';
import '../common/gesture.dart';
import '../common/health.dart';
import '../common/layer_tree.dart';
import '../common/message.dart';
import '../common/render_tree.dart';
import '../common/request_data.dart';
import '../common/screenshot.dart';
import '../common/semantics.dart';
import '../common/text.dart';
import '../common/text_input_action.dart';
import '../common/wait.dart';
import 'timeline.dart';
import 'vmservice_driver.dart';
import 'web_driver.dart';

export '../common/screenshot.dart' show ScreenshotFormat;
export 'vmservice_driver.dart';
export 'web_driver.dart';

/// Timeline stream identifier.
enum TimelineStream {
  /// A meta-identifier that instructs the Dart VM to record all streams.
  all,

  /// Marks events related to calls made via Dart's C API.
  api,

  /// Marks events from the Dart VM's JIT compiler.
  compiler,

  /// The verbose version of compiler.
  compilerVerbose,

  /// Marks events emitted using the `dart:developer` API.
  dart,

  /// Marks events from the Dart VM debugger.
  debugger,

  /// Marks events emitted using the `dart_tools_api.h` C API.
  embedder,

  /// Marks events from the garbage collector.
  gc,

  /// Marks events related to message passing between Dart isolates.
  isolate,

  /// Marks internal VM events.
  vm,
}

/// How long to wait before showing a message saying that
/// things seem to be taking a long time.
@internal
const Duration kUnusuallyLongTimeout = Duration(seconds: 5);

/// A convenient accessor to frequently used finders.
///
/// Examples:
///
///     driver.tap(find.text('Save'));
///     driver.scroll(find.byValueKey(42));
const CommonFinders find = CommonFinders._();

/// Computes a value.
///
/// If computation is asynchronous, the function may return a [Future].
///
/// See also [FlutterDriver.waitFor].
typedef EvaluatorFunction = dynamic Function();

// Examples can assume:
// import 'package:flutter_driver/flutter_driver.dart';
// import 'package:test/test.dart';
// late FlutterDriver driver;

/// Drives a Flutter Application running in another process.
abstract class FlutterDriver {
  /// Default constructor.
  @visibleForTesting
  FlutterDriver();

  /// Creates a driver that uses a connection provided by either the combination
  /// of [webConnection], or the combination of [serviceClient] and [appIsolate]
  /// for the VM.
  @visibleForTesting
  factory FlutterDriver.connectedTo({
    FlutterWebConnection? webConnection,
    vms.VmService? serviceClient,
    vms.Isolate? appIsolate,
  }) {
    if (webConnection != null) {
      return WebFlutterDriver.connectedTo(webConnection);
    }
    return VMServiceFlutterDriver.connectedTo(serviceClient!, appIsolate!);
  }

  /// Connects to a Flutter application.
  ///
  /// Resumes the application if it is currently paused (e.g. at a breakpoint).
  ///
  /// The `dartVmServiceUrl` parameter is the URL to Dart observatory
  /// (a.k.a. VM service). If not specified, the URL specified by the
  /// `VM_SERVICE_URL` environment variable is used. One or the other must be
  /// specified.
  ///
  /// The `printCommunication` parameter determines whether the command
  /// communication between the test and the app should be printed to stdout.
  ///
  /// The `logCommunicationToFile` parameter determines whether the command
  /// communication between the test and the app should be logged to
  /// `flutter_driver_commands.log`.
  ///
  /// The `isolateNumber` parameter determines the specific isolate to connect
  /// to. If this is left as `null`, will connect to the first isolate found
  /// running on `dartVmServiceUrl`.
  ///
  /// The `fuchsiaModuleTarget` parameter specifies the pattern for determining
  /// which mod to control. When running on a Fuchsia device, either this or the
  /// environment variable `FUCHSIA_MODULE_TARGET` must be set (the environment
  /// variable is treated as a substring pattern). This field will be ignored if
  /// `isolateNumber` is set, as this is already enough information to connect
  /// to an isolate. This parameter is ignored on non-fuchsia devices.
  ///
  /// The `headers` parameter optionally specifies HTTP headers to be included
  /// in the [WebSocket] connection. This is only used for
  /// [VMServiceFlutterDriver] connections.
  ///
  /// The return value is a future. This method never times out, though it may
  /// fail (completing with an error). A timeout can be applied by the caller
  /// using [Future.timeout] if necessary.
  static Future<FlutterDriver> connect({
    String? dartVmServiceUrl,
    bool printCommunication = false,
    bool logCommunicationToFile = true,
    int? isolateNumber,
    Pattern? fuchsiaModuleTarget,
    Duration? timeout,
    Map<String, dynamic>? headers,
  }) async {
    if (Platform.environment['FLUTTER_WEB_TEST'] != null) {
      return WebFlutterDriver.connectWeb(
        hostUrl: dartVmServiceUrl,
        timeout: timeout,
        printCommunication: printCommunication,
        logCommunicationToFile: logCommunicationToFile,
      );
    }
    return VMServiceFlutterDriver.connect(
      dartVmServiceUrl: dartVmServiceUrl,
      printCommunication: printCommunication,
      logCommunicationToFile: logCommunicationToFile,
      isolateNumber: isolateNumber,
      fuchsiaModuleTarget: fuchsiaModuleTarget,
      headers: headers,
    );
  }

  /// Getter of appIsolate.
  vms.Isolate get appIsolate => throw UnimplementedError();

  /// Getter of serviceClient.
  vms.VmService get serviceClient => throw UnimplementedError();

  /// Getter of webDriver.
  async_io.WebDriver get webDriver => throw UnimplementedError();

  /// Sends [command] to the Flutter Driver extensions.
  /// This must be implemented by subclass.
  ///
  /// See also:
  ///
  ///  * [VMServiceFlutterDriver], which uses vmservice to implement.
  ///  * [WebFlutterDriver], which uses webdriver to implement.
  Future<Map<String, dynamic>> sendCommand(Command command) async => throw UnimplementedError();

  /// Checks the status of the Flutter Driver extension.
  Future<Health> checkHealth({Duration? timeout}) async {
    return Health.fromJson(await sendCommand(GetHealth(timeout: timeout)));
  }

  /// Returns a dump of the render tree.
  Future<RenderTree> getRenderTree({Duration? timeout}) async {
    return RenderTree.fromJson(await sendCommand(GetRenderTree(timeout: timeout)));
  }

  /// Returns a dump of the layer tree.
  Future<LayerTree> getLayerTree({Duration? timeout}) async {
    return LayerTree.fromJson(await sendCommand(GetLayerTree(timeout: timeout)));
  }

  /// Taps at the center of the widget located by [finder].
  Future<void> tap(SerializableFinder finder, {Duration? timeout}) async {
    await sendCommand(Tap(finder, timeout: timeout));
  }

  /// Waits until [finder] locates the target.
  ///
  /// The [finder] will wait until there is no pending frame scheduled
  /// in the app under test before executing an action.
  ///
  /// See also:
  ///
  ///  * [FlutterDriver.runUnsynchronized], which will execute an action
  ///    with frame sync disabled even while frames are pending.
  Future<void> waitFor(SerializableFinder finder, {Duration? timeout}) async {
    await sendCommand(WaitFor(finder, timeout: timeout));
  }

  /// Waits until [finder] can no longer locate the target.
  Future<void> waitForAbsent(SerializableFinder finder, {Duration? timeout}) async {
    await sendCommand(WaitForAbsent(finder, timeout: timeout));
  }

  /// Waits until [finder] is tappable.
  Future<void> waitForTappable(SerializableFinder finder, {Duration? timeout}) async {
    await sendCommand(WaitForTappable(finder, timeout: timeout));
  }

  /// Waits until the given [waitCondition] is satisfied.
  Future<void> waitForCondition(
    SerializableWaitCondition waitCondition, {
    Duration? timeout,
  }) async {
    await sendCommand(WaitForCondition(waitCondition, timeout: timeout));
  }

  /// Waits until there are no more transient callbacks in the queue.
  ///
  /// Use this method when you need to wait for the moment when the application
  /// becomes "stable", for example, prior to taking a [screenshot].
  Future<void> waitUntilNoTransientCallbacks({Duration? timeout}) async {
    await sendCommand(WaitForCondition(const NoTransientCallbacks(), timeout: timeout));
  }

  /// Waits until the next [dart:ui.PlatformDispatcher.onReportTimings] is
  /// called.
  ///
  /// Use this method to wait for the first frame to be rasterized during the
  /// app launch.
  ///
  /// Throws [UnimplementedError] on [WebFlutterDriver] instances.
  Future<void> waitUntilFirstFrameRasterized() async {
    await sendCommand(const WaitForCondition(FirstFrameRasterized()));
  }

  Future<DriverOffset> _getOffset(
    SerializableFinder finder,
    OffsetType type, {
    Duration? timeout,
  }) async {
    final GetOffset command = GetOffset(finder, type, timeout: timeout);
    final GetOffsetResult result = GetOffsetResult.fromJson(await sendCommand(command));
    return DriverOffset(result.dx, result.dy);
  }

  /// Returns the point at the top left of the widget identified by `finder`.
  ///
  /// The offset is expressed in logical pixels and can be translated to
  /// device pixels via [dart:ui.FlutterView.devicePixelRatio].
  Future<DriverOffset> getTopLeft(SerializableFinder finder, {Duration? timeout}) async {
    return _getOffset(finder, OffsetType.topLeft, timeout: timeout);
  }

  /// Returns the point at the top right of the widget identified by `finder`.
  ///
  /// The offset is expressed in logical pixels and can be translated to
  /// device pixels via [dart:ui.FlutterView.devicePixelRatio].
  Future<DriverOffset> getTopRight(SerializableFinder finder, {Duration? timeout}) async {
    return _getOffset(finder, OffsetType.topRight, timeout: timeout);
  }

  /// Returns the point at the bottom left of the widget identified by `finder`.
  ///
  /// The offset is expressed in logical pixels and can be translated to
  /// device pixels via [dart:ui.FlutterView.devicePixelRatio].
  Future<DriverOffset> getBottomLeft(SerializableFinder finder, {Duration? timeout}) async {
    return _getOffset(finder, OffsetType.bottomLeft, timeout: timeout);
  }

  /// Returns the point at the bottom right of the widget identified by `finder`.
  ///
  /// The offset is expressed in logical pixels and can be translated to
  /// device pixels via [dart:ui.FlutterView.devicePixelRatio].
  Future<DriverOffset> getBottomRight(SerializableFinder finder, {Duration? timeout}) async {
    return _getOffset(finder, OffsetType.bottomRight, timeout: timeout);
  }

  /// Returns the point at the center of the widget identified by `finder`.
  ///
  /// The offset is expressed in logical pixels and can be translated to
  /// device pixels via [dart:ui.FlutterView.devicePixelRatio].
  Future<DriverOffset> getCenter(SerializableFinder finder, {Duration? timeout}) async {
    return _getOffset(finder, OffsetType.center, timeout: timeout);
  }

  /// Returns a JSON map of the [DiagnosticsNode] that is associated with the
  /// [RenderObject] identified by `finder`.
  ///
  /// The `subtreeDepth` argument controls how many layers of children will be
  /// included in the result. It defaults to zero, which means that no children
  /// of the [RenderObject] identified by `finder` will be part of the result.
  ///
  /// The `includeProperties` argument controls whether properties of the
  /// [DiagnosticsNode]s will be included in the result. It defaults to true.
  ///
  /// [RenderObject]s are responsible for positioning, layout, and painting on
  /// the screen, based on the configuration from a [Widget]. Callers that need
  /// information about size or position should use this method.
  ///
  /// A widget may indirectly create multiple [RenderObject]s, which each
  /// implement some aspect of the widget configuration. A 1:1 relationship
  /// should not be assumed.
  ///
  /// See also:
  ///
  ///  * [getWidgetDiagnostics], which gets the [DiagnosticsNode] of a [Widget].
  Future<Map<String, Object?>> getRenderObjectDiagnostics(
    SerializableFinder finder, {
    int subtreeDepth = 0,
    bool includeProperties = true,
    Duration? timeout,
  }) async {
    return sendCommand(
      GetDiagnosticsTree(
        finder,
        DiagnosticsType.renderObject,
        subtreeDepth: subtreeDepth,
        includeProperties: includeProperties,
        timeout: timeout,
      ),
    );
  }

  /// Returns a JSON map of the [DiagnosticsNode] that is associated with the
  /// [Widget] identified by `finder`.
  ///
  /// The `subtreeDepth` argument controls how many layers of children will be
  /// included in the result. It defaults to zero, which means that no children
  /// of the [Widget] identified by `finder` will be part of the result.
  ///
  /// The `includeProperties` argument controls whether properties of the
  /// [DiagnosticsNode]s will be included in the result. It defaults to true.
  ///
  /// [Widget]s describe configuration for the rendering tree. Individual
  /// widgets may create multiple [RenderObject]s to actually layout and paint
  /// the desired configuration.
  ///
  /// See also:
  ///
  ///  * [getRenderObjectDiagnostics], which gets the [DiagnosticsNode] of a
  ///    [RenderObject].
  Future<Map<String, Object?>> getWidgetDiagnostics(
    SerializableFinder finder, {
    int subtreeDepth = 0,
    bool includeProperties = true,
    Duration? timeout,
  }) async {
    return sendCommand(
      GetDiagnosticsTree(
        finder,
        DiagnosticsType.widget,
        subtreeDepth: subtreeDepth,
        includeProperties: includeProperties,
        timeout: timeout,
      ),
    );
  }

  /// Tell the driver to perform a scrolling action.
  ///
  /// A scrolling action begins with a "pointer down" event, which commonly maps
  /// to finger press on the touch screen or mouse button press. A series of
  /// "pointer move" events follow. The action is completed by a "pointer up"
  /// event.
  ///
  /// [dx] and [dy] specify the total offset for the entire scrolling action.
  ///
  /// [duration] specifies the length of the action.
  ///
  /// The move events are generated at a given [frequency] in Hz (or events per
  /// second). It defaults to 60Hz.
  Future<void> scroll(
    SerializableFinder finder,
    double dx,
    double dy,
    Duration duration, {
    int frequency = 60,
    Duration? timeout,
  }) async {
    await sendCommand(Scroll(finder, dx, dy, duration, frequency, timeout: timeout));
  }

  /// Scrolls the Scrollable ancestor of the widget located by [finder]
  /// until the widget is completely visible.
  ///
  /// If the widget located by [finder] is contained by a scrolling widget
  /// that lazily creates its children, like [ListView] or [CustomScrollView],
  /// then this method may fail because [finder] doesn't actually exist.
  /// The [scrollUntilVisible] method can be used in this case.
  Future<void> scrollIntoView(
    SerializableFinder finder, {
    double alignment = 0.0,
    Duration? timeout,
  }) async {
    await sendCommand(ScrollIntoView(finder, alignment: alignment, timeout: timeout));
  }

  /// Repeatedly [scroll] the widget located by [scrollable] by [dxScroll] and
  /// [dyScroll] until [item] is visible, and then use [scrollIntoView] to
  /// ensure the item's final position matches [alignment].
  ///
  /// The [scrollable] must locate the scrolling widget that contains [item].
  /// Typically `find.byType('ListView')` or `find.byType('CustomScrollView')`.
  ///
  /// At least one of [dxScroll] and [dyScroll] must be non-zero.
  ///
  /// If [item] is below the currently visible items, then specify a negative
  /// value for [dyScroll] that's a small enough increment to expose [item]
  /// without potentially scrolling it up and completely out of view. Similarly
  /// if [item] is above, then specify a positive value for [dyScroll].
  ///
  /// If [item] is to the right of the currently visible items, then
  /// specify a negative value for [dxScroll] that's a small enough increment to
  /// expose [item] without potentially scrolling it up and completely out of
  /// view. Similarly if [item] is to the left, then specify a positive value
  /// for [dyScroll].
  ///
  /// The [timeout] value should be long enough to accommodate as many scrolls
  /// as needed to bring an item into view. The default is to not time out.
  Future<void> scrollUntilVisible(
    SerializableFinder scrollable,
    SerializableFinder item, {
    double alignment = 0.0,
    double dxScroll = 0.0,
    double dyScroll = 0.0,
    Duration? timeout,
  }) async {
    assert(dxScroll != 0.0 || dyScroll != 0.0);

    // Kick off an (unawaited) waitFor that will complete when the item we're
    // looking for finally scrolls onscreen. We add an initial pause to give it
    // the chance to complete if the item is already onscreen; if not, scroll
    // repeatedly until we either find the item or time out.
    bool isVisible = false;
    waitFor(item, timeout: timeout).then<void>((_) {
      isVisible = true;
    });
    await Future<void>.delayed(const Duration(milliseconds: 500));
    while (!isVisible) {
      await scroll(scrollable, dxScroll, dyScroll, const Duration(milliseconds: 100));
      await Future<void>.delayed(const Duration(milliseconds: 500));
    }

    return scrollIntoView(item, alignment: alignment);
  }

  /// Returns the text in the `Text` widget located by [finder].
  Future<String> getText(SerializableFinder finder, {Duration? timeout}) async {
    return GetTextResult.fromJson(await sendCommand(GetText(finder, timeout: timeout))).text;
  }

  /// Enters `text` into the currently focused text input, such as the
  /// [EditableText] widget.
  ///
  /// This method does not use the operating system keyboard to enter text.
  /// Instead it emulates text entry by sending events identical to those sent
  /// by the operating system keyboard (the "TextInputClient.updateEditingState"
  /// method channel call).
  ///
  /// Generally the behavior is dependent on the implementation of the widget
  /// receiving the input. Usually, editable widgets, such as [EditableText] and
  /// those built on top of it would replace the currently entered text with the
  /// provided `text`.
  ///
  /// It is assumed that the widget receiving text input is focused prior to
  /// calling this method. Typically, a test would activate a widget, e.g. using
  /// [tap], then call this method.
  ///
  /// For this method to work, text emulation must be enabled (see
  /// [setTextEntryEmulation]). Text emulation is enabled by default.
  ///
  /// Example:
  ///
  /// ```dart
  /// test('enters text in a text field', () async {
  ///   final SerializableFinder textField = find.byValueKey('enter-text-field');
  ///   await driver.tap(textField);  // acquire focus
  ///   await driver.enterText('Hello!');  // enter text
  ///   await driver.waitFor(find.text('Hello!'));  // verify text appears on UI
  ///   await driver.enterText('World!');  // enter another piece of text
  ///   await driver.waitFor(find.text('World!'));  // verify new text appears
  /// });
  /// ```
  Future<void> enterText(String text, {Duration? timeout}) async {
    await sendCommand(EnterText(text, timeout: timeout));
  }

  /// Configures text entry emulation.
  ///
  /// If `enabled` is true, enables text entry emulation via [enterText]. If
  /// `enabled` is false, disables it. By default text entry emulation is
  /// enabled.
  ///
  /// When disabled, [enterText] will fail with a [DriverError]. When an
  /// [EditableText] is focused, the operating system's configured keyboard
  /// method is invoked, such as an on-screen keyboard on a phone or a tablet.
  ///
  /// When enabled, the operating system's configured keyboard will not be
  /// invoked when the widget is focused, as the [SystemChannels.textInput]
  /// channel will be mocked out.
  Future<void> setTextEntryEmulation({required bool enabled, Duration? timeout}) async {
    await sendCommand(SetTextEntryEmulation(enabled, timeout: timeout));
  }

  /// Simulate the user posting a text input action.
  ///
  /// The available action types can be found in [TextInputAction]. The [sendTextInputAction]
  /// does not check whether the [TextInputAction] performed is acceptable
  /// based on the client arguments of the text input.
  ///
  /// This can be called even if the [TestTextInput] has not been [TestTextInput.register]ed.
  ///
  /// Example:
  /// {@tool snippet}
  ///
  /// ```dart
  /// test('submit text in a text field', () async {
  ///   final SerializableFinder textField = find.byValueKey('enter-text-field');
  ///   await driver.tap(textField);  // acquire focus
  ///   await driver.enterText('Hello!');  // enter text
  ///   await driver.waitFor(find.text('Hello!'));  // verify text appears on UI
  ///   await driver.sendTextInputAction(TextInputAction.done);  // submit text
  /// });
  /// ```
  /// {@end-tool}
  ///
  Future<void> sendTextInputAction(TextInputAction action, {Duration? timeout}) async {
    await sendCommand(SendTextInputAction(action, timeout: timeout));
  }

  /// Sends a string and returns a string.
  ///
  /// This enables generic communication between the driver and the application.
  /// It's expected that the application has registered a [DataHandler]
  /// callback in [enableFlutterDriverExtension] that can successfully handle
  /// these requests.
  Future<String> requestData(String? message, {Duration? timeout}) async {
    return RequestDataResult.fromJson(
      await sendCommand(RequestData(message, timeout: timeout)),
    ).message;
  }

  /// Turns semantics on or off in the Flutter app under test.
  ///
  /// Returns true when the call actually changed the state from on to off or
  /// vice versa.
  ///
  /// Does not enable or disable the assistive technology installed on the
  /// device. For example, this does not enable VoiceOver on iOS, TalkBack on
  /// Android, or NVDA on Windows.
  ///
  /// Enabling semantics on the web causes the engine to render ARIA-annotated
  /// HTML.
  Future<bool> setSemantics(bool enabled, {Duration? timeout}) async {
    final SetSemanticsResult result = SetSemanticsResult.fromJson(
      await sendCommand(SetSemantics(enabled, timeout: timeout)),
    );
    return result.changedState;
  }

  /// Retrieves the semantics node id for the object returned by `finder`, or
  /// the nearest ancestor with a semantics node.
  ///
  /// Throws an error if `finder` returns multiple elements or a semantics
  /// node is not found.
  ///
  /// Semantics must be enabled to use this method, either using a platform
  /// specific shell command or [setSemantics].
  Future<int> getSemanticsId(SerializableFinder finder, {Duration? timeout}) async {
    final Map<String, dynamic> jsonResponse = await sendCommand(
      GetSemanticsId(finder, timeout: timeout),
    );
    final GetSemanticsIdResult result = GetSemanticsIdResult.fromJson(jsonResponse);
    return result.id;
  }

  /// Take a screenshot.
  ///
  /// The image will be returned as a PNG.
  ///
  /// **Warning:** This is not reliable.
  ///
  /// There is a two-second artificial delay before screenshotting. The delay
  /// here is to deal with a race between the driver script and the raster
  /// thread (formerly known as the GPU thread). The issue is that the driver
  /// API synchronizes with the framework based on transient callbacks, which
  /// are out of sync with the raster thread.
  ///
  /// Here's the timeline of events in ASCII art:
  ///
  ///     ---------------------------------------------------------------
  ///     Without this delay:
  ///     ---------------------------------------------------------------
  ///     UI    : <-- build -->
  ///     Raster:               <-- rasterize -->
  ///     Gap   :              | random |
  ///     Driver:                        <-- screenshot -->
  ///
  /// In the diagram above, the gap is the time between the last driver action
  /// taken, such as a `tap()`, and the subsequent call to `screenshot()`. The
  /// gap is random because it is determined by the unpredictable communication
  /// channel between the driver process and the application. If this gap is too
  /// short, which it typically will be, the screenshot is taken before the
  /// raster thread is done rasterizing the frame, so the screenshot of the
  /// previous frame is taken, which is not what is intended.
  ///
  ///     ---------------------------------------------------------------
  ///     With this delay, if we're lucky:
  ///     ---------------------------------------------------------------
  ///     UI    : <-- build -->
  ///     Raster:               <-- rasterize -->
  ///     Gap   :              |    2 seconds or more   |
  ///     Driver:                                        <-- screenshot -->
  ///
  /// The two-second gap should be long enough for the raster thread to finish
  /// rasterizing the frame, but not longer than necessary to keep driver tests
  /// as fast a possible.
  ///
  ///     ---------------------------------------------------------------
  ///     With this delay, if we're not lucky:
  ///     ---------------------------------------------------------------
  ///     UI    : <-- build -->
  ///     Raster:               <-- rasterize randomly slow today -->
  ///     Gap   :              |    2 seconds or more   |
  ///     Driver:                                        <-- screenshot -->
  ///
  /// In practice, sometimes the device gets really busy for a while and even
  /// two seconds isn't enough, which means that this is still racy and a source
  /// of flakes.
  Future<List<int>> screenshot({ScreenshotFormat format = ScreenshotFormat.png}) async {
    await Future<void>.delayed(const Duration(seconds: 2));
    final Map<String, Object?> jsonResponse = await sendCommand(ScreenshotCommand(format: format));
    return base64.decode(jsonResponse['data']! as String);
  }

  /// Returns the Flags set in the Dart VM as JSON.
  ///
  /// See the complete documentation for [the `getFlagList` Dart VM service
  /// method][getFlagList].
  ///
  /// Example return value:
  ///
  ///     [
  ///       {
  ///         "name": "timeline_recorder",
  ///         "comment": "Select the timeline recorder used. Valid values: ring, endless, startup, and systrace.",
  ///         "modified": false,
  ///         "_flagType": "String",
  ///         "valueAsString": "ring"
  ///       },
  ///       ...
  ///     ]
  ///
  /// [getFlagList]: https://github.com/dart-lang/sdk/blob/main/runtime/vm/service/service.md#getflaglist
  ///
  /// Throws [UnimplementedError] on [WebFlutterDriver] instances.
  Future<List<Map<String, dynamic>>> getVmFlags() async {
    throw UnimplementedError();
  }

  /// Starts recording performance traces.
  ///
  /// The `timeout` argument causes a warning to be displayed to the user if the
  /// operation exceeds the specified timeout; it does not actually cancel the
  /// operation.
  ///
  /// For [WebFlutterDriver], this is only supported for Chrome.
  Future<void> startTracing({
    List<TimelineStream> streams = const <TimelineStream>[TimelineStream.all],
    Duration timeout = kUnusuallyLongTimeout,
  }) async {
    throw UnimplementedError();
  }

  /// Stops recording performance traces and downloads the timeline.
  ///
  /// The `timeout` argument causes a warning to be displayed to the user if the
  /// operation exceeds the specified timeout; it does not actually cancel the
  /// operation.
  ///
  /// For [WebFlutterDriver], this is only supported for Chrome.
  Future<Timeline> stopTracingAndDownloadTimeline({
    Duration timeout = kUnusuallyLongTimeout,
  }) async {
    throw UnimplementedError();
  }

  /// Runs [action] and outputs a performance trace for it.
  ///
  /// Waits for the `Future` returned by [action] to complete prior to stopping
  /// the trace.
  ///
  /// This is merely a convenience wrapper on top of [startTracing] and
  /// [stopTracingAndDownloadTimeline].
  ///
  /// [streams] limits the recorded timeline event streams to only the ones
  /// listed. By default, all streams are recorded.
  ///
  /// If [retainPriorEvents] is true, retains events recorded prior to calling
  /// [action]. Otherwise, prior events are cleared before calling [action]. By
  /// default, prior events are cleared.
  ///
  /// If this is run in debug mode, a warning message will be printed to suggest
  /// running the benchmark in profile mode instead.
  ///
  /// For [WebFlutterDriver], this is only supported for Chrome.
  Future<Timeline> traceAction(
    Future<dynamic> Function() action, {
    List<TimelineStream> streams = const <TimelineStream>[TimelineStream.all],
    bool retainPriorEvents = false,
  }) async {
    throw UnimplementedError();
  }

  /// Clears all timeline events recorded up until now.
  ///
  /// The `timeout` argument causes a warning to be displayed to the user if the
  /// operation exceeds the specified timeout; it does not actually cancel the
  /// operation.
  ///
  /// For [WebFlutterDriver], this is only supported for Chrome.
  Future<void> clearTimeline({Duration timeout = kUnusuallyLongTimeout}) async {
    throw UnimplementedError();
  }

  /// [action] will be executed with the frame sync mechanism disabled.
  ///
  /// By default, Flutter Driver waits until there is no pending frame scheduled
  /// in the app under test before executing an action. This mechanism is called
  /// "frame sync". It greatly reduces flakiness because Flutter Driver will not
  /// execute an action while the app under test is undergoing a transition.
  ///
  /// Having said that, sometimes it is necessary to disable the frame sync
  /// mechanism (e.g. if there is an ongoing animation in the app, it will
  /// never reach a state where there are no pending frames scheduled and the
  /// action will time out). For these cases, the sync mechanism can be disabled
  /// by wrapping the actions to be performed by this [runUnsynchronized] method.
  ///
  /// With frame sync disabled, it's the responsibility of the test author to
  /// ensure that no action is performed while the app is undergoing a
  /// transition to avoid flakiness.
  Future<T> runUnsynchronized<T>(Future<T> Function() action, {Duration? timeout}) async {
    await sendCommand(SetFrameSync(false, timeout: timeout));
    T result;
    try {
      result = await action();
    } finally {
      await sendCommand(SetFrameSync(true, timeout: timeout));
    }
    return result;
  }

  /// Force a garbage collection run in the VM.
  ///
  /// Throws [UnimplementedError] on [WebFlutterDriver] instances.
  Future<void> forceGC() async {
    throw UnimplementedError();
  }

  /// Closes the underlying connection to the VM service.
  ///
  /// Returns a [Future] that fires once the connection has been closed.
  Future<void> close() async {
    throw UnimplementedError();
  }
}

/// Provides convenient accessors to frequently used finders.
class CommonFinders {
  const CommonFinders._();

  /// Finds [Text] and [EditableText] widgets containing string
  /// equal to [text].
  SerializableFinder text(String text) => ByText(text);

  /// Finds widgets by [key]. Only [String] and [int] values can be used.
  SerializableFinder byValueKey(dynamic key) => ByValueKey(key);

  /// Finds widgets with a tooltip with the given [message].
  SerializableFinder byTooltip(String message) => ByTooltipMessage(message);

  /// Finds widgets with the given semantics [label].
  SerializableFinder bySemanticsLabel(Pattern label) => BySemanticsLabel(label);

  /// Finds widgets whose class name matches the given string.
  SerializableFinder byType(String type) => ByType(type);

  /// Finds the back button on a Material or Cupertino page's scaffold.
  SerializableFinder pageBack() => const PageBack();

  /// Finds the widget that is an ancestor of the `of` parameter and that
  /// matches the `matching` parameter.
  ///
  /// If the `matchRoot` argument is true then the widget specified by `of` will
  /// be considered for a match. The argument defaults to false.
  ///
  /// If `firstMatchOnly` is true then only the first ancestor matching
  /// `matching` will be returned. Defaults to false.
  SerializableFinder ancestor({
    required SerializableFinder of,
    required SerializableFinder matching,
    bool matchRoot = false,
    bool firstMatchOnly = false,
  }) => Ancestor(of: of, matching: matching, matchRoot: matchRoot, firstMatchOnly: firstMatchOnly);

  /// Finds the widget that is an descendant of the `of` parameter and that
  /// matches the `matching` parameter.
  ///
  /// If the `matchRoot` argument is true then the widget specified by `of` will
  /// be considered for a match. The argument defaults to false.
  ///
  /// If `firstMatchOnly` is true then only the first descendant matching
  /// `matching` will be returned. Defaults to false.
  SerializableFinder descendant({
    required SerializableFinder of,
    required SerializableFinder matching,
    bool matchRoot = false,
    bool firstMatchOnly = false,
  }) =>
      Descendant(of: of, matching: matching, matchRoot: matchRoot, firstMatchOnly: firstMatchOnly);
}

/// An immutable 2D floating-point offset used by Flutter Driver.
@immutable
class DriverOffset {
  /// Creates an offset.
  const DriverOffset(this.dx, this.dy);

  /// The x component of the offset.
  final double dx;

  /// The y component of the offset.
  final double dy;

  @override
  String toString() => '$runtimeType($dx, $dy)'; // ignore: no_runtimetype_tostring, can't access package:flutter here to use objectRuntimeType

  @override
  bool operator ==(Object other) {
    return other is DriverOffset && other.dx == dx && other.dy == dy;
  }

  @override
  int get hashCode => Object.hash(dx, dy);
}
