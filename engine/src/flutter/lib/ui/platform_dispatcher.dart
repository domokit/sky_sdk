// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
part of dart.ui;

/// Signature of callbacks that have no arguments and return no data.
typedef VoidCallback = void Function();

/// Signature for [PlatformDispatcher.onBeginFrame].
///
/// The `duration` argument is the point at which the current frame interval
/// began, expressed as a duration since some epoch. The epoch in all
/// frames will be the same, but it may not match [DateTime]'s epoch.
///
/// For any two frames `a` and `b` such that the frame number of `a` is less
/// than the frame number of `b`, the duration argument for `a` will be less
/// than or equal to the duration argument for `b`.
typedef FrameCallback = void Function(Duration duration);

/// Signature for [PlatformDispatcher.onReportTimings].
///
/// {@template dart.ui.TimingsCallback.list}
/// The callback takes a list of [FrameTiming] because it may not be
/// immediately triggered after each frame. Instead, Flutter tries to batch
/// frames together and send all their timings at once to decrease the
/// overhead (as this is available in the release mode). The list is sorted in
/// ascending order of time (earliest frame first). The timing of any frame
/// will be sent within about 1 second (100ms if in the profile/debug mode)
/// even if there are no later frames to batch. The timing of the first frame
/// will be sent immediately without batching.
/// {@endtemplate}
typedef TimingsCallback = void Function(List<FrameTiming> timings);

/// Signature for [PlatformDispatcher.onPointerDataPacket].
typedef PointerDataPacketCallback = void Function(PointerDataPacket packet);

/// Signature for [PlatformDispatcher.onKeyData].
///
/// The callback should return true if the key event has been handled by the
/// framework and should not be propagated further.
typedef KeyDataCallback = bool Function(KeyData data);

/// Signature for [PlatformDispatcher.onSemanticsActionEvent].
typedef SemanticsActionEventCallback = void Function(SemanticsActionEvent action);

/// Signature for responses to platform messages.
///
/// Used as a parameter to [PlatformDispatcher.sendPlatformMessage] and
/// [PlatformDispatcher.onPlatformMessage].
typedef PlatformMessageResponseCallback = void Function(ByteData? data);

/// Deprecated. Migrate to [ChannelBuffers.setListener] instead.
///
/// Signature for [PlatformDispatcher.onPlatformMessage].
@Deprecated(
  'Migrate to ChannelBuffers.setListener instead. '
  'This feature was deprecated after v3.11.0-20.0.pre.',
)
typedef PlatformMessageCallback =
    void Function(String name, ByteData? data, PlatformMessageResponseCallback? callback);

// Signature for _setNeedsReportTimings.
typedef _SetNeedsReportTimingsFunc = void Function(bool value);

/// Signature for [PlatformDispatcher.onError].
///
/// If this method returns false, the engine may use some fallback method to
/// provide information about the error.
///
/// After calling this method, the process or the VM may terminate. Some severe
/// unhandled errors may not be able to call this method either, such as Dart
/// compilation errors or process terminating errors.
typedef ErrorCallback = bool Function(Object exception, StackTrace stackTrace);

// A gesture setting value that indicates it has not been set by the engine.
const double _kUnsetGestureSetting = -1.0;

// A message channel to receive KeyData from the platform.
//
// See embedder.cc::kFlutterKeyDataChannel for more information.
const String _kFlutterKeyDataChannel = 'flutter/keydata';

@pragma('vm:entry-point')
ByteData? _wrapUnmodifiableByteData(ByteData? byteData) => byteData?.asUnmodifiableView();

/// A token that represents a root isolate.
class RootIsolateToken {
  RootIsolateToken._(this._token);

  /// An enumeration representing the root isolate (0 if not a root isolate).
  final int _token;

  /// The token for the root isolate that is executing this Dart code.  If this
  /// Dart code is not executing on a root isolate [instance] will be null.
  static final RootIsolateToken? instance = () {
    final int token = __getRootIsolateToken();
    return token == 0 ? null : RootIsolateToken._(token);
  }();

  @Native<Int64 Function()>(symbol: 'PlatformConfigurationNativeApi::GetRootIsolateToken')
  external static int __getRootIsolateToken();
}

/// Platform event dispatcher singleton.
///
/// The most basic interface to the host operating system's interface.
///
/// This is the central entry point for platform messages and configuration
/// events from the platform.
///
/// It exposes the core scheduler API, the input event callback, the graphics
/// drawing API, and other such core services.
///
/// It manages the list of the application's [views] as well as the
/// [configuration] of various platform attributes.
///
/// Consider avoiding static references to this singleton through
/// [PlatformDispatcher.instance] and instead prefer using a binding for
/// dependency resolution such as `WidgetsBinding.instance.platformDispatcher`.
/// See [PlatformDispatcher.instance] for more information about why this is
/// preferred.
class PlatformDispatcher {
  /// Private constructor, since only dart:ui is supposed to create one of
  /// these. Use [instance] to access the singleton.
  PlatformDispatcher._() {
    _setNeedsReportTimings = _nativeSetNeedsReportTimings;
  }

  /// The [PlatformDispatcher] singleton.
  ///
  /// Consider avoiding static references to this singleton through
  /// [PlatformDispatcher.instance] and instead prefer using a binding for
  /// dependency resolution such as `WidgetsBinding.instance.platformDispatcher`.
  ///
  /// Static access of this object means that Flutter has few, if any options to
  /// fake or mock the given object in tests. Even in cases where Dart offers
  /// special language constructs to forcefully shadow such properties, those
  /// mechanisms would only be reasonable for tests and they would not be
  /// reasonable for a future of Flutter where we legitimately want to select an
  /// appropriate implementation at runtime.
  ///
  /// The only place that `WidgetsBinding.instance.platformDispatcher` is
  /// inappropriate is if access to these APIs is required before the binding is
  /// initialized by invoking `runApp()` or
  /// `WidgetsFlutterBinding.instance.ensureInitialized()`. In that case, it is
  /// necessary (though unfortunate) to use the [PlatformDispatcher.instance]
  /// object statically.
  static PlatformDispatcher get instance => _instance;
  static final PlatformDispatcher _instance = PlatformDispatcher._();

  _PlatformConfiguration _configuration = const _PlatformConfiguration();

  /// Called when the platform configuration changes.
  ///
  /// The engine invokes this callback in the same zone in which the callback
  /// was set.
  VoidCallback? get onPlatformConfigurationChanged => _onPlatformConfigurationChanged;
  VoidCallback? _onPlatformConfigurationChanged;
  Zone _onPlatformConfigurationChangedZone = Zone.root;
  set onPlatformConfigurationChanged(VoidCallback? callback) {
    _onPlatformConfigurationChanged = callback;
    _onPlatformConfigurationChangedZone = Zone.current;
  }

  /// The current list of displays.
  ///
  /// If any of their configurations change, [onMetricsChanged] will be called.
  ///
  /// To get the display for a [FlutterView], use [FlutterView.display].
  ///
  /// Platforms may limit what information is available to the application with
  /// regard to secondary displays and/or displays that do not have an active
  /// application window.
  ///
  /// Presently, on Android and Web this collection will only contain the
  /// display that the current window is on. On iOS, it will only contains the
  /// main display on the phone or tablet. On Desktop, it will contain only
  /// a main display with a valid refresh rate but invalid size and device
  /// pixel ratio values.
  // TODO(dnfield): Update these docs when https://github.com/flutter/flutter/issues/125939
  // and https://github.com/flutter/flutter/issues/125938 are resolved.
  Iterable<Display> get displays => _displays.values;
  final Map<int, Display> _displays = <int, Display>{};

  /// The current list of views, including top level platform windows used by
  /// the application.
  ///
  /// If any of their configurations change, [onMetricsChanged] will be called.
  Iterable<FlutterView> get views => _views.values;
  final Map<int, FlutterView> _views = <int, FlutterView>{};

  /// Returns the [FlutterView] with the provided ID if one exists, or null
  /// otherwise.
  FlutterView? view({required int id}) => _views[id];

  /// The [FlutterView] provided by the engine if the platform is unable to
  /// create windows, or, for backwards compatibility.
  ///
  /// If the platform provides an implicit view, it can be used to bootstrap
  /// the framework. This is common for platforms designed for single-view
  /// applications like mobile devices with a single display.
  ///
  /// Applications and libraries must not rely on this property being set
  /// as it may be null depending on the engine's configuration. Instead,
  /// consider using [View.of] to lookup the [FlutterView] the current
  /// [BuildContext] is drawing into.
  ///
  /// While the properties on the referenced [FlutterView] may change,
  /// the reference itself is guaranteed to never change over the lifetime
  /// of the application: if this property is null at startup, it will remain
  /// so throughout the entire lifetime of the application. If it points to a
  /// specific [FlutterView], it will continue to point to the same view until
  /// the application is shut down (although the engine may replace or remove
  /// the underlying backing surface of the view at its discretion).
  ///
  /// See also:
  ///
  /// * [View.of], for accessing the current view.
  /// * [PlatformDispatcher.views] for a list of all [FlutterView]s provided
  ///   by the platform.
  FlutterView? get implicitView {
    final FlutterView? result = _views[_implicitViewId];
    // Make sure [implicitView] agrees with `_implicitViewId`.
    assert(
      (result != null) == (_implicitViewId != null),
      (_implicitViewId != null)
          ? 'The implicit view ID is $_implicitViewId, but the implicit view does not exist.'
          : 'The implicit view ID is null, but the implicit view exists.',
    );
    // Make sure [implicitView] never chages.
    assert(() {
      if (_debugRecordedLastImplicitView) {
        assert(
          identical(_debugLastImplicitView, result),
          'The implicitView has changed:\n'
          'Last: $_debugLastImplicitView\nCurrent: $result',
        );
      } else {
        _debugLastImplicitView = result;
        _debugRecordedLastImplicitView = true;
      }
      return true;
    }());
    return result;
  }

  FlutterView? _debugLastImplicitView;
  bool _debugRecordedLastImplicitView = false;

  /// A callback that is invoked whenever the [ViewConfiguration] of any of the
  /// [views] changes.
  ///
  /// For example when the device is rotated or when the application is resized
  /// (e.g. when showing applications side-by-side on Android),
  /// `onMetricsChanged` is called.
  ///
  /// The engine invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// The framework registers with this callback and updates the layout
  /// appropriately.
  ///
  /// See also:
  ///
  /// * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///   register for notifications when this is called.
  /// * [MediaQuery.of], a simpler mechanism for the same.
  VoidCallback? get onMetricsChanged => _onMetricsChanged;
  VoidCallback? _onMetricsChanged;
  Zone _onMetricsChangedZone = Zone.root;
  set onMetricsChanged(VoidCallback? callback) {
    _onMetricsChanged = callback;
    _onMetricsChangedZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  //
  // Adds a new view with the specific view configuration.
  //
  // The implicit view must be added before [implicitView] is first called,
  // which is typically the main function.
  void _addView(int id, _ViewConfiguration viewConfiguration) {
    assert(!_views.containsKey(id), 'View ID $id already exists.');
    _views[id] = FlutterView._(id, this, viewConfiguration);
    _invoke(onMetricsChanged, _onMetricsChangedZone);
  }

  // Called from the engine, via hooks.dart
  //
  // Removes the specific view.
  //
  // The target view must must exist. The implicit view must not be removed,
  // or an assertion will be triggered.
  void _removeView(int id) {
    assert(id != _implicitViewId, 'The implicit view #$id can not be removed.');
    if (id == _implicitViewId) {
      return;
    }
    assert(_views.containsKey(id), 'View ID $id does not exist.');
    _views.remove(id);
    _invoke(onMetricsChanged, _onMetricsChangedZone);
  }

  // Called from the engine, via hooks.dart.
  //
  // Updates the available displays.
  void _updateDisplays(List<Display> displays) {
    _displays.clear();
    for (final Display display in displays) {
      _displays[display.id] = display;
    }
    _invoke(onMetricsChanged, _onMetricsChangedZone);
  }

  // Called from the engine, via hooks.dart
  //
  // Updates the metrics of the window with the given id.
  void _updateWindowMetrics(int viewId, _ViewConfiguration viewConfiguration) {
    assert(_views.containsKey(viewId), 'View $viewId does not exist.');
    _views[viewId]!._viewConfiguration = viewConfiguration;
    _invoke(onMetricsChanged, _onMetricsChangedZone);
  }

  /// A callback invoked immediately after the focus is transitioned across [FlutterView]s.
  ///
  /// When the platform moves the focus from one [FlutterView] to another, this
  /// callback is invoked indicating the new view that has focus and the direction
  /// in which focus was received. For example, if focus is moved to the [FlutterView]
  /// with ID 2 in the forward direction (could be the result of pressing tab)
  /// the callback receives a [ViewFocusEvent] with [ViewFocusState.focused] and
  /// [ViewFocusDirection.forward].
  ///
  /// Typically, receivers of this event respond by moving the focus to the first
  /// focusable widget inside the [FlutterView] with ID 2. If a view receives
  /// focus in the backward direction (could be the result of pressing shift + tab),
  /// typically the last focusable widget inside that view is focused.
  ///
  /// The platform may remove focus from a [FlutterView]. For example, on the web,
  /// the browser can move focus to another element, or to the browser's built-in UI.
  /// On desktop, the operating system can switch to another window (e.g. using Alt + Tab on Windows).
  /// In scenarios like these, [onViewFocusChange] will be invoked with [ViewFocusState.unfocused] and
  /// [ViewFocusDirection.undefined].
  ///
  /// Receivers typically respond to this event by removing all focus indications
  /// from the app.
  ///
  /// Apps can also programmatically request to move the focus to a desired
  /// [FlutterView] by calling [requestViewFocusChange].
  ///
  /// The callback is invoked in the same zone in which the callback was set.
  ///
  /// See also:
  ///
  ///   * [requestViewFocusChange] to programmatically instruct the platform to move focus to a different [FlutterView].
  ///   * [ViewFocusState] for a list of allowed focus transitions.
  ///   * [ViewFocusDirection] for a list of allowed focus directions.
  ///   * [ViewFocusEvent], which is the event object provided to the callback.
  ViewFocusChangeCallback? get onViewFocusChange => _onViewFocusChange;
  ViewFocusChangeCallback? _onViewFocusChange;
  // ignore: unused_field, field will be used when platforms other than web use these focus APIs.
  Zone _onViewFocusChangeZone = Zone.root;
  set onViewFocusChange(ViewFocusChangeCallback? callback) {
    _onViewFocusChange = callback;
    _onViewFocusChangeZone = Zone.current;
  }

  /// Requests a focus change of the [FlutterView] with ID [viewId].
  ///
  /// If an app would like to request the engine to move focus, in forward direction,
  /// to the [FlutterView] with ID 1 it should call this method with [ViewFocusState.focused]
  /// and [ViewFocusDirection.forward].
  ///
  /// There is no need to call this method if the view in question already has
  /// focus as it won't have any effect.
  ///
  /// A call to this method will lead to the engine calling [onViewFocusChange]
  /// if the request is successfully fulfilled.
  ///
  /// See also:
  ///
  ///  * [onViewFocusChange], a callback to subscribe to view focus change events.
  void requestViewFocusChange({
    required int viewId,
    required ViewFocusState state,
    required ViewFocusDirection direction,
  }) {
    // TODO(tugorez): implement this method. At the moment will be a no op call.
  }

  /// A callback invoked when any view begins a frame.
  ///
  /// A callback that is invoked to notify the application that it is an
  /// appropriate time to provide a scene using the [SceneBuilder] API and the
  /// [FlutterView.render] method.
  ///
  /// When possible, this is driven by the hardware VSync signal of the attached
  /// screen with the highest VSync rate. This is only called if
  /// [PlatformDispatcher.scheduleFrame] has been called since the last time
  /// this callback was invoked.
  FrameCallback? get onBeginFrame => _onBeginFrame;
  FrameCallback? _onBeginFrame;
  Zone _onBeginFrameZone = Zone.root;
  set onBeginFrame(FrameCallback? callback) {
    _onBeginFrame = callback;
    _onBeginFrameZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _beginFrame(int microseconds) {
    _invoke1<Duration>(onBeginFrame, _onBeginFrameZone, Duration(microseconds: microseconds));
  }

  /// A callback that is invoked for each frame after [onBeginFrame] has
  /// completed and after the microtask queue has been drained.
  ///
  /// This can be used to implement a second phase of frame rendering that
  /// happens after any deferred work queued by the [onBeginFrame] phase.
  VoidCallback? get onDrawFrame => _onDrawFrame;
  VoidCallback? _onDrawFrame;
  Zone _onDrawFrameZone = Zone.root;
  set onDrawFrame(VoidCallback? callback) {
    _onDrawFrame = callback;
    _onDrawFrameZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _drawFrame() {
    _invoke(onDrawFrame, _onDrawFrameZone);
  }

  /// A callback that is invoked when pointer data is available.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// See also:
  ///
  ///  * [GestureBinding], the Flutter framework class which manages pointer
  ///    events.
  PointerDataPacketCallback? get onPointerDataPacket => _onPointerDataPacket;
  PointerDataPacketCallback? _onPointerDataPacket;
  Zone _onPointerDataPacketZone = Zone.root;
  set onPointerDataPacket(PointerDataPacketCallback? callback) {
    _onPointerDataPacket = callback;
    _onPointerDataPacketZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _dispatchPointerDataPacket(ByteData packet) {
    if (onPointerDataPacket != null) {
      _invoke1<PointerDataPacket>(
        onPointerDataPacket,
        _onPointerDataPacketZone,
        _unpackPointerDataPacket(packet),
      );
    }
  }

  // This value must match kPointerDataFieldCount in pointer_data.cc. (The
  // pointer_data.cc also lists other locations that must be kept consistent.)
  static const int _kPointerDataFieldCount = 36;

  static PointerDataPacket _unpackPointerDataPacket(ByteData packet) {
    const int kStride = Int64List.bytesPerElement;
    const int kBytesPerPointerData = _kPointerDataFieldCount * kStride;
    final int length = packet.lengthInBytes ~/ kBytesPerPointerData;
    assert(length * kBytesPerPointerData == packet.lengthInBytes);
    final List<PointerData> data = <PointerData>[];
    for (int i = 0; i < length; ++i) {
      int offset = i * _kPointerDataFieldCount;
      data.add(
        PointerData(
          // The unpacking code must match the struct in pointer_data.h.
          embedderId: packet.getInt64(kStride * offset++, _kFakeHostEndian),
          timeStamp: Duration(microseconds: packet.getInt64(kStride * offset++, _kFakeHostEndian)),
          change: PointerChange.values[packet.getInt64(kStride * offset++, _kFakeHostEndian)],
          kind: PointerDeviceKind.values[packet.getInt64(kStride * offset++, _kFakeHostEndian)],
          signalKind:
              PointerSignalKind.values[packet.getInt64(kStride * offset++, _kFakeHostEndian)],
          device: packet.getInt64(kStride * offset++, _kFakeHostEndian),
          pointerIdentifier: packet.getInt64(kStride * offset++, _kFakeHostEndian),
          physicalX: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          physicalY: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          physicalDeltaX: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          physicalDeltaY: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          buttons: packet.getInt64(kStride * offset++, _kFakeHostEndian),
          obscured: packet.getInt64(kStride * offset++, _kFakeHostEndian) != 0,
          synthesized: packet.getInt64(kStride * offset++, _kFakeHostEndian) != 0,
          pressure: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          pressureMin: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          pressureMax: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          distance: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          distanceMax: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          size: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          radiusMajor: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          radiusMinor: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          radiusMin: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          radiusMax: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          orientation: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          tilt: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          platformData: packet.getInt64(kStride * offset++, _kFakeHostEndian),
          scrollDeltaX: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          scrollDeltaY: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          panX: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          panY: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          panDeltaX: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          panDeltaY: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          scale: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          rotation: packet.getFloat64(kStride * offset++, _kFakeHostEndian),
          viewId: packet.getInt64(kStride * offset++, _kFakeHostEndian),
        ),
      );
      assert(offset == (i + 1) * _kPointerDataFieldCount);
    }
    return PointerDataPacket(data: data);
  }

  static ChannelCallback _keyDataListener(KeyDataCallback onKeyData, Zone zone) => (
    ByteData? packet,
    PlatformMessageResponseCallback callback,
  ) {
    _invoke1<KeyData>(
      (KeyData keyData) {
        final bool handled = onKeyData(keyData);
        final Uint8List response = Uint8List(1);
        response[0] = handled ? 1 : 0;
        callback(response.buffer.asByteData());
      },
      zone,
      _unpackKeyData(packet!),
    );
  };

  /// A callback that is invoked when key data is available.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// The callback should return true if the key event has been handled by the
  /// framework and should not be propagated further.
  KeyDataCallback? get onKeyData => _onKeyData;
  KeyDataCallback? _onKeyData;
  set onKeyData(KeyDataCallback? callback) {
    _onKeyData = callback;
    if (callback != null) {
      channelBuffers.setListener(_kFlutterKeyDataChannel, _keyDataListener(callback, Zone.current));
    } else {
      channelBuffers.clearListener(_kFlutterKeyDataChannel);
    }
  }

  // If this value changes, update the encoding code in the following files:
  //
  //  * key_data.h (kKeyDataFieldCount)
  //  * KeyData.java (KeyData.FIELD_COUNT)
  static const int _kKeyDataFieldCount = 6;

  // The packet structure is described in `key_data_packet.h`.
  static KeyData _unpackKeyData(ByteData packet) {
    const int kStride = Int64List.bytesPerElement;

    int offset = 0;
    final int charDataSize = packet.getUint64(kStride * offset++, _kFakeHostEndian);
    final String? character =
        charDataSize == 0
            ? null
            : utf8.decoder.convert(
              packet.buffer.asUint8List(kStride * (offset + _kKeyDataFieldCount), charDataSize),
            );

    final KeyData keyData = KeyData(
      timeStamp: Duration(microseconds: packet.getUint64(kStride * offset++, _kFakeHostEndian)),
      type: KeyEventType.values[packet.getInt64(kStride * offset++, _kFakeHostEndian)],
      physical: packet.getUint64(kStride * offset++, _kFakeHostEndian),
      logical: packet.getUint64(kStride * offset++, _kFakeHostEndian),
      character: character,
      synthesized: packet.getUint64(kStride * offset++, _kFakeHostEndian) != 0,
    );

    return keyData;
  }

  /// A callback that is invoked to report the [FrameTiming] of recently
  /// rasterized frames.
  ///
  /// It's preferred to use [SchedulerBinding.addTimingsCallback] than to use
  /// [onReportTimings] directly because [SchedulerBinding.addTimingsCallback]
  /// allows multiple callbacks.
  ///
  /// This can be used to see if the application has missed frames (through
  /// [FrameTiming.buildDuration] and [FrameTiming.rasterDuration]), or high
  /// latencies (through [FrameTiming.totalSpan]).
  ///
  /// Unlike [Timeline], the timing information here is available in the release
  /// mode (additional to the profile and the debug mode). Hence this can be
  /// used to monitor the application's performance in the wild.
  ///
  /// {@macro dart.ui.TimingsCallback.list}
  ///
  /// If this is null, no additional work will be done. If this is not null,
  /// Flutter spends less than 0.1ms every 1 second to report the timings
  /// (measured on iPhone6S). The 0.1ms is about 0.6% of 16ms (frame budget for
  /// 60fps), or 0.01% CPU usage per second.
  TimingsCallback? get onReportTimings => _onReportTimings;
  TimingsCallback? _onReportTimings;
  Zone _onReportTimingsZone = Zone.root;
  set onReportTimings(TimingsCallback? callback) {
    if ((callback == null) != (_onReportTimings == null)) {
      _setNeedsReportTimings(callback != null);
    }
    _onReportTimings = callback;
    _onReportTimingsZone = Zone.current;
  }

  late _SetNeedsReportTimingsFunc _setNeedsReportTimings;

  void _nativeSetNeedsReportTimings(bool value) => __nativeSetNeedsReportTimings(value);

  @Native<Void Function(Bool)>(symbol: 'PlatformConfigurationNativeApi::SetNeedsReportTimings')
  external static void __nativeSetNeedsReportTimings(bool value);

  // Called from the engine, via hooks.dart
  void _reportTimings(List<int> timings) {
    assert(timings.length % FrameTiming._dataLength == 0);
    final List<FrameTiming> frameTimings = <FrameTiming>[];
    for (int i = 0; i < timings.length; i += FrameTiming._dataLength) {
      frameTimings.add(FrameTiming._(timings.sublist(i, i + FrameTiming._dataLength)));
    }
    _invoke1(onReportTimings, _onReportTimingsZone, frameTimings);
  }

  /// Sends a message to a platform-specific plugin.
  ///
  /// The `name` parameter determines which plugin receives the message. The
  /// `data` parameter contains the message payload and is typically UTF-8
  /// encoded JSON but can be arbitrary data. If the plugin replies to the
  /// message, `callback` will be called with the response.
  ///
  /// The framework invokes [callback] in the same zone in which this method was
  /// called.
  void sendPlatformMessage(String name, ByteData? data, PlatformMessageResponseCallback? callback) {
    final String? error = _sendPlatformMessage(
      name,
      _zonedPlatformMessageResponseCallback(callback),
      data,
    );
    if (error != null) {
      throw Exception(error);
    }
  }

  String? _sendPlatformMessage(
    String name,
    PlatformMessageResponseCallback? callback,
    ByteData? data,
  ) => __sendPlatformMessage(name, callback, data);

  @Native<Handle Function(Handle, Handle, Handle)>(
    symbol: 'PlatformConfigurationNativeApi::SendPlatformMessage',
  )
  external static String? __sendPlatformMessage(
    String name,
    PlatformMessageResponseCallback? callback,
    ByteData? data,
  );

  /// Sends a message to a platform-specific plugin via a [SendPort].
  ///
  /// This operates similarly to [sendPlatformMessage] but is used when sending
  /// messages from background isolates. The [port] parameter allows Flutter to
  /// know which isolate to send the result to. The [name] parameter is the name
  /// of the channel communication will happen on. The [data] parameter is the
  /// payload of the message. The [identifier] parameter is a unique integer
  /// assigned to the message.
  void sendPortPlatformMessage(String name, ByteData? data, int identifier, SendPort port) {
    final String? error = _sendPortPlatformMessage(name, identifier, port.nativePort, data);
    if (error != null) {
      throw Exception(error);
    }
  }

  String? _sendPortPlatformMessage(String name, int identifier, int port, ByteData? data) =>
      __sendPortPlatformMessage(name, identifier, port, data);

  @Native<Handle Function(Handle, Handle, Handle, Handle)>(
    symbol: 'PlatformConfigurationNativeApi::SendPortPlatformMessage',
  )
  external static String? __sendPortPlatformMessage(
    String name,
    int identifier,
    int port,
    ByteData? data,
  );

  /// Registers the current isolate with the isolate identified with by the
  /// [token]. This is required if platform channels are to be used on a
  /// background isolate.
  void registerBackgroundIsolate(RootIsolateToken token) {
    DartPluginRegistrant.ensureInitialized();
    __registerBackgroundIsolate(token._token);
  }

  @Native<Void Function(Int64)>(symbol: 'PlatformConfigurationNativeApi::RegisterBackgroundIsolate')
  external static void __registerBackgroundIsolate(int rootIsolateId);

  /// Deprecated. Migrate to [ChannelBuffers.setListener] instead.
  ///
  /// Called whenever this platform dispatcher receives a message from a
  /// platform-specific plugin.
  ///
  /// The `name` parameter determines which plugin sent the message. The `data`
  /// parameter is the payload and is typically UTF-8 encoded JSON but can be
  /// arbitrary data.
  ///
  /// Message handlers must call the function given in the `callback` parameter.
  /// If the handler does not need to respond, the handler should pass null to
  /// the callback.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  @Deprecated(
    'Migrate to ChannelBuffers.setListener instead. '
    'This feature was deprecated after v3.11.0-20.0.pre.',
  )
  PlatformMessageCallback? get onPlatformMessage => _onPlatformMessage;
  PlatformMessageCallback? _onPlatformMessage;
  Zone _onPlatformMessageZone = Zone.root;
  @Deprecated(
    'Migrate to ChannelBuffers.setListener instead. '
    'This feature was deprecated after v3.11.0-20.0.pre.',
  )
  set onPlatformMessage(PlatformMessageCallback? callback) {
    _onPlatformMessage = callback;
    _onPlatformMessageZone = Zone.current;
  }

  /// Called by [_dispatchPlatformMessage].
  void _respondToPlatformMessage(int responseId, ByteData? data) =>
      __respondToPlatformMessage(responseId, data);

  @Native<Void Function(IntPtr, Handle)>(
    symbol: 'PlatformConfigurationNativeApi::RespondToPlatformMessage',
  )
  external static void __respondToPlatformMessage(int responseId, ByteData? data);

  /// Wraps the given [callback] in another callback that ensures that the
  /// original callback is called in the zone it was registered in.
  static PlatformMessageResponseCallback? _zonedPlatformMessageResponseCallback(
    PlatformMessageResponseCallback? callback,
  ) {
    if (callback == null) {
      return null;
    }

    // Store the zone in which the callback is being registered.
    final Zone registrationZone = Zone.current;

    return (ByteData? data) {
      registrationZone.runUnaryGuarded(callback, data);
    };
  }

  /// Send a message to the framework using the [ChannelBuffers].
  ///
  /// This method constructs the appropriate callback to respond
  /// with the given `responseId`. It should only be called for messages
  /// from the platform.
  void _dispatchPlatformMessage(String name, ByteData? data, int responseId) {
    if (name == ChannelBuffers.kControlChannelName) {
      try {
        channelBuffers.handleMessage(data!);
      } finally {
        _respondToPlatformMessage(responseId, null);
      }
    } else if (onPlatformMessage != null) {
      _invoke3<String, ByteData?, PlatformMessageResponseCallback>(
        onPlatformMessage,
        _onPlatformMessageZone,
        name,
        data,
        (ByteData? responseData) {
          _respondToPlatformMessage(responseId, responseData);
        },
      );
    } else {
      channelBuffers.push(name, data, (ByteData? responseData) {
        _respondToPlatformMessage(responseId, responseData);
      });
    }
  }

  /// Set the debug name associated with this platform dispatcher's root
  /// isolate.
  ///
  /// Normally debug names are automatically generated from the Dart port, entry
  /// point, and source file. For example: `main.dart$main-1234`.
  ///
  /// This can be combined with flutter tools `--isolate-filter` flag to debug
  /// specific root isolates. For example: `flutter attach --isolate-filter=[name]`.
  /// Note that this does not rename any child isolates of the root.
  void setIsolateDebugName(String name) => _setIsolateDebugName(name);

  @Native<Void Function(Handle)>(symbol: 'PlatformConfigurationNativeApi::SetIsolateDebugName')
  external static void _setIsolateDebugName(String name);

  /// Requests the Dart VM to adjusts the GC heuristics based on the requested `performance_mode`.
  ///
  /// This operation is a no-op of web. The request to change a performance may be ignored by the
  /// engine or not resolve in a predictable way.
  ///
  /// See [DartPerformanceMode] for more information on individual performance modes.
  void requestDartPerformanceMode(DartPerformanceMode mode) {
    _requestDartPerformanceMode(mode.index);
  }

  @Native<Int Function(Int)>(symbol: 'PlatformConfigurationNativeApi::RequestDartPerformanceMode')
  external static int _requestDartPerformanceMode(int mode);

  /// The embedder can specify data that the isolate can request synchronously
  /// on launch. This accessor fetches that data.
  ///
  /// This data is persistent for the duration of the Flutter application and is
  /// available even after isolate restarts. Because of this lifecycle, the size
  /// of this data must be kept to a minimum.
  ///
  /// For asynchronous communication between the embedder and isolate, a
  /// platform channel may be used.
  ByteData? getPersistentIsolateData() => _getPersistentIsolateData();

  @Native<Handle Function()>(symbol: 'PlatformConfigurationNativeApi::GetPersistentIsolateData')
  external static ByteData? _getPersistentIsolateData();

  /// Requests that, at the next appropriate opportunity, the [onBeginFrame] and
  /// [onDrawFrame] callbacks be invoked.
  ///
  /// See also:
  ///
  ///  * [SchedulerBinding], the Flutter framework class which manages the
  ///    scheduling of frames.
  ///  * [scheduleWarmUpFrame], which should only be used to schedule warm up
  ///    frames.
  void scheduleFrame() => _scheduleFrame();

  @Native<Void Function()>(symbol: 'PlatformConfigurationNativeApi::ScheduleFrame')
  external static void _scheduleFrame();

  /// Schedule a frame to run as soon as possible, rather than waiting for the
  /// engine to request a frame in response to a system "Vsync" signal.
  ///
  /// The application can call this method as soon as it starts up so that the
  /// first frame (which is likely to be quite expensive) can start a few extra
  /// milliseconds earlier. Using it in other situations might lead to
  /// unintended results, such as screen tearing. Depending on platforms and
  /// situations, the warm up frame might or might not be actually rendered onto
  /// the screen.
  ///
  /// For more introduction to the warm up frame, see
  /// [SchedulerBinding.scheduleWarmUpFrame].
  ///
  /// This method uses the provided callbacks as the begin frame callback and
  /// the draw frame callback instead of [onBeginFrame] and [onDrawFrame].
  ///
  /// See also:
  ///
  ///  * [SchedulerBinding.scheduleWarmUpFrame], which uses this method, and
  ///    introduces the warm up frame in more details.
  ///  * [scheduleFrame], which schedules the frame at the next appropriate
  ///    opportunity and should be used to render regular frames.
  void scheduleWarmUpFrame({required VoidCallback beginFrame, required VoidCallback drawFrame}) {
    // We use timers here to ensure that microtasks flush in between.
    Timer.run(beginFrame);
    Timer.run(() {
      drawFrame();
      _endWarmUpFrame();
    });
  }

  @Native<Void Function()>(symbol: 'PlatformConfigurationNativeApi::EndWarmUpFrame')
  external static void _endWarmUpFrame();

  /// Additional accessibility features that may be enabled by the platform.
  AccessibilityFeatures get accessibilityFeatures => _configuration.accessibilityFeatures;

  /// A callback that is invoked when the value of [accessibilityFeatures]
  /// changes.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  VoidCallback? get onAccessibilityFeaturesChanged => _onAccessibilityFeaturesChanged;
  VoidCallback? _onAccessibilityFeaturesChanged;
  Zone _onAccessibilityFeaturesChangedZone = Zone.root;
  set onAccessibilityFeaturesChanged(VoidCallback? callback) {
    _onAccessibilityFeaturesChanged = callback;
    _onAccessibilityFeaturesChangedZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _updateAccessibilityFeatures(int values) {
    final AccessibilityFeatures newFeatures = AccessibilityFeatures._(values);
    final _PlatformConfiguration previousConfiguration = _configuration;
    if (newFeatures == previousConfiguration.accessibilityFeatures) {
      return;
    }
    _configuration = previousConfiguration.copyWith(accessibilityFeatures: newFeatures);
    _invoke(onPlatformConfigurationChanged, _onPlatformConfigurationChangedZone);
    _invoke(onAccessibilityFeaturesChanged, _onAccessibilityFeaturesChangedZone);
  }

  /// Change the retained semantics data about this platform dispatcher.
  ///
  /// If [semanticsEnabled] is true, the user has requested that this function
  /// be called whenever the semantic content of this platform dispatcher
  /// changes.
  ///
  /// In either case, this function disposes the given update, which means the
  /// semantics update cannot be used further.
  @Deprecated('''
    In a multi-view world, the platform dispatcher can no longer provide apis
    to update semantics since each view will host its own semantics tree.

    Semantics updates must be passed to an individual [FlutterView]. To update
    semantics, use PlatformDispatcher.instance.views to get a [FlutterView] and
    call `updateSemantics`.
  ''')
  void updateSemantics(SemanticsUpdate update) =>
      _updateSemantics(update as _NativeSemanticsUpdate);

  @Native<Void Function(Pointer<Void>)>(symbol: 'PlatformConfigurationNativeApi::UpdateSemantics')
  external static void _updateSemantics(_NativeSemanticsUpdate update);

  /// The system-reported default locale of the device.
  ///
  /// This establishes the language and formatting conventions that application
  /// should, if possible, use to render their user interface.
  ///
  /// This is the first locale selected by the user and is the user's primary
  /// locale (the locale the device UI is displayed in)
  ///
  /// This is equivalent to `locales.first`, except that it will provide an
  /// undefined (using the language tag "und") non-null locale if the [locales]
  /// list has not been set or is empty.
  Locale get locale => locales.isEmpty ? const Locale.fromSubtags() : locales.first;

  /// The full system-reported supported locales of the device.
  ///
  /// This establishes the language and formatting conventions that application
  /// should, if possible, use to render their user interface.
  ///
  /// The list is ordered in order of priority, with lower-indexed locales being
  /// preferred over higher-indexed ones. The first element is the primary
  /// [locale].
  ///
  /// The [onLocaleChanged] callback is called whenever this value changes.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this value changes.
  List<Locale> get locales => _configuration.locales;

  /// Performs the platform-native locale resolution.
  ///
  /// Each platform may return different results.
  ///
  /// If the platform fails to resolve a locale, then this will return null.
  ///
  /// This method returns synchronously and is a direct call to
  /// platform specific APIs without invoking method channels.
  Locale? computePlatformResolvedLocale(List<Locale> supportedLocales) {
    final List<String?> supportedLocalesData = <String?>[];
    for (final Locale locale in supportedLocales) {
      supportedLocalesData.add(locale.languageCode);
      supportedLocalesData.add(locale.countryCode);
      supportedLocalesData.add(locale.scriptCode);
    }

    final List<String> result = _computePlatformResolvedLocale(supportedLocalesData);

    if (result.isNotEmpty) {
      return Locale.fromSubtags(
        languageCode: result[0],
        countryCode: result[1] == '' ? null : result[1],
        scriptCode: result[2] == '' ? null : result[2],
      );
    }
    return null;
  }

  List<String> _computePlatformResolvedLocale(List<String?> supportedLocalesData) =>
      __computePlatformResolvedLocale(supportedLocalesData);

  @Native<Handle Function(Handle)>(
    symbol: 'PlatformConfigurationNativeApi::ComputePlatformResolvedLocale',
  )
  external static List<String> __computePlatformResolvedLocale(List<String?> supportedLocalesData);

  /// A callback that is invoked whenever [locale] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  VoidCallback? get onLocaleChanged => _onLocaleChanged;
  VoidCallback? _onLocaleChanged;
  Zone _onLocaleChangedZone = Zone.root;
  set onLocaleChanged(VoidCallback? callback) {
    _onLocaleChanged = callback;
    _onLocaleChangedZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _updateLocales(List<String> locales) {
    const int stringsPerLocale = 4;
    final int numLocales = locales.length ~/ stringsPerLocale;
    final _PlatformConfiguration previousConfiguration = _configuration;
    final List<Locale> newLocales = <Locale>[];
    bool localesDiffer = numLocales != previousConfiguration.locales.length;
    for (int localeIndex = 0; localeIndex < numLocales; localeIndex++) {
      final String countryCode = locales[localeIndex * stringsPerLocale + 1];
      final String scriptCode = locales[localeIndex * stringsPerLocale + 2];

      newLocales.add(
        Locale.fromSubtags(
          languageCode: locales[localeIndex * stringsPerLocale],
          countryCode: countryCode.isEmpty ? null : countryCode,
          scriptCode: scriptCode.isEmpty ? null : scriptCode,
        ),
      );
      if (!localesDiffer && newLocales[localeIndex] != previousConfiguration.locales[localeIndex]) {
        localesDiffer = true;
      }
    }
    if (!localesDiffer) {
      return;
    }
    _configuration = previousConfiguration.copyWith(locales: newLocales);
    _invoke(onPlatformConfigurationChanged, _onPlatformConfigurationChangedZone);
    _invoke(onLocaleChanged, _onLocaleChangedZone);
  }

  // Called from the engine, via hooks.dart
  String _localeClosure() => locale.toString();

  /// The lifecycle state immediately after dart isolate initialization.
  ///
  /// This property will not be updated as the lifecycle changes.
  ///
  /// It is used to initialize [SchedulerBinding.lifecycleState] at startup with
  /// any buffered lifecycle state events.
  String get initialLifecycleState {
    _initialLifecycleStateAccessed = true;
    return _initialLifecycleState;
  }

  late String _initialLifecycleState;

  /// Tracks if the initial state has been accessed. Once accessed, we will stop
  /// updating the [initialLifecycleState], as it is not the preferred way to
  /// access the state.
  bool _initialLifecycleStateAccessed = false;

  // Called from the engine, via hooks.dart
  void _updateInitialLifecycleState(String state) {
    // We do not update the state if the state has already been used to initialize
    // the lifecycleState.
    if (!_initialLifecycleStateAccessed) {
      _initialLifecycleState = state;
    }
  }

  /// The setting indicating whether time should always be shown in the 24-hour
  /// format.
  ///
  /// This option is used by [showTimePicker].
  bool get alwaysUse24HourFormat => _configuration.alwaysUse24HourFormat;

  /// The system-reported text scale.
  ///
  /// This establishes the text scaling factor to use when rendering text,
  /// according to the user's platform preferences.
  ///
  /// The [onTextScaleFactorChanged] callback is called whenever this value
  /// changes.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this value changes.
  double get textScaleFactor => _configuration.textScaleFactor;

  /// A callback that is invoked whenever [textScaleFactor] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  VoidCallback? get onTextScaleFactorChanged => _onTextScaleFactorChanged;
  VoidCallback? _onTextScaleFactorChanged;
  Zone _onTextScaleFactorChangedZone = Zone.root;
  set onTextScaleFactorChanged(VoidCallback? callback) {
    _onTextScaleFactorChanged = callback;
    _onTextScaleFactorChangedZone = Zone.current;
  }

  /// Whether the spell check service is supported on the current platform.
  ///
  /// This option is used by [EditableTextState] to define its
  /// [SpellCheckConfiguration] when a default spell check service
  /// is requested.
  bool get nativeSpellCheckServiceDefined => _nativeSpellCheckServiceDefined;
  bool _nativeSpellCheckServiceDefined = false;

  /// Whether showing system context menu is supported on the current platform.
  ///
  /// This option is used by [AdaptiveTextSelectionToolbar] to decide whether
  /// to show system context menu, or to fallback to the default Flutter context
  /// menu.
  bool get supportsShowingSystemContextMenu => _supportsShowingSystemContextMenu;
  bool _supportsShowingSystemContextMenu = false;

  /// Whether briefly displaying the characters as you type in obscured text
  /// fields is enabled in system settings.
  ///
  /// See also:
  ///
  ///  * [EditableText.obscureText], which when set to true hides the text in
  ///    the text field.
  bool get brieflyShowPassword => _brieflyShowPassword;
  bool _brieflyShowPassword = true;

  /// The setting indicating the current brightness mode of the host platform.
  /// If the platform has no preference, [platformBrightness] defaults to
  /// [Brightness.light].
  Brightness get platformBrightness => _configuration.platformBrightness;

  /// A callback that is invoked whenever [platformBrightness] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  VoidCallback? get onPlatformBrightnessChanged => _onPlatformBrightnessChanged;
  VoidCallback? _onPlatformBrightnessChanged;
  Zone _onPlatformBrightnessChangedZone = Zone.root;
  set onPlatformBrightnessChanged(VoidCallback? callback) {
    _onPlatformBrightnessChanged = callback;
    _onPlatformBrightnessChangedZone = Zone.current;
  }

  /// The setting indicating the current system font of the host platform.
  String? get systemFontFamily => _configuration.systemFontFamily;

  /// A callback that is invoked whenever [systemFontFamily] changes value.
  ///
  /// The framework invokes this callback in the same zone in which the callback
  /// was set.
  ///
  /// See also:
  ///
  ///  * [WidgetsBindingObserver], for a mechanism at the widgets layer to
  ///    observe when this callback is invoked.
  VoidCallback? get onSystemFontFamilyChanged => _onSystemFontFamilyChanged;
  VoidCallback? _onSystemFontFamilyChanged;
  Zone _onSystemFontFamilyChangedZone = Zone.root;
  set onSystemFontFamilyChanged(VoidCallback? callback) {
    _onSystemFontFamilyChanged = callback;
    _onSystemFontFamilyChangedZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _updateUserSettingsData(String jsonData) {
    final Map<String, Object?> data = json.decode(jsonData) as Map<String, Object?>;
    if (data.isEmpty) {
      return;
    }

    final double textScaleFactor = (data['textScaleFactor']! as num).toDouble();
    final bool alwaysUse24HourFormat = data['alwaysUse24HourFormat']! as bool;
    final bool? nativeSpellCheckServiceDefined = data['nativeSpellCheckServiceDefined'] as bool?;
    if (nativeSpellCheckServiceDefined != null) {
      _nativeSpellCheckServiceDefined = nativeSpellCheckServiceDefined;
    } else {
      _nativeSpellCheckServiceDefined = false;
    }

    final bool? supportsShowingSystemContextMenu =
        data['supportsShowingSystemContextMenu'] as bool?;
    if (supportsShowingSystemContextMenu != null) {
      _supportsShowingSystemContextMenu = supportsShowingSystemContextMenu;
    } else {
      _supportsShowingSystemContextMenu = false;
    }

    // This field is optional.
    final bool? brieflyShowPassword = data['brieflyShowPassword'] as bool?;
    if (brieflyShowPassword != null) {
      _brieflyShowPassword = brieflyShowPassword;
    }
    final Brightness platformBrightness = switch (data['platformBrightness']) {
      'dark' => Brightness.dark,
      'light' => Brightness.light,
      final Object? value => throw StateError('$value is not a valid platformBrightness.'),
    };
    final String? systemFontFamily = data['systemFontFamily'] as String?;
    final int? configurationId = data['configurationId'] as int?;
    final _PlatformConfiguration previousConfiguration = _configuration;
    final bool platformBrightnessChanged =
        previousConfiguration.platformBrightness != platformBrightness;
    final bool textScaleFactorChanged = previousConfiguration.textScaleFactor != textScaleFactor;
    final bool alwaysUse24HourFormatChanged =
        previousConfiguration.alwaysUse24HourFormat != alwaysUse24HourFormat;
    final bool systemFontFamilyChanged = previousConfiguration.systemFontFamily != systemFontFamily;
    if (!platformBrightnessChanged &&
        !textScaleFactorChanged &&
        !alwaysUse24HourFormatChanged &&
        !systemFontFamilyChanged &&
        configurationId == null) {
      return;
    }
    _configuration = previousConfiguration.copyWith(
      textScaleFactor: textScaleFactor,
      alwaysUse24HourFormat: alwaysUse24HourFormat,
      platformBrightness: platformBrightness,
      systemFontFamily: systemFontFamily,
      configurationId: configurationId,
    );
    _invoke(onPlatformConfigurationChanged, _onPlatformConfigurationChangedZone);
    if (textScaleFactorChanged) {
      _cachedFontSizes = null;
      _invoke(onTextScaleFactorChanged, _onTextScaleFactorChangedZone);
    }
    if (platformBrightnessChanged) {
      _invoke(onPlatformBrightnessChanged, _onPlatformBrightnessChangedZone);
    }
    if (systemFontFamilyChanged) {
      _invoke(onSystemFontFamilyChanged, _onSystemFontFamilyChangedZone);
    }
  }

  /// Whether the user has requested that updateSemantics be called when the
  /// semantic contents of a view changes.
  ///
  /// The [onSemanticsEnabledChanged] callback is called whenever this value
  /// changes.
  bool get semanticsEnabled => _configuration.semanticsEnabled;

  /// A callback that is invoked when the value of [semanticsEnabled] changes.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  VoidCallback? get onSemanticsEnabledChanged => _onSemanticsEnabledChanged;
  VoidCallback? _onSemanticsEnabledChanged;
  Zone _onSemanticsEnabledChangedZone = Zone.root;
  set onSemanticsEnabledChanged(VoidCallback? callback) {
    _onSemanticsEnabledChanged = callback;
    _onSemanticsEnabledChangedZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _updateSemanticsEnabled(bool enabled) {
    final _PlatformConfiguration previousConfiguration = _configuration;
    if (previousConfiguration.semanticsEnabled == enabled) {
      return;
    }
    _configuration = previousConfiguration.copyWith(semanticsEnabled: enabled);
    _invoke(onPlatformConfigurationChanged, _onPlatformConfigurationChangedZone);
    _invoke(onSemanticsEnabledChanged, _onSemanticsEnabledChangedZone);
  }

  /// A callback that is invoked whenever the user requests an action to be
  /// performed on a semantics node.
  ///
  /// This callback is used when the user expresses the action they wish to
  /// perform based on the semantics node supplied by updateSemantics.
  ///
  /// The framework invokes this callback in the same zone in which the
  /// callback was set.
  SemanticsActionEventCallback? get onSemanticsActionEvent => _onSemanticsActionEvent;
  SemanticsActionEventCallback? _onSemanticsActionEvent;
  Zone _onSemanticsActionEventZone = Zone.root;
  set onSemanticsActionEvent(SemanticsActionEventCallback? callback) {
    _onSemanticsActionEvent = callback;
    _onSemanticsActionEventZone = Zone.current;
  }

  // Called from the engine via hooks.dart.
  void _updateFrameData(int frameNumber) {
    final FrameData previous = _frameData;
    if (previous.frameNumber == frameNumber) {
      return;
    }
    _frameData = FrameData._(frameNumber: frameNumber);
    _invoke(onFrameDataChanged, _onFrameDataChangedZone);
  }

  /// The [FrameData] object for the current frame.
  FrameData get frameData => _frameData;
  FrameData _frameData = const FrameData._();

  /// A callback that is invoked when the window updates the [FrameData].
  VoidCallback? get onFrameDataChanged => _onFrameDataChanged;
  VoidCallback? _onFrameDataChanged;
  Zone _onFrameDataChangedZone = Zone.root;
  set onFrameDataChanged(VoidCallback? callback) {
    _onFrameDataChanged = callback;
    _onFrameDataChangedZone = Zone.current;
  }

  // Called from the engine, via hooks.dart
  void _dispatchSemanticsAction(int nodeId, int action, ByteData? args) {
    _invoke1<SemanticsActionEvent>(
      onSemanticsActionEvent,
      _onSemanticsActionEventZone,
      SemanticsActionEvent(
        type: SemanticsAction.fromIndex(action)!,
        nodeId: nodeId,
        viewId: 0, // TODO(goderbauer): Wire up the real view ID.
        arguments: args,
      ),
    );
  }

  ErrorCallback? _onError;
  Zone? _onErrorZone;

  /// A callback that is invoked when an unhandled error occurs in the root
  /// isolate.
  ///
  /// This callback must return `true` if it has handled the error. Otherwise,
  /// it must return `false` and a fallback mechanism such as printing to stderr
  /// will be used, as configured by the specific platform embedding via
  /// `Settings::unhandled_exception_callback`.
  ///
  /// The VM or the process may exit or become unresponsive after calling this
  /// callback. The callback will not be called for exceptions that cause the VM
  /// or process to terminate or become unresponsive before the callback can be
  /// invoked.
  ///
  /// This callback is not directly invoked by errors in child isolates of the
  /// root isolate. Programs that create new isolates must listen for errors on
  /// those isolates and forward the errors to the root isolate.
  ErrorCallback? get onError => _onError;
  set onError(ErrorCallback? callback) {
    _onError = callback;
    _onErrorZone = Zone.current;
  }

  bool _dispatchError(Object error, StackTrace stackTrace) {
    if (_onError == null) {
      return false;
    }
    assert(_onErrorZone != null);

    if (identical(_onErrorZone, Zone.current)) {
      return _onError!(error, stackTrace);
    } else {
      try {
        return _onErrorZone!.runBinary<bool, Object, StackTrace>(_onError!, error, stackTrace);
      } catch (e, s) {
        _onErrorZone!.handleUncaughtError(e, s);
        return false;
      }
    }
  }

  /// The route or path that the embedder requested when the application was
  /// launched.
  ///
  /// This will be the string "`/`" if no particular route was requested.
  ///
  /// ## Android
  ///
  /// On Android, calling
  /// [`FlutterView.setInitialRoute`](/javadoc/io/flutter/view/FlutterView.html#setInitialRoute-java.lang.String-)
  /// will set this value. The value must be set sufficiently early, i.e. before
  /// the [runApp] call is executed in Dart, for this to have any effect on the
  /// framework. The `createFlutterView` method in your `FlutterActivity`
  /// subclass is a suitable time to set the value. The application's
  /// `AndroidManifest.xml` file must also be updated to have a suitable
  /// [`<intent-filter>`](https://developer.android.com/guide/topics/manifest/intent-filter-element.html).
  ///
  /// ## iOS
  ///
  /// On iOS, calling
  /// [`FlutterViewController.setInitialRoute`](/ios-embedder/interface_flutter_view_controller.html#a7f269c2da73312f856d42611cc12a33f)
  /// will set this value. The value must be set sufficiently early, i.e. before
  /// the [runApp] call is executed in Dart, for this to have any effect on the
  /// framework. The `application:didFinishLaunchingWithOptions:` method is a
  /// suitable time to set this value.
  ///
  /// See also:
  ///
  ///  * [Navigator], a widget that handles routing.
  ///  * [SystemChannels.navigation], which handles subsequent navigation
  ///    requests from the embedder.
  String get defaultRouteName => _defaultRouteName();

  @Native<Handle Function()>(symbol: 'PlatformConfigurationNativeApi::DefaultRouteName')
  external static String _defaultRouteName();

  /// Computes the scaled font size from the given `unscaledFontSize`, according
  /// to the user's platform preferences.
  ///
  /// Many platforms allow users to scale text globally for better readability.
  /// Given the font size the app developer specified in logical pixels, this
  /// method converts it to the preferred font size (also in logical pixels) that
  /// accounts for platform-wide text scaling. The return value is always
  /// non-negative.
  ///
  /// The scaled value of the same font size input may change if the user changes
  /// the text scaling preference (in system settings for example). The
  /// [onTextScaleFactorChanged] callback can be used to monitor such changes.
  ///
  /// Instead of directly calling this method, applications should typically use
  /// [MediaQuery.textScalerOf] to retrive the scaled font size in a widget tree,
  /// so text in the app resizes properly when the text scaling preference
  /// changes.
  double scaleFontSize(double unscaledFontSize) {
    assert(unscaledFontSize >= 0);
    assert(unscaledFontSize.isFinite);

    if (textScaleFactor == 1.0) {
      return unscaledFontSize;
    }

    final int unscaledFloor = unscaledFontSize.floor();
    final int unscaledCeil = unscaledFontSize.ceil();
    if (unscaledFloor == unscaledCeil) {
      // No need to interpolate if the input value is an integer.
      return _scaleAndMemoize(unscaledFloor) ?? unscaledFontSize * textScaleFactor;
    }
    assert(
      unscaledCeil - unscaledFloor == 1,
      'Unexpected interpolation range: $unscaledFloor - $unscaledCeil.',
    );

    return switch ((_scaleAndMemoize(unscaledFloor), _scaleAndMemoize(unscaledCeil))) {
      (null, _) || (_, null) => unscaledFontSize * textScaleFactor,
      (final double lower, final double upper) =>
        lower + (upper - lower) * (unscaledFontSize - unscaledFloor),
    };
  }

  // The cache is cleared when the text scale factor changes.
  Map<int, double>? _cachedFontSizes;
  // This method returns null if an error is encountered.
  double? _scaleAndMemoize(int unscaledFontSize) {
    final int? configurationId = _configuration.configurationId;
    if (configurationId == null) {
      // The platform uses linear scaling, or the platform hasn't sent us a
      // configuration yet.
      return null;
    }
    final double? cachedValue = _cachedFontSizes?[unscaledFontSize];
    if (cachedValue != null) {
      assert(cachedValue >= 0);
      return cachedValue;
    }

    final double unscaledFontSizeDouble = unscaledFontSize.toDouble();
    final double fontSize = PlatformDispatcher._getScaledFontSize(
      unscaledFontSizeDouble,
      configurationId,
    );
    if (fontSize >= 0) {
      return (_cachedFontSizes ??= <int, double>{})[unscaledFontSize] = fontSize;
    }
    switch (fontSize) {
      case -1:
        // Invalid configuration id. This error can be unrecoverable as the
        // _getScaledFontSize function can be destructive.
        assert(false, 'Flutter Error: incorrect configuration id: $configurationId.');
      case final double errorCode:
        assert(false, 'Unknown error: GetScaledFontSize failed with $errorCode.');
    }
    return null;
  }

  // Calls the platform's text scaling implementation to scale the given
  // `unscaledFontSize`.
  //
  // The `configurationId` parameter tells the embedder which platform
  // configuration to use for computing the scaled font size. When the user
  // changes the platform configuration, the configuration data will first be
  // made available on the platform thread before being dispatched asynchronously
  // to the Flutter UI thread. Since this call is synchronous, without this
  // identifier, it could call into the embber who's using a newer configuration
  // that Flutter has not received yet. The `configurationId` parameter must be
  // the lastest configuration id received from the platform
  // (`_configuration.configurationId`). Using an incorrect id could result in
  // an unrecoverable error.
  //
  // Currently this is only implemented on newer versions of Android (SDK level
  // 34, using the `TypedValue#applyDimension` API). Platforms that do not have
  // the capability will never send a `configurationId` to [PlatformDispatcher],
  // and should not call this method. This method returns -1 when the specified
  // configurationId does not match any configuration.
  @Native<Double Function(Double, Int)>(symbol: 'PlatformConfigurationNativeApi::GetScaledFontSize')
  external static double _getScaledFontSize(double unscaledFontSize, int configurationId);
}

/// Configuration of the platform.
///
/// Immutable class (but can't use @immutable in dart:ui)
class _PlatformConfiguration {
  const _PlatformConfiguration({
    this.accessibilityFeatures = const AccessibilityFeatures._(0),
    this.alwaysUse24HourFormat = false,
    this.semanticsEnabled = false,
    this.platformBrightness = Brightness.light,
    this.textScaleFactor = 1.0,
    this.locales = const <Locale>[],
    this.defaultRouteName,
    this.systemFontFamily,
    this.configurationId,
  });

  _PlatformConfiguration copyWith({
    AccessibilityFeatures? accessibilityFeatures,
    bool? alwaysUse24HourFormat,
    bool? semanticsEnabled,
    Brightness? platformBrightness,
    double? textScaleFactor,
    List<Locale>? locales,
    String? defaultRouteName,
    String? systemFontFamily,
    int? configurationId,
  }) {
    return _PlatformConfiguration(
      accessibilityFeatures: accessibilityFeatures ?? this.accessibilityFeatures,
      alwaysUse24HourFormat: alwaysUse24HourFormat ?? this.alwaysUse24HourFormat,
      semanticsEnabled: semanticsEnabled ?? this.semanticsEnabled,
      platformBrightness: platformBrightness ?? this.platformBrightness,
      textScaleFactor: textScaleFactor ?? this.textScaleFactor,
      locales: locales ?? this.locales,
      defaultRouteName: defaultRouteName ?? this.defaultRouteName,
      systemFontFamily: systemFontFamily ?? this.systemFontFamily,
      configurationId: configurationId ?? this.configurationId,
    );
  }

  /// Additional accessibility features that may be enabled by the platform.
  final AccessibilityFeatures accessibilityFeatures;

  /// The setting indicating whether time should always be shown in the 24-hour
  /// format.
  final bool alwaysUse24HourFormat;

  /// Whether the user has requested that updateSemantics be called when the
  /// semantic contents of a view changes.
  final bool semanticsEnabled;

  /// The setting indicating the current brightness mode of the host platform.
  /// If the platform has no preference, [platformBrightness] defaults to
  /// [Brightness.light].
  final Brightness platformBrightness;

  /// The system-reported text scale.
  final double textScaleFactor;

  /// The full system-reported supported locales of the device.
  final List<Locale> locales;

  /// The route or path that the embedder requested when the application was
  /// launched.
  final String? defaultRouteName;

  /// The system-reported default font family.
  final String? systemFontFamily;

  /// A unique identifier for this [_PlatformConfiguration].
  ///
  /// This unique identifier is optionally assigned by the platform embedder.
  /// Dart code that runs on the Flutter UI thread and synchronously invokes
  /// platform APIs can use this identifier to tell the embedder to use the
  /// configuration that matches the current [_PlatformConfiguration] in
  /// dart:ui. See the [_getScaledFontSize] function for an example.
  ///
  /// This field's nullability also indicates whether the platform supports
  /// nonlinear text scaling (as it's the only feature that requires synchronous
  /// invocation of platform APIs). This field is always null if the platform
  /// does not use nonlinear text scaling, or when dart:ui has not received any
  /// configuration updates from the embedder yet. The _getScaledFontSize
  /// function should not be called in either case.
  final int? configurationId;
}

/// An immutable view configuration.
class _ViewConfiguration {
  const _ViewConfiguration({
    this.devicePixelRatio = 1.0,
    this.size = Size.zero,
    this.viewInsets = ViewPadding.zero,
    this.viewPadding = ViewPadding.zero,
    this.systemGestureInsets = ViewPadding.zero,
    this.padding = ViewPadding.zero,
    this.gestureSettings = const GestureSettings(),
    this.displayFeatures = const <DisplayFeature>[],
    this.displayId = 0,
  });

  /// The identifier for a display for this view, in
  /// [PlatformDispatcher._displays].
  final int displayId;

  /// The pixel density of the output surface.
  final double devicePixelRatio;

  /// The size requested for the view in physical pixels.
  final Size size;

  /// The number of physical pixels on each side of the display rectangle into
  /// which the view can render, but over which the operating system will likely
  /// place system UI, such as the keyboard, that fully obscures any content.
  ///
  /// The relationship between this [viewInsets], [viewPadding], and [padding]
  /// are described in more detail in the documentation for [FlutterView].
  final ViewPadding viewInsets;

  /// The number of physical pixels on each side of the display rectangle into
  /// which the view can render, but which may be partially obscured by system
  /// UI (such as the system notification area), or physical intrusions in
  /// the display (e.g. overscan regions on television screens or phone sensor
  /// housings).
  ///
  /// Unlike [padding], this value does not change relative to [viewInsets].
  /// For example, on an iPhone X, it will not change in response to the soft
  /// keyboard being visible or hidden, whereas [padding] will.
  ///
  /// The relationship between this [viewInsets], [viewPadding], and [padding]
  /// are described in more detail in the documentation for [FlutterView].
  final ViewPadding viewPadding;

  /// The number of physical pixels on each side of the display rectangle into
  /// which the view can render, but where the operating system will consume
  /// input gestures for the sake of system navigation.
  ///
  /// For example, an operating system might use the vertical edges of the
  /// screen, where swiping inwards from the edges takes users backward
  /// through the history of screens they previously visited.
  final ViewPadding systemGestureInsets;

  /// The number of physical pixels on each side of the display rectangle into
  /// which the view can render, but which may be partially obscured by system
  /// UI (such as the system notification area), or physical intrusions in
  /// the display (e.g. overscan regions on television screens or phone sensor
  /// housings).
  ///
  /// The relationship between this [viewInsets], [viewPadding], and [padding]
  /// are described in more detail in the documentation for [FlutterView].
  final ViewPadding padding;

  /// Additional configuration for touch gestures performed on this view.
  ///
  /// For example, the touch slop defined in physical pixels may be provided
  /// by the gesture settings and should be preferred over the framework
  /// touch slop constant.
  final GestureSettings gestureSettings;

  /// Areas of the display that are obstructed by hardware features.
  ///
  /// This list is populated only on Android. If the device has no display
  /// features, this list is empty.
  ///
  /// The coordinate space in which the [DisplayFeature.bounds] are defined spans
  /// across the screens currently in use. This means that the space between the screens
  /// is virtually part of the Flutter view space, with the [DisplayFeature.bounds]
  /// of the display feature as an obstructed area. The [DisplayFeature.type] can
  /// be used to determine if this display feature obstructs the screen or not.
  /// For example, [DisplayFeatureType.hinge] and [DisplayFeatureType.cutout] both
  /// obstruct the display, while [DisplayFeatureType.fold] is a crease in the display.
  ///
  /// Folding [DisplayFeature]s like the [DisplayFeatureType.hinge] and
  /// [DisplayFeatureType.fold] also have a [DisplayFeature.state] which can be
  /// used to determine the posture the device is in.
  final List<DisplayFeature> displayFeatures;

  @override
  String toString() {
    return '$runtimeType[size: $size]';
  }
}

/// Various important time points in the lifetime of a frame.
///
/// [FrameTiming] records a timestamp of each phase for performance analysis.
enum FramePhase {
  /// The timestamp of the vsync signal given by the operating system.
  ///
  /// See also [FrameTiming.vsyncOverhead].
  vsyncStart,

  /// When the UI thread starts building a frame.
  ///
  /// See also [FrameTiming.buildDuration].
  buildStart,

  /// When the UI thread finishes building a frame.
  ///
  /// See also [FrameTiming.buildDuration].
  buildFinish,

  /// When the raster thread starts rasterizing a frame.
  ///
  /// See also [FrameTiming.rasterDuration].
  rasterStart,

  /// When the raster thread finishes rasterizing a frame.
  ///
  /// See also [FrameTiming.rasterDuration].
  rasterFinish,

  /// When the raster thread finished rasterizing a frame in wall-time.
  ///
  /// This is useful for correlating time raster finish time with the system
  /// clock to integrate with other profiling tools.
  rasterFinishWallTime,
}

enum _FrameTimingInfo {
  /// The number of engine layers cached in the raster cache during the frame.
  layerCacheCount,

  /// The number of bytes used to cache engine layers during the frame.
  layerCacheBytes,

  /// The number of picture layers cached in the raster cache during the frame.
  pictureCacheCount,

  /// The number of bytes used to cache pictures during the frame.
  pictureCacheBytes,

  /// The frame number of the frame.
  frameNumber,
}

/// Time-related performance metrics of a frame.
///
/// If you're using the whole Flutter framework, please use
/// [SchedulerBinding.addTimingsCallback] to get this. It's preferred over using
/// [PlatformDispatcher.onReportTimings] directly because
/// [SchedulerBinding.addTimingsCallback] allows multiple callbacks. If
/// [SchedulerBinding] is unavailable, then see [PlatformDispatcher.onReportTimings]
/// for how to get this.
///
/// The metrics in debug mode (`flutter run` without any flags) may be very
/// different from those in profile and release modes due to the debug overhead.
/// Therefore it's recommended to only monitor and analyze performance metrics
/// in profile and release modes.
class FrameTiming {
  /// Construct [FrameTiming] with raw timestamps in microseconds.
  ///
  /// This constructor is used for unit test only. Real [FrameTiming]s should
  /// be retrieved from [PlatformDispatcher.onReportTimings].
  ///
  /// If the [frameNumber] is not provided, it defaults to `-1`.
  factory FrameTiming({
    required int vsyncStart,
    required int buildStart,
    required int buildFinish,
    required int rasterStart,
    required int rasterFinish,
    required int rasterFinishWallTime,
    int layerCacheCount = 0,
    int layerCacheBytes = 0,
    int pictureCacheCount = 0,
    int pictureCacheBytes = 0,
    int frameNumber = -1,
  }) {
    return FrameTiming._(<int>[
      vsyncStart,
      buildStart,
      buildFinish,
      rasterStart,
      rasterFinish,
      rasterFinishWallTime,
      layerCacheCount,
      layerCacheBytes,
      pictureCacheCount,
      pictureCacheBytes,
      frameNumber,
    ]);
  }

  /// Construct [FrameTiming] with raw timestamps in microseconds.
  ///
  /// List [timestamps] must have the same number of elements as
  /// [FramePhase.values].
  ///
  /// This constructor is usually only called by the Flutter engine, or a test.
  /// To get the [FrameTiming] of your app, see [PlatformDispatcher.onReportTimings].
  FrameTiming._(this._data) : assert(_data.length == _dataLength);

  static final int _dataLength = FramePhase.values.length + _FrameTimingInfo.values.length;

  /// This is a raw timestamp in microseconds from some epoch. The epoch in all
  /// [FrameTiming] is the same, but it may not match [DateTime]'s epoch.
  int timestampInMicroseconds(FramePhase phase) => _data[phase.index];

  Duration _rawDuration(FramePhase phase) => Duration(microseconds: _data[phase.index]);

  int _rawInfo(_FrameTimingInfo info) => _data[FramePhase.values.length + info.index];

  /// The duration to build the frame on the UI thread.
  ///
  /// The build starts approximately when [PlatformDispatcher.onBeginFrame] is
  /// called. The [Duration] in the [PlatformDispatcher.onBeginFrame] callback
  /// is exactly the `Duration(microseconds:
  /// timestampInMicroseconds(FramePhase.buildStart))`.
  ///
  /// The build finishes when [FlutterView.render] is called.
  ///
  /// {@template dart.ui.FrameTiming.fps_smoothness_milliseconds}
  /// To ensure smooth animations of X fps, this should not exceed 1000/X
  /// milliseconds.
  /// {@endtemplate}
  /// {@template dart.ui.FrameTiming.fps_milliseconds}
  /// That's about 16ms for 60fps, and 8ms for 120fps.
  /// {@endtemplate}
  Duration get buildDuration =>
      _rawDuration(FramePhase.buildFinish) - _rawDuration(FramePhase.buildStart);

  /// The duration to rasterize the frame on the raster thread.
  ///
  /// {@macro dart.ui.FrameTiming.fps_smoothness_milliseconds}
  /// {@macro dart.ui.FrameTiming.fps_milliseconds}
  Duration get rasterDuration =>
      _rawDuration(FramePhase.rasterFinish) - _rawDuration(FramePhase.rasterStart);

  /// The duration between receiving the vsync signal and starting building the
  /// frame.
  Duration get vsyncOverhead =>
      _rawDuration(FramePhase.buildStart) - _rawDuration(FramePhase.vsyncStart);

  /// The timespan between vsync start and raster finish.
  ///
  /// To achieve the lowest latency on an X fps display, this should not exceed
  /// 1000/X milliseconds.
  /// {@macro dart.ui.FrameTiming.fps_milliseconds}
  ///
  /// See also [vsyncOverhead], [buildDuration] and [rasterDuration].
  Duration get totalSpan =>
      _rawDuration(FramePhase.rasterFinish) - _rawDuration(FramePhase.vsyncStart);

  /// The number of layers stored in the raster cache during the frame.
  ///
  /// See also [layerCacheBytes], [pictureCacheCount] and [pictureCacheBytes].
  int get layerCacheCount => _rawInfo(_FrameTimingInfo.layerCacheCount);

  /// The number of bytes of image data used to cache layers during the frame.
  ///
  /// See also [layerCacheCount], [layerCacheMegabytes], [pictureCacheCount] and [pictureCacheBytes].
  int get layerCacheBytes => _rawInfo(_FrameTimingInfo.layerCacheBytes);

  /// The number of megabytes of image data used to cache layers during the frame.
  ///
  /// See also [layerCacheCount], [layerCacheBytes], [pictureCacheCount] and [pictureCacheBytes].
  double get layerCacheMegabytes => layerCacheBytes / 1024.0 / 1024.0;

  /// The number of pictures stored in the raster cache during the frame.
  ///
  /// See also [layerCacheCount], [layerCacheBytes] and [pictureCacheBytes].
  int get pictureCacheCount => _rawInfo(_FrameTimingInfo.pictureCacheCount);

  /// The number of bytes of image data used to cache pictures during the frame.
  ///
  /// See also [layerCacheCount], [layerCacheBytes], [pictureCacheCount] and [pictureCacheMegabytes].
  int get pictureCacheBytes => _rawInfo(_FrameTimingInfo.pictureCacheBytes);

  /// The number of megabytes of image data used to cache pictures during the frame.
  ///
  /// See also [layerCacheCount], [layerCacheBytes], [pictureCacheCount] and [pictureCacheBytes].
  double get pictureCacheMegabytes => pictureCacheBytes / 1024.0 / 1024.0;

  /// The frame key associated with this frame measurement.
  int get frameNumber => _data.last;

  final List<int> _data; // some elements in microseconds, some in bytes, some are counts

  String _formatMS(Duration duration) => '${duration.inMicroseconds * 0.001}ms';

  @override
  String toString() {
    return '$runtimeType(buildDuration: ${_formatMS(buildDuration)}, '
        'rasterDuration: ${_formatMS(rasterDuration)}, '
        'vsyncOverhead: ${_formatMS(vsyncOverhead)}, '
        'totalSpan: ${_formatMS(totalSpan)}, '
        'layerCacheCount: $layerCacheCount, '
        'layerCacheBytes: $layerCacheBytes, '
        'pictureCacheCount: $pictureCacheCount, '
        'pictureCacheBytes: $pictureCacheBytes, '
        'frameNumber: ${_data.last})';
  }
}

/// States that an application can be in once it is running.
///
/// States not supported on a platform will be synthesized by the framework when
/// transitioning between states which are supported, so that all
/// implementations share the same state machine.
///
/// The initial value for the state is the [detached] state, updated to the
/// current state (usually [resumed]) as soon as the first lifecycle update is
/// received from the platform.
///
/// For historical and name collision reasons, Flutter's application state names
/// do not correspond one to one with the state names on all platforms. On
/// Android, for instance, when the OS calls
/// [`Activity.onPause`](https://developer.android.com/reference/android/app/Activity#onPause()),
/// Flutter will enter the [inactive] state, but when Android calls
/// [`Activity.onStop`](https://developer.android.com/reference/android/app/Activity#onStop()),
/// Flutter enters the [paused] state. See the individual state's documentation
/// for descriptions of what they mean on each platform.
///
/// The current application state can be obtained from
/// [SchedulerBinding.instance.lifecycleState], and changes to the state can be
/// observed by creating an [AppLifecycleListener], or by using a
/// [WidgetsBindingObserver] by overriding the
/// [WidgetsBindingObserver.didChangeAppLifecycleState] method.
///
/// Applications should not rely on always receiving all possible notifications.
///
/// For example, if the application is killed with a task manager, a kill
/// signal, the user pulls the power from the device, or there is a rapid
/// unscheduled disassembly of the device, no notification will be sent before
/// the application is suddenly terminated, and some states may be skipped.
///
/// See also:
///
/// * [AppLifecycleListener], an object used observe the lifecycle state that
///   provides state transition callbacks.
/// * [WidgetsBindingObserver], for a mechanism to observe the lifecycle state
///   from the widgets layer.
/// * iOS's [UIKit activity
///   lifecycle](https://developer.apple.com/documentation/uikit/app_and_environment/managing_your_app_s_life_cycle?language=objc)
///   documentation.
/// * Android's [activity
///   lifecycle](https://developer.android.com/guide/components/activities/activity-lifecycle)
///   documentation.
/// * macOS's [AppKit activity
///   lifecycle](https://developer.apple.com/documentation/appkit/nsapplicationdelegate?language=objc)
///   documentation.
enum AppLifecycleState {
  /// The application is still hosted by a Flutter engine but is detached from
  /// any host views.
  ///
  /// The application defaults to this state before it initializes, and can be
  /// in this state (applicable on Android, iOS, and web) after all views have been
  /// detached.
  ///
  /// When the application is in this state, the engine is running without a
  /// view.
  ///
  /// This state is only entered on iOS, Android, and web, although on all platforms
  /// it is the default state before the application begins running.
  detached,

  /// On all platforms, this state indicates that the application is in the
  /// default running mode for a running application that has input focus and is
  /// visible.
  ///
  /// On Android, this state corresponds to the Flutter host view having focus
  /// ([`Activity.onWindowFocusChanged`](https://developer.android.com/reference/android/app/Activity#onWindowFocusChanged(boolean))
  /// was called with true) while in Android's "resumed" state. It is possible
  /// for the Flutter app to be in the [inactive] state while still being in
  /// Android's
  /// ["onResume"](https://developer.android.com/guide/components/activities/activity-lifecycle)
  /// state if the app has lost focus
  /// ([`Activity.onWindowFocusChanged`](https://developer.android.com/reference/android/app/Activity#onWindowFocusChanged(boolean))
  /// was called with false), but hasn't had
  /// [`Activity.onPause`](https://developer.android.com/reference/android/app/Activity#onPause())
  /// called on it.
  ///
  /// On iOS and macOS, this corresponds to the app running in the foreground
  /// active state.
  resumed,

  /// At least one view of the application is visible, but none have input
  /// focus. The application is otherwise running normally.
  ///
  /// On non-web desktop platforms, this corresponds to an application that is
  /// not in the foreground, but still has visible windows.
  ///
  /// On the web, this corresponds to an application that is running in a
  /// window or tab that does not have input focus.
  ///
  /// On iOS and macOS, this state corresponds to the Flutter host view running in the
  /// foreground inactive state. Apps transition to this state when in a phone
  /// call, when responding to a TouchID request, when entering the app switcher
  /// or the control center, or when the UIViewController hosting the Flutter
  /// app is transitioning.
  ///
  /// On Android, this corresponds to the Flutter host view running in Android's
  /// paused state (i.e.
  /// [`Activity.onPause`](https://developer.android.com/reference/android/app/Activity#onPause())
  /// has been called), or in Android's "resumed" state (i.e.
  /// [`Activity.onResume`](https://developer.android.com/reference/android/app/Activity#onResume())
  /// has been called) but does not have window focus. Examples of when apps
  /// transition to this state include when the app is partially obscured or
  /// another activity is focused, a app running in a split screen that isn't
  /// the current app, an app interrupted by a phone call, a picture-in-picture
  /// app, a system dialog, another view. It will also be inactive when the
  /// notification window shade is down, or the application switcher is visible.
  ///
  /// On Android and iOS, apps in this state should assume that they may be
  /// [hidden] and [paused] at any time.
  inactive,

  /// All views of an application are hidden, either because the application is
  /// about to be paused (on iOS and Android), or because it has been minimized
  /// or placed on a desktop that is no longer visible (on non-web desktop), or
  /// is running in a window or tab that is no longer visible (on the web).
  ///
  /// On iOS and Android, in order to keep the state machine the same on all
  /// platforms, a transition to this state is synthesized before the [paused]
  /// state is entered when coming from [inactive], and before the [inactive]
  /// state is entered when coming from [paused]. This allows cross-platform
  /// implementations that want to know when an app is conceptually "hidden" to
  /// only write one handler.
  hidden,

  /// The application is not currently visible to the user, and not responding
  /// to user input.
  ///
  /// When the application is in this state, the engine will not call the
  /// [PlatformDispatcher.onBeginFrame] and [PlatformDispatcher.onDrawFrame]
  /// callbacks.
  ///
  /// This state is only entered on iOS and Android.
  paused,
}

/// The possible responses to a request to exit the application.
///
/// The request is typically responded to by creating an [AppLifecycleListener]
/// and supplying an [AppLifecycleListener.onExitRequested] callback, or by
/// overriding [WidgetsBindingObserver.didRequestAppExit].
enum AppExitResponse {
  /// Exiting the application can proceed.
  exit,

  /// Cancel the exit: do not exit the application.
  cancel,
}

/// The type of application exit to perform when calling
/// [ServicesBinding.exitApplication].
enum AppExitType {
  /// Requests that the application start an orderly exit, sending a request
  /// back to the framework through the [WidgetsBinding]. If that responds
  /// with [AppExitResponse.exit], then proceed with the same steps as a
  /// [required] exit. If that responds with [AppExitResponse.cancel], then the
  /// exit request is canceled and the application continues executing normally.
  cancelable,

  /// A non-cancelable orderly exit request. The engine will shut down the
  /// engine and call the native UI toolkit's exit API.
  ///
  /// If you need an even faster and more dangerous exit, then call `dart:io`'s
  /// `exit()` directly, and even the native toolkit's exit API won't be called.
  /// This is quite dangerous, though, since it's possible that the engine will
  /// crash because it hasn't been properly shut down, causing the app to crash
  /// on exit.
  required,
}

/// A representation of distances for each of the four edges of a rectangle,
/// used to encode the view insets and padding that applications should place
/// around their user interface, as exposed by [FlutterView.viewInsets] and
/// [FlutterView.padding]. View insets and padding are preferably read via
/// [MediaQuery.of].
///
/// For a generic class that represents distances around a rectangle, see the
/// [EdgeInsets] class.
///
/// See also:
///
///  * [WidgetsBindingObserver], for a widgets layer mechanism to receive
///    notifications when the padding changes.
///  * [MediaQuery.of], for the preferred mechanism for accessing these values.
///  * [Scaffold], which automatically applies the padding in material design
///    applications.
class ViewPadding {
  const ViewPadding._({
    required this.left,
    required this.top,
    required this.right,
    required this.bottom,
  });

  /// The distance from the left edge to the first unpadded pixel, in physical pixels.
  final double left;

  /// The distance from the top edge to the first unpadded pixel, in physical pixels.
  final double top;

  /// The distance from the right edge to the first unpadded pixel, in physical pixels.
  final double right;

  /// The distance from the bottom edge to the first unpadded pixel, in physical pixels.
  final double bottom;

  /// A view padding that has zeros for each edge.
  static const ViewPadding zero = ViewPadding._(left: 0.0, top: 0.0, right: 0.0, bottom: 0.0);

  @override
  String toString() {
    return 'ViewPadding(left: $left, top: $top, right: $right, bottom: $bottom)';
  }
}

/// Deprecated. Will be removed in a future version of Flutter.
///
/// Use [ViewPadding] instead.
@Deprecated(
  'Use ViewPadding instead. '
  'This feature was deprecated after v3.8.0-14.0.pre.',
)
typedef WindowPadding = ViewPadding;

/// Immutable layout constraints for [FlutterView]s.
///
/// Similar to [BoxConstraints], a [Size] respects a [ViewConstraints] if, and
/// only if, all of the following relations hold:
///
/// * [minWidth] <= [Size.width] <= [maxWidth]
/// * [minHeight] <= [Size.height] <= [maxHeight]
///
/// The constraints themselves must satisfy these relations:
///
/// * 0.0 <= [minWidth] <= [maxWidth] <= [double.infinity]
/// * 0.0 <= [minHeight] <= [maxHeight] <= [double.infinity]
///
/// For each constraint, [double.infinity] is a legal value.
///
/// For a generic class that represents these kind of constraints, see the
/// [BoxConstraints] class.
class ViewConstraints {
  /// Creates view constraints with the given constraints.
  const ViewConstraints({
    this.minWidth = 0.0,
    this.maxWidth = double.infinity,
    this.minHeight = 0.0,
    this.maxHeight = double.infinity,
  });

  /// Creates view constraints that is respected only by the given size.
  ViewConstraints.tight(Size size)
    : minWidth = size.width,
      maxWidth = size.width,
      minHeight = size.height,
      maxHeight = size.height;

  /// The minimum width that satisfies the constraints.
  final double minWidth;

  /// The maximum width that satisfies the constraints.
  ///
  /// Might be [double.infinity].
  final double maxWidth;

  /// The minimum height that satisfies the constraints.
  final double minHeight;

  /// The maximum height that satisfies the constraints.
  ///
  /// Might be [double.infinity].
  final double maxHeight;

  /// Whether the given size satisfies the constraints.
  bool isSatisfiedBy(Size size) {
    return (minWidth <= size.width) &&
        (size.width <= maxWidth) &&
        (minHeight <= size.height) &&
        (size.height <= maxHeight);
  }

  /// Whether there is exactly one size that satisfies the constraints.
  bool get isTight => minWidth >= maxWidth && minHeight >= maxHeight;

  /// Scales each constraint parameter by the inverse of the given factor.
  ViewConstraints operator /(double factor) {
    return ViewConstraints(
      minWidth: minWidth / factor,
      maxWidth: maxWidth / factor,
      minHeight: minHeight / factor,
      maxHeight: maxHeight / factor,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ViewConstraints &&
        other.minWidth == minWidth &&
        other.maxWidth == maxWidth &&
        other.minHeight == minHeight &&
        other.maxHeight == maxHeight;
  }

  @override
  int get hashCode => Object.hash(minWidth, maxWidth, minHeight, maxHeight);

  @override
  String toString() {
    if (minWidth == double.infinity && minHeight == double.infinity) {
      return 'ViewConstraints(biggest)';
    }
    if (minWidth == 0 &&
        maxWidth == double.infinity &&
        minHeight == 0 &&
        maxHeight == double.infinity) {
      return 'ViewConstraints(unconstrained)';
    }
    String describe(double min, double max, String dim) {
      if (min == max) {
        return '$dim=${min.toStringAsFixed(1)}';
      }
      return '${min.toStringAsFixed(1)}<=$dim<=${max.toStringAsFixed(1)}';
    }

    final String width = describe(minWidth, maxWidth, 'w');
    final String height = describe(minHeight, maxHeight, 'h');
    return 'ViewConstraints($width, $height)';
  }
}

/// Area of the display that may be obstructed by a hardware feature.
///
/// This is populated only on Android.
///
/// The [bounds] are measured in logical pixels. On devices with two screens the
/// coordinate system starts with (0,0) in the top-left corner of the left or top screen
/// and expands to include both screens and the visual space between them.
///
/// The [type] describes the behaviour and if [DisplayFeature] obstructs the display.
/// For example, [DisplayFeatureType.hinge] and [DisplayFeatureType.cutout] both obstruct the display,
/// while [DisplayFeatureType.fold] does not.
///
/// ![Device with a hinge display feature](https://flutter.github.io/assets-for-api-docs/assets/hardware/display_feature_hinge.png)
///
/// ![Device with a fold display feature](https://flutter.github.io/assets-for-api-docs/assets/hardware/display_feature_fold.png)
///
/// ![Device with a cutout display feature](https://flutter.github.io/assets-for-api-docs/assets/hardware/display_feature_cutout.png)
///
/// The [state] contains information about the posture for foldable features
/// ([DisplayFeatureType.hinge] and [DisplayFeatureType.fold]). The posture is
/// the shape of the display, for example [DisplayFeatureState.postureFlat] or
/// [DisplayFeatureState.postureHalfOpened]. For [DisplayFeatureType.cutout],
/// the state is not used and has the [DisplayFeatureState.unknown] value.
class DisplayFeature {
  // TODO(matanlurey): have original authors document; see https://github.com/flutter/flutter/issues/151917.
  // ignore: public_member_api_docs
  const DisplayFeature({required this.bounds, required this.type, required this.state})
    : assert(
        !identical(type, DisplayFeatureType.cutout) ||
            identical(state, DisplayFeatureState.unknown),
      );

  /// The area of the flutter view occupied by this display feature, measured in logical pixels.
  ///
  /// On devices with two screens, the Flutter view spans from the top-left corner
  /// of the left or top screen to the bottom-right corner of the right or bottom screen,
  /// including the visual area occupied by any display feature. Bounds of display
  /// features are reported in this coordinate system.
  ///
  /// For example, on a dual screen device in portrait mode:
  ///
  /// * [Rect.left] gives you the size of left screen, in logical pixels.
  /// * [Rect.right] gives you the size of the left screen + the hinge width.
  final Rect bounds;

  /// Type of display feature, e.g. hinge, fold, cutout.
  final DisplayFeatureType type;

  /// Posture of display feature, which is populated only for folds and hinges.
  ///
  /// For cutouts, this is [DisplayFeatureState.unknown]
  final DisplayFeatureState state;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is DisplayFeature &&
        bounds == other.bounds &&
        type == other.type &&
        state == other.state;
  }

  @override
  int get hashCode => Object.hash(bounds, type, state);

  @override
  String toString() {
    return 'DisplayFeature(rect: $bounds, type: $type, state: $state)';
  }
}

/// Type of [DisplayFeature], describing the [DisplayFeature] behaviour and if
/// it obstructs the display.
///
/// Some types of [DisplayFeature], like [DisplayFeatureType.fold], can be
/// reported without actually impeding drawing on the screen. They are useful
/// for knowing where the display is bent or has a crease. The
/// [DisplayFeature.bounds] can be 0-width in such cases.
///
/// The shape formed by the screens for types [DisplayFeatureType.fold] and
/// [DisplayFeatureType.hinge] is called the posture and is exposed in
/// [DisplayFeature.state]. For example, the [DisplayFeatureState.postureFlat] posture
/// means the screens form a flat surface.
///
/// ![Device with a hinge display feature](https://flutter.github.io/assets-for-api-docs/assets/hardware/display_feature_hinge.png)
///
/// ![Device with a fold display feature](https://flutter.github.io/assets-for-api-docs/assets/hardware/display_feature_fold.png)
///
/// ![Device with a cutout display feature](https://flutter.github.io/assets-for-api-docs/assets/hardware/display_feature_cutout.png)
enum DisplayFeatureType {
  /// [DisplayFeature] type is new and not yet known to Flutter.
  unknown,

  /// A fold in the flexible screen without a physical gap.
  ///
  /// The bounds for this display feature type indicate where the display makes a crease.
  fold,

  /// A physical separation with a hinge that allows two display panels to fold.
  hinge,

  /// A non-displaying area of the screen, usually housing cameras or sensors.
  cutout,
}

/// State of the display feature, which contains information about the posture
/// for foldable features.
///
/// The posture is the shape made by the parts of the flexible screen or
/// physical screen panels. They are inspired by and similar to
/// [Android Postures](https://developer.android.com/guide/topics/ui/foldables#postures).
///
/// * For [DisplayFeatureType.fold]s & [DisplayFeatureType.hinge]s, the state is
///   the posture.
/// * For [DisplayFeatureType.cutout]s, the state is not used and has the
/// [DisplayFeatureState.unknown] value.
enum DisplayFeatureState {
  /// The display feature is a [DisplayFeatureType.cutout] or this state is new
  /// and not yet known to Flutter.
  unknown,

  /// The foldable device is completely open.
  ///
  /// The screen space that is presented to the user is flat.
  postureFlat,

  /// Fold angle is in an intermediate position between opened and closed state.
  ///
  /// There is a non-flat angle between parts of the flexible screen or between
  /// physical screen panels such that the screens start to face each other.
  postureHalfOpened,
}

/// An identifier used to select a user's language and formatting preferences.
///
/// This represents a [Unicode Language
/// Identifier](https://www.unicode.org/reports/tr35/#Unicode_language_identifier)
/// (i.e. without Locale extensions), except variants are not supported.
///
/// Locales are canonicalized according to the "preferred value" entries in the
/// [IANA Language Subtag
/// Registry](https://www.iana.org/assignments/language-subtag-registry/language-subtag-registry).
/// For example, `const Locale('he')` and `const Locale('iw')` are equal and
/// both have the [languageCode] `he`, because `iw` is a deprecated language
/// subtag that was replaced by the subtag `he`.
///
/// See also:
///
///  * [PlatformDispatcher.locale], which specifies the system's currently selected
///    [Locale].
class Locale {
  /// Creates a new Locale object. The first argument is the
  /// primary language subtag, the second is the region (also
  /// referred to as 'country') subtag.
  ///
  /// For example:
  ///
  /// ```dart
  /// const Locale swissFrench = Locale('fr', 'CH');
  /// const Locale canadianFrench = Locale('fr', 'CA');
  /// ```
  ///
  /// The primary language subtag must not be null. The region subtag is
  /// optional. When there is no region/country subtag, the parameter should
  /// be omitted or passed `null` instead of an empty-string.
  ///
  /// The subtag values are _case sensitive_ and must be one of the valid
  /// subtags according to CLDR supplemental data:
  /// [language](https://github.com/unicode-org/cldr/blob/master/common/validity/language.xml),
  /// [region](https://github.com/unicode-org/cldr/blob/master/common/validity/region.xml). The
  /// primary language subtag must be at least two and at most eight lowercase
  /// letters, but not four letters. The region subtag must be two
  /// uppercase letters or three digits. See the [Unicode Language
  /// Identifier](https://www.unicode.org/reports/tr35/#Unicode_language_identifier)
  /// specification.
  ///
  /// Validity is not checked by default, but some methods may throw away
  /// invalid data.
  ///
  /// See also:
  ///
  ///  * [Locale.fromSubtags], which also allows a [scriptCode] to be
  ///    specified.
  const Locale(this._languageCode, [this._countryCode])
    : assert(_languageCode != ''),
      scriptCode = null;

  /// Creates a new Locale object.
  ///
  /// The keyword arguments specify the subtags of the Locale.
  ///
  /// The subtag values are _case sensitive_ and must be valid subtags according
  /// to CLDR supplemental data:
  /// [language](https://github.com/unicode-org/cldr/blob/master/common/validity/language.xml),
  /// [script](https://github.com/unicode-org/cldr/blob/master/common/validity/script.xml) and
  /// [region](https://github.com/unicode-org/cldr/blob/master/common/validity/region.xml) for
  /// each of languageCode, scriptCode and countryCode respectively.
  ///
  /// The [languageCode] subtag is optional. When there is no language subtag,
  /// the parameter should be omitted or set to "und". When not supplied, the
  /// [languageCode] defaults to "und", an undefined language code.
  ///
  /// The [countryCode] subtag is optional. When there is no country subtag,
  /// the parameter should be omitted or passed `null` instead of an empty-string.
  ///
  /// Validity is not checked by default, but some methods may throw away
  /// invalid data.
  const Locale.fromSubtags({String languageCode = 'und', this.scriptCode, String? countryCode})
    : assert(languageCode != ''),
      _languageCode = languageCode,
      assert(scriptCode != ''),
      assert(countryCode != ''),
      _countryCode = countryCode;

  /// The primary language subtag for the locale.
  ///
  /// This must not be null. It may be 'und', representing 'undefined'.
  ///
  /// This is expected to be string registered in the [IANA Language Subtag
  /// Registry](https://www.iana.org/assignments/language-subtag-registry/language-subtag-registry)
  /// with the type "language". The string specified must match the case of the
  /// string in the registry.
  ///
  /// Language subtags that are deprecated in the registry and have a preferred
  /// code are changed to their preferred code. For example, `const
  /// Locale('he')` and `const Locale('iw')` are equal, and both have the
  /// [languageCode] `he`, because `iw` is a deprecated language subtag that was
  /// replaced by the subtag `he`.
  ///
  /// This must be a valid Unicode Language subtag as listed in [Unicode CLDR
  /// supplemental
  /// data](https://github.com/unicode-org/cldr/blob/master/common/validity/language.xml).
  ///
  /// See also:
  ///
  ///  * [Locale.fromSubtags], which describes the conventions for creating
  ///    [Locale] objects.
  String get languageCode => _deprecatedLanguageSubtagMap[_languageCode] ?? _languageCode;
  final String _languageCode;

  // This map is generated by //flutter/tools/gen_locale.dart
  // Mappings generated for language subtag registry as of 2019-02-27.
  static const Map<String, String> _deprecatedLanguageSubtagMap = <String, String>{
    'in': 'id', // Indonesian; deprecated 1989-01-01
    'iw': 'he', // Hebrew; deprecated 1989-01-01
    'ji': 'yi', // Yiddish; deprecated 1989-01-01
    'jw': 'jv', // Javanese; deprecated 2001-08-13
    'mo': 'ro', // Moldavian, Moldovan; deprecated 2008-11-22
    'aam': 'aas', // Aramanik; deprecated 2015-02-12
    'adp': 'dz', // Adap; deprecated 2015-02-12
    'aue': 'ktz', // ǂKxʼauǁʼein; deprecated 2015-02-12
    'ayx': 'nun', // Ayi (China); deprecated 2011-08-16
    'bgm': 'bcg', // Baga Mboteni; deprecated 2016-05-30
    'bjd': 'drl', // Bandjigali; deprecated 2012-08-12
    'ccq': 'rki', // Chaungtha; deprecated 2012-08-12
    'cjr': 'mom', // Chorotega; deprecated 2010-03-11
    'cka': 'cmr', // Khumi Awa Chin; deprecated 2012-08-12
    'cmk': 'xch', // Chimakum; deprecated 2010-03-11
    'coy': 'pij', // Coyaima; deprecated 2016-05-30
    'cqu': 'quh', // Chilean Quechua; deprecated 2016-05-30
    'drh': 'khk', // Darkhat; deprecated 2010-03-11
    'drw': 'prs', // Darwazi; deprecated 2010-03-11
    'gav': 'dev', // Gabutamon; deprecated 2010-03-11
    'gfx': 'vaj', // Mangetti Dune ǃXung; deprecated 2015-02-12
    'ggn': 'gvr', // Eastern Gurung; deprecated 2016-05-30
    'gti': 'nyc', // Gbati-ri; deprecated 2015-02-12
    'guv': 'duz', // Gey; deprecated 2016-05-30
    'hrr': 'jal', // Horuru; deprecated 2012-08-12
    'ibi': 'opa', // Ibilo; deprecated 2012-08-12
    'ilw': 'gal', // Talur; deprecated 2013-09-10
    'jeg': 'oyb', // Jeng; deprecated 2017-02-23
    'kgc': 'tdf', // Kasseng; deprecated 2016-05-30
    'kgh': 'kml', // Upper Tanudan Kalinga; deprecated 2012-08-12
    'koj': 'kwv', // Sara Dunjo; deprecated 2015-02-12
    'krm': 'bmf', // Krim; deprecated 2017-02-23
    'ktr': 'dtp', // Kota Marudu Tinagas; deprecated 2016-05-30
    'kvs': 'gdj', // Kunggara; deprecated 2016-05-30
    'kwq': 'yam', // Kwak; deprecated 2015-02-12
    'kxe': 'tvd', // Kakihum; deprecated 2015-02-12
    'kzj': 'dtp', // Coastal Kadazan; deprecated 2016-05-30
    'kzt': 'dtp', // Tambunan Dusun; deprecated 2016-05-30
    'lii': 'raq', // Lingkhim; deprecated 2015-02-12
    'lmm': 'rmx', // Lamam; deprecated 2014-02-28
    'meg': 'cir', // Mea; deprecated 2013-09-10
    'mst': 'mry', // Cataelano Mandaya; deprecated 2010-03-11
    'mwj': 'vaj', // Maligo; deprecated 2015-02-12
    'myt': 'mry', // Sangab Mandaya; deprecated 2010-03-11
    'nad': 'xny', // Nijadali; deprecated 2016-05-30
    'ncp': 'kdz', // Ndaktup; deprecated 2018-03-08
    'nnx': 'ngv', // Ngong; deprecated 2015-02-12
    'nts': 'pij', // Natagaimas; deprecated 2016-05-30
    'oun': 'vaj', // ǃOǃung; deprecated 2015-02-12
    'pcr': 'adx', // Panang; deprecated 2013-09-10
    'pmc': 'huw', // Palumata; deprecated 2016-05-30
    'pmu': 'phr', // Mirpur Panjabi; deprecated 2015-02-12
    'ppa': 'bfy', // Pao; deprecated 2016-05-30
    'ppr': 'lcq', // Piru; deprecated 2013-09-10
    'pry': 'prt', // Pray 3; deprecated 2016-05-30
    'puz': 'pub', // Purum Naga; deprecated 2014-02-28
    'sca': 'hle', // Sansu; deprecated 2012-08-12
    'skk': 'oyb', // Sok; deprecated 2017-02-23
    'tdu': 'dtp', // Tempasuk Dusun; deprecated 2016-05-30
    'thc': 'tpo', // Tai Hang Tong; deprecated 2016-05-30
    'thx': 'oyb', // The; deprecated 2015-02-12
    'tie': 'ras', // Tingal; deprecated 2011-08-16
    'tkk': 'twm', // Takpa; deprecated 2011-08-16
    'tlw': 'weo', // South Wemale; deprecated 2012-08-12
    'tmp': 'tyj', // Tai Mène; deprecated 2016-05-30
    'tne': 'kak', // Tinoc Kallahan; deprecated 2016-05-30
    'tnf': 'prs', // Tangshewi; deprecated 2010-03-11
    'tsf': 'taj', // Southwestern Tamang; deprecated 2015-02-12
    'uok': 'ema', // Uokha; deprecated 2015-02-12
    'xba': 'cax', // Kamba (Brazil); deprecated 2016-05-30
    'xia': 'acn', // Xiandao; deprecated 2013-09-10
    'xkh': 'waw', // Karahawyana; deprecated 2016-05-30
    'xsj': 'suj', // Subi; deprecated 2015-02-12
    'ybd': 'rki', // Yangbye; deprecated 2012-08-12
    'yma': 'lrr', // Yamphe; deprecated 2012-08-12
    'ymt': 'mtm', // Mator-Taygi-Karagas; deprecated 2015-02-12
    'yos': 'zom', // Yos; deprecated 2013-09-10
    'yuu': 'yug', // Yugh; deprecated 2014-02-28
  };

  /// The script subtag for the locale.
  ///
  /// This may be null, indicating that there is no specified script subtag.
  ///
  /// This must be a valid Unicode Language Identifier script subtag as listed
  /// in [Unicode CLDR supplemental
  /// data](https://github.com/unicode-org/cldr/blob/master/common/validity/script.xml).
  ///
  /// See also:
  ///
  ///  * [Locale.fromSubtags], which describes the conventions for creating
  ///    [Locale] objects.
  final String? scriptCode;

  /// The region subtag for the locale.
  ///
  /// This may be null, indicating that there is no specified region subtag.
  ///
  /// This is expected to be string registered in the [IANA Language Subtag
  /// Registry](https://www.iana.org/assignments/language-subtag-registry/language-subtag-registry)
  /// with the type "region". The string specified must match the case of the
  /// string in the registry.
  ///
  /// Region subtags that are deprecated in the registry and have a preferred
  /// code are changed to their preferred code. For example, `const Locale('de',
  /// 'DE')` and `const Locale('de', 'DD')` are equal, and both have the
  /// [countryCode] `DE`, because `DD` is a deprecated language subtag that was
  /// replaced by the subtag `DE`.
  ///
  /// See also:
  ///
  ///  * [Locale.fromSubtags], which describes the conventions for creating
  ///    [Locale] objects.
  String? get countryCode => _deprecatedRegionSubtagMap[_countryCode] ?? _countryCode;
  final String? _countryCode;

  // This map is generated by //flutter/tools/gen_locale.dart
  // Mappings generated for language subtag registry as of 2019-02-27.
  static const Map<String, String> _deprecatedRegionSubtagMap = <String, String>{
    'BU': 'MM', // Burma; deprecated 1989-12-05
    'DD': 'DE', // German Democratic Republic; deprecated 1990-10-30
    'FX': 'FR', // Metropolitan France; deprecated 1997-07-14
    'TP': 'TL', // East Timor; deprecated 2002-05-20
    'YD': 'YE', // Democratic Yemen; deprecated 1990-08-14
    'ZR': 'CD', // Zaire; deprecated 1997-07-14
  };

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other is! Locale) {
      return false;
    }
    final String? thisCountryCode = countryCode;
    final String? otherCountryCode = other.countryCode;
    return other.languageCode == languageCode &&
        other.scriptCode ==
            scriptCode // scriptCode cannot be ''
            &&
        (other.countryCode ==
                thisCountryCode // Treat '' as equal to null.
                ||
            otherCountryCode != null && otherCountryCode.isEmpty && thisCountryCode == null ||
            thisCountryCode != null && thisCountryCode.isEmpty && other.countryCode == null);
  }

  @override
  int get hashCode => Object.hash(languageCode, scriptCode, countryCode == '' ? null : countryCode);

  static Locale? _cachedLocale;
  static String? _cachedLocaleString;

  /// Returns a string representing the locale.
  ///
  /// This identifier happens to be a valid Unicode Locale Identifier using
  /// underscores as separator, however it is intended to be used for debugging
  /// purposes only. For parsable results, use [toLanguageTag] instead.
  @keepToString
  @override
  String toString() {
    if (!identical(_cachedLocale, this)) {
      _cachedLocale = this;
      _cachedLocaleString = _rawToString('_');
    }
    return _cachedLocaleString!;
  }

  /// Returns a syntactically valid Unicode BCP47 Locale Identifier.
  ///
  /// Some examples of such identifiers: "en", "es-419", "hi-Deva-IN" and
  /// "zh-Hans-CN". See http://www.unicode.org/reports/tr35/ for technical
  /// details.
  String toLanguageTag() => _rawToString('-');

  String _rawToString(String separator) {
    final StringBuffer out = StringBuffer(languageCode);
    if (scriptCode != null && scriptCode!.isNotEmpty) {
      out.write('$separator$scriptCode');
    }
    final String? countryCode = _countryCode;
    if (countryCode != null && countryCode.isNotEmpty) {
      out.write('$separator${this.countryCode}');
    }
    return out.toString();
  }
}

/// Various performance modes for tuning the Dart VM's GC performance.
///
/// For the editor of this enum, please keep the order in sync with `Dart_PerformanceMode`
/// in [dart_api.h](https://github.com/dart-lang/sdk/blob/main/runtime/include/dart_api.h#L1302).
enum DartPerformanceMode {
  /// This is the default mode that the Dart VM is in.
  balanced,

  /// Optimize for low latency, at the expense of throughput and memory overhead
  /// by performing work in smaller batches (requiring more overhead) or by
  /// delaying work (requiring more memory). An embedder should not remain in
  /// this mode indefinitely.
  latency,

  /// Optimize for high throughput, at the expense of latency and memory overhead
  /// by performing work in larger batches with more intervening growth.
  throughput,

  /// Optimize for low memory, at the expensive of throughput and latency by more
  /// frequently performing work.
  memory,
}

/// An event to request a [SemanticsAction] of [type] to be performed on the
/// [SemanticsNode] identified by [nodeId] owned by the [FlutterView] identified
/// by [viewId].
///
/// Used by [SemanticsBinding.performSemanticsAction].
class SemanticsActionEvent {
  /// Creates a [SemanticsActionEvent].
  const SemanticsActionEvent({
    required this.type,
    required this.viewId,
    required this.nodeId,
    this.arguments,
  });

  /// The type of action to be performed.
  final SemanticsAction type;

  /// The id of the [FlutterView] the [SemanticsNode] identified by [nodeId] is
  /// associated with.
  final int viewId;

  /// The id of the [SemanticsNode] on which the action is to be performed.
  final int nodeId;

  /// Optional arguments for the action.
  final Object? arguments;

  static const Object _noArgumentPlaceholder = Object();

  /// Create a clone of the [SemanticsActionEvent] but with provided parameters
  /// replaced.
  SemanticsActionEvent copyWith({
    SemanticsAction? type,
    int? viewId,
    int? nodeId,
    Object? arguments = _noArgumentPlaceholder,
  }) {
    return SemanticsActionEvent(
      type: type ?? this.type,
      viewId: viewId ?? this.viewId,
      nodeId: nodeId ?? this.nodeId,
      arguments: arguments == _noArgumentPlaceholder ? this.arguments : arguments,
    );
  }
}

/// Signature for [PlatformDispatcher.onViewFocusChange].
typedef ViewFocusChangeCallback = void Function(ViewFocusEvent viewFocusEvent);

/// An event for the engine to communicate view focus changes to the app.
///
/// This value will be typically passed to the [PlatformDispatcher.onViewFocusChange]
/// callback.
final class ViewFocusEvent {
  /// Creates a [ViewFocusChange].
  const ViewFocusEvent({required this.viewId, required this.state, required this.direction});

  /// The ID of the [FlutterView] that experienced a focus change.
  final int viewId;

  /// The state focus changed to.
  final ViewFocusState state;

  /// The direction focus changed to.
  final ViewFocusDirection direction;

  @override
  String toString() {
    return 'ViewFocusEvent(viewId: $viewId, state: $state, direction: $direction)';
  }
}

/// Represents the focus state of a given [FlutterView].
///
/// When focus is lost, the view's focus state changes to [ViewFocusState.unfocused].
///
/// When focus is gained, the view's focus state changes to [ViewFocusState.focused].
///
/// Valid transitions within a view are:
///
/// - [ViewFocusState.focused] to [ViewFocusState.unfocused].
/// - [ViewFocusState.unfocused] to [ViewFocusState.focused].
///
/// See also:
///
///   * [ViewFocusDirection], that specifies the focus direction.
///   * [ViewFocusEvent], that conveys information about a [FlutterView] focus change.
enum ViewFocusState {
  /// Specifies that a view does not have platform focus.
  unfocused,

  /// Specifies that a view has platform focus.
  focused,
}

/// Represents the direction in which the focus transitioned across [FlutterView]s.
///
/// See also:
///
///   * [ViewFocusState], that specifies the current focus state of a [FlutterView].
///   * [ViewFocusEvent], that conveys information about a [FlutterView] focus change.
enum ViewFocusDirection {
  /// Indicates the focus transition did not have a direction.
  ///
  /// This is typically associated with focus being programmatically requested or
  /// when focus is lost.
  undefined,

  /// Indicates the focus transition was performed in a forward direction.
  ///
  /// This is typically result of the user pressing tab.
  forward,

  /// Indicates the focus transition was performed in a backward direction.
  ///
  /// This is typically result of the user pressing shift + tab.
  backward,
}
