// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

/// Used in internal testing.
class FakePlatformViewController extends PlatformViewController {
  FakePlatformViewController(this.viewId);

  bool disposed = false;
  bool focusCleared = false;

  /// Events that are dispatched.
  List<PointerEvent> dispatchedPointerEvents = <PointerEvent>[];

  @override
  final int viewId;

  @override
  Future<void> dispatchPointerEvent(PointerEvent event) async {
    dispatchedPointerEvents.add(event);
  }

  void clearTestingVariables() {
    dispatchedPointerEvents.clear();
    disposed = false;
    focusCleared = false;
  }

  @override
  Future<void> dispose() async {
    disposed = true;
  }

  @override
  Future<void> clearFocus() async {
    focusCleared = true;
  }
}

class FakeAndroidViewController implements AndroidViewController {
  FakeAndroidViewController(
    this.viewId, {
    this.requiresSize = false,
    this.requiresViewComposition = false,
  });

  bool disposed = false;
  bool focusCleared = false;
  bool created = false;
  // If true, [create] won't be considered to have been called successfully
  // unless it includes a size.
  bool requiresSize;

  bool _createCalledSuccessfully = false;

  Offset? createPosition;

  final List<PlatformViewCreatedCallback> _createdCallbacks = <PlatformViewCreatedCallback>[];

  /// Events that are dispatched.
  List<PointerEvent> dispatchedPointerEvents = <PointerEvent>[];

  @override
  final int viewId;

  @override
  late PointTransformer pointTransformer;

  @override
  Future<void> dispatchPointerEvent(PointerEvent event) async {
    dispatchedPointerEvents.add(event);
  }

  void clearTestingVariables() {
    dispatchedPointerEvents.clear();
    disposed = false;
    focusCleared = false;
  }

  @override
  Future<void> dispose() async {
    disposed = true;
  }

  @override
  Future<void> clearFocus() async {
    focusCleared = true;
  }

  @override
  Future<Size> setSize(Size size) {
    return Future<Size>.value(size);
  }

  @override
  Future<void> setOffset(Offset off) async {}

  @override
  int get textureId => 0;

  @override
  bool get awaitingCreation => !_createCalledSuccessfully;

  @override
  bool get isCreated => created;

  @override
  void addOnPlatformViewCreatedListener(PlatformViewCreatedCallback listener) {
    created = true;
    createdCallbacks.add(listener);
  }

  @override
  void removeOnPlatformViewCreatedListener(PlatformViewCreatedCallback listener) {
    createdCallbacks.remove(listener);
  }

  @override
  Future<void> sendMotionEvent(AndroidMotionEvent event) {
    throw UnimplementedError();
  }

  @override
  Future<void> setLayoutDirection(TextDirection layoutDirection) {
    throw UnimplementedError();
  }

  @override
  Future<void> create({Size? size, Offset? position}) async {
    assert(!_createCalledSuccessfully);
    if (requiresSize && size != null) {
      assert(!size.isEmpty);
    }
    _createCalledSuccessfully = size != null && position != null || !requiresSize;
    createPosition = position;
  }

  @override
  List<PlatformViewCreatedCallback> get createdCallbacks => _createdCallbacks;

  @override
  bool requiresViewComposition;
}

class FakeAndroidPlatformViewsController {
  FakeAndroidPlatformViewsController() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform_views,
      _onMethodCall,
    );
  }

  Iterable<FakeAndroidPlatformView> get views => _views.values;
  final Map<int, FakeAndroidPlatformView> _views = <int, FakeAndroidPlatformView>{};

  final Map<int, List<FakeAndroidMotionEvent>> motionEvents = <int, List<FakeAndroidMotionEvent>>{};

  final Set<String> _registeredViewTypes = <String>{};

  int _textureCounter = 0;

  Completer<void>? resizeCompleter;

  Completer<void>? createCompleter;

  int? lastClearedFocusViewId;

  Map<int, Offset> offsets = <int, Offset>{};

  /// True if Texture Layer Hybrid Composition mode should be enabled.
  ///
  /// When false, `create` will simulate the engine's fallback mode.
  bool allowTextureLayerMode = true;

  void registerViewType(String viewType) {
    _registeredViewTypes.add(viewType);
  }

  void invokeViewFocused(int viewId) {
    final MethodCodec codec = SystemChannels.platform_views.codec;
    final ByteData data = codec.encodeMethodCall(MethodCall('viewFocused', viewId));
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      SystemChannels.platform_views.name,
      data,
      (ByteData? data) {},
    );
  }

  Future<dynamic> _onMethodCall(MethodCall call) {
    return switch (call.method) {
      'create' => _create(call),
      'dispose' => _dispose(call),
      'resize' => _resize(call),
      'touch' => _touch(call),
      'setDirection' => _setDirection(call),
      'clearFocus' => _clearFocus(call),
      'offset' => _offset(call),
      _ => Future<dynamic>.sync(() => null),
    };
  }

  Future<dynamic> _create(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    final String viewType = args['viewType'] as String;
    final double? width = args['width'] as double?;
    final double? height = args['height'] as double?;
    final int layoutDirection = args['direction'] as int;
    final bool? hybrid = args['hybrid'] as bool?;
    final bool? hybridFallback = args['hybridFallback'] as bool?;
    final Uint8List? creationParams = args['params'] as Uint8List?;
    final double? top = args['top'] as double?;
    final double? left = args['left'] as double?;

    if (_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create an already created platform view, view id: $id',
      );
    }

    if (!_registeredViewTypes.contains(viewType)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create a platform view of unregistered type: $viewType',
      );
    }

    if (createCompleter != null) {
      await createCompleter!.future;
    }

    _views[id] = FakeAndroidPlatformView(
      id,
      viewType,
      width != null && height != null ? Size(width, height) : null,
      layoutDirection,
      hybrid: hybrid,
      hybridFallback: hybridFallback,
      creationParams: creationParams,
      position: left != null && top != null ? Offset(left, top) : null,
    );
    // Return a hybrid result (null rather than a texture ID) if:
    final bool hybridResult =
        // hybrid was explicitly requested, or
        (hybrid ?? false) ||
        // hybrid fallback was requested and simulated.
        (!allowTextureLayerMode && (hybridFallback ?? false));
    if (hybridResult) {
      return Future<void>.value();
    }
    final int textureId = _textureCounter++;
    return Future<int>.value(textureId);
  }

  Future<dynamic> _dispose(MethodCall call) {
    assert(call.arguments is Map);
    final Map<Object?, Object?> arguments = call.arguments as Map<Object?, Object?>;

    final int id = arguments['id']! as int;
    final bool hybrid = arguments['hybrid']! as bool;

    if (hybrid && !_views[id]!.hybrid!) {
      throw ArgumentError(
        'An $AndroidViewController using hybrid composition must pass `hybrid: true`',
      );
    } else if (!hybrid && (_views[id]!.hybrid ?? false)) {
      throw ArgumentError(
        'An $AndroidViewController not using hybrid composition must pass `hybrid: false`',
      );
    }

    if (!_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to dispose a platform view with unknown id: $id',
      );
    }

    _views.remove(id);
    return Future<dynamic>.sync(() => null);
  }

  Future<dynamic> _resize(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    final double width = args['width'] as double;
    final double height = args['height'] as double;

    if (!_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to resize a platform view with unknown id: $id',
      );
    }

    if (resizeCompleter != null) {
      await resizeCompleter!.future;
    }
    _views[id] = _views[id]!.copyWith(size: Size(width, height));

    return Future<Map<dynamic, dynamic>>.sync(
      () => <dynamic, dynamic>{'width': width, 'height': height},
    );
  }

  Future<dynamic> _offset(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    final double top = args['top'] as double;
    final double left = args['left'] as double;
    offsets[id] = Offset(left, top);
    return Future<dynamic>.sync(() => null);
  }

  Future<dynamic> _touch(MethodCall call) {
    final List<dynamic> args = call.arguments as List<dynamic>;
    final int id = args[0] as int;
    final int action = args[3] as int;
    final List<List<dynamic>> pointerProperties = (args[5] as List<dynamic>).cast<List<dynamic>>();
    final List<List<dynamic>> pointerCoords = (args[6] as List<dynamic>).cast<List<dynamic>>();
    final List<Offset> pointerOffsets = <Offset>[];
    final List<int> pointerIds = <int>[];
    for (int i = 0; i < pointerCoords.length; i++) {
      pointerIds.add(pointerProperties[i][0] as int);
      final double x = pointerCoords[i][7] as double;
      final double y = pointerCoords[i][8] as double;
      pointerOffsets.add(Offset(x, y));
    }

    if (!motionEvents.containsKey(id)) {
      motionEvents[id] = <FakeAndroidMotionEvent>[];
    }

    motionEvents[id]!.add(FakeAndroidMotionEvent(action, pointerIds, pointerOffsets));
    return Future<dynamic>.sync(() => null);
  }

  Future<dynamic> _setDirection(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    final int layoutDirection = args['direction'] as int;

    if (!_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to resize a platform view with unknown id: $id',
      );
    }

    _views[id] = _views[id]!.copyWith(layoutDirection: layoutDirection);

    return Future<dynamic>.sync(() => null);
  }

  Future<dynamic> _clearFocus(MethodCall call) {
    final int id = call.arguments as int;

    if (!_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to clear the focus on a platform view with unknown id: $id',
      );
    }

    lastClearedFocusViewId = id;
    return Future<dynamic>.sync(() => null);
  }
}

class FakeIosPlatformViewsController {
  FakeIosPlatformViewsController() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform_views,
      _onMethodCall,
    );
  }

  Iterable<FakeUiKitView> get views => _views.values;
  final Map<int, FakeUiKitView> _views = <int, FakeUiKitView>{};

  final Set<String> _registeredViewTypes = <String>{};

  // When this completer is non null, the 'create' method channel call will be
  // delayed until it completes.
  Completer<void>? creationDelay;

  // Maps a view id to the number of gestures it accepted so far.
  final Map<int, int> gesturesAccepted = <int, int>{};

  // Maps a view id to the number of gestures it rejected so far.
  final Map<int, int> gesturesRejected = <int, int>{};

  void registerViewType(String viewType) {
    _registeredViewTypes.add(viewType);
  }

  void invokeViewFocused(int viewId) {
    final MethodCodec codec = SystemChannels.platform_views.codec;
    final ByteData data = codec.encodeMethodCall(MethodCall('viewFocused', viewId));
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      SystemChannels.platform_views.name,
      data,
      (ByteData? data) {},
    );
  }

  Future<dynamic> _onMethodCall(MethodCall call) {
    return switch (call.method) {
      'create' => _create(call),
      'dispose' => _dispose(call),
      'acceptGesture' => _acceptGesture(call),
      'rejectGesture' => _rejectGesture(call),
      _ => Future<dynamic>.sync(() => null),
    };
  }

  Future<dynamic> _create(MethodCall call) async {
    if (creationDelay != null) {
      await creationDelay!.future;
    }
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    final String viewType = args['viewType'] as String;
    final Uint8List? creationParams = args['params'] as Uint8List?;

    if (_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create an already created platform view, view id: $id',
      );
    }

    if (!_registeredViewTypes.contains(viewType)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create a platform view of unregistered type: $viewType',
      );
    }

    _views[id] = FakeUiKitView(id, viewType, creationParams);
    gesturesAccepted[id] = 0;
    gesturesRejected[id] = 0;
    return Future<int?>.sync(() => null);
  }

  Future<dynamic> _acceptGesture(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    gesturesAccepted[id] = gesturesAccepted[id]! + 1;
    return Future<int?>.sync(() => null);
  }

  Future<dynamic> _rejectGesture(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    gesturesRejected[id] = gesturesRejected[id]! + 1;
    return Future<int?>.sync(() => null);
  }

  Future<dynamic> _dispose(MethodCall call) {
    final int id = call.arguments as int;

    if (!_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to dispose a platform view with unknown id: $id',
      );
    }

    _views.remove(id);
    return Future<dynamic>.sync(() => null);
  }
}

class FakeMacosPlatformViewsController {
  FakeMacosPlatformViewsController() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform_views,
      _onMethodCall,
    );
  }

  Iterable<FakeAppKitView> get views => _views.values;
  final Map<int, FakeAppKitView> _views = <int, FakeAppKitView>{};

  final Set<String> _registeredViewTypes = <String>{};

  // When this completer is non null, the 'create' method channel call will be
  // delayed until it completes.
  Completer<void>? creationDelay;

  // Maps a view id to the number of gestures it accepted so far.
  final Map<int, int> gesturesAccepted = <int, int>{};

  // Maps a view id to the number of gestures it rejected so far.
  final Map<int, int> gesturesRejected = <int, int>{};

  void registerViewType(String viewType) {
    _registeredViewTypes.add(viewType);
  }

  void invokeViewFocused(int viewId) {
    final MethodCodec codec = SystemChannels.platform_views.codec;
    final ByteData data = codec.encodeMethodCall(MethodCall('viewFocused', viewId));
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
      SystemChannels.platform_views.name,
      data,
      (ByteData? data) {},
    );
  }

  Future<dynamic> _onMethodCall(MethodCall call) {
    return switch (call.method) {
      'create' => _create(call),
      'dispose' => _dispose(call),
      'acceptGesture' => _acceptGesture(call),
      'rejectGesture' => _rejectGesture(call),
      _ => Future<dynamic>.sync(() => null),
    };
  }

  Future<dynamic> _create(MethodCall call) async {
    if (creationDelay != null) {
      await creationDelay!.future;
    }
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    final String viewType = args['viewType'] as String;
    final Uint8List? creationParams = args['params'] as Uint8List?;

    if (_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create an already created platform view, view id: $id',
      );
    }

    if (!_registeredViewTypes.contains(viewType)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to create a platform view of unregistered type: $viewType',
      );
    }

    _views[id] = FakeAppKitView(id, viewType, creationParams);
    gesturesAccepted[id] = 0;
    gesturesRejected[id] = 0;
    return Future<int?>.sync(() => null);
  }

  Future<dynamic> _acceptGesture(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    gesturesAccepted[id] = gesturesAccepted[id]! + 1;
    return Future<int?>.sync(() => null);
  }

  Future<dynamic> _rejectGesture(MethodCall call) async {
    final Map<dynamic, dynamic> args = call.arguments as Map<dynamic, dynamic>;
    final int id = args['id'] as int;
    gesturesRejected[id] = gesturesRejected[id]! + 1;
    return Future<int?>.sync(() => null);
  }

  Future<dynamic> _dispose(MethodCall call) {
    final int id = call.arguments as int;

    if (!_views.containsKey(id)) {
      throw PlatformException(
        code: 'error',
        message: 'Trying to dispose a platform view with unknown id: $id',
      );
    }

    _views.remove(id);
    return Future<dynamic>.sync(() => null);
  }
}

@immutable
class FakeAndroidPlatformView {
  const FakeAndroidPlatformView(
    this.id,
    this.type,
    this.size,
    this.layoutDirection, {
    this.hybrid,
    this.hybridFallback,
    this.creationParams,
    this.position,
  });

  final int id;
  final String type;
  final Uint8List? creationParams;
  final Size? size;
  final int layoutDirection;
  final bool? hybrid;
  final bool? hybridFallback;
  final Offset? position;

  FakeAndroidPlatformView copyWith({Size? size, int? layoutDirection}) => FakeAndroidPlatformView(
    id,
    type,
    size ?? this.size,
    layoutDirection ?? this.layoutDirection,
    hybrid: hybrid,
    hybridFallback: hybridFallback,
    creationParams: creationParams,
    position: position,
  );

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is FakeAndroidPlatformView &&
        other.id == id &&
        other.type == type &&
        listEquals<int>(other.creationParams, creationParams) &&
        other.size == size &&
        other.hybrid == hybrid &&
        other.hybridFallback == hybridFallback &&
        other.layoutDirection == layoutDirection &&
        other.position == position;
  }

  @override
  int get hashCode => Object.hash(
    id,
    type,
    creationParams == null ? null : Object.hashAll(creationParams!),
    size,
    layoutDirection,
    hybrid,
    hybridFallback,
    position,
  );

  @override
  String toString() {
    return 'FakeAndroidPlatformView(id: $id, type: $type, size: $size, '
        'layoutDirection: $layoutDirection, hybrid: $hybrid, '
        'hybridFallback: $hybridFallback, creationParams: $creationParams, position: $position)';
  }
}

@immutable
class FakeAndroidMotionEvent {
  const FakeAndroidMotionEvent(this.action, this.pointerIds, this.pointers);

  final int action;
  final List<Offset> pointers;
  final List<int> pointerIds;

  @override
  bool operator ==(Object other) {
    return other is FakeAndroidMotionEvent &&
        listEquals<int>(other.pointerIds, pointerIds) &&
        other.action == action &&
        listEquals<Offset>(other.pointers, pointers);
  }

  @override
  int get hashCode => Object.hash(action, Object.hashAll(pointers), Object.hashAll(pointerIds));

  @override
  String toString() {
    return 'FakeAndroidMotionEvent(action: $action, pointerIds: $pointerIds, pointers: $pointers)';
  }
}

@immutable
class FakeUiKitView {
  const FakeUiKitView(this.id, this.type, [this.creationParams]);

  final int id;
  final String type;
  final Uint8List? creationParams;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is FakeUiKitView &&
        other.id == id &&
        other.type == type &&
        other.creationParams == creationParams;
  }

  @override
  int get hashCode => Object.hash(id, type);

  @override
  String toString() {
    return 'FakeUiKitView(id: $id, type: $type, creationParams: $creationParams)';
  }
}

@immutable
class FakeAppKitView {
  const FakeAppKitView(this.id, this.type, [this.creationParams]);

  final int id;
  final String type;
  final Uint8List? creationParams;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is FakeAppKitView &&
        other.id == id &&
        other.type == type &&
        other.creationParams == creationParams;
  }

  @override
  int get hashCode => Object.hash(id, type);

  @override
  String toString() {
    return 'FakeAppKitView(id: $id, type: $type, creationParams: $creationParams)';
  }
}
